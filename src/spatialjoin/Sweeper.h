// Copyright 2023, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_SWEEPER_H_
#define SPATIALJOINS_SWEEPER_H_

#include <bzlib.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>
#include <zlib.h>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include "GeometryCache.h"
#include "Stats.h"
#include "util/JobQueue.h"
#include "util/geo/Geo.h"
#include "util/geo/IntervalIdx.h"

#ifndef POSIX_FADV_SEQUENTIAL
#define POSIX_FADV_SEQUENTIAL 2
#endif

namespace sj {

enum GeomType : uint8_t {
  POLYGON = 0,
  LINE = 1,
  POINT = 2,
  SIMPLE_LINE = 3,
  SIMPLE_POLYGON = 4
};
enum OutMode : uint8_t { PLAIN = 0, BZ2 = 1, GZ = 2, COUT = 3, NONE = 4 };

struct BoxVal {
  size_t id : 60;
  int32_t loY;
  int32_t upY;
  int32_t val;
  bool out : 1;
  GeomType type : 3;
  double areaOrLen;
  util::geo::I32Box b45;
  bool side;
  bool large;
};

struct WriteCand {
  std::string raw;
  std::string gid;
  BoxVal boxvalIn;
  BoxVal boxvalOut;
  size_t subid;
};

struct WriteBatch {
  std::vector<WriteCand> points;
  std::vector<WriteCand> simpleLines;
  std::vector<WriteCand> lines;
  std::vector<WriteCand> simpleAreas;
  std::vector<WriteCand> areas;
  std::vector<WriteCand> refs;

  size_t size() const {
    return points.size() + simpleLines.size() + lines.size() +
           simpleAreas.size() + areas.size() + refs.size();
  }
};

inline bool operator==(const BoxVal& a, const BoxVal& b) {
  return a.id == b.id && a.loY == b.loY && a.upY == b.upY && a.type == b.type;
}

struct SweepVal {
  SweepVal(size_t id, GeomType type)
      : id(id), type(type), side(false), large(false) {}
  SweepVal(size_t id, GeomType type, util::geo::I32Box b45, bool side,
           bool large)
      : id(id), type(type), b45(b45), side(side), large(large) {}
  SweepVal() : id(0), type(POLYGON) {}
  size_t id : 60;
  GeomType type : 3;
  util::geo::I32Box b45;
  bool side;
  bool large;
};

inline bool operator==(const SweepVal& a, const SweepVal& b) {
  return a.id == b.id && a.type == b.type;
}

inline bool operator<(const SweepVal& a, const SweepVal& b) {
  return a.id < b.id || (a.id == b.id && a.type < b.type);
}

struct Job {
  BoxVal boxVal;
  SweepVal sweepVal;
  std::string multiOut;
};

inline bool operator==(const Job& a, const Job& b) {
  return a.boxVal == b.boxVal && a.sweepVal == b.sweepVal &&
         a.multiOut == b.multiOut;
}

typedef std::vector<Job> JobBatch;

struct SweeperCfg {
  size_t numThreads;
  size_t numCacheThreads;
  size_t geomCacheMaxSize;
  std::string pairStart;
  std::string sepIsect;
  std::string sepContains;
  std::string sepCovers;
  std::string sepTouches;
  std::string sepEquals;
  std::string sepOverlaps;
  std::string sepCrosses;
  std::string pairEnd;
  bool useBoxIds;
  bool useArea;
  bool useOBB;
  bool useCutouts;
  bool useDiagBox;
  bool useFastSweepSkip;
  bool useInnerOuter;
  bool noGeometryChecks;
  double withinDist;
  std::function<void(size_t t, const char* a, const char* b, const char* pred)>
      writeRelCb;
  std::function<void(const std::string&)> logCb;
  std::function<void(const std::string&)> statsCb;
  std::function<void(size_t)> sweepProgressCb;
};

// buffer size _must_ be multiples of sizeof(BoxVal)
static const ssize_t BUFFER_S = sizeof(BoxVal) * 64 * 1024 * 512;

static const size_t BUFFER_S_PAIRS = 1024 * 1024 * 10;

static const size_t MAX_OUT_LINE_LENGTH = 1000;

static const size_t POINT_CACHE_SIZE = 1000;

// only use large geom cache for extreme geometries
static const size_t GEOM_LARGENESS_THRESHOLD = 1024 * 1024 * 1024;
;

// static const size_t AREA_CACHE_SIZE = 10000;
// static const size_t SIMPLE_AREA_CACHE_SIZE = 10000;
// static const size_t LINE_CACHE_SIZE = 10000;
// static const size_t SIMPLE_LINE_CACHE_SIZE = 10000;

class Sweeper {
 public:
  explicit Sweeper(SweeperCfg cfg, const std::string cache,
                   const std::string out)
      : _cfg(cfg),
        _obufpos(0),
        _pointCache(POINT_CACHE_SIZE, cfg.numCacheThreads, cache),
        _areaCache(cfg.geomCacheMaxSize, cfg.numCacheThreads, cache),
        _simpleAreaCache(cfg.geomCacheMaxSize, cfg.numCacheThreads, cache),
        _lineCache(cfg.geomCacheMaxSize, cfg.numCacheThreads, cache),
        _simpleLineCache(cfg.geomCacheMaxSize, cfg.numCacheThreads, cache),
        _cache(cache),
        _out(out),
        _jobs(100) {
    if (!_cfg.writeRelCb) {
      if (util::endsWith(_out, ".bz2"))
        _outMode = BZ2;
      else if (util::endsWith(_out, ".gz"))
        _outMode = GZ;
      else if (out.size())
        _outMode = PLAIN;
      else {
        struct stat std_out;
        struct stat dev_null;
        if (fstat(STDOUT_FILENO, &std_out) == 0 && S_ISCHR(std_out.st_mode) &&
            stat("/dev/null", &dev_null) == 0 &&
            std_out.st_dev == dev_null.st_dev &&
            std_out.st_ino == dev_null.st_ino) {
          // output to /dev/null, print nothing
          _outMode = NONE;
        } else {
          _outMode = COUT;
        }
      }
    }

    // OUTFACTOR 1
    _fname = util::getTmpFName(_cache, ".spatialjoin", "events");
    _file = open(_fname.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0666);

    if (_file < 0) {
      throw std::runtime_error("Could not open temporary file " + _fname);
    }

    // immediately unlink
    unlink(_fname.c_str());

#ifdef __unix__
    posix_fadvise(_file, 0, 0, POSIX_FADV_SEQUENTIAL);
#endif

    // OUTFACTOR 1

    _outBuffer = new unsigned char[BUFFER_S];
  };

  ~Sweeper() {
    close(_file);

    for (size_t i = 0; i < _outBuffers.size(); i++) {
      if (_outBuffers[i]) delete[] _outBuffers[i];
    }
  }

  void add(const util::geo::I32MultiPolygon& a, const std::string& gid,
           bool side, WriteBatch& batch) const;
  void add(const util::geo::I32MultiPolygon& a, const std::string& gid, size_t,
           bool side, WriteBatch& batch) const;
  void add(const util::geo::I32Polygon& a, const std::string& gid, bool side,
           WriteBatch& batch) const;
  void add(const util::geo::I32Polygon& a, const std::string& gid, size_t subId,
           bool side, WriteBatch& batch) const;

  void add(const util::geo::I32MultiLine& a, const std::string& gid, size_t,
           bool side, WriteBatch& batch) const;
  void add(const util::geo::I32MultiLine& a, const std::string& gid, bool side,
           WriteBatch& batch) const;
  void add(const util::geo::I32Line& a, const std::string& gid, bool side,
           WriteBatch& batch) const;
  void add(const util::geo::I32Line& a, const std::string& gid, size_t subid,
           bool side, WriteBatch& batch) const;

  void add(const util::geo::I32Point& a, const std::string& gid, bool side,
           WriteBatch& batch) const;
  void add(const util::geo::I32Point& a, const std::string& gid, size_t subid,
           bool side, WriteBatch& batch) const;
  void add(const util::geo::I32MultiPoint& a, const std::string& gid, size_t,
           bool side, WriteBatch& batch) const;
  void add(const util::geo::I32MultiPoint& a, const std::string& gid, bool side,
           WriteBatch& batch) const;

  void add(const std::string& a, const util::geo::I32Box& box,
           const std::string& gid, bool side, WriteBatch& batch) const;
  void add(const std::string& a, const util::geo::I32Box& box,
           const std::string& gid, size_t subid, bool side,
           WriteBatch& batch) const;

  void addBatch(WriteBatch& cands);

  void flush();

  RelStats sweep();
  void refDuplicates();

  size_t numElements() const { return _curSweepId / 2; }

  static std::string intToBase126(uint64_t id);
  static uint64_t base126ToInt(const std::string& id);

// _____________________________________________________________________________
  template <typename G>
  util::geo::I32Box getPaddedBoundingBox(const G& geom) const {
    auto bbox = util::geo::getBoundingBox(geom);

    if (_cfg.withinDist >= 0) {
      auto bbox = util::geo::getBoundingBox(geom);
      double xScaleFactor = getMaxScaleFactor(bbox);

      uint32_t padX = ((_cfg.withinDist / 2) * xScaleFactor) * PREC;
      uint32_t padY = (_cfg.withinDist / 2) * PREC;

      return {{bbox.getLowerLeft().getX() - padX,
               bbox.getLowerLeft().getY() - padY},
              {bbox.getUpperRight().getX() + padX,
               bbox.getUpperRight().getY() + padY}};
    }

    return bbox;
  }

 private:
  const SweeperCfg _cfg;
  size_t _curSweepId = 0;
  std::string _fname;
  int _file;
  unsigned char* _outBuffer;
  ssize_t _obufpos;

  OutMode _outMode;

  std::vector<FILE*> _rawFiles;
  std::vector<BZFILE*> _files;
  std::vector<gzFile> _gzFiles;
  std::vector<size_t> _outBufPos;
  std::vector<unsigned char*> _outBuffers;

  std::vector<size_t> _checks;
  std::vector<int32_t> _curX;
  std::vector<std::atomic<int32_t>> _atomicCurX;

  std::vector<RelStats> _relStats;

  mutable std::vector<Stats> _stats;

  GeometryCache<Point> _pointCache;
  GeometryCache<Area> _areaCache;
  GeometryCache<SimpleArea> _simpleAreaCache;
  GeometryCache<Line> _lineCache;
  GeometryCache<SimpleLine> _simpleLineCache;

  std::vector<std::map<std::string, std::map<std::string, std::set<size_t>>>>
      _subContains;
  std::vector<std::map<std::string, std::map<std::string, std::set<size_t>>>>
      _subCovered;
  std::vector<std::map<std::string, std::map<std::string, std::set<size_t>>>>
      _subEquals;
  std::vector<std::map<std::string, std::set<std::string>>> _subTouches;
  std::vector<std::map<std::string, std::set<std::string>>> _subNotTouches;
  std::vector<std::map<std::string, std::set<std::string>>> _subCrosses;
  std::vector<std::map<std::string, std::set<std::string>>> _subNotCrosses;
  std::vector<std::map<std::string, std::set<std::string>>> _subOverlaps;
  std::vector<std::map<std::string, std::set<std::string>>> _subNotOverlaps;
  std::map<std::string, size_t> _subSizes;

  std::set<size_t> _activeMultis[2];
  std::vector<std::string> _multiIds[2];
  std::vector<int32_t> _multiRightX[2];
  std::vector<int32_t> _multiLeftX[2];
  std::map<std::string, size_t> _multiGidToId[2];

  std::string _cache;
  std::string _out;

  util::JobQueue<JobBatch> _jobs;

  uint8_t _numSides = 1;

  std::vector<std::mutex> _mutsEquals;
  std::vector<std::mutex> _mutsCovers;
  std::vector<std::mutex> _mutsContains;
  std::vector<std::mutex> _mutsTouches;
  std::vector<std::mutex> _mutsNotTouches;
  std::vector<std::mutex> _mutsCrosses;
  std::vector<std::mutex> _mutsNotCrosses;
  std::vector<std::mutex> _mutsOverlaps;
  std::vector<std::mutex> _mutsNotOverlaps;

  Area areaFromSimpleArea(const SimpleArea* sa) const;

  std::tuple<bool, bool, bool, bool, bool> check(const Area* a, const Area* b,
                                                 size_t t) const;
  std::tuple<bool, bool, bool, bool, bool> check(const Line* a, const Area* b,
                                                 size_t t) const;
  std::tuple<bool, bool, bool, bool, bool> check(const SimpleLine* a,
                                                 const Area* b, size_t t) const;
  std::tuple<bool, bool, bool, bool, bool> check(const Line* a, const Line* b,
                                                 size_t t) const;
  std::tuple<bool, bool, bool, bool, bool> check(const Line* a,
                                                 const SimpleLine* b,
                                                 size_t t) const;
  std::tuple<bool, bool, bool, bool, bool> check(const SimpleLine* a,
                                                 const SimpleLine* b,
                                                 size_t t) const;
  std::tuple<bool, bool, bool, bool, bool> check(const SimpleLine* a,
                                                 const Line* b, size_t t) const;
  std::pair<bool, bool> check(const util::geo::I32Point& a, const Area* b,
                              size_t t) const;
  std::tuple<bool, bool> check(const util::geo::I32Point& a, const Line* b,
                               size_t t) const;

  double distCheck(const util::geo::I32Point& a, const Area* b, size_t t) const;
  double distCheck(const util::geo::I32Point& a, const Line* b, size_t t) const;
  double distCheck(const util::geo::I32Point& a, const SimpleLine* b,
                   size_t t) const;
  double distCheck(const SimpleLine* a, const SimpleLine* b, size_t t) const;
  double distCheck(const SimpleLine* a, const Line* b, size_t t) const;
  double distCheck(const SimpleLine* a, const Area* b, size_t t) const;
  double distCheck(const Line* a, const Line* b, size_t t) const;
  double distCheck(const Area* a, const Area* b, size_t t) const;
  double distCheck(const Line* a, const Area* b, size_t t) const;

  bool refRelated(const std::string& a, const std::string& b) const;

  double getMaxScaleFactor(const util::geo::I32Box& geom) const;
  double getMaxScaleFactor(const util::geo::I32Point& geom) const;

  void diskAdd(const BoxVal& bv);

  void multiOut(size_t t, const std::string& gid);
  void multiAdd(const std::string& gid, bool side, int32_t xLeft,
                int32_t xRight);
  void clearMultis(bool force);

  void writeIntersect(size_t t, const std::string& a, const std::string& b);
  void writeRel(size_t t, const std::string& a, const std::string& b,
                const std::string& pred);
  void writeRelToBuf(size_t t, const std::string& a, const std::string& b,
                     const std::string& pred);
  void writeContains(size_t t, const std::string& a, const std::string& b,
                     size_t bSub);
  void writeCovers(size_t t, const std::string& a, const std::string& b,
                   size_t bSub);
  void writeEquals(size_t t, const std::string& a, size_t aSub,
                   const std::string& b, size_t bSub);
  void writeDist(size_t t, const std::string& a, size_t aSub,
                 const std::string& b, size_t bSub, double dist);
  void writeTouches(size_t t, const std::string& a, size_t aSub,
                    const std::string& b, size_t bSub);
  void writeNotTouches(size_t t, const std::string& a, size_t aSub,
                       const std::string& b, size_t bSub);

  void writeOverlaps(size_t t, const std::string& a, size_t aSub,
                     const std::string& b, size_t bSub);
  void writeNotOverlaps(size_t t, const std::string& a, size_t aSub,
                        const std::string& b, size_t bSub);

  void writeCrosses(size_t t, const std::string& a, size_t aSub,
                    const std::string& b, size_t bSub);
  void writeNotCrosses(size_t t, const std::string& a, size_t aSub,
                       const std::string& b, size_t bSub);

  void doCheck(BoxVal cur, SweepVal sv, size_t t);
  void doDistCheck(BoxVal cur, SweepVal sv, size_t t);
  void selfCheck(BoxVal cur, size_t t);

  void processQueue(size_t t);

  void prepareOutputFiles();
  void flushOutputFiles();

  void log(const std::string& msg);

  bool notOverlaps(const std::string& a, const std::string& b);
  bool notTouches(const std::string& a, const std::string& b);
  bool notCrosses(const std::string& a, const std::string& b);

  static double meterDist(const util::geo::I32Point& p1,
                          const util::geo::I32Point& p2);

  void fillBatch(JobBatch* batch,
                 const util::geo::IntervalIdx<int32_t, SweepVal>* actives,
                 const BoxVal* cur) const;

  static int boxCmp(const void* a, const void* b) {
    const auto& boxa = static_cast<const BoxVal*>(a);
    const auto& boxb = static_cast<const BoxVal*>(b);
    if (boxa->val < boxb->val) return -1;
    if (boxa->val > boxb->val) return 1;

    if (!boxa->out && boxb->out) return -1;
    if (boxa->out && !boxb->out) return 1;

    // everything before a polygon
    if (boxa->type != POLYGON && boxa->type != SIMPLE_POLYGON &&
        (boxb->type == POLYGON || boxb->type == SIMPLE_POLYGON))
      return -1;
    if ((boxa->type == POLYGON || boxa->type == SIMPLE_POLYGON) &&
        boxb->type != POLYGON && boxb->type != SIMPLE_POLYGON)
      return 1;

    // points before lines
    if (boxa->type == POINT &&
        (boxb->type == SIMPLE_LINE || boxb->type == LINE))
      return -1;
    if (boxb->type == POINT &&
        (boxa->type == SIMPLE_LINE || boxa->type == LINE))
      return 1;

    // smaller polygons before larger
    if ((boxa->type == POLYGON || boxa->type == SIMPLE_POLYGON) &&
        (boxb->type == POLYGON || boxb->type == SIMPLE_POLYGON) &&
        boxa->areaOrLen < boxb->areaOrLen)
      return -1;
    if ((boxa->type == POLYGON || boxa->type == SIMPLE_POLYGON) &&
        (boxb->type == POLYGON || boxb->type == SIMPLE_POLYGON) &&
        boxa->areaOrLen > boxb->areaOrLen)
      return 1;

    // shorter lines before longer
    if ((boxa->type == LINE || boxa->type == SIMPLE_LINE) &&
        (boxb->type == LINE || boxb->type == SIMPLE_LINE) &&
        boxa->areaOrLen < boxb->areaOrLen)
      return -1;
    if ((boxa->type == LINE || boxa->type == SIMPLE_LINE) == LINE &&
        (boxb->type == LINE || boxb->type == SIMPLE_LINE) &&
        boxa->areaOrLen > boxb->areaOrLen)
      return 1;

    return 0;
  }

  mutable std::mutex _multiAddMtx;
  mutable std::mutex _sweepEventWriteMtx;
  mutable std::mutex _pointGeomCacheWriteMtx;
  mutable std::mutex _lineGeomCacheWriteMtx;
  mutable std::mutex _simpleLineGeomCacheWriteMtx;
  mutable std::mutex _areaGeomCacheWriteMtx;
  mutable std::mutex _simpleAreaGeomCacheWriteMtx;

  std::unordered_map<std::string, std::unordered_map<std::string, size_t>>
      _refs;
};
}  // namespace sj

#endif
