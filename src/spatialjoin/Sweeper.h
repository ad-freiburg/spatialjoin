// Copyright 2023, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_SWEEPER_H_
#define SPATIALJOINS_SWEEPER_H_

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

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
  SIMPLE_POLYGON = 4,
  FOLDED_POINT = 5,
  // currently not used
  FOLDED_SIMPLE_LINE_UP = 6,
  FOLDED_SIMPLE_LINE_DOWN = 7
};

struct BoxVal {
  size_t id;
  int32_t loY;
  int32_t upY;
  int32_t val;
  bool out : 1;
  GeomType type : 3;
  double areaOrLen;
  util::geo::I32Point point;
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
  std::vector<WriteCand> foldedPoints;
  std::vector<WriteCand> simpleLines;
  std::vector<WriteCand> lines;
  std::vector<WriteCand> simpleAreas;
  std::vector<WriteCand> areas;
  std::vector<WriteCand> refs;

  size_t size() const {
    return points.size() + foldedPoints.size() + simpleLines.size() +
           lines.size() + simpleAreas.size() + areas.size() + refs.size();
  }
};

inline bool operator==(const BoxVal& a, const BoxVal& b) {
  return a.id == b.id && a.loY == b.loY && a.upY == b.upY && a.type == b.type;
}

struct SweepVal {
  SweepVal(size_t id, GeomType type)
      : id(id), type(type), side(false), large(false) {}
  SweepVal(size_t id, GeomType type, util::geo::I32Box b45,
           util::geo::I32Point point, bool side, bool large)
      : id(id), type(type), b45(b45), point(point), side(side), large(large) {}
  SweepVal() : id(0), type(POLYGON) {}
  size_t id;
  GeomType type : 3;
  util::geo::I32Box b45;
  util::geo::I32Point point;
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

typedef std::tuple<bool, bool, bool, bool, bool> GeomCheckRes;

struct SweeperCfg {
  size_t numThreads;
  size_t numCacheThreads;
  size_t geomCacheMaxSize;
  size_t geomCacheMaxNumElements;
  std::string sepIsect;
  std::string sepContains;
  std::string sepCovers;
  std::string sepTouches;
  std::string sepEquals;
  std::string sepOverlaps;
  std::string sepCrosses;
  bool useBoxIds;
  bool useArea;
  bool useOBB;
  bool useDiagBox;
  bool useFastSweepSkip;
  bool useInnerOuter;
  bool noGeometryChecks;
  double withinDist;
  bool computeDE9IM;
  std::function<void(size_t t, const char* a, size_t an, const char* b,
                     size_t bn, const char* pred, size_t predn)>
      writeRelCb;
  std::function<void(const std::string&)> logCb;
  std::function<void(const std::string&)> statsCb;
  std::function<void(size_t)> sweepProgressCb;
  std::function<void()> sweepCancellationCb;
};

// buffer size _must_ be multiples of sizeof(BoxVal)
static const ssize_t BUFFER_S = sizeof(BoxVal) * 64 * 1024 * 512;

static const size_t MAX_OUT_LINE_LENGTH = 1000;

static const size_t POINT_CACHE_MAX_ELEMENTS = 10000;
static const size_t SIMPLE_LINE_CACHE_MAX_ELEMENTS = 10000;

// only use large geom cache for extreme geometries
static const size_t GEOM_LARGENESS_THRESHOLD = 1024 * 1024 * 1024;

class Sweeper {
 public:
  Sweeper(SweeperCfg cfg, const std::string& cache)
      : Sweeper(cfg, cache, ".spatialjoin") {}
  Sweeper(SweeperCfg cfg, const std::string& cache,
          const std::string& tmpPrefix)
      : _cfg(cfg),
        _obufpos(0),
        _pointCache({cfg.useOBB, cfg.useInnerOuter}, cfg.geomCacheMaxSize,
                    POINT_CACHE_MAX_ELEMENTS, cfg.numCacheThreads, cache,
                    tmpPrefix),
        _areaCache({cfg.useOBB, cfg.useInnerOuter}, cfg.geomCacheMaxSize,
                   cfg.geomCacheMaxNumElements, cfg.numCacheThreads, cache,
                   tmpPrefix),
        _simpleAreaCache({cfg.useOBB, cfg.useInnerOuter}, cfg.geomCacheMaxSize,
                         cfg.geomCacheMaxNumElements, cfg.numCacheThreads,
                         cache, tmpPrefix),
        _lineCache({cfg.useOBB, cfg.useInnerOuter}, cfg.geomCacheMaxSize,
                   cfg.geomCacheMaxNumElements, cfg.numCacheThreads, cache,
                   tmpPrefix),
        _simpleLineCache({cfg.useOBB, cfg.useInnerOuter}, cfg.geomCacheMaxSize,
                         SIMPLE_LINE_CACHE_MAX_ELEMENTS, cfg.numCacheThreads,
                         cache, tmpPrefix),
        _cache(cache),
        _jobs(100) {
    if (!_cfg.writeRelCb) {
    }

    // OUTFACTOR 1
    _fname = util::getTmpFName(_cache, tmpPrefix, "events");
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

  ~Sweeper() { close(_file); }

  void log(const std::string& msg);

  util::geo::I32Box add(const util::geo::I32MultiPolygon& a,
                        const std::string& gid, bool side,
                        WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32MultiPolygon& a,
                        const std::string& gid, size_t, bool side,
                        WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32Polygon& a, const std::string& gid,
                        bool side, WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32Polygon& a, const std::string& gid,
                        size_t subId, bool side, WriteBatch& batch) const;

  util::geo::I32Box add(const util::geo::I32MultiLine& a,
                        const std::string& gid, size_t, bool side,
                        WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32MultiLine& a,
                        const std::string& gid, bool side,
                        WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32Line& a, const std::string& gid,
                        bool side, WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32Line& a, const std::string& gid,
                        size_t subid, bool side, WriteBatch& batch) const;

  util::geo::I32Box add(const util::geo::I32Point& a, const std::string& gid,
                        bool side, WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32Point& a, const std::string& gid,
                        size_t subid, bool side, WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32MultiPoint& a,
                        const std::string& gid, size_t, bool side,
                        WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32MultiPoint& a,
                        const std::string& gid, bool side,
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

  void setFilterBox(const util::geo::I32Box& filterBox) {
    _filterBox = filterBox;
  }

  // _____________________________________________________________________________
  template <template <typename> class G, typename T>
  util::geo::I32Box getPaddedBoundingBox(const G<T>& geom) const {
    return getPaddedBoundingBox(geom, geom);
  }

  // _____________________________________________________________________________
  template <template <typename> class G1, template <typename> class G2,
            typename T>
  util::geo::I32Box getPaddedBoundingBox(const G1<T>& geom,
                                         const G2<T>& refGeom) const {
    auto bbox = util::geo::getBoundingBox(geom);

    if (_cfg.withinDist >= 0) {
      double scaleFactor =
          getMaxScaleFactor(reinterpret_cast<const void*>(&geom) ==
                                    reinterpret_cast<const void*>(&refGeom)
                                ? bbox
                                : util::geo::getBoundingBox(refGeom));

      double pad = (_cfg.withinDist / 2.0) * scaleFactor * PREC;

      double llx = bbox.getLowerLeft().getX();
      double lly = bbox.getLowerLeft().getY();
      double urx = bbox.getUpperRight().getX();
      double ury = bbox.getUpperRight().getY();

      double m = sj::boxids::WORLD_W / 2.0;

      T llxt = -m;
      T llyt = -m;
      T urxt = m;
      T uryt = m;

      if (llx - pad > -m) {
        llxt = llx - pad;
      }

      if (lly - pad > -m) {
        llyt = lly - pad;
      }

      if (urx + pad < m) {
        urxt = urx + pad;
      }

      if (ury + pad < m) {
        uryt = ury + pad;
      }

      return {{llxt, llyt}, {urxt, uryt}};
    }

    return bbox;
  }

  static size_t foldString(const std::string& s) {
    size_t ret = 0;
    for (size_t i = 0; i < std::min((size_t)7, s.size()); i++) {
      size_t tmp = static_cast<unsigned char>(s[i]);
      ret |= tmp << (i * 8);
    }

    // highest byte stores the length
    ret |= (s.size() << 56);

    return ret;
  };

  static std::string unfoldString(size_t folded) {
    // shift by 7 bytes to get size
    size_t n = folded >> 56;

    std::string ret;
    ret.reserve(n);

    for (size_t i = 0; i < n; i++) {
      ret.push_back(static_cast<char>(folded >> (i * 8)) & 0xFF);
    }

    return ret;
  };

 private:
  const SweeperCfg _cfg;
  size_t _curSweepId = 0;
  std::string _fname;
  int _file;
  unsigned char* _outBuffer;
  ssize_t _obufpos;

  std::vector<size_t> _checks;
  std::vector<int32_t> _curX;
  std::vector<std::atomic<int32_t>> _atomicCurX;
  std::atomic<bool> _cancelled;

  std::vector<RelStats> _relStats;

  mutable std::vector<Stats> _stats;

  GeometryCache<Point> _pointCache;
  GeometryCache<Area> _areaCache;
  GeometryCache<SimpleArea> _simpleAreaCache;
  GeometryCache<Line> _lineCache;
  GeometryCache<SimpleLine> _simpleLineCache;

  std::vector<std::map<std::string, std::map<std::string, double>>>
      _subDistance;
  std::vector<
      std::map<std::string, std::map<std::string, util::geo::DE9IMatrix>>>
      _subDE9IM;
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
  std::vector<std::mutex> _mutsDistance;
  std::vector<std::mutex> _mutsDE9IM;

  Area areaFromSimpleArea(const SimpleArea* sa) const;
  Line lineFromSimpleLine(const SimpleLine* sl) const;

  GeomCheckRes check(const Area* a, const Area* b, size_t t) const;
  GeomCheckRes check(const Line* a, const Area* b, size_t t) const;
  GeomCheckRes check(const SimpleLine* a, const Area* b, size_t t) const;
  GeomCheckRes check(const Line* a, const Line* b, size_t t) const;
  GeomCheckRes check(const Line* a, const SimpleLine* b, size_t t) const;
  GeomCheckRes check(const SimpleLine* a, const SimpleLine* b, size_t t) const;
  GeomCheckRes check(const SimpleLine* a, const Line* b, size_t t) const;
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

  util::geo::DE9IMatrix DE9IMCheck(const util::geo::I32Point& a, const Area* b,
                                   size_t t) const;
  util::geo::DE9IMatrix DE9IMCheck(const util::geo::I32Point& a, const Line* b,
                                   size_t t) const;
  util::geo::DE9IMatrix DE9IMCheck(const util::geo::I32Point& a,
                                   const SimpleLine* b, size_t t) const;
  util::geo::DE9IMatrix DE9IMCheck(const SimpleLine* a, const SimpleLine* b,
                                   size_t t) const;
  util::geo::DE9IMatrix DE9IMCheck(const SimpleLine* a, const Line* b,
                                   size_t t) const;
  util::geo::DE9IMatrix DE9IMCheck(const SimpleLine* a, const Area* b,
                                   size_t t) const;
  util::geo::DE9IMatrix DE9IMCheck(const Line* a, const Line* b,
                                   size_t t) const;
  util::geo::DE9IMatrix DE9IMCheck(const Area* a, const Area* b,
                                   size_t t) const;
  util::geo::DE9IMatrix DE9IMCheck(const Line* a, const Area* b,
                                   size_t t) const;

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
  void writeContains(size_t t, const std::string& a, const std::string& b,
                     size_t bSub);
  void writeCovers(size_t t, const std::string& a, const std::string& b,
                   size_t bSub);
  void writeEquals(size_t t, const std::string& a, size_t aSub,
                   const std::string& b, size_t bSub);
  void writeDE9IM(size_t t, const std::string& a, size_t aSub,
                  const std::string& b, size_t bSub,
                  util::geo::DE9IMatrix de9im);
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
  void doDE9IMCheck(BoxVal cur, SweepVal sv, size_t t);
  void selfCheck(BoxVal cur, size_t t);

  void processQueue(size_t t);

  bool notOverlaps(const std::string& a, const std::string& b);
  bool notTouches(const std::string& a, const std::string& b);
  bool notCrosses(const std::string& a, const std::string& b);

  std::shared_ptr<sj::Point> getPoint(size_t id, GeomType gt, size_t t) const;
  static bool isPoint(GeomType gt) { return gt == POINT || gt == FOLDED_POINT; }

  std::shared_ptr<sj::SimpleLine> getSimpleLine(size_t id, GeomType gt,
                                                size_t t) const;
  static bool isLine(GeomType gt) {
    return gt == LINE || gt == SIMPLE_LINE || gt == FOLDED_SIMPLE_LINE_UP ||
           gt == FOLDED_SIMPLE_LINE_DOWN;
  }

  static bool isSimpleLine(GeomType gt) {
    return gt == SIMPLE_LINE || gt == FOLDED_SIMPLE_LINE_UP ||
           gt == FOLDED_SIMPLE_LINE_DOWN;
  }

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
    if ((boxa->type == POINT || boxa->type == FOLDED_POINT) &&
        (boxb->type == SIMPLE_LINE || boxb->type == LINE))
      return -1;
    if ((boxb->type == POINT || boxb->type == FOLDED_POINT) &&
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

  util::geo::I32Box _filterBox = {{std::numeric_limits<int32_t>::lowest(),
                                   std::numeric_limits<int32_t>::lowest()},
                                  {std::numeric_limits<int32_t>::max(),
                                   std::numeric_limits<int32_t>::max()}};
};
}  // namespace sj

#endif
