// Copyright 2023, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_SWEEPER_H_
#define SPATIALJOINS_SWEEPER_H_

#include <bzlib.h>
#include <zlib.h>
#include <fcntl.h>

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

#include "GeometryCache.h"
#include "IntervalIdx.h"
#include "Stats.h"
#include "util/JobQueue.h"
#include "util/geo/Geo.h"

#ifndef POSIX_FADV_SEQUENTIAL
#define POSIX_FADV_SEQUENTIAL 2
#endif

namespace sj {

enum GeomType : uint8_t { POLYGON = 0, LINE = 1, POINT = 2, SIMPLE_LINE = 3 };
enum OutMode : uint8_t { PLAIN = 0, BZ2 = 1, GZ = 2, COUT = 3 };

struct BoxVal {
  size_t id : 60;
  int32_t loY;
  int32_t upY;
  int32_t val;
  bool out : 1;
  GeomType type : 2;
  double areaOrLen;
};

inline bool operator==(const BoxVal& a, const BoxVal& b) {
  return a.id == b.id && a.loY == b.loY && a.upY == b.upY && a.type == b.type;
}

struct SweepVal {
  SweepVal(size_t id, GeomType type) : id(id), type(type) {}
  SweepVal() : id(0), type(POLYGON) {}
  size_t id : 60;
  GeomType type : 2;
};

inline bool operator==(const SweepVal& a, const SweepVal& b) {
  return a.id == b.id && a.type == b.type;
}

inline bool operator<(const SweepVal& a, const SweepVal& b) {
  return a.id < b.id || (a.id == b.id && a.type < b.type);
}

typedef std::vector<std::pair<BoxVal, SweepVal>> JobBatch;

struct SweeperCfg {
  size_t numThreads;
  std::string pairStart;
  std::string sepIsect;
  std::string sepContains;
  std::string sepCovers;
  std::string pairEnd;
  bool useBoxIds;
  bool useArea;
};

// buffer sizes _must_ be multiples of sizeof(BoxVal)
static const size_t BUFFER_S = sizeof(BoxVal) * 64 * 1024 * 1024;
static const size_t BUFFER_S_PAIRS = 1024 * 1024 * 10;

class Sweeper {
 public:
  explicit Sweeper(SweeperCfg cfg, bool reUse, const std::string cache,
                   const std::string out)
      : _cfg(cfg),
        _obufpos(0),
        _pointCache(100000, cfg.numThreads, cache, reUse),
        _areaCache(100000, cfg.numThreads, cache, reUse),
        _lineCache(100000, cfg.numThreads, cache, reUse),
        _simpleLineCache(100000, cfg.numThreads, cache, reUse),
        _cache(cache),
        _out(out),
        _jobs(100) {
    if (util::endsWith(_out, ".bz2")) _outMode = BZ2;
    else if (util::endsWith(_out, ".gz")) _outMode = GZ;
    else if (out.size()) _outMode = PLAIN;
    else _outMode = COUT;

    std::string fname = _cache + "/events";

    if (reUse) {
      _file = open(fname.c_str(), O_RDONLY);
      _curSweepId = (lseek(_file, 0, SEEK_END) + 1) / (2 * sizeof(BoxVal));
      lseek(_file, 0, SEEK_SET);
    } else {
      _file = open(fname.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0666);
    }

    if (_file < 0) {
      throw std::runtime_error("Could not open temporary file " + fname);
    }

#ifdef __unix__
    posix_fadvise(_file, 0, 0, POSIX_FADV_SEQUENTIAL);
#endif
    _outBuffer = new unsigned char[BUFFER_S];
  };

  void add(const util::geo::I32MultiPolygon& a, const std::string& gid);
  void add(const util::geo::I32MultiLine& a, const std::string& gid);
  void add(const util::geo::I32MultiPoint& a, const std::string& gid);
  size_t add(const util::geo::I32MultiPolygon& a, const std::string& gid,
             size_t);
  size_t add(const util::geo::I32MultiLine& a, const std::string& gid, size_t);
  size_t add(const util::geo::I32MultiPoint& a, const std::string& gid, size_t);
  void add(const util::geo::I32Polygon& a, const std::string& gid);
  void add(const util::geo::I32Polygon& a, const std::string& gid,
           size_t subId);
  void add(const util::geo::I32Line& a, const std::string& gid);
  void add(const util::geo::I32Line& a, const std::string& gid, size_t subid);
  void add(const util::geo::I32Point& a, const std::string& gid);
  void add(const util::geo::I32Point& a, const std::string& gid, size_t subid);
  void flush();

  void sweep();

 private:
  const SweeperCfg _cfg;
  size_t _curSweepId = 0;
  int _file;
  unsigned char* _outBuffer;
  size_t _obufpos;

  OutMode _outMode;

  std::vector<FILE*> _rawFiles;
  std::vector<BZFILE*> _files;
  std::vector<gzFile> _gzFiles;
  std::vector<size_t> _outBufPos;
  std::vector<unsigned char*> _outBuffers;

  mutable std::vector<Stats> _stats;

  GeometryCache<Point> _pointCache;
  GeometryCache<Area> _areaCache;
  GeometryCache<Line> _lineCache;
  GeometryCache<SimpleLine> _simpleLineCache;

  std::map<std::string, std::map<std::string, std::set<size_t>>> _subContains;
  std::map<std::string, std::map<std::string, std::set<size_t>>> _subCovered;
  std::map<std::string, size_t> _subSizes;

  std::string _cache;
  std::string _out;

  util::JobQueue<JobBatch> _jobs;

  std::mutex _mut;

  std::tuple<bool, bool, bool> check(const Area* a, const Area* b,
                                     size_t t) const;
  std::tuple<bool, bool, bool> check(const Line* a, const Area* b,
                                     size_t t) const;
  std::tuple<bool, bool, bool> check(const SimpleLine* a, const Area* b,
                                     size_t t) const;
  std::pair<bool, bool> check(const Line* a, const Line* b, size_t t) const;
  std::pair<bool, bool> check(const Line* a, const SimpleLine* b,
                              size_t t) const;
  std::pair<bool, bool> check(const SimpleLine* a, const SimpleLine* b,
                              size_t t) const;
  std::pair<bool, bool> check(const SimpleLine* a, const Line* b,
                              size_t t) const;
  std::pair<bool, bool> check(const util::geo::I32Point& a, const Area* b,
                              size_t t) const;
  bool check(const util::geo::I32Point& a, const Line* b, size_t t) const;

  void diskAdd(const BoxVal& bv);

  void writeIntersect(size_t t, const std::string& a, const std::string& b);
  void writeContains(size_t t, const std::string& a, const std::string& b);
  void writeCovers(size_t t, const std::string& a, const std::string& b);
  void writeRel(size_t t, const std::string& a, const std::string& b,
                const std::string& pred);
  void writeContainsMulti(size_t t, const std::string& a, const std::string& b,
                          size_t bSub);
  void writeCoversMulti(size_t t, const std::string& a, const std::string& b,
                        size_t bSub);

  void doCheck(const BoxVal cur, const SweepVal sv, size_t t);

  void processQueue(size_t t);

  void prepareOutputFiles();
  void flushOutputFiles();

  void fillBatch(JobBatch* batch,
                 const sj::IntervalIdx<int32_t, SweepVal>* actives,
                 const BoxVal* cur) const;

  static int boxCmp(const void* a, const void* b) {
    const auto& boxa = static_cast<const BoxVal*>(a);
    const auto& boxb = static_cast<const BoxVal*>(b);
    if (boxa->val < boxb->val) return -1;
    if (boxa->val > boxb->val) return 1;

    if (!boxa->out && boxb->out) return -1;
    if (boxa->out && !boxb->out) return 1;

    // everything before a polygon
    if (boxa->type != POLYGON && boxb->type == POLYGON) return -1;
    if (boxa->type == POLYGON && boxb->type != POLYGON) return 1;

    // points before lines
    if (boxa->type == POINT &&
        (boxb->type == SIMPLE_LINE || boxb->type == LINE))
      return -1;
    if (boxb->type == POINT &&
        (boxa->type == SIMPLE_LINE || boxa->type == LINE))
      return 1;

    // smaller polygons before larger
    if (boxa->type == POLYGON && boxb->type == POLYGON &&
        boxa->areaOrLen < boxb->areaOrLen)
      return -1;
    if (boxa->type == POLYGON && boxb->type == POLYGON &&
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
};
}  // namespace sj

#endif
