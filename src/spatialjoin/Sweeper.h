// Copyright 2023, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_SWEEPER_H_
#define SPATIALJOINS_SWEEPER_H_

#include <bzlib.h>
#include <fcntl.h>

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

#include "GeometryCache.h"
#include "IntervalIdx.h"
#include "util/JobQueue.h"
#include "util/geo/Geo.h"

#ifndef POSIX_FADV_SEQUENTIAL
#define POSIX_FADV_SEQUENTIAL 2
#endif

namespace sj {

enum GeomType : uint8_t { POLYGON = 0, LINE = 1, POINT = 2, SIMPLE_LINE = 3 };

struct BoxVal {
  size_t id : 60;
  int32_t loY;
  int32_t upY;
  int32_t val;
  bool out : 1;
  GeomType type : 2;
};

inline bool operator==(const BoxVal& a, const BoxVal& b) {
  return a.id == b.id && a.loY == b.loY &&
         a.upY == b.upY && a.type == b.type;
}

struct SweepVal {
  SweepVal(size_t id,  GeomType type)
      : id(id),  type(type) {}
  SweepVal() : id(0), type(POLYGON) {}
  size_t id : 60;
  GeomType type : 2;
};

inline bool operator==(const SweepVal& a, const SweepVal& b) {
  return a.id == b.id && a.type == b.type;
}

inline bool operator<(const SweepVal& a, const SweepVal& b) {
  return a.id < b.id || (a.id == b.id  && a.type < b.type);
}

typedef std::vector<std::pair<BoxVal, SweepVal>> JobBatch;

// buffer sizes _must_ be multiples of sizeof(BoxVal)
static const size_t BUFFER_S = sizeof(BoxVal) * 64 * 1024 * 1024;
static const size_t BUFFER_S_PAIRS = 1024 * 1024 * 10;

class Sweeper {
 public:
  explicit Sweeper(size_t numThreads,
                   const std::string& pairStart, const std::string& sepIsect,
                   const std::string& sepContains, const std::string& pairEnd,
                   bool reUse)
      : _obufpos(0),
        _numThrds(numThreads),
        _areaCache(100000, numThreads, reUse),
        _lineCache(100000, numThreads, reUse),
        _simpleLineCache(100000, numThreads, reUse),
        _sepIsect(sepIsect),
        _sepContains(sepContains),
        _pairStart(pairStart),
        _pairEnd(pairEnd),
        _jobs(100) {
    std::string fname = ".spatialjoins";

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

    // immediately unlink
    // unlink(fname.c_str());

#ifdef __unix__
    posix_fadvise(_file, 0, 0, POSIX_FADV_SEQUENTIAL);
#endif
    _outBuffer = new unsigned char[BUFFER_S];
  };

  void add(const util::geo::I32MultiPolygon& a, size_t gid);
  void add(const util::geo::I32Polygon& a, size_t gid);
  void add(const util::geo::I32Line& a, size_t gid);
  void add(const util::geo::I32Point& a, size_t gid);
  void flush();

  void sweepLine();

 private:
  size_t _curSweepId = 0;
  int _file;
  unsigned char* _outBuffer;
  unsigned char* _outBufferPairs;
  size_t _obufpos;
  size_t _numThrds;

  std::vector<FILE*> _rawFiles;
  std::vector<BZFILE*> _files;
  std::vector<size_t> _outBufPos;
  std::vector<unsigned char*> _outBuffers;

  GeometryCache<Area> _areaCache;
  GeometryCache<Line> _lineCache;
  GeometryCache<SimpleLine> _simpleLineCache;

  std::map<size_t, std::map<size_t, std::set<size_t>>> _subContains;
  std::map<size_t, size_t> _subSizes;

  std::string _sepIsect;
  std::string _sepContains;
  std::string _pairStart;
  std::string _pairEnd;

  util::JobQueue<JobBatch> _jobs;

  std::mutex _mut;

  std::pair<bool, bool> check(const Area* a, const Area* b) const;
  std::pair<bool, bool> check(const Line* a, const Area* b) const;
  std::pair<bool, bool> check(const SimpleLine* a, const Area* b) const;
  bool check(const Line* a, const Line* b) const;
  bool check(const Line* a, const SimpleLine* b) const;
  bool check(const util::geo::I32Point& a, const Area* b) const;

  void diskAdd(const BoxVal& bv);

  void writeIntersect(size_t t, size_t a, size_t b);
  void writeContains(size_t t, size_t a, size_t b);
  void writeContainsMulti(size_t t, size_t a, size_t b, size_t bSub);
  void writeRel(size_t t, size_t a, size_t b, const std::string& pred);

  void doCheck(const BoxVal cur, const SweepVal sv, size_t t);

  void processQueue(size_t t);

  void prepareOutputFiles();
  void flushOutputFiles();

  void fillBatch(JobBatch* batch,
                 const sj::IntervalIdx<int32_t, SweepVal>* actives,
                 const BoxVal* cur) const;

  static int boxCmp(const void* a, const void* b) {
    if (static_cast<const BoxVal*>(a)->val < static_cast<const BoxVal*>(b)->val)
      return -1;
    if (static_cast<const BoxVal*>(a)->val > static_cast<const BoxVal*>(b)->val)
      return 1;
    return 0;
  }
};
}  // namespace sj

#endif
