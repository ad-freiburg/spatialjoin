// Copyright 2026, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_GEOMETRYCACHEMANAGER_H_
#define SPATIALJOINS_GEOMETRYCACHEMANAGER_H_

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <list>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include "GeometryStore.h"
#include "Stats.h"
#include "SweepEventList.h"
#include "util/JobQueue.h"
#include "util/geo/Geo.h"
#include "util/geo/IntervalIdx.h"

#ifndef POSIX_FADV_SEQUENTIAL
#define POSIX_FADV_SEQUENTIAL 2
#endif

namespace sj {

inline std::string toString(const BoxVal& bv) {
  std::stringstream ret;

  ret << "(id=" << getId(bv);
  ret << " loY=" << bv.loY;
  ret << " upY=" << bv.upY;
  ret << " val=" << bv.val;
  ret << " out=" << bv.out;
  ret << " type=" << (int)bv.getType();
  ret << " point=" << util::geo::getWKT(bv.point);
  // ret << " b45=" << util::geo::getWKT(bv.b45);
  ret << " side=" << bv.side;
  ret << ")";

  return ret.str();
}

inline bool operator==(const BoxVal& a, const BoxVal& b) {
  return a.id == b.id && a.loY == b.loY && a.upY == b.upY;
}

struct SweepVal {
  SweepVal(size_t id) : id(id), side(false) {}
  SweepVal(size_t id, util::geo::I32Box b45, util::geo::I32Point point,
           util::geo::I32Point point2, bool side)
      : id(id), b45(b45), point(point), point2(point2), side(side) {}
  SweepVal() : id(typedId(0, POLYGON)) {}
  size_t id;  // typed ID, see typedId()
  util::geo::I32Box b45;
  util::geo::I32Point point, point2;
  bool side;

  size_t getId() const { return untypedId(id); }
  GeomType getType() const {
    return static_cast<GeomType>(id >> ID_TYPE_SHIFT);
  }
};

struct JobVal {
  size_t id;  // typed ID, see typedId()
  util::geo::I32Point point, point2;
  int32_t val;

  JobVal() : id(typedId(0, POLYGON)) {}
  JobVal(const BoxVal& bv)
      : id(bv.id), point(bv.point), point2(getOtherPoint(bv)), val(bv.val){};
  JobVal(const SweepVal& sv)
      : id(sv.id), point(sv.point), point2(sv.point2), val(0){};

  size_t getId() const { return untypedId(id); }
  GeomType getType() const {
    return static_cast<GeomType>(id >> ID_TYPE_SHIFT);
  }
};

inline bool operator==(const JobVal& a, const JobVal& b) {
  return a.id == b.id;
}

inline bool operator==(const SweepVal& a, const SweepVal& b) {
  return a.id == b.id;
}

inline bool operator<(const SweepVal& a, const SweepVal& b) {
  return a.id < b.id;
}

struct Job {
  JobVal boxVal, sweepVal;
  std::string multiOut;
};

inline bool operator==(const Job& a, const Job& b) {
  return a.boxVal == b.boxVal && a.sweepVal == b.sweepVal &&
         a.multiOut == b.multiOut;
}

typedef std::vector<Job> JobBatch;

// intersects, contains, covers, touches, crosses / overlaps
typedef std::tuple<bool, bool, bool, bool, bool> GeomCheckRes;

static const size_t MAX_OUT_LINE_LENGTH = 1000;

static const size_t POINT_CACHE_MAX_ELEMENTS = 10000;
static const size_t SIMPLE_LINE_CACHE_MAX_ELEMENTS = 10000;

template <typename W>
struct ValEntry {
  size_t estimatedSize;
  std::shared_ptr<W> val;
};

// per-thread LRU caches for the geometries of a single GeometryStore
template <typename W>
struct GeomLRUCache {
  GeomLRUCache(size_t maxSize, size_t maxNumElements, size_t numThreads)
      : vals(numThreads),
        idMap(numThreads),
        valSizes(numThreads),
        mutexes(numThreads),
        maxSize(maxSize),
        maxNumElements(maxNumElements) {}

  std::vector<std::list<std::pair<size_t, ValEntry<W>>>> vals;
  std::vector<std::unordered_map<
      size_t, typename std::list<std::pair<size_t, ValEntry<W>>>::iterator>>
      idMap;
  std::vector<size_t> valSizes;
  mutable std::vector<std::mutex> mutexes;

  size_t maxSize, maxNumElements;
};

class GeometryCacheManager {
 public:
  GeometryCacheManager(SweeperCfg cfg, const std::string& cache)
      : GeometryCacheManager(cfg, cache, ".spatialjoin") {}
  GeometryCacheManager(SweeperCfg cfg, const std::string& cache,
                       const std::string& tmpPrefix)
      : _cfg(cfg),
        _store(cfg, cache, tmpPrefix),
        _pointCache(cfg.geomCacheMaxSize, POINT_CACHE_MAX_ELEMENTS,
                    cfg.numCacheThreads),
        _areaCache(cfg.geomCacheMaxSize, cfg.geomCacheMaxNumElements,
                   cfg.numCacheThreads),
        _simpleAreaCache(cfg.geomCacheMaxSize, cfg.geomCacheMaxNumElements,
                         cfg.numCacheThreads),
        _lineCache(cfg.geomCacheMaxSize, cfg.geomCacheMaxNumElements,
                   cfg.numCacheThreads),
        _simpleLineCache(cfg.geomCacheMaxSize, SIMPLE_LINE_CACHE_MAX_ELEMENTS,
                         cfg.numCacheThreads) {}

  void log(const std::string& msg);

  GeometryStore& getStore() { return _store; }
  const GeometryStore& getStore() const { return _store; }

  std::shared_ptr<sj::Line> getLine(size_t id, size_t tid) const {
    return get(_store.lineStore(), _lineCache, id, tid);
  }
  std::shared_ptr<sj::SimpleLine> getSimpleLine(size_t id, size_t tid) const {
    return get(_store.simpleLineStore(), _simpleLineCache, id, tid);
  }

  std::shared_ptr<sj::Area> getArea(size_t id, size_t tid) const {
    return get(_store.areaStore(), _areaCache, id, tid);
  }

  std::shared_ptr<sj::SimpleArea> getSimpleArea(size_t id, size_t tid) const {
    return get(_store.simpleAreaStore(), _simpleAreaCache, id, tid);
  }

  std::shared_ptr<sj::Point> getPoint(size_t id, size_t tid) const {
    return get(_store.pointStore(), _pointCache, id, tid);
  }

  std::pair<size_t, size_t> size() const;

 private:
  // returns the geometry stored at off in store, cached in the LRU cache of
  // thread tid
  template <typename W>
  std::shared_ptr<W> get(const TypedGeometryStore<W>& store,
                         GeomLRUCache<W>& cache, size_t off, size_t tid) const;

  template <typename W>
  std::shared_ptr<W> cache(GeomLRUCache<W>& cache, size_t off, const W& val,
                           size_t estSize, size_t tid) const;

  template <typename W>
  static std::pair<size_t, size_t> size(const GeomLRUCache<W>& cache);

  const SweeperCfg _cfg;

  GeometryStore _store;

  mutable GeomLRUCache<Point> _pointCache;
  mutable GeomLRUCache<Area> _areaCache;
  mutable GeomLRUCache<SimpleArea> _simpleAreaCache;
  mutable GeomLRUCache<Line> _lineCache;
  mutable GeomLRUCache<SimpleLine> _simpleLineCache;
};

}  // namespace sj

#endif
