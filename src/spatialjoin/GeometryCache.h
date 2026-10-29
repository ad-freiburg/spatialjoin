// Copyright 2026, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_GEOMETRYCACHE_H_
#define SPATIALJOINS_GEOMETRYCACHE_H_

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
struct TypedGeometryCache {
  TypedGeometryCache(size_t maxSize, size_t maxNumElements, size_t numThreads)
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

class GeometryCache {
 public:
  GeometryCache(SweeperCfg cfg) : _cfg(cfg) {}

  void log(const std::string& msg);

  // adds a (non-owned) geometry store, returns its store id
  size_t addStore(const GeometryStore* store);

  const GeometryStore& getStore(size_t storeId = 0) const {
    return *_stores[storeId];
  }

  std::shared_ptr<sj::Line> getLine(size_t id, size_t tid,
                                    size_t storeId = 0) const {
    return get(_stores[storeId]->lineStore(), _caches[storeId]->lineCache, id,
               tid);
  }
  std::shared_ptr<sj::SimpleLine> getSimpleLine(size_t id, size_t tid,
                                                size_t storeId = 0) const {
    return get(_stores[storeId]->simpleLineStore(),
               _caches[storeId]->simpleLineCache, id, tid);
  }

  std::shared_ptr<sj::Area> getArea(size_t id, size_t tid,
                                    size_t storeId = 0) const {
    return get(_stores[storeId]->areaStore(), _caches[storeId]->areaCache, id,
               tid);
  }

  std::shared_ptr<sj::SimpleArea> getSimpleArea(size_t id, size_t tid,
                                                size_t storeId = 0) const {
    return get(_stores[storeId]->simpleAreaStore(),
               _caches[storeId]->simpleAreaCache, id, tid);
  }

  std::shared_ptr<sj::Point> getPoint(size_t id, size_t tid,
                                      size_t storeId = 0) const {
    return get(_stores[storeId]->pointStore(), _caches[storeId]->pointCache,
               id, tid);
  }

  std::pair<size_t, size_t> size() const;

 private:
  // the LRU caches for the geometries of a single GeometryStore
  struct StoreCaches {
    StoreCaches(const SweeperCfg& cfg)
        : pointCache(cfg.geomCacheMaxSize, POINT_CACHE_MAX_ELEMENTS,
                     cfg.numCacheThreads),
          areaCache(cfg.geomCacheMaxSize, cfg.geomCacheMaxNumElements,
                    cfg.numCacheThreads),
          simpleAreaCache(cfg.geomCacheMaxSize, cfg.geomCacheMaxNumElements,
                          cfg.numCacheThreads),
          lineCache(cfg.geomCacheMaxSize, cfg.geomCacheMaxNumElements,
                    cfg.numCacheThreads),
          simpleLineCache(cfg.geomCacheMaxSize, SIMPLE_LINE_CACHE_MAX_ELEMENTS,
                          cfg.numCacheThreads) {}

    TypedGeometryCache<Point> pointCache;
    TypedGeometryCache<Area> areaCache;
    TypedGeometryCache<SimpleArea> simpleAreaCache;
    TypedGeometryCache<Line> lineCache;
    TypedGeometryCache<SimpleLine> simpleLineCache;
  };

  // returns the geometry stored at off in store, cached in the LRU cache of
  // thread tid
  template <typename W>
  std::shared_ptr<W> get(const TypedGeometryStore<W>& store,
                         TypedGeometryCache<W>& cache, size_t off,
                         size_t tid) const;

  template <typename W>
  std::shared_ptr<W> cache(TypedGeometryCache<W>& cache, size_t off,
                           const W& val, size_t estSize, size_t tid) const;

  template <typename W>
  static std::pair<size_t, size_t> size(const TypedGeometryCache<W>& cache);

  const SweeperCfg _cfg;

  std::vector<const GeometryStore*> _stores;
  std::vector<std::unique_ptr<StoreCaches>> _caches;
};

}  // namespace sj

#endif
