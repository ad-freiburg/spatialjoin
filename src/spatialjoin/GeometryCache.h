// Copyright 2026, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_GEOMETRYCACHE_H_
#define SPATIALJOINS_GEOMETRYCACHE_H_

#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "GeometryStore.h"
#include "SweepEventList.h"
#include "util/geo/Geo.h"

namespace sj {

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
    return get(_stores[storeId]->pointStore(), _caches[storeId]->pointCache, id,
               tid);
  }

  std::pair<size_t, size_t> size() const;

 private:
  // the LRU caches for the geometries of a single GeometryStore
  struct StoreCaches {
    StoreCaches(const SweeperCfg& cfg)
        : pointCache(cfg.geomCacheMaxSize, cfg.geomCacheMaxNumElements,
                     cfg.numCacheThreads),
          areaCache(cfg.geomCacheMaxSize, cfg.geomCacheMaxNumElements,
                    cfg.numCacheThreads),
          simpleAreaCache(cfg.geomCacheMaxSize, cfg.geomCacheMaxNumElements,
                          cfg.numCacheThreads),
          lineCache(cfg.geomCacheMaxSize, cfg.geomCacheMaxNumElements,
                    cfg.numCacheThreads),
          simpleLineCache(cfg.geomCacheMaxSize, cfg.geomCacheMaxNumElements,
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
