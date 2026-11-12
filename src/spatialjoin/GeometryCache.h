// Copyright 2026, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_GEOMETRYCACHE_H_
#define SPATIALJOINS_GEOMETRYCACHE_H_

#include <atomic>
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
  ValEntry(size_t estimatedSize, std::shared_ptr<W> val)
      : estimatedSize(estimatedSize), val(std::move(val)), touched(false) {}
  size_t estimatedSize;
  std::shared_ptr<W> val;

  mutable std::atomic<bool> touched;
};

template <typename W>
struct TypedGeometryCache {
  TypedGeometryCache(size_t maxSize, size_t maxNumElements, size_t numBuckets)
      : vals(numBuckets),
        idMap(numBuckets),
        valSizes(numBuckets),
        mutexes(numBuckets),
        maxSize(maxSize),
        maxNumElements(maxNumElements) {}

  std::vector<std::list<std::pair<size_t, ValEntry<W>>>> vals;
  std::vector<std::unordered_map<
      size_t, typename std::list<std::pair<size_t, ValEntry<W>>>::iterator>>
      idMap;
  std::vector<size_t> valSizes;
  mutable std::vector<SharedMutex> mutexes;

  size_t maxSize, maxNumElements;
};

class GeometryCache {
 public:
  GeometryCache(SpatialJoinCfg cfg) : _cfg(cfg) {
    if (cfg.numCacheBuckets == 0)
      throw std::runtime_error("Need at least one cache bucket");
  }

  GeometryCache(const GeometryCache&) = delete;
  GeometryCache& operator=(const GeometryCache&) = delete;

  void log(const std::string& msg);

  // adds a geometry store, returns its store id
  uint8_t addStore(const GeometryStore* store);

  const GeometryStore& getStore(size_t storeId = 0) const {
    return *_stores[storeId];
  }

  const SpatialJoinCfg& cfg() const { return _cfg; }

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

  // the gid and sub id of the area / line with id, read without reading the
  // geometry, and cached in the id cache
  GeomId getAreaId(size_t id, size_t tid, size_t storeId = 0) const {
    return *getId(_stores[storeId]->areaStore(), _caches[storeId]->areaIdCache,
                  id, tid);
  }

  GeomId getLineId(size_t id, size_t tid, size_t storeId = 0) const {
    return *getId(_stores[storeId]->lineStore(), _caches[storeId]->lineIdCache,
                  id, tid);
  }

  std::pair<size_t, size_t> size() const;

 private:
  // the caches for the geometries of a single GeometryStore
  struct StoreCaches {
    StoreCaches(const SpatialJoinCfg& cfg)
        : pointCache(cfg.geomCacheMaxSize, cfg.geomCacheMaxNumElements,
                     cfg.numCacheBuckets),
          areaCache(cfg.geomCacheMaxSize, cfg.geomCacheMaxNumElements,
                    cfg.numCacheBuckets),
          simpleAreaCache(cfg.geomCacheMaxSize, cfg.geomCacheMaxNumElements,
                          cfg.numCacheBuckets),
          lineCache(cfg.geomCacheMaxSize, cfg.geomCacheMaxNumElements,
                    cfg.numCacheBuckets),
          simpleLineCache(cfg.geomCacheMaxSize, cfg.geomCacheMaxNumElements,
                          cfg.numCacheBuckets),
          areaIdCache(cfg.geomCacheMaxSize, cfg.geomCacheMaxNumElements,
                      cfg.numCacheBuckets),
          lineIdCache(cfg.geomCacheMaxSize, cfg.geomCacheMaxNumElements,
                      cfg.numCacheBuckets) {}

    TypedGeometryCache<Point> pointCache;
    TypedGeometryCache<Area> areaCache;
    TypedGeometryCache<SimpleArea> simpleAreaCache;
    TypedGeometryCache<Line> lineCache;
    TypedGeometryCache<SimpleLine> simpleLineCache;

    // caches for the ids only, see getAreaId() / getLineId()
    TypedGeometryCache<GeomId> areaIdCache;
    TypedGeometryCache<GeomId> lineIdCache;
  };

  // returns the geometry stored at off in store, cached in the cache bucket of
  // off
  template <typename W>
  std::shared_ptr<W> get(const TypedGeometryStore<W>& store,
                         TypedGeometryCache<W>& cache, size_t off,
                         size_t tid) const;

  // returns the gid and sub id of the geometry stored at off in store, cached
  template <typename W>
  std::shared_ptr<GeomId> getId(const TypedGeometryStore<W>& store,
                                TypedGeometryCache<GeomId>& cache, size_t off,
                                size_t tid) const;

  template <typename W>
  std::shared_ptr<W> cache(TypedGeometryCache<W>& cache, size_t off, W&& val,
                           size_t estSize, size_t tid) const;

  template <typename W>
  static std::pair<size_t, size_t> size(const TypedGeometryCache<W>& cache);

  const SpatialJoinCfg _cfg;

  std::vector<const GeometryStore*> _stores;
  std::vector<std::unique_ptr<StoreCaches>> _caches;
};

}  // namespace sj

#endif
