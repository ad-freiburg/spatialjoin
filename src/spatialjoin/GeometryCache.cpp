#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#ifndef SPATIALJOIN_NO_ZLIB
#include <zlib.h>
#endif

#ifndef SPATIALJOIN_NO_BZIP2
#include <bzlib.h>
#endif

#include <algorithm>
#include <cassert>
#include <climits>
#include <iterator>
#include <tuple>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

#include "BoxIds.h"
#include "GeometryCache.h"
#include "util/Misc.h"
#include "util/log/Log.h"

using sj::GeometryCache;
using sj::GeomType;
using sj::boxids::boxIdIsect;
using sj::boxids::BoxIdList;
using sj::boxids::getBoxId;
using sj::boxids::getBoxIds;
using sj::boxids::packBoxIds;
using util::preadAll;
using util::pwriteAll;
using util::readAll;
using util::writeAll;
using util::geo::area;
using util::geo::DE9IM;
using util::geo::DPoint;
using util::geo::FPoint;
using util::geo::getBoundingBox;
using util::geo::I32Box;
using util::geo::I32Line;
using util::geo::I32MultiLine;
using util::geo::I32MultiPoint;
using util::geo::I32MultiPolygon;
using util::geo::I32Point;
using util::geo::I32Polygon;
using util::geo::I32XSortedLine;
using util::geo::I32XSortedPolygon;
using util::geo::intersectsContainsCovers;
using util::geo::intersectsCovers;
using util::geo::LineSegment;
using util::geo::Point;
using util::geo::webMercToLatLng;
using util::LogLevel::DEBUG;
using util::LogLevel::ERROR;
using util::LogLevel::INFO;
using util::LogLevel::VDEBUG;
using util::LogLevel::WARN;

// _____________________________________________________________________________
template <typename W>
std::shared_ptr<W> GeometryCache::get(const TypedGeometryStore<W>& store,
                                      TypedGeometryCache<W>& cache, size_t off,
                                      size_t tid) const {
  // completely circumvent cache system
  if (store.inMemory()) return store.getFromMem(off);

  size_t cid = off % _cfg.numCacheBuckets;

  {
    SharedLock lock(cache.mutexes[cid]);
    auto it = cache.idMap[cid].find(off);
    if (it != cache.idMap[cid].end()) {
      it->second->second.touched.store(true, std::memory_order_relaxed);
      return it->second->second.val;
    }
  }

  // not in cache!
  auto val = store.getFrom(off, tid);

  return this->cache(cache, off, std::move(val.second), val.first, cid);
}

// _____________________________________________________________________________
template <typename W>
std::shared_ptr<sj::GeomId> GeometryCache::getId(
    const TypedGeometryStore<W>& store, TypedGeometryCache<GeomId>& cache,
    size_t off, size_t tid) const {
  // completely circumvent cache system
  if (store.inMemory()) {
    return std::make_shared<GeomId>(store.getId(off, 0).second);
  }

  size_t cid = off % _cfg.numCacheBuckets;

  {
    SharedLock lock(cache.mutexes[cid]);
    auto it = cache.idMap[cid].find(off);
    if (it != cache.idMap[cid].end()) {
      it->second->second.touched.store(true, std::memory_order_relaxed);
      return it->second->second.val;
    }
  }

  // not in cache!
  auto val = store.getId(off, tid);

  return this->cache(cache, off, std::move(val.second), val.first, cid);
}

// _____________________________________________________________________________
template <typename W>
std::shared_ptr<W> GeometryCache::cache(TypedGeometryCache<W>& cache,
                                        size_t off, W&& val, size_t estSize,
                                        size_t cid) const {
  ExclusiveLock lock(cache.mutexes[cid]);

  // another thread may have inserted the value while we were reading it,
  // in that case re-use that
  auto it = cache.idMap[cid].find(off);
  if (it != cache.idMap[cid].end()) {
    it->second->second.touched.store(true, std::memory_order_relaxed);
    return it->second->second.val;
  }

  auto& vals = cache.vals[cid];

  // while the cache is too large or has too many elements, evict the last
  // element
  while (vals.size() > 0 &&
         ((cache.maxSize > 0 && cache.valSizes[cid] > cache.maxSize) ||
          (cache.maxNumElements > 0 &&
           vals.size() >= cache.maxNumElements))) {
    auto last = std::prev(vals.end());
    if (last->second.touched.load(std::memory_order_relaxed)) {
      last->second.touched.store(false, std::memory_order_relaxed);
      vals.splice(vals.begin(), vals, last);
      continue;
    }
    cache.valSizes[cid] -= last->second.estimatedSize;
    cache.idMap[cid].erase(last->first);
    vals.pop_back();
  }

  // push value to front
  vals.emplace_front(std::piecewise_construct, std::forward_as_tuple(off),
                     std::forward_as_tuple(
                         estSize, std::make_shared<W>(std::move(val))));

  // set map to front iterator
  cache.idMap[cid][off] = vals.begin();

  // increase total estimated size
  cache.valSizes[cid] += estSize;

  return vals.front().second.val;
}

// _____________________________________________________________________________
template <typename W>
std::pair<size_t, size_t> GeometryCache::size(
    const TypedGeometryCache<W>& cache) {
  size_t numEntries = 0;
  size_t bytes = 0;
  for (size_t tid = 0; tid < cache.vals.size(); tid++) {
    SharedLock lock(cache.mutexes[tid]);
    numEntries += cache.vals[tid].size();
    bytes += cache.valSizes[tid];
  }
  return {numEntries, bytes};
}

// _____________________________________________________________________________
std::pair<size_t, size_t> GeometryCache::size() const {
  std::pair<size_t, size_t> ret{0, 0};
  for (const auto& caches : _caches) {
    for (const auto& cacheSize :
         {size(caches->pointCache), size(caches->areaCache),
          size(caches->simpleAreaCache), size(caches->simpleLineCache),
          size(caches->lineCache), size(caches->areaIdCache),
          size(caches->lineIdCache)}) {
      ret.first += cacheSize.first;
      ret.second += cacheSize.second;
    }
  }
  return ret;
}

// _____________________________________________________________________________
uint8_t GeometryCache::addStore(const GeometryStore* store) {
  // store IDs are kept in 8 bits
  if (_stores.size() > std::numeric_limits<uint8_t>::max()) {
    throw std::runtime_error(
        "Cannot add more than " +
        std::to_string(std::numeric_limits<uint8_t>::max() + 1) +
        " stores");
  }

  _stores.push_back(store);
  _caches.push_back(std::unique_ptr<StoreCaches>(new StoreCaches(_cfg)));
  return _stores.size() - 1;
}

// _____________________________________________________________________________
void GeometryCache::log(const std::string& msg) {
  if (_cfg.logCb) _cfg.logCb(msg);
}

template std::shared_ptr<sj::GeomId> GeometryCache::getId(
    const sj::TypedGeometryStore<sj::Area>&,
    sj::TypedGeometryCache<sj::GeomId>&, size_t, size_t) const;
template std::shared_ptr<sj::GeomId> GeometryCache::getId(
    const sj::TypedGeometryStore<sj::Line>&,
    sj::TypedGeometryCache<sj::GeomId>&, size_t, size_t) const;

template std::shared_ptr<sj::Point> GeometryCache::get(
    const sj::TypedGeometryStore<sj::Point>&,
    sj::TypedGeometryCache<sj::Point>&, size_t, size_t) const;
template std::shared_ptr<sj::Area> GeometryCache::get(
    const sj::TypedGeometryStore<sj::Area>&, sj::TypedGeometryCache<sj::Area>&,
    size_t, size_t) const;
template std::shared_ptr<sj::SimpleArea> GeometryCache::get(
    const sj::TypedGeometryStore<sj::SimpleArea>&,
    sj::TypedGeometryCache<sj::SimpleArea>&, size_t, size_t) const;
template std::shared_ptr<sj::Line> GeometryCache::get(
    const sj::TypedGeometryStore<sj::Line>&, sj::TypedGeometryCache<sj::Line>&,
    size_t, size_t) const;
template std::shared_ptr<sj::SimpleLine> GeometryCache::get(
    const sj::TypedGeometryStore<sj::SimpleLine>&,
    sj::TypedGeometryCache<sj::SimpleLine>&, size_t, size_t) const;

