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
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <set>
#include <sstream>

#include "BoxIds.h"
#include "GeometryCacheManager.h"
#include "util/Misc.h"
#include "util/geo/IntervalIdx.h"
#include "util/log/Log.h"

using sj::GeomCheckRes;
using sj::GeometryCacheManager;
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
std::shared_ptr<W> GeometryCacheManager::get(const TypedGeometryStore<W>& store,
                                             GeomLRUCache<W>& cache, size_t off,
                                             size_t tid) const {
  // completely circumvent cache system
  if (store.inMemory()) return store.getFromMem(off);

  tid = tid % _cfg.numCacheThreads;

  std::unique_lock<std::mutex> lock(cache.mutexes[tid]);

  // check if value is in cache
  auto it = cache.idMap[tid].find(off);

  if (it == cache.idMap[tid].end()) {
    // if not, load, cache and return
    const auto& val = store.getFrom(off, tid);
    return this->cache(cache, off, val.second, val.first, tid);
  }

  // if in cache, move to front of list and return
  // splice only changes pointers in the linked list, no copying here
  cache.vals[tid].splice(cache.vals[tid].begin(), cache.vals[tid], it->second);

  return it->second->second.val;
}

template std::shared_ptr<sj::Point> GeometryCacheManager::get(
    const sj::TypedGeometryStore<sj::Point>&, sj::GeomLRUCache<sj::Point>&, size_t,
    size_t) const;
template std::shared_ptr<sj::Area> GeometryCacheManager::get(
    const sj::TypedGeometryStore<sj::Area>&, sj::GeomLRUCache<sj::Area>&, size_t,
    size_t) const;
template std::shared_ptr<sj::SimpleArea> GeometryCacheManager::get(
    const sj::TypedGeometryStore<sj::SimpleArea>&, sj::GeomLRUCache<sj::SimpleArea>&,
    size_t, size_t) const;
template std::shared_ptr<sj::Line> GeometryCacheManager::get(
    const sj::TypedGeometryStore<sj::Line>&, sj::GeomLRUCache<sj::Line>&, size_t,
    size_t) const;
template std::shared_ptr<sj::SimpleLine> GeometryCacheManager::get(
    const sj::TypedGeometryStore<sj::SimpleLine>&, sj::GeomLRUCache<sj::SimpleLine>&,
    size_t, size_t) const;

// _____________________________________________________________________________
template <typename W>
std::shared_ptr<W> GeometryCacheManager::cache(GeomLRUCache<W>& cache,
                                               size_t off, const W& val,
                                               size_t estSize,
                                               size_t tid) const {
  // if cache is too large, pop last elements until we have space
  while (cache.maxSize > 0 && cache.vals[tid].size() > 0 &&
         cache.valSizes[tid] > cache.maxSize) {
    auto last = cache.vals[tid].rbegin();
    cache.valSizes[tid] -= last->second.estimatedSize;
    cache.idMap[tid].erase(last->first);
    cache.vals[tid].pop_back();
  }

  // push value to front
  cache.vals[tid].push_front({off, {estSize, std::make_shared<W>(val)}});

  // if cache has too many elements, pop last element
  if (cache.maxNumElements > 0 &&
      cache.vals[tid].size() > cache.maxNumElements) {
    auto last = cache.vals[tid].rbegin();
    cache.valSizes[tid] -= last->second.estimatedSize;
    cache.idMap[tid].erase(last->first);
    cache.vals[tid].pop_back();
  }

  // set map to front iterator
  cache.idMap[tid][off] = cache.vals[tid].begin();

  // increase total estimated size
  cache.valSizes[tid] += estSize;

  return cache.vals[tid].front().second.val;
}

// _____________________________________________________________________________
template <typename W>
std::pair<size_t, size_t> GeometryCacheManager::size(
    const GeomLRUCache<W>& cache) {
  size_t numEntries = 0;
  size_t bytes = 0;
  for (size_t tid = 0; tid < cache.vals.size(); tid++) {
    std::unique_lock<std::mutex> lock(cache.mutexes[tid]);
    numEntries += cache.vals[tid].size();
    bytes += cache.valSizes[tid];
  }
  return {numEntries, bytes};
}

// _____________________________________________________________________________
std::pair<size_t, size_t> GeometryCacheManager::size() const {
  auto cacheSizePoint = size(_pointCache);
  auto cacheSizeArea = size(_areaCache);
  auto cacheSizeSimpleArea = size(_simpleAreaCache);
  auto cacheSizeSimpleLine = size(_simpleLineCache);
  auto cacheSizeLine = size(_lineCache);

  return {cacheSizePoint.first + cacheSizeArea.first +
              cacheSizeSimpleArea.first + cacheSizeSimpleLine.first +
              cacheSizeLine.first,
          cacheSizePoint.second + cacheSizeArea.second +
              cacheSizeSimpleArea.second + cacheSizeSimpleLine.second +
              cacheSizeLine.second};
}

// _____________________________________________________________________________
void GeometryCacheManager::log(const std::string& msg) {
  if (_cfg.logCb) _cfg.logCb(msg);
}
