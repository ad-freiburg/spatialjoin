// Copyright 2023, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>

#include "GeometryCache.h"

#include <fstream>
#include <iostream>
#include <mutex>
#include <vector>

#include "util/geo/Geo.h"

const static size_t MAX_MEM_CACHE_SIZE = 1 * 1024 * 1024 * 20l;

// ____________________________________________________________________________
template <typename W>
std::shared_ptr<W> sj::GeometryCache<W>::get(size_t off, ssize_t desTid) const {
  size_t tid;

  if (_inMemory) {
    // completely circumvent cache system
    return std::make_shared<W>(_memStore.at(off));
  } else if (desTid == -1) {
    // special cache for large geometries
    tid = _numThreads;
  } else {
    tid = desTid % _numThreads;
  }

  std::unique_lock<std::mutex> lock(_mutexes[tid]);

  // check if value is in cache
  auto it = _idMap[tid].find(off);

  if (it == _idMap[tid].end()) {
    // if not, load, cache and return
    const auto& val = getFrom(off, _geomsFReads[tid]);
    return cache(off, val.second, val.first, tid);
  }

  // if in cache, move to front of list and return
  // splice only changes pointers in the linked list, no copying here
  _vals[tid].splice(_vals[tid].begin(), _vals[tid], it->second);

  return it->second->second.val;
}

// ____________________________________________________________________________
template <typename W>
std::shared_ptr<W> sj::GeometryCache<W>::cache(size_t off, const W& val, size_t estSize,
                                               size_t tid) const {
  // if cache is too large, pop last elements until we have space
  while (_vals[tid].size() > 0 && _valSizes[tid] > _maxSize) {
    auto last = _vals[tid].rbegin();
    _valSizes[tid] -= last->second.estimatedSize;
    _idMap[tid].erase(last->first);
    _vals[tid].pop_back();
  }

  // push value to front
  _vals[tid].push_front({off, {estSize, std::make_shared<W>(val)}});

  // set map to front iterator
  _idMap[tid][off] = _vals[tid].begin();

  // increase total estimated size
  _valSizes[tid] += estSize;

  return _vals[tid].front().second.val;
}

// ____________________________________________________________________________
template <>
std::pair<size_t, sj::SimpleLine> sj::GeometryCache<sj::SimpleLine>::getFrom(
    size_t off, std::istream& str) const {
  sj::SimpleLine ret;

  str.seekg(off);

  // geom
  str.read(reinterpret_cast<char*>(&ret.a), sizeof(util::geo::I32Point));
  str.read(reinterpret_cast<char*>(&ret.b), sizeof(util::geo::I32Point));

  // id
  uint16_t len;
  str.read(reinterpret_cast<char*>(&len), sizeof(uint16_t));
  ret.id.resize(len);

  if (len) {
    str.read(reinterpret_cast<char*>(&ret.id[0]), len * sizeof(char));
  }

  return {sizeof(util::geo::I32Point) * 2 + sizeof(char) * len, ret};
}

// ____________________________________________________________________________
template <>
std::pair<size_t, sj::Point> sj::GeometryCache<sj::Point>::getFrom(size_t off,
                                                std::istream& str) const {
  sj::Point ret;

  str.seekg(off);

  // id
  uint16_t len;
  str.read(reinterpret_cast<char*>(&len), sizeof(uint16_t));
  ret.id.resize(len);

  if (len) {
    str.read(reinterpret_cast<char*>(&ret.id[0]), len * sizeof(char));
  }

  // sub id
  str.read(reinterpret_cast<char*>(&ret.subId), sizeof(uint16_t));

  return {len* sizeof(char) + sizeof(uint16_t), ret};
}

// ____________________________________________________________________________
template <>
std::pair<size_t, sj::SimpleArea> sj::GeometryCache<sj::SimpleArea>::getFrom(
    size_t off, std::istream& str) const {
  sj::SimpleArea ret;

  str.seekg(off);

  // geom
  uint32_t size;
  str.read(reinterpret_cast<char*>(&size), sizeof(uint32_t));
  ret.geom.resize(size);

  if (size) {
    str.read(reinterpret_cast<char*>(&ret.geom[0]),
             size * sizeof(util::geo::I32Point));
  }

  // id
  uint16_t len;
  str.read(reinterpret_cast<char*>(&len), sizeof(uint16_t));
  ret.id.resize(len);

  if (len) {
    str.read(reinterpret_cast<char*>(&ret.id[0]), len * sizeof(char));
  }

  return {sizeof(uint32_t) + size * sizeof(util::geo::I32Point) + len * sizeof(char), ret};
}

// ____________________________________________________________________________
template <>
std::pair<size_t, sj::Line> sj::GeometryCache<sj::Line>::getFrom(size_t off,
                                              std::istream& str) const {
  sj::Line ret;
  size_t estSize = 0;

  str.seekg(off);

  // geom
  estSize += readLine(str, ret.geom);

  // envelope
  str.read(reinterpret_cast<char*>(&ret.box), sizeof(util::geo::I32Box));
  estSize += sizeof(util::geo::I32Box);

  // id
  uint16_t len;
  str.read(reinterpret_cast<char*>(&len), sizeof(uint16_t));
  ret.id.resize(len);

  if (len) {
    str.read(reinterpret_cast<char*>(&ret.id[0]), len * sizeof(char));
    estSize += len * sizeof(char);
  }

  // sub id
  str.read(reinterpret_cast<char*>(&ret.subId), sizeof(uint16_t));
    estSize += sizeof(uint16_t);

  // length
  str.read(reinterpret_cast<char*>(&ret.length), sizeof(double));
    estSize += sizeof(double);

  // boxIds
  uint32_t numBoxIds;
  str.read(reinterpret_cast<char*>(&numBoxIds), sizeof(uint32_t));
  ret.boxIds.resize(numBoxIds);
  if (numBoxIds > 0) {
    str.read(reinterpret_cast<char*>(&ret.boxIds[0]),
             sizeof(sj::boxids::BoxId) * numBoxIds);
    estSize += sizeof(sj::boxids::BoxId) * numBoxIds;
  }

  // OBB
  estSize += readPoly(str, ret.obb);

  return {estSize, ret};
}

// ____________________________________________________________________________
template <>
std::pair<size_t, sj::Area> sj::GeometryCache<sj::Area>::getFrom(size_t off,
                                              std::istream& str) const {
  sj::Area ret;
  size_t estSize = 0;

  str.seekg(off);

  // geom
  estSize += readPoly(str, ret.geom);

  // envelope
  str.read(reinterpret_cast<char*>(&ret.box), sizeof(util::geo::I32Box));
  estSize += sizeof(util::geo::I32Box);

  // id
  uint16_t len;
  str.read(reinterpret_cast<char*>(&len), sizeof(uint16_t));
  ret.id.resize(len);

  if (len) {
    str.read(reinterpret_cast<char*>(&ret.id[0]), len * sizeof(char));
    estSize += len * sizeof(char);
  }

  // sub id
  str.read(reinterpret_cast<char*>(&ret.subId), sizeof(uint16_t));
  estSize += sizeof(uint16_t);

  // area
  str.read(reinterpret_cast<char*>(&ret.area), sizeof(double));
  estSize += sizeof(double);

  // outer area
  str.read(reinterpret_cast<char*>(&ret.outerArea), sizeof(double));
  estSize += sizeof(double);

  // boxIds
  uint32_t numBoxIds;
  str.read(reinterpret_cast<char*>(&numBoxIds), sizeof(uint32_t));
  ret.boxIds.resize(numBoxIds);
  if (numBoxIds > 0) {
    str.read(reinterpret_cast<char*>(&ret.boxIds[0]),
             sizeof(sj::boxids::BoxId) * numBoxIds);
    estSize += sizeof(sj::boxids::BoxId) * numBoxIds;
  }

  // OBB
  estSize += readPoly(str, ret.obb);

  // simplified inner
  estSize += readPoly(str, ret.inner);

  if (!ret.inner.empty()) {
    str.read(reinterpret_cast<char*>(&ret.innerBox), sizeof(util::geo::I32Box));
    str.read(reinterpret_cast<char*>(&ret.innerOuterArea), sizeof(double));
    estSize += sizeof(double) + sizeof(util::geo::I32Box);
  }

  // simplified outer
  estSize += readPoly(str, ret.outer);

  if (!ret.outer.empty()) {
    str.read(reinterpret_cast<char*>(&ret.outerBox), sizeof(util::geo::I32Box));
    str.read(reinterpret_cast<char*>(&ret.outerOuterArea), sizeof(double));
    estSize += sizeof(double) + sizeof(util::geo::I32Box);
  }

  return {estSize, ret};
}

// ____________________________________________________________________________
template <>
size_t sj::GeometryCache<sj::Point>::writeTo(const sj::Point& val,
                                             std::ostream& str) {
  size_t ret = 0;

  // id
  uint16_t s = val.id.size();
  str.write(reinterpret_cast<const char*>(&s), sizeof(uint16_t));
  ret += sizeof(uint16_t);
  str.write(reinterpret_cast<const char*>(val.id.c_str()),
            val.id.size() * sizeof(char));
  ret += sizeof(char) * val.id.size();

  // sub id
  str.write(reinterpret_cast<const char*>(&val.subId), sizeof(uint16_t));
  ret += sizeof(uint16_t);

  return ret;
}

// ____________________________________________________________________________
template <typename W>
size_t sj::GeometryCache<W>::add(const std::string& raw) {
  size_t ret = _geomsOffset;
  _geomsOffset += raw.size();

  if (_inMemory) {
    // cache for later use
    std::istringstream ss(raw);
    const auto& val = getFrom(0, ss);
    _memStore[ret] = std::move(val.second);

    if (_geomsOffset > MAX_MEM_CACHE_SIZE) {
      _inMemory = false;

      for (const auto& val : _memStore) {
        writeTo(val.second, _geomsF);
      }

      // clear mem store
      _memStore = {};
    }

    return ret;
  }

  _geomsF.write(raw.c_str(), raw.size());

  return ret;
}

// ____________________________________________________________________________
template <>
size_t sj::GeometryCache<sj::SimpleArea>::writeTo(const sj::SimpleArea& val,
                                                  std::ostream& str) {
  size_t ret = 0;

  // geom
  uint32_t len = val.geom.size();
  str.write(reinterpret_cast<const char*>(&len), sizeof(uint32_t));
  ret += sizeof(uint32_t);

  if (len) {
    str.write(reinterpret_cast<const char*>(&val.geom[0]),
              len * sizeof(util::geo::I32Point));
    ret += sizeof(util::geo::I32Point) * len;
  }

  // id
  uint16_t s = val.id.size();
  str.write(reinterpret_cast<const char*>(&s), sizeof(uint16_t));
  ret += sizeof(uint16_t);
  str.write(reinterpret_cast<const char*>(val.id.c_str()),
            val.id.size() * sizeof(char));
  ret += sizeof(char) * val.id.size();

  return ret;
}

// ____________________________________________________________________________

template <>
size_t sj::GeometryCache<sj::SimpleLine>::writeTo(const sj::SimpleLine& val,
                                                  std::ostream& str) {
  size_t ret = 0;

  // geoms
  str.write(reinterpret_cast<const char*>(&val.a), sizeof(util::geo::I32Point));
  str.write(reinterpret_cast<const char*>(&val.b), sizeof(util::geo::I32Point));
  ret += sizeof(util::geo::I32Point) * 2;

  // id
  uint16_t s = val.id.size();
  str.write(reinterpret_cast<const char*>(&s), sizeof(uint16_t));
  ret += sizeof(uint16_t);
  str.write(reinterpret_cast<const char*>(val.id.c_str()),
            val.id.size() * sizeof(char));
  ret += sizeof(char) * val.id.size();

  return ret;
}

// ____________________________________________________________________________
template <>
size_t sj::GeometryCache<sj::Line>::writeTo(const sj::Line& val,
                                            std::ostream& str) {
  size_t ret = 0;

  // geoms
  ret += writeLine(val.geom, str);

  // envelopes
  str.write(reinterpret_cast<const char*>(&val.box), sizeof(util::geo::I32Box));
  ret += sizeof(util::geo::I32Box);

  // id
  uint16_t s = val.id.size();
  str.write(reinterpret_cast<const char*>(&s), sizeof(uint16_t));
  ret += sizeof(uint16_t);
  str.write(reinterpret_cast<const char*>(val.id.c_str()),
            val.id.size() * sizeof(char));
  ret += sizeof(char) * val.id.size();

  // sub id
  str.write(reinterpret_cast<const char*>(&val.subId), sizeof(uint16_t));
  ret += sizeof(uint16_t);

  // length
  str.write(reinterpret_cast<const char*>(&val.length), sizeof(double));
  ret += sizeof(double);

  // boxIds
  uint32_t size = val.boxIds.size();
  str.write(reinterpret_cast<const char*>(&size), sizeof(uint32_t));
  if (size > 0) {
    str.write(reinterpret_cast<const char*>(&val.boxIds[0]),
              sizeof(sj::boxids::BoxId) * size);
  }

  ret += sizeof(uint32_t) + sizeof(sj::boxids::BoxId) * size;

  // OBB
  ret += writePoly(val.obb, str);

  return ret;
}

// ____________________________________________________________________________
template <>
size_t sj::GeometryCache<sj::Area>::writeTo(const sj::Area& val,
                                            std::ostream& str) {
  size_t ret = 0;

  // geoms
  ret += writePoly(val.geom, str);

  // envelope
  str.write(reinterpret_cast<const char*>(&val.box), sizeof(util::geo::I32Box));
  ret += sizeof(util::geo::I32Box);

  // id
  uint16_t s = val.id.size();
  str.write(reinterpret_cast<const char*>(&s), sizeof(uint16_t));
  ret += sizeof(uint16_t);
  str.write(reinterpret_cast<const char*>(val.id.c_str()),
            val.id.size() * sizeof(char));
  ret += sizeof(char) * val.id.size();

  // sub id
  str.write(reinterpret_cast<const char*>(&val.subId), sizeof(uint16_t));
  ret += sizeof(uint16_t);

  // area
  str.write(reinterpret_cast<const char*>(&val.area), sizeof(double));
  ret += sizeof(double);

  // outer area
  str.write(reinterpret_cast<const char*>(&val.outerArea), sizeof(double));
  ret += sizeof(double);

  // boxIds
  uint32_t size = val.boxIds.size();
  str.write(reinterpret_cast<const char*>(&size), sizeof(uint32_t));
  if (size > 0) {
    str.write(reinterpret_cast<const char*>(&val.boxIds[0]),
              sizeof(sj::boxids::BoxId) * size);
  }

  ret += sizeof(uint32_t) + sizeof(sj::boxids::BoxId) * size;

  // OBB
  ret += writePoly(val.obb, str);

  // innerGeom
  ret += writePoly(val.inner, str);

  if (!val.inner.empty()) {
    str.write(reinterpret_cast<const char*>(&val.innerBox),
              sizeof(util::geo::I32Box));
    ret += sizeof(util::geo::I32Box);

    // inner area
    str.write(reinterpret_cast<const char*>(&val.innerOuterArea),
              sizeof(double));
    ret += sizeof(double);
  }

  // outerGeom
  ret += writePoly(val.outer, str);

  if (!val.outer.empty()) {
    str.write(reinterpret_cast<const char*>(&val.outerBox),
              sizeof(util::geo::I32Box));
    ret += sizeof(util::geo::I32Box);

    // outer area
    str.write(reinterpret_cast<const char*>(&val.outerOuterArea),
              sizeof(double));
    ret += sizeof(double);
  }

  return ret;
}

// ____________________________________________________________________________
template <typename W>
void sj::GeometryCache<W>::flush() {
  if (_geomsF.is_open()) {
    _geomsF.flush();
    _geomsF.close();
  }
}

// ____________________________________________________________________________
template <typename W>
size_t sj::GeometryCache<W>::readPoly(std::istream& str,
                                    util::geo::I32XSortedPolygon& ret) const {
  size_t estSize = 0;
  double maxSegLen;
  str.read(reinterpret_cast<char*>(&maxSegLen), sizeof(double));
  estSize += sizeof(double);
  ret.getOuter().setMaxSegLen(maxSegLen);

  uint32_t sizeOuter;
  str.read(reinterpret_cast<char*>(&sizeOuter), sizeof(uint32_t));

  // TODO: careful, resize initializes entire vector!
  if (sizeOuter) {
    ret.getOuter().rawRing().resize(sizeOuter);

    str.read(reinterpret_cast<char*>(&ret.getOuter().rawRing()[0]),
             sizeof(util::geo::XSortedTuple<int32_t>) * sizeOuter);
    estSize += sizeof(util::geo::XSortedTuple<int32_t>) * sizeOuter;
  }

  uint32_t numInners;
  str.read(reinterpret_cast<char*>(&numInners), sizeof(uint32_t));
    estSize += sizeof(uint32_t) ;

  double innerMaxSegLen;
  str.read(reinterpret_cast<char*>(&innerMaxSegLen), sizeof(double));
    estSize += sizeof(double) ;

  ret.setInnerMaxSegLen(innerMaxSegLen);

  // TODO: careful, resize initializes entire vector!
  ret.getInners().resize(numInners);
  ret.getInnerBoxes().resize(numInners);
  ret.getInnerBoxIdx().resize(numInners);
  ret.getInnerAreas().resize(numInners);

  if (numInners) {
    str.read(reinterpret_cast<char*>(&ret.getInnerBoxes()[0]),
             sizeof(util::geo::Box<int32_t>) * numInners);
    estSize += sizeof(util::geo::Box<int32_t>) * numInners;
    str.read(reinterpret_cast<char*>(&ret.getInnerBoxIdx()[0]),
             sizeof(std::pair<int32_t, size_t>) * numInners);
    estSize += sizeof(std::pair<int32_t, size_t>) * numInners;
    str.read(reinterpret_cast<char*>(&ret.getInnerAreas()[0]),
             sizeof(double) * numInners);
    estSize += sizeof(double) * numInners;
  }

  for (uint32_t j = 0; j < numInners; j++) {
    double maxSegLen;
    str.read(reinterpret_cast<char*>(&maxSegLen), sizeof(double));
    ret.getInners()[j].setMaxSegLen(maxSegLen);
    estSize += sizeof(double);

    uint32_t sizeInner;
    str.read(reinterpret_cast<char*>(&sizeInner), sizeof(uint32_t));
    estSize += sizeof(uint32_t);

    // TODO: careful, resize initializes entire vector!
    ret.getInners()[j].rawRing().resize(sizeInner);

    if (sizeInner) {
      str.read(reinterpret_cast<char*>(&ret.getInners()[j].rawRing()[0]),
               sizeof(util::geo::XSortedTuple<int32_t>) * sizeInner);
      estSize += sizeof(util::geo::XSortedTuple<int32_t>) * sizeInner;
    }
  }

  return estSize;
}

// ____________________________________________________________________________
template <typename W>
size_t sj::GeometryCache<W>::writePoly(const util::geo::I32XSortedPolygon& geom,
                                       std::ostream& str) {
  size_t ret = 0;

  // geom, outer
  double maxSegLen = geom.getOuter().getMaxSegLen();
  str.write(reinterpret_cast<const char*>(&maxSegLen), sizeof(double));
  ret += sizeof(double);

  uint32_t locSize = geom.getOuter().rawRing().size();
  str.write(reinterpret_cast<const char*>(&locSize), sizeof(uint32_t));
  if (locSize) {
    str.write(reinterpret_cast<const char*>(&geom.getOuter().rawRing()[0]),
              sizeof(util::geo::XSortedTuple<int32_t>) * locSize);
  }
  ret += sizeof(uint32_t) + sizeof(util::geo::XSortedTuple<int32_t>) * locSize;

  // geom, inners
  locSize = geom.getInners().size();
  str.write(reinterpret_cast<const char*>(&locSize), sizeof(uint32_t));
  ret += sizeof(uint32_t);

  // inner max box seg len
  double innerMaxSegLen = geom.getInnerMaxSegLen();
  str.write(reinterpret_cast<const char*>(&innerMaxSegLen), sizeof(double));
  ret += sizeof(double);

  // inner boxes
  if (locSize) {
    str.write(reinterpret_cast<const char*>(&geom.getInnerBoxes()[0]),
              sizeof(util::geo::Box<int32_t>) * locSize);
    ret += sizeof(util::geo::Box<int32_t>) * locSize;

    // inner box idx
    str.write(reinterpret_cast<const char*>(&geom.getInnerBoxIdx()[0]),
              sizeof(std::pair<int32_t, size_t>) * locSize);
    ret += sizeof(std::pair<int32_t, size_t>) * locSize;

    // inner areas
    str.write(reinterpret_cast<const char*>(&geom.getInnerAreas()[0]),
              sizeof(double) * locSize);
    ret += sizeof(double) * locSize;
  }

  for (const auto& inner : geom.getInners()) {
    locSize = inner.rawRing().size();
    double maxSegLen = inner.getMaxSegLen();
    str.write(reinterpret_cast<const char*>(&maxSegLen), sizeof(double));
    ret += sizeof(double);

    str.write(reinterpret_cast<const char*>(&locSize), sizeof(uint32_t));
    if (locSize) {
      str.write(reinterpret_cast<const char*>(&inner.rawRing()[0]),
                sizeof(util::geo::XSortedTuple<int32_t>) * locSize);
    }

    ret +=
        sizeof(uint32_t) + sizeof(util::geo::XSortedTuple<int32_t>) * locSize;
  }

  return ret;
}

// ____________________________________________________________________________
template <typename W>
size_t sj::GeometryCache<W>::readLine(std::istream& str,
                                    util::geo::I32XSortedLine& ret) const {
  size_t estSize = 0;
  double maxSegLen;
  str.read(reinterpret_cast<char*>(&maxSegLen), sizeof(double));
  estSize += sizeof(double);
  ret.setMaxSegLen(maxSegLen);

  util::geo::I32Point firstPoint;
  util::geo::I32Point lastPoint;
  str.read(reinterpret_cast<char*>(&firstPoint), sizeof(uint32_t) * 2);
  estSize += sizeof(uint32_t) * 2;
  str.read(reinterpret_cast<char*>(&lastPoint), sizeof(uint32_t) * 2);
  estSize += sizeof(uint32_t) * 2;
  ret.setFirstPoint(firstPoint);
  ret.setLastPoint(lastPoint);

  uint32_t sizeOuter;
  str.read(reinterpret_cast<char*>(&sizeOuter), sizeof(uint32_t));
  estSize += sizeof(uint32_t);

  // TODO: careful, resize initializes entire vector!
  ret.rawLine().resize(sizeOuter);

  if (sizeOuter) {
    str.read(reinterpret_cast<char*>(&ret.rawLine()[0]),
             sizeof(util::geo::XSortedTuple<int32_t>) * sizeOuter);
    estSize += sizeof(util::geo::XSortedTuple<int32_t>) * sizeOuter;
  }

  return estSize;
}

// ____________________________________________________________________________
template <typename W>
size_t sj::GeometryCache<W>::writeLine(const util::geo::I32XSortedLine& geom,
                                       std::ostream& str) {
  size_t ret = 0;

  double maxSegLen = geom.getMaxSegLen();
  str.write(reinterpret_cast<const char*>(&maxSegLen), sizeof(double));
  ret += sizeof(double);

  auto firstPoint = geom.firstPoint();
  auto lastPoint = geom.lastPoint();
  str.write(reinterpret_cast<const char*>(&firstPoint), sizeof(uint32_t) * 2);
  str.write(reinterpret_cast<const char*>(&lastPoint), sizeof(uint32_t) * 2);

  uint32_t locSize = geom.rawLine().size();
  str.write(reinterpret_cast<const char*>(&locSize), sizeof(uint32_t));
  if (locSize)
    str.write(reinterpret_cast<const char*>(&geom.rawLine()[0]),
              sizeof(util::geo::XSortedTuple<int32_t>) * locSize);
  ret += sizeof(uint32_t) + sizeof(util::geo::XSortedTuple<int32_t>) * locSize;

  return ret;
}

// ____________________________________________________________________________
template <typename W>
std::pair<size_t, size_t> sj::GeometryCache<W>::size() const {
  size_t numEntries = 0;
  size_t bytes = 0;
  for (size_t tid = 0; tid < _vals.size(); tid++) {
    std::unique_lock<std::mutex> lock(_mutexes[tid]);
    numEntries += _vals[tid].size();
    bytes += _valSizes[tid];
  }
  return {numEntries, bytes};
}

// ____________________________________________________________________________
template <>
std::string sj::GeometryCache<sj::Area>::getFName() const {
  return util::getTmpFName(_dir, _tmpPrefix, "areas");
}

// ____________________________________________________________________________
template <>
std::string sj::GeometryCache<sj::Line>::getFName() const {
  return util::getTmpFName(_dir, _tmpPrefix, "lines");
}

// ____________________________________________________________________________
template <>
std::string sj::GeometryCache<sj::Point>::getFName() const {
  return util::getTmpFName(_dir, _tmpPrefix, "points");
}

// ____________________________________________________________________________
template <>
std::string sj::GeometryCache<sj::SimpleLine>::getFName() const {
  return util::getTmpFName(_dir, _tmpPrefix, "simplelines");
}

// ____________________________________________________________________________
template <>
std::string sj::GeometryCache<sj::SimpleArea>::getFName() const {
  return util::getTmpFName(_dir, ".spatialjoin", "simpleareas");
}

// ____________________________________________________________________________
template class sj::GeometryCache<sj::Area>;
template class sj::GeometryCache<sj::Line>;
template class sj::GeometryCache<sj::SimpleLine>;
template class sj::GeometryCache<sj::Point>;
template class sj::GeometryCache<sj::SimpleArea>;
