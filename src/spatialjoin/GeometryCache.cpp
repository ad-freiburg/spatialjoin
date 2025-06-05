// Copyright 2023, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>

#include "GeometryCache.h"

#include <fstream>
#include <iostream>
#include <mutex>
#include <vector>

#include "util/geo/Geo.h"

const static size_t MAX_MEM_CACHE_SIZE = 1 * 1024 * 1024 * 1024l;

// ____________________________________________________________________________
template <typename W>
std::shared_ptr<W> sj::GeometryCache<W>::get(size_t off, ssize_t desTid) const {
  size_t tid;

  if (_inMemory) {
    // completely circumvent cache system
    return _memStore.at(off);
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
    auto val = getFrom(off, _geomsFReads[tid], _GEOScontextHandles[tid]);
    return cache(off, val, tid);
  }

  // if in cache, move to front of list and return
  // splice only changes pointers in the linked list, no copying here
  _vals[tid].splice(_vals[tid].begin(), _vals[tid], it->second);

  return it->second->second;
}

// ____________________________________________________________________________
template <typename W>
std::shared_ptr<W> sj::GeometryCache<W>::cache(size_t off, W& val,
                                               size_t tid) const {
  // push value to front
  _vals[tid].push_front({off, std::make_shared<W>(std::move(val))});

  // set map to front iterator
  _idMap[tid][off] = _vals[tid].begin();

  // if cache is too large, pop last element
  if (_vals[tid].size() > _maxSize) {
    auto last = _vals[tid].rbegin();
    _idMap[tid].erase(last->first);
    _vals[tid].pop_back();
  }

  return _vals[tid].front().second;
}

// ____________________________________________________________________________
template <>
sj::SimpleLine sj::GeometryCache<sj::SimpleLine>::getFrom(
    size_t off, std::istream& str, GEOSContextHandle_t) const {
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

  return ret;
}

// ____________________________________________________________________________
template <>
sj::Point sj::GeometryCache<sj::Point>::getFrom(size_t off, std::istream& str,
                                                GEOSContextHandle_t) const {
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

  return ret;
}

// ____________________________________________________________________________
template <>
sj::SimpleArea sj::GeometryCache<sj::SimpleArea>::getFrom(
    size_t off, std::istream& str, GEOSContextHandle_t) const {
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

  return ret;
}

// ____________________________________________________________________________
template <>
sj::Line sj::GeometryCache<sj::Line>::getFrom(
    size_t off, std::istream& str, GEOSContextHandle_t geosHndl) const {
  sj::Line ret;

  str.seekg(off);

  // geom
  readLine(str, ret.geom);

  // optional GEOS geom
  readGEOSLine(str, ret.geosGeom, geosHndl);

  // envelope
  str.read(reinterpret_cast<char*>(&ret.box), sizeof(util::geo::I32Box));

  // id
  uint16_t len;
  str.read(reinterpret_cast<char*>(&len), sizeof(uint16_t));
  ret.id.resize(len);

  if (len) {
    str.read(reinterpret_cast<char*>(&ret.id[0]), len * sizeof(char));
  }

  // sub id
  str.read(reinterpret_cast<char*>(&ret.subId), sizeof(uint16_t));

  // length
  str.read(reinterpret_cast<char*>(&ret.length), sizeof(double));

  // boxIds
  uint32_t numBoxIds;
  str.read(reinterpret_cast<char*>(&numBoxIds), sizeof(uint32_t));
  ret.boxIds.resize(numBoxIds);
  if (numBoxIds > 0) {
    str.read(reinterpret_cast<char*>(&ret.boxIds[0]),
             sizeof(sj::boxids::BoxId) * numBoxIds);
  }

  // OBB
  readPoly(str, ret.obb);

  return ret;
}

// ____________________________________________________________________________
template <>
sj::Area sj::GeometryCache<sj::Area>::getFrom(
    size_t off, std::istream& str, GEOSContextHandle_t geosHndl) const {
  sj::Area ret;

  str.seekg(off);

  // geom
  readPoly(str, ret.geom);

  // optional GEOS geom
  readGEOSPoly(str, ret.geosGeom, geosHndl);

  // envelope
  str.read(reinterpret_cast<char*>(&ret.box), sizeof(util::geo::I32Box));

  // id
  uint16_t len;
  str.read(reinterpret_cast<char*>(&len), sizeof(uint16_t));
  ret.id.resize(len);

  if (len) {
    str.read(reinterpret_cast<char*>(&ret.id[0]), len * sizeof(char));
  }

  // sub id
  str.read(reinterpret_cast<char*>(&ret.subId), sizeof(uint16_t));

  // area
  str.read(reinterpret_cast<char*>(&ret.area), sizeof(double));

  // outer area
  str.read(reinterpret_cast<char*>(&ret.outerArea), sizeof(double));

  // boxIds
  uint32_t numBoxIds;
  str.read(reinterpret_cast<char*>(&numBoxIds), sizeof(uint32_t));
  ret.boxIds.resize(numBoxIds);
  if (numBoxIds > 0) {
    str.read(reinterpret_cast<char*>(&ret.boxIds[0]),
             sizeof(sj::boxids::BoxId) * numBoxIds);
  }

  // OBB
  readPoly(str, ret.obb);

  // simplified inner
  readPoly(str, ret.inner);

  // optional simplified inner GEOS geom
  readGEOSPoly(str, ret.innerGeosGeom, geosHndl);

  if (!ret.inner.empty()) {
    str.read(reinterpret_cast<char*>(&ret.innerBox), sizeof(util::geo::I32Box));
    str.read(reinterpret_cast<char*>(&ret.innerOuterArea), sizeof(double));
  }

  // simplified outer
  readPoly(str, ret.outer);

  // optional simplified inner GEOS geom
  readGEOSPoly(str, ret.outerGeosGeom, geosHndl);

  if (!ret.outer.empty()) {
    str.read(reinterpret_cast<char*>(&ret.outerBox), sizeof(util::geo::I32Box));
    str.read(reinterpret_cast<char*>(&ret.outerOuterArea), sizeof(double));
  }

  return ret;
}

// ____________________________________________________________________________
template <>
size_t sj::GeometryCache<sj::Point>::writeTo(const sj::Point& val,
                                             std::ostream& str,
                                             GEOSContextHandle_t) {
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
    _memStore[ret] =
        std::make_shared<W>(std::move(getFrom(0, ss, _GEOScontextHandles[0])));

    if (_geomsOffset > MAX_MEM_CACHE_SIZE) {
      _inMemory = false;

      for (const auto& val : _memStore) {
        writeTo(*val.second, _geomsF, _GEOScontextHandles[0]);
      }

      // clear mem store
      _memStore.clear();
    }

    return ret;
  }

  _geomsF.write(raw.c_str(), raw.size());

  return ret;
}

// ____________________________________________________________________________
template <>
size_t sj::GeometryCache<sj::SimpleArea>::writeTo(const sj::SimpleArea& val,
                                                  std::ostream& str,
                                                  GEOSContextHandle_t) {
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
                                                  std::ostream& str,
                                                  GEOSContextHandle_t) {
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
                                            std::ostream& str,
                                            GEOSContextHandle_t geosHndl) {
  size_t ret = 0;

  // geoms
  ret += writeLine(val.geom, str);

  // libgeosline
  ret += writeGEOSLine(val.geosGeom, str, geosHndl);

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
                                            std::ostream& str,
                                            GEOSContextHandle_t geosHndl) {
  size_t ret = 0;

  // geoms
  ret += writePoly(val.geom, str);

  // libgeospoly
  ret += writeGEOSPoly(val.geosGeom, str, geosHndl);

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

  // libgeospoly
  ret += writeGEOSPoly(val.innerGeosGeom, str, geosHndl);

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

  ret += writeGEOSPoly(val.outerGeosGeom, str, geosHndl);

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
void sj::GeometryCache<W>::readPoly(std::istream& str,
                                    util::geo::I32XSortedPolygon& ret) const {
  double maxSegLen;
  str.read(reinterpret_cast<char*>(&maxSegLen), sizeof(double));
  ret.getOuter().setMaxSegLen(maxSegLen);

  uint32_t sizeOuter;
  str.read(reinterpret_cast<char*>(&sizeOuter), sizeof(uint32_t));

  // TODO: careful, resize initializes entire vector!
  if (sizeOuter) {
    ret.getOuter().rawRing().resize(sizeOuter);

    str.read(reinterpret_cast<char*>(&ret.getOuter().rawRing()[0]),
             sizeof(util::geo::XSortedTuple<int32_t>) * sizeOuter);
  }

  uint32_t numInners;
  str.read(reinterpret_cast<char*>(&numInners), sizeof(uint32_t));

  double innerMaxSegLen;
  str.read(reinterpret_cast<char*>(&innerMaxSegLen), sizeof(double));

  ret.setInnerMaxSegLen(innerMaxSegLen);

  // TODO: careful, resize initializes entire vector!
  ret.getInners().resize(numInners);
  ret.getInnerBoxes().resize(numInners);
  ret.getInnerBoxIdx().resize(numInners);
  ret.getInnerAreas().resize(numInners);

  if (numInners) {
    str.read(reinterpret_cast<char*>(&ret.getInnerBoxes()[0]),
             sizeof(util::geo::Box<int32_t>) * numInners);
    str.read(reinterpret_cast<char*>(&ret.getInnerBoxIdx()[0]),
             sizeof(std::pair<int32_t, size_t>) * numInners);
    str.read(reinterpret_cast<char*>(&ret.getInnerAreas()[0]),
             sizeof(double) * numInners);
  }

  for (uint32_t j = 0; j < numInners; j++) {
    double maxSegLen;
    str.read(reinterpret_cast<char*>(&maxSegLen), sizeof(double));
    ret.getInners()[j].setMaxSegLen(maxSegLen);

    uint32_t sizeInner;
    str.read(reinterpret_cast<char*>(&sizeInner), sizeof(uint32_t));

    // TODO: careful, resize initializes entire vector!
    ret.getInners()[j].rawRing().resize(sizeInner);

    if (sizeInner) {
      str.read(reinterpret_cast<char*>(&ret.getInners()[j].rawRing()[0]),
               sizeof(util::geo::XSortedTuple<int32_t>) * sizeInner);
    }
  }
}

// ____________________________________________________________________________
template <typename W>
void sj::GeometryCache<W>::readGEOSPoly(std::istream& str, GEOSPolygon& ret,
                                        GEOSContextHandle_t geosHndl) const {
  uint32_t outerSize;
  str.read(reinterpret_cast<char*>(&outerSize), sizeof(uint32_t));

  if (outerSize) {
    double* buffer = new double[outerSize * 2];

    str.read(reinterpret_cast<char*>(buffer), sizeof(double) * outerSize * 2);

    auto seq = GEOSCoordSeq_copyFromBuffer_r(geosHndl, buffer, outerSize, 0, 0);
    delete[] buffer;

    uint32_t numInners;
    str.read(reinterpret_cast<char*>(&numInners), sizeof(uint32_t));
    GEOSGeometry** innerRings = new GEOSGeometry*[numInners];

    if (numInners) {
      for (uint32_t j = 0; j < numInners; j++) {
        uint32_t innerSize;
        str.read(reinterpret_cast<char*>(&innerSize), sizeof(uint32_t));

        double* buffer = new double[innerSize * 2];

        str.read(reinterpret_cast<char*>(buffer),
                 sizeof(double) * innerSize * 2);
        auto seq =
            GEOSCoordSeq_copyFromBuffer_r(geosHndl, buffer, innerSize, 0, 0);

        innerRings[j] = GEOSGeom_createLinearRing_r(geosHndl, seq);

        delete[] buffer;
      }
    }

    ret = std::move(GEOSPolygon(
        GEOSGeom_createPolygon_r(geosHndl,
                                 GEOSGeom_createLinearRing_r(geosHndl, seq),
                                 innerRings, numInners),
        geosHndl));
    delete[] innerRings;
  } else {
    ret = std::move(
        GEOSPolygon(GEOSGeom_createEmptyPolygon_r(geosHndl), geosHndl));
  }
}

// ____________________________________________________________________________
template <typename W>
size_t sj::GeometryCache<W>::writeGEOSLine(const GEOSLineString& geom,
                                           std::ostream& str,
                                           GEOSContextHandle_t geosHndl) {
  size_t ret = 0;

  // dummy value of 0
  if (geom.getGEOSGeom() == 0) {
    uint32_t i = 0;
    str.write(reinterpret_cast<const char*>(&i), sizeof(uint32_t));
    ret += sizeof(uint32_t);
    return ret;
  }

  // geom, outer
  auto* coordSeq = GEOSGeom_getCoordSeq_r(geosHndl, geom.getGEOSGeom());
  uint32_t locSize = GEOSGeomGetNumPoints_r(geosHndl, geom.getGEOSGeom());

  double* buffer = new double[locSize * 2];
  GEOSCoordSeq_copyToBuffer_r(geosHndl, coordSeq, buffer, 0, 0);

  str.write(reinterpret_cast<const char*>(&locSize), sizeof(uint32_t));
  if (locSize) {
    str.write(reinterpret_cast<const char*>(buffer),
              sizeof(double) * locSize * 2);
  }

  delete[] buffer;

  ret += sizeof(uint32_t) + sizeof(double) * locSize * 2;

  return ret;
}

// ____________________________________________________________________________
template <typename W>
size_t sj::GeometryCache<W>::writeGEOSPoly(const GEOSPolygon& geom,
                                           std::ostream& str,
                                           GEOSContextHandle_t geosHndl) {
  size_t ret = 0;

  // dummy value of 0
  if (geom.getGEOSGeom() == 0 || GEOSisEmpty_r(geosHndl, geom.getGEOSGeom())) {
    uint32_t i = 0;
    str.write(reinterpret_cast<const char*>(&i), sizeof(uint32_t));
    return sizeof(uint32_t);
  }

  // geom, outer
  auto* outerRing = GEOSGetExteriorRing_r(geosHndl, geom.getGEOSGeom());
  auto* coordSeq = GEOSGeom_getCoordSeq_r(geosHndl, outerRing);
  uint32_t locSize = GEOSGeomGetNumPoints_r(geosHndl, outerRing);

  double* buffer = new double[locSize * 2];
  GEOSCoordSeq_copyToBuffer_r(geosHndl, coordSeq, buffer, 0, 0);

  str.write(reinterpret_cast<const char*>(&locSize), sizeof(uint32_t));
  if (locSize) {
    str.write(reinterpret_cast<const char*>(buffer),
              sizeof(double) * locSize * 2);
  }

  delete[] buffer;

  ret += sizeof(uint32_t) + sizeof(double) * locSize * 2;

  // geom, inners
  uint32_t numInners = GEOSGetNumInteriorRings_r(geosHndl,
  geom.getGEOSGeom());
  str.write(reinterpret_cast<const char*>(&numInners), sizeof(uint32_t));
  ret += sizeof(uint32_t);

  for (size_t i = 0; i < numInners; i++) {
    auto* innerRing = GEOSGetInteriorRingN_r(geosHndl, geom.getGEOSGeom(), i);
    auto* coordSeq = GEOSGeom_getCoordSeq_r(geosHndl, innerRing);
    uint32_t locSize = GEOSGeomGetNumPoints_r(geosHndl, innerRing);

    str.write(reinterpret_cast<const char*>(&locSize), sizeof(uint32_t));

    double* buffer = new double[locSize * 2];
    GEOSCoordSeq_copyToBuffer_r(geosHndl, coordSeq, buffer, 0, 0);

    if (locSize) {
      str.write(reinterpret_cast<const char*>(buffer),
                sizeof(double) * locSize * 2);
    }

    ret += sizeof(uint32_t) + sizeof(double) * locSize * 2;
    delete[] buffer;
  }

  return ret;
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
void sj::GeometryCache<W>::readLine(std::istream& str,
                                    util::geo::I32XSortedLine& ret) const {
  double maxSegLen;
  str.read(reinterpret_cast<char*>(&maxSegLen), sizeof(double));
  ret.setMaxSegLen(maxSegLen);

  util::geo::I32Point firstPoint;
  util::geo::I32Point lastPoint;
  str.read(reinterpret_cast<char*>(&firstPoint), sizeof(uint32_t) * 2);
  str.read(reinterpret_cast<char*>(&lastPoint), sizeof(uint32_t) * 2);
  ret.setFirstPoint(firstPoint);
  ret.setLastPoint(lastPoint);

  uint32_t sizeOuter;
  str.read(reinterpret_cast<char*>(&sizeOuter), sizeof(uint32_t));

  // TODO: careful, resize initializes entire vector!
  ret.rawLine().resize(sizeOuter);

  if (sizeOuter) {
    str.read(reinterpret_cast<char*>(&ret.rawLine()[0]),
             sizeof(util::geo::XSortedTuple<int32_t>) * sizeOuter);
  }
}

// ____________________________________________________________________________
template <typename W>
void sj::GeometryCache<W>::readGEOSLine(std::istream& str, GEOSLineString& ret,
                                        GEOSContextHandle_t geosHndl) const {
  uint32_t size;
  str.read(reinterpret_cast<char*>(&size), sizeof(uint32_t));

  if (size) {
    double* buffer = new double[size * 2];

    str.read(reinterpret_cast<char*>(buffer), sizeof(double) * size * 2);

    auto seq = GEOSCoordSeq_copyFromBuffer_r(geosHndl, buffer, size, 0, 0);
    delete[] buffer;

    ret = std::move(
        GEOSLineString(GEOSGeom_createLineString_r(geosHndl, seq), geosHndl));
  } else {
    ret = std::move(
        GEOSLineString(GEOSGeom_createEmptyLineString_r(geosHndl), geosHndl));
  }
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
