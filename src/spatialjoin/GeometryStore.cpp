// Copyright 2023, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>

#include <errno.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_set>
#include <vector>

#include "BoxIds.h"
#include "GeometryStore.h"
#include "util/Misc.h"
#include "util/geo/Geo.h"

const static size_t MAX_MEM_CACHE_SIZE = 3 * 1024 * 1024 * 20l;

// ____________________________________________________________________________
template <>
std::pair<size_t, sj::SimpleLine> sj::TypedGeometryStore<sj::SimpleLine>::getFrom(
    size_t off, std::istream& str) const {
  sj::SimpleLine ret;

  str.seekg(off);

  // id
  uint16_t len;
  str.read(reinterpret_cast<char*>(&len), sizeof(uint16_t));
  ret.id.resize(len);

  if (len) {
    str.read(reinterpret_cast<char*>(&ret.id[0]), len * sizeof(char));
  }

  return {sizeof(char) * len, ret};
}

// ____________________________________________________________________________
template <>
std::pair<size_t, sj::MultiGeom> sj::TypedGeometryStore<sj::MultiGeom>::getFrom(
    size_t off, std::istream& str) const {
  sj::MultiGeom ret;

  str.seekg(off);

  // id
  uint16_t len;
  str.read(reinterpret_cast<char*>(&len), sizeof(uint16_t));
  ret.id.resize(len);

  if (len) {
    str.read(reinterpret_cast<char*>(&ret.id[0]), len * sizeof(char));
  }

  // number of members
  str.read(reinterpret_cast<char*>(&ret.numMembers), sizeof(size_t));

  // rightmost point
  int32_t x, y;
  str.read(reinterpret_cast<char*>(&x), sizeof(int32_t));
  str.read(reinterpret_cast<char*>(&y), sizeof(int32_t));
  ret.rightPoint = {x, y};

  return {len * sizeof(char) + sizeof(size_t) + 2 * sizeof(int32_t), ret};
}

// ____________________________________________________________________________
template <>
std::pair<size_t, sj::Point> sj::TypedGeometryStore<sj::Point>::getFrom(
    size_t off, std::istream& str) const {
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
  str.read(reinterpret_cast<char*>(&ret.subId), sizeof(size_t));

  return {len * sizeof(char) + sizeof(size_t), ret};
}

// ____________________________________________________________________________
template <>
std::pair<size_t, sj::SimpleArea> sj::TypedGeometryStore<sj::SimpleArea>::getFrom(
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

  return {sizeof(uint32_t) + size * sizeof(util::geo::I32Point) +
              len * sizeof(char),
          ret};
}

// ____________________________________________________________________________
template <>
std::pair<size_t, sj::Line> sj::TypedGeometryStore<sj::Line>::getFrom(
    size_t off, std::istream& str) const {
  sj::Line ret;
  size_t estSize = 0;

  str.seekg(off);

  // geom
  estSize += readLine(str, ret.geom);

  // id
  uint16_t len;
  str.read(reinterpret_cast<char*>(&len), sizeof(uint16_t));
  ret.id.resize(len);

  if (len) {
    str.read(reinterpret_cast<char*>(&ret.id[0]), len * sizeof(char));
    estSize += len * sizeof(char);
  }

  // sub id
  str.read(reinterpret_cast<char*>(&ret.subId), sizeof(size_t));
  estSize += sizeof(size_t);

  // boxIds
  uint32_t numBoxIds;
  str.read(reinterpret_cast<char*>(&numBoxIds), sizeof(uint32_t));
  ret.boxIds.resize(numBoxIds);
  if (numBoxIds > 0) {
    str.read(reinterpret_cast<char*>(&ret.boxIds[0]),
             sizeof(sj::boxids::BoxId) * numBoxIds);
    estSize += sizeof(sj::boxids::BoxId) * numBoxIds;
  }

  return {estSize, ret};
}

// ____________________________________________________________________________
template <>
std::pair<size_t, sj::Area> sj::TypedGeometryStore<sj::Area>::getFrom(
    size_t off, std::istream& str) const {
  sj::Area ret;
  size_t estSize = 0;

  str.seekg(off);

  // geom
  estSize += readPoly(str, ret.geom);

  // id
  uint16_t len;
  str.read(reinterpret_cast<char*>(&len), sizeof(uint16_t));
  ret.id.resize(len);

  if (len) {
    str.read(reinterpret_cast<char*>(&ret.id[0]), len * sizeof(char));
    estSize += len * sizeof(char);
  }

  // sub id
  str.read(reinterpret_cast<char*>(&ret.subId), sizeof(size_t));
  estSize += sizeof(size_t);

  // boxIds
  uint32_t numBoxIds;
  str.read(reinterpret_cast<char*>(&numBoxIds), sizeof(uint32_t));
  ret.boxIds.resize(numBoxIds);
  if (numBoxIds > 0) {
    str.read(reinterpret_cast<char*>(&ret.boxIds[0]),
             sizeof(sj::boxids::BoxId) * numBoxIds);
    estSize += sizeof(sj::boxids::BoxId) * numBoxIds;
  }

  return {estSize, ret};
}

// ____________________________________________________________________________
template <>
size_t sj::TypedGeometryStore<sj::Point>::writeTo(const sj::Point& val,
                                             std::ostream& str) const {
  size_t ret = 0;

  // id
  if (val.id.size() > std::numeric_limits<uint16_t>::max()) {
    throw std::out_of_range(
        "Geometry ID is longer than " +
        std::to_string(std::numeric_limits<uint16_t>::max()));
  }
  uint16_t s = val.id.size();
  str.write(reinterpret_cast<const char*>(&s), sizeof(uint16_t));
  ret += sizeof(uint16_t);
  str.write(reinterpret_cast<const char*>(val.id.c_str()),
            val.id.size() * sizeof(char));
  ret += sizeof(char) * val.id.size();

  // sub id
  str.write(reinterpret_cast<const char*>(&val.subId), sizeof(size_t));
  ret += sizeof(size_t);

  return ret;
}

// ____________________________________________________________________________
template <>
size_t sj::TypedGeometryStore<sj::MultiGeom>::writeTo(const sj::MultiGeom& val,
                                                 std::ostream& str) const {
  size_t ret = 0;

  // id
  if (val.id.size() > std::numeric_limits<uint16_t>::max()) {
    throw std::out_of_range(
        "Geometry ID is longer than " +
        std::to_string(std::numeric_limits<uint16_t>::max()));
  }
  uint16_t s = val.id.size();
  str.write(reinterpret_cast<const char*>(&s), sizeof(uint16_t));
  ret += sizeof(uint16_t);
  str.write(reinterpret_cast<const char*>(val.id.c_str()),
            val.id.size() * sizeof(char));
  ret += sizeof(char) * val.id.size();

  // number of members
  str.write(reinterpret_cast<const char*>(&val.numMembers), sizeof(size_t));
  ret += sizeof(size_t);

  // rightmost point
  int32_t x = val.rightPoint.getX();
  int32_t y = val.rightPoint.getY();
  str.write(reinterpret_cast<const char*>(&x), sizeof(int32_t));
  str.write(reinterpret_cast<const char*>(&y), sizeof(int32_t));
  ret += 2 * sizeof(int32_t);

  return ret;
}

// ____________________________________________________________________________
template <typename W>
size_t sj::TypedGeometryStore<W>::add(const std::string& raw) {
  size_t ret = _geomsOffset;
  _geomsOffset += raw.size();

  if (_inMemory) {
    // cache for later use
    std::istringstream ss(raw);
    const auto& val = getFrom(0, ss);
    _memStore[ret] = std::make_shared<W>(std::move(val.second));

    if (_geomsOffset > MAX_MEM_CACHE_SIZE) {
      _inMemory = false;

      for (const auto& val : _memStore) {
        writeTo(*val.second.get(), _geomsF);
      }

      // clear mem store
      _memStore = {};
    }

    return typedId(ret, geomType());
  }

  _geomsF.write(raw.c_str(), raw.size());

  return typedId(ret, geomType());
}

// ____________________________________________________________________________
template <>
size_t sj::TypedGeometryStore<sj::SimpleArea>::writeTo(const sj::SimpleArea& val,
                                                  std::ostream& str) const {
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
  if (val.id.size() > std::numeric_limits<uint16_t>::max()) {
    throw std::out_of_range(
        "Geometry ID is longer than " +
        std::to_string(std::numeric_limits<uint16_t>::max()));
  }
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
size_t sj::TypedGeometryStore<sj::SimpleLine>::writeTo(const sj::SimpleLine& val,
                                                  std::ostream& str) const {
  size_t ret = 0;

  // id
  if (val.id.size() > std::numeric_limits<uint16_t>::max()) {
    throw std::out_of_range(
        "Geometry ID is longer than " +
        std::to_string(std::numeric_limits<uint16_t>::max()));
  }
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
size_t sj::TypedGeometryStore<sj::Line>::writeTo(const sj::Line& val,
                                            std::ostream& str) const {
  size_t ret = 0;

  // geoms
  ret += writeLine(val.geom, str);

  // id
  if (val.id.size() > std::numeric_limits<uint16_t>::max()) {
    throw std::out_of_range(
        "Geometry ID is longer than " +
        std::to_string(std::numeric_limits<uint16_t>::max()));
  }
  uint16_t s = val.id.size();
  str.write(reinterpret_cast<const char*>(&s), sizeof(uint16_t));
  ret += sizeof(uint16_t);
  str.write(reinterpret_cast<const char*>(val.id.c_str()),
            val.id.size() * sizeof(char));
  ret += sizeof(char) * val.id.size();

  // sub id
  str.write(reinterpret_cast<const char*>(&val.subId), sizeof(size_t));
  ret += sizeof(size_t);

  // boxIds
  uint32_t size = val.boxIds.size();
  str.write(reinterpret_cast<const char*>(&size), sizeof(uint32_t));
  if (size > 0) {
    str.write(reinterpret_cast<const char*>(&val.boxIds[0]),
              sizeof(sj::boxids::BoxId) * size);
  }

  ret += sizeof(uint32_t) + sizeof(sj::boxids::BoxId) * size;

  return ret;
}

// ____________________________________________________________________________
template <>
size_t sj::TypedGeometryStore<sj::Area>::writeTo(const sj::Area& val,
                                            std::ostream& str) const {
  size_t ret = 0;

  // geoms
  ret += writePoly(val.geom, str);

  // id
  if (val.id.size() > std::numeric_limits<uint16_t>::max()) {
    throw std::out_of_range(
        "Geometry ID is longer than " +
        std::to_string(std::numeric_limits<uint16_t>::max()));
  }
  uint16_t s = val.id.size();
  str.write(reinterpret_cast<const char*>(&s), sizeof(uint16_t));
  ret += sizeof(uint16_t);

  str.write(reinterpret_cast<const char*>(val.id.c_str()),
            val.id.size() * sizeof(char));
  ret += sizeof(char) * val.id.size();

  // sub id
  str.write(reinterpret_cast<const char*>(&val.subId), sizeof(size_t));
  ret += sizeof(size_t);

  // boxIds
  uint32_t size = val.boxIds.size();
  str.write(reinterpret_cast<const char*>(&size), sizeof(uint32_t));
  if (size > 0) {
    str.write(reinterpret_cast<const char*>(&val.boxIds[0]),
              sizeof(sj::boxids::BoxId) * size);
  }

  ret += sizeof(uint32_t) + sizeof(sj::boxids::BoxId) * size;

  return ret;
}

// ____________________________________________________________________________
template <typename W>
void sj::TypedGeometryStore<W>::flush() {
  if (_geomsF.is_open()) {
    _geomsF.flush();
    _geomsF.close();
  }
}

// ____________________________________________________________________________
template <typename W>
size_t sj::TypedGeometryStore<W>::readPoly(std::istream& str,
                                      util::geo::I32XSortedPolygon& ret) const {
  size_t estSize = 0;

  util::geo::I32Box box;
  str.read(reinterpret_cast<char*>(&box), sizeof(util::geo::I32Box));
  estSize += sizeof(util::geo::I32Box);

  double maxSegLen;
  str.read(reinterpret_cast<char*>(&maxSegLen), sizeof(double));
  estSize += sizeof(double);
  ret.getOuter().setMaxSegLen(maxSegLen);

  // outer area
  double area;
  str.read(reinterpret_cast<char*>(&area), sizeof(double));
  estSize += sizeof(double);
  ret.getOuter().setArea(area);

  uint32_t sizeOuter;
  str.read(reinterpret_cast<char*>(&sizeOuter), sizeof(uint32_t));

  // TODO: careful, resize initializes entire vector!
  if (sizeOuter) {
    ret.getOuter().rawRing().resize(sizeOuter);

    str.read(reinterpret_cast<char*>(&ret.getOuter().rawRing()[0]),
             sizeof(util::geo::XSortedTuple<int32_t>) * sizeOuter);
    estSize += sizeof(util::geo::XSortedTuple<int32_t>) * sizeOuter;

    ret.getOuter().setBoundingBox(box);
  }

  uint32_t numInners;
  str.read(reinterpret_cast<char*>(&numInners), sizeof(uint32_t));
  estSize += sizeof(uint32_t);

  double innerMaxSegLen;
  str.read(reinterpret_cast<char*>(&innerMaxSegLen), sizeof(double));
  estSize += sizeof(double);

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
      ret.getInners()[j].setBoundingBox(ret.getInnerBoxes()[j]);
      ret.getInners()[j].setArea(ret.getInnerAreas()[j]);
    }
  }

  return estSize;
}

// ____________________________________________________________________________
template <typename W>
size_t sj::TypedGeometryStore<W>::writePoly(const util::geo::I32XSortedPolygon& geom,
                                       std::ostream& str) {
  size_t ret = 0;

  // outer envelope
  const auto box = geom.boundingBox();
  str.write(reinterpret_cast<const char*>(&box), sizeof(util::geo::I32Box));
  ret += sizeof(util::geo::I32Box);

  // max seg len
  double maxSegLen = geom.getOuter().getMaxSegLen();
  str.write(reinterpret_cast<const char*>(&maxSegLen), sizeof(double));
  ret += sizeof(double);

  // outer area
  double area = geom.getOuter().area();
  str.write(reinterpret_cast<const char*>(&area), sizeof(double));
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
size_t sj::TypedGeometryStore<W>::readLine(std::istream& str,
                                      util::geo::I32XSortedLine& ret) const {
  size_t estSize = 0;

  util::geo::I32Box box;
  str.read(reinterpret_cast<char*>(&box), sizeof(util::geo::I32Box));
  estSize += sizeof(util::geo::I32Box);
  ret.setBoundingBox(box);

  // length
  double length;
  str.read(reinterpret_cast<char*>(&length), sizeof(double));
  estSize += sizeof(double);
  ret.setLength(length);

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
size_t sj::TypedGeometryStore<W>::writeLine(const util::geo::I32XSortedLine& geom,
                                       std::ostream& str) {
  size_t ret = 0;

  // outer envelope
  const auto box = geom.boundingBox();
  str.write(reinterpret_cast<const char*>(&box), sizeof(util::geo::I32Box));
  ret += sizeof(util::geo::I32Box);

  // length
  double length = geom.length();
  str.write(reinterpret_cast<const char*>(&length), sizeof(double));
  ret += sizeof(double);

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
std::string sj::TypedGeometryStore<sj::Area>::getFName() const {
  return util::getTmpFName(_dir, _tmpPrefix, "areas");
}

// ____________________________________________________________________________
template <>
std::string sj::TypedGeometryStore<sj::Line>::getFName() const {
  return util::getTmpFName(_dir, _tmpPrefix, "lines");
}

// ____________________________________________________________________________
template <>
std::string sj::TypedGeometryStore<sj::Point>::getFName() const {
  return util::getTmpFName(_dir, _tmpPrefix, "points");
}

// ____________________________________________________________________________
template <>
std::string sj::TypedGeometryStore<sj::SimpleLine>::getFName() const {
  return util::getTmpFName(_dir, _tmpPrefix, "simplelines");
}

// ____________________________________________________________________________
template <>
std::string sj::TypedGeometryStore<sj::SimpleArea>::getFName() const {
  return util::getTmpFName(_dir, ".spatialjoin", "simpleareas");
}

// ____________________________________________________________________________
template <>
std::string sj::TypedGeometryStore<sj::MultiGeom>::getFName() const {
  return util::getTmpFName(_dir, _tmpPrefix, "multis");
}

// ____________________________________________________________________________
template <>
sj::GeomType sj::TypedGeometryStore<sj::Area>::geomType() {
  return sj::POLYGON;
}

// ____________________________________________________________________________
template <>
sj::GeomType sj::TypedGeometryStore<sj::Line>::geomType() {
  return sj::LINE;
}

// ____________________________________________________________________________
template <>
sj::GeomType sj::TypedGeometryStore<sj::Point>::geomType() {
  return sj::POINT;
}

// ____________________________________________________________________________
template <>
sj::GeomType sj::TypedGeometryStore<sj::SimpleLine>::geomType() {
  return sj::SIMPLE_LINE;
}

// ____________________________________________________________________________
template <>
sj::GeomType sj::TypedGeometryStore<sj::SimpleArea>::geomType() {
  return sj::SIMPLE_POLYGON;
}

// ____________________________________________________________________________
template <>
sj::GeomType sj::TypedGeometryStore<sj::MultiGeom>::geomType() {
  // multi geometries are swept as special POINT events
  return sj::POINT;
}

// ____________________________________________________________________________
template class sj::TypedGeometryStore<sj::Area>;
template class sj::TypedGeometryStore<sj::Line>;
template class sj::TypedGeometryStore<sj::SimpleLine>;
template class sj::TypedGeometryStore<sj::Point>;
template class sj::TypedGeometryStore<sj::SimpleArea>;
template class sj::TypedGeometryStore<sj::MultiGeom>;

using sj::GeometryStore;
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

const static double sin45 = 1.0 / sqrt(2);
const static double cos45 = 1.0 / sqrt(2);

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32MultiPolygon& a,
                                 const std::string& gid, bool side,
                                 WriteBatch& batch) const {
  size_t subid = 0;  // a subid of 0 means "single polygon"
  if (a.size() > 1) subid = 1;

  return add(a, gid, subid, side, batch);
}

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32MultiLine& a, const std::string& gid,
                                 bool side, WriteBatch& batch) const {
  size_t subid = 0;  // a subid of 0 means "single line"
  if (a.size() > 1) subid = 1;

  return add(a, gid, subid, side, batch);
}

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32MultiPoint& a, const std::string& gid,
                                 bool side, WriteBatch& batch) const {
  size_t subid = 0;  // a subid of 0 means "single point"
  if (a.size() > 1) subid = 1;

  return add(a, gid, subid, side, batch);
}

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32MultiPolygon& a,
                                 const std::string& gid, size_t subId,
                                 bool side, WriteBatch& batch) const {
  I32Box ret;
  for (const auto& poly : a) {
    if (poly.getOuter().size() < 2) continue;
    auto box = add(poly, gid, subId, side, batch);
    if (box.isNull()) continue;
    ret = util::geo::extendBox(box, ret);
    subId++;
  }

  return ret;
}

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32MultiLine& a, const std::string& gid,
                                 size_t subId, bool side,
                                 WriteBatch& batch) const {
  I32Box ret;
  for (const auto& line : a) {
    if (line.size() < 2) continue;
    auto box = add(line, gid, subId, side, batch);
    if (box.isNull()) continue;
    ret = util::geo::extendBox(box, ret);
    subId++;
  }

  return ret;
}

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32MultiPoint& a, const std::string& gid,
                                 size_t subid, bool side,
                                 WriteBatch& batch) const {
  I32Box ret;
  size_t newId = subid;
  for (const auto& point : a) {
    auto box = add(point, gid, newId, side, batch);
    if (box.isNull()) continue;
    ret = util::geo::extendBox(box, ret);
    newId++;
  }

  return ret;
}

// _____________________________________________________________________________
void GeometryStore::multiAdd(const std::string& gid, bool side,
                                    int32_t xRight,
                                    const I32Point& pointRight) {
  auto i = _multiGidToId[side].find(gid);

  if (i == _multiGidToId[side].end()) {
    _multiIds[side].push_back(gid);
    _multiRightX[side].push_back(xRight);
    _multiRightPoint[gid] = pointRight;
    _multiGidToId[side][gid] = _multiIds[side].size() - 1;
    _subSizes[gid] = 1;
  } else {
    size_t id = _multiGidToId[side][gid];
    if (xRight > _multiRightX[side][id]) _multiRightX[side][id] = xRight;
    if (pointRight.getX() > _multiRightPoint[gid].getX())
      _multiRightPoint[gid] = pointRight;
    _subSizes[gid] = _subSizes[gid] + 1;
  }
}

// _____________________________________________________________________________
void GeometryStore::add(const std::string& parentR,
                               const util::geo::I32Box& box,
                               const std::string& gidR, size_t subid, bool side,
                               WriteBatch& batch) const {
  // NOTE: referencing atm *only* works if the referenced geometry is a non-
  // multi geometry. If a multi geometry is referenced, the behavior is
  // undefined.
  std::string gid = (side ? ("B" + gidR) : ("A" + gidR));
  std::string parent = (side ? ("B" + parentR) : ("A" + parentR));

  BoxVal boxl, boxr;
  boxl.side = side;
  boxr.side = side;

  boxl.val = box.getLowerLeft().getX();
  boxr.val = box.getUpperRight().getX();

  batch.refs.push_back({parent, gid, boxl, boxr, subid});
}

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32Polygon& poly, const std::string& gid,
                                 bool side, WriteBatch& batch) const {
  return add(poly, gid, 0, side, batch);
}

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32Polygon& poly,
                                 const std::string& gidR, size_t subid,
                                 bool side, WriteBatch& batch) const {
  if (subid == 0 && _cfg.de9imFilter != util::geo::FANY) {
    // drop certain geometries if we can be sure that they will never match
    // the given DE-9IM filter
    if (_cfg.de9imFilter.minBoundaryDim() > 1) return {};
    if (_cfg.de9imFilter.maxInteriorDim() < 2) return {};
    if (side && _cfg.de9imFilter.maxRightInteriorDim() < 2) return {};
    if (side && _cfg.de9imFilter.minRightBoundaryDim() > 1) return {};
    if (_numSides > 1 && !side && _cfg.de9imFilter.maxLeftInteriorDim() < 2)
      return {};
    if (_numSides > 1 && !side && _cfg.de9imFilter.minLeftBoundaryDim() > 1)
      return {};
  }

  std::string gid = (side ? ("B" + gidR) : ("A" + gidR));

  WriteCand cur;
  const auto& densePoly = util::geo::densifyX(poly, 500 * PREC);
  I32XSortedPolygon spoly(densePoly);
  const auto& rawBox = spoly.boundingBox();
  const auto& box = getPaddedBoundingBox(rawBox);
  if (!util::geo::intersects(box, _filterBox)) return {};

  if (spoly.empty()) return box;

  size_t polySize = poly.size();
  double areaSize = area(poly);

  double outerAreaSize = outerArea(poly);
  BoxIdList boxIds;

  if (_cfg.useBoxIds) {
    boxIds = packBoxIds(getBoxIds(spoly, rawBox, outerAreaSize));
  }

  I32Box box45;
  if (_cfg.useDiagBox) {
    auto polyR = util::geo::rotateSinCos(poly, sin45, cos45, I32Point(0, 0));
    box45 = getPaddedBoundingBox(polyR, rawBox);
  }

  cur.subid = subid;
  cur.gid = gid;

  if (poly.getInners().size() == 0 && subid == 0 && gid.size() < 8 &&
      poly.getOuter().size() == densePoly.getOuter().size() &&
      (!_cfg.useBoxIds || boxIds.front().first == 1) &&
      area(rawBox) == areaSize) {
    cur.boxvalIn = {typedId(0, FOLDED_BOX_POLYGON),  // placeholder ID
                    box.getLowerLeft().getY(),
                    box.getUpperRight().getY(),
                    box.getLowerLeft().getX(),
                    false,
                    areaSize,
                    rawBox.getUpperRight(),
                    packPoint(rawBox.getLowerLeft()),
                    box45,
                    side};
    cur.boxvalOut = {typedId(0, FOLDED_BOX_POLYGON),  // placeholder ID
                     box.getLowerLeft().getY(),
                     box.getUpperRight().getY(),
                     box.getUpperRight().getX(),
                     true,
                     areaSize,
                     rawBox.getLowerLeft(),
                     packPoint(rawBox.getUpperRight()),
                     box45,
                     side};
    batch.foldedBoxAreas.emplace_back(cur);
  } else if (poly.getInners().size() == 0 && poly.getOuter().size() < 10 &&
             poly.getOuter().size() == densePoly.getOuter().size() &&
             subid == 0 && (!_cfg.useBoxIds || boxIds.front().first == 1)) {
    std::stringstream str;
    _simpleAreaStore.writeTo({poly.getOuter(), gid}, str);
    cur.raw = str.str();

    auto rightPoint = poly.getOuter().front();

    for (const auto& p : poly.getOuter()) {
      if (p.getX() > rightPoint.getX()) rightPoint = p;
    }

    cur.boxvalIn = {typedId(0, SIMPLE_POLYGON),  // placeholder ID
                    box.getLowerLeft().getY(),
                    box.getUpperRight().getY(),
                    box.getLowerLeft().getX(),
                    false,
                    areaSize,
                    {},
                    poly.size(),
                    box45,
                    side};
    cur.boxvalOut = {typedId(0, SIMPLE_POLYGON),  // placeholder ID
                     box.getLowerLeft().getY(),
                     box.getUpperRight().getY(),
                     box.getUpperRight().getX(),
                     true,
                     areaSize,
                     rightPoint,
                     poly.size(),
                     box45,
                     side};
    batch.simpleAreas.emplace_back(cur);
  } else {
    if (!_cfg.useFastSweepSkip) {
      spoly.setInnerMaxSegLen(std::numeric_limits<int32_t>::max());
      spoly.getOuter().setMaxSegLen(std::numeric_limits<int32_t>::max());
      for (auto& inner : spoly.getInners()) {
        inner.setMaxSegLen(std::numeric_limits<int32_t>::max());
      }
    }

    // careful, assign this before move below
    auto rightPoint = spoly.getOuter().rawRing().back().p;

    std::stringstream str;
    _areaStore.writeTo({std::move(spoly), gid, subid, boxIds}, str);
    ;

    cur.raw = str.str();

    cur.boxvalIn = {typedId(0, POLYGON),  // placeholder ID
                    box.getLowerLeft().getY(),
                    box.getUpperRight().getY(),
                    box.getLowerLeft().getX(),
                    false,
                    areaSize,
                    {},
                    polySize,
                    box45,
                    side};
    cur.boxvalOut = {typedId(0, POLYGON),  // placeholder ID
                     box.getLowerLeft().getY(),
                     box.getUpperRight().getY(),
                     box.getUpperRight().getX(),
                     true,
                     areaSize,
                     rightPoint,
                     polySize,
                     box45,
                     side};
    batch.areas.emplace_back(cur);
  }

  return box;
}

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32Line& line, const std::string& gid,
                                 bool side, WriteBatch& batch) const {
  return add(line, gid, 0, side, batch);
}

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32Line& line, const std::string& gidR,
                                 size_t subid, bool side,
                                 WriteBatch& batch) const {
  if (line.size() < 2) return {};

  if (subid == 0 && _cfg.de9imFilter != util::geo::FANY) {
    // drop certain geometries if we can be sure that they will never match
    // the given DE-9IM filter
    if (_cfg.de9imFilter.minInteriorDim() > 1) return {};
    if (_cfg.de9imFilter.minBoundaryDim() > 0) return {};
    if (_cfg.de9imFilter.maxInteriorDim() < 1) return {};
    if (side && _cfg.de9imFilter.minRightInteriorDim() > 1) return {};
    if (side && _cfg.de9imFilter.minRightBoundaryDim() > 0) return {};
    if (side && _cfg.de9imFilter.maxRightInteriorDim() < 1) return {};
    if (_numSides > 1 && !side && _cfg.de9imFilter.minLeftInteriorDim() > 1)
      return {};
    if (_numSides > 1 && !side && _cfg.de9imFilter.maxLeftInteriorDim() < 1)
      return {};
    if (_numSides > 1 && !side && _cfg.de9imFilter.minLeftBoundaryDim() > 0)
      return {};
  }
  if (_cfg.de9imFilter.maxExteriorDim() < 2) return {};

  std::string gid = (side ? ("B" + gidR) : ("A" + gidR));

  WriteCand cur;

  const auto& denseLine = util::geo::densifyX(line, 500 * PREC);
  I32XSortedLine sline(denseLine);

  const auto& rawBox = sline.boundingBox();
  const auto& box = getPaddedBoundingBox(rawBox);

  if (!util::geo::intersects(box, _filterBox)) return {};
  BoxIdList boxIds;

  if (_cfg.useBoxIds) {
    boxIds = packBoxIds(getBoxIds(line, rawBox));
  }

  const double len = util::geo::len(line);
  size_t lineSize = line.size();

  I32Box box45;
  if (_cfg.useDiagBox) {
    auto lineR = util::geo::rotateSinCos(line, sin45, cos45, I32Point(0, 0));
    box45 = getPaddedBoundingBox(lineR, rawBox);
  }

  cur.subid = subid;
  cur.gid = gid;

  if (line.size() == 2 && denseLine.size() == line.size() &&
      (!_cfg.useBoxIds || boxIds.front().first == 1) && subid == 0) {
    // simple line

    cur.boxvalIn = {
        typedId(0, SIMPLE_LINE),  // placeholder ID
        box.getLowerLeft().getY(),
        box.getUpperRight().getY(),
        box.getLowerLeft().getX(),
        false,
        len,
        line.front().getX() < line.back().getX() ? line.back() : line.front(),
        packPoint(line.front().getX() < line.back().getX() ? line.front()
                                                           : line.back()),
        box45,
        side};
    cur.boxvalOut = {
        typedId(0, SIMPLE_LINE),  // placeholder ID
        box.getLowerLeft().getY(),
        box.getUpperRight().getY(),
        box.getUpperRight().getX(),
        true,
        len,
        line.front().getX() < line.back().getX() ? line.front() : line.back(),
        packPoint(line.front().getX() < line.back().getX() ? line.back()
                                                           : line.front()),
        box45,
        side};

    // check if we can fold the gid into the offset id, because the gid is all
    // we store in the cache for points
    if (subid == 0 && gid.size() < 8) {
      cur.boxvalIn.setType(FOLDED_SIMPLE_LINE);
      cur.boxvalOut.setType(FOLDED_SIMPLE_LINE);
      batch.foldedSimpleLines.emplace_back(cur);
    } else {
      std::stringstream str;
      _simpleLineStore.writeTo({gid}, str);

      cur.raw = str.str();

      batch.simpleLines.emplace_back(cur);
    }
  } else {
    // normal line
    if (line.empty()) return {};
    if (sline.rawLine().empty()) return {};
    auto rightPoint = sline.rawLine().back().p;

    if (!_cfg.useFastSweepSkip) {
      sline.setMaxSegLen(std::numeric_limits<int32_t>::max());
    }

    std::stringstream str;
    _lineStore.writeTo({std::move(sline), gid, subid, boxIds}, str);
    cur.raw = str.str();

    cur.boxvalIn = {typedId(0, LINE),  // placeholder ID
                    box.getLowerLeft().getY(),
                    box.getUpperRight().getY(),
                    box.getLowerLeft().getX(),
                    false,
                    len,
                    {},
                    lineSize,
                    box45,
                    side};
    cur.boxvalOut = {typedId(0, LINE),  // placeholder ID
                     box.getLowerLeft().getY(),
                     box.getUpperRight().getY(),
                     box.getUpperRight().getX(),
                     true,
                     len,
                     rightPoint,
                     lineSize,
                     box45,
                     side};
    batch.lines.emplace_back(cur);
  }

  return box;
}

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32Point& point, const std::string& gid,
                                 bool side, WriteBatch& batch) const {
  return add(point, gid, 0, side, batch);
}

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32Point& point, const std::string& gidR,
                                 size_t subid, bool side,
                                 WriteBatch& batch) const {
  // THIS NO LONGER WORKS IF THE CACHE IS BUILD INDEPENDENT OF THE JOINPREDICATE
  if (subid == 0 && _cfg.de9imFilter != util::geo::FANY) {
    // drop certain geometries if we can be sure that they will never match
    // the given DE-9IM filter
    if (_cfg.de9imFilter.minInteriorDim() > 0) return {};
    if (_cfg.de9imFilter.minBoundaryDim() >= 0) return {};
    if (_cfg.de9imFilter.maxInteriorDim() < 0) return {};
    if (side && _cfg.de9imFilter.minRightInteriorDim() > 0) return {};
    if (side && _cfg.de9imFilter.maxRightInteriorDim() < 0) return {};
    if (side && _cfg.de9imFilter.minRightBoundaryDim() >= 0) return {};
    if (_numSides > 1 && !side && _cfg.de9imFilter.minLeftInteriorDim() > 0)
      return {};
    if (_numSides > 1 && !side && _cfg.de9imFilter.maxLeftInteriorDim() < 0)
      return {};
    if (_numSides > 1 && !side && _cfg.de9imFilter.minLeftBoundaryDim() >= 0)
      return {};
  }

  if (_cfg.de9imFilter.maxExteriorDim() < 2) return {};

  std::string gid = (side ? ("B" + gidR) : ("A" + gidR));

  WriteCand cur;

  const auto& rawBox = getBoundingBox(point);
  const auto& box = getPaddedBoundingBox(rawBox);

  cur.subid = subid;

  if (!util::geo::intersects(box, _filterBox)) return {};

  auto pointR = util::geo::rotateSinCos(point, sin45, cos45, I32Point(0, 0));
  cur.boxvalIn = {typedId(0, POINT),  // placeholder ID
                  box.getLowerLeft().getY(),
                  box.getUpperRight().getY(),
                  box.getLowerLeft().getX(),
                  false,
                  0,
                  point,
                  1,
                  getPaddedBoundingBox(pointR, rawBox),
                  side};
  cur.boxvalOut = {typedId(0, POINT),  // placeholder ID
                   box.getLowerLeft().getY(),
                   box.getUpperRight().getY(),
                   box.getUpperRight().getX(),
                   true,
                   0,
                   point,
                   1,
                   getPaddedBoundingBox(pointR, rawBox),
                   side};

  cur.gid = gid;

  // check if we can fold the gid into the offset id, because the gid is all
  // we store in the cache for points
  if (subid == 0 && gid.size() < 8) {
    cur.boxvalIn.setType(FOLDED_POINT);
    cur.boxvalOut.setType(FOLDED_POINT);
    batch.foldedPoints.emplace_back(cur);
  } else {
    std::stringstream str;
    _pointStore.writeTo({gid, subid}, str);

    cur.raw = str.str();

    batch.points.emplace_back(cur);
  }

  return box;
}

// _____________________________________________________________________________
void GeometryStore::addBatch(WriteBatch& cands,
                                    sj::SweepEventList* events) {
  {
    for (auto& cand : cands.foldedPoints) {
      if (cand.boxvalIn.side) _numSides = 2;
      cand.boxvalIn.id =
          typedId(foldString(cand.gid), cand.boxvalIn.getType());
      cand.boxvalOut.id = cand.boxvalIn.id;
    }
  }

  {
    std::unique_lock<std::mutex> lock(_pointGeomCacheWriteMtx);
    for (auto& cand : cands.points) {
      if (cand.boxvalIn.side) _numSides = 2;
      cand.boxvalIn.id = _pointStore.add(cand.raw);
      cand.boxvalOut.id = cand.boxvalIn.id;
    }
  }

  {
    std::unique_lock<std::mutex> lock(_lineGeomCacheWriteMtx);
    for (auto& cand : cands.lines) {
      if (cand.boxvalIn.side) _numSides = 2;
      cand.boxvalIn.id = _lineStore.add(cand.raw);
      cand.boxvalOut.id = cand.boxvalIn.id;
    }
  }

  {
    std::unique_lock<std::mutex> lock(_simpleLineGeomCacheWriteMtx);
    for (auto& cand : cands.simpleLines) {
      if (cand.boxvalIn.side) _numSides = 2;
      cand.boxvalIn.id = _simpleLineStore.add(cand.raw);
      cand.boxvalOut.id = cand.boxvalIn.id;
    }
  }

  {
    for (auto& cand : cands.foldedSimpleLines) {
      if (cand.boxvalIn.side) _numSides = 2;
      cand.boxvalIn.id =
          typedId(foldString(cand.gid), cand.boxvalIn.getType());
      cand.boxvalOut.id = cand.boxvalIn.id;
    }
  }

  {
    for (auto& cand : cands.foldedBoxAreas) {
      if (cand.boxvalIn.side) _numSides = 2;
      cand.boxvalIn.id =
          typedId(foldString(cand.gid), cand.boxvalIn.getType());
      cand.boxvalOut.id = cand.boxvalIn.id;
    }
  }

  {
    std::unique_lock<std::mutex> lock(_simpleAreaGeomCacheWriteMtx);
    for (auto& cand : cands.simpleAreas) {
      if (cand.boxvalIn.side) _numSides = 2;
      cand.boxvalIn.id = _simpleAreaStore.add(cand.raw);
      cand.boxvalOut.id = cand.boxvalIn.id;
    }
  }

  {
    std::unique_lock<std::mutex> lock(_areaGeomCacheWriteMtx);
    for (auto& cand : cands.areas) {
      if (cand.boxvalIn.side) _numSides = 2;
      cand.boxvalIn.id = _areaStore.add(cand.raw);
      cand.boxvalOut.id = cand.boxvalIn.id;
    }
  }

  for (const auto& cand : cands.points) {
    if (cand.subid > 0) {
      std::unique_lock<std::mutex> lock(_multiAddMtx);
      multiAdd(cand.gid, cand.boxvalIn.side, cand.boxvalOut.val,
               cand.boxvalOut.point);
    }
  }

  for (const auto& cand : cands.simpleLines) {
    if (cand.subid > 0) {
      std::unique_lock<std::mutex> lock(_multiAddMtx);
      multiAdd(cand.gid, cand.boxvalIn.side, cand.boxvalOut.val,
               cand.boxvalOut.point);
    }
  }

  for (const auto& cand : cands.lines) {
    if (cand.subid > 0) {
      std::unique_lock<std::mutex> lock(_multiAddMtx);
      multiAdd(cand.gid, cand.boxvalIn.side, cand.boxvalOut.val,
               cand.boxvalOut.point);
    }
  }

  for (const auto& cand : cands.simpleAreas) {
    if (cand.subid > 0) {
      std::unique_lock<std::mutex> lock(_multiAddMtx);
      multiAdd(cand.gid, cand.boxvalIn.side, cand.boxvalOut.val,
               cand.boxvalOut.point);
    }
  }

  for (const auto& cand : cands.areas) {
    if (cand.subid > 0) {
      std::unique_lock<std::mutex> lock(_multiAddMtx);
      multiAdd(cand.gid, cand.boxvalIn.side, cand.boxvalOut.val,
               cand.boxvalOut.point);
    }
  }

  for (const auto& cand : cands.refs) {
    if (cand.subid > 0) {
      std::unique_lock<std::mutex> lock(_multiAddMtx);
      multiAdd(cand.gid, cand.boxvalIn.side, cand.boxvalOut.val,
               cand.boxvalOut.point);
    }
  }

  {
    std::unique_lock<std::mutex> lock(_sweepEventWriteMtx);
    for (const auto& cand : cands.foldedPoints) {
      events->add(cand.boxvalIn);
      events->add(cand.boxvalOut);
      if (events->numObjects() % 1000000 == 0)
        log("@ " + std::to_string(events->numObjects()));
    }
    for (const auto& cand : cands.points) {
      events->add(cand.boxvalIn);
      events->add(cand.boxvalOut);
      if (events->numObjects() % 1000000 == 0)
        log("@ " + std::to_string(events->numObjects()));
    }
    for (const auto& cand : cands.foldedSimpleLines) {
      events->add(cand.boxvalIn);
      events->add(cand.boxvalOut);
      if (events->numObjects() % 1000000 == 0)
        log("@ " + std::to_string(events->numObjects()));
    }
    for (const auto& cand : cands.foldedBoxAreas) {
      events->add(cand.boxvalIn);
      events->add(cand.boxvalOut);
      if (events->numObjects() % 1000000 == 0)
        log("@ " + std::to_string(events->numObjects()));
    }
    for (const auto& cand : cands.simpleLines) {
      events->add(cand.boxvalIn);
      events->add(cand.boxvalOut);
      if (events->numObjects() % 1000000 == 0)
        log("@ " + std::to_string(events->numObjects()));
    }
    for (const auto& cand : cands.lines) {
      events->add(cand.boxvalIn);
      events->add(cand.boxvalOut);
      if (events->numObjects() % 1000000 == 0)
        log("@ " + std::to_string(events->numObjects()));
    }
    for (const auto& cand : cands.simpleAreas) {
      events->add(cand.boxvalIn);
      events->add(cand.boxvalOut);
      if (events->numObjects() % 1000000 == 0)
        log("@ " + std::to_string(events->numObjects()));
    }
    for (const auto& cand : cands.areas) {
      events->add(cand.boxvalIn);
      events->add(cand.boxvalOut);
      if (events->numObjects() % 1000000 == 0)
        log("@ " + std::to_string(events->numObjects()));
    }
    for (const auto& cand : cands.refs) {
      _refs[cand.raw][0][cand.gid] = cand.subid;
      _selfCheckBounds[cand.raw] = util::geo::getBoundingBox(
          I32Point{cand.boxvalIn.val, cand.boxvalIn.loY});
      if (events->numObjects() % 1000000 == 0)
        log("@ " + std::to_string(events->numObjects()));
    }
  }
}

// _____________________________________________________________________________
void GeometryStore::flush(sj::SweepEventList* events) {
  if (_numSides > 1) log("(Non-self join between 2 datasets)");

  log(std::to_string(_multiIds[0].size() + _multiIds[1].size()) +
      " multi geometries");

  for (const auto& ref : _refs) {
    for (const auto& sub : ref.second) {
      _selfChecks.push_back({ref.first, sub.first});

      events->add({typedId(_selfChecks.size() - 1, SELF_CHECK),
                   1,
                   0,
                   _selfCheckBounds[ref.first].getLowerLeft().getX(),
                   false,
                   0.0,
                   {},
                   0,
                   {},
                   false});
    }
  }

  // special multi-OUT event, at the right x of the multi geometry
  for (size_t side = 0; side < 2; side++) {
    for (size_t i = 0; i < _multiIds[side].size(); i++) {
      events->add({typedId(i, POINT),
                   1,
                   0,
                   _multiRightX[side][i],
                   true,
                   0.0,
                   {},
                   0,
                   {},
                   static_cast<bool>(side)});
    }
  }

  _pointStore.flush();
  _areaStore.flush();
  _simpleAreaStore.flush();
  _lineStore.flush();
  _simpleLineStore.flush();

  // TODO!!!!
  // duplicatesToReferences();

  log(std::to_string(_refs.size()) + " reference geometries");
}

// _____________________________________________________________________________
void GeometryStore::duplicatesToReferences(int file) {
  // start at beginning of file
  lseek(file, 0, SEEK_SET);

  const size_t RBUF_SIZE = 100000;
  unsigned char* buf = new unsigned char[sizeof(BoxVal) * RBUF_SIZE];

  std::unordered_set<size_t> deletedLines;
  std::unordered_set<size_t> referencedLines;
  std::unordered_set<size_t> deletedPolys;
  std::unordered_set<size_t> referencedPolys;

  log("Removing duplicates...");

  ssize_t len;
  size_t jj = 0;

  int32_t curX = 0;

  std::unordered_map<uint64_t, size_t> duplicatePolys, duplicateLines;

  size_t pos = 0;

  try {
    while ((len = preadAll(file, buf, sizeof(BoxVal) * RBUF_SIZE, pos)) != 0) {
      size_t posOld = pos;
      pos += len;
      if (len < 0) {
        std::stringstream ss;
        ss << "Could not read from events file \n";
        ss << strerror(errno) << std::endl;
        throw std::runtime_error(ss.str());
      }

      if (len % sizeof(BoxVal))
        throw std::runtime_error("Corrupted events file");

      bool updated = false;

      for (ssize_t i = 0; i < len; i += sizeof(BoxVal)) {
        auto cur = reinterpret_cast<BoxVal*>(buf + i);

        if (_cfg.sweepCancellationCb && jj % 10000 == 0) {
          _cfg.sweepCancellationCb();
        }

        jj++;

        if (cur->out) {
          if (cur->getType() == POLYGON) {
            if (deletedPolys.erase(getId(*cur))) {
              // erase it if present, to avoid unnecessary memory consumption
              cur->setType(DELETED);
              updated = true;
            }
            referencedPolys.erase(getId(*cur));
          }

          if (cur->getType() == LINE) {
            if (deletedLines.erase(getId(*cur))) {
              // erase it if present, to avoid unnecessary memory consumption
              cur->setType(DELETED);
              updated = true;
            }
            referencedLines.erase(getId(*cur));
          }

          continue;
        }

        if (curX != cur->val) {
          // new equal-X block
          duplicatePolys = {};
          duplicateLines = {};
          curX = cur->val;
        }

        if (cur->getType() == POLYGON &&
            cur->numAnchors >= DUPLICATE_REMOVAL_MIN_SIZE) {
          size_t h = cur->numAnchors;
          const auto& existing = duplicatePolys.find(h);

          if (existing != duplicatePolys.end()) {
            auto a = get(_areaStore, getId(*cur));
            auto b = get(_areaStore, existing->second);

            if (a->geom == b->geom) {
              deletedPolys.insert(getId(*cur));
              if (referencedPolys.insert(existing->second).second) {
                // for the first element referencing this, modify this
                // event to the self check of the referenced geom
                cur->setType(SELF_CHECK_AREA);
                _selfChecks.push_back({b->id, b->subId});
                cur->id = typedId(_selfChecks.size() - 1, cur->getType());
              } else {
                cur->setType(DELETED);
              }

              updated = true;
              _refs[b->id][b->subId][a->id] = a->subId;
            }
          } else {
            duplicatePolys[h] = getId(*cur);
          }
        }

        if (cur->getType() == LINE &&
            cur->numAnchors >= DUPLICATE_REMOVAL_MIN_SIZE) {
          size_t h = cur->numAnchors;
          const auto& existing = duplicateLines.find(h);

          if (existing != duplicateLines.end()) {
            auto a = get(_lineStore, getId(*cur));
            auto b = get(_lineStore, existing->second);

            if (a->geom == b->geom) {
              deletedLines.insert(getId(*cur));
              if (referencedLines.insert(existing->second).second) {
                // for the first element referencing this, modify this
                // event to the self check of the referenced geom
                cur->setType(SELF_CHECK_LINE);
                _selfChecks.push_back({b->id, b->subId});
                cur->id = typedId(_selfChecks.size() - 1, cur->getType());
              } else {
                cur->setType(DELETED);
              }
              updated = true;
              _refs[b->id][b->subId][a->id] = a->subId;
            }
          } else {
            duplicateLines[h] = getId(*cur);
          }
        }
      }

      // if we changed something in this buffer, write it back
      if (updated) pwriteAll(file, buf, len, posOld);
    }
  } catch (...) {
    // graceful handling of an exception during sweep

    delete[] buf;

    // rethrow exception
    throw;
  }

  delete[] buf;

  log("...done");
}

// _____________________________________________________________________________
void GeometryStore::log(const std::string& msg) {
  if (_cfg.logCb) _cfg.logCb(msg);
}

// _____________________________________________________________________________
template <template <typename> class G1, template <typename> class G2,
          typename T>
util::geo::I32Box GeometryStore::getPaddedBoundingBox(
    const G1<T>& geom, const G2<T>& refGeom) const {
  auto bbox = util::geo::getBoundingBox(geom);

  if (_cfg.withinDist >= 0) {
    if (_cfg.euclideanDist && !_cfg.haversineApprox)
      return util::geo::pad(bbox, _cfg.withinDist * PREC / 2.0);

    auto a = (reinterpret_cast<const void*>(&geom) ==
                      reinterpret_cast<const void*>(&refGeom)
                  ? bbox
                  : util::geo::getBoundingBox(refGeom));

    // convert distanceUpperBound (meters) to maximum latitude padding (degrees)
    // we have to "pad" the box by -dy and dy because the distance path could
    // be within that padded box, and thus the distortions have to be computed
    // based on that path
    double dLat =
        _cfg.withinDist / util::geo::MIN_METERS_PER_LAT_RAD * util::geo::IRAD;
    auto upper = util::geo::webMercToLatLng<double>(
        0.0, a.getUpperRight().getY() * 1.0 / PREC);
    auto lower = util::geo::webMercToLatLng<double>(
        0.0, a.getLowerLeft().getY() * 1.0 / PREC);

    double scaleLatUp =
        cos(std::min(90.0 - util::geo::EPSILON, (upper.getY() + dLat)) *
            util::geo::RAD);
    double scaleLatLow =
        cos(std::max(-90.0 + util::geo::EPSILON, (lower.getY() - dLat)) *
            util::geo::RAD);
    double scaleFactor = std::min(scaleLatUp, scaleLatLow);

    double pad = (_cfg.withinDist / 2.0) / scaleFactor * PREC;

    double llx = bbox.getLowerLeft().getX();
    double lly = bbox.getLowerLeft().getY();
    double urx = bbox.getUpperRight().getX();
    double ury = bbox.getUpperRight().getY();

    double m = sj::boxids::WORLD_W / 2.0;

    // restrict padding to world extent
    T llxt = -m;
    T llyt = -m;
    T urxt = m;
    T uryt = m;

    if (llx - pad > -m) {
      llxt = llx - pad;
    }

    if (lly - pad > -m) {
      llyt = lly - pad;
    }

    if (urx + pad < m) {
      urxt = urx + pad;
    }

    if (ury + pad < m) {
      uryt = ury + pad;
    }

    return {{llxt, llyt}, {urxt, uryt}};
  }

  return bbox;
}

// _____________________________________________________________________________
size_t GeometryStore::foldString(const std::string& s) {
  size_t ret = 0;
  for (size_t i = 0; i < std::min((size_t)7, s.size()); i++) {
    size_t tmp = static_cast<unsigned char>(s[i]);
    ret |= tmp << (i * 8);
  }

  // highest byte stores the length
  ret |= (s.size() << 56);

  return ret;
}
// _____________________________________________________________________________
template <typename W>
std::shared_ptr<W> GeometryStore::get(const sj::TypedGeometryStore<W>& store,
                                      size_t off) {
  if (store.inMemory()) return store.getFromMem(off);
  return std::make_shared<W>(store.getFrom(off, 0).second);
}
