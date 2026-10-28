// Copyright 2023, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>

#include <fstream>
#include <iostream>
#include <vector>

#include "GeometryStore.h"
#include "util/geo/Geo.h"

const static size_t MAX_MEM_CACHE_SIZE = 3 * 1024 * 1024 * 20l;

// ____________________________________________________________________________
template <>
std::pair<size_t, sj::SimpleLine> sj::GeometryStore<sj::SimpleLine>::getFrom(
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
std::pair<size_t, sj::MultiGeom> sj::GeometryStore<sj::MultiGeom>::getFrom(
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
std::pair<size_t, sj::Point> sj::GeometryStore<sj::Point>::getFrom(
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
std::pair<size_t, sj::SimpleArea> sj::GeometryStore<sj::SimpleArea>::getFrom(
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
std::pair<size_t, sj::Line> sj::GeometryStore<sj::Line>::getFrom(
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
std::pair<size_t, sj::Area> sj::GeometryStore<sj::Area>::getFrom(
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
size_t sj::GeometryStore<sj::Point>::writeTo(const sj::Point& val,
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
size_t sj::GeometryStore<sj::MultiGeom>::writeTo(const sj::MultiGeom& val,
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
size_t sj::GeometryStore<W>::add(const std::string& raw) {
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
size_t sj::GeometryStore<sj::SimpleArea>::writeTo(const sj::SimpleArea& val,
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
size_t sj::GeometryStore<sj::SimpleLine>::writeTo(const sj::SimpleLine& val,
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
size_t sj::GeometryStore<sj::Line>::writeTo(const sj::Line& val,
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
size_t sj::GeometryStore<sj::Area>::writeTo(const sj::Area& val,
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
void sj::GeometryStore<W>::flush() {
  if (_geomsF.is_open()) {
    _geomsF.flush();
    _geomsF.close();
  }
}

// ____________________________________________________________________________
template <typename W>
size_t sj::GeometryStore<W>::readPoly(std::istream& str,
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
size_t sj::GeometryStore<W>::writePoly(const util::geo::I32XSortedPolygon& geom,
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
size_t sj::GeometryStore<W>::readLine(std::istream& str,
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
size_t sj::GeometryStore<W>::writeLine(const util::geo::I32XSortedLine& geom,
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
std::string sj::GeometryStore<sj::Area>::getFName() const {
  return util::getTmpFName(_dir, _tmpPrefix, "areas");
}

// ____________________________________________________________________________
template <>
std::string sj::GeometryStore<sj::Line>::getFName() const {
  return util::getTmpFName(_dir, _tmpPrefix, "lines");
}

// ____________________________________________________________________________
template <>
std::string sj::GeometryStore<sj::Point>::getFName() const {
  return util::getTmpFName(_dir, _tmpPrefix, "points");
}

// ____________________________________________________________________________
template <>
std::string sj::GeometryStore<sj::SimpleLine>::getFName() const {
  return util::getTmpFName(_dir, _tmpPrefix, "simplelines");
}

// ____________________________________________________________________________
template <>
std::string sj::GeometryStore<sj::SimpleArea>::getFName() const {
  return util::getTmpFName(_dir, ".spatialjoin", "simpleareas");
}

// ____________________________________________________________________________
template <>
std::string sj::GeometryStore<sj::MultiGeom>::getFName() const {
  return util::getTmpFName(_dir, _tmpPrefix, "multis");
}

// ____________________________________________________________________________
template <>
sj::GeomType sj::GeometryStore<sj::Area>::geomType() {
  return sj::POLYGON;
}

// ____________________________________________________________________________
template <>
sj::GeomType sj::GeometryStore<sj::Line>::geomType() {
  return sj::LINE;
}

// ____________________________________________________________________________
template <>
sj::GeomType sj::GeometryStore<sj::Point>::geomType() {
  return sj::POINT;
}

// ____________________________________________________________________________
template <>
sj::GeomType sj::GeometryStore<sj::SimpleLine>::geomType() {
  return sj::SIMPLE_LINE;
}

// ____________________________________________________________________________
template <>
sj::GeomType sj::GeometryStore<sj::SimpleArea>::geomType() {
  return sj::SIMPLE_POLYGON;
}

// ____________________________________________________________________________
template <>
sj::GeomType sj::GeometryStore<sj::MultiGeom>::geomType() {
  // multi geometries are swept as special POINT events
  return sj::POINT;
}

// ____________________________________________________________________________
template class sj::GeometryStore<sj::Area>;
template class sj::GeometryStore<sj::Line>;
template class sj::GeometryStore<sj::SimpleLine>;
template class sj::GeometryStore<sj::Point>;
template class sj::GeometryStore<sj::SimpleArea>;
template class sj::GeometryStore<sj::MultiGeom>;
