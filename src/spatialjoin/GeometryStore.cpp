// Copyright 2023, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>

#include "GeometryStore.h"

#include <errno.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <unordered_set>
#include <vector>

#include "BoxIds.h"
#include "util/Misc.h"
#include "util/geo/Geo.h"

const static size_t MAX_MEM_CACHE_SIZE = 3 * 1024 * 1024 * 20l;

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
using util::geo::I32Collection;
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


// ____________________________________________________________________________
template <>
std::pair<size_t, sj::SimpleLine>
sj::TypedGeometryStore<sj::SimpleLine>::getFrom(size_t off,
                                                std::istream& str) const {
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

  size_t estSize = len * sizeof(char);

  // sweep events
  uint32_t size;
  str.read(reinterpret_cast<char*>(&size), sizeof(uint32_t));
  ret.events.resize(size);

  if (size) {
    str.read(reinterpret_cast<char*>(&ret.events[0]), size * sizeof(BoxVal));
  }

  estSize += size * sizeof(BoxVal);

  return {estSize, ret};
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
std::pair<size_t, sj::SimpleArea>
sj::TypedGeometryStore<sj::SimpleArea>::getFrom(size_t off,
                                                std::istream& str) const {
  sj::SimpleArea ret;

  str.seekg(off);

  // id
  uint16_t len;
  str.read(reinterpret_cast<char*>(&len), sizeof(uint16_t));
  ret.id.resize(len);

  if (len) {
    str.read(reinterpret_cast<char*>(&ret.id[0]), len * sizeof(char));
  }

  // geom
  uint32_t size;
  str.read(reinterpret_cast<char*>(&size), sizeof(uint32_t));
  ret.geom.resize(size);

  if (size) {
    str.read(reinterpret_cast<char*>(&ret.geom[0]),
             size * sizeof(util::geo::I32Point));
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

  // geom
  estSize += readLine(str, ret.geom);

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

  // geom
  estSize += readPoly(str, ret.geom);

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

  // sweep events
  if (val.events.size() > std::numeric_limits<uint32_t>::max()) {
    throw std::out_of_range(
        "Multi geometry has more than " +
        std::to_string(std::numeric_limits<uint32_t>::max()) +
        " sweep events");
  }
  uint32_t size = val.events.size();
  str.write(reinterpret_cast<const char*>(&size), sizeof(uint32_t));
  ret += sizeof(uint32_t);

  str.write(reinterpret_cast<const char*>(val.events.data()),
            val.events.size() * sizeof(BoxVal));
  ret += val.events.size() * sizeof(BoxVal);

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
      openFile();

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
size_t sj::TypedGeometryStore<sj::SimpleArea>::writeTo(
    const sj::SimpleArea& val, std::ostream& str) const {
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

  // geom
  uint32_t len = val.geom.size();
  str.write(reinterpret_cast<const char*>(&len), sizeof(uint32_t));
  ret += sizeof(uint32_t);

  if (len) {
    str.write(reinterpret_cast<const char*>(&val.geom[0]),
              len * sizeof(util::geo::I32Point));
    ret += sizeof(util::geo::I32Point) * len;
  }

  return ret;
}

// ____________________________________________________________________________

template <>
size_t sj::TypedGeometryStore<sj::SimpleLine>::writeTo(
    const sj::SimpleLine& val, std::ostream& str) const {
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

  // geoms
  ret += writeLine(val.geom, str);

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

  // geoms
  ret += writePoly(val.geom, str);

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
std::pair<size_t, sj::GeomId> sj::TypedGeometryStore<W>::getIdFrom(
    size_t off, std::istream& str) const {
  // all geometry types are serialized starting with the gid, followed by the
  // sub id for those types which have one
  constexpr bool hasSubId = std::is_same<W, sj::Area>::value ||
                            std::is_same<W, sj::Line>::value ||
                            std::is_same<W, sj::Point>::value;

  sj::GeomId ret{"", 0};

  str.seekg(off);

  // id
  uint16_t len;
  str.read(reinterpret_cast<char*>(&len), sizeof(uint16_t));
  ret.id.resize(len);

  if (len) {
    str.read(reinterpret_cast<char*>(&ret.id[0]), len * sizeof(char));
  }

  // sub id
  if (hasSubId) str.read(reinterpret_cast<char*>(&ret.subId), sizeof(size_t));

  return {len * sizeof(char) + sizeof(size_t), ret};
}

// ____________________________________________________________________________
template <typename W>
std::pair<size_t, sj::GeomId> sj::TypedGeometryStore<W>::getId(
    size_t off, size_t tid) const {
  if (_inMemory) {
    const auto& val = getFromMem(off);
    return {val->id.size() * sizeof(char) + sizeof(size_t),
            {val->id, val->subId}};
  }
  return getIdFrom(untypedId(off), _geomsFReads[tid]);
}

// ____________________________________________________________________________
template <>
std::pair<size_t, sj::GeomId> sj::TypedGeometryStore<sj::SimpleArea>::getId(
    size_t off, size_t tid) const {
  // no sub id
  if (_inMemory) {
    const auto& val = getFromMem(off);
    return {val->id.size() * sizeof(char) + sizeof(size_t), {val->id, 0}};
  }
  return getIdFrom(untypedId(off), _geomsFReads[tid]);
}

// ____________________________________________________________________________
template <>
std::pair<size_t, sj::GeomId> sj::TypedGeometryStore<sj::SimpleLine>::getId(
    size_t off, size_t tid) const {
  // no sub id
  if (_inMemory) {
    const auto& val = getFromMem(off);
    return {val->id.size() * sizeof(char) + sizeof(size_t), {val->id, 0}};
  }
  return getIdFrom(untypedId(off), _geomsFReads[tid]);
}

// ____________________________________________________________________________
template <>
std::pair<size_t, sj::GeomId> sj::TypedGeometryStore<sj::MultiGeom>::getId(
    size_t off, size_t tid) const {
  // no sub id
  if (_inMemory) {
    const auto& val = getFromMem(off);
    return {val->id.size() * sizeof(char) + sizeof(size_t), {val->id, 0}};
  }
  return getIdFrom(untypedId(off), _geomsFReads[tid]);
}

// ____________________________________________________________________________
template <typename W>
sj::TypedGeometryStore<W>::TypedGeometryStore(size_t numthreads,
                                              const std::string& path)
    : _numThreads(numthreads) {
  _inMemory = false;

  struct stat st;
  if (stat(path.c_str(), &st) != 0) {
    throw std::runtime_error("Could not open geometry store file " + path);
  }
  _geomsOffset = st.st_size;

  _geomsFReads.resize(numthreads);
  for (auto& read : _geomsFReads) {
    read.open(path, std::ios::in | std::ios::binary);
    if (!read.is_open()) {
      throw std::runtime_error("Could not open geometry store file " + path);
    }
  }
}

// ____________________________________________________________________________
template <typename W>
void sj::TypedGeometryStore<W>::flush() {
  if (_geomsF.is_open()) {
    _geomsF.flush();
    _geomsF.close();
  }

  if (_inMemory && _persistOnDisk) {
    const auto fName = persistFName(_dir, _prefix);
    std::ofstream out(fName,
                      std::ios::out | std::ios::binary | std::ios::trunc);

      for (const auto& val : _memStore) {
        writeTo(*val.second.get(), out);
      }
    out.close();

    if (!out) {
      throw std::runtime_error("Could not write geometry store file " + fName);
    }
  }
}

// ____________________________________________________________________________
template <typename W>
std::string sj::TypedGeometryStore<W>::getFName() const {
  return util::getTmpFName(_dir, _prefix, typeName());
}

// ____________________________________________________________________________
template <typename W>
std::string sj::TypedGeometryStore<W>::persistFName(
    const std::string& dir, const std::string& prefix) {
  std::string d = dir == "<tmp>" ? util::getTmpDir() : dir;
  if (d.size() && d.back() != '/') d += "/";
  return d + prefix + typeName();
}

// ____________________________________________________________________________
template <typename W>
size_t sj::TypedGeometryStore<W>::readPoly(
    std::istream& str, util::geo::I32XSortedPolygon& ret) const {
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
size_t sj::TypedGeometryStore<W>::writePoly(
    const util::geo::I32XSortedPolygon& geom, std::ostream& str) {
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
size_t sj::TypedGeometryStore<W>::readLine(
    std::istream& str, util::geo::I32XSortedLine& ret) const {
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
size_t sj::TypedGeometryStore<W>::writeLine(
    const util::geo::I32XSortedLine& geom, std::ostream& str) {
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
std::string sj::TypedGeometryStore<sj::Area>::typeName() {
  return "areas";
}

// ____________________________________________________________________________
template <>
std::string sj::TypedGeometryStore<sj::Line>::typeName() {
  return "lines";
}

// ____________________________________________________________________________
template <>
std::string sj::TypedGeometryStore<sj::Point>::typeName() {
  return "points";
}

// ____________________________________________________________________________
template <>
std::string sj::TypedGeometryStore<sj::SimpleLine>::typeName() {
  return "simplelines";
}

// ____________________________________________________________________________
template <>
std::string sj::TypedGeometryStore<sj::SimpleArea>::typeName() {
  return "simpleareas";
}

// ____________________________________________________________________________
template <>
sj::MultiGeom sj::TypedGeometryStore<sj::MultiGeom>::getMulti(
    size_t off) const {
  if (_inMemory) return *getFromMem(off);

  // only ever read by a single thread, so we can always use read stream 0
  return getFrom(off, 0).second;
}

// ____________________________________________________________________________
template <>
std::string sj::TypedGeometryStore<sj::MultiGeom>::typeName() {
  return "multis";
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
  return sj::SIMPLE_LINE_RISING;
}

// ____________________________________________________________________________
template <>
sj::GeomType sj::TypedGeometryStore<sj::SimpleArea>::geomType() {
  return sj::SIMPLE_POLYGON;
}

// ____________________________________________________________________________
template <>
sj::GeomType sj::TypedGeometryStore<sj::MultiGeom>::geomType() {
  return sj::MULTI;
}

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32MultiPolygon& a, const std::string& gid,
                          WriteBatch& batch) const {
  // a subid of 0 means "single polygon"
  if (a.size() < 2) return add(a, gid, 0, batch);

  WriteBatch members;
  auto box = add(a, gid, 1, members);
  addMulti(std::move(members), box, gid, batch);
  return box;
}

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32MultiLine& a, const std::string& gid,
                          WriteBatch& batch) const {
  // a subid of 0 means "single line"
  if (a.size() < 2) return add(a, gid, 0, batch);

  WriteBatch members;
  auto box = add(a, gid, 1, members);
  addMulti(std::move(members), box, gid, batch);
  return box;
}

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32MultiPoint& a, const std::string& gid,
                          WriteBatch& batch) const {
  // a subid of 0 means "single point"
  if (a.size() < 2) return add(a, gid, 0, batch);

  WriteBatch members;
  auto box = add(a, gid, 1, members);
  addMulti(std::move(members), box, gid, batch);
  return box;
}

// _____________________________________________________________________________
static size_t numMembers(const I32Collection& c) {
  size_t ret = 0;
  for (const auto& a : c) {
    if (a.getType() == 0 || a.getType() == 1 || a.getType() == 2) ret++;
    if (a.getType() == 3) ret += a.getMultiLine().size();
    if (a.getType() == 4) ret += a.getMultiPolygon().size();
    if (a.getType() == 5) ret += numMembers(a.getCollection());
    if (a.getType() == 6) ret += a.getMultiPoint().size();
  }
  return ret;
}

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32Collection& col, const std::string& gid,
                          WriteBatch& batch) const {
  size_t numGeoms = numMembers(col);

  // a subid of 0 means "single geometry"
  size_t subId = numGeoms > 1 ? 1 : 0;

  // the members of a multi collection are kept together
  WriteBatch members;
  auto colBox = addMembers(col, gid, subId, numGeoms > 1 ? members : batch);

  if (numGeoms > 1) addMulti(std::move(members), colBox, gid, batch);

  return colBox;
}

// _____________________________________________________________________________
I32Box GeometryStore::addMembers(const I32Collection& col,
                                 const std::string& gid, size_t& subId,
                                 WriteBatch& target) const {
  I32Box colBox;

  for (const auto& a : col) {
    if (a.getType() == 0)
      colBox = util::geo::extendBox(add(a.getPoint(), gid, subId, target),
                                    colBox);
    if (a.getType() == 1)
      colBox =
          util::geo::extendBox(add(a.getLine(), gid, subId, target), colBox);
    if (a.getType() == 2)
      colBox = util::geo::extendBox(add(a.getPolygon(), gid, subId, target),
                                    colBox);
    if (a.getType() == 3)
      colBox = util::geo::extendBox(add(a.getMultiLine(), gid, subId, target),
                                    colBox);
    if (a.getType() == 4)
      colBox = util::geo::extendBox(
          add(a.getMultiPolygon(), gid, subId, target), colBox);
    if (a.getType() == 5)
      // nested collections are flattened, their members take the next sub IDs
      colBox = util::geo::extendBox(
          addMembers(a.getCollection(), gid, subId, target), colBox);
    if (a.getType() == 6)
      colBox = util::geo::extendBox(
          add(a.getMultiPoint(), gid, subId, target), colBox);

    // the members of a multi element each take their own sub ID
    if (a.getType() == 3)
      subId += a.getMultiLine().size();
    else if (a.getType() == 4)
      subId += a.getMultiPolygon().size();
    else if (a.getType() == 6)
      subId += a.getMultiPoint().size();
    else if (a.getType() != 5)  // already advanced by addMembers()
      subId++;
  }

  return colBox;
}

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32MultiPolygon& a, const std::string& gid,
                          size_t subId, WriteBatch& batch) const {
  I32Box ret;
  for (const auto& poly : a) {
    auto box = add(poly, gid, subId, batch);
    if (box.isNull()) continue;
    ret = util::geo::extendBox(box, ret);
    subId++;
  }

  return ret;
}

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32MultiLine& a, const std::string& gid,
                          size_t subId, WriteBatch& batch) const {
  I32Box ret;
  for (const auto& line : a) {
    if (line.size() < 2) continue;
    auto box = add(line, gid, subId, batch);
    if (box.isNull()) continue;
    ret = util::geo::extendBox(box, ret);
    subId++;
  }

  return ret;
}

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32MultiPoint& a, const std::string& gid,
                          size_t subid, WriteBatch& batch) const {
  I32Box ret;
  size_t newId = subid;
  for (const auto& point : a) {
    auto box = add(point, gid, newId, batch);
    if (box.isNull()) continue;
    ret = util::geo::extendBox(box, ret);
    newId++;
  }

  return ret;
}

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32Polygon& poly, const std::string& gid,
                          WriteBatch& batch) const {
  return add(poly, gid, 0, batch);
}

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32Polygon& poly, const std::string& gid,
                          size_t subid, WriteBatch& batch) const {
  if (poly.getOuter().size() < 2) return {};

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
    box45 = util::geo::getBoundingBox(polyR);
  }

  cur.subid = subid;
  cur.gid = gid;
  cur.meta.bbox = rawBox;
  cur.meta.box45 = box45;

  if (poly.getInners().size() == 0 && subid == 0 &&
      gid.size() <= FOLD_MAX_LEN &&
      poly.getOuter().size() == densePoly.getOuter().size() &&
      (!_cfg.useBoxIds || boxIds.front().first == 1) &&
      area(rawBox) == areaSize) {
    cur.type = FOLDED_BOX_POLYGON;
    cur.meta.numAnchors = poly.size();
    batch.foldedBoxAreas.emplace_back(cur);
  } else if (poly.getInners().size() == 0 && poly.getOuter().size() < 10 &&
             poly.getOuter().size() == densePoly.getOuter().size() &&
             subid == 0 && (!_cfg.useBoxIds || boxIds.front().first == 1)) {
    std::stringstream str;
    _simpleAreaStore.writeTo({gid, poly.getOuter()}, str);
    cur.raw = str.str();

    auto rightPoint = poly.getOuter().front();

    for (const auto& p : poly.getOuter()) {
      if (p.getX() > rightPoint.getX()) rightPoint = p;
    }

    cur.type = SIMPLE_POLYGON;
    cur.rightPoint = rightPoint;
    cur.meta.numAnchors = poly.size();
    batch.simpleAreas.emplace_back(cur);
  } else {
    // careful, assign this before move below
    auto rightPoint = spoly.getOuter().rawRing().back().p;

    std::stringstream str;
    _areaStore.writeTo({gid, subid, std::move(spoly), boxIds}, str);
    ;

    cur.raw = str.str();

    cur.type = POLYGON;
    cur.rightPoint = rightPoint;
    cur.meta.numAnchors = polySize;
    batch.areas.emplace_back(cur);
  }

  return box;
}

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32Line& line, const std::string& gid,
                          WriteBatch& batch) const {
  return add(line, gid, 0, batch);
}

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32Line& line, const std::string& gid,
                          size_t subid, WriteBatch& batch) const {
  if (line.size() < 2) return {};

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

  size_t lineSize = line.size();

  I32Box box45;
  if (_cfg.useDiagBox) {
    auto lineR = util::geo::rotateSinCos(line, sin45, cos45, I32Point(0, 0));
    box45 = util::geo::getBoundingBox(lineR);
  }

  cur.subid = subid;
  cur.gid = gid;
  cur.meta.bbox = rawBox;
  cur.meta.box45 = box45;

  if (line.size() == 2 && denseLine.size() == line.size() &&
      (!_cfg.useBoxIds || boxIds.front().first == 1) && subid == 0) {
    // simple line

    const auto& right =
        line.front().getX() < line.back().getX() ? line.back() : line.front();
    const auto& left =
        line.front().getX() < line.back().getX() ? line.front() : line.back();
    const bool rising = right.getY() >= left.getY();

    cur.type = rising ? SIMPLE_LINE_RISING : SIMPLE_LINE_FALLING;
    cur.meta.numAnchors = lineSize;

    // check if we can fold the gid into the offset id, because the gid is all
    // we store in the cache for points
    if (subid == 0 && gid.size() <= FOLD_MAX_LEN) {
      cur.type =
          rising ? FOLDED_SIMPLE_LINE_RISING : FOLDED_SIMPLE_LINE_FALLING;
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

    std::stringstream str;
    _lineStore.writeTo({gid, subid, std::move(sline), boxIds}, str);
    cur.raw = str.str();

    cur.type = LINE;
    cur.rightPoint = rightPoint;
    cur.meta.numAnchors = lineSize;
    batch.lines.emplace_back(cur);
  }

  return box;
}

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32Point& point, const std::string& gid,
                          WriteBatch& batch) const {
  return add(point, gid, 0, batch);
}

// _____________________________________________________________________________
I32Box GeometryStore::add(const I32Point& point, const std::string& gid,
                          size_t subid, WriteBatch& batch) const {
  if (_cfg.de9imFilter.maxExteriorDim() < 2) return {};

  WriteCand cur;

  const auto& rawBox = getBoundingBox(point);
  const auto& box = getPaddedBoundingBox(rawBox);

  cur.subid = subid;

  if (!util::geo::intersects(box, _filterBox)) return {};

  auto pointR = util::geo::rotateSinCos(point, sin45, cos45, I32Point(0, 0));
  cur.meta.bbox = rawBox;
  cur.type = POINT;
  cur.meta.box45 = util::geo::getBoundingBox(pointR);
  cur.meta.numAnchors = 1;

  cur.gid = gid;

  // check if we can fold the gid into the offset id, because the gid is all
  // we store in the cache for points
  if (subid == 0 && gid.size() <= FOLD_MAX_LEN) {
    cur.type = FOLDED_POINT;
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
void GeometryStore::addMulti(WriteBatch&& members, const I32Box& box,
                             const std::string& gid, WriteBatch& batch) const {
  if (members.size() == 0) return;

  batch.multiGeoms.push_back({gid, 0, box, std::move(members)});
}

// _____________________________________________________________________________
void GeometryStore::addBatch(WriteBatch& cands, size_t t,
                             const PreparedGeomMetaCb& metaCb) {
  // we go over each class of candidates. Note that in .multiGeoms, multi-geoms
  // are kept together, and each class (POINT, POLYGON...) may appear there,
  // too. The id is written into the candidate and will be available at the
  // end of this method for further processing, see below

  {
    for (auto& cand : cands.foldedPoints) {
      cand.meta.id = typedId(foldString(cand.gid), cand.type);
    }
    // note: folded geoms can never be multi
  }

  {
    std::unique_lock<std::mutex> lock(_pointGeomCacheWriteMtx);
    for (auto& cand : cands.points) {
      cand.meta.id = _pointStore.add(cand.raw);
    }
    for (auto& multi : cands.multiGeoms) {
      for (auto& cand : multi.members.points) {
        cand.meta.id = _pointStore.add(cand.raw);
      }
    }
  }

  {
    std::unique_lock<std::mutex> lock(_lineGeomCacheWriteMtx);
    for (auto& cand : cands.lines) {
      cand.meta.id = _lineStore.add(cand.raw);
    }
    for (auto& multi : cands.multiGeoms) {
      for (auto& cand : multi.members.lines) {
        cand.meta.id = _lineStore.add(cand.raw);
      }
    }
  }

  {
    std::unique_lock<std::mutex> lock(_simpleLineGeomCacheWriteMtx);
    for (auto& cand : cands.simpleLines) {
      cand.meta.id =
          typedId(untypedId(_simpleLineStore.add(cand.raw)), cand.type);
    }
    for (auto& multi : cands.multiGeoms) {
      for (auto& cand : multi.members.simpleLines) {
        cand.meta.id = typedId(untypedId(_simpleLineStore.add(cand.raw)),
                               cand.type);
      }
    }
  }

  {
    for (auto& cand : cands.foldedSimpleLines) {
      cand.meta.id = typedId(foldString(cand.gid), cand.type);
    }
    // note: folded geoms can never be multi
  }

  {
    for (auto& cand : cands.foldedBoxAreas) {
      cand.meta.id = typedId(foldString(cand.gid), cand.type);
    }
    // note: folded geoms can never be multi
  }

  {
    std::unique_lock<std::mutex> lock(_simpleAreaGeomCacheWriteMtx);
    for (auto& cand : cands.simpleAreas) {
      cand.meta.id = _simpleAreaStore.add(cand.raw);
    }
    for (auto& multi : cands.multiGeoms) {
      for (auto& cand : multi.members.simpleAreas) {
        cand.meta.id = _simpleAreaStore.add(cand.raw);
      }
    }
  }

  {
    std::unique_lock<std::mutex> lock(_areaGeomCacheWriteMtx);
    for (auto& cand : cands.areas) {
      cand.meta.id = _areaStore.add(cand.raw);
    }
    for (auto& multi : cands.multiGeoms) {
      for (auto& cand : multi.members.areas) {
        cand.meta.id = _areaStore.add(cand.raw);
      }
    }
  }

  // the multi geometries, with the sweep events of all their members, can only
  // be written once the IDs of their members are known
  std::vector<std::string> multiRaws;
  multiRaws.reserve(cands.multiGeoms.size());

  for (const auto& multi : cands.multiGeoms) {
    MultiGeom multiGeom{multi.gid, {}};
    multiGeom.events.reserve(2 * multi.members.size());

    for (const auto* members :
         {&multi.members.points, &multi.members.simpleLines,
          &multi.members.lines, &multi.members.simpleAreas,
          &multi.members.areas}) {
      for (const auto& cand : *members) {
        const auto& events = getSweepEvents(cand.meta, 0, cand.rightPoint);
        multiGeom.events.push_back(events.first);
        multiGeom.events.push_back(events.second);
      }
    }

    // sort the events like a SweepEventList
    std::sort(multiGeom.events.begin(), multiGeom.events.end(),
              [](const BoxVal& a, const BoxVal& b) {
                return boxCmp(&a, &b) < 0;
              });

    std::stringstream str;
    _multiStore.writeTo(multiGeom, str);
    multiRaws.push_back(str.str());
  }

  {
    std::unique_lock<std::mutex> lock(_multiGeomCacheWriteMtx);
    for (size_t i = 0; i < cands.multiGeoms.size(); i++) {
      cands.multiGeoms[i].id = _multiStore.add(multiRaws[i]);
    }
    _numMultis += cands.multiGeoms.size();
  }

  std::vector<PreparedGeomMeta> multiMetas;
  multiMetas.reserve(cands.multiGeoms.size());

  for (const auto& multi : cands.multiGeoms) {
    PreparedGeomMeta multiMeta{multi.id, {}, {}, 0};

    for (const auto* members :
         {&multi.members.points, &multi.members.simpleLines,
          &multi.members.lines, &multi.members.simpleAreas,
          &multi.members.areas}) {
      for (const auto& cand : *members) {
        multiMeta.bbox = util::geo::extendBox(cand.meta.bbox, multiMeta.bbox);
        multiMeta.box45 =
            util::geo::extendBox(cand.meta.box45, multiMeta.box45);
        multiMeta.numAnchors += cand.meta.numAnchors;
      }
    }

    multiMetas.push_back(multiMeta);
  }

  for (const auto* list :
       {&cands.foldedPoints, &cands.points, &cands.foldedSimpleLines,
        &cands.foldedBoxAreas, &cands.simpleLines, &cands.lines,
        &cands.simpleAreas, &cands.areas}) {
    for (const auto& cand : *list) {
      metaCb(t, cand.gid, cand.meta);
      size_t n = ++_numReported;
      if (n % 1000000 == 0) log("@ " + std::to_string(n));
    }
  }

  // multiMetas[i] belongs to cands.multiGeoms[i]
  for (size_t i = 0; i < multiMetas.size(); i++) {
    metaCb(t, cands.multiGeoms[i].gid, multiMetas[i]);
    size_t n = ++_numReported;
    if (n % 1000000 == 0) log("@ " + std::to_string(n));
  }
}

// _____________________________________________________________________________
void GeometryStore::flush() {
  log(std::to_string(_numMultis) + " multi geometries");

  _pointStore.flush();
  _areaStore.flush();
  _simpleAreaStore.flush();
  _lineStore.flush();
  _simpleLineStore.flush();
  _multiStore.flush();
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
  return padBox(util::geo::getBoundingBox(geom),
                util::geo::getBoundingBox(refGeom), _cfg);
}

// _____________________________________________________________________________
size_t GeometryStore::foldString(const std::string& s) {
  size_t ret = 0;
  for (size_t i = 0; i < std::min(FOLD_MAX_LEN, s.size()); i++) {
    size_t tmp = static_cast<unsigned char>(s[i]);
    ret |= tmp << (i * 8);
  }

  // the byte below the type bits stores the length
  ret |= (s.size() << FOLD_LEN_SHIFT);

  return ret;
}
// _____________________________________________________________________________
template <typename W>
std::shared_ptr<W> GeometryStore::get(const sj::TypedGeometryStore<W>& store,
                                      size_t off) {
  if (store.inMemory()) return store.getFromMem(off);
  return std::make_shared<W>(store.getFrom(off, 0).second);
}

template class sj::TypedGeometryStore<sj::Area>;
template class sj::TypedGeometryStore<sj::Line>;
template class sj::TypedGeometryStore<sj::SimpleLine>;
template class sj::TypedGeometryStore<sj::Point>;
template class sj::TypedGeometryStore<sj::SimpleArea>;
template class sj::TypedGeometryStore<sj::MultiGeom>;
