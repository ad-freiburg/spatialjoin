// Copyright 2023, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>

#include <fstream>
#include <iostream>
#include <vector>

#include "GeometryCache.h"
#include "util/geo/Geo.h"

// ____________________________________________________________________________
template <typename W>
std::shared_ptr<W> sj::GeometryCache<W>::get(size_t off, size_t tid) const {
  _accessCount[tid]++;

  // check if value is in cache
  auto it = _idMap[tid].find(off);
  if (it == _idMap[tid].end()) {
    // if not, load, cache and return
    const auto& ret = getFromDisk(off, tid);
    return cache(off, ret, tid);
  }

  // if in cache, move to front of list and return
  // splice only changes pointers in the linked list, no copying here
  _vals[tid].splice(_vals[tid].begin(), _vals[tid], it->second);
  return it->second->second;
}

// ____________________________________________________________________________
template <typename W>
std::shared_ptr<W> sj::GeometryCache<W>::cache(size_t off, const W& val,
                                               size_t tid) const {
  // push value to front
  _vals[tid].push_front({off, std::make_shared<W>(val)});

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
sj::SimpleLine sj::GeometryCache<sj::SimpleLine>::getFromDisk(
    size_t off, size_t tid) const {
  _diskAccessCount[tid]++;
  sj::SimpleLine ret;

  _geomsFReads[tid].seekg(off);

  // geom
  _geomsFReads[tid].read(reinterpret_cast<char*>(&ret.a),
                         sizeof(util::geo::I32Point));
  _geomsFReads[tid].read(reinterpret_cast<char*>(&ret.b),
                         sizeof(util::geo::I32Point));

  // id
  size_t len;
  _geomsFReads[tid].read(reinterpret_cast<char*>(&len), sizeof(size_t));
  ret.id.resize(len);
  _geomsFReads[tid].read(reinterpret_cast<char*>(&ret.id[0]),
                         len * sizeof(char));

  return ret;
}

// ____________________________________________________________________________
template <>
sj::Point sj::GeometryCache<sj::Point>::getFromDisk(size_t off,
                                                    size_t tid) const {
  _diskAccessCount[tid]++;
  sj::Point ret;

  _geomsFReads[tid].seekg(off);

  // id
  size_t len;
  _geomsFReads[tid].read(reinterpret_cast<char*>(&len), sizeof(size_t));
  ret.id.resize(len);
  _geomsFReads[tid].read(reinterpret_cast<char*>(&ret.id[0]),
                         len * sizeof(char));

  // sub id
  _geomsFReads[tid].read(reinterpret_cast<char*>(&ret.subId), sizeof(uint16_t));

  return ret;
}

// ____________________________________________________________________________
template <>
sj::Line sj::GeometryCache<sj::Line>::getFromDisk(size_t off,
                                                  size_t tid) const {
  _diskAccessCount[tid]++;
  sj::Line ret;

  _geomsFReads[tid].seekg(off);

  // geom
  readLine(_geomsFReads[tid], ret.geom);

  // envelope
  _geomsFReads[tid].read(reinterpret_cast<char*>(&ret.box),
                         sizeof(util::geo::I32Box));

  // id
  size_t len;
  _geomsFReads[tid].read(reinterpret_cast<char*>(&len), sizeof(size_t));
  ret.id.resize(len);
  _geomsFReads[tid].read(reinterpret_cast<char*>(&ret.id[0]),
                         len * sizeof(char));

  // sub id
  _geomsFReads[tid].read(reinterpret_cast<char*>(&ret.subId), sizeof(uint16_t));

  // length
  _geomsFReads[tid].read(reinterpret_cast<char*>(&ret.length), sizeof(double));

  // boxIds
  uint32_t numBoxIds;
  _geomsFReads[tid].read(reinterpret_cast<char*>(&numBoxIds), sizeof(uint32_t));
  ret.boxIds.resize(numBoxIds);
  if (numBoxIds > 0) {
    _geomsFReads[tid].read(reinterpret_cast<char*>(&ret.boxIds[0]),
                           sizeof(sj::boxids::BoxId) * numBoxIds);
  }

  // cutouts
  uint32_t numCutouts;
  _geomsFReads[tid].read(reinterpret_cast<char*>(&numCutouts),
                         sizeof(uint32_t));

  for (size_t i = 0; i < numCutouts; i++) {
    int32_t boxid, cutout;
    _geomsFReads[tid].read(reinterpret_cast<char*>(&boxid), sizeof(int32_t));
    _geomsFReads[tid].read(reinterpret_cast<char*>(&cutout), sizeof(int32_t));
    ret.cutouts[boxid] = cutout;
  }

  // OBB
  readPoly(_geomsFReads[tid], ret.obb);

  return ret;
}

// ____________________________________________________________________________
template <>
sj::Area sj::GeometryCache<sj::Area>::getFromDisk(size_t off,
                                                  size_t tid) const {
  _diskAccessCount[tid]++;
  sj::Area ret;

  _geomsFReads[tid].seekg(off);

  // geom
  readPoly(_geomsFReads[tid], ret.geom);

  // TODO: careful, resize initializes entire vector!

  // envelopes
  _geomsFReads[tid].read(reinterpret_cast<char*>(&ret.box),
                         sizeof(util::geo::I32Box));

  // id
  size_t len;
  _geomsFReads[tid].read(reinterpret_cast<char*>(&len), sizeof(size_t));
  ret.id.resize(len);
  _geomsFReads[tid].read(reinterpret_cast<char*>(&ret.id[0]),
                         len * sizeof(char));

  // sub id
  _geomsFReads[tid].read(reinterpret_cast<char*>(&ret.subId), sizeof(uint16_t));

  // area
  _geomsFReads[tid].read(reinterpret_cast<char*>(&ret.area), sizeof(double));

  // outer area
  _geomsFReads[tid].read(reinterpret_cast<char*>(&ret.outerArea),
                         sizeof(double));

  // simplified inner
  // readPoly(_geomsFReads[tid], ret.inner);

  // simplified outer
  // readPoly(_geomsFReads[tid], ret.outer);

  // boxIds
  uint32_t numBoxIds;
  _geomsFReads[tid].read(reinterpret_cast<char*>(&numBoxIds), sizeof(uint32_t));
  ret.boxIds.resize(numBoxIds);
  if (numBoxIds > 0) {
    _geomsFReads[tid].read(reinterpret_cast<char*>(&ret.boxIds[0]),
                           sizeof(sj::boxids::BoxId) * numBoxIds);
  }

  // cutouts
  uint32_t numCutouts;
  _geomsFReads[tid].read(reinterpret_cast<char*>(&numCutouts),
                         sizeof(uint32_t));

  for (size_t i = 0; i < numCutouts; i++) {
    int32_t boxid, cutout;
    _geomsFReads[tid].read(reinterpret_cast<char*>(&boxid), sizeof(int32_t));
    _geomsFReads[tid].read(reinterpret_cast<char*>(&cutout), sizeof(int32_t));
    ret.cutouts[boxid] = cutout;
  }

  // OBB
  readPoly(_geomsFReads[tid], ret.obb);

  return ret;
}

// ____________________________________________________________________________
template <>
size_t sj::GeometryCache<sj::Point>::add(const sj::Point& val) {
  size_t ret = _geomsOffset;

  // id
  size_t s = val.id.size();
  _geomsF.write(reinterpret_cast<const char*>(&s), sizeof(size_t));
  _geomsOffset += sizeof(size_t);
  _geomsF.write(reinterpret_cast<const char*>(val.id.c_str()),
                val.id.size() * sizeof(char));
  _geomsOffset += sizeof(char) * val.id.size();

  // sub id
  _geomsF.write(reinterpret_cast<const char*>(&val.subId), sizeof(uint16_t));
  _geomsOffset += sizeof(uint16_t);

  return ret;
}
// ____________________________________________________________________________

template <>
size_t sj::GeometryCache<sj::SimpleLine>::add(const sj::SimpleLine& val) {
  size_t ret = _geomsOffset;

  // geoms
  _geomsF.write(reinterpret_cast<const char*>(&val.a),
                sizeof(util::geo::I32Point));
  _geomsF.write(reinterpret_cast<const char*>(&val.b),
                sizeof(util::geo::I32Point));
  _geomsOffset += sizeof(util::geo::I32Point) * 2;

  // id
  size_t s = val.id.size();
  _geomsF.write(reinterpret_cast<const char*>(&s), sizeof(size_t));
  _geomsOffset += sizeof(size_t);
  _geomsF.write(reinterpret_cast<const char*>(val.id.c_str()),
                val.id.size() * sizeof(char));
  _geomsOffset += sizeof(char) * val.id.size();

  return ret;
}

// ____________________________________________________________________________
template <>
size_t sj::GeometryCache<sj::Line>::add(const sj::Line& val) {
  size_t ret = _geomsOffset;

  // geoms
  writeLine(val.geom);

  // envelopes
  _geomsF.write(reinterpret_cast<const char*>(&val.box),
                sizeof(util::geo::I32Box));
  _geomsOffset += sizeof(util::geo::I32Box);

  // id
  size_t s = val.id.size();
  _geomsF.write(reinterpret_cast<const char*>(&s), sizeof(size_t));
  _geomsOffset += sizeof(size_t);
  _geomsF.write(reinterpret_cast<const char*>(val.id.c_str()),
                val.id.size() * sizeof(char));
  _geomsOffset += sizeof(char) * val.id.size();

  // sub id
  _geomsF.write(reinterpret_cast<const char*>(&val.subId), sizeof(uint16_t));
  _geomsOffset += sizeof(uint16_t);

  // length
  _geomsF.write(reinterpret_cast<const char*>(&val.length), sizeof(double));
  _geomsOffset += sizeof(double);

  // boxIds
  uint32_t size = val.boxIds.size();
  _geomsF.write(reinterpret_cast<const char*>(&size), sizeof(uint32_t));
  if (size > 0) {
    _geomsF.write(reinterpret_cast<const char*>(&val.boxIds[0]),
                  sizeof(sj::boxids::BoxId) * size);
  }

  _geomsOffset += sizeof(uint32_t) + sizeof(sj::boxids::BoxId) * size;

  // cutouts
  size = val.cutouts.size();
  _geomsF.write(reinterpret_cast<const char*>(&size), sizeof(uint32_t));
  _geomsOffset += sizeof(uint32_t);

  for (const auto& el : val.cutouts) {
    int32_t boxid = el.first;
    int32_t cutout = el.second;
    _geomsF.write(reinterpret_cast<const char*>(&boxid), sizeof(int32_t));
    _geomsF.write(reinterpret_cast<const char*>(&cutout), sizeof(int32_t));
    _geomsOffset += sizeof(int32_t) + sizeof(int32_t);
  }

  // OBB
  writePoly(val.obb);

  return ret;
}

// ____________________________________________________________________________
template <>
size_t sj::GeometryCache<sj::Area>::add(const sj::Area& val) {
  size_t ret = _geomsOffset;

  // geoms
  writePoly(val.geom);

  // envelopes
  _geomsF.write(reinterpret_cast<const char*>(&val.box),
                sizeof(util::geo::I32Box));
  _geomsOffset += sizeof(util::geo::I32Box);

  // id
  size_t s = val.id.size();
  _geomsF.write(reinterpret_cast<const char*>(&s), sizeof(size_t));
  _geomsOffset += sizeof(size_t);
  _geomsF.write(reinterpret_cast<const char*>(val.id.c_str()),
                val.id.size() * sizeof(char));
  _geomsOffset += sizeof(char) * val.id.size();

  // sub id
  _geomsF.write(reinterpret_cast<const char*>(&val.subId), sizeof(uint16_t));
  _geomsOffset += sizeof(uint16_t);

  // area
  _geomsF.write(reinterpret_cast<const char*>(&val.area), sizeof(double));
  _geomsOffset += sizeof(double);

  // outer area
  _geomsF.write(reinterpret_cast<const char*>(&val.outerArea), sizeof(double));
  _geomsOffset += sizeof(double);

  // innerGeom
  // writePoly(val.inner);

  // outerGeom
  // writePoly(val.outer);

  // boxIds
  uint32_t size = val.boxIds.size();
  _geomsF.write(reinterpret_cast<const char*>(&size), sizeof(uint32_t));
  if (size > 0) {
    _geomsF.write(reinterpret_cast<const char*>(&val.boxIds[0]),
                  sizeof(sj::boxids::BoxId) * size);
  }

  _geomsOffset += sizeof(uint32_t) + sizeof(sj::boxids::BoxId) * size;

  // cutouts
  size = val.cutouts.size();
  _geomsF.write(reinterpret_cast<const char*>(&size), sizeof(uint32_t));
  _geomsOffset += sizeof(uint32_t);

  for (const auto& el : val.cutouts) {
    int32_t boxid = el.first;
    int32_t cutout = el.second;
    _geomsF.write(reinterpret_cast<const char*>(&boxid), sizeof(int32_t));
    _geomsF.write(reinterpret_cast<const char*>(&cutout), sizeof(int32_t));
    _geomsOffset += sizeof(int32_t) + sizeof(int32_t);
  }

  // OBB
  writePoly(val.obb);

  return ret;
}

// ____________________________________________________________________________
template <typename W>
void sj::GeometryCache<W>::flush() {
  if (_geomsF.is_open()) {
    _geomsF.flush();
    _geomsF.close();
  }

  for (size_t i = 0; i < _geomsFReads.size(); i++) {
    _geomsFReads[i].open(getFName(), std::ios::in | std::ios::binary);
  }
}

// ____________________________________________________________________________
template <typename W>
void sj::GeometryCache<W>::readPoly(std::fstream& str,
                                    util::geo::I32XSortedPolygon& ret) const {
  double maxSegLen;
  str.read(reinterpret_cast<char*>(&maxSegLen), sizeof(double));
  ret.getOuter().setMaxSegLen(maxSegLen);

  uint32_t sizeOuter;
  str.read(reinterpret_cast<char*>(&sizeOuter), sizeof(uint32_t));

  // TODO: careful, resize initializes entire vector!
  ret.getOuter().rawRing().resize(sizeOuter);

  str.read(reinterpret_cast<char*>(&ret.getOuter().rawRing()[0]),
           sizeof(util::geo::XSortedTuple<int32_t>) * sizeOuter);

  uint32_t numInners;
  str.read(reinterpret_cast<char*>(&numInners), sizeof(uint32_t));

  double innerMaxSegLen;
  str.read(reinterpret_cast<char*>(&innerMaxSegLen), sizeof(size_t));

  ret.setInnerMaxSegLen(innerMaxSegLen);

  // TODO: careful, resize initializes entire vector!
  ret.getInners().resize(numInners);
  ret.getInnerBoxes().resize(numInners);
  ret.getInnerBoxIdx().resize(numInners);
  ret.getInnerAreas().resize(numInners);

  str.read(reinterpret_cast<char*>(&ret.getInnerBoxes()[0]),
           sizeof(util::geo::Box<int32_t>) * numInners);
  str.read(reinterpret_cast<char*>(&ret.getInnerBoxIdx()[0]),
           sizeof(std::pair<int32_t, size_t>) * numInners);
  str.read(reinterpret_cast<char*>(&ret.getInnerAreas()[0]),
           sizeof(double) * numInners);

  for (uint32_t j = 0; j < numInners; j++) {
    double maxSegLen;
    str.read(reinterpret_cast<char*>(&maxSegLen), sizeof(double));
    ret.getInners()[j].setMaxSegLen(maxSegLen);

    uint32_t sizeInner;
    str.read(reinterpret_cast<char*>(&sizeInner), sizeof(uint32_t));

    // TODO: careful, resize initializes entire vector!
    ret.getInners()[j].rawRing().resize(sizeInner);

    str.read(reinterpret_cast<char*>(&ret.getInners()[j].rawRing()[0]),
             sizeof(util::geo::XSortedTuple<int32_t>) * sizeInner);
  }
}

// ____________________________________________________________________________
template <typename W>
void sj::GeometryCache<W>::writePoly(const util::geo::I32XSortedPolygon& geom) {
  // geom, outer
  double maxSegLen = geom.getOuter().getMaxSegLen();
  _geomsF.write(reinterpret_cast<const char*>(&maxSegLen), sizeof(double));
  _geomsOffset += sizeof(double);

  uint32_t locSize = geom.getOuter().rawRing().size();
  _geomsF.write(reinterpret_cast<const char*>(&locSize), sizeof(uint32_t));
  if (locSize)
    _geomsF.write(reinterpret_cast<const char*>(&geom.getOuter().rawRing()[0]),
                  sizeof(util::geo::XSortedTuple<int32_t>) * locSize);
  _geomsOffset +=
      sizeof(uint32_t) + sizeof(util::geo::XSortedTuple<int32_t>) * locSize;

  // geom, inners
  locSize = geom.getInners().size();
  _geomsF.write(reinterpret_cast<const char*>(&locSize), sizeof(uint32_t));
  _geomsOffset += sizeof(uint32_t);

  // inner max box seg len
  double innerMaxSegLen = geom.getInnerMaxSegLen();
  _geomsF.write(reinterpret_cast<const char*>(&innerMaxSegLen), sizeof(double));
  _geomsOffset += sizeof(double);

  // inner boxes
  _geomsF.write(reinterpret_cast<const char*>(&geom.getInnerBoxes()[0]),
                sizeof(util::geo::Box<int32_t>) * locSize);
  _geomsOffset += sizeof(util::geo::Box<int32_t>) * locSize;

  // inner box idx
  _geomsF.write(reinterpret_cast<const char*>(&geom.getInnerBoxIdx()[0]),
                sizeof(std::pair<int32_t, size_t>) * locSize);
  _geomsOffset += sizeof(std::pair<int32_t, size_t>) * locSize;

  // inner areas
  _geomsF.write(reinterpret_cast<const char*>(&geom.getInnerAreas()[0]),
                sizeof(double) * locSize);
  _geomsOffset += sizeof(double) * locSize;

  for (const auto& inner : geom.getInners()) {
    locSize = inner.rawRing().size();
    double maxSegLen = inner.getMaxSegLen();
    _geomsF.write(reinterpret_cast<const char*>(&maxSegLen), sizeof(double));
    _geomsOffset += sizeof(double);

    _geomsF.write(reinterpret_cast<const char*>(&locSize), sizeof(uint32_t));
    if (locSize)
      _geomsF.write(reinterpret_cast<const char*>(&inner.rawRing()[0]),
                    sizeof(util::geo::XSortedTuple<int32_t>) * locSize);
    _geomsOffset +=
        sizeof(uint32_t) + sizeof(util::geo::XSortedTuple<int32_t>) * locSize;
  }
}

// ____________________________________________________________________________
template <typename W>
void sj::GeometryCache<W>::readLine(std::fstream& str,
                                    util::geo::I32XSortedLine& ret) const {
  double maxSegLen;
  str.read(reinterpret_cast<char*>(&maxSegLen), sizeof(double));
  ret.setMaxSegLen(maxSegLen);

  uint32_t sizeOuter;
  str.read(reinterpret_cast<char*>(&sizeOuter), sizeof(uint32_t));

  // TODO: careful, resize initializes entire vector!
  ret.rawLine().resize(sizeOuter);

  str.read(reinterpret_cast<char*>(&ret.rawLine()[0]),
           sizeof(util::geo::XSortedTuple<int32_t>) * sizeOuter);
}

// ____________________________________________________________________________
template <typename W>
void sj::GeometryCache<W>::writeLine(const util::geo::I32XSortedLine& geom) {
  // geom, outer
  double maxSegLen = geom.getMaxSegLen();
  _geomsF.write(reinterpret_cast<const char*>(&maxSegLen), sizeof(double));
  _geomsOffset += sizeof(double);

  uint32_t locSize = geom.rawLine().size();
  _geomsF.write(reinterpret_cast<const char*>(&locSize), sizeof(uint32_t));
  if (locSize)
    _geomsF.write(reinterpret_cast<const char*>(&geom.rawLine()[0]),
                  sizeof(util::geo::XSortedTuple<int32_t>) * locSize);
  _geomsOffset +=
      sizeof(uint32_t) + sizeof(util::geo::XSortedTuple<int32_t>) * locSize;
}

// ____________________________________________________________________________
template <>
std::string sj::GeometryCache<sj::Area>::getFName() const {
  return _dir + "/areas";
}

// ____________________________________________________________________________
template <>
std::string sj::GeometryCache<sj::Line>::getFName() const {
  return _dir + "/lines";
}

// ____________________________________________________________________________
template <>
std::string sj::GeometryCache<sj::Point>::getFName() const {
  return _dir + "/points";
}

// ____________________________________________________________________________
template <>
std::string sj::GeometryCache<sj::SimpleLine>::getFName() const {
  return _dir + "/simplelines";
}

// ____________________________________________________________________________
template class sj::GeometryCache<sj::Area>;
template class sj::GeometryCache<sj::Line>;
template class sj::GeometryCache<sj::SimpleLine>;
template class sj::GeometryCache<sj::Point>;
