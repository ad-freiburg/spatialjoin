// Copyright 2023, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_GEOMETRYCACHE_H_
#define SPATIALJOINS_GEOMETRYCACHE_H_

#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "BoxIds.h"
#include "util/geo/Geo.h"

namespace sj {

struct SimpleArea {
  // polygon
  util::geo::Ring<int32_t> geom;

  // id
  std::string id;
};

struct Area {
  // polygons
  util::geo::I32XSortedPolygon geom;

  // envelope
  util::geo::I32Box box;

  // id
  std::string id;

  // sub id (for multipolygons)
  uint16_t subId;

  // area
  double area;

  // outer area
  double outerArea;

  // box ids
  std::vector<sj::boxids::BoxId> boxIds;

  // cutouts
  std::map<int32_t, size_t> cutouts;

  // Convex hull
  util::geo::I32XSortedPolygon convexHull;

  // OBB
  util::geo::I32XSortedPolygon obb;

  // inner geom
  util::geo::I32XSortedPolygon inner;

  // inner polygon envelope
  util::geo::I32Box innerBox;

  // outer area for inner polygon
  double innerOuterArea;

  // outer geom
  util::geo::I32XSortedPolygon outer;

  // outer polygon envelope
  util::geo::I32Box outerBox;

  // outer area for outer polygon
  double outerOuterArea;
};

struct SimpleLine {
  // line
  util::geo::I32Point a;
  util::geo::I32Point b;

  // id
  std::string id;
};

struct Line {
  // line
  util::geo::I32XSortedLine geom;

  // envelope
  util::geo::I32Box box;

  // id
  std::string id;

  // sub id (for multilines)
  uint16_t subId;

  // length
  double length;

  // box ids
  std::vector<sj::boxids::BoxId> boxIds;

  // cutouts
  std::map<int32_t, size_t> cutouts;

  // Convex hull
  util::geo::I32XSortedPolygon convexHull;

  // OBB
  util::geo::I32XSortedPolygon obb;
};

struct Point {
  // id
  std::string id;

  // sub id (for multilines)
  uint16_t subId;
};

// const static size_t READ_BUFF_SIZE = 1024 * 5 ;

template <typename W>
class GeometryCache {
 public:
  GeometryCache(size_t maxSize, size_t numthreads, const std::string& dir)
      : _maxSize(maxSize),
        _numThreads(numthreads),
        _dir(dir),
        _mutexes(numthreads) {
    _geomsFReads.resize(numthreads);
    _geomsFReadsBuffs.resize(numthreads);
    _accessCount.resize(numthreads);
    _diskAccessCount.resize(numthreads);

    _vals.resize(numthreads);
    _idMap.resize(numthreads);

    _fName = getFName();

    _geomsF.open(_fName, std::ios::out | std::ios::in | std::ios::binary |
                             std::ios::trunc);

    for (size_t i = 0; i < _geomsFReads.size(); i++) {
      _geomsFReads[i].open(_fName, std::ios::in | std::ios::binary);
      // _geomsFReadsBuffs[i].resize(READ_BUFF_SIZE);
      // _geomsFReads[i].rdbuf()->pubsetbuf(&_geomsFReadsBuffs[i][0],
      // READ_BUFF_SIZE);
    }
    unlink(_fName.c_str());
  }

  ~GeometryCache() {
    size_t access = 0;
    size_t diskAccess = 0;
    for (size_t i = 0; i < _accessCount.size(); i++) {
      access += _accessCount[i];
      diskAccess += _diskAccessCount[i];
    }
    std::cerr << "Geometry cache <" << getFName() << ">: " << access
              << " accesses, " << diskAccess << " disk lookups" << std::endl;

    if (_geomsF.is_open()) _geomsF.close();
    for (size_t i = 0; i < _geomsFReads.size(); i++) {
      if (_geomsFReads[i].is_open()) _geomsFReads[i].close();
    }
  }

  size_t add(const W& val);
  size_t add(const std::string& raw);
  static size_t writeTo(const W& val, std::ostream& str);

  std::shared_ptr<W> get(size_t off, size_t tid) const;
  W getFromDisk(size_t off, size_t tid) const;
  std::shared_ptr<W> cache(size_t off, const W& val, size_t tid) const;

  std::shared_ptr<W> get(size_t off) const { return get(off, 0); }

  void flush();

  GeometryCache& operator=(GeometryCache&& other) {
    other._geomsF.flush();
    _accessCount = std::move(other._accessCount);
    _diskAccessCount = std::move(other._diskAccessCount);
    _geomsF = std::move(other._geomsF);
    _geomsFReads = std::move(other._geomsFReads);
    _geomsOffset = other._geomsOffset;

    _vals = std::move(other._vals);
    _idMap = std::move(other._idMap);
    _maxSize = other._maxSize;
    _dir = other._dir;
    _fName = other._fName;
    _numThreads = other._numThreads;

    return *this;
  }

 private:
  std::string getFName() const;
  void readLine(std::fstream& str, util::geo::I32XSortedLine& ret) const;
  static size_t writeLine(const util::geo::I32XSortedLine& ret, std::ostream& str);

  void readPoly(std::fstream& str, util::geo::I32XSortedPolygon& ret) const;
  static size_t writePoly(const util::geo::I32XSortedPolygon& ret, std::ostream& str);

  void readMultiPoly(std::fstream& str,
                     util::geo::I32XSortedMultiPolygon& ret) const;
  void writeMultiPoly(const util::geo::I32XSortedMultiPolygon& ret);

  mutable std::vector<size_t> _accessCount;
  mutable std::vector<size_t> _diskAccessCount;

  mutable std::fstream _geomsF;
  mutable std::vector<std::fstream> _geomsFReads;
  mutable std::vector<std::vector<char>> _geomsFReadsBuffs;
  size_t _geomsOffset = 0;

  mutable std::vector<std::list<std::pair<size_t, std::shared_ptr<W>>>> _vals;
  mutable std::vector<std::unordered_map<
      size_t,
      typename std::list<std::pair<size_t, std::shared_ptr<W>>>::iterator>>
      _idMap;

  size_t _maxSize, _numThreads;
  std::string _dir;
  std::string _fName;

  mutable std::vector<std::mutex> _mutexes;
};
}  // namespace sj

#endif
