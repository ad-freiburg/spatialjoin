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

  // OBB
  util::geo::I32XSortedPolygon obb;
};

struct Point {
  // id
  std::string id;

  // sub id (for multipoints)
  uint16_t subId;
};

template <typename W>
struct ValEntry {
  size_t estimatedSize;
  std::shared_ptr<W> val;
};

const static size_t WRITE_BUFF_SIZE = 1024 * 1024 * 4l;

template <typename W>
class GeometryCache {
 public:
  GeometryCache(size_t maxSize, size_t numthreads, const std::string& dir)
      : GeometryCache(maxSize, numthreads, dir, ".spatialjoin"){};
  GeometryCache(size_t maxSize, size_t numthreads, const std::string& dir,
                const std::string& tmpPrefix)
      : _maxSize(maxSize),
        _numThreads(numthreads),
        _dir(dir),
        _tmpPrefix(tmpPrefix),
        _mutexes(numthreads + 1) {
    _geomsFReads.resize(numthreads + 1);

    _vals.resize(numthreads + 1);
    _valSizes.resize(numthreads + 1);
    _idMap.resize(numthreads + 1);

    _fName = getFName();

    _writeBuffer = new char[WRITE_BUFF_SIZE];

    _geomsF.rdbuf()->pubsetbuf(_writeBuffer, WRITE_BUFF_SIZE);
    _geomsF.open(_fName, std::ios::out | std::ios::binary |
                             std::ios::trunc);

    for (size_t i = 0; i < _geomsFReads.size(); i++) {
      _geomsFReads[i].open(_fName, std::ios::in | std::ios::binary);
    }
    unlink(_fName.c_str());
  }

  ~GeometryCache() {
    if (_geomsF.is_open()) _geomsF.close();
    for (size_t i = 0; i < _geomsFReads.size(); i++) {
      if (_geomsFReads[i].is_open()) _geomsFReads[i].close();
    }
    if (_writeBuffer) delete[]_writeBuffer;
  }

  size_t add(const std::string& raw);
  static size_t writeTo(const W& val, std::ostream& str);

  std::shared_ptr<W> get(size_t off, ssize_t tid) const;
  std::pair<size_t, W> getFrom(size_t off, std::istream& str) const;
  std::shared_ptr<W> cache(size_t off, const W& val, size_t estSize, size_t tid) const;

  std::shared_ptr<W> get(size_t off) const { return get(off, 0); }

  std::pair<size_t, size_t> size() const;

  void flush();

  GeometryCache& operator=(GeometryCache&& other) {
    other._geomsF.flush();
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
  size_t readLine(std::istream& str, util::geo::I32XSortedLine& ret) const;
  static size_t writeLine(const util::geo::I32XSortedLine& ret,
                          std::ostream& str);

  size_t readPoly(std::istream& str, util::geo::I32XSortedPolygon& ret) const;
  static size_t writePoly(const util::geo::I32XSortedPolygon& ret,
                          std::ostream& str);

  mutable std::fstream _geomsF;
  mutable std::vector<std::fstream> _geomsFReads;
  size_t _geomsOffset = 0;

  mutable std::vector<std::list<std::pair<size_t, ValEntry<W>>>> _vals;
  mutable std::vector<std::unordered_map<
      size_t,
      typename std::list<std::pair<size_t, ValEntry<W>>>::iterator>>
      _idMap;
  mutable std::vector<size_t> _valSizes;

  size_t _maxSize, _numThreads;
  std::string _dir, _tmpPrefix;
  std::string _fName;

  std::map<size_t, W> _memStore;
  bool _inMemory = true;

  char* _writeBuffer = 0;

  mutable std::vector<std::mutex> _mutexes;
};
}  // namespace sj

#endif
