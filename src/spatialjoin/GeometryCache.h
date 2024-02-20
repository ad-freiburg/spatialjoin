// Copyright 2023, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_GEOMETRYCACHE_H_
#define SPATIALJOINS_GEOMETRYCACHE_H_

#include <fstream>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "BoxIds.h"
#include "util/geo/Geo.h"

namespace sj {

struct SimpleArea {
  // polygon
  util::geo::I32Polygon geom;

  // id
  size_t id;
};

struct Area {
  // polygons
  util::geo::I32XSortedPolygon geom;

  // envelope
  util::geo::I32Box box;

  // id
  size_t id;

  // sub id (for multipolygons)
  uint16_t subId;

  // area
  double area;

  // inner geom
  // util::geo::I32XSortedPolygon inner;

  // outer geom
  // util::geo::I32XSortedPolygon outer;

  // box ids
  std::vector<sj::boxids::BoxId> boxIds;

  // cutouts
  // std::unordered_map<int32_t, util::geo::I32XSortedMultiPolygon> cutouts;

  // OBB
  // util::geo::I32Polygon obb;
};

struct SimpleLine {
  // line
  util::geo::I32Point a;
  util::geo::I32Point b;

  // id
  size_t id;
};

struct Line {
  // line
  util::geo::I32XSortedLine geom;

  // envelope
  util::geo::I32Box box;

  // id
  size_t id;

  // sub id (for multilines)
  uint16_t subId;

  // box ids
  std::vector<sj::boxids::BoxId> boxIds;

  // OBB
  // util::geo::I32Polygon obb;
};

struct Point {
  // line
  util::geo::I32Point geom;

  // id
  size_t id;

  // sub id (for multilines)
  uint16_t subId;
};

template <typename W>
class GeometryCache {
 public:
  GeometryCache(size_t maxSize, size_t numthreads, bool reuse)
      : _maxSize(maxSize) {
    _geomsFReads.resize(numthreads);
    _accessCount.resize(numthreads);
    _diskAccessCount.resize(numthreads);

    _vals.resize(numthreads);
    _idMap.resize(numthreads);

    if (reuse) {
      flush();
    } else {
      _geomsF.open(getFName(), std::ios::out | std::ios::in | std::ios::binary |
                                   std::ios::trunc);
    }
  }

  GeometryCache(size_t maxSize, size_t numthreads)
      : GeometryCache(maxSize, numthreads, false) {}

  ~GeometryCache() {
    size_t access = 0;
    size_t diskAccess = 0;
    for (size_t i = 0; i < _accessCount.size(); i++) {
      access += _accessCount[i];
      diskAccess += _diskAccessCount[i];
    }
    std::cerr << "Geometry cache <" << getFName() << ">: " << access
              << " accesses, " << diskAccess << " disk lookups" << std::endl;
  }

  size_t add(const W& val);

  std::shared_ptr<W> get(size_t off, size_t tid) const;
  W getFromDisk(size_t off, size_t tid) const;
  std::shared_ptr<W> cache(size_t off, const W& val, size_t tid) const;

  std::shared_ptr<W> get(size_t off) const { return get(off, 0); }

  void flush();

 private:
  std::string getFName() const;
  void readLine(std::fstream& str, util::geo::I32XSortedLine& ret) const;
  void writeLine(const util::geo::I32XSortedLine& ret);

  void readPoly(std::fstream& str, util::geo::I32XSortedPolygon& ret) const;
  void writePoly(const util::geo::I32XSortedPolygon& ret);

  void readMultiPoly(std::fstream& str,
                     util::geo::I32XSortedMultiPolygon& ret) const;
  void writeMultiPoly(const util::geo::I32XSortedMultiPolygon& ret);

  mutable std::vector<size_t> _accessCount;
  mutable std::vector<size_t> _diskAccessCount;

  mutable std::fstream _geomsF;
  mutable std::vector<std::fstream> _geomsFReads;
  size_t _geomsOffset = 0;

  mutable std::vector<std::list<std::pair<size_t, std::shared_ptr<W>>>> _vals;
  mutable std::vector<std::unordered_map<
      size_t,
      typename std::list<std::pair<size_t, std::shared_ptr<W>>>::iterator>>
      _idMap;

  size_t _maxSize;
  mutable std::mutex _mut;
};
}  // namespace sj

#endif
