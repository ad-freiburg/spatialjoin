// Copyright 2023, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_GEOMETRYSTORE_H_
#define SPATIALJOINS_GEOMETRYSTORE_H_

#include <fstream>
#include <iostream>
#include <map>
#include <memory>

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

  // id
  std::string id;

  // sub id (for multipolygons)
  size_t subId;

  // box ids
  std::vector<sj::boxids::BoxId> boxIds;
};

struct SimpleLine {
  // id
  std::string id;
};

struct Line {
  // line
  util::geo::I32XSortedLine geom;

  // id
  std::string id;

  // sub id (for multilines)
  size_t subId;

  // box ids
  std::vector<sj::boxids::BoxId> boxIds;
};

struct Point {
  // id
  std::string id;

  // sub id (for multipoints)
  size_t subId;
};

const static size_t WRITE_BUFF_SIZE = 1024 * 1024 * 4l;

template <typename W>
class GeometryStore {
 public:
  GeometryStore(size_t numthreads, const std::string& dir)
      : GeometryStore(numthreads, dir, ".spatialjoin"){};
  GeometryStore(size_t numthreads, const std::string& dir,
                const std::string& tmpPrefix)
      : _numThreads(numthreads), _dir(dir), _tmpPrefix(tmpPrefix) {
    _geomsFReads.resize(numthreads);

    _fName = getFName();

    _writeBuffer = new char[WRITE_BUFF_SIZE];

    _geomsF.rdbuf()->pubsetbuf(_writeBuffer, WRITE_BUFF_SIZE);
    _geomsF.open(_fName, std::ios::out | std::ios::binary | std::ios::trunc);

    for (size_t i = 0; i < _geomsFReads.size(); i++) {
      _geomsFReads[i].open(_fName, std::ios::in | std::ios::binary);
    }
    unlink(_fName.c_str());
  }

  ~GeometryStore() {
    if (_geomsF.is_open()) _geomsF.close();
    for (size_t i = 0; i < _geomsFReads.size(); i++) {
      if (_geomsFReads[i].is_open()) _geomsFReads[i].close();
    }
    if (_writeBuffer) delete[] _writeBuffer;
  }

  size_t add(const std::string& raw);
  size_t writeTo(const W& val, std::ostream& str) const;

  std::pair<size_t, W> getFrom(size_t off, std::istream& str) const;

  // reads the geometry at off from disk, using the read stream of thread tid
  // (0 <= tid < numthreads). Not thread-safe for the same tid!
  std::pair<size_t, W> getFrom(size_t off, size_t tid) const {
    return getFrom(off, _geomsFReads[tid]);
  }

  // if true, all geometries are held in memory and must be retrieved via
  // getFromMem()
  bool inMemory() const { return _inMemory; }

  std::shared_ptr<W> getFromMem(size_t off) const { return _memStore.at(off); }

  void flush();

  GeometryStore& operator=(GeometryStore&& other) {
    other._geomsF.flush();
    _geomsF = std::move(other._geomsF);
    _geomsFReads = std::move(other._geomsFReads);
    _geomsOffset = other._geomsOffset;

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

  size_t _numThreads;
  std::string _dir, _tmpPrefix;
  std::string _fName;

  // must be ordered map to ensure correct writing order when flushing to disk!
  std::map<size_t, std::shared_ptr<W>> _memStore;
  bool _inMemory = true;

  char* _writeBuffer = 0;
};
}  // namespace sj

#endif
