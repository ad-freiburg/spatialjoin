// Copyright 2023, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_GEOMETRYSTORE_H_
#define SPATIALJOINS_GEOMETRYSTORE_H_

#include <atomic>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "BoxIds.h"
#include "SweepEventList.h"
#include "SweeperCfg.h"
#include "util/geo/Geo.h"

namespace sj {

struct MultiGeom {
  // rightmost point
  util::geo::Point<int32_t> rightPoint;

  // number of members
  size_t numMembers;

  // id
  std::string id;
};

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
class TypedGeometryStore {
 public:
  TypedGeometryStore(size_t numthreads, const std::string& dir)
      : TypedGeometryStore(numthreads, dir, ".spatialjoin"){};
  TypedGeometryStore(size_t numthreads, const std::string& dir,
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

  ~TypedGeometryStore() {
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
    return getFrom(untypedId(off), _geomsFReads[tid]);
  }

  // if true, all geometries are held in memory and must be retrieved via
  // getFromMem()
  bool inMemory() const { return _inMemory; }

  std::shared_ptr<W> getFromMem(size_t off) const {
    return _memStore.at(untypedId(off));
  }

  void flush();

  TypedGeometryStore& operator=(TypedGeometryStore&& other) {
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

  // the geometry type the IDs returned by add() are tagged with
  static GeomType geomType();
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
struct WriteCand {
  std::string raw;
  std::string gid;
  size_t subid;
  PreparedGeomMeta meta;
};

struct MultiWriteCand;

struct WriteBatch {
  std::vector<WriteCand> points;
  std::vector<WriteCand> foldedPoints;
  std::vector<WriteCand> simpleLines;
  std::vector<WriteCand> foldedSimpleLines;
  std::vector<WriteCand> lines;
  std::vector<WriteCand> simpleAreas;
  std::vector<WriteCand> foldedBoxAreas;
  std::vector<WriteCand> areas;
  std::vector<MultiWriteCand> multiGeoms;

  size_t size() const;
};

// a multi geometry, whose members are kept together in their own WriteBatch
struct MultiWriteCand {
  std::string gid;
  // the (padded) bounding box of all members
  util::geo::I32Box bbox;
  bool side;
  WriteBatch members;
};

inline size_t WriteBatch::size() const {
  size_t ret = points.size() + foldedSimpleLines.size() + foldedPoints.size() +
               simpleLines.size() + lines.size() + simpleAreas.size() +
               foldedBoxAreas.size() + areas.size();
  for (const auto& multi : multiGeoms) ret += multi.members.size();
  return ret;
}

// holds all geometries and their metadata written during the add phase
class GeometryStore {
 public:
  GeometryStore(const SweeperCfg& cfg, const std::string& cache,
                const std::string& tmpPrefix)
      : _cfg(cfg),
        _pointStore(cfg.numCacheThreads, cache, tmpPrefix),
        _areaStore(cfg.numCacheThreads, cache, tmpPrefix),
        _simpleAreaStore(cfg.numCacheThreads, cache, tmpPrefix),
        _lineStore(cfg.numCacheThreads, cache, tmpPrefix),
        _simpleLineStore(cfg.numCacheThreads, cache, tmpPrefix),
        _numSides(1) {
    if (_cfg.forceTwoSided) _numSides = 2;
  };

  void log(const std::string& msg);

  util::geo::I32Box add(const util::geo::I32MultiPolygon& a,
                        const std::string& gid, bool side,
                        WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32MultiPolygon& a,
                        const std::string& gid, size_t, bool side,
                        WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32Polygon& a, const std::string& gid,
                        bool side, WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32Polygon& a, const std::string& gid,
                        size_t subId, bool side, WriteBatch& batch) const;

  util::geo::I32Box add(const util::geo::I32MultiLine& a,
                        const std::string& gid, size_t, bool side,
                        WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32MultiLine& a,
                        const std::string& gid, bool side,
                        WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32Line& a, const std::string& gid,
                        bool side, WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32Line& a, const std::string& gid,
                        size_t subid, bool side, WriteBatch& batch) const;

  util::geo::I32Box add(const util::geo::I32Point& a, const std::string& gid,
                        bool side, WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32Point& a, const std::string& gid,
                        size_t subid, bool side, WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32MultiPoint& a,
                        const std::string& gid, size_t, bool side,
                        WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32MultiPoint& a,
                        const std::string& gid, bool side,
                        WriteBatch& batch) const;

  // adds the members (added with a subid > 0) of the multi geometry gid with
  // bounding box box as a single multi geometry to batch
  void addMulti(WriteBatch&& members, const util::geo::I32Box& box,
                const std::string& gid, bool side, WriteBatch& batch) const;

  void addBatch(WriteBatch& cands, sj::SweepEventList* events);

  void flush();

  size_t numReferences() const {
    size_t ret = 0;
    for (const auto& subs : _refs) {
      for (const auto& refd : subs.second) {
        ret += refd.second.size();
      }
    }
    return ret;
  }

  const TypedGeometryStore<Point>& pointStore() const { return _pointStore; }
  const TypedGeometryStore<Area>& areaStore() const { return _areaStore; }
  const TypedGeometryStore<SimpleArea>& simpleAreaStore() const {
    return _simpleAreaStore;
  }
  const TypedGeometryStore<Line>& lineStore() const { return _lineStore; }
  const TypedGeometryStore<SimpleLine>& simpleLineStore() const {
    return _simpleLineStore;
  }

  size_t numSides() const { return _numSides; }

  void setNumSides(size_t sides) { _numSides = sides; }

  size_t subSize(const std::string& id) const { return _subSizes.at(id); }

  bool isMulti(const std::string& id) const {
    return _subSizes.find(id) != _subSizes.end();
  }

  size_t numMultis(bool side) const { return _multiIds[side].size(); }

  const std::string& multiId(bool side, size_t id) const {
    return _multiIds[side][id];
  }

  util::geo::I32Point multiRightPoint(const std::string& gid) const {
    auto i = _multiRightPoint.find(gid);
    if (i == _multiRightPoint.end()) return {};
    return i->second;
  }

  bool hasRefs() const { return _refs.size() != 0; }

  bool isRefed(const std::string& gid) const { return _refs.count(gid); }

  const std::unordered_map<std::string, size_t>* getRefs(const std::string& gid,
                                                         size_t subId) const {
    auto i = _refs.find(gid);
    if (i == _refs.end()) return nullptr;
    auto j = i->second.find(subId);
    if (j == i->second.end()) return nullptr;
    return &j->second;
  }

  std::pair<std::string, size_t> selfCheck(size_t id) const {
    return _selfChecks[id];
  }

  void setFilterBox(const util::geo::I32Box& filterBox) {
    _filterBox = filterBox;
  }

  template <template <typename> class G, typename T>
  util::geo::I32Box getPaddedBoundingBox(const G<T>& geom) const {
    return getPaddedBoundingBox(geom, geom);
  }

  template <template <typename> class G1, template <typename> class G2,
            typename T>
  util::geo::I32Box getPaddedBoundingBox(const G1<T>& geom,
                                         const G2<T>& refGeom) const;
  static size_t foldString(const std::string& s);

  double DUPLICATE_REMOVAL_MIN_SIZE = 500;

 private:
  // returns the geometry stored at off in store, bypassing any cache
  template <typename W>
  static std::shared_ptr<W> get(const TypedGeometryStore<W>& store, size_t off);

  // returns the multi ID of gid
  size_t multiAdd(const std::string& gid, bool side, int32_t xRight,
                  const util::geo::I32Point& pointRight);

  void duplicatesToReferences(int file);

  const SweeperCfg _cfg;

  TypedGeometryStore<Point> _pointStore;
  TypedGeometryStore<Area> _areaStore;
  TypedGeometryStore<SimpleArea> _simpleAreaStore;
  TypedGeometryStore<Line> _lineStore;
  TypedGeometryStore<SimpleLine> _simpleLineStore;

  // these are written during the geometry add phase
  std::vector<std::string> _multiIds[2];  // required on OUT to get the GID
  std::vector<int32_t> _multiRightX[2];
  std::unordered_map<std::string, util::geo::I32Point> _multiRightPoint;
  std::unordered_map<std::string, size_t> _multiGidToId[2];
  std::unordered_map<std::string, size_t> _subSizes;

  std::atomic<uint8_t> _numSides;

  mutable std::mutex _multiAddMtx;
  mutable std::mutex _sweepEventWriteMtx;
  mutable std::mutex _pointGeomCacheWriteMtx;
  mutable std::mutex _lineGeomCacheWriteMtx;
  mutable std::mutex _simpleLineGeomCacheWriteMtx;
  mutable std::mutex _areaGeomCacheWriteMtx;
  mutable std::mutex _simpleAreaGeomCacheWriteMtx;

  std::unordered_map<std::string, util::geo::I32Box> _selfCheckBounds;

  std::unordered_map<
      std::string,
      std::unordered_map<size_t, std::unordered_map<std::string, size_t>>>
      _refs;

  std::vector<std::pair<std::string, size_t>> _selfChecks;

  util::geo::I32Box _filterBox = {{std::numeric_limits<int32_t>::lowest(),
                                   std::numeric_limits<int32_t>::lowest()},
                                  {std::numeric_limits<int32_t>::max(),
                                   std::numeric_limits<int32_t>::max()}};
};

}  // namespace sj

#endif
