// Copyright 2023, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_GEOMETRYSTORE_H_
#define SPATIALJOINS_GEOMETRYSTORE_H_

#include <fcntl.h>
#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#if __cplusplus >= 201402L
#include <shared_mutex>
#include <stdexcept>
#endif
#include <unordered_map>
#include <utility>
#include <vector>

#include "BoxIds.h"
#include "SpatialJoinCfg.h"
#include "SweepEventList.h"
#include "util/geo/Geo.h"

namespace sj {

struct GeomId {
  std::string id;
  size_t subId;
};

struct MultiGeom {
  // id
  std::string id;

  // the sweep events (IN and OUT) of all members (sorted)
  std::vector<BoxVal> events;
};

struct SimpleArea {
  // id
  std::string id;

  // polygon
  util::geo::Ring<int32_t> geom;
};

struct Area {
  // id
  std::string id;

  // sub id (for multipolygons)
  size_t subId;

  // polygons
  util::geo::I32XSortedPolygon geom;

  // box ids
  std::vector<sj::boxids::BoxId> boxIds;
};

struct SimpleLine {
  // id
  std::string id;
};

struct Line {
  // id
  std::string id;

  // sub id (for multilines)
  size_t subId;

  // line
  util::geo::I32XSortedLine geom;

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

// header of the geometries, rawLen is after decompression, atm no compression
// is used
struct RecordHeader {
  uint32_t storedLen;
  uint32_t rawLen;
};

// reads the payload of a single record from a mem buffer
class RecordReader;

// default size (in bytes of serialized geometries) up to which a
// TypedGeometryStore is kept in memory
const static size_t DEFAULT_MAX_MEM_CACHE_S = 3 * 1024 * 1024 * 20l;

// if available, use a shared_timed_mutex, if not, fall back to a normal mutex
#if __cplusplus >= 201402L
typedef std::shared_timed_mutex SharedMutex;
typedef std::shared_lock<SharedMutex> SharedLock;
#else
typedef std::mutex SharedMutex;
typedef std::unique_lock<SharedMutex> SharedLock;
#endif
typedef std::unique_lock<SharedMutex> ExclusiveLock;

template <typename W>
class TypedGeometryStore {
 public:
  // store is kept in memory until the geometries exceed
  // maxMemCacheSize bytes. If persistOnDisk is true, flush() writes the geoms
  // to a file <dir>/<prefix><type>
  TypedGeometryStore(const std::string& dir, const std::string& prefix,
                     bool persistOnDisk = false,
                     size_t maxMemCacheSize = DEFAULT_MAX_MEM_CACHE_S)
      : _dir(dir),
        _prefix(prefix),
        _maxMemCacheSize(maxMemCacheSize),
        _persistOnDisk(persistOnDisk) {}

  // read the geometries persisted at path
  explicit TypedGeometryStore(const std::string& path);

  ~TypedGeometryStore() {
    if (_geomsF.is_open()) _geomsF.close();
    if (_readFd >= 0) close(_readFd);
    if (_writeBuffer) delete[] _writeBuffer;
  }

  // may optionally be provided a pointer to the processed geom
  size_t add(const std::string& raw, std::shared_ptr<W> val = nullptr);

  // serializes val to str (the payload of a record, without the header)
  size_t writeTo(const W& val, std::ostream& str) const;

  // reads only the gid and sub id of the geometry at off
  std::pair<size_t, GeomId> getId(size_t off) const;

  // reads the geometry at off from disk
  std::pair<size_t, W> getFrom(size_t off) const;

  // if true, all geometries are held in memory and must be retrieved via
  // getFromMem()
  bool inMemory() const { return _inMemory; }

  std::shared_ptr<W> getFromMem(size_t off) const {
    return _memStore.at(untypedId(off));
  }

  // reads the multi geometry at off, only available for W = MultiGeom.
  // Thread-safe.
  W getMulti(size_t off) const;

  void flush();

  static std::string persistFName(const std::string& dir,
                                  const std::string& prefix);

  TypedGeometryStore(const TypedGeometryStore&) = delete;
  TypedGeometryStore& operator=(const TypedGeometryStore&) = delete;

  TypedGeometryStore(TypedGeometryStore&& other) noexcept { swap(other); }

  TypedGeometryStore& operator=(TypedGeometryStore&& other) noexcept {
    TypedGeometryStore tmp(std::move(other));
    swap(tmp);
    return *this;
  }

 private:
  // deserializes a geometry from the payload of a record, returns (estimated)
  // size and geom
  std::pair<size_t, W> decode(RecordReader& str) const;

  // deserializes only the gid and sub id from the payload of a record, returns
  // the estimated size and the id
  std::pair<size_t, GeomId> decodeId(RecordReader& str) const;

  // string repr of the type this store holds
  static std::string typeName();

  // the name of the (temporary) geometry file
  std::string getFName() const;

  void swap(TypedGeometryStore& other) {
    _geomsF.swap(other._geomsF);
    std::swap(_writeBuffer, other._writeBuffer);
    std::swap(_readFd, other._readFd);
    std::swap(_geomsOffset, other._geomsOffset);
    _dir.swap(other._dir);
    _prefix.swap(other._prefix);
    _fName.swap(other._fName);
    _memStore.swap(other._memStore);
    std::swap(_inMemory, other._inMemory);
    std::swap(_maxMemCacheSize, other._maxMemCacheSize);
    std::swap(_persistOnDisk, other._persistOnDisk);
  }

  void openFile() {
    _fName = _persistOnDisk ? persistFName(_dir, _prefix) : getFName();

    _writeBuffer = new char[WRITE_BUFF_SIZE];

    _geomsF.rdbuf()->pubsetbuf(_writeBuffer, WRITE_BUFF_SIZE);
    _geomsF.open(_fName, std::ios::out | std::ios::binary | std::ios::trunc);
    if (!_geomsF.is_open()) {
      throw std::runtime_error("Could not open geometry store file " + _fName);
    }

    _readFd = open(_fName.c_str(), O_RDONLY);
    if (_readFd < 0) {
      throw std::runtime_error("Could not open geometry store file " + _fName);
    }
#ifdef __unix__
    posix_fadvise(_readFd, 0, 0, POSIX_FADV_RANDOM);
#endif

    if (!_persistOnDisk) unlink(_fName.c_str());
  }

  // the maximum payload size of a record that getId() has to read: the gid
  // (at most 2^16 - 1 chars, with its length) and the sub id are serialized
  // first, so the full record never has to be read
  static const size_t MAX_ID_PAYLOAD =
      sizeof(uint16_t) + std::numeric_limits<uint16_t>::max() + sizeof(size_t);

  // reads the record at off from disk, but at most maxLen bytes of its
  // payload. The returned reader points into a thread-local buffer, which is
  // valid until the next call of readRecord() of this thread
  RecordReader readRecord(size_t off, size_t maxLen) const;

  // writes payload as a record (header followed by payload) to str, returns
  // the number of bytes written
  static size_t writeRecord(const std::string& payload, std::ostream& str);

  // writes the records of all geometries held in memory to str, checking that
  // each record ends up at its offset
  void writeMemStore(std::ostream& str) const;

  // the geometry type the IDs returned by add() are tagged with
  static GeomType geomType();
  size_t readLine(RecordReader& str, util::geo::I32XSortedLine& ret) const;
  static size_t writeLine(const util::geo::I32XSortedLine& ret,
                          std::ostream& str);

  size_t readPoly(RecordReader& str, util::geo::I32XSortedPolygon& ret) const;
  static size_t writePoly(const util::geo::I32XSortedPolygon& ret,
                          std::ostream& str);

  mutable std::fstream _geomsF;

  // read-only file descriptor of the geometry file, records are read from it
  // with pread(), which is thread-safe
  int _readFd = -1;

  size_t _geomsOffset = 0;

  std::string _dir, _prefix;
  std::string _fName;

  // must be ordered map to ensure correct writing order when flushing to disk!
  std::map<size_t, std::shared_ptr<W>> _memStore;
  bool _inMemory = true;

  size_t _maxMemCacheSize = DEFAULT_MAX_MEM_CACHE_S;

  // whether flush() writes the geometries to a file which is kept
  bool _persistOnDisk = false;

  char* _writeBuffer = 0;
};

struct WriteCand {
  std::string raw;
  std::shared_ptr<void> geom;
  std::string gid;
  size_t subid;
  PreparedGeomMeta meta;
  GeomType type;
  util::geo::I32Point rightPoint;
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
  // typed ID in the multi geometry store, assigned in GeometryStore::addBatch()
  size_t id;
  // the (padded) bounding box of all members
  util::geo::I32Box bbox;
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
  // if persistOnDisk is true, flush() writes the geometries to files
  // <cache>/<prefix><type>, which can be read again with
  // GeometryStore(cfg, cache, prefix). Each typed store is kept in memory
  // until its serialized geometries exceed maxMemCacheSize bytes
  GeometryStore(const SpatialJoinCfg& cfg, const std::string& cache,
                const std::string& prefix, bool persistOnDisk,
                size_t maxMemCacheSize = DEFAULT_MAX_MEM_CACHE_S)
      : _cfg(cfg),
        _pointStore(cache, prefix, persistOnDisk, maxMemCacheSize),
        _areaStore(cache, prefix, persistOnDisk, maxMemCacheSize),
        _simpleAreaStore(cache, prefix, persistOnDisk, maxMemCacheSize),
        _lineStore(cache, prefix, persistOnDisk, maxMemCacheSize),
        _simpleLineStore(cache, prefix, persistOnDisk, maxMemCacheSize),
        _multiStore(cache, prefix, persistOnDisk, maxMemCacheSize) {}

  // construct a GeometryStore from the files persisted in dir with prefix
  // by a GeometryStore constructed with persistOnDisk
  GeometryStore(const SpatialJoinCfg& cfg, const std::string& dir,
                const std::string& prefix)
      : _cfg(cfg),
        _pointStore(TypedGeometryStore<Point>::persistFName(dir, prefix)),
        _areaStore(TypedGeometryStore<Area>::persistFName(dir, prefix)),
        _simpleAreaStore(
            TypedGeometryStore<SimpleArea>::persistFName(dir, prefix)),
        _lineStore(TypedGeometryStore<Line>::persistFName(dir, prefix)),
        _simpleLineStore(
            TypedGeometryStore<SimpleLine>::persistFName(dir, prefix)),
        _multiStore(TypedGeometryStore<MultiGeom>::persistFName(dir, prefix)) {}

  GeometryStore(const GeometryStore&) = delete;
  GeometryStore& operator=(const GeometryStore&) = delete;

  void log(const std::string& msg);

  util::geo::I32Box add(const util::geo::I32MultiPolygon& a,
                        const std::string& gid, WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32MultiPolygon& a,
                        const std::string& gid, size_t,
                        WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32Polygon& a, const std::string& gid,
                        WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32Polygon& a, const std::string& gid,
                        size_t subId, WriteBatch& batch) const;

  util::geo::I32Box add(const util::geo::I32MultiLine& a,
                        const std::string& gid, size_t,
                        WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32MultiLine& a,
                        const std::string& gid, WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32Line& a, const std::string& gid,
                        WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32Line& a, const std::string& gid,
                        size_t subid, WriteBatch& batch) const;

  util::geo::I32Box add(const util::geo::I32Point& a, const std::string& gid,
                        WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32Point& a, const std::string& gid,
                        size_t subid, WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32MultiPoint& a,
                        const std::string& gid, size_t,
                        WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32MultiPoint& a,
                        const std::string& gid, WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32Collection& a,
                        const std::string& gid, WriteBatch& batch) const;

  void addMulti(WriteBatch&& members, const util::geo::I32Box& box,
                const std::string& gid, WriteBatch& batch) const;

  void addBatch(WriteBatch& cands, size_t t, const PreparedGeomMetaCb& metaCb);

  void flush();

  const TypedGeometryStore<Point>& pointStore() const { return _pointStore; }
  const TypedGeometryStore<Area>& areaStore() const { return _areaStore; }
  const TypedGeometryStore<SimpleArea>& simpleAreaStore() const {
    return _simpleAreaStore;
  }
  const TypedGeometryStore<Line>& lineStore() const { return _lineStore; }
  const TypedGeometryStore<SimpleLine>& simpleLineStore() const {
    return _simpleLineStore;
  }
  const TypedGeometryStore<MultiGeom>& multiStore() const {
    return _multiStore;
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
                                         const G2<T>& refGeom) const {
    return padBox(util::geo::getBoundingBox(geom),
                  util::geo::getBoundingBox(refGeom), _cfg);
  }
  static size_t foldString(const std::string& s);

  double DUPLICATE_REMOVAL_MIN_SIZE = 500;

 private:
  util::geo::I32Box addMembers(const util::geo::I32Collection& col,
                               const std::string& gid, size_t& subId,
                               WriteBatch& target) const;

  // returns the geometry stored at off in store, bypassing any cache
  template <typename W>
  static std::shared_ptr<W> get(const TypedGeometryStore<W>& store, size_t off);

  const SpatialJoinCfg _cfg;

  TypedGeometryStore<Point> _pointStore;
  TypedGeometryStore<Area> _areaStore;
  TypedGeometryStore<SimpleArea> _simpleAreaStore;
  TypedGeometryStore<Line> _lineStore;
  TypedGeometryStore<SimpleLine> _simpleLineStore;
  TypedGeometryStore<MultiGeom> _multiStore;

  // number of multi geometries, guarded by _multiGeomCacheWriteMtx
  size_t _numMultis = 0;

  // number of geometries reported by addBatch()
  std::atomic<size_t> _numReported{0};
  mutable std::mutex _pointGeomCacheWriteMtx;
  mutable std::mutex _lineGeomCacheWriteMtx;
  mutable std::mutex _simpleLineGeomCacheWriteMtx;
  mutable std::mutex _areaGeomCacheWriteMtx;
  mutable std::mutex _simpleAreaGeomCacheWriteMtx;
  mutable std::mutex _multiGeomCacheWriteMtx;

  util::geo::I32Box _filterBox = {{std::numeric_limits<int32_t>::lowest(),
                                   std::numeric_limits<int32_t>::lowest()},
                                  {std::numeric_limits<int32_t>::max(),
                                   std::numeric_limits<int32_t>::max()}};
};

}  // namespace sj

#endif
