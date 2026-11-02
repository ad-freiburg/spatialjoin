// Copyright 2026, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_SWEEPEVENTLIST_H_
#define SPATIALJOINS_SWEEPEVENTLIST_H_

#include <fcntl.h>
#include <sys/stat.h>

#include <algorithm>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "util/Misc.h"
#include "util/geo/Geo.h"

namespace sj {

enum GeomType : uint8_t {
  POLYGON = 0,
  LINE = 1,
  POINT = 2,
  SIMPLE_LINE = 3,
  SIMPLE_POLYGON = 4,
  FOLDED_POINT = 5,
  FOLDED_SIMPLE_LINE = 6,
  FOLDED_BOX_POLYGON = 7,
  DELETED = 8,
  SELF_CHECK = 9,
  SELF_CHECK_AREA = 10,
  SELF_CHECK_LINE = 11,
  SELF_CHECK_POINT = 12,
  MULTI = 13
};

// the upper 4 bits of a BoxVal ID hold the geometry type, see typedId()
const static size_t ID_TYPE_SHIFT = 56;
const static size_t ID_MASK = (size_t(1) << ID_TYPE_SHIFT) - 1;

// a folded gid (see GeometryStore::foldString()) stores its length in the byte
// directly below the type bits, and up to FOLD_MAX_LEN chars below that
const static size_t FOLD_LEN_SHIFT = ID_TYPE_SHIFT - 8;
const static size_t FOLD_MAX_LEN = FOLD_LEN_SHIFT / 8;

struct BoxVal {
  size_t id;  // typed ID, the lower 56 bits are the dist offset returned from
              // the GeometryStore, bits 57-60 are geometry type
  int32_t loY;       // the lower Y value of the box
  int32_t upY;       // the upper Y value of the box
  int32_t val;       // the left X value of the box
  bool out : 1;      // whether this is an IN or OUT event
  uint8_t storeId;   // the store (see GeometryCache::addStore()) the geometry
                     // is stored in
  double areaOrLen;  // area or len
  util::geo::I32Point point;
  size_t numAnchors;      // for polygons and lines: number of anchor points,
                          // used for DUPLICATE REMOVAL
                          // for simple lines: the other endpoint, for folded
                          // box polygons: the other box corner, see
                          // packPoint()
  util::geo::I32Box b45;  // oriented bounding box

  GeomType getType() const {
    return static_cast<GeomType>(id >> ID_TYPE_SHIFT);
  }

  void setType(GeomType type) {
    id = (id & ID_MASK) | (static_cast<size_t>(type) << ID_TYPE_SHIFT);
  }
};

struct PreparedGeomMeta {
  size_t id;               // typed ID
  util::geo::I32Box bbox;  // (padded) bounding box
  GeomType type;
  util::geo::I32Box box45;  // (padded) rotated bounding box
  util::geo::I32Point point1, point2;
  // number of anchor points (for simple lines and folded box polygons, the
  // events instead store the packed other point, see packPoint())
  size_t numAnchors;
  // area for polygons, length for lines, 0 for points
  double areaOrLen;
};

// default size of the write buffer of a SweepEventList, must be multiple of
// sizeof(BoxVal)
static const size_t DEFAULT_BUFFER_S =
    ((16 * 1024 * 1024 + sizeof(BoxVal)) / sizeof(BoxVal)) * sizeof(BoxVal);

// default size of the read buffer of a SweepEventList::Reader, must be
// multiple of sizeof(BoxVal)
static const size_t DEFAULT_RBUF_S = 100000 * sizeof(BoxVal);

// _____________________________________________________________________________
inline size_t typedId(size_t id, GeomType type) {
  return (id & ID_MASK) | (static_cast<size_t>(type) << ID_TYPE_SHIFT);
}

// _____________________________________________________________________________
inline size_t untypedId(size_t id) { return id & ID_MASK; }

// _____________________________________________________________________________
inline size_t getId(const BoxVal& bv) { return untypedId(bv.id); }

// _____________________________________________________________________________
inline size_t packPoint(const util::geo::I32Point& p) {
  return (static_cast<size_t>(static_cast<uint32_t>(p.getX())) << 32) |
         static_cast<uint32_t>(p.getY());
}

// _____________________________________________________________________________
inline util::geo::I32Point unpackPoint(size_t v) {
  return {static_cast<int32_t>(static_cast<uint32_t>(v >> 32)),
          static_cast<int32_t>(static_cast<uint32_t>(v))};
}

// _____________________________________________________________________________
inline util::geo::I32Point getOtherPoint(const BoxVal& bv) {
  // for simple lines the second endpoint, and for folded box polygons the
  // second box corner cannot be reconstructed from the box because it may be
  // padded in case withinDist > 0
  if (bv.getType() == SIMPLE_LINE || bv.getType() == FOLDED_SIMPLE_LINE ||
      bv.getType() == FOLDED_BOX_POLYGON)
    return unpackPoint(bv.numAnchors);
  return {bv.val, bv.point.getY() == bv.loY ? bv.upY : bv.loY};
}

// _____________________________________________________________________________
inline std::pair<BoxVal, BoxVal> getSweepEvents(
    size_t id, const util::geo::I32Box& bbox, const util::geo::I32Box& box45,
    const util::geo::I32Point& point1, const util::geo::I32Point& point2,
    size_t numAnchors, double areaOrLen, uint8_t storeId) {
  BoxVal in{id,
            bbox.getLowerLeft().getY(),
            bbox.getUpperRight().getY(),
            bbox.getLowerLeft().getX(),
            false,
            storeId,
            areaOrLen,
            point1,
            numAnchors,
            box45};
  BoxVal out{id,
             bbox.getLowerLeft().getY(),
             bbox.getUpperRight().getY(),
             bbox.getUpperRight().getX(),
             true,
             storeId,
             areaOrLen,
             point2,
             numAnchors,
             box45};

  // simple lines and folded box polygons store the packed point of the other
  // event instead of the number of anchors, see getOtherPoint()
  if (in.getType() == SIMPLE_LINE || in.getType() == FOLDED_SIMPLE_LINE ||
      in.getType() == FOLDED_BOX_POLYGON) {
    in.numAnchors = packPoint(point2);
    out.numAnchors = packPoint(point1);
  }

  return {in, out};
}

// _____________________________________________________________________________
inline std::pair<BoxVal, BoxVal> getSweepEvents(const PreparedGeomMeta& meta,
                                                uint8_t storeId) {
  return getSweepEvents(meta.id, meta.bbox, meta.box45, meta.point1,
                        meta.point2, meta.numAnchors, meta.areaOrLen, storeId);
}

// _____________________________________________________________________________
static int boxCmp(const void* a, const void* b) {
  const auto& boxa = static_cast<const BoxVal*>(a);
  const auto& boxb = static_cast<const BoxVal*>(b);
  if (boxa->val < boxb->val) return -1;
  if (boxa->val > boxb->val) return 1;

  // special multi events before everything else
  if (boxa->getType() == MULTI && boxb->getType() != MULTI) return -1;
  if (boxa->getType() != MULTI && boxb->getType() == MULTI) return 1;

  if (!boxa->out && boxb->out) return -1;
  if (boxa->out && !boxb->out) return 1;

  // everything before a polygon
  if (boxa->getType() != POLYGON && boxa->getType() != SIMPLE_POLYGON &&
      boxa->getType() != FOLDED_BOX_POLYGON &&
      (boxb->getType() == POLYGON || boxb->getType() == SIMPLE_POLYGON ||
       boxb->getType() == FOLDED_BOX_POLYGON))
    return -1;
  if ((boxa->getType() == POLYGON || boxa->getType() == SIMPLE_POLYGON ||
       boxa->getType() == FOLDED_BOX_POLYGON) &&
      boxb->getType() != POLYGON && boxb->getType() != SIMPLE_POLYGON &&
      boxb->getType() != FOLDED_BOX_POLYGON)
    return 1;

  // points before lines
  if ((boxa->getType() == POINT || boxa->getType() == FOLDED_POINT) &&
      (boxb->getType() == SIMPLE_LINE || boxb->getType() == LINE ||
       boxb->getType() == FOLDED_SIMPLE_LINE))
    return -1;
  if ((boxb->getType() == POINT || boxb->getType() == FOLDED_POINT) &&
      (boxa->getType() == SIMPLE_LINE || boxa->getType() == LINE ||
       boxa->getType() == FOLDED_SIMPLE_LINE))
    return 1;

  // smaller polygons before larger
  if ((boxa->getType() == POLYGON || boxa->getType() == SIMPLE_POLYGON ||
       boxa->getType() == FOLDED_BOX_POLYGON) &&
      (boxb->getType() == POLYGON || boxb->getType() == SIMPLE_POLYGON ||
       boxb->getType() == FOLDED_BOX_POLYGON) &&
      boxa->areaOrLen < boxb->areaOrLen)
    return -1;
  if ((boxa->getType() == POLYGON || boxa->getType() == SIMPLE_POLYGON ||
       boxa->getType() == FOLDED_BOX_POLYGON) &&
      (boxb->getType() == POLYGON || boxb->getType() == SIMPLE_POLYGON ||
       boxb->getType() == FOLDED_BOX_POLYGON) &&
      boxa->areaOrLen > boxb->areaOrLen)
    return 1;

  // shorter lines before longer
  if ((boxa->getType() == LINE || boxa->getType() == SIMPLE_LINE ||
       boxa->getType() == FOLDED_SIMPLE_LINE) &&
      (boxb->getType() == LINE || boxb->getType() == SIMPLE_LINE ||
       boxb->getType() == FOLDED_SIMPLE_LINE) &&
      boxa->areaOrLen < boxb->areaOrLen)
    return -1;
  if ((boxa->getType() == LINE || boxa->getType() == SIMPLE_LINE ||
       boxa->getType() == FOLDED_SIMPLE_LINE) &&
      (boxb->getType() == LINE || boxb->getType() == SIMPLE_LINE ||
       boxb->getType() == FOLDED_SIMPLE_LINE) &&
      boxa->areaOrLen > boxb->areaOrLen)
    return 1;

  // make ordernig of equal geoms deterministic
  if (boxa->getType() != boxb->getType())
    return boxa->getType() < boxb->getType() ? -1 : 1;
  if (boxa->id != boxb->id) return boxa->id < boxb->id ? -1 : 1;

  return 0;
}

class SweepEventList {
 public:
  // reader for streaming the event list
  class Reader {
   public:
    Reader() : _file(-1) {}
    Reader(int file, size_t bufSize) : _file(file), _buf(bufSize) {}
    // reader for an event list held in memory, in mem with length len
    Reader(const unsigned char* mem, size_t len)
        : _file(-1), _mem(mem), _memLen(len) {}

    // returns 0 at the end of the event list. Returned pointer is
    // valid until a call to next().
    const BoxVal* next() {
      if (_mem) {
        if (_readPtr >= static_cast<ssize_t>(_memLen)) return 0;
        auto ret = reinterpret_cast<const BoxVal*>(_mem + _readPtr);
        _readPtr += sizeof(BoxVal);
        return ret;
      }

      if (_file < 0) return 0;
      if (_readPtr >= _readBlockLen) {
        _readBlockLen =
            util::preadAll(_file, _buf.data(), _buf.size(), _fileOffset);
        _readPtr = 0;

        if (_readBlockLen == 0) return 0;  // end of file

        if (_readBlockLen < 0) {
          std::stringstream ss;
          ss << "Could not read from events file\n";
          ss << strerror(errno) << std::endl;
          throw std::runtime_error(ss.str());
        }

        if (_readBlockLen % sizeof(BoxVal))
          throw std::runtime_error("Corrupted events file");

        _fileOffset += _readBlockLen;
      }

      auto ret = reinterpret_cast<const BoxVal*>(_buf.data() + _readPtr);

      _readPtr += sizeof(BoxVal);

      return ret;
    }

   private:
    int _file;
    std::vector<unsigned char> _buf;
    const unsigned char* _mem = 0;
    size_t _memLen = 0;
    size_t _fileOffset = 0;
    ssize_t _readPtr = 0;
    ssize_t _readBlockLen = 0;
  };

  class CombinedReader {
   public:
    CombinedReader(const SweepEventList* a, const SweepEventList* b = 0,
                   size_t bufSize = DEFAULT_RBUF_S)
        : _readerA(a->newReader(bufSize)),
          _readerB(b ? b->newReader(bufSize) : Reader()) {
      _headA = _readerA.next();
      _headB = _readerB.next();
    }

    // returns 0 at the end of both event lists. Returned pointer is
    // valid until a call to next().
    //
    // TODO: store max-seg-length just like in util and quickly advance using
    // binary search, useful if one side is << the other
    const BoxVal* next() {
      if (_last == &_headA) _headA = _readerA.next();
      if (_last == &_headB) _headB = _readerB.next();

      if (!_headB || (_headA && boxCmp(_headA, _headB) <= 0))
        _last = &_headA;
      else
        _last = &_headB;

      return *_last;
    }

    // the side (0 for list a, 1 for list b) of the event last returned by
    // next()
    bool lastSide() const { return _last == &_headB; }

   private:
    Reader _readerA;
    Reader _readerB;
    const BoxVal* _headA = 0;
    const BoxVal* _headB = 0;
    const BoxVal** _last = 0;
  };

  // if persistOnDisk is true, finalize() writes the sorted event list to
  // a file <cacheDir>/<tmpPrefix>events
  SweepEventList(const std::string& cacheDir, const std::string& tmpPrefix,
                 size_t bufferSize = DEFAULT_BUFFER_S,
                 bool persistOnDisk = false)
      : _bufferSize(std::max(bufferSize / sizeof(BoxVal), size_t(1)) *
                    sizeof(BoxVal)),
        _persistOnDisk(persistOnDisk),
        _cacheDir(cacheDir),
        _tmpPrefix(tmpPrefix),
        _file(-1) {}

  // construct SweepEventList directly from file on disk
  explicit SweepEventList(const std::string& path)
      : _bufferSize(0), _onDisk(true) {
    _file = open(path.c_str(), O_RDONLY);
    if (_file < 0) {
      throw std::runtime_error("Could not open event list file " + path);
    }

    struct stat st;
    if (fstat(_file, &st) != 0) {
      close(_file);
      throw std::runtime_error("Could not stat event list file " + path);
    }

    _curSweepId = st.st_size / sizeof(BoxVal);

#ifdef __unix__
    posix_fadvise(_file, 0, 0, POSIX_FADV_SEQUENTIAL);
#endif
  }

  SweepEventList(const SweepEventList&) = delete;
  SweepEventList& operator=(const SweepEventList&) = delete;

  SweepEventList(SweepEventList&& other) noexcept : _bufferSize(0), _file(-1) {
    swap(other);
  }

  SweepEventList& operator=(SweepEventList&& other) noexcept {
    SweepEventList tmp(std::move(other));
    swap(tmp);
    return *this;
  }

  ~SweepEventList() {
    if (_file >= 0) close(_file);
  }

  void add(const BoxVal& bv) {
    const size_t N = _bufferSize / sizeof(BoxVal);

    // buffer must have size of max _bufferSize
    if (_buffer.size() == _buffer.capacity()) {
      _buffer.reserve(std::min(std::max(2 * _buffer.capacity(), size_t(1)), N));
    }

    _buffer.push_back(bv);

    if (_buffer.size() == N) flushToDisk();
    _curSweepId++;
  }

  void add(size_t id, const util::geo::I32Box& bbox,
           const util::geo::I32Box& box45, const util::geo::I32Point& point1,
           const util::geo::I32Point& point2, size_t numAnchors,
           double areaOrLen, uint8_t storeId) {
    const auto& events = getSweepEvents(id, bbox, box45, point1, point2,
                                        numAnchors, areaOrLen, storeId);
    add(events.first);
    add(events.second);
  }

  // add meta to this list, using store storeId
  void add(const PreparedGeomMeta& meta, uint8_t storeId) {
    add(meta.id, meta.bbox, meta.box45, meta.point1, meta.point2,
        meta.numAnchors, meta.areaOrLen, storeId);
  }

  size_t numObjects() const { return numEvents() / 2; }
  size_t numEvents() const { return _curSweepId; }

  Reader newReader(size_t bufSize = DEFAULT_RBUF_S) const {
    if (!_onDisk) {
      return Reader(reinterpret_cast<const unsigned char*>(_buffer.data()),
                    _buffer.size() * sizeof(BoxVal));
    }

    bufSize = std::max(bufSize / sizeof(BoxVal), size_t(1)) * sizeof(BoxVal);
    return Reader(_file, bufSize);
  }

  void flushToDisk() {
    // the events file is only created once it is required
    if (_file < 0) {
      auto fname = util::getTmpFName(_cacheDir, _tmpPrefix, "events");
      _file = open(fname.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0666);

      if (_file < 0) {
        throw std::runtime_error("Could not open temporary file " + fname);
      }

      // immediately unlink
      unlink(fname.c_str());

#ifdef __unix__
      posix_fadvise(_file, 0, 0, POSIX_FADV_SEQUENTIAL);
#endif
    }

    ssize_t r = util::writeAll(
        _file, reinterpret_cast<const unsigned char*>(_buffer.data()),
        _buffer.size() * sizeof(BoxVal));
    if (r < 0) {
      std::stringstream ss;
      ss << "Could not write to events file\n";
      ss << strerror(errno) << std::endl;
      throw std::runtime_error(ss.str());
    }
    // keeps the capacity of the buffer for the next events
    _buffer.clear();
    _onDisk = true;
  }

  void finalize(size_t numThreads = 4) {
    if (!_onDisk) {
      // in-memory sort
      std::sort(
          _buffer.begin(), _buffer.end(),
          [](const BoxVal& a, const BoxVal& b) { return boxCmp(&a, &b) < 0; });

      if (_persistOnDisk) {
        auto fName = persistFName();
        int out = open(fName.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0666);
        if (out < 0) {
          throw std::runtime_error("Could not open event list file " + fName);
        }

        ssize_t r = util::writeAll(
            out, reinterpret_cast<const unsigned char*>(_buffer.data()),
            _buffer.size() * sizeof(BoxVal));
        close(out);

        if (r < 0) {
          std::stringstream ss;
          ss << "Could not write event list file " << fName << "\n";
          ss << strerror(errno) << std::endl;
          throw std::runtime_error(ss.str());
        }
      }
      return;
    }

    flushToDisk();

    // clear buffer
    std::vector<BoxVal>().swap(_buffer);

    // on-disk merge sort
    std::string newFName =
        _persistOnDisk ? persistFName()
                       : util::getTmpFName(_cacheDir, _tmpPrefix, "sorttmp");
    int newFile = open(newFName.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0666);

    if (newFile < 0) {
      throw std::runtime_error("Could not open file " + newFName);
    }

    if (!_persistOnDisk) unlink(newFName.c_str());

#ifdef __unix__
    posix_fadvise(newFile, 0, 0, POSIX_FADV_SEQUENTIAL);
#endif
    ssize_t r = util::externalSort(_file, newFile, sizeof(BoxVal), _curSweepId,
                                   numThreads, boxCmp);

    if (r < 0) {
      std::stringstream ss;
      ss << "Could not sort events file\n";
      ss << strerror(errno) << std::endl;
      throw std::runtime_error(ss.str());
    }

    close(_file);

    _file = newFile;
  }

 private:
  std::string persistFName() const {
    std::string dir = _cacheDir == "<tmp>" ? util::getTmpDir() : _cacheDir;
    if (dir.size() && dir.back() != '/') dir += "/";
    return dir + _tmpPrefix + "events";
  }

  void swap(SweepEventList& other) noexcept {
    std::swap(_bufferSize, other._bufferSize);
    _buffer.swap(other._buffer);
    std::swap(_onDisk, other._onDisk);
    std::swap(_persistOnDisk, other._persistOnDisk);
    _cacheDir.swap(other._cacheDir);
    _tmpPrefix.swap(other._tmpPrefix);
    std::swap(_file, other._file);
    std::swap(_curSweepId, other._curSweepId);
  }

  // the maximum size of _buffer in bytes
  size_t _bufferSize;

  // the events not yet written to the events file. If no events were written
  // to it, the (finalized) event list is held here
  std::vector<BoxVal> _buffer;

  // any events on disk? If not, the final list is fully in memory
  bool _onDisk = false;

  bool _persistOnDisk = false;

  std::string _cacheDir;
  std::string _tmpPrefix;
  int _file;

  size_t _curSweepId = 0;
};

}  // namespace sj

#endif
