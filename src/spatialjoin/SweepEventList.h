// Copyright 2026, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_SWEEPEVENTLIST_H_
#define SPATIALJOINS_SWEEPEVENTLIST_H_

#include <fcntl.h>

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
  double areaOrLen;  // area or len
  util::geo::I32Point point;
  size_t numAnchors;      // for polygons and lines: number of anchor points,
                          // used for DUPLICATE REMOVAL
                          // for simple lines: the other endpoint, for folded
                          // box polygons: the other box corner, see
                          // packPoint()
  util::geo::I32Box b45;  // oriented bounding box
  bool side;

  GeomType getType() const {
    return static_cast<GeomType>(id >> ID_TYPE_SHIFT);
  }

  void setType(GeomType type) {
    id = (id & ID_MASK) | (static_cast<size_t>(type) << ID_TYPE_SHIFT);
  }
};

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

// the metadata of a prepared geometry, from which its sweep events are built
struct PreparedGeomMeta {
  // typed ID, assigned in GeometryStore::addBatch(), see typedId()
  size_t id;
  // the (padded) bounding box
  util::geo::I32Box bbox;
  GeomType type;
  // the (padded) bounding box of the geometry rotated by 45 degrees
  util::geo::I32Box box45;
  // the point of the IN event and of the OUT event, see BoxVal::point
  util::geo::I32Point point1, point2;
  // number of anchor points (for simple lines and folded box polygons, the
  // events instead store the packed other point, see packPoint())
  size_t numAnchors;
  // area for polygons, length for lines, 0 for points
  double areaOrLen;
  bool side;
};

// _____________________________________________________________________________
// returns the IN and the OUT event of a geometry with typed ID id (see
// typedId()), point1 and point2 are the points of the IN and the OUT event
inline std::pair<BoxVal, BoxVal> getSweepEvents(
    size_t id, const util::geo::I32Box& bbox, const util::geo::I32Box& box45,
    const util::geo::I32Point& point1, const util::geo::I32Point& point2,
    size_t numAnchors, double areaOrLen, bool side) {
  BoxVal in{id,
            bbox.getLowerLeft().getY(),
            bbox.getUpperRight().getY(),
            bbox.getLowerLeft().getX(),
            false,
            areaOrLen,
            point1,
            numAnchors,
            box45,
            side};
  BoxVal out{id,
             bbox.getLowerLeft().getY(),
             bbox.getUpperRight().getY(),
             bbox.getUpperRight().getX(),
             true,
             areaOrLen,
             point2,
             numAnchors,
             box45,
             side};

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
// returns the IN and the OUT event of a prepared geometry
inline std::pair<BoxVal, BoxVal> getSweepEvents(const PreparedGeomMeta& meta) {
  return getSweepEvents(meta.id, meta.bbox, meta.box45, meta.point1,
                        meta.point2, meta.numAnchors, meta.areaOrLen,
                        meta.side);
}

static int boxCmp(const void* a, const void* b) {
  const auto& boxa = static_cast<const BoxVal*>(a);
  const auto& boxb = static_cast<const BoxVal*>(b);
  if (boxa->val < boxb->val) return -1;
  if (boxa->val > boxb->val) return 1;

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
  if (boxa->side != boxb->side) return boxa->side ? 1 : -1;
  if (boxa->id != boxb->id) return boxa->id < boxb->id ? -1 : 1;

  return 0;
}

// buffer size _must_ be multiples of sizeof(BoxVal) and should hold at least
// one element
static const ssize_t BUFFER_S =
    ((16 * 1024 * 1024 + sizeof(BoxVal)) / sizeof(BoxVal)) * sizeof(BoxVal);

static const size_t RBUF_SIZE = 100000 * sizeof(BoxVal);

class SweepEventList {
 public:
  // reader for streaming the event list
  class Reader {
   public:
    Reader() : _file(-1) {}
    Reader(int file, size_t bufSize) : _file(file), _buf(bufSize) {}

    // returns 0 at the end of the event list. Returned pointer is
    // valid until a call to next() or reset().
    const BoxVal* next() {
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

    void reset() {
      _fileOffset = 0;
      _readPtr = 0;
      _readBlockLen = 0;
    }

   private:
    int _file;
    std::vector<unsigned char> _buf;
    size_t _fileOffset = 0;
    ssize_t _readPtr = 0;
    ssize_t _readBlockLen = 0;
  };

  class CombinedReader {
   public:
    CombinedReader(const SweepEventList* a, const SweepEventList* b = 0,
                   size_t bufSize = RBUF_SIZE)
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

   private:
    Reader _readerA;
    Reader _readerB;
    const BoxVal* _headA = 0;
    const BoxVal* _headB = 0;
    const BoxVal** _last = 0;
  };

  SweepEventList(const std::string& cacheDir, const std::string& tmpPrefix)
      : _obufpos(0), _cacheDir(cacheDir) {
    _outBuffer = new unsigned char[BUFFER_S];

    auto fname = util::getTmpFName(cacheDir, tmpPrefix, "events");
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

  ~SweepEventList() { close(_file); }

  void add(const BoxVal& bv) {
    memcpy(_outBuffer + _obufpos, &bv, sizeof(BoxVal));
    _obufpos += sizeof(BoxVal);

    if (_obufpos + sizeof(BoxVal) > BUFFER_S) {
      ssize_t r = util::writeAll(_file, _outBuffer, _obufpos);
      if (r < 0) {
        std::stringstream ss;
        ss << "Could not write to events file\n";
        ss << strerror(errno) << std::endl;
        throw std::runtime_error(ss.str());
      }
      _obufpos = 0;
    }
    _curSweepId++;
  }

  // adds the IN and the OUT event of a geometry with typed ID id (see
  // typedId()), point1 and point2 are the points of the IN and the OUT event
  void add(size_t id, const util::geo::I32Box& bbox,
           const util::geo::I32Box& box45, const util::geo::I32Point& point1,
           const util::geo::I32Point& point2, size_t numAnchors,
           double areaOrLen, bool side) {
    const auto& events = getSweepEvents(id, bbox, box45, point1, point2,
                                        numAnchors, areaOrLen, side);
    add(events.first);
    add(events.second);
  }

  // adds the IN and the OUT event of a prepared geometry
  void add(const PreparedGeomMeta& meta) {
    add(meta.id, meta.bbox, meta.box45, meta.point1, meta.point2,
        meta.numAnchors, meta.areaOrLen, meta.side);
  }

  size_t numObjects() const { return _curSweepId / 2; }
  size_t numEvents() const { return _curSweepId; }

  Reader newReader(size_t bufSize = RBUF_SIZE) const {
    // the buffer size must be a multiple of sizeof(BoxVal) and must
    // hold _at least_ 1 element
    if (bufSize < sizeof(BoxVal)) bufSize = sizeof(BoxVal);
    return Reader(_file, (bufSize / sizeof(BoxVal)) * sizeof(BoxVal));
  }

  void flush(size_t numThreads = 4) {
    ssize_t r = util::writeAll(_file, _outBuffer, _obufpos);
    if (r < 0) {
      std::stringstream ss;
      ss << "Could not write to events file\n";
      ss << strerror(errno) << std::endl;
      throw std::runtime_error(ss.str());
    }

    delete[] _outBuffer;

    _obufpos = 0;

    std::string newFName =
        util::getTmpFName(_cacheDir, ".spatialjoin", "sorttmp");
    int newFile = open(newFName.c_str(), O_RDWR | O_CREAT, 0666);
    unlink(newFName.c_str());

    if (newFile < 0) {
      throw std::runtime_error("Could not open temporary file " + newFName);
      exit(1);
    }

#ifdef __unix__
    posix_fadvise(newFile, 0, 0, POSIX_FADV_SEQUENTIAL);
#endif
    r = util::externalSort(_file, newFile, sizeof(BoxVal), _curSweepId,
                           numThreads, boxCmp);

    if (r < 0) {
      std::stringstream ss;
      ss << "Could not sort events file\n";
      ss << strerror(errno) << std::endl;
      throw std::runtime_error(ss.str());
    }

    fsync(newFile);

    close(_file);

    _file = newFile;

#ifdef __unix__
    posix_fadvise(_file, 0, 0, POSIX_FADV_SEQUENTIAL);
#endif
  }

 private:
  unsigned char* _outBuffer;
  ssize_t _obufpos;
  std::string _cacheDir;
  int _file;

  size_t _curSweepId = 0;
};

}  // namespace sj

#endif
