// Copyright 2026, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_SWEEPEVENTLIST_H_
#define SPATIALJOINS_SWEEPEVENTLIST_H_

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "BoxIds.h"
#include "SpatialJoinCfg.h"
#include "util/Misc.h"
#include "util/geo/Geo.h"

namespace sj {

class GeometryCache;

// The type of a geometry stored in this SweepEventList, this determines
// how geometries are reconstructed and/or retrieved from caches and stores
enum GeomType : uint8_t {
  POLYGON = 0,
  LINE = 1,
  POINT = 2,
  SIMPLE_LINE_RISING = 3,
  SIMPLE_POLYGON = 4,
  FOLDED_POINT = 5,
  FOLDED_SIMPLE_LINE_RISING = 6,  // line segment goin up
  FOLDED_BOX_POLYGON = 7,
  DELETED = 8,
  SELF_CHECK = 9,
  SELF_CHECK_AREA = 10,
  SELF_CHECK_LINE = 11,
  SELF_CHECK_POINT = 12,
  MULTI = 13,
  SIMPLE_LINE_FALLING = 14,  // line segment going down
  FOLDED_SIMPLE_LINE_FALLING = 15
};

// ture if a line is folded
inline bool isFoldedSimpleLine(GeomType t) {
  return t == FOLDED_SIMPLE_LINE_RISING || t == FOLDED_SIMPLE_LINE_FALLING;
}

// true if a line is simple
inline bool isSimpleLine(GeomType t) {
  return t == SIMPLE_LINE_RISING || t == SIMPLE_LINE_FALLING ||
         isFoldedSimpleLine(t);
}

// the upper 4 bits of a BoxVal ID hold the geometry type, see typedId()
const static size_t ID_TYPE_SHIFT = 56;
const static size_t ID_MASK = (size_t(1) << ID_TYPE_SHIFT) - 1;

// the geometry type of a typed ID, see typedId()
inline GeomType idType(size_t id) {
  return static_cast<GeomType>(id >> ID_TYPE_SHIFT);
}

// a folded gid stores its length in the byte
// directly below the type bits, and up to FOLD_MAX_LEN chars below that
const static size_t FOLD_LEN_SHIFT = ID_TYPE_SHIFT - 8;
const static size_t FOLD_MAX_LEN = FOLD_LEN_SHIFT / 8;

// sin and cos of 45 degrees, for the diagonal boxes
const static double SIN45 = 1.0 / sqrt(2);
const static double COS45 = 1.0 / sqrt(2);

// the number of anchor points stored in a BoxVal is capped at this value
const static size_t MAX_EVENT_NUM_ANCHORS =
    std::numeric_limits<uint16_t>::max();

// a sweep event (IN or OUT) of a geometry, with various meta informations
class BoxVal {
 public:
  BoxVal() = default;
  BoxVal(size_t id, int32_t loY, int32_t upY, int32_t val, bool out,
         uint8_t storeId, const util::geo::I32Point& point)
      : _id(id),
        _loY(loY),
        _upY(upY),
        _val(val),
        _point(point),
        _out(out),
        _storeId(storeId),
        _numAnchors(0),
        _ext{0, 0, 0, 0} {}

  // typed ID, the lower 56 bits are the dist offset returned from the
  // GeometryStore, bits 57-60 are geometry type
  size_t id() const { return _id; }
  void setId(size_t id) { _id = id; }

  // the lower Y value of the box
  int32_t loY() const { return _loY; }
  void setLoY(int32_t loY) { _loY = loY; }

  // the upper Y value of the box
  int32_t upY() const { return _upY; }
  void setUpY(int32_t upY) { _upY = upY; }

  // the X value of the event (left X of the box for IN events, right X for
  // OUT events)
  int32_t val() const { return _val; }
  void setVal(int32_t val) { _val = val; }

  // the point payload of this event. For simple lines its the endpoint, for
  // folded box polygons the box corner
  const util::geo::I32Point& point() const { return _point; }

  // the type if this event's geom
  GeomType type() const { return idType(_id); }
  void setType(GeomType type) {
    _id = (_id & ID_MASK) | (static_cast<size_t>(type) << ID_TYPE_SHIFT);
  }

  // whether this is an OUT event
  bool out() const { return _out; }

  // whether this is an IN event
  bool in() const { return !_out; }

  // the store the geometry is in
  uint8_t storeId() const { return _storeId; }
  void setStoreId(uint8_t storeId) { _storeId = storeId; }

  // for polygons and lines: the number of anchor points (capped at
  // MAX_EVENT_NUM_ANCHORS), 0 for other types.
  size_t numAnchors() const { return _numAnchors; }
  void setNumAnchors(size_t numAnchors) {
    if (!hasB45()) return;
    _numAnchors = std::min(numAnchors, MAX_EVENT_NUM_ANCHORS);
  }

  // the other payload point. For simple lines the
  // second endpoint, for folded box polygons the second box corner
  util::geo::I32Point otherPoint() const {
    if (!hasB45()) return {_ext[0], _ext[1]};
    return {_val, _point.getY() == _loY ? _upY : _loY};
  }
  void setOtherPoint(const util::geo::I32Point& p) {
    if (hasB45()) return;
    _ext[0] = p.getX();
    _ext[1] = p.getY();
  }

  // the bounding box of the geometry rotated by 45 degs
  util::geo::I32Box b45() const {
    if (hasB45()) return {{_ext[0], _ext[1]}, {_ext[2], _ext[3]}};

    double pad = b45Pad();
    if (pad < 0) return {};

    const auto& a = _point;
    const auto& b = otherPoint();
    util::geo::I32Box ret;
    ret = util::geo::extendBox(rot45(a), ret);
    ret = util::geo::extendBox(rot45(b), ret);
    if (type() == FOLDED_BOX_POLYGON) {
      ret = util::geo::extendBox(rot45({a.getX(), b.getY()}), ret);
      ret = util::geo::extendBox(rot45({b.getX(), a.getY()}), ret);
    }

    if (pad > 0) return util::geo::pad(ret, pad);
    return ret;
  }

  void setB45(const util::geo::I32Box& b45) {
    if (!hasB45()) {
      setB45Pad(b45.isNull() ? -1 : 0);
      return;
    }
    _ext[0] = b45.getLowerLeft().getX();
    _ext[1] = b45.getLowerLeft().getY();
    _ext[2] = b45.getUpperRight().getX();
    _ext[3] = b45.getUpperRight().getY();
  }

  // pads the rotated bounding box by pad
  void padB45(double pad) {
    if (hasB45()) {
      setB45(util::geo::pad(b45(), pad));
      return;
    }
    // pad only computed boxes, keep a missing box missing
    if (b45Pad() >= 0) setB45Pad(b45Pad() + pad);
  }

  bool hasB45() const {
    return !(isSimpleLine(type()) || type() == FOLDED_BOX_POLYGON);
  }

 private:
  static util::geo::I32Point rot45(const util::geo::I32Point& p) {
    return util::geo::rotateSinCos(p, SIN45, COS45, util::geo::I32Point(0, 0));
  }

  // the padding of a computed rotated bounding box, < 0 if there is none
  double b45Pad() const {
    double pad;
    memcpy(&pad, &_ext[2], sizeof(double));
    return pad;
  }

  void setB45Pad(double pad) { memcpy(&_ext[2], &pad, sizeof(double)); }

  size_t _id;
  int32_t _loY;
  int32_t _upY;
  int32_t _val;
  util::geo::I32Point _point;

  bool _out;
  uint8_t _storeId;
  uint16_t _numAnchors;

  // if not hasB45(): the other point in (_ext[0], _ext[1]) and the padding
  // of the rotated bounding box in (a double in _ext[2], _ext[3]), else: the
  // rotated bounding box directly
  int32_t _ext[4];
};

struct PreparedGeomMeta {
  size_t id;               // typed ID
  util::geo::I32Box bbox;  // bounding box (unpadded!!)
  util::geo::I32Box box45;  // rotated bounding box (unpadded!!)
  // number of anchor points (only stored in the events of polygons and lines)
  size_t numAnchors;

  GeomType getType() const { return idType(id); }

  util::geo::DBox getBoundingBox(util::geo::CRSType crs) const {
    if (bbox.isNull()) return {};
    util::geo::DPoint ll(bbox.getLowerLeft().getX() * 1.0 / PREC,
                         bbox.getLowerLeft().getY() * 1.0 / PREC);
    util::geo::DPoint ur(bbox.getUpperRight().getX() * 1.0 / PREC,
                         bbox.getUpperRight().getY() * 1.0 / PREC);
    return {util::geo::projectToCRS(ll, util::geo::CRSType::WEB_MERCATOR, crs),
            util::geo::projectToCRS(ur, util::geo::CRSType::WEB_MERCATOR, crs)};
  }
};

typedef std::function<void(size_t t, const std::string& gid,
                           const PreparedGeomMeta&)>
    PreparedGeomMetaCb;

void padSweepEventForDist(BoxVal* bv, const SpatialJoinCfg& cfg);

// default size of the write buffer of a SweepEventList, must be multiple of
// sizeof(BoxVal)
static const size_t DEFAULT_BUFFER_S =
    ((16 * 1024 * 1024 + sizeof(BoxVal)) / sizeof(BoxVal)) * sizeof(BoxVal);

// default max memory used by the sort
static const size_t DEFAULT_SORT_MEM_S = 512 * 1024 * 1024;

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
inline size_t getId(const BoxVal& bv) { return untypedId(bv.id()); }

// _____________________________________________________________________________
inline double boxPadding(const util::geo::I32Box& refBox,
                         const SpatialJoinCfg& cfg) {
  // the padding of the boxes of geometries near refBox for within-distance
  // joins
  if (cfg.withinDist < 0) return 0;

  if (cfg.euclideanDist && !cfg.haversineApprox)
    return cfg.withinDist * PREC / 2.0;

  // convert distanceUpperBound (meters) to maximum latitude padding (degrees)
  // we have to "pad" the box by -dy and dy because the distance path could
  // be within that padded box, and thus the distortions have to be computed
  // based on that path
  double dLat =
      cfg.withinDist / util::geo::MIN_METERS_PER_LAT_RAD * util::geo::IRAD;
  auto upper = util::geo::webMercToLatLng<double>(
      0.0, refBox.getUpperRight().getY() * 1.0 / PREC);
  auto lower = util::geo::webMercToLatLng<double>(
      0.0, refBox.getLowerLeft().getY() * 1.0 / PREC);

  double scaleLatUp =
      cos(std::min(90.0 - util::geo::EPSILON, (upper.getY() + dLat)) *
          util::geo::RAD);
  double scaleLatLow =
      cos(std::max(-90.0 + util::geo::EPSILON, (lower.getY() - dLat)) *
          util::geo::RAD);
  double scaleFactor = std::min(scaleLatUp, scaleLatLow);

  return (cfg.withinDist / 2.0) / scaleFactor * PREC;
}

// _____________________________________________________________________________
inline util::geo::I32Box padBox(const util::geo::I32Box& box,
                                const util::geo::I32Box& refBox,
                                const SpatialJoinCfg& cfg) {
  if (cfg.withinDist < 0 || box.isNull()) return box;

  double pad = boxPadding(refBox, cfg);

  if (cfg.euclideanDist && !cfg.haversineApprox)
    return util::geo::pad(box, pad);

  double llx = box.getLowerLeft().getX();
  double lly = box.getLowerLeft().getY();
  double urx = box.getUpperRight().getX();
  double ury = box.getUpperRight().getY();

  double m = sj::boxids::WORLD_W / 2.0;

  // restrict padding to world extent
  int32_t llxt = -m;
  int32_t llyt = -m;
  int32_t urxt = m;
  int32_t uryt = m;

  if (llx - pad > -m) llxt = llx - pad;
  if (lly - pad > -m) llyt = lly - pad;
  if (urx + pad < m) urxt = urx + pad;
  if (ury + pad < m) uryt = ury + pad;

  return {{llxt, llyt}, {urxt, uryt}};
}

// _____________________________________________________________________________
inline void padSweepEvent(BoxVal* bv, const SpatialJoinCfg& cfg) {
  if (cfg.withinDist >= 0) padSweepEventForDist(bv, cfg);
}

// _____________________________________________________________________________
inline std::pair<BoxVal, BoxVal> getSweepEvents(
    size_t id, const util::geo::I32Box& bbox, const util::geo::I32Box& box45,
    size_t numAnchors, uint8_t storeId,
    const util::geo::I32Point& rightPoint = {}) {
  const auto& ll = bbox.getLowerLeft();
  const auto& ur = bbox.getUpperRight();

  util::geo::I32Point point1, point2;
  switch (idType(id)) {
    case POINT:
    case FOLDED_POINT:
      point1 = ll;
      point2 = ll;
      break;
    case FOLDED_BOX_POLYGON:
    case SIMPLE_LINE_RISING:
    case FOLDED_SIMPLE_LINE_RISING:
      point1 = ur;
      point2 = ll;
      break;
    case SIMPLE_LINE_FALLING:
    case FOLDED_SIMPLE_LINE_FALLING:
      point1 = {ur.getX(), ll.getY()};
      point2 = {ll.getX(), ur.getY()};
      break;
    default:
      point2 = rightPoint;
  }

  BoxVal in(id, ll.getY(), ur.getY(), ll.getX(), false, storeId, point1);
  BoxVal out(id, ll.getY(), ur.getY(), ur.getX(), true, storeId, point2);

  in.setNumAnchors(numAnchors);
  out.setNumAnchors(numAnchors);

  // simple lines and folded box polygons store the point of the other event,
  // see otherPoint()
  in.setOtherPoint(point2);
  out.setOtherPoint(point1);

  in.setB45(box45);
  out.setB45(box45);

  return {in, out};
}

// _____________________________________________________________________________
inline std::pair<BoxVal, BoxVal> getSweepEvents(
    const PreparedGeomMeta& meta, uint8_t storeId,
    const util::geo::I32Point& rightPoint = {}) {
  return getSweepEvents(meta.id, meta.bbox, meta.box45, meta.numAnchors,
                        storeId, rightPoint);
}

// _____________________________________________________________________________
static int boxCmp(const void* a, const void* b) {
  const auto& boxa = static_cast<const BoxVal*>(a);
  const auto& boxb = static_cast<const BoxVal*>(b);
  if (boxa->val() < boxb->val()) return -1;
  if (boxa->val() > boxb->val()) return 1;

  // special multi events before everything else
  if (boxa->type() == MULTI && boxb->type() != MULTI) return -1;
  if (boxa->type() != MULTI && boxb->type() == MULTI) return 1;

  if (!boxa->out() && boxb->out()) return -1;
  if (boxa->out() && !boxb->out()) return 1;

  // everything before a polygon
  if (boxa->type() != POLYGON && boxa->type() != SIMPLE_POLYGON &&
      boxa->type() != FOLDED_BOX_POLYGON &&
      (boxb->type() == POLYGON || boxb->type() == SIMPLE_POLYGON ||
       boxb->type() == FOLDED_BOX_POLYGON))
    return -1;
  if ((boxa->type() == POLYGON || boxa->type() == SIMPLE_POLYGON ||
       boxa->type() == FOLDED_BOX_POLYGON) &&
      boxb->type() != POLYGON && boxb->type() != SIMPLE_POLYGON &&
      boxb->type() != FOLDED_BOX_POLYGON)
    return 1;

  // points before lines
  if ((boxa->type() == POINT || boxa->type() == FOLDED_POINT) &&
      (isSimpleLine(boxb->type()) || boxb->type() == LINE))
    return -1;
  if ((boxb->type() == POINT || boxb->type() == FOLDED_POINT) &&
      (isSimpleLine(boxa->type()) || boxa->type() == LINE))
    return 1;

  // make ordernig of equal geoms deterministic
  if (boxa->type() != boxb->type())
    return boxa->type() < boxb->type() ? -1 : 1;
  if (boxa->id() != boxb->id()) return boxa->id() < boxb->id() ? -1 : 1;

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
  SweepEventList(const SpatialJoinCfg& cfg, const std::string& cacheDir,
                 const std::string& tmpPrefix,
                 size_t bufferSize = DEFAULT_BUFFER_S,
                 bool persistOnDisk = false)
      : _cfg(cfg),
        _bufferSize(std::max(bufferSize / sizeof(BoxVal), size_t(1)) *
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

    // the references created by duplicatesToReferences(), if any
    readRefs(refsFName(path));
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

  // adds bv as given, without any padding
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
           const util::geo::I32Box& box45, size_t numAnchors,
           uint8_t storeId) {
    auto events = getSweepEvents(id, bbox, box45, numAnchors, storeId);
    padSweepEvent(&events.first, _cfg);
    padSweepEvent(&events.second, _cfg);
    add(events.first);
    add(events.second);
  }

  void add(size_t id, const util::geo::I32Box& bbox, uint8_t storeId) {
    add(id, bbox, util::geo::I32Box(), 0, storeId);
  }

  void add(size_t id, const util::geo::DBox& bbox, util::geo::CRSType boxCrs,
           uint8_t storeId) {
    auto ll = util::geo::projectToWebMerc(bbox.getLowerLeft(), boxCrs);
    auto ur = util::geo::projectToWebMerc(bbox.getUpperRight(), boxCrs);
    add(id,
        util::geo::I32Box({static_cast<int>(std::floor(ll.getX() * PREC)),
                           static_cast<int>(std::floor(ll.getY() * PREC))},
                          {static_cast<int>(std::ceil(ur.getX() * PREC)),
                           static_cast<int>(std::ceil(ur.getY() * PREC))}),
        storeId);
  }

  // add the IN and OUT event of meta (with unpadded boxes) to this list, using
  // store storeId. The boxes are padded for within-distance joins from the cfg
  void add(const PreparedGeomMeta& meta, uint8_t storeId) {
    add(meta.id, meta.bbox, meta.box45, meta.numAnchors, storeId);
  }

  // thread safe callback for `WKTParser()`
  PreparedGeomMetaCb addCb(uint8_t storeId) {
    return [this, storeId](size_t, const std::string&,
                           const PreparedGeomMeta& meta) {
      std::lock_guard<std::mutex> lock(_addMtx);
      add(meta, storeId);
    };
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

  void finalize(size_t numThreads = 4,
                size_t maxSortMem = DEFAULT_SORT_MEM_S) {
    if (!_onDisk) {
      // in-memory sort
      std::sort(
          _buffer.begin(), _buffer.end(),
          [](const BoxVal& a, const BoxVal& b) { return boxCmp(&a, &b) < 0; });

      if (_persistOnDisk) {
        // remove stale references of a previous list persisted to the same file
        unlink(refsFName(persistFName()).c_str());
        writeBufferToPersistFile();
      }
      return;
    }

    if (_persistOnDisk) unlink(refsFName(persistFName()).c_str());

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
                                   numThreads, maxSortMem, boxCmp);

    if (r < 0) {
      std::stringstream ss;
      ss << "Could not sort events file\n";
      ss << strerror(errno) << std::endl;
      throw std::runtime_error(ss.str());
    }

    close(_file);

    _file = newFile;
  }

  void duplicatesToReferences(GeometryCache& cache);

  // whether duplicatesToReferences() created any references
  bool hasRefs() const { return _hasRefs; }

  // whether geometry gid of store storeId is referenced by duplicates
  bool isRefed(uint8_t storeId, const std::string& gid) const {
    return storeId < _refs.size() && _refs[storeId].count(gid);
  }

  // the duplicates (gid -> subId) of geometry (gid, subId) of store storeId, or
  // nullptr if there are none
  const std::unordered_map<std::string, size_t>* getRefs(
      uint8_t storeId, const std::string& gid, size_t subId) const {
    if (storeId >= _refs.size()) return nullptr;
    auto i = _refs[storeId].find(gid);
    if (i == _refs[storeId].end()) return nullptr;
    auto j = i->second.find(subId);
    if (j == i->second.end()) return nullptr;
    return &j->second;
  }

  // total number of duplicates replaced by references
  size_t numReferences() const;

 private:
  // referenced geometry gid -> its subId -> duplicates (gid -> subId)
  typedef std::unordered_map<
      std::string,
      std::unordered_map<size_t, std::unordered_map<std::string, size_t>>>
      RefMap;

  struct DuplicateState {
    std::unordered_set<size_t> deletedLines;
    std::unordered_set<size_t> referencedLines;
    std::unordered_set<size_t> deletedPolys;
    std::unordered_set<size_t> referencedPolys;
    std::unordered_map<size_t, size_t> duplicatePolys, duplicateLines;
    int32_t curX = 0;
    size_t numEvents = 0;
  };

  bool duplicatesToReferences(BoxVal* events, size_t n, GeometryCache& cache,
                              DuplicateState& state);

  // duplicatesToReferences(GeometryCache&) for an event list on disk
  void duplicatesOnDiskToReferences(GeometryCache& cache,
                                    DuplicateState& state);

  // registers geometry (gid, subId) of store storeId as a duplicate of
  // (refGid, refSubId)
  void addReference(uint8_t storeId, const std::string& refGid,
                    size_t refSubId, const std::string& gid, size_t subId);

  static size_t duplicateId(uint8_t storeId, size_t v) {
    return (static_cast<size_t>(storeId) << ID_TYPE_SHIFT) | untypedId(v);
  }

  static std::string refsFName(const std::string& path) {
    return path + "refs";
  }

  void writeRefs(const std::string& fName) const;
  void readRefs(const std::string& fName);

  // writes the (in-memory) _buffer to persistFName()
  void writeBufferToPersistFile() const {
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

  std::string persistFName() const {
    std::string dir = _cacheDir == "<tmp>" ? util::getTmpDir() : _cacheDir;
    if (dir.size() && dir.back() != '/') dir += "/";
    return dir + _tmpPrefix + "events";
  }

  void swap(SweepEventList& other) noexcept {
    std::swap(_cfg, other._cfg);
    std::swap(_bufferSize, other._bufferSize);
    _buffer.swap(other._buffer);
    std::swap(_onDisk, other._onDisk);
    std::swap(_persistOnDisk, other._persistOnDisk);
    _cacheDir.swap(other._cacheDir);
    _tmpPrefix.swap(other._tmpPrefix);
    std::swap(_file, other._file);
    std::swap(_curSweepId, other._curSweepId);
    _refs.swap(other._refs);
    std::swap(_hasRefs, other._hasRefs);
  }

  SpatialJoinCfg _cfg;

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

  // references created by duplicatesToReferences(), indexed by store id
  std::vector<RefMap> _refs;

  bool _hasRefs = false;

  // guards the addCb()
  std::mutex _addMtx;
};

}  // namespace sj

#endif
