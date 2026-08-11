// Copyright 2026, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_SWEEPEVENTLIST_H_
#define SPATIALJOINS_SWEEPEVENTLIST_H_

#include <fcntl.h>

#include "util/Misc.h"

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
  SELF_CHECK_POINT = 12
};

struct BoxVal {
  size_t id;     // the ID returned from the GeometryCache (offset into file)
  int32_t loY;   // the lower Y value of the box
  int32_t upY;   // the upper Y value of the box
  int32_t val;   // the left X value of the box
  bool out : 1;  // whether this is an IN or OUT event
  GeomType type : 4;  // geometry type
  double areaOrLen;   // area or len
  util::geo::I32Point point;
  size_t numAnchors;      // DUPLICATE REMOVAL: used as hash value
  util::geo::I32Box b45;  // oriented bounding box
  bool side;
  bool large;
  int32_t size;  // DUPLICATE REMOVAL: size of geom
};

static int boxCmp(const void* a, const void* b) {
  const auto& boxa = static_cast<const BoxVal*>(a);
  const auto& boxb = static_cast<const BoxVal*>(b);
  if (boxa->val < boxb->val) return -1;
  if (boxa->val > boxb->val) return 1;

  if (!boxa->out && boxb->out) return -1;
  if (boxa->out && !boxb->out) return 1;

  // everything before a polygon
  if (boxa->type != POLYGON && boxa->type != SIMPLE_POLYGON &&
      boxa->type != FOLDED_BOX_POLYGON &&
      (boxb->type == POLYGON || boxb->type == SIMPLE_POLYGON ||
       boxb->type == FOLDED_BOX_POLYGON))
    return -1;
  if ((boxa->type == POLYGON || boxa->type == SIMPLE_POLYGON ||
       boxa->type == FOLDED_BOX_POLYGON) &&
      boxb->type != POLYGON && boxb->type != SIMPLE_POLYGON &&
      boxb->type != FOLDED_BOX_POLYGON)
    return 1;

  // points before lines
  if ((boxa->type == POINT || boxa->type == FOLDED_POINT) &&
      (boxb->type == SIMPLE_LINE || boxb->type == LINE ||
       boxb->type == FOLDED_SIMPLE_LINE))
    return -1;
  if ((boxb->type == POINT || boxb->type == FOLDED_POINT) &&
      (boxa->type == SIMPLE_LINE || boxa->type == LINE ||
       boxa->type == FOLDED_SIMPLE_LINE))
    return 1;

  // smaller polygons before larger
  if ((boxa->type == POLYGON || boxa->type == SIMPLE_POLYGON ||
       boxa->type == FOLDED_BOX_POLYGON) &&
      (boxb->type == POLYGON || boxb->type == SIMPLE_POLYGON ||
       boxb->type == FOLDED_BOX_POLYGON) &&
      boxa->areaOrLen < boxb->areaOrLen)
    return -1;
  if ((boxa->type == POLYGON || boxa->type == SIMPLE_POLYGON ||
       boxa->type == FOLDED_BOX_POLYGON) &&
      (boxb->type == POLYGON || boxb->type == SIMPLE_POLYGON ||
       boxb->type == FOLDED_BOX_POLYGON) &&
      boxa->areaOrLen > boxb->areaOrLen)
    return 1;

  // shorter lines before longer
  if ((boxa->type == LINE || boxa->type == SIMPLE_LINE ||
       boxa->type == FOLDED_SIMPLE_LINE) &&
      (boxb->type == LINE || boxb->type == SIMPLE_LINE ||
       boxb->type == FOLDED_SIMPLE_LINE) &&
      boxa->areaOrLen < boxb->areaOrLen)
    return -1;
  if ((boxa->type == LINE || boxa->type == SIMPLE_LINE ||
       boxa->type == FOLDED_SIMPLE_LINE) &&
      (boxb->type == LINE || boxb->type == SIMPLE_LINE ||
       boxb->type == FOLDED_SIMPLE_LINE) &&
      boxa->areaOrLen > boxb->areaOrLen)
    return 1;

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
    Reader(int file, size_t bufSize) : _file(file), _buf(bufSize) {}

    // returns 0 at the end of the event list. Returned pointer is
    // valid until a call to next() or reset().
    const BoxVal* next() {
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

  SweepEventList(const std::string& cacheDir, const std::string& tmpPrefix,
                 size_t numThreads)
      : _obufpos(0), _cacheDir(cacheDir), _numThreads(numThreads) {
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

  size_t numObjects() const { return _curSweepId / 2; }
  size_t numEvents() const { return _curSweepId; }

  Reader newReader(size_t bufSize = RBUF_SIZE) const {
    // the buffer size must be a multiple of sizeof(BoxVal) and must
    // hold _at least_ 1 element
    if (bufSize < sizeof(BoxVal)) bufSize = sizeof(BoxVal);
    return Reader(_file, (bufSize / sizeof(BoxVal)) * sizeof(BoxVal));
  }

  void flush() {
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
                           _numThreads, boxCmp);

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
  size_t _numThreads;

  size_t _curSweepId = 0;
};

}  // namespace sj

#endif
