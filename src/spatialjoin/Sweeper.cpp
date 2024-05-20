#include <bzlib.h>
#include <stdio.h>
#include <unistd.h>
#include <zlib.h>

#include <algorithm>
#include <cassert>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <set>

#include "BoxIds.h"
#include "InnerOuter.h"
#include "IntervalIdx.h"
#include "Sweeper.h"
#include "util/Misc.h"
#include "util/log/Log.h"

using sj::GeomType;
using sj::Sweeper;
using sj::boxids::boxIdIsect;
using sj::boxids::BoxIdList;
using sj::boxids::getBoxId;
using sj::boxids::getBoxIds;
using sj::boxids::packBoxIds;
using sj::innerouter::Mode;
using util::writeAll;
using util::geo::area;
using util::geo::getBoundingBox;
using util::geo::I32Box;
using util::geo::I32Line;
using util::geo::I32MultiLine;
using util::geo::I32MultiPoint;
using util::geo::I32MultiPolygon;
using util::geo::I32Point;
using util::geo::I32Polygon;
using util::geo::I32XSortedLine;
using util::geo::I32XSortedPolygon;
using util::geo::intersectsContainsCovers;
using util::geo::intersectsCovers;
using util::geo::intersectsLineStrict;
using util::geo::LineSegment;
using util::geo::webMercToLatLng;

const static size_t CUTOUTS_MIN_SIZE = 100;
const static size_t OBB_MIN_SIZE = 100;

const static double sin45 = 1.0 / sqrt(2);
const static double cos45 = 1.0 / sqrt(2);

// _____________________________________________________________________________
void Sweeper::add(const I32MultiPolygon& a, const std::string& gid) {
  uint16_t subid = 0;  // a subid of 0 means "single polygon"
  if (a.size() > 1) subid = 1;

  add(a, gid, subid);
}

// _____________________________________________________________________________
void Sweeper::add(const I32MultiLine& a, const std::string& gid) {
  uint16_t subid = 0;  // a subid of 0 means "single line"
  if (a.size() > 1) subid = 1;

  add(a, gid, subid);
}

// _____________________________________________________________________________
void Sweeper::addMp(const I32MultiPoint& a, const std::string& gid) {
  uint16_t subid = 0;  // a subid of 0 means "single point"
  if (a.size() > 1) subid = 1;

  addMp(a, gid, subid);
}

// _____________________________________________________________________________
size_t Sweeper::add(const I32MultiPolygon& a, const std::string& gid,
                    size_t subId) {
  for (const auto& poly : a) {
    if (poly.getOuter().size() < 2) continue;
    add(poly, gid, subId);
    if (subId > 0) _subSizes[gid] = _subSizes[gid] + 1;
    subId++;
  }
  return subId;
}

// _____________________________________________________________________________
size_t Sweeper::add(const I32MultiLine& a, const std::string& gid,
                    size_t subId) {
  for (const auto& line : a) {
    if (line.size() < 2) continue;
    add(line, gid, subId);
    if (subId > 0) _subSizes[gid] = _subSizes[gid] + 1;
    subId++;
  }
  return subId;
}

// _____________________________________________________________________________
size_t Sweeper::addMp(const I32MultiPoint& a, const std::string& gid,
                      size_t subid) {
  if (subid > 0) {
    _subSizes[gid] = _subSizes[gid] + a.size();
  }
  size_t newId = subid;
  for (const auto& point : a) {
    add(point, gid, newId);
    newId++;
  }
  return newId;
}

// _____________________________________________________________________________
void Sweeper::multiAdd(const std::string& gid, int32_t xLeft, int32_t xRight) {
  auto i = _multiGidToId.find(gid);

  if (i == _multiGidToId.end()) {
    _multiIds.push_back(gid);
    _multiRightX.push_back(xRight);
    _multiLeftX.push_back(xLeft);
    _multiGidToId[gid] = _multiIds.size() - 1;
  } else {
    size_t id = _multiGidToId[gid];
    if (xRight > _multiRightX[id]) _multiRightX[id] = xRight;
    if (xLeft < _multiLeftX[id]) _multiLeftX[id] = xLeft;
  }
}

// _____________________________________________________________________________
void Sweeper::add(const I32Polygon& poly, const std::string& gid) {
  add(poly, gid, 0);
}

// _____________________________________________________________________________
void Sweeper::add(const I32Polygon& poly, const std::string& gid,
                  size_t subid) {
  const auto& box = getBoundingBox(poly);
  I32XSortedPolygon spoly(poly);

  double areaSize = area(poly);
  double outerAreaSize = outerArea(poly);
  BoxIdList boxIds;
  std::map<int32_t, size_t> cutouts;

  if (_cfg.useBoxIds) {
    if (_cfg.useCutouts && poly.getOuter().size() > CUTOUTS_MIN_SIZE) {
      boxIds = packBoxIds(getBoxIds(spoly, box, areaSize, &cutouts));
    } else {
      boxIds = packBoxIds(getBoxIds(spoly, box, areaSize, 0));
    }
  }

  I32Box box45;
  if (_cfg.useDiagBox) {
    auto polyR = util::geo::rotateSinCos(poly, sin45, cos45, I32Point(0, 0));
    box45 = getBoundingBox(polyR);
  }

  if (poly.getInners().size() == 0 && poly.getOuter().size() < 10 &&
      subid == 0 && (!_cfg.useBoxIds || boxIds.front().first == 1)) {
    size_t id = _simpleAreaCache.add({poly.getOuter(), gid});

    diskAdd({id, box.getLowerLeft().getY(), box.getUpperRight().getY(),
             box.getLowerLeft().getX(), false, SIMPLE_POLYGON, areaSize,
             box45});
    diskAdd({id, box.getLowerLeft().getY(), box.getUpperRight().getY(),
             box.getUpperRight().getX(), true, SIMPLE_POLYGON, areaSize,
             box45});
  } else {
    if (!_cfg.useFastSweepSkip) {
      spoly.setInnerMaxSegLen(std::numeric_limits<int32_t>::max());
      spoly.getOuter().setMaxSegLen(std::numeric_limits<int32_t>::max());
      for (auto& inner : spoly.getInners()) {
        inner.setMaxSegLen(std::numeric_limits<int32_t>::max());
      }
    }

    I32XSortedPolygon inner, outer;
    I32Box innerBox, outerBox;
    double outerOuterAreaSize = 0;
    double innerOuterAreaSize = 0;

    if (_cfg.useInnerOuter) {
      const auto& innerPoly =
          sj::innerouter::simplifiedPoly<Mode::INNER>(poly, 1 / (3.14 * 20));
      const auto& outerPoly =
          sj::innerouter::simplifiedPoly<Mode::OUTER>(poly, 1 / (3.14 * 20));

      innerBox = getBoundingBox(innerPoly);
      outerBox = getBoundingBox(outerPoly);

      innerOuterAreaSize = outerArea(innerPoly);
      outerOuterAreaSize = outerArea(outerPoly);

      inner = innerPoly;
      outer = outerPoly;
    }

    util::geo::I32Polygon obb;

    if (_cfg.useOBB && poly.getOuter().size() >= OBB_MIN_SIZE) {
      obb = util::geo::convexHull(
          util::geo::pad(util::geo::getOrientedEnvelope(poly), 10));

      // drop redundant oriented bbox
      if (obb.getOuter().size() >= poly.getOuter().size()) obb = {};
    }

    if (subid > 0)
      multiAdd(gid, box.getLowerLeft().getX(), box.getUpperRight().getX());

    size_t id = _areaCache.add(
        {spoly, box, gid, subid, areaSize, _cfg.useArea ? outerAreaSize : 0,
         boxIds, cutouts, obb, inner, innerBox, innerOuterAreaSize, outer,
         outerBox, outerOuterAreaSize});

    diskAdd({id, box.getLowerLeft().getY(), box.getUpperRight().getY(),
             box.getLowerLeft().getX(), false, POLYGON, areaSize, box45});
    diskAdd({id, box.getLowerLeft().getY(), box.getUpperRight().getY(),
             box.getUpperRight().getX(), true, POLYGON, areaSize, box45});
  }

  if (_curSweepId / 2 % 1000000 == 0)
    LOGTO(INFO, std::cerr) << "@ " << _curSweepId / 2;
}

// _____________________________________________________________________________
void Sweeper::add(const I32Line& line, const std::string& gid) {
  add(line, gid, 0);
}

// _____________________________________________________________________________
void Sweeper::add(const I32Line& line, const std::string& gid, size_t subid) {
  if (line.size() < 2) return;

  const auto& box = getBoundingBox(line);
  BoxIdList boxIds;
  std::map<int32_t, size_t> cutouts;

  if (_cfg.useBoxIds) {
    if (_cfg.useCutouts && line.size() > CUTOUTS_MIN_SIZE) {
      boxIds = packBoxIds(getBoxIds(line, box, &cutouts));
    } else {
      boxIds = packBoxIds(getBoxIds(line, box, 0));
    }
  }

  const double len = util::geo::len(line);

  I32Box box45;
  if (_cfg.useDiagBox) {
    auto polyR = util::geo::rotateSinCos(line, sin45, cos45, I32Point(0, 0));
    box45 = getBoundingBox(polyR);
  }

  if (subid > 0)
    multiAdd(gid, box.getLowerLeft().getX(), box.getUpperRight().getX());

  if (line.size() == 2 && (!_cfg.useBoxIds || boxIds.front().first == 1) &&
      subid == 0) {
    // simple line
    size_t id = _simpleLineCache.add({line.front(), line.back(), gid});

    diskAdd({id, box.getLowerLeft().getY(), box.getUpperRight().getY(),
             box.getLowerLeft().getX(), false, SIMPLE_LINE, len, box45});
    diskAdd({id, box.getLowerLeft().getY(), box.getUpperRight().getY(),
             box.getUpperRight().getX(), true, SIMPLE_LINE, len, box45});
  } else {
    // normal line
    I32XSortedLine sline(line);
    util::geo::I32Polygon obb;
    if (_cfg.useOBB && line.size() >= OBB_MIN_SIZE) {
      obb = util::geo::convexHull(
          util::geo::pad(util::geo::getOrientedEnvelope(line), 10));

      // drop redundant oriented bbox
      if (obb.getOuter().size() >= line.size()) obb = {};
    }

    if (!_cfg.useFastSweepSkip) {
      sline.setMaxSegLen(std::numeric_limits<int32_t>::max());
    }

    const size_t id =
        _lineCache.add({sline, box, gid, subid, len, boxIds, cutouts, obb});

    diskAdd({id, box.getLowerLeft().getY(), box.getUpperRight().getY(),
             box.getLowerLeft().getX(), false, LINE, len, box45});
    diskAdd({id, box.getLowerLeft().getY(), box.getUpperRight().getY(),
             box.getUpperRight().getX(), true, LINE, len, box45});
  }

  if (_curSweepId / 2 % 1000000 == 0)
    LOGTO(INFO, std::cerr) << "@ " << _curSweepId / 2;
}

// _____________________________________________________________________________
void Sweeper::add(const I32Point& point, const std::string& gid) {
  add(point, gid, 0);
}

// _____________________________________________________________________________
void Sweeper::add(const I32Point& point, const std::string& gid, size_t subid) {
  size_t id = _pointCache.add({gid, subid});
  if (subid > 0) multiAdd(gid, point.getX(), point.getX());
  auto pointR = util::geo::rotateSinCos(point, sin45, cos45, I32Point(0, 0));
  diskAdd({id, point.getY(), point.getY(), point.getX(), false, POINT, 0,
           util::geo::getBoundingBox(pointR)});
  diskAdd({id, point.getY(), point.getY(), point.getX(), true, POINT, 0,
           util::geo::getBoundingBox(pointR)});

  if (_curSweepId / 2 % 1000000 == 0)
    LOGTO(INFO, std::cerr) << "@ " << _curSweepId / 2;
}

// _____________________________________________________________________________
void Sweeper::clearMultis(bool force) {
  int32_t curMinThreadX = std::numeric_limits<int32_t>::max();

  for (size_t i = 0; i < _cfg.numThreads; i++) {
    if (_atomicCurX[i] < curMinThreadX) curMinThreadX = _atomicCurX[i];
  }

  size_t c = 0;

  for (auto a = _activeMultis.begin(); a != _activeMultis.end();) {
    size_t mid = *a;
    if (mid >= _multiIds.size()) {
      LOG(WARN) << "Invalid multi ID " << mid << " detected!";
      a++;
      continue;
    }
    const std::string& gid = _multiIds[mid];
    int32_t rightX = _multiRightX[mid];
    if (force || rightX < curMinThreadX) {
      multiOut(_cfg.numThreads, gid);
      a = _activeMultis.erase(a);
      c++;
    } else {
      a++;
    }
  }
}

// _____________________________________________________________________________
void Sweeper::multiOut(size_t t, const std::string& gidA) {
  // write touches
  {
    std::unique_lock<std::mutex> lock(_mutTouches);
    auto i = _subTouches.find(gidA);
    if (i != _subTouches.end()) {
      for (const auto& b : i->second) {
        auto gidB = b;
        std::unique_lock<std::mutex> lock(_mutNotTouches);
        auto j = _subNotTouches.find(gidA);
        if (j != _subNotTouches.end()) {
          auto k = j->second.find(gidB);
          if (k == j->second.end()) {
            writeRel(t, gidA, gidB, _cfg.sepTouches);
            writeRel(t, gidB, gidA, _cfg.sepTouches);

            auto bEntry = _subTouches.find(gidB);
            if (bEntry != _subTouches.end()) bEntry->second.erase(gidA);

            auto bEntryN = _subNotTouches.find(gidB);
            if (bEntryN != _subNotTouches.end()) bEntryN->second.erase(gidA);
          }
        } else {
          writeRel(t, gidA, gidB, _cfg.sepTouches);
          writeRel(t, gidB, gidA, _cfg.sepTouches);
        }
      }
    }
  }

  // write crosses
  {
    std::unique_lock<std::mutex> lock(_mutCrosses);
    auto i = _subCrosses.find(gidA);
    if (i != _subCrosses.end()) {
      for (const auto& b : i->second) {
        auto gidB = b;
        std::unique_lock<std::mutex> lock(_mutNotCrosses);
        auto j = _subNotCrosses.find(gidA);
        if (j != _subNotCrosses.end()) {
          auto k = j->second.find(gidB);
          if (k == j->second.end()) {
            writeRel(t, gidA, gidB, _cfg.sepCrosses);
            writeRel(t, gidB, gidA, _cfg.sepCrosses);

            auto bEntry = _subCrosses.find(gidB);
            if (bEntry != _subCrosses.end()) bEntry->second.erase(gidA);

            auto bEntryN = _subNotCrosses.find(gidB);
            if (bEntryN != _subNotCrosses.end()) bEntryN->second.erase(gidA);
          }
        } else {
          writeRel(t, gidA, gidB, _cfg.sepCrosses);
          writeRel(t, gidB, gidA, _cfg.sepCrosses);
        }
      }
    }
  }

  // write overlaps
  {
    std::unique_lock<std::mutex> lock(_mutOverlaps);
    auto i = _subOverlaps.find(gidA);
    if (i != _subOverlaps.end()) {
      for (const auto& b : i->second) {
        auto gidB = b;
        std::unique_lock<std::mutex> lock(_mutNotOverlaps);
        if (!notOverlaps(gidA, gidB)) {
          writeRel(t, gidA, gidB, _cfg.sepOverlaps);
          writeRel(t, gidB, gidA, _cfg.sepOverlaps);

          auto bEntry = _subOverlaps.find(gidB);
          if (bEntry != _subOverlaps.end()) bEntry->second.erase(gidA);

          auto bEntryN = _subNotOverlaps.find(gidB);
          if (bEntryN != _subNotOverlaps.end()) bEntryN->second.erase(gidA);
        }
      }
    }
  }

  // write overlaps caused by incomplete covers
  {
    std::unique_lock<std::mutex> lock(_mutCovers);
    auto i = _subCovered.find(gidA);
    if (i != _subCovered.end()) {
      for (const auto& b : i->second) {
        auto gidB = b.first;
        if (b.second.size() == _subSizes[gidA]) continue;

        std::unique_lock<std::mutex> lock(_mutOverlaps);
        std::unique_lock<std::mutex> lock2(_mutNotOverlaps);
        if (!notOverlaps(gidA, gidB)) {
          writeRel(t, gidA, gidB, _cfg.sepOverlaps);
          writeRel(t, gidB, gidA, _cfg.sepOverlaps);

          auto bEntry = _subOverlaps.find(gidB);
          if (bEntry != _subOverlaps.end()) bEntry->second.erase(gidA);

          auto bEntryN = _subNotOverlaps.find(gidB);
          if (bEntryN != _subNotOverlaps.end()) bEntryN->second.erase(gidA);
        }
      }
    }
  }

  {
    std::unique_lock<std::mutex> lock(_mutContains);
    _subContains.erase(gidA);
  }
  {
    std::unique_lock<std::mutex> lock(_mutCovers);
    _subCovered.erase(gidA);
  }
  {
    std::unique_lock<std::mutex> lock(_mutEquals);
    _subEquals.erase(gidA);
  }
  {
    std::unique_lock<std::mutex> lock(_mutTouches);
    _subTouches.erase(gidA);
  }
  {
    std::unique_lock<std::mutex> lock(_mutNotTouches);
    _subNotTouches.erase(gidA);
  }
  {
    std::unique_lock<std::mutex> lock(_mutCrosses);
    _subCrosses.erase(gidA);
  }
  {
    std::unique_lock<std::mutex> lock(_mutNotCrosses);
    _subNotCrosses.erase(gidA);
  }
  {
    std::unique_lock<std::mutex> lock(_mutOverlaps);
    _subOverlaps.erase(gidA);
  }
  {
    std::unique_lock<std::mutex> lock(_mutNotOverlaps);
    _subNotOverlaps.erase(gidA);
  }
}

// _____________________________________________________________________________
void Sweeper::flush() {
  LOGTO(INFO, std::cerr) << _multiIds.size() << " multi geometries";
  for (size_t i = 0; i < _multiIds.size(); i++) {
    diskAdd({i, 1, 0, _multiLeftX[i] - 1, false, POINT, 0.0, {}});
  }

  writeAll(_file, _outBuffer, _obufpos);
  _obufpos = 0;

  _pointCache.flush();
  _areaCache.flush();
  _simpleAreaCache.flush();
  _lineCache.flush();
  _simpleLineCache.flush();

  LOGTO(INFO, std::cerr) << "Sorting events...";

  std::string newFName = util::getTmpFName(_cache, ".spatialjoin", "sorttmp");
  int newFile = open(newFName.c_str(), O_RDWR | O_CREAT, 0666);
  unlink(newFName.c_str());

  if (newFile < 0) {
    throw std::runtime_error("Could not open temporary file " + newFName);
    exit(1);
  }

#ifdef __unix__
  posix_fadvise(newFile, 0, 0, POSIX_FADV_SEQUENTIAL);
#endif
  util::externalSort(_file, newFile, sizeof(BoxVal), _curSweepId, boxCmp);

  fsync(newFile);

  _file = newFile;

#ifdef __unix__
  posix_fadvise(_file, 0, 0, POSIX_FADV_SEQUENTIAL);
#endif

  LOGTO(INFO, std::cerr) << "...done";
}

// _____________________________________________________________________________
void Sweeper::diskAdd(const BoxVal& bv) {
  memcpy(_outBuffer + _obufpos, &bv, sizeof(BoxVal));
  _obufpos += sizeof(BoxVal);

  if (_obufpos + sizeof(BoxVal) > BUFFER_S) {
    writeAll(_file, _outBuffer, _obufpos);
    _obufpos = 0;
  }
  _curSweepId++;
}

// _____________________________________________________________________________
void Sweeper::sortCache() {
  // start at beginning of _file
  lseek(_file, 0, SEEK_SET);

  const size_t RBUF_SIZE = 100000;
  char* buf = new char[sizeof(BoxVal) * RBUF_SIZE];

  ssize_t len;
  size_t jj = 0;

  // new caches
  GeometryCache<Point> _pointCacheNew(100000, _cfg.numCacheThreads, _cache);
  GeometryCache<SimpleLine> _simpleLineCacheNew(100000, _cfg.numCacheThreads,
                                                _cache);
  GeometryCache<Line> _lineCacheNew(100000, _cfg.numCacheThreads, _cache);
  GeometryCache<Area> _areaCacheNew(100000, _cfg.numCacheThreads, _cache);
  GeometryCache<SimpleArea> _simpleAreaCacheNew(100000, _cfg.numCacheThreads,
                                                _cache);

  int oldFile = _file;

  // OUTFACTOR 1
  _fname = util::getTmpFName(_cache, ".spatialjoin", "events");
  _file = open(_fname.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0666);

  if (_file < 0) {
    throw std::runtime_error("Could not open temporary file " + _fname);
  }

  // immediately unlink
  unlink(_fname.c_str());

#ifdef __unix__
  posix_fadvise(_file, 0, 0, POSIX_FADV_SEQUENTIAL);
#endif
  // OUTFACTOR 1

  size_t numEvents = _curSweepId;
  _curSweepId = 0;

  while ((len = read(oldFile, buf, sizeof(BoxVal) * RBUF_SIZE)) > 0) {
    for (ssize_t i = 0; i < len; i += sizeof(BoxVal)) {
      auto cur = reinterpret_cast<const BoxVal*>(buf + i);
      jj++;

      if (jj % 500000 == 0) {
        LOGTO(INFO, std::cerr)
            << jj / 2 << " / " << numEvents / 2 << " ("
            << (((1.0 * jj) / (1.0 * numEvents)) * 100) << "%)";
      }

      if (!cur->out) {
        size_t id = 0;

        if (cur->type == POINT && !(cur->loY == 1 && cur->upY == 0)) {
          const auto& curC = _pointCache.getFromDisk(cur->id, 0);
          id = _pointCacheNew.add(curC);

          // re-add events with new ID
          diskAdd({id, cur->loY, cur->upY, cur->val, false, cur->type,
                   cur->areaOrLen, cur->b45});
          diskAdd({id, cur->loY, cur->upY, cur->val, true, cur->type,
                   cur->areaOrLen, cur->b45});
        }

        if (cur->type == SIMPLE_LINE) {
          const auto& curC = _simpleLineCache.getFromDisk(cur->id, 0);
          id = _simpleLineCacheNew.add(curC);

          // re-add events with new ID
          diskAdd({id, cur->loY, cur->upY, cur->val, false, cur->type,
                   cur->areaOrLen, cur->b45});
          diskAdd(
              {id, cur->loY, cur->upY,
               curC.a.getX() <= curC.b.getX() ? curC.b.getX() : curC.a.getX(),
               true, cur->type, cur->areaOrLen, cur->b45});
        }

        if (cur->type == LINE) {
          const auto& curC = _lineCache.getFromDisk(cur->id, 0);
          id = _lineCacheNew.add(curC);

          // re-add events with new ID
          diskAdd({id, cur->loY, cur->upY, cur->val, false, cur->type,
                   cur->areaOrLen, cur->b45});
          diskAdd({id, cur->loY, cur->upY, curC.box.getUpperRight().getX(),
                   true, cur->type, cur->areaOrLen, cur->b45});
        }

        if (cur->type == POLYGON) {
          const auto& curC = _areaCache.getFromDisk(cur->id, 0);
          id = _areaCacheNew.add(curC);

          // re-add events with new ID
          diskAdd({id, cur->loY, cur->upY, cur->val, false, cur->type,
                   cur->areaOrLen, cur->b45});
          diskAdd({id, cur->loY, cur->upY, curC.box.getUpperRight().getX(),
                   true, cur->type, cur->areaOrLen, cur->b45});
        }

        if (cur->type == SIMPLE_POLYGON) {
          const auto& curC = _simpleAreaCache.getFromDisk(cur->id, 0);
          id = _simpleAreaCacheNew.add(curC);

          // re-add events with new ID
          diskAdd({id, cur->loY, cur->upY, cur->val, false, cur->type,
                   cur->areaOrLen, cur->b45});
          diskAdd({id, cur->loY, cur->upY,
                   util::geo::getBoundingBox(curC.geom).getUpperRight().getX(),
                   true, cur->type, cur->areaOrLen, cur->b45});
        }
      }
    }
  }

  close(oldFile);

  _pointCache = std::move(_pointCacheNew);
  _lineCache = std::move(_lineCacheNew);
  _simpleLineCache = std::move(_simpleLineCacheNew);
  _areaCache = std::move(_areaCacheNew);
  _simpleAreaCache = std::move(_simpleAreaCacheNew);
}

// _____________________________________________________________________________
void Sweeper::sweep() {
  // start at beginning of _file
  lseek(_file, 0, SEEK_SET);

  const size_t batchSize = 100000;
  JobBatch curBatch;

  const size_t RBUF_SIZE = 100000;
  char* buf = new char[sizeof(BoxVal) * RBUF_SIZE];

  sj::IntervalIdx<int32_t, SweepVal> actives;

  _rawFiles = {}, _files = {}, _outBufPos = {}, _outBuffers = {};

  _files.resize(_cfg.numThreads + 1);
  _gzFiles.resize(_cfg.numThreads + 1);
  _rawFiles.resize(_cfg.numThreads + 1);
  _outBufPos.resize(_cfg.numThreads + 1);
  _outBuffers.resize(_cfg.numThreads + 1);
  _stats.resize(_cfg.numThreads + 1);
  _checks.resize(_cfg.numThreads);
  _curX.resize(_cfg.numThreads);
  _atomicCurX = std::vector<std::atomic<int32_t>>(_cfg.numThreads);

  size_t counts = 0, checkCount = 0, jj = 0, checkPairs = 0;
  auto t = TIME();

  prepareOutputFiles();

  // fire up worker threads for geometry checking
  std::vector<std::thread> thrds(_cfg.numThreads);
  for (size_t i = 0; i < thrds.size(); i++)
    thrds[i] = std::thread(&Sweeper::processQueue, this, i);

  ssize_t len;

  while ((len = read(_file, buf, sizeof(BoxVal) * RBUF_SIZE)) > 0) {
    for (ssize_t i = 0; i < len; i += sizeof(BoxVal)) {
      auto cur = reinterpret_cast<const BoxVal*>(buf + i);
      jj++;

      if (!cur->out && cur->loY == 1 && cur->upY == 0 && cur->type == POINT) {
        // special multi-IN
        _activeMultis.insert(cur->id);
      } else if (!cur->out) {
        // IN event
        actives.insert({cur->loY, cur->upY}, {cur->id, cur->type, cur->b45});

        if (jj % 500000 == 0) {
          auto lon = webMercToLatLng<double>((1.0 * cur->val) / PREC, 0).getX();
          checkCount += checkPairs;
          LOGTO(INFO, std::cerr)
              << jj / 2 << " / " << _curSweepId / 2 << " ("
              << (((1.0 * jj) / (1.0 * _curSweepId)) * 100) << "%, "
              << (500000.0 / double(TOOK(t))) * 1000000000.0 << " geoms/s, "
              << (checkPairs / double(TOOK(t))) * 1000000000.0
              << " pairs/s), avg. " << ((1.0 * checkCount) / (1.0 * counts))
              << " checks/geom, sweepLon=" << lon << "°, |A|=" << actives.size()
              << ", |JQ|=" << _jobs.size() << " (x" << batchSize
              << "), |A_mult|=" << _activeMultis.size();
          t = TIME();
          checkPairs = 0;
        }

        if (jj % 100000 == 0) clearMultis(false);
      } else {
        // OUT event
        actives.erase({cur->loY, cur->upY}, {cur->id, cur->type});

        counts++;

        fillBatch(&curBatch, &actives, cur);

        if (curBatch.size() > batchSize) {
          checkPairs += curBatch.size();
          _jobs.add(std::move(curBatch));
          curBatch.clear();  // std doesnt guarantee that after move
          curBatch.reserve(batchSize + 100);
        }
      }
    }
  }

  if (len < 0) throw std::runtime_error("Could not read from events file.");

  delete[] buf;

  if (curBatch.size()) _jobs.add(std::move(curBatch));

  // the DONE element on the job queue to signal all threads to shut down
  _jobs.add({});

  // wait for all workers to finish
  for (auto& thr : thrds) thr.join();

  clearMultis(true);

  flushOutputFiles();

  // aggregate total stats
  Stats sum;
  for (auto s : _stats) sum += s;

  std::cerr << sum.toString() << std::endl;
}

// _____________________________________________________________________________
sj::Area Sweeper::areaFromSimpleArea(const SimpleArea* sa) const {
  double areaSize = util::geo::ringArea(sa->geom);

  auto spoly = I32XSortedPolygon(sa->geom);

  if (!_cfg.useFastSweepSkip) {
    spoly.getOuter().setMaxSegLen(std::numeric_limits<int32_t>::max());
  }

  return {spoly,
          util::geo::getBoundingBox(sa->geom),
          sa->id,
          0,
          areaSize,
          _cfg.useArea ? areaSize : 0,
          (_cfg.useBoxIds ? BoxIdList{{1, 0}, {-getBoxId(sa->geom.front()), 0}}
                          : BoxIdList{}),
          {},
          {},
          {},
          {},
          0,
          {},
          {},
          0};
}

// _____________________________________________________________________________
std::tuple<bool, bool, bool, bool, bool> Sweeper::check(const Area* a,
                                                        const Area* b,
                                                        size_t t) const {
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect(a->boxIds, b->boxIds);
    _stats[t].timeBoxIdIsectAreaArea += TOOK(ts);

    // all boxes of a are fully contained in b, we intersect and we are
    // contained and we do not touch or overlap
    if (r.first == a->boxIds.front().first) return {1, 1, 1, 0, 0};

    // no box shared, we cannot have any spatial relation
    if (r.first + r.second == 0) return {0, 0, 0, 0, 0};

    // at least one box is fully contained, so we intersect
    // but the number of fully and partially contained boxes is smaller
    // than the number of boxes of A, so we cannot possible by contained
    if (r.first + r.second < a->boxIds.front().first && r.first > 0) {
      // we surely overlap if the area of b is greater than the area of b
      // of if the bounding box of b is not in a
      // otherwise, we cannot be sure
      if (b->area > a->area || !util::geo::contains(b->box, a->box))
        return {1, 0, 0, 0, 1};
    }

    // a has exactly one box, but this is not fully contained, use cutout
    // if available
    if (a->boxIds.front().first == 1 && b->cutouts.size()) {
      auto i = b->cutouts.find(abs(a->boxIds[1].first));
      if (i != b->cutouts.end()) {
        size_t firstInA = 0;
        size_t firstInB = i->second;
        auto ts = TIME();
        auto res = intersectsContainsCovers(a->geom, a->box, a->outerArea,
                                            b->geom, b->box, b->outerArea,
                                            &firstInA, &firstInB);
        _stats[t].timeCutoutGeoCheckAreaArea += TOOK(ts);
        _stats[t].cutoutGeoChecksAreaArea++;
        return res;
      }
    }
  }

  if (_cfg.useOBB) {
    if (!a->obb.empty() && !b->obb.empty()) {
      auto ts = TIME();
      auto r = util::geo::intersectsContainsCovers(a->obb, b->obb);
      _stats[t].timeOBBIsectAreaArea += TOOK(ts);
      if (!std::get<0>(r)) return {0, 0, 0, 0, 0};
    }
  }

  if (_cfg.useInnerOuter && !a->outer.empty() && !b->outer.empty()) {
    auto ts = TIME();
    auto r = util::geo::intersectsContainsCovers(
        a->outer, a->outerBox, a->outerOuterArea, b->outer, b->outerBox,
        b->outerOuterArea);
    _stats[t].timeInnerOuterCheckAreaArea += TOOK(ts);
    _stats[t].innerOuterChecksAreaArea++;
    if (!std::get<0>(r)) return {0, 0, 0, 0, 0};
  }

  if (_cfg.useInnerOuter && !a->outer.empty() && !b->inner.empty()) {
    auto ts = TIME();
    auto r = util::geo::intersectsContainsCovers(
        a->outer, a->outerBox, a->outerOuterArea, b->inner, b->innerBox,
        b->innerOuterArea);
    _stats[t].timeInnerOuterCheckAreaArea += TOOK(ts);
    _stats[t].innerOuterChecksAreaArea++;
    if (std::get<1>(r)) return {1, 1, 1, 0, 0};
  }

  if (_cfg.useInnerOuter && a->outer.empty() && !b->outer.empty()) {
    auto ts = TIME();
    auto r = util::geo::intersectsContainsCovers(a->geom, a->box, a->outerArea,
                                                 b->outer, b->outerBox,
                                                 b->outerOuterArea);
    _stats[t].timeInnerOuterCheckAreaArea += TOOK(ts);
    _stats[t].innerOuterChecksAreaArea++;
    if (!std::get<0>(r)) return {0, 0, 0, 0, 0};
  }

  if (_cfg.useInnerOuter && a->outer.empty() && !b->inner.empty()) {
    auto ts = TIME();
    auto r = util::geo::intersectsContainsCovers(a->geom, a->box, a->outerArea,
                                                 b->inner, b->innerBox,
                                                 b->innerOuterArea);
    _stats[t].timeInnerOuterCheckAreaArea += TOOK(ts);
    _stats[t].innerOuterChecksAreaArea++;
    if (std::get<1>(r)) return {1, 1, 1, 0, 0};
  }

  auto ts = TIME();
  auto res = intersectsContainsCovers(a->geom, a->box, a->outerArea, b->geom,
                                      b->box, b->outerArea);
  _stats[t].timeFullGeoCheckAreaArea += TOOK(ts);
  _stats[t].fullGeoChecksAreaArea++;
  return res;
}

// _____________________________________________________________________________
std::tuple<bool, bool, bool, bool, bool> Sweeper::check(const Line* a,
                                                        const Area* b,
                                                        size_t t) const {
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect(a->boxIds, b->boxIds);
    _stats[t].timeBoxIdIsectAreaLine += TOOK(ts);

    // all boxes of a are fully contained in b, we intersect and we are
    // contained
    if (r.first == a->boxIds.front().first) return {1, 1, 1, 0, 0};

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return {0, 0, 0, 0, 0};

    // at least one box is fully contained, so we intersect
    // but the number of fully and partially contained boxed is smaller
    // than the number of boxes of A, so we cannot possible by contained
    if (r.first + r.second < a->boxIds.front().first && r.first > 0) {
      return {1, 0, 0, 0, 1};
    }

    // a has exactly one box, but this is not fully contained, use cutout
    // if available
    if (a->boxIds.front().first == 1 && b->cutouts.size()) {
      auto i = b->cutouts.find(a->boxIds[1].first);
      if (i != b->cutouts.end()) {
        size_t firstInA = 0;
        size_t firstInB = i->second;
        auto ts = TIME();
        auto res = intersectsContainsCovers(a->geom, a->box, b->geom, b->box,
                                            &firstInA, &firstInB);
        _stats[t].timeCutoutGeoCheckAreaLine += TOOK(ts);
        _stats[t].cutoutGeoChecksAreaLine++;
        return res;
      }
    }
  }

  if (_cfg.useOBB) {
    if (!a->obb.empty() && !b->obb.empty()) {
      auto ts = TIME();
      auto r = intersectsContainsCovers(a->obb, b->obb);
      _stats[t].timeOBBIsectAreaLine += TOOK(ts);
      if (!std::get<0>(r)) return {0, 0, 0, 0, 0};
    }
  }

  if (_cfg.useInnerOuter && !b->outer.empty()) {
    auto ts = TIME();
    auto r = util::geo::intersectsContainsCovers(a->geom, a->box, b->outer,
                                                 b->outerBox);
    _stats[t].timeInnerOuterCheckAreaLine += TOOK(ts);
    _stats[t].innerOuterChecksAreaLine++;
    if (!std::get<0>(r)) return {0, 0, 0, 0, 0};
  }

  if (_cfg.useInnerOuter && !b->inner.empty()) {
    auto ts = TIME();
    auto r = util::geo::intersectsContainsCovers(a->geom, a->box, b->inner,
                                                 b->innerBox);
    _stats[t].timeInnerOuterCheckAreaLine += TOOK(ts);
    _stats[t].innerOuterChecksAreaLine++;
    if (std::get<1>(r)) return {1, 1, 1, 0, 0};
  }

  auto ts = TIME();
  auto res = intersectsContainsCovers(a->geom, a->box, b->geom, b->box);
  _stats[t].timeFullGeoCheckAreaLine += TOOK(ts);
  _stats[t].fullGeoChecksAreaLine++;

  return res;
}

// _____________________________________________________________________________
std::tuple<bool, bool, bool, bool, bool> Sweeper::check(const Line* a,
                                                        const Line* b,
                                                        size_t t) const {
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect(a->boxIds, b->boxIds);
    _stats[t].timeBoxIdIsectLineLine += TOOK(ts);

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return {0, 0, 0, 0, 0};

    // a has exactly one box, use cutout if available
    if (a->boxIds.front().first == 1 && b->cutouts.size()) {
      auto i = b->cutouts.find(abs(a->boxIds[1].first));
      if (i != b->cutouts.end()) {
        size_t firstInA = 0;
        size_t firstInB = i->second;
        auto ts = TIME();
        auto res = intersectsCovers(a->geom, b->geom, a->box, b->box, &firstInA,
                                    &firstInB);
        _stats[t].timeCutoutGeoCheckLineLine += TOOK(ts);
        _stats[t].cutoutGeoChecksLineLine++;
        return res;
      }
    }

    // b has exactly one box, use cutout if available
    if (b->boxIds.front().first == 1 && a->cutouts.size()) {
      auto i = a->cutouts.find(abs(b->boxIds[1].first));
      if (i != a->cutouts.end()) {
        size_t firstInA = i->second;
        size_t firstInB = 0;
        auto ts = TIME();
        auto res = intersectsCovers(a->geom, b->geom, a->box, b->box, &firstInA,
                                    &firstInB);
        _stats[t].timeCutoutGeoCheckLineLine += TOOK(ts);
        _stats[t].cutoutGeoChecksLineLine++;
        return res;
      }
    }
  }

  if (_cfg.useOBB) {
    if (!a->obb.empty() && !b->obb.empty()) {
      auto ts = TIME();
      auto r = util::geo::intersectsContainsCovers(a->obb, b->obb);
      _stats[t].timeOBBIsectLineLine += TOOK(ts);
      if (!std::get<0>(r)) return {0, 0, 0, 0, 0};
    }
  }

  auto ts = TIME();
  auto res = intersectsCovers(a->geom, b->geom, a->box, b->box);
  _stats[t].timeFullGeoCheckLineLine += TOOK(ts);
  _stats[t].fullGeoChecksLineLine++;

  return res;
}

// _____________________________________________________________________________
std::tuple<bool, bool, bool, bool, bool> Sweeper::check(const SimpleLine* a,
                                                        const Area* b,
                                                        size_t t) const {
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect({{1, 0}, {getBoxId(a->a), 0}}, b->boxIds);
    _stats[t].timeBoxIdIsectAreaLine += TOOK(ts);

    // all boxes of a are fully contained in b, we intersect and we are
    // contained
    if (r.first == 1) return {1, 1, 1, 0, 0};

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return {0, 0, 0, 0, 0};

    // a has exactly one box, use cutout if available
    if (b->cutouts.size()) {
      auto i = b->cutouts.find(getBoxId(a->a));
      if (i != b->cutouts.end()) {
        size_t firstInA = 0;
        size_t firstInB = i->second;
        auto ts = TIME();
        auto res = intersectsContainsCovers(
            I32XSortedLine(LineSegment<int32_t>(a->a, a->b)),
            getBoundingBox(LineSegment<int32_t>(a->a, a->b)), b->geom, b->box,
            &firstInA, &firstInB);
        _stats[t].timeCutoutGeoCheckLineLine += TOOK(ts);
        _stats[t].cutoutGeoChecksLineLine++;
        return res;
      }
    }
  }

  if (_cfg.useOBB && !b->obb.empty()) {
    auto ts = TIME();
    auto r = intersectsContainsCovers(
        I32XSortedLine(LineSegment<int32_t>(a->a, a->b)), b->obb);
    _stats[t].timeOBBIsectAreaLine += TOOK(ts);
    if (!std::get<0>(r)) return {0, 0, 0, 0, 0};
  }

  if (_cfg.useInnerOuter && !b->outer.empty()) {
    auto ts = TIME();
    auto r = util::geo::intersectsContainsCovers(
        I32XSortedLine(LineSegment<int32_t>(a->a, a->b)),
        getBoundingBox(LineSegment<int32_t>(a->a, a->b)), b->outer,
        b->outerBox);
    _stats[t].timeInnerOuterCheckAreaLine += TOOK(ts);
    _stats[t].innerOuterChecksAreaLine++;
    if (!std::get<0>(r)) return {0, 0, 0, 0, 0};
  }

  if (_cfg.useInnerOuter && !b->inner.empty()) {
    auto ts = TIME();
    auto r = util::geo::intersectsContainsCovers(
        I32XSortedLine(LineSegment<int32_t>(a->a, a->b)),
        getBoundingBox(LineSegment<int32_t>(a->a, a->b)), b->inner, b->box);
    _stats[t].timeInnerOuterCheckAreaLine += TOOK(ts);
    _stats[t].innerOuterChecksAreaLine++;
    if (std::get<1>(r)) return {1, 1, 1, 0, 0};
  }

  auto ts = TIME();
  auto res = intersectsContainsCovers(
      I32XSortedLine(LineSegment<int32_t>(a->a, a->b)),
      getBoundingBox(LineSegment<int32_t>(a->a, a->b)), b->geom, b->box);
  _stats[t].timeFullGeoCheckAreaLine += TOOK(ts);
  _stats[t].fullGeoChecksAreaLine++;
  return res;
}

// _____________________________________________________________________________
std::tuple<bool, bool, bool, bool, bool> Sweeper::check(const SimpleLine* a,
                                                        const SimpleLine* b,
                                                        size_t t) const {
  auto ts = TIME();

  // no need to do a full sweep for two simple lines with all the required
  // datastructures, just unroll the individual checks here

  auto r = intersectsLineStrict(LineSegment<int32_t>(a->a, a->b), 32767, 32767,
                                LineSegment<int32_t>(b->a, b->b), 32767, 32767);

  bool weakIntersect = (r >> 0) & 1;
  bool strictIntersect = (r >> 1) & 1;
  bool overlaps = (r >> 2) & 1;
  bool touches = (r >> 3) & 1;

  if (strictIntersect && !touches && !overlaps) {
    _stats[t].timeFullGeoCheckLineLine += TOOK(ts);
    _stats[t].fullGeoChecksLineLine++;
    return {1, 0, 0, 0, 1};
  }

  _stats[t].timeFullGeoCheckLineLine += TOOK(ts);
  _stats[t].fullGeoChecksLineLine++;

  return {weakIntersect, !strictIntersect && weakIntersect,
          touches && !overlaps, overlaps && !touches, 0};
}

// _____________________________________________________________________________
std::tuple<bool, bool, bool, bool, bool> Sweeper::check(const SimpleLine* a,
                                                        const Line* b,
                                                        size_t t) const {
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect({{1, 0}, {getBoxId(a->a), 0}}, b->boxIds);
    _stats[t].timeBoxIdIsectLineLine += TOOK(ts);

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return {0, 0, 0, 0, 0};

    // use cutout if available
    if (b->cutouts.size()) {
      auto i = b->cutouts.find(getBoxId(a->a));
      if (i != b->cutouts.end()) {
        size_t firstInA = 0;
        size_t firstInB = i->second;
        auto ts = TIME();
        auto res = intersectsCovers(
            I32XSortedLine(LineSegment<int32_t>(a->a, a->b)), b->geom,
            getBoundingBox(LineSegment<int32_t>(a->a, a->b)), b->box, &firstInA,
            &firstInB);
        _stats[t].timeCutoutGeoCheckLineLine += TOOK(ts);
        _stats[t].cutoutGeoChecksLineLine++;
        return res;
      }
    }
  }

  if (_cfg.useOBB && !b->obb.empty()) {
    auto ts = TIME();
    auto r = intersectsContainsCovers(
        I32XSortedLine(LineSegment<int32_t>(a->a, a->b)), b->obb);
    _stats[t].timeOBBIsectLineLine += TOOK(ts);
    if (!std::get<0>(r)) return {0, 0, 0, 0, 0};
  }

  auto ts = TIME();
  auto res = intersectsCovers(
      I32XSortedLine(LineSegment<int32_t>(a->a, a->b)), b->geom,
      getBoundingBox(LineSegment<int32_t>(a->a, a->b)), b->box);
  _stats[t].timeFullGeoCheckLineLine += TOOK(ts);
  _stats[t].fullGeoChecksLineLine++;
  return res;
}

// _____________________________________________________________________________
std::tuple<bool, bool, bool, bool, bool> Sweeper::check(const Line* a,
                                                        const SimpleLine* b,
                                                        size_t t) const {
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect(a->boxIds, {{1, 0}, {getBoxId(b->a), 0}});
    _stats[t].timeBoxIdIsectLineLine += TOOK(ts);

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return {0, 0, 0, 0, 0};

    // use cutout if available
    if (a->cutouts.size()) {
      auto i = a->cutouts.find(getBoxId(b->a));
      if (i != a->cutouts.end()) {
        size_t firstInA = i->second;
        size_t firstInB = 0;
        auto ts = TIME();
        auto res = intersectsCovers(
            a->geom, I32XSortedLine(LineSegment<int32_t>(b->a, b->b)), a->box,
            getBoundingBox(LineSegment<int32_t>(b->a, b->b)), &firstInA,
            &firstInB);
        _stats[t].timeCutoutGeoCheckLineLine += TOOK(ts);
        _stats[t].cutoutGeoChecksLineLine++;
        return res;
      }
    }
  }

  if (_cfg.useOBB && !a->obb.empty()) {
    auto ts = TIME();
    auto r = intersectsContainsCovers(
        I32XSortedLine(LineSegment<int32_t>(b->a, b->b)), a->obb);
    _stats[t].timeOBBIsectLineLine += TOOK(ts);
    if (!std::get<0>(r)) return {0, 0, 0, 0, 0};
  }

  auto ts = TIME();
  auto res = intersectsCovers(
      a->geom, I32XSortedLine(LineSegment<int32_t>(b->a, b->b)), a->box,
      getBoundingBox(LineSegment<int32_t>(b->a, b->b)));
  _stats[t].timeFullGeoCheckLineLine += TOOK(ts);
  _stats[t].fullGeoChecksLineLine++;
  return res;
}

// _____________________________________________________________________________
std::pair<bool, bool> Sweeper::check(const I32Point& a, const Area* b,
                                     size_t t) const {
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect({{1, 0}, {getBoxId(a), 0}}, b->boxIds);
    _stats[t].timeBoxIdIsectAreaPoint += TOOK(ts);

    // all boxes of a are fully contained in b, we are contained
    if (r.first) return {1, 1};

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return {0, 0};

    // use cutout if available
    if (b->cutouts.size()) {
      auto i = b->cutouts.find(getBoxId(a));
      if (i != b->cutouts.end()) {
        auto ts = TIME();
        auto res = containsCovers(a, b->geom, i->second);
        _stats[t].timeCutoutGeoCheckAreaPoint += TOOK(ts);
        _stats[t].cutoutGeoChecksAreaPoint++;
        return res;
      }
    }
  }

  if (_cfg.useOBB && b->obb.getOuter().rawRing().size()) {
    auto ts = TIME();
    auto r = containsCovers(a, b->obb);
    _stats[t].timeOBBIsectAreaPoint += TOOK(ts);
    if (!std::get<1>(r)) return {0, 0};
  }

  if (_cfg.useInnerOuter && !b->outer.empty()) {
    auto ts = TIME();
    auto r = containsCovers(a, b->outer);
    _stats[t].timeInnerOuterCheckAreaPoint += TOOK(ts);
    _stats[t].innerOuterChecksAreaPoint++;
    if (!std::get<1>(r)) return {0, 0};
  }

  if (_cfg.useInnerOuter && !b->inner.empty()) {
    auto ts = TIME();
    auto r = containsCovers(a, b->inner);
    _stats[t].timeInnerOuterCheckAreaPoint += TOOK(ts);
    _stats[t].innerOuterChecksAreaPoint++;
    if (std::get<1>(r)) return {1, 1};
  }

  auto ts = TIME();
  auto res = containsCovers(a, b->geom);
  _stats[t].timeFullGeoCheckAreaPoint += TOOK(ts);
  _stats[t].fullGeoChecksAreaPoint++;

  return res;
}

// _____________________________________________________________________________
std::tuple<bool, bool> Sweeper::check(const I32Point& a, const Line* b,
                                      size_t t) const {
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect({{1, 0}, {getBoxId(a), 0}}, b->boxIds);
    _stats[t].timeBoxIdIsectLinePoint += TOOK(ts);

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return {0, 0};

    // use cutout if available
    if (b->cutouts.size()) {
      auto i = b->cutouts.find(getBoxId(a));
      if (i != b->cutouts.end()) {
        auto ts = TIME();
        auto res = util::geo::intersectsContains(a, b->geom, i->second);
        _stats[t].timeCutoutGeoCheckLinePoint += TOOK(ts);
        _stats[t].cutoutGeoChecksLinePoint++;
        return res;
      }
    }
  }

  auto ts = TIME();
  auto res = util::geo::intersectsContains(a, b->geom);
  _stats[t].timeFullGeoCheckLinePoint += TOOK(ts);
  _stats[t].fullGeoChecksLinePoint++;

  return res;
}

// ____________________________________________________________________________
void Sweeper::writeCrosses(size_t t, const std::string& a,
                           const std::string& b) {
  writeRel(t, a, b, _cfg.sepCrosses);
}

// ____________________________________________________________________________
void Sweeper::writeOverlaps(size_t t, const std::string& a,
                            const std::string& b) {
  writeRel(t, a, b, _cfg.sepOverlaps);
}

// ____________________________________________________________________________
void Sweeper::writeCovers(size_t t, const std::string& a,
                          const std::string& b) {
  writeRel(t, a, b, _cfg.sepCovers);
}

// ____________________________________________________________________________
void Sweeper::writeContains(size_t t, const std::string& a,
                            const std::string& b) {
  writeRel(t, a, b, _cfg.sepContains);
}

// ____________________________________________________________________________
void Sweeper::writeRelToBuf(size_t t, const std::string& a,
                            const std::string& b, const std::string& pred) {
  memcpy(_outBuffers[t] + _outBufPos[t], _cfg.pairStart.c_str(),
         _cfg.pairStart.size());
  _outBufPos[t] += _cfg.pairStart.size();
  memcpy(_outBuffers[t] + _outBufPos[t], a.c_str(), a.size());
  _outBufPos[t] += a.size();
  memcpy(_outBuffers[t] + _outBufPos[t], pred.c_str(), pred.size());
  _outBufPos[t] += pred.size();
  memcpy(_outBuffers[t] + _outBufPos[t], b.c_str(), b.size());
  _outBufPos[t] += b.size();
  memcpy(_outBuffers[t] + _outBufPos[t], _cfg.pairEnd.c_str(),
         _cfg.pairEnd.size());
  _outBufPos[t] += _cfg.pairEnd.size();
}

// ____________________________________________________________________________
void Sweeper::writeRel(size_t t, const std::string& a, const std::string& b,
                       const std::string& pred) {
  auto ts = TIME();

  if (_outMode == NONE) return;

  size_t totSize = _cfg.pairStart.size() + a.size() + pred.size() + b.size() +
                   _cfg.pairEnd.size();

  if (_outMode == BZ2) {
    if (_outBufPos[t] + totSize >= BUFFER_S_PAIRS) {
      int err = 0;
      BZ2_bzWrite(&err, _files[t], _outBuffers[t], _outBufPos[t]);
      if (err == BZ_IO_ERROR) {
        BZ2_bzWriteClose(&err, _files[t], 0, 0, 0);
        throw std::runtime_error("Could not write to file.");
      }
      _outBufPos[t] = 0;
    }

    writeRelToBuf(t, a, b, pred);
  } else if (_outMode == GZ) {
    if (_outBufPos[t] + totSize >= BUFFER_S_PAIRS) {
      int r = gzwrite(_gzFiles[t], _outBuffers[t], _outBufPos[t]);
      if (r != (int)_outBufPos[t]) {
        gzclose(_gzFiles[t]);
        throw std::runtime_error("Could not write to file.");
      }
      _outBufPos[t] = 0;
    }

    writeRelToBuf(t, a, b, pred);
  } else if (_outMode == PLAIN) {
    if (_outBufPos[t] + totSize >= BUFFER_S_PAIRS) {
      size_t r =
          fwrite(_outBuffers[t], sizeof(char), _outBufPos[t], _rawFiles[t]);
      if (r != _outBufPos[t]) {
        throw std::runtime_error("Could not write to file.");
      }
      _outBufPos[t] = 0;
    }

    writeRelToBuf(t, a, b, pred);
  } else if (_outMode == COUT) {
    if (_outBufPos[t] + totSize + 1 >= BUFFER_S_PAIRS) {
      _outBuffers[t][_outBufPos[t]] = 0;
      fputs(reinterpret_cast<const char*>(_outBuffers[t]), stdout);
      _outBufPos[t] = 0;
    }
    writeRelToBuf(t, a, b, pred);
  }

  _stats[t].timeWrite += TOOK(ts);
}

// ____________________________________________________________________________
void Sweeper::writeIntersect(size_t t, const std::string& a,
                             const std::string& b) {
  writeRel(t, a, b, _cfg.sepIsect);
}

// ____________________________________________________________________________
void Sweeper::doCheck(const BoxVal cur, const SweepVal sv, size_t t) {
  _checks[t]++;
  _curX[t] = cur.val;

  // every 10000 checks, update our position
  if (_checks[t] % 10000 == 0) _atomicCurX[t] = _curX[t];

  if ((cur.type == SIMPLE_POLYGON || cur.type == POLYGON) &&
      (sv.type == SIMPLE_POLYGON || sv.type == POLYGON)) {
    auto ts = TIME();

    const Area* a;
    const Area* b;

    std::shared_ptr<Area> asp;
    std::shared_ptr<Area> bsp;

    Area al, bl;

    if (cur.type == SIMPLE_POLYGON) {
      auto p = _simpleAreaCache.get(cur.id, t);
      al = areaFromSimpleArea(p.get());
      a = &al;
    } else {
      asp = _areaCache.get(cur.id, t);
      a = asp.get();
    }

    if (sv.type == SIMPLE_POLYGON) {
      bl = areaFromSimpleArea(_simpleAreaCache.get(sv.id, t).get());
      b = &bl;
    } else {
      bsp = _areaCache.get(sv.id, t);
      b = bsp.get();
    }

    _stats[t].timeGeoCacheRetrievalArea += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    auto res = check(a, b, t);

    // intersects
    if (std::get<0>(res)) {
      writeIntersect(t, a->id, b->id);
      writeIntersect(t, b->id, a->id);
    }

    // contained
    if (std::get<1>(res)) {
      if (a->subId > 0) {
        // a is a multigeometry, *all* its parts must be contained.
        // we cache them, and write them as soon as we know that yes,
        // they are all contained
        writeContainsMulti(t, b->id, a->id, a->subId);
      } else {
        writeContains(t, b->id, a->id);
      }
    }

    // covered
    if (std::get<2>(res)) {
      if (a->subId > 0) {
        // a is a multigeometry, *all* its parts must be covered.
        // we cache them, and write them as soon as we know that yes,
        // they are all covered
        writeCoversMulti(t, b->id, a->id, a->subId);
      } else {
        writeCovers(t, b->id, a->id);
      }

      if (fabs(a->area - b->area) < util::geo::EPSILON) {
        // both areas were equivalent
        writeEquals(t, a->id, a->subId, b->id, b->subId);

        if (b->subId > 0) {
          // b is a multigeometry, *all* its parts must be equal.
          // we cache them, and write them as soon as we know that yes,
          // they are all equal
          writeCoversMulti(t, a->id, b->id, b->subId);
        } else {
          writeCovers(t, a->id, b->id);
        }
      }
    }

    // touches
    if (std::get<3>(res)) {
      writeTouches(t, a->id, a->subId, b->id, b->subId);
    } else if (std::get<0>(res)) {
      // if a is not a multi-geom, and is completey covered, we wont
      // be finding a touch as we assume non-self-intersecting geoms
      if (!(a->subId == 0 && std::get<2>(res))) {
        writeNotTouches(t, a->id, a->subId, b->id, b->subId);
      }
    }

    // overlaps
    if (std::get<4>(res)) {
      writeOverlaps(t, a->id, b->id);
      writeOverlaps(t, b->id, a->id);
    }
  } else if (cur.type == LINE &&
             (sv.type == SIMPLE_POLYGON || sv.type == POLYGON)) {
    const Area* b;
    std::shared_ptr<Area> bsp;
    Area bl;

    auto ts = TIME();
    auto a = _lineCache.get(cur.id, t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);
    ts = TIME();
    if (sv.type == SIMPLE_POLYGON) {
      bl = areaFromSimpleArea(_simpleAreaCache.get(sv.id, t).get());
      b = &bl;
    } else {
      bsp = _areaCache.get(sv.id, t);
      b = bsp.get();
    }
    _stats[t].timeGeoCacheRetrievalArea += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    auto res = check(a.get(), b, t);

    // intersects
    if (std::get<0>(res)) {
      writeIntersect(t, a->id, b->id);
      writeIntersect(t, b->id, a->id);
    }

    // contains
    if (std::get<1>(res)) {
      if (a->subId > 0) {
        // a is a multigeometry, *all* its parts must be contained.
        // we cache them, and write them as soon as we know that yes,
        // they are all contained
        writeContainsMulti(t, b->id, a->id, a->subId);
      } else {
        writeContains(t, b->id, a->id);
      }
    }

    // covers
    if (std::get<2>(res)) {
      if (a->subId > 0) {
        // a is a multigeometry, *all* its parts must be covered.
        // we cache them, and write them as soon as we know that yes,
        // they are all covered
        writeCoversMulti(t, b->id, a->id, a->subId);
      } else {
        writeCovers(t, b->id, a->id);
      }
    }

    // touches
    if (std::get<3>(res)) {
      writeTouches(t, a->id, a->subId, b->id, b->subId);
    } else if (std::get<0>(res)) {
      // if a is not a multi-geom, and is completey covered, we wont
      // be finding a touch as we assume non-self-intersecting geoms
      if (!(a->subId == 0 && std::get<2>(res))) {
        writeNotTouches(t, a->id, a->subId, b->id, b->subId);
      }
    }

    // crosses
    if (std::get<4>(res)) {
      writeCrosses(t, a->id, b->id);
    }
  } else if (cur.type == SIMPLE_LINE &&
             (sv.type == SIMPLE_POLYGON || sv.type == POLYGON)) {
    const Area* b;
    std::shared_ptr<Area> bsp;
    Area bl;

    auto ts = TIME();
    auto a = _simpleLineCache.get(cur.id, t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);
    ts = TIME();
    if (sv.type == SIMPLE_POLYGON) {
      bl = areaFromSimpleArea(_simpleAreaCache.get(sv.id, t).get());
      b = &bl;
    } else {
      bsp = _areaCache.get(sv.id, t);
      b = bsp.get();
    }
    _stats[t].timeGeoCacheRetrievalArea += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    auto res = check(a.get(), b, t);

    // intersects
    if (std::get<0>(res)) {
      writeIntersect(t, a->id, b->id);
      writeIntersect(t, b->id, a->id);
    }

    // contains
    if (std::get<1>(res)) {
      writeContains(t, b->id, a->id);
    }

    // covers
    if (std::get<2>(res)) {
      writeCovers(t, b->id, a->id);
    }

    // touches
    if (std::get<3>(res)) {
      writeTouches(t, a->id, 0, b->id, b->subId);
    } else if (std::get<0>(res)) {
      // if a is completey covered, we wont
      // be finding a touch as we assume non-self-intersecting geoms
      if (!(std::get<2>(res))) {
        writeNotTouches(t, a->id, 0, b->id, b->subId);
      }
    }

    // crosses
    if (std::get<4>(res)) {
      writeCrosses(t, a->id, b->id);
    }
  } else if ((cur.type == SIMPLE_POLYGON || cur.type == POLYGON) &&
             sv.type == LINE) {
    const Area* a;
    std::shared_ptr<Area> asp;
    Area al;

    auto ts = TIME();
    if (cur.type == SIMPLE_POLYGON) {
      al = areaFromSimpleArea(_simpleAreaCache.get(cur.id, t).get());
      a = &al;
    } else {
      asp = _areaCache.get(cur.id, t);
      a = asp.get();
    }
    _stats[t].timeGeoCacheRetrievalArea += TOOK(ts);
    ts = TIME();
    auto b = _lineCache.get(sv.id, t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    auto res = check(b.get(), a, t);

    // intersects
    if (std::get<0>(res)) {
      writeIntersect(t, a->id, b->id);
      writeIntersect(t, b->id, a->id);
    }

    // contains
    if (std::get<1>(res)) {
      if (b->subId > 0) {
        // b is a multigeometry, *all* its parts must be covered.
        // we cache them, and write them as soon as we know that yes,
        // they are all contained
        writeContainsMulti(t, a->id, b->id, b->subId);
      } else {
        writeContains(t, a->id, b->id);
      }
    }

    // covers
    if (std::get<2>(res)) {
      if (b->subId > 0) {
        // b is a multigeometry, *all* its parts must be covered.
        // we cache them, and write them as soon as we know that yes,
        // they are all covered
        writeCoversMulti(t, a->id, b->id, b->subId);
      } else {
        writeCovers(t, a->id, b->id);
      }
    }

    // touches
    if (std::get<3>(res)) {
      writeTouches(t, a->id, a->subId, b->id, b->subId);
    } else if (std::get<0>(res)) {
      // if b is not a multi-geom, and is completey covered, we wont
      // be finding a touch as we assume non-self-intersecting geoms
      if (!(b->subId == 0 && std::get<2>(res))) {
        writeNotTouches(t, a->id, a->subId, b->id, b->subId);
      }
    }

    // crosses
    if (std::get<4>(res)) {
      writeCrosses(t, b->id, a->id);
    }
  } else if ((cur.type == SIMPLE_POLYGON || cur.type == POLYGON) &&
             sv.type == SIMPLE_LINE) {
    const Area* a;
    std::shared_ptr<Area> asp;
    Area al;

    auto ts = TIME();
    if (cur.type == SIMPLE_POLYGON) {
      al = areaFromSimpleArea(_simpleAreaCache.get(cur.id, t).get());
      a = &al;
    } else {
      asp = _areaCache.get(cur.id, t);
      a = asp.get();
    }
    _stats[t].timeGeoCacheRetrievalArea += TOOK(ts);
    ts = TIME();
    auto b = _simpleLineCache.get(sv.id, t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    auto res = check(b.get(), a, t);

    // intersects
    if (std::get<0>(res)) {
      writeIntersect(t, a->id, b->id);
      writeIntersect(t, b->id, a->id);
    }

    // contains
    if (std::get<1>(res)) {
      writeContains(t, a->id, b->id);
    }

    // covers
    if (std::get<2>(res)) {
      writeCovers(t, a->id, b->id);
    }

    // touches
    if (std::get<3>(res)) {
      writeTouches(t, a->id, a->subId, b->id, 0);
    } else if (std::get<0>(res)) {
      // if b is completey covered, we wont
      // be finding a touch as we assume non-self-intersecting geoms
      if (!std::get<2>(res)) {
        writeNotTouches(t, a->id, a->subId, b->id, 0);
      }
    }

    // crosses
    if (std::get<4>(res)) {
      writeCrosses(t, b->id, a->id);
    }
  } else if (cur.type == LINE && sv.type == LINE) {
    auto ts = TIME();
    auto a = _lineCache.get(cur.id, t);
    auto b = _lineCache.get(sv.id, t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    auto res = check(a.get(), b.get(), t);

    // intersects
    if (std::get<0>(res)) {
      writeIntersect(t, a->id, b->id);
      writeIntersect(t, b->id, a->id);
    }

    // covers
    if (std::get<1>(res)) {
      writeNotCrosses(t, a->id, a->subId, b->id, b->subId);

      if (a->subId == 0) writeNotOverlaps(t, a->id, a->subId, b->id, b->subId);

      if (a->subId > 0) {
        // a is a multigeometry, *all* its parts must be covered.
        // we cache them, and write them as soon as we know that yes,
        // they are all covered
        writeCoversMulti(t, b->id, a->id, a->subId);
      } else {
        writeCovers(t, b->id, a->id);
      }

      if (fabs(a->length - b->length) < util::geo::EPSILON) {
        // both lines were equivalent
        writeEquals(t, a->id, a->subId, b->id, b->subId);

        if (b->subId > 0) {
          // a is a multigeometry, *all* its parts must be covered.
          // we cache them, and write them as soon as we know that yes,
          // they are all covered
          writeCoversMulti(t, a->id, b->id, b->subId);
        } else {
          writeCovers(t, a->id, b->id);
        }
      }
    }

    // touches
    if (std::get<2>(res)) {
      writeTouches(t, a->id, a->subId, b->id, b->subId);
    } else if (std::get<0>(res)) {
      writeNotTouches(t, a->id, a->subId, b->id, b->subId);
    }

    // crosses
    if (std::get<4>(res)) {
      writeNotOverlaps(t, a->id, a->subId, b->id, b->subId);
      writeCrosses(t, a->id, a->subId, b->id, b->subId);
    }

    // overlaps
    if (std::get<3>(res)) {
      if (!std::get<1>(res))
        writeNotCrosses(t, a->id, a->subId, b->id, b->subId);
      writeOverlaps(t, a->id, a->subId, b->id, b->subId);
    }
  } else if (cur.type == LINE && sv.type == SIMPLE_LINE) {
    auto ts = TIME();
    auto a = _lineCache.get(cur.id, t);
    auto b = _simpleLineCache.get(sv.id, t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    auto res = check(a.get(), b.get(), t);

    // intersects
    if (std::get<0>(res)) {
      writeIntersect(t, a->id, b->id);
      writeIntersect(t, b->id, a->id);
    }

    // covers
    if (std::get<1>(res)) {
      writeNotCrosses(t, a->id, a->subId, b->id, 0);

      if (a->subId > 0) {
        // a is a multigeometry, *all* its parts must be covered.
        // we cache them, and write them as soon as we know that yes,
        // they are all covered
        writeCoversMulti(t, b->id, a->id, a->subId);
      } else {
        writeCovers(t, b->id, a->id);
      }

      if (fabs(a->length - util::geo::len(LineSegment<int32_t>(b->a, b->b))) <
          util::geo::EPSILON) {
        // both lines were equivalent
        writeCovers(t, a->id, b->id);

        writeEquals(t, a->id, a->subId, b->id, 0);
      }
    }

    // touches
    if (std::get<2>(res)) {
      writeTouches(t, a->id, a->subId, b->id, 0);
    } else if (std::get<0>(res)) {
      writeNotTouches(t, a->id, a->subId, b->id, 0);
    }

    // crosses
    if (std::get<4>(res)) {
      writeNotOverlaps(t, a->id, a->subId, b->id, 0);
      writeCrosses(t, a->id, a->subId, b->id, 0);
    }

    // overlaps
    if (std::get<3>(res)) {
      if (!std::get<1>(res)) writeNotCrosses(t, a->id, a->subId, b->id, 0);
      writeOverlaps(t, a->id, a->subId, b->id, 0);
    }
  } else if (cur.type == SIMPLE_LINE && sv.type == LINE) {
    auto ts = TIME();
    auto a = _simpleLineCache.get(cur.id, t);
    auto b = _lineCache.get(sv.id, t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    auto res = check(a.get(), b.get(), t);

    // intersects
    if (std::get<0>(res)) {
      writeIntersect(t, a->id, b->id);
      writeIntersect(t, b->id, a->id);
    }

    // covers
    if (std::get<1>(res)) {
      writeNotCrosses(t, a->id, 0, b->id, b->subId);
      writeCovers(t, b->id, a->id);

      writeNotOverlaps(t, a->id, 0, b->id, b->subId);

      if (fabs(util::geo::len(LineSegment<int32_t>(a->a, a->b)) - b->length) <
          util::geo::EPSILON) {
        writeEquals(t, a->id, 0, b->id, b->subId);

        if (b->subId > 0) {
          // b is a multigeometry, *all* its parts must be covered.
          // we cache them, and write them as soon as we know that yes,
          // they are all covered
          writeCoversMulti(t, a->id, b->id, b->subId);
        } else {
          writeCovers(t, a->id, b->id);
        }
      }
    }

    // touches
    if (std::get<2>(res)) {
      writeTouches(t, a->id, 0, b->id, b->subId);
    } else if (std::get<0>(res)) {
      writeNotTouches(t, a->id, 0, b->id, b->subId);
    }

    // crosses
    if (std::get<4>(res)) {
      writeNotOverlaps(t, a->id, 0, b->id, b->subId);
      writeCrosses(t, a->id, 0, b->id, b->subId);
    }

    // overlaps
    if (std::get<3>(res)) {
      if (!std::get<1>(res)) writeNotCrosses(t, a->id, 0, b->id, b->subId);
      writeOverlaps(t, a->id, 0, b->id, b->subId);
    }
  } else if (cur.type == SIMPLE_LINE && sv.type == SIMPLE_LINE) {
    auto ts = TIME();
    auto a = _simpleLineCache.get(cur.id, t);
    auto b = _simpleLineCache.get(sv.id, t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    auto res = check(a.get(), b.get(), t);

    if (std::get<0>(res)) {
      writeIntersect(t, a->id, b->id);
      writeIntersect(t, b->id, a->id);
    }

    if (std::get<1>(res)) {
      writeCovers(t, b->id, a->id);

      if (fabs(util::geo::len(LineSegment<int32_t>(a->a, a->b)) -
               util::geo::len(LineSegment<int32_t>(b->a, b->b))) <
          util::geo::EPSILON) {
        writeEquals(t, a->id, 0, b->id, 0);
        writeCovers(t, a->id, b->id);
      }
    }

    // touches
    if (std::get<2>(res)) {
      writeTouches(t, a->id, 0, b->id, 0);
    }

    // crosses
    if (std::get<4>(res)) {
      writeCrosses(t, a->id, 0, b->id, 0);
    }

    // overlaps
    if (std::get<3>(res)) {
      writeOverlaps(t, a->id, 0, b->id, 0);
    }
  } else if (cur.type == POINT && sv.type == POINT) {
    // point/point: trivial intersect & cover & contains

    auto ts = TIME();
    auto a = _pointCache.get(cur.id, t);
    auto b = _pointCache.get(sv.id, t);
    _stats[t].timeGeoCacheRetrievalPoint += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    writeIntersect(t, a->id, b->id);
    writeIntersect(t, b->id, a->id);
    writeEquals(t, a->id, a->subId, b->id, b->subId);

    if (a->subId > 0) {
      // a is a multigeometry, *all* its parts must be covered.
      // we cache them, and write them as soon as we know that yes,
      // they are all covered
      writeCoversMulti(t, b->id, a->id, a->subId);
      writeContainsMulti(t, b->id, a->id, a->subId);
    } else {
      writeCovers(t, b->id, a->id);
      writeContains(t, b->id, a->id);
    }

    if (b->subId > 0) {
      // b is a multigeometry, *all* its parts must be covered.
      // we cache them, and write them as soon as we know that yes,
      // they are all covered
      writeCoversMulti(t, a->id, b->id, b->subId);
      writeContainsMulti(t, a->id, b->id, b->subId);
    } else {
      writeCovers(t, a->id, b->id);
      writeContains(t, a->id, b->id);
    }
  } else if (cur.type == POINT && sv.type == SIMPLE_LINE) {
    auto p = I32Point(cur.val, cur.loY);
    auto ts = TIME();
    auto b = _simpleLineCache.get(sv.id, t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    if (util::geo::contains(p, LineSegment<int32_t>(b->a, b->b))) {
      auto ts = TIME();
      auto a = _pointCache.get(cur.id, t);
      _stats[t].timeGeoCacheRetrievalPoint += TOOK(ts);
      writeIntersect(t, a->id, b->id);
      writeIntersect(t, b->id, a->id);

      if (a->subId > 0) {
        // a is a multigeometry, *all* its parts must be covered.
        // we cache them, and write them as soon as we know that yes,
        // they are all covered
        writeCoversMulti(t, b->id, a->id, a->subId);
      } else {
        writeCovers(t, b->id, a->id);
      }

      if (p != b->a && p != b->b) {
        if (a->subId > 0) {
          // a is a multigeometry, *all* its parts must be contained.
          // we cache them, and write them as soon as we know that yes,
          // they are all contained
          writeContainsMulti(t, b->id, a->id, a->subId);
        } else {
          writeContains(t, b->id, a->id);
        }

        writeNotTouches(t, a->id, a->subId, b->id, 0);
      } else {
        writeTouches(t, a->id, a->subId, b->id, 0);
      }
    }
  } else if (cur.type == POINT && sv.type == LINE) {
    auto ts = TIME();
    auto a = I32Point(cur.val, cur.loY);
    auto b = _lineCache.get(sv.id, t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    auto res = check(a, b.get(), t);

    if (std::get<0>(res)) {
      auto ts = TIME();
      auto a = _pointCache.get(cur.id, t);
      _stats[t].timeGeoCacheRetrievalPoint += TOOK(ts);

      if (a->id == b->id) return;  // no self-checks in multigeometries

      writeIntersect(t, a->id, b->id);
      writeIntersect(t, b->id, a->id);

      if (a->subId > 0) {
        // a is a multigeometry, *all* its parts must be covered.
        // we cache them, and write them as soon as we know that yes,
        // they are all covered
        writeCoversMulti(t, b->id, a->id, a->subId);
      } else {
        writeCovers(t, b->id, a->id);
      }

      if (std::get<1>(res)) {
        if (a->subId > 0) {
          // a is a multigeometry, *all* its parts must be contained.
          // we cache them, and write them as soon as we know that yes,
          // they are all contained
          writeContainsMulti(t, b->id, a->id, a->subId);
        } else {
          writeContains(t, b->id, a->id);
        }
        writeNotTouches(t, a->id, a->subId, b->id, b->subId);
      } else {
        writeTouches(t, a->id, a->subId, b->id, b->subId);
      }

      if (b->length == 0) {
        // zero length line, point also covers line

        if (b->subId > 0) {
          // a is a multigeometry, *all* its parts must be covered.
          // we cache them, and write them as soon as we know that yes,
          // they are all contained
          writeCoversMulti(t, a->id, b->id, b->subId);
        } else {
          writeCovers(t, a->id, b->id);
        }
      }
    }
  } else if (cur.type == POINT &&
             (sv.type == SIMPLE_POLYGON || sv.type == POLYGON)) {
    const Area* b;
    std::shared_ptr<Area> bsp;
    Area bl;

    auto a = I32Point(cur.val, cur.loY);
    auto ts = TIME();
    if (sv.type == SIMPLE_POLYGON) {
      bl = areaFromSimpleArea(_simpleAreaCache.get(sv.id, t).get());
      b = &bl;
    } else {
      bsp = _areaCache.get(sv.id, t);
      b = bsp.get();
    }
    _stats[t].timeGeoCacheRetrievalArea += TOOK(ts);

    auto res = check(a, b, t);

    if (res.second) {
      auto ts = TIME();
      auto a = _pointCache.get(cur.id, t);
      _stats[t].timeGeoCacheRetrievalPoint += TOOK(ts);

      if (a->subId > 0) {
        // a is a multigeometry, *all* its parts must be covered.
        // we cache them, and write them as soon as we know that yes,
        // they are all covered
        writeCoversMulti(t, b->id, a->id, a->subId);
      } else {
        writeCovers(t, b->id, a->id);
      }
      writeIntersect(t, a->id, b->id);
      writeIntersect(t, b->id, a->id);

      if (res.first) {
        if (a->subId > 0) {
          // a is a multigeometry, *all* its parts must be covered.
          // we cache them, and write them as soon as we know that yes,
          // they are all contained
          writeContainsMulti(t, b->id, a->id, a->subId);
        } else {
          writeContains(t, b->id, a->id);
        }

        if (a->subId != 0) {
          writeNotTouches(t, a->id, a->subId, b->id, b->subId);
        }
      } else {
        writeTouches(t, a->id, a->subId, b->id, b->subId);
      }
    }
  }
}

// _____________________________________________________________________________
void Sweeper::flushOutputFiles() {
  if (_outMode == COUT) {
    for (size_t i = 0; i < _cfg.numThreads + 1; i++) {
      _outBuffers[i][_outBufPos[i]] = 0;
      fputs(reinterpret_cast<const char*>(_outBuffers[i]), stdout);
    }
  }
  if (_outMode == BZ2 || _outMode == GZ || _outMode == PLAIN) {
    if (_outMode == BZ2) {
      for (size_t i = 0; i < _cfg.numThreads + 1; i++) {
        int err = 0;
        BZ2_bzWrite(&err, _files[i], _outBuffers[i], _outBufPos[i]);
        if (err == BZ_IO_ERROR) {
          BZ2_bzWriteClose(&err, _files[i], 0, 0, 0);
          throw std::runtime_error("Could not write to file.");
        }
        BZ2_bzWriteClose(&err, _files[i], 0, 0, 0);
        fclose(_rawFiles[i]);
      }
    } else if (_outMode == GZ) {
      for (size_t i = 0; i < _cfg.numThreads + 1; i++) {
        int r = gzwrite(_gzFiles[i], _outBuffers[i], _outBufPos[i]);
        if (r != (int)_outBufPos[i]) {
          gzclose(_gzFiles[i]);
          throw std::runtime_error("Could not write to file.");
        }
        gzclose(_gzFiles[i]);
      }
    } else {
      for (size_t i = 0; i < _cfg.numThreads + 1; i++) {
        size_t r =
            fwrite(_outBuffers[i], sizeof(char), _outBufPos[i], _rawFiles[i]);
        if (r != _outBufPos[i]) {
          throw std::runtime_error("Could not write to file.");
        }
        fclose(_rawFiles[i]);
      }
    }

    // merge files into first file
    std::ofstream out(
        _cache + "/.rels" + std::to_string(getpid()) + "-0",
        std::ios_base::binary | std::ios_base::app | std::ios_base::ate);
    for (size_t i = 1; i < _cfg.numThreads + 1; i++) {
      std::string fName = _cache + "/.rels" + std::to_string(getpid()) + "-" +
                          std::to_string(i);
      std::ifstream ifCur(fName, std::ios_base::binary);
      out << ifCur.rdbuf();
      std::remove(fName.c_str());
    }

    // move first file to output file
    std::rename((_cache + "/.rels" + std::to_string(getpid()) + "-0").c_str(),
                _out.c_str());
  }
}

// _____________________________________________________________________________
void Sweeper::prepareOutputFiles() {
  if (_outMode == BZ2) {
    for (size_t i = 0; i < _cfg.numThreads + 1; i++) {
      _rawFiles[i] = fopen((_cache + "/.rels" + std::to_string(getpid()) + "-" +
                            std::to_string(i))
                               .c_str(),
                           "w");
      int err = 0;
      _files[i] = BZ2_bzWriteOpen(&err, _rawFiles[i], 6, 0, 30);
      if (err != BZ_OK) {
        throw std::runtime_error("Could not open bzip file for writing.");
      }
      _outBuffers[i] = new unsigned char[BUFFER_S_PAIRS];
    }
  } else if (_outMode == GZ) {
    for (size_t i = 0; i < _cfg.numThreads + 1; i++) {
      _gzFiles[i] = gzopen((_cache + "/.rels" + std::to_string(getpid()) + "-" +
                            std::to_string(i))
                               .c_str(),
                           "w");
      if (_gzFiles[i] == Z_NULL) {
        throw std::runtime_error("Could not open bzip file for writing.");
      }
      _outBuffers[i] = new unsigned char[BUFFER_S_PAIRS];
    }
  } else if (_outMode == PLAIN) {
    for (size_t i = 0; i < _cfg.numThreads + 1; i++) {
      _rawFiles[i] = fopen((_cache + "/.rels" + std::to_string(getpid()) + "-" +
                            std::to_string(i))
                               .c_str(),
                           "w");
      _outBuffers[i] = new unsigned char[BUFFER_S_PAIRS];
    }
  } else if (_outMode == COUT) {
    for (size_t i = 0; i < _cfg.numThreads + 1; i++) {
      _outBuffers[i] = new unsigned char[BUFFER_S_PAIRS];
    }
  }
}

// _____________________________________________________________________________
void Sweeper::processQueue(size_t t) {
  JobBatch batch;
  while ((batch = _jobs.get()).size()) {
    for (const auto& job : batch) doCheck(job.first, job.second, t);
  }

  _atomicCurX[t] = _curX[t];
}

// _____________________________________________________________________________
void Sweeper::fillBatch(JobBatch* batch,
                        const IntervalIdx<int32_t, SweepVal>* actives,
                        const BoxVal* cur) const {
  const auto& overlaps = actives->overlap_find_all({cur->loY, cur->upY});

  for (const auto& p : overlaps) {
    if (_cfg.useDiagBox && !util::geo::intersects(p.v.b45, cur->b45)) continue;

    batch->push_back({*cur, p.v});
  }
}

// _____________________________________________________________________________
void Sweeper::writeOverlaps(size_t t, const std::string& a, size_t aSub,
                            const std::string& b, size_t bSub) {
  if (a == b) return;
  if (aSub == 0 && bSub == 0) {
    writeRel(t, a, b, _cfg.sepOverlaps);
    writeRel(t, b, a, _cfg.sepOverlaps);
    return;
  }

  {
    std::unique_lock<std::mutex> lock(_mutOverlaps);

    if (bSub != 0) _subOverlaps[b].insert(a);
    if (aSub != 0) _subOverlaps[a].insert(b);
  }
}

// _____________________________________________________________________________
void Sweeper::writeNotOverlaps(size_t, const std::string& a, size_t aSub,
                               const std::string& b, size_t bSub) {
  if (a == b) return;
  if (aSub == 0 && bSub == 0) return;

  {
    std::unique_lock<std::mutex> lock(_mutNotOverlaps);

    _subNotOverlaps[b].insert(a);
    _subNotOverlaps[a].insert(b);
  }
}

// _____________________________________________________________________________
void Sweeper::writeCrosses(size_t t, const std::string& a, size_t aSub,
                           const std::string& b, size_t bSub) {
  if (a == b) return;
  if (aSub == 0 && bSub == 0) {
    writeRel(t, a, b, _cfg.sepCrosses);
    writeRel(t, b, a, _cfg.sepCrosses);
    return;
  }

  {
    // TODO: distinct locks for individual methods
    std::unique_lock<std::mutex> lock(_mutCrosses);

    if (bSub != 0) _subCrosses[b].insert(a);
    if (aSub != 0) _subCrosses[a].insert(b);
  }
}

// _____________________________________________________________________________
void Sweeper::writeNotCrosses(size_t, const std::string& a, size_t aSub,
                              const std::string& b, size_t bSub) {
  if (a == b) return;
  if (aSub == 0 && bSub == 0) return;

  {
    std::unique_lock<std::mutex> lock(_mutNotCrosses);

    if (bSub != 0) _subNotCrosses[b].insert(a);
    if (aSub != 0) _subNotCrosses[a].insert(b);
  }
}

// _____________________________________________________________________________
void Sweeper::writeTouches(size_t t, const std::string& a, size_t aSub,
                           const std::string& b, size_t bSub) {
  if (a == b) return;
  if (aSub == 0 && bSub == 0) {
    writeRel(t, a, b, _cfg.sepTouches);
    writeRel(t, b, a, _cfg.sepTouches);
    return;
  }

  {
    std::unique_lock<std::mutex> lock(_mutTouches);

    if (bSub != 0) _subTouches[b].insert(a);
    if (aSub != 0) _subTouches[a].insert(b);
  }
}

// _____________________________________________________________________________
void Sweeper::writeNotTouches(size_t, const std::string& a, size_t aSub,
                              const std::string& b, size_t bSub) {
  if (a == b) return;
  if (aSub == 0 && bSub == 0) return;

  {
    std::unique_lock<std::mutex> lock(_mutNotTouches);

    if (bSub != 0) _subNotTouches[b].insert(a);
    if (aSub != 0) _subNotTouches[a].insert(b);
  }
}

// _____________________________________________________________________________
void Sweeper::writeEquals(size_t t, const std::string& a, size_t aSub,
                          const std::string& b, size_t bSub) {
  if (a == b) return;
  if (aSub == 0 && bSub == 0) {
    writeRel(t, a, b, _cfg.sepEquals);
    writeRel(t, b, a, _cfg.sepEquals);
    return;
  }

  if (aSub == 0 || bSub == 0) {
    writeNotOverlaps(t, a, aSub, b, bSub);
    return;  // multi and non-multi cannot be equal
  }

  if (_subSizes[a] != _subSizes[b]) {
    return;  // multi of different sizes cannot be equal
  }

  {
    std::unique_lock<std::mutex> lock(_mutEquals);

    _subEquals[b][a].insert(aSub);
    _subEquals[a][b].insert(bSub);

    if (_subEquals[a][b].size() != _subSizes[a] ||
        _subEquals[b][a].size() != _subSizes[b])
      return;

    _subEquals[b].erase(a);
    _subEquals[a].erase(b);
  }

  writeRel(t, a, b, _cfg.sepEquals);
  writeRel(t, b, a, _cfg.sepEquals);
}

// _____________________________________________________________________________
void Sweeper::writeCoversMulti(size_t t, const std::string& a,
                               const std::string& b, size_t bSub) {
  if (a == b) return;
  {
    std::unique_lock<std::mutex> lock(_mutCovers);

    _subCovered[b][a].insert(bSub);

    if (_subCovered[b][a].size() != _subSizes[b]) return;

    _subCovered[b].erase(a);
  }

  writeNotOverlaps(t, a, 0, b, bSub);
  writeCovers(t, a, b);
}

// _____________________________________________________________________________
void Sweeper::writeContainsMulti(size_t t, const std::string& a,
                                 const std::string& b, size_t bSub) {
  if (a == b) return;
  {
    std::unique_lock<std::mutex> lock(_mutContains);

    _subContains[b][a].insert(bSub);

    if (_subContains[b][a].size() != _subSizes[b]) return;

    _subContains[b].erase(a);
  }

  writeContains(t, a, b);
}

// _____________________________________________________________________________
bool Sweeper::notOverlaps(const std::string& a, const std::string& b) const {
  auto j = _subNotOverlaps.find(a);
  auto jj = _subNotOverlaps.find(b);

  if (j != _subNotOverlaps.end()) {
    auto k = j->second.find(b);
    if (k != j->second.end()) return true;
  }

  if (jj != _subNotOverlaps.end()) {
    auto k = jj->second.find(a);
    if (k != jj->second.end()) return true;
  }

  return false;
}
