#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#ifndef SPATIALJOIN_NO_ZLIB
#include <zlib.h>
#endif

#ifndef SPATIALJOIN_NO_BZIP2
#include <bzlib.h>
#endif

#include <algorithm>
#include <cassert>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <set>
#include <sstream>

#include "BoxIds.h"
#include "InnerOuter.h"
#include "Sweeper.h"
#include "util/Misc.h"
#include "util/geo/IntervalIdx.h"
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
using util::geo::FPoint;
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
using util::geo::LineSegment;
using util::geo::webMercToLatLng;
using util::LogLevel::DEBUG;
using util::LogLevel::ERROR;
using util::LogLevel::INFO;
using util::LogLevel::VDEBUG;
using util::LogLevel::WARN;

const static size_t CUTOUTS_MIN_SIZE = 100;
const static size_t OBB_MIN_SIZE = 100;

const static double sin45 = 1.0 / sqrt(2);
const static double cos45 = 1.0 / sqrt(2);

// _____________________________________________________________________________
I32Box Sweeper::add(const I32MultiPolygon& a, const std::string& gid, bool side,
                  WriteBatch& batch) const {
  uint16_t subid = 0;  // a subid of 0 means "single polygon"
  if (a.size() > 1) subid = 1;

  return add(a, gid, subid, side, batch);
}

// _____________________________________________________________________________
I32Box Sweeper::add(const I32MultiLine& a, const std::string& gid, bool side,
                  WriteBatch& batch) const {
  uint16_t subid = 0;  // a subid of 0 means "single line"
  if (a.size() > 1) subid = 1;

  return add(a, gid, subid, side, batch);
}

// _____________________________________________________________________________
I32Box Sweeper::add(const I32MultiPoint& a, const std::string& gid, bool side,
                  WriteBatch& batch) const {
  uint16_t subid = 0;  // a subid of 0 means "single point"
  if (a.size() > 1) subid = 1;

  return add(a, gid, subid, side, batch);
}

// _____________________________________________________________________________
I32Box Sweeper::add(const I32MultiPolygon& a, const std::string& gid,
                  size_t subId, bool side, WriteBatch& batch) const {
  I32Box ret;
  for (const auto& poly : a) {
    if (poly.getOuter().size() < 2) continue;
    auto box = add(poly, gid, subId, side, batch);
    if (box.isNull()) continue;
    ret = util::geo::extendBox(box, ret);
    subId++;
  }

  return ret;
}

// _____________________________________________________________________________
I32Box Sweeper::add(const I32MultiLine& a, const std::string& gid, size_t subId,
                  bool side, WriteBatch& batch) const {
  I32Box ret;
  for (const auto& line : a) {
    if (line.size() < 2) continue;
    auto box = add(line, gid, subId, side, batch);
    if (box.isNull()) continue;
    ret = util::geo::extendBox(box, ret);
    subId++;
  }

  return ret;
}

// _____________________________________________________________________________
I32Box Sweeper::add(const I32MultiPoint& a, const std::string& gid, size_t subid,
                  bool side, WriteBatch& batch) const {
  I32Box ret;
  size_t newId = subid;
  for (const auto& point : a) {
    auto box = add(point, gid, newId, side, batch);
    if (box.isNull()) continue;
    ret = util::geo::extendBox(box, ret);
    newId++;
  }

  return ret;
}

// _____________________________________________________________________________
void Sweeper::multiAdd(const std::string& gid, bool side, int32_t xLeft,
                       int32_t xRight) {
  auto i = _multiGidToId[side].find(gid);

  if (i == _multiGidToId[side].end()) {
    _multiIds[side].push_back(gid);
    _multiRightX[side].push_back(xRight);
    _multiLeftX[side].push_back(xLeft);
    _multiGidToId[side][gid] = _multiIds[side].size() - 1;
    _subSizes[gid] = 1;
  } else {
    size_t id = _multiGidToId[side][gid];
    if (xRight > _multiRightX[side][id]) _multiRightX[side][id] = xRight;
    if (xLeft < _multiLeftX[side][id]) _multiLeftX[side][id] = xLeft;
    _subSizes[gid] = _subSizes[gid] + 1;
  }
}

// _____________________________________________________________________________
void Sweeper::add(const std::string& parent, const util::geo::I32Box& box,
                  const std::string& gid, bool side, WriteBatch& batch) const {
  add(parent, box, gid, 0, side, batch);
}

// _____________________________________________________________________________
void Sweeper::add(const std::string& parentR, const util::geo::I32Box& box,
                  const std::string& gidR, size_t subid, bool side,
                  WriteBatch& batch) const {
  std::string gid = (side ? ("B" + gidR) : ("A" + gidR));
  std::string parent = (side ? ("B" + parentR) : ("A" + parentR));

  BoxVal boxl, boxr;
  boxl.side = side;
  boxr.side = side;

  boxl.val = box.getLowerLeft().getX();
  boxr.val = box.getUpperRight().getX();

  batch.refs.push_back({parent, gid, boxl, boxr, subid});
}

// _____________________________________________________________________________
I32Box Sweeper::add(const I32Polygon& poly, const std::string& gid, bool side,
                  WriteBatch& batch) const {
  return add(poly, gid, 0, side, batch);
}

// _____________________________________________________________________________
I32Box Sweeper::add(const I32Polygon& poly, const std::string& gidR, size_t subid,
                  bool side, WriteBatch& batch) const {
  std::string gid = (side ? ("B" + gidR) : ("A" + gidR));

  WriteCand cur;
  const auto& rawBox = getBoundingBox(poly);
  const auto& box = getPaddedBoundingBox(rawBox);
  if (!util::geo::intersects(box, _filterBox)) return {};
  I32XSortedPolygon spoly(poly);

  if (spoly.empty()) return box;

  double areaSize = area(poly);
  double outerAreaSize = outerArea(poly);
  BoxIdList boxIds;
  std::map<int32_t, size_t> cutouts;

  if (_cfg.useBoxIds) {
    if (_cfg.useCutouts && poly.getOuter().size() > CUTOUTS_MIN_SIZE) {
      boxIds = packBoxIds(getBoxIds(spoly, rawBox, outerAreaSize, &cutouts));
    } else {
      boxIds = packBoxIds(getBoxIds(spoly, rawBox, outerAreaSize, 0));
    }
  }

  I32Box box45;
  if (_cfg.useDiagBox) {
    auto polyR = util::geo::rotateSinCos(poly, sin45, cos45, I32Point(0, 0));
    box45 = getPaddedBoundingBox(polyR, rawBox);
  }

  cur.subid = subid;
  cur.gid = gid;

  if (poly.getInners().size() == 0 && poly.getOuter().size() < 10 &&
      subid == 0 && (!_cfg.useBoxIds || boxIds.front().first == 1)) {
    std::stringstream str;
    GeometryCache<SimpleArea>::writeTo({poly.getOuter(), gid}, str);
    cur.raw = str.str();

    size_t estimatedSize =
        poly.getOuter().size() * sizeof(util::geo::XSortedTuple<int32_t>);

    cur.boxvalIn = {0,  // placeholder, will be overwritten later on
                    box.getLowerLeft().getY(),
                    box.getUpperRight().getY(),
                    box.getLowerLeft().getX(),
                    false,
                    SIMPLE_POLYGON,
                    areaSize,
                    {},
                    box45,
                    side,
                    estimatedSize > GEOM_LARGENESS_THRESHOLD};
    cur.boxvalOut = {0,  // placeholder, will be overwritten later on
                     box.getLowerLeft().getY(),
                     box.getUpperRight().getY(),
                     box.getUpperRight().getX(),
                     true,
                     SIMPLE_POLYGON,
                     areaSize,
                    {},
                     box45,
                     side,
                     estimatedSize > GEOM_LARGENESS_THRESHOLD};
    batch.simpleAreas.emplace_back(cur);
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

    std::stringstream str;
    GeometryCache<Area>::writeTo(
        {spoly, box, gid, subid, areaSize, _cfg.useArea ? outerAreaSize : 0,
         boxIds, cutouts, obb, inner, innerBox, innerOuterAreaSize, outer,
         outerBox, outerOuterAreaSize},
        str);
    ;

    size_t estimatedSize = spoly.getOuter().rawRing().size() *
                           sizeof(util::geo::XSortedTuple<int32_t>);
    for (const auto& p : spoly.getInners()) {
      estimatedSize +=
          p.rawRing().size() * sizeof(util::geo::XSortedTuple<int32_t>);
    }

    cur.raw = str.str();

    cur.boxvalIn = {0,  // placeholder, will be overwritten later on
                    box.getLowerLeft().getY(),
                    box.getUpperRight().getY(),
                    box.getLowerLeft().getX(),
                    false,
                    POLYGON,
                    areaSize,
                    {},
                    box45,
                    side,
                    estimatedSize > GEOM_LARGENESS_THRESHOLD};
    cur.boxvalOut = {0,  // placeholder, will be overwritten later on
                     box.getLowerLeft().getY(),
                     box.getUpperRight().getY(),
                     box.getUpperRight().getX(),
                     true,
                     POLYGON,
                     areaSize,
                    {},
                     box45,
                     side,
                     estimatedSize > GEOM_LARGENESS_THRESHOLD};
    batch.areas.emplace_back(cur);
  }

  return box;
}

// _____________________________________________________________________________
I32Box Sweeper::add(const I32Line& line, const std::string& gid, bool side,
                  WriteBatch& batch) const {
  return add(line, gid, 0, side, batch);
}

// _____________________________________________________________________________
I32Box Sweeper::add(const I32Line& line, const std::string& gidR, size_t subid,
                  bool side, WriteBatch& batch) const {
  if (line.size() < 2) return {};

  std::string gid = (side ? ("B" + gidR) : ("A" + gidR));

  WriteCand cur;

  const auto& rawBox = getBoundingBox(line);
  const auto& box = getPaddedBoundingBox(rawBox);
  if (!util::geo::intersects(box, _filterBox)) return {};
  BoxIdList boxIds;
  std::map<int32_t, size_t> cutouts;

  if (_cfg.useBoxIds) {
    if (_cfg.useCutouts && line.size() > CUTOUTS_MIN_SIZE) {
      boxIds = packBoxIds(getBoxIds(line, rawBox, &cutouts));
    } else {
      boxIds = packBoxIds(getBoxIds(line, rawBox, 0));
    }
  }

  const double len = util::geo::len(line);

  I32Box box45;
  if (_cfg.useDiagBox) {
    auto lineR = util::geo::rotateSinCos(line, sin45, cos45, I32Point(0, 0));
    box45 = getPaddedBoundingBox(lineR, rawBox);
  }

  cur.subid = subid;
  cur.gid = gid;

  if (line.size() == 2 && (!_cfg.useBoxIds || boxIds.front().first == 1) &&
      subid == 0) {
    // simple line

    std::stringstream str;
    GeometryCache<SimpleLine>::writeTo({line.front(), line.back(), gid}, str);

    cur.raw = str.str();

    cur.boxvalIn = {0,  // placeholder, will be overwritten later on
                    box.getLowerLeft().getY(),
                    box.getUpperRight().getY(),
                    box.getLowerLeft().getX(),
                    false,
                    SIMPLE_LINE,
                    len,
                    {},
                    box45,
                    side,
                    false};
    cur.boxvalOut = {0,  // placeholder, will be overwritten later on,
                     box.getLowerLeft().getY(),
                     box.getUpperRight().getY(),
                     box.getUpperRight().getX(),
                     true,
                     SIMPLE_LINE,
                     len,
                    {},
                     box45,
                     side,
                     false};
    batch.simpleLines.emplace_back(cur);
  } else {
    // normal line
    I32XSortedLine sline(line);
    if (line.empty()) return {};
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

    std::stringstream str;
    GeometryCache<Line>::writeTo(
        {sline, box, gid, subid, len, boxIds, cutouts, obb}, str);
    cur.raw = str.str();

    size_t estimatedSize =
        line.size() * sizeof(util::geo::XSortedTuple<int32_t>);

    cur.boxvalIn = {0,  // placeholder, will be overwritten later on
                    box.getLowerLeft().getY(),
                    box.getUpperRight().getY(),
                    box.getLowerLeft().getX(),
                    false,
                    LINE,
                    len,
                    {},
                    box45,
                    side,
                    estimatedSize > GEOM_LARGENESS_THRESHOLD};
    cur.boxvalOut = {0,  // placeholder, will be overwritten later on
                     box.getLowerLeft().getY(),
                     box.getUpperRight().getY(),
                     box.getUpperRight().getX(),
                     true,
                     LINE,
                     len,
                    {},
                     box45,
                     side,
                     estimatedSize > GEOM_LARGENESS_THRESHOLD};
    batch.lines.emplace_back(cur);
  }

  return box;
}

// _____________________________________________________________________________
I32Box Sweeper::add(const I32Point& point, const std::string& gid, bool side,
                  WriteBatch& batch) const {
  return add(point, gid, 0, side, batch);
}

// _____________________________________________________________________________
I32Box Sweeper::add(const I32Point& point, const std::string& gidR, size_t subid,
                  bool side, WriteBatch& batch) const {
  std::string gid = (side ? ("B" + gidR) : ("A" + gidR));

  WriteCand cur;

  std::stringstream str;
  GeometryCache<Point>::writeTo({gid, subid}, str);

  cur.raw = str.str();

  cur.subid = subid;
  cur.gid = gid;

  const auto& rawBox = getBoundingBox(point);
  const auto& box = getPaddedBoundingBox(rawBox);

  if (!util::geo::intersects(box, _filterBox)) return {};

  auto pointR = util::geo::rotateSinCos(point, sin45, cos45, I32Point(0, 0));
  cur.boxvalIn = {0,  // placeholder, will be overwritten later on
                  box.getLowerLeft().getY(),
                  box.getUpperRight().getY(),
                  box.getLowerLeft().getX(),
                  false,
                  POINT,
                  0,
                  point,
                  getPaddedBoundingBox(pointR, rawBox),
                  side,
                  false};
  cur.boxvalOut = {0,  // placeholder, will be overwritten later on
                   box.getLowerLeft().getY(),
                   box.getUpperRight().getY(),
                   box.getUpperRight().getX(),
                   true,
                   POINT,
                   0,
                   point,
                   getPaddedBoundingBox(pointR, rawBox),
                   side,
                   false};

  batch.points.emplace_back(cur);

  return box;
}

// _____________________________________________________________________________
void Sweeper::addBatch(WriteBatch& cands) {
  {
    std::unique_lock<std::mutex> lock(_pointGeomCacheWriteMtx);
    for (auto& cand : cands.points) {
      if (cand.boxvalIn.side) _numSides = 2;
      cand.boxvalIn.id = _pointCache.add(cand.raw);
      cand.boxvalOut.id = cand.boxvalIn.id;
    }
  }

  {
    std::unique_lock<std::mutex> lock(_lineGeomCacheWriteMtx);
    for (auto& cand : cands.lines) {
      if (cand.boxvalIn.side) _numSides = 2;
      cand.boxvalIn.id = _lineCache.add(cand.raw);
      cand.boxvalOut.id = cand.boxvalIn.id;
    }
  }

  {
    std::unique_lock<std::mutex> lock(_simpleLineGeomCacheWriteMtx);
    for (auto& cand : cands.simpleLines) {
      if (cand.boxvalIn.side) _numSides = 2;
      cand.boxvalIn.id = _simpleLineCache.add(cand.raw);
      cand.boxvalOut.id = cand.boxvalIn.id;
    }
  }

  {
    std::unique_lock<std::mutex> lock(_simpleAreaGeomCacheWriteMtx);
    for (auto& cand : cands.simpleAreas) {
      if (cand.boxvalIn.side) _numSides = 2;
      cand.boxvalIn.id = _simpleAreaCache.add(cand.raw);
      cand.boxvalOut.id = cand.boxvalIn.id;
    }
  }

  {
    std::unique_lock<std::mutex> lock(_areaGeomCacheWriteMtx);
    for (auto& cand : cands.areas) {
      if (cand.boxvalIn.side) _numSides = 2;
      cand.boxvalIn.id = _areaCache.add(cand.raw);
      cand.boxvalOut.id = cand.boxvalIn.id;
    }
  }

  for (const auto& cand : cands.points) {
    if (cand.subid > 0) {
      std::unique_lock<std::mutex> lock(_multiAddMtx);
      multiAdd(cand.gid, cand.boxvalIn.side, cand.boxvalIn.val,
               cand.boxvalOut.val);
    }
  }

  for (const auto& cand : cands.simpleLines) {
    if (cand.subid > 0) {
      std::unique_lock<std::mutex> lock(_multiAddMtx);
      multiAdd(cand.gid, cand.boxvalIn.side, cand.boxvalIn.val,
               cand.boxvalOut.val);
    }
  }

  for (const auto& cand : cands.lines) {
    if (cand.subid > 0) {
      std::unique_lock<std::mutex> lock(_multiAddMtx);
      multiAdd(cand.gid, cand.boxvalIn.side, cand.boxvalIn.val,
               cand.boxvalOut.val);
    }
  }

  for (const auto& cand : cands.simpleAreas) {
    if (cand.subid > 0) {
      std::unique_lock<std::mutex> lock(_multiAddMtx);
      multiAdd(cand.gid, cand.boxvalIn.side, cand.boxvalIn.val,
               cand.boxvalOut.val);
    }
  }

  for (const auto& cand : cands.areas) {
    if (cand.subid > 0) {
      std::unique_lock<std::mutex> lock(_multiAddMtx);
      multiAdd(cand.gid, cand.boxvalIn.side, cand.boxvalIn.val,
               cand.boxvalOut.val);
    }
  }

  for (const auto& cand : cands.refs) {
    if (cand.subid > 0) {
      std::unique_lock<std::mutex> lock(_multiAddMtx);
      multiAdd(cand.gid, cand.boxvalIn.side, cand.boxvalIn.val,
               cand.boxvalOut.val);
    }
  }

  {
    std::unique_lock<std::mutex> lock(_sweepEventWriteMtx);
    for (const auto& cand : cands.points) {
      diskAdd(cand.boxvalIn);
      diskAdd(cand.boxvalOut);
      if (_curSweepId / 2 % 1000000 == 0)
        log("@ " + std::to_string(_curSweepId / 2));
    }
    for (const auto& cand : cands.simpleLines) {
      diskAdd(cand.boxvalIn);
      diskAdd(cand.boxvalOut);
      if (_curSweepId / 2 % 1000000 == 0)
        log("@ " + std::to_string(_curSweepId / 2));
    }
    for (const auto& cand : cands.lines) {
      diskAdd(cand.boxvalIn);
      diskAdd(cand.boxvalOut);
      if (_curSweepId / 2 % 1000000 == 0)
        log("@ " + std::to_string(_curSweepId / 2));
    }
    for (const auto& cand : cands.simpleAreas) {
      diskAdd(cand.boxvalIn);
      diskAdd(cand.boxvalOut);
      if (_curSweepId / 2 % 1000000 == 0)
        log("@ " + std::to_string(_curSweepId / 2));
    }
    for (const auto& cand : cands.areas) {
      diskAdd(cand.boxvalIn);
      diskAdd(cand.boxvalOut);
      if (_curSweepId / 2 % 1000000 == 0)
        log("@ " + std::to_string(_curSweepId / 2));
    }
    for (const auto& cand : cands.refs) {
      _refs[cand.raw][cand.gid] = cand.subid;
      if (_curSweepId / 2 % 1000000 == 0)
        log("@ " + std::to_string(_curSweepId / 2));
    }
  }
}

// _____________________________________________________________________________
void Sweeper::clearMultis(bool force) {
  JobBatch curBatch;
  size_t batchSize = 1000;
  int32_t curMinThreadX = std::numeric_limits<int32_t>::max();

  for (size_t i = 0; i < _cfg.numThreads; i++) {
    if (_atomicCurX[i] < curMinThreadX) curMinThreadX = _atomicCurX[i];
  }

  for (size_t i = 0; i < 2; i++) {
    for (auto a = _activeMultis[i].begin(); a != _activeMultis[i].end();) {
      size_t mid = *a;
      if (mid >= _multiIds[i].size()) {
        LOG(WARN) << "Invalid multi ID " << mid << " detected!";
        a++;
        continue;
      }
      const std::string& gid = _multiIds[i][mid];
      int32_t rightX = _multiRightX[i][mid];
      if (force || rightX < curMinThreadX) {
        curBatch.push_back({{}, {}, gid});

        if (_refs.size()) {
          auto i = _refs.find(gid);
          if (i != _refs.end()) {
            for (const auto& ref : i->second) {
              curBatch.push_back({{}, {}, ref.first});
            }
          }
        }
        a = _activeMultis[i].erase(a);
      } else {
        a++;
      }

      if (curBatch.size() > batchSize) {
        _jobs.add(std::move(curBatch));
        curBatch.clear();  // std doesnt guarantee that after move
        curBatch.reserve(batchSize);
      }
    }
  }

  if (curBatch.size()) _jobs.add(std::move(curBatch));
}

// _____________________________________________________________________________
void Sweeper::multiOut(size_t tOut, const std::string& gidA) {
  // collect dist
  if (_cfg.withinDist >= 0) {
    for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
      std::unique_lock<std::mutex> lock(_mutsDistance[t]);
      auto i = _subDistance[t].find(gidA);
      if (i != _subDistance[t].end()) {
        for (const auto& a : i->second) {
          writeRel(tOut, gidA, a.first, "\t" + std::to_string(a.second) + "\t");
          writeRel(tOut, a.first, gidA, "\t" + std::to_string(a.second) + "\t");
        }
        _subDistance[t].erase(i);
      }
    }
    return;
  }

  std::unordered_map<std::string, size_t> subContains, subCovered;

  std::unordered_map<std::string, std::unordered_map<std::string, size_t>>
      subEquals;

  // collect equals
  for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
    std::unique_lock<std::mutex> lock(_mutsEquals[t]);
    auto i = _subEquals[t].find(gidA);
    if (i != _subEquals[t].end()) {
      for (const auto& a : i->second) {
        subEquals[gidA][a.first] += a.second.size();
        auto j = _subEquals[t].find(a.first);
        if (j != _subEquals[t].end()) {
          auto k = j->second.find(gidA);
          if (k != j->second.end()) {
            subEquals[a.first][gidA] += k->second.size();
            j->second.erase(gidA);
          }
        }
      }
      _subEquals[t].erase(i);
    }
  }

  // collect contains
  for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
    std::unique_lock<std::mutex> lock(_mutsContains[t]);
    auto i = _subContains[t].find(gidA);
    if (i != _subContains[t].end()) {
      for (const auto& a : i->second) {
        subContains[a.first] += a.second.size();
      }
      _subContains[t].erase(i);
    }
  }

  // collect covers
  for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
    std::unique_lock<std::mutex> lock(_mutsCovers[t]);
    auto i = _subCovered[t].find(gidA);
    if (i != _subCovered[t].end()) {
      for (const auto& a : i->second) {
        subCovered[a.first] += a.second.size();
      }
      _subCovered[t].erase(i);
    }
  }

  // write equals
  for (auto i : subEquals[gidA]) {
    if (i.second == _subSizes[gidA] &&
        subEquals[i.first][gidA] == _subSizes[i.first]) {
      writeRel(tOut, i.first, gidA, _cfg.sepEquals);
      _relStats[tOut].equals++;
      writeRel(tOut, gidA, i.first, _cfg.sepEquals);
      _relStats[tOut].equals++;
    }
  }

  // write contains
  for (auto i : subContains) {
    if (i.second == _subSizes[gidA]) {
      writeRel(tOut, i.first, gidA, _cfg.sepContains);
      _relStats[tOut].contains++;
    }
  }

  // write covers
  for (auto i : subCovered) {
    if (i.second == _subSizes[gidA]) {
      writeNotOverlaps(tOut, i.first,
                       _subSizes.find(i.first) != _subSizes.end() ? 1 : 0, gidA,
                       1);
      writeRel(tOut, i.first, gidA, _cfg.sepCovers);
      _relStats[tOut].covers++;
    }
  }

  // write touches, aggregate first to avoid locking during I/O
  std::vector<std::pair<std::string, std::string>> touchesTmp;

  for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
    {
      std::unique_lock<std::mutex> lock(_mutsTouches[t]);
      auto i = _subTouches[t].find(gidA);
      if (i != _subTouches[t].end()) {
        for (const auto& b : i->second) {
          auto gidB = b;
          if (!notTouches(gidA, gidB)) touchesTmp.push_back({gidA, gidB});

          {
            std::unique_lock<std::mutex> lock2(_mutsNotTouches[t]);
            auto j = _subNotTouches[t].find(gidB);
            if (j != _subNotTouches[t].end()) j->second.erase(gidA);
          }

          auto k = _subTouches[t].find(gidB);
          if (k != _subTouches[t].end()) k->second.erase(gidA);
        }

        _subTouches[t].erase(i);
      }
    }

    std::unique_lock<std::mutex> lock2(_mutsNotTouches[t]);
    _subNotTouches[t].erase(gidA);
  }

  for (const auto& p : touchesTmp) {
    _relStats[tOut].touches++;
    writeRel(tOut, p.first, p.second, _cfg.sepTouches);
    _relStats[tOut].touches++;
    writeRel(tOut, p.second, p.first, _cfg.sepTouches);
  }

  // write crosses, aggregate first to avoid locking during I/O
  std::vector<std::pair<std::string, std::string>> crossesTmp;
  for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
    {
      std::unique_lock<std::mutex> lock(_mutsCrosses[t]);
      auto i = _subCrosses[t].find(gidA);
      if (i != _subCrosses[t].end()) {
        for (const auto& b : i->second) {
          auto gidB = b;
          if (!notCrosses(gidA, gidB)) crossesTmp.push_back({gidA, gidB});

          {
            std::unique_lock<std::mutex> lock2(_mutsNotCrosses[t]);
            auto j = _subNotCrosses[t].find(gidB);
            if (j != _subNotCrosses[t].end()) j->second.erase(gidA);
          }

          auto k = _subCrosses[t].find(gidB);
          if (k != _subCrosses[t].end()) k->second.erase(gidA);
        }

        _subCrosses[t].erase(i);
      }
    }

    std::unique_lock<std::mutex> lock2(_mutsNotCrosses[t]);
    _subNotCrosses[t].erase(gidA);
  }

  for (const auto& p : crossesTmp) {
    _relStats[tOut].crosses++;
    writeRel(tOut, p.first, p.second, _cfg.sepCrosses);
    _relStats[tOut].crosses++;
    writeRel(tOut, p.second, p.first, _cfg.sepCrosses);
  }

  // write overlaps caused by incomplete covers
  {
    for (const auto& b : subCovered) {
      auto gidB = b.first;
      if (b.second == _subSizes[gidA]) continue;

      if (!notOverlaps(gidA, gidB)) {
        _relStats[tOut].overlaps++;
        writeRel(tOut, gidA, gidB, _cfg.sepOverlaps);
        _relStats[tOut].overlaps++;
        writeRel(tOut, gidB, gidA, _cfg.sepOverlaps);
      }
    }
  }

  // write overlaps, aggregate first to avoid locking during I/O
  std::vector<std::pair<std::string, std::string>> overlapsTmp;
  for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
    {
      std::unique_lock<std::mutex> lock(_mutsOverlaps[t]);
      auto i = _subOverlaps[t].find(gidA);
      if (i != _subOverlaps[t].end()) {
        for (const auto& b : i->second) {
          auto gidB = b;
          if (!notOverlaps(gidA, gidB)) overlapsTmp.push_back({gidA, gidB});

          {
            std::unique_lock<std::mutex> lock2(_mutsNotOverlaps[t]);
            auto j = _subNotOverlaps[t].find(gidB);
            if (j != _subNotOverlaps[t].end()) j->second.erase(gidA);
          }

          auto k = _subOverlaps[t].find(gidB);
          if (k != _subOverlaps[t].end()) k->second.erase(gidA);
        }

        _subOverlaps[t].erase(i);
      }
    }
    std::unique_lock<std::mutex> lock2(_mutsNotOverlaps[t]);
    _subNotOverlaps[t].erase(gidA);
  }

  for (const auto& p : overlapsTmp) {
    _relStats[tOut].overlaps++;
    writeRel(tOut, p.first, p.second, _cfg.sepOverlaps);
    _relStats[tOut].overlaps++;
    writeRel(tOut, p.second, p.first, _cfg.sepOverlaps);
  }
}

// _____________________________________________________________________________
void Sweeper::flush() {
  if (_numSides > 1) log("(Non-self join between 2 datasets)");

  log(std::to_string(_multiIds[0].size() + _multiIds[1].size()) +
      " multi geometries");

  for (size_t side = 0; side < 2; side++) {
    for (size_t i = 0; i < _multiIds[side].size(); i++) {
      diskAdd({i,
               1,
               0,
               _multiLeftX[side][i] - 1,
               false,
               POINT,
               0.0,
               {},
               {},
               side,
               false});
    }
  }

  ssize_t r = writeAll(_file, _outBuffer, _obufpos);
  if (r < 0) {
    std::stringstream ss;
    ss << "Could not write to events file '" << _fname << "'\n";
    ss << strerror(errno) << std::endl;
    throw std::runtime_error(ss.str());
  }

  delete[] _outBuffer;

  _obufpos = 0;

  _pointCache.flush();
  _areaCache.flush();
  _simpleAreaCache.flush();
  _lineCache.flush();
  _simpleLineCache.flush();

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
  r = util::externalSort(_file, newFile, sizeof(BoxVal), _curSweepId,
                         _cfg.numThreads, boxCmp);

  if (r < 0) {
    std::stringstream ss;
    ss << "Could not sort events file '" << _fname << "'\n";
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

// _____________________________________________________________________________
void Sweeper::diskAdd(const BoxVal& bv) {
  memcpy(_outBuffer + _obufpos, &bv, sizeof(BoxVal));
  _obufpos += sizeof(BoxVal);

  if (_obufpos + sizeof(BoxVal) > BUFFER_S) {
    ssize_t r = writeAll(_file, _outBuffer, _obufpos);
    if (r < 0) {
      std::stringstream ss;
      ss << "Could not write to events file '" << _fname << "'\n";
      ss << strerror(errno) << std::endl;
      throw std::runtime_error(ss.str());
    }
    _obufpos = 0;
  }
  _curSweepId++;
}

// _____________________________________________________________________________
RelStats Sweeper::sweep() {
  // start at beginning of _file
  lseek(_file, 0, SEEK_SET);

  const size_t batchSize = 100000;
  JobBatch curBatch;

  const size_t RBUF_SIZE = 100000;
  unsigned char* buf = new unsigned char[sizeof(BoxVal) * RBUF_SIZE];

  util::geo::IntervalIdx<int32_t, SweepVal> actives[2];

  _rawFiles = {};

#ifndef SPATIALJOIN_NO_BZIP2
  _bzFiles = {};
  _bzFiles.resize(_cfg.numThreads + 1);
#endif

#ifndef SPATIALJOIN_NO_ZLIB
  _gzFiles = {};
  _gzFiles.resize(_cfg.numThreads + 1);
#endif

  _outBufPos = {};
  _outBuffers = {};

  _rawFiles.resize(_cfg.numThreads + 1);
  _outBufPos.resize(_cfg.numThreads + 1);
  _outBuffers.resize(_cfg.numThreads + 1);
  _stats.resize(_cfg.numThreads + 1);
  _relStats.resize(_cfg.numThreads + 1);
  _checks.resize(_cfg.numThreads);
  _curX.resize(_cfg.numThreads);
  _subEquals.resize(_cfg.numThreads + 1);
  _subCovered.resize(_cfg.numThreads + 1);
  _subContains.resize(_cfg.numThreads + 1);
  _subDistance.resize(_cfg.numThreads + 1);
  _subNotOverlaps.resize(_cfg.numThreads + 1);
  _subOverlaps.resize(_cfg.numThreads + 1);
  _subNotTouches.resize(_cfg.numThreads + 1);
  _subNotCrosses.resize(_cfg.numThreads + 1);
  _subCrosses.resize(_cfg.numThreads + 1);
  _subTouches.resize(_cfg.numThreads + 1);

  _mutsEquals = std::vector<std::mutex>(_cfg.numThreads + 1);
  _mutsCovers = std::vector<std::mutex>(_cfg.numThreads + 1);
  _mutsContains = std::vector<std::mutex>(_cfg.numThreads + 1);
  _mutsOverlaps = std::vector<std::mutex>(_cfg.numThreads + 1);
  _mutsNotOverlaps = std::vector<std::mutex>(_cfg.numThreads + 1);
  _mutsNotTouches = std::vector<std::mutex>(_cfg.numThreads + 1);
  _mutsTouches = std::vector<std::mutex>(_cfg.numThreads + 1);
  _mutsNotCrosses = std::vector<std::mutex>(_cfg.numThreads + 1);
  _mutsCrosses = std::vector<std::mutex>(_cfg.numThreads + 1);
  _mutsDistance = std::vector<std::mutex>(_cfg.numThreads + 1);
  _atomicCurX = std::vector<std::atomic<int32_t>>(_cfg.numThreads + 1);

  size_t counts = 0, totalCheckCount = 0, jj = 0, checkPairs = 0;
  auto t = TIME();

  prepareOutputFiles();

  // fire up worker threads for geometry checking
  std::vector<std::thread> thrds(_cfg.numThreads);
  for (size_t i = 0; i < thrds.size(); i++)
    thrds[i] = std::thread(&Sweeper::processQueue, this, i);

  ssize_t len;

  while ((len = util::readAll(_file, buf, sizeof(BoxVal) * RBUF_SIZE)) != 0) {
    if (len < 0) {
      std::stringstream ss;
      ss << "Could not read from events file '" << _fname << "'\n";
      ss << strerror(errno) << std::endl;
      throw std::runtime_error(ss.str());
    }

    for (ssize_t i = 0; i < len; i += sizeof(BoxVal)) {
      auto cur = reinterpret_cast<const BoxVal*>(buf + i);
      jj++;

      if (!cur->out && cur->loY == 1 && cur->upY == 0 && cur->type == POINT) {
        // special multi-IN
        _activeMultis[cur->side].insert(cur->id);
      } else if (!cur->out) {
        // IN event
        actives[cur->side].insert(
            {cur->loY, cur->upY},
            {cur->id, cur->type, cur->b45,cur->point,  cur->side, cur->large});

        if (jj % 500000 == 0) {
          auto lon = webMercToLatLng<double>((1.0 * cur->val) / PREC, 0).getX();
          totalCheckCount += checkPairs;
          log(std::to_string(jj / 2) + " / " + std::to_string(_curSweepId / 2) +
              " (" +
              std::to_string((((1.0 * jj) / (1.0 * _curSweepId)) * 100)) +
              "%, " +
              std::to_string((500000.0 / double(TOOK(t))) * 1000000000.0) +
              " geoms/s, " +
              std::to_string((checkPairs / double(TOOK(t))) * 1000000000.0) +
              " pairs/s), avg. " +
              std::to_string(((1.0 * totalCheckCount) / (1.0 * counts))) +
              " checks/geom, sweepLon=" + std::to_string(lon) + "°, |A|=" +
              std::to_string(actives[0].size() + actives[1].size()) +
              ", |JQ|=" + std::to_string(_jobs.size()) + " (x" +
              std::to_string(batchSize) + "), |A_mult|=" +
              std::to_string(_activeMultis[0].size() +
                             _activeMultis[1].size()));
          t = TIME();
          checkPairs = 0;
        }

        if ((jj % 100 == 0) && _cfg.sweepProgressCb)
          _cfg.sweepProgressCb(jj / 2);

        if (jj % 200000 == 0) clearMultis(false);
      } else {
        // OUT event
        actives[cur->side].erase({cur->loY, cur->upY}, {cur->id, cur->type});

        // optional self checks, required if we have reference geoms
        if (_refs.size()) curBatch.push_back({*cur, {cur->id, cur->type}, ""});

        counts++;

        int sideB = ((int)(cur->side) + 1) % _numSides;

        fillBatch(&curBatch, &actives[sideB], cur);

        if (curBatch.size() > batchSize) {
          checkPairs += curBatch.size();
          if (!_cfg.noGeometryChecks) _jobs.add(std::move(curBatch));
          curBatch.clear();  // std doesnt guarantee that after move
          curBatch.reserve(batchSize + 100);
        }
      }
    }
  }

  delete[] buf;

  if (!_cfg.noGeometryChecks && curBatch.size()) _jobs.add(std::move(curBatch));

  // the DONE element on the job queue to signal all threads to shut down
  _jobs.add({});

  // wait for all workers to finish
  for (auto& thr : thrds) thr.join();

  // empty job queue
  _jobs.reset();
  // fire up new workers to clear multis
  for (size_t i = 0; i < thrds.size(); i++)
    thrds[i] = std::thread(&Sweeper::processQueue, this, i);

  // now also clear the multis
  clearMultis(true);
  // the DONE element on the job queue to signal all threads to shut down
  _jobs.add({});

  // again wait for all workers to finish
  for (auto& thr : thrds) thr.join();

  flushOutputFiles();

  // final check count aggregation
  totalCheckCount += checkPairs;

  // aggregate total stats
  Stats sum;
  for (auto s : _stats) sum += s;

  // aggregate total stats
  RelStats sumRel;
  for (auto s : _relStats) sumRel += s;

  if (_cfg.statsCb) {
    _cfg.statsCb(sum.toString() + "\n\n");
    _cfg.statsCb(sumRel.toString() + "\n");
    _cfg.statsCb(std::string("Checked ") + std::to_string(totalCheckCount) +
                 " candidates (with overlapping bounding box" +
                 (_cfg.useDiagBox ? " and overlapping diagonal box" : "") +
                 ")\n\n");
  }

  return sumRel;
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
  // cheap equivalence check
  if (a->box == b->box && a->area == b->area && a->geom == b->geom) {
    // equivalent!
    return {1, 0, 1, 0, 0};
  }

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
  // cheap equivalence check
  if (a->box == b->box && a->geom == b->geom) {
    // equivalent!
    return {1, 1, 0, 0, 0};
  }

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

  auto r = util::geo::IntersectorLine<int32_t>::check(
      LineSegment<int32_t>(a->a, a->b), 32767, 32767,
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
void Sweeper::writeRelToBuf(size_t t, const std::string& a,
                            const std::string& b, const std::string& pred) {
  size_t off = 1;
  memcpy(_outBuffers[t] + _outBufPos[t], _cfg.pairStart.c_str(),
         _cfg.pairStart.size());
  _outBufPos[t] += _cfg.pairStart.size();
  memcpy(_outBuffers[t] + _outBufPos[t], a.c_str() + off, a.size() - off);
  _outBufPos[t] += a.size() - off;
  memcpy(_outBuffers[t] + _outBufPos[t], pred.c_str(), pred.size());
  _outBufPos[t] += pred.size();
  memcpy(_outBuffers[t] + _outBufPos[t], b.c_str() + off, b.size() - off);
  _outBufPos[t] += b.size() - off;
  memcpy(_outBuffers[t] + _outBufPos[t], _cfg.pairEnd.c_str(),
         _cfg.pairEnd.size());
  _outBufPos[t] += _cfg.pairEnd.size();
}

// ____________________________________________________________________________
void Sweeper::writeRel(size_t t, const std::string& a, const std::string& b,
                       const std::string& pred) {
  auto ts = TIME();

  if (!_cfg.writeRelCb && _outMode == NONE) return;

  size_t off = 1;

  if (_numSides == 2) {
    if (a[0] != 'A' || a[0] == b[0]) return;
  }

  if (_cfg.writeRelCb) {
    _cfg.writeRelCb(t, a.c_str() + 1, b.c_str() + 1, pred.c_str());
  } else {
    size_t totSize = _cfg.pairStart.size() + a.size() + pred.size() + b.size() +
                     _cfg.pairEnd.size() - off - off;

    if (_outMode == BZ2) {
#ifndef SPATIALJOIN_NO_BZIP2
      if (_outBufPos[t] + totSize >= BUFFER_S_PAIRS) {
        int err = 0;
        BZ2_bzWrite(&err, _bzFiles[t], _outBuffers[t], _outBufPos[t]);
        if (err == BZ_IO_ERROR) {
          BZ2_bzWriteClose(&err, _bzFiles[t], 0, 0, 0);
          std::string fname = _cache + "/.rels" + std::to_string(getpid()) +
                              "-" + std::to_string(t);
          std::stringstream ss;
          ss << "Could not write spatial relation to temporary bzip2 file '"
             << fname << "':\n";
          ss << strerror(errno) << std::endl;
          throw std::runtime_error(ss.str());
        }
        _outBufPos[t] = 0;
      }

      writeRelToBuf(t, a, b, pred);
#endif
    } else if (_outMode == GZ) {
#ifndef SPATIALJOIN_NO_ZLIB
      if (_outBufPos[t] + totSize >= BUFFER_S_PAIRS) {
        int r = gzwrite(_gzFiles[t], _outBuffers[t], _outBufPos[t]);
        if (r != (int)_outBufPos[t]) {
          gzclose(_gzFiles[t]);
          std::string fname = _cache + "/.rels" + std::to_string(getpid()) +
                              "-" + std::to_string(t);
          std::stringstream ss;
          ss << "Could not write spatial relation to temporary gzip file '"
             << fname << "':\n";
          ss << strerror(errno) << std::endl;
          throw std::runtime_error(ss.str());
        }
        _outBufPos[t] = 0;
      }

      writeRelToBuf(t, a, b, pred);
#endif
    } else if (_outMode == PLAIN) {
      if (_outBufPos[t] + totSize >= BUFFER_S_PAIRS) {
        size_t r =
            fwrite(_outBuffers[t], sizeof(char), _outBufPos[t], _rawFiles[t]);
        if (r != _outBufPos[t]) {
          std::string fname = _cache + "/.rels" + std::to_string(getpid()) +
                              "-" + std::to_string(t);
          std::stringstream ss;
          ss << "Could not write spatial relation to temporary file '" << fname
             << "':\n";
          ss << strerror(errno) << std::endl;
          throw std::runtime_error(ss.str());
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
  }

  _stats[t].timeWrite += TOOK(ts);
}

// ____________________________________________________________________________
void Sweeper::writeDist(size_t t, const std::string& a, size_t aSub,
                        const std::string& b, size_t bSub, double dist) {
  if (a != b) {
    if (bSub > 0 || aSub > 0) {
      std::unique_lock<std::mutex> lock(_mutsDistance[t]);
      if (bSub > 0 && (_subDistance[t][b].find(a) == _subDistance[t][b].end() ||
                       _subDistance[t][b][a] > dist))
        _subDistance[t][b][a] = dist;
      if (aSub > 0 && (_subDistance[t][a].find(b) == _subDistance[t][a].end() ||
                       _subDistance[t][a][b] > dist))
        _subDistance[t][a][b] = dist;
    } else {
      writeRel(t, a, b, "\t" + std::to_string(dist) + "\t");
      writeRel(t, b, a, "\t" + std::to_string(dist) + "\t");
    }
  }

  // TODO: handle references
}

// ____________________________________________________________________________
void Sweeper::writeIntersect(size_t t, const std::string& a,
                             const std::string& b) {
  if (a != b) {
    _relStats[t].intersects++;
    _relStats[t].intersects++;
    writeRel(t, a, b, _cfg.sepIsect);
    writeRel(t, b, a, _cfg.sepIsect);
  }

  if (_refs.size() == 0) return;

  // handle references

  auto referersA = _refs.find(a);
  auto referersB = _refs.find(b);

  if (referersB != _refs.end()) {
    for (const auto& idB : referersB->second) {
      writeIntersect(t, a, idB.first);
    }
  }

  if (referersA != _refs.end()) {
    for (const auto& idA : referersA->second) {
      writeIntersect(t, idA.first, b);
    }
  }
}

// ____________________________________________________________________________
void Sweeper::selfCheck(const BoxVal cur, size_t t) {
  if (cur.type == POINT) {
    auto ts = TIME();
    auto a = _pointCache.get(cur.id, cur.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalPoint += TOOK(ts);

    writeIntersect(t, a->id, a->id);
    writeEquals(t, a->id, a->subId, a->id, a->subId);
    writeCovers(t, a->id, a->id, a->subId);
  } else if (cur.type == LINE) {
    auto a = _lineCache.get(cur.id, cur.large ? -1 : t);

    writeIntersect(t, a->id, a->id);
    writeEquals(t, a->id, a->subId, a->id, a->subId);
    writeCovers(t, a->id, a->id, a->subId);
  } else if (cur.type == SIMPLE_LINE) {
    auto a = _simpleLineCache.get(cur.id, cur.large ? -1 : t);

    writeIntersect(t, a->id, a->id);
    writeEquals(t, a->id, 0, a->id, 0);
    writeCovers(t, a->id, a->id, 0);
  } else if (cur.type == POLYGON) {
    auto a = _areaCache.get(cur.id, cur.large ? -1 : t);

    writeIntersect(t, a->id, a->id);
    writeEquals(t, a->id, a->subId, a->id, a->subId);
    writeCovers(t, a->id, a->id, a->subId);
  } else if (cur.type == SIMPLE_POLYGON) {
    auto a = _simpleAreaCache.get(cur.id, cur.large ? -1 : t);

    writeIntersect(t, a->id, a->id);
    writeEquals(t, a->id, 0, a->id, 0);
    writeCovers(t, a->id, a->id, 0);
  }
}

// ____________________________________________________________________________
void Sweeper::doDistCheck(const BoxVal cur, const SweepVal sv, size_t t) {
  _checks[t]++;
  _curX[t] = cur.val;

  // every 10000 checks, update our position
  if (_checks[t] % 10000 == 0) _atomicCurX[t] = _curX[t];

  if (cur.type == POINT && sv.type == POINT) {
    auto p1 = cur.point;
    auto p2 = sv.point;

    auto dist = meterDist(p1, p2);

    if (dist <= _cfg.withinDist) {
      auto a = _pointCache.get(cur.id, cur.large ? -1 : t);
      auto b = _pointCache.get(sv.id, sv.large ? -1 : t);

      writeDist(t, a->id, a->subId, b->id, b->subId, dist);
    }
  } else if (cur.type == POINT &&
             (sv.type == POLYGON || sv.type == SIMPLE_POLYGON)) {
    auto p = cur.point;

    const Area* a;
    std::shared_ptr<Area> asp;
    Area al;

    if (sv.type == SIMPLE_POLYGON) {
      auto p = _simpleAreaCache.get(sv.id, sv.large ? -1 : t);
      al = areaFromSimpleArea(p.get());
      a = &al;
    } else {
      asp = _areaCache.get(sv.id, sv.large ? -1 : t);
      a = asp.get();
    }

    double dist = distCheck(p, a, t);

    if (dist <= _cfg.withinDist) {
      auto b = _pointCache.get(cur.id, cur.large ? -1 : t);
      writeDist(t, a->id, a->subId, b->id, b->subId, dist);
    }
  } else if ((cur.type == POLYGON || cur.type == SIMPLE_POLYGON) &&
             sv.type == POINT) {
    auto p = sv.point;

    const Area* a;
    std::shared_ptr<Area> asp;
    Area al;

    if (cur.type == SIMPLE_POLYGON) {
      auto p = _simpleAreaCache.get(cur.id, cur.large ? -1 : t);
      al = areaFromSimpleArea(p.get());
      a = &al;
    } else {
      asp = _areaCache.get(cur.id, cur.large ? -1 : t);
      a = asp.get();
    }

    double dist = distCheck(p, a, t);

    if (dist <= _cfg.withinDist) {
      auto b = _pointCache.get(sv.id, sv.large ? -1 : t);
      writeDist(t, a->id, a->subId, b->id, b->subId, dist);
    }
  } else if ((cur.type == SIMPLE_LINE || cur.type == LINE) &&
             sv.type == POINT) {
    auto p = sv.point;

    double dist = std::numeric_limits<double>::max();

    if (cur.type == SIMPLE_LINE) {
      auto b = _simpleLineCache.get(cur.id, cur.large ? -1 : t);
      dist = distCheck(p, b.get(), t);

      if (dist <= _cfg.withinDist) {
        auto a = _pointCache.get(sv.id, sv.large ? -1 : t);
        writeDist(t, a->id, a->subId, b->id, 0, dist);
      }
    } else {
      auto b = _lineCache.get(cur.id, cur.large ? -1 : t);
      dist = distCheck(p, b.get(), t);

      if (dist <= _cfg.withinDist) {
        auto a = _pointCache.get(sv.id, sv.large ? -1 : t);
        writeDist(t, a->id, a->subId, b->id, b->subId, dist);
      }
    }
  } else if ((sv.type == SIMPLE_LINE || sv.type == LINE) && cur.type == POINT) {
    auto p = cur.point;

    double dist = std::numeric_limits<double>::max();

    if (sv.type == SIMPLE_LINE) {
      auto b = _simpleLineCache.get(sv.id, sv.large ? -1 : t);
      dist = distCheck(p, b.get(), t);

      if (dist <= _cfg.withinDist) {
        auto a = _pointCache.get(cur.id, cur.large ? -1 : t);
        writeDist(t, a->id, a->subId, b->id, 0, dist);
      }
    } else {
      auto b = _lineCache.get(sv.id, sv.large ? -1 : t);
      dist = distCheck(p, b.get(), t);

      if (dist <= _cfg.withinDist) {
        auto a = _pointCache.get(cur.id, cur.large ? -1 : t);
        writeDist(t, a->id, a->subId, b->id, b->subId, dist);
      }
    }
  } else if (sv.type == LINE && cur.type == LINE) {
    auto a = _lineCache.get(sv.id, sv.large ? -1 : t);
    auto b = _lineCache.get(cur.id, cur.large ? -1 : t);
    auto dist = distCheck(a.get(), b.get(), t);

    if (dist <= _cfg.withinDist) {
      writeDist(t, a->id, a->subId, b->id, b->subId, dist);
    }
  } else if (sv.type == SIMPLE_LINE && cur.type == SIMPLE_LINE) {
    auto a = _simpleLineCache.get(sv.id, sv.large ? -1 : t);
    auto b = _simpleLineCache.get(cur.id, cur.large ? -1 : t);
    auto dist = distCheck(a.get(), b.get(), t);

    if (dist <= _cfg.withinDist) {
      writeDist(t, a->id, 0, b->id, 0, dist);
    }
  } else if (sv.type == SIMPLE_LINE && cur.type == LINE) {
    auto a = _simpleLineCache.get(sv.id, sv.large ? -1 : t);
    auto b = _lineCache.get(cur.id, cur.large ? -1 : t);
    auto dist = distCheck(a.get(), b.get(), t);

    if (dist <= _cfg.withinDist) {
      writeDist(t, a->id, 0, b->id, b->subId, dist);
    }
  } else if (sv.type == LINE && cur.type == SIMPLE_LINE) {
    auto a = _lineCache.get(sv.id, sv.large ? -1 : t);
    auto b = _simpleLineCache.get(cur.id, cur.large ? -1 : t);
    auto dist = distCheck(b.get(), a.get(), t);

    if (dist <= _cfg.withinDist) {
      writeDist(t, a->id, a->subId, b->id, 0, dist);
    }
  } else if ((sv.type == SIMPLE_POLYGON || sv.type == POLYGON) &&
             (cur.type == SIMPLE_POLYGON || cur.type == POLYGON)) {
    const Area* a;
    std::shared_ptr<Area> asp;
    Area al;

    if (sv.type == SIMPLE_POLYGON) {
      auto p = _simpleAreaCache.get(sv.id, sv.large ? -1 : t);
      al = areaFromSimpleArea(p.get());
      a = &al;
    } else {
      asp = _areaCache.get(sv.id, sv.large ? -1 : t);
      a = asp.get();
    }

    const Area* b;
    std::shared_ptr<Area> bsp;
    Area bl;

    if (cur.type == SIMPLE_POLYGON) {
      auto p = _simpleAreaCache.get(cur.id, cur.large ? -1 : t);
      bl = areaFromSimpleArea(p.get());
      b = &bl;
    } else {
      bsp = _areaCache.get(cur.id, cur.large ? -1 : t);
      b = bsp.get();
    }

    auto dist = distCheck(a, b, t);

    if (dist <= _cfg.withinDist) {
      writeDist(t, a->id, a->subId, b->id, b->subId, dist);
    }
  } else if (sv.type == LINE &&
             (cur.type == SIMPLE_POLYGON || cur.type == POLYGON)) {
    auto a = _lineCache.get(sv.id, sv.large ? -1 : t);

    const Area* b;
    std::shared_ptr<Area> bsp;
    Area bl;

    if (cur.type == SIMPLE_POLYGON) {
      auto p = _simpleAreaCache.get(cur.id, cur.large ? -1 : t);
      bl = areaFromSimpleArea(p.get());
      b = &bl;
    } else {
      bsp = _areaCache.get(cur.id, cur.large ? -1 : t);
      b = bsp.get();
    }

    auto dist = distCheck(a.get(), b, t);

    if (dist <= _cfg.withinDist) {
      writeDist(t, a->id, a->subId, b->id, b->subId, dist);
    }
  } else if ((sv.type == SIMPLE_POLYGON || sv.type == POLYGON) &&
             cur.type == LINE) {
    const Area* a;
    std::shared_ptr<Area> asp;
    Area al;

    if (sv.type == SIMPLE_POLYGON) {
      auto p = _simpleAreaCache.get(sv.id, sv.large ? -1 : t);
      al = areaFromSimpleArea(p.get());
      a = &al;
    } else {
      asp = _areaCache.get(sv.id, sv.large ? -1 : t);
      a = asp.get();
    }
    auto b = _lineCache.get(cur.id, cur.large ? -1 : t);
    auto dist = distCheck(b.get(), a, t);

    if (dist <= _cfg.withinDist) {
      writeDist(t, a->id, a->subId, b->id, b->subId, dist);
    }
  } else if (sv.type == SIMPLE_LINE &&
             (cur.type == SIMPLE_POLYGON || cur.type == POLYGON)) {
    auto a = _simpleLineCache.get(sv.id, sv.large ? -1 : t);
    const Area* b;
    std::shared_ptr<Area> bsp;
    Area bl;

    if (cur.type == SIMPLE_POLYGON) {
      auto p = _simpleAreaCache.get(cur.id, cur.large ? -1 : t);
      bl = areaFromSimpleArea(p.get());
      b = &bl;
    } else {
      bsp = _areaCache.get(cur.id, cur.large ? -1 : t);
      b = bsp.get();
    }
    auto dist = distCheck(a.get(), b, t);

    if (dist <= _cfg.withinDist) {
      writeDist(t, a->id, 0, b->id, b->subId, dist);
    }
  } else if ((sv.type == SIMPLE_POLYGON || sv.type == POLYGON) &&
             cur.type == SIMPLE_LINE) {
    const Area* a;
    std::shared_ptr<Area> asp;
    Area al;

    if (sv.type == SIMPLE_POLYGON) {
      auto p = _simpleAreaCache.get(sv.id, sv.large ? -1 : t);
      al = areaFromSimpleArea(p.get());
      a = &al;
    } else {
      asp = _areaCache.get(sv.id, sv.large ? -1 : t);
      a = asp.get();
    }
    auto b = _simpleLineCache.get(cur.id, cur.large ? -1 : t);
    auto dist = distCheck(b.get(), a, t);

    if (dist <= _cfg.withinDist) {
      writeDist(t, a->id, a->subId, b->id, 0, dist);
    }
  }
}

// ____________________________________________________________________________
void Sweeper::doCheck(const BoxVal cur, const SweepVal sv, size_t t) {
  _checks[t]++;
  _curX[t] = cur.val;

  if (cur.type == sv.type && cur.id == sv.id) return selfCheck(cur, t);

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
      auto p = _simpleAreaCache.get(cur.id, cur.large ? -1 : t);
      al = areaFromSimpleArea(p.get());
      a = &al;
    } else {
      asp = _areaCache.get(cur.id, cur.large ? -1 : t);
      a = asp.get();
    }

    if (sv.type == SIMPLE_POLYGON) {
      bl = areaFromSimpleArea(
          _simpleAreaCache.get(sv.id, sv.large ? -1 : t).get());
      b = &bl;
    } else {
      bsp = _areaCache.get(sv.id, sv.large ? -1 : t);
      b = bsp.get();
    }

    _stats[t].timeGeoCacheRetrievalArea += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    _stats[t].areaCmps++;
    _stats[t].areaSizeSum += std::max(a->area, b->area);

    _stats[t].anchorSum += std::max(a->geom.size() / 2, b->geom.size() / 2);

    _stats[t].totalComps++;
    auto totTime = TIME();

    auto res = check(a, b, t);

    _stats[t].timeHisto(std::max(a->geom.getOuter().rawRing().size(),
                                 b->geom.getOuter().rawRing().size()),
                        TOOK(totTime));

    // intersects
    if (std::get<0>(res)) {
      writeIntersect(t, a->id, b->id);
    }

    // contained
    if (std::get<1>(res)) {
      writeContains(t, b->id, a->id, a->subId);
    }

    // covered
    if (std::get<2>(res)) {
      writeCovers(t, b->id, a->id, a->subId);

      if (fabs(a->area - b->area) < util::geo::EPSILON) {
        // both areas were equivalent
        writeEquals(t, a->id, a->subId, b->id, b->subId);

        writeCovers(t, a->id, b->id, b->subId);
      }
    }

    // touches
    if (std::get<3>(res)) {
      writeTouches(t, a->id, a->subId, b->id, b->subId);
    } else if (std::get<0>(res)) {
      // if a is not a multi-geom, and is completey covered, we wont
      // be finding a touch as we assume non-self-intersecting geoms
      if (_refs.count(a->id) || !(a->subId == 0 && std::get<2>(res))) {
        writeNotTouches(t, a->id, a->subId, b->id, b->subId);
      }
    }

    // overlaps
    if (std::get<4>(res)) {
      _relStats[t].overlaps++;
      writeRel(t, a->id, b->id, _cfg.sepOverlaps);
      _relStats[t].overlaps++;
      writeRel(t, b->id, a->id, _cfg.sepOverlaps);
    }
  } else if (cur.type == LINE &&
             (sv.type == SIMPLE_POLYGON || sv.type == POLYGON)) {
    const Area* b;
    std::shared_ptr<Area> bsp;
    Area bl;

    auto ts = TIME();
    auto a = _lineCache.get(cur.id, cur.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);
    ts = TIME();
    if (sv.type == SIMPLE_POLYGON) {
      bl = areaFromSimpleArea(
          _simpleAreaCache.get(sv.id, sv.large ? -1 : t).get());
      b = &bl;
    } else {
      bsp = _areaCache.get(sv.id, sv.large ? -1 : t);
      b = bsp.get();
    }
    _stats[t].timeGeoCacheRetrievalArea += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    _stats[t].areaCmps++;
    _stats[t].areaSizeSum += b->area;

    _stats[t].lineCmps++;
    _stats[t].lineLenSum += a->length;

    _stats[t].anchorSum += std::max(a->geom.size() / 2, b->geom.size() / 2);

    _stats[t].totalComps++;
    auto totTime = TIME();

    auto res = check(a.get(), b, t);

    _stats[t].timeHisto(
        std::max(a->geom.rawLine().size(), b->geom.getOuter().rawRing().size()),
        TOOK(totTime));

    // intersects
    if (std::get<0>(res)) {
      writeIntersect(t, a->id, b->id);
    }

    // contains
    if (std::get<1>(res)) {
      writeContains(t, b->id, a->id, a->subId);
    }

    // covers
    if (std::get<2>(res)) {
      writeCovers(t, b->id, a->id, a->subId);
    }

    // touches
    if (std::get<3>(res)) {
      writeTouches(t, a->id, a->subId, b->id, b->subId);
    } else if (std::get<0>(res)) {
      // if a is not a multi-geom, and is completey covered, we wont
      // be finding a touch as we assume non-self-intersecting geoms
      if (_refs.count(a->id) || !(a->subId == 0 && std::get<2>(res))) {
        writeNotTouches(t, a->id, a->subId, b->id, b->subId);
      }
    }

    // crosses
    if (std::get<4>(res)) {
      _relStats[t].crosses++;
      writeRel(t, a->id, b->id, _cfg.sepCrosses);
    }
  } else if (cur.type == SIMPLE_LINE &&
             (sv.type == SIMPLE_POLYGON || sv.type == POLYGON)) {
    const Area* b;
    std::shared_ptr<Area> bsp;
    Area bl;

    auto ts = TIME();
    auto a = _simpleLineCache.get(cur.id, cur.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);
    ts = TIME();
    if (sv.type == SIMPLE_POLYGON) {
      bl = areaFromSimpleArea(
          _simpleAreaCache.get(sv.id, sv.large ? -1 : t).get());
      b = &bl;
    } else {
      bsp = _areaCache.get(sv.id, sv.large ? -1 : t);
      b = bsp.get();
    }
    _stats[t].timeGeoCacheRetrievalArea += TOOK(ts);

    if (a->id == b->id)
      return;  // no self-checks in multigeometries
               //
    _stats[t].areaCmps++;
    _stats[t].areaSizeSum += b->area;

    _stats[t].lineCmps++;
    _stats[t].lineLenSum += util::geo::dist(a->a, a->b);

    _stats[t].anchorSum += std::max((size_t)2, b->geom.size() / 2);

    _stats[t].totalComps++;
    auto totTime = TIME();

    auto res = check(a.get(), b, t);

    _stats[t].timeHisto(b->geom.getOuter().rawRing().size(), TOOK(totTime));

    // intersects
    if (std::get<0>(res)) {
      writeIntersect(t, a->id, b->id);
    }

    // contains
    if (std::get<1>(res)) {
      writeContains(t, b->id, a->id, 0);
    }

    // covers
    if (std::get<2>(res)) {
      writeCovers(t, b->id, a->id, 0);
    }

    // touches
    if (std::get<3>(res)) {
      writeTouches(t, a->id, 0, b->id, b->subId);
    } else if (std::get<0>(res)) {
      if (_refs.count(a->id) || !(std::get<2>(res))) {
        writeNotTouches(t, a->id, 0, b->id, b->subId);
      }
    }

    // crosses
    if (std::get<4>(res)) {
      _relStats[t].crosses++;
      writeRel(t, a->id, b->id, _cfg.sepCrosses);
    }
  } else if ((cur.type == SIMPLE_POLYGON || cur.type == POLYGON) &&
             sv.type == LINE) {
    const Area* a;
    std::shared_ptr<Area> asp;
    Area al;

    auto ts = TIME();
    if (cur.type == SIMPLE_POLYGON) {
      al = areaFromSimpleArea(
          _simpleAreaCache.get(cur.id, cur.large ? -1 : t).get());
      a = &al;
    } else {
      asp = _areaCache.get(cur.id, cur.large ? -1 : t);
      a = asp.get();
    }
    _stats[t].timeGeoCacheRetrievalArea += TOOK(ts);
    ts = TIME();
    auto b = _lineCache.get(sv.id, cur.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    if (a->id == b->id)
      return;  // no self-checks in multigeometries
               //
    _stats[t].areaCmps++;
    _stats[t].areaSizeSum += a->area;

    _stats[t].lineCmps++;
    _stats[t].lineLenSum += b->length;

    _stats[t].anchorSum += std::max(a->geom.size() / 2, b->geom.size() / 2);

    _stats[t].totalComps++;
    auto totTime = TIME();

    auto res = check(b.get(), a, t);

    _stats[t].timeHisto(
        std::max(a->geom.getOuter().rawRing().size(), b->geom.rawLine().size()),
        TOOK(totTime));

    // intersects
    if (std::get<0>(res)) {
      writeIntersect(t, a->id, b->id);
    }

    // contains
    if (std::get<1>(res)) {
      writeContains(t, a->id, b->id, b->subId);
    }

    // covers
    if (std::get<2>(res)) {
      writeCovers(t, a->id, b->id, b->subId);
    }

    // touches
    if (std::get<3>(res)) {
      writeTouches(t, a->id, a->subId, b->id, b->subId);
    } else if (std::get<0>(res)) {
      // if b is not a multi-geom, and is completey covered, we wont
      // be finding a touch as we assume non-self-intersecting geoms
      if (_refs.count(a->id) || !(b->subId == 0 && std::get<2>(res))) {
        writeNotTouches(t, a->id, a->subId, b->id, b->subId);
      }
    }

    // crosses
    if (std::get<4>(res)) {
      _relStats[t].crosses++;
      writeRel(t, b->id, a->id, _cfg.sepCrosses);
    }
  } else if ((cur.type == SIMPLE_POLYGON || cur.type == POLYGON) &&
             sv.type == SIMPLE_LINE) {
    const Area* a;
    std::shared_ptr<Area> asp;
    Area al;

    auto ts = TIME();
    if (cur.type == SIMPLE_POLYGON) {
      al = areaFromSimpleArea(
          _simpleAreaCache.get(cur.id, cur.large ? -1 : t).get());
      a = &al;
    } else {
      asp = _areaCache.get(cur.id, cur.large ? -1 : t);
      a = asp.get();
    }
    _stats[t].timeGeoCacheRetrievalArea += TOOK(ts);
    ts = TIME();
    auto b = _simpleLineCache.get(sv.id, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    _stats[t].areaCmps++;
    _stats[t].areaSizeSum += a->area;

    _stats[t].lineCmps++;
    _stats[t].lineLenSum += util::geo::dist(b->a, b->b);

    _stats[t].anchorSum += std::max(a->geom.size() / 2, (size_t)2);

    _stats[t].totalComps++;
    auto totTime = TIME();

    auto res = check(b.get(), a, t);

    _stats[t].timeHisto(a->geom.getOuter().rawRing().size(), TOOK(totTime));

    // intersects
    if (std::get<0>(res)) {
      writeIntersect(t, a->id, b->id);
    }

    // contains
    if (std::get<1>(res)) {
      writeContains(t, a->id, b->id, 0);
    }

    // covers
    if (std::get<2>(res)) {
      writeCovers(t, a->id, b->id, 0);
    }

    // touches
    if (std::get<3>(res)) {
      writeTouches(t, a->id, a->subId, b->id, 0);
    } else if (std::get<0>(res)) {
      if (_refs.count(a->id) || !std::get<2>(res)) {
        writeNotTouches(t, a->id, a->subId, b->id, 0);
      }
    }

    // crosses
    if (std::get<4>(res)) {
      _relStats[t].crosses++;
      writeRel(t, b->id, a->id, _cfg.sepCrosses);
    }
  } else if (cur.type == LINE && sv.type == LINE) {
    auto ts = TIME();
    auto a = _lineCache.get(cur.id, cur.large ? -1 : t);
    auto b = _lineCache.get(sv.id, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    _stats[t].lineCmps++;
    _stats[t].lineLenSum += std::max(a->length, b->length);

    _stats[t].anchorSum += std::max(a->geom.size() / 2, b->geom.size() / 2);

    _stats[t].totalComps++;
    auto totTime = TIME();

    auto res = check(a.get(), b.get(), t);

    _stats[t].timeHisto(
        std::max(a->geom.rawLine().size(), b->geom.rawLine().size()),
        TOOK(totTime));

    // intersects
    if (std::get<0>(res)) {
      writeIntersect(t, a->id, b->id);
    }

    // covers
    if (std::get<1>(res)) {
      writeNotCrosses(t, a->id, a->subId, b->id, b->subId);

      if (a->subId == 0) writeNotOverlaps(t, a->id, a->subId, b->id, b->subId);

      writeCovers(t, b->id, a->id, a->subId);

      if (fabs(a->length - b->length) < util::geo::EPSILON) {
        // both lines were equivalent
        writeEquals(t, a->id, a->subId, b->id, b->subId);

        writeCovers(t, a->id, b->id, b->subId);
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
    auto a = _lineCache.get(cur.id, cur.large ? -1 : t);
    auto b = _simpleLineCache.get(sv.id, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    _stats[t].lineCmps++;
    _stats[t].lineLenSum += std::max(a->length, util::geo::dist(b->a, b->b));

    _stats[t].anchorSum += std::max(a->geom.size() / 2, (size_t)2);

    _stats[t].totalComps++;
    auto totTime = TIME();

    auto res = check(a.get(), b.get(), t);

    _stats[t].timeHisto(a->geom.rawLine().size(), TOOK(totTime));

    // intersects
    if (std::get<0>(res)) {
      writeIntersect(t, a->id, b->id);
    }

    // covers
    if (std::get<1>(res)) {
      writeNotCrosses(t, a->id, a->subId, b->id, 0);

      writeCovers(t, b->id, a->id, a->subId);

      if (fabs(a->length - util::geo::len(LineSegment<int32_t>(b->a, b->b))) <
          util::geo::EPSILON) {
        // both lines were equivalent
        writeCovers(t, a->id, b->id, 0);

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
    auto a = _simpleLineCache.get(cur.id, sv.large ? -1 : t);
    auto b = _lineCache.get(sv.id, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    _stats[t].lineCmps++;
    _stats[t].lineLenSum += std::max(b->length, util::geo::dist(a->a, a->b));

    _stats[t].anchorSum += std::max(b->geom.size() / 2, (size_t)2);

    _stats[t].totalComps++;
    auto totTime = TIME();

    auto res = check(a.get(), b.get(), t);

    _stats[t].timeHisto(b->geom.rawLine().size(), TOOK(totTime));

    // intersects
    if (std::get<0>(res)) {
      writeIntersect(t, a->id, b->id);
    }

    // covers
    if (std::get<1>(res)) {
      writeNotCrosses(t, a->id, 0, b->id, b->subId);
      writeCovers(t, b->id, a->id, 0);

      writeNotOverlaps(t, a->id, 0, b->id, b->subId);

      if (fabs(util::geo::len(LineSegment<int32_t>(a->a, a->b)) - b->length) <
          util::geo::EPSILON) {
        writeEquals(t, a->id, 0, b->id, b->subId);

        writeCovers(t, a->id, b->id, b->subId);
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
    auto a = _simpleLineCache.get(cur.id, cur.large ? -1 : t);
    auto b = _simpleLineCache.get(sv.id, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    _stats[t].lineCmps++;
    _stats[t].lineLenSum +=
        std::max(util::geo::dist(a->a, a->b), util::geo::dist(b->a, b->b));

    _stats[t].anchorSum += 2;

    _stats[t].totalComps++;
    auto totTime = TIME();

    auto res = check(a.get(), b.get(), t);

    _stats[t].timeHisto(2, TOOK(totTime));

    if (std::get<0>(res)) {
      writeIntersect(t, a->id, b->id);
    }

    if (std::get<1>(res)) {
      writeCovers(t, b->id, a->id, 0);

      if (fabs(util::geo::len(LineSegment<int32_t>(a->a, a->b)) -
               util::geo::len(LineSegment<int32_t>(b->a, b->b))) <
          util::geo::EPSILON) {
        writeEquals(t, a->id, 0, b->id, 0);
        writeCovers(t, a->id, b->id, 0);
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
    auto a = _pointCache.get(cur.id, cur.large ? -1 : t);
    auto b = _pointCache.get(sv.id, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalPoint += TOOK(ts);

    _stats[t].anchorSum += 1;

    if (a->id == b->id) return;  // no self-checks in multigeometries

    writeIntersect(t, a->id, b->id);
    writeEquals(t, a->id, a->subId, b->id, b->subId);

    writeCovers(t, b->id, a->id, a->subId);
    writeContains(t, b->id, a->id, a->subId);

    writeCovers(t, a->id, b->id, b->subId);
    writeContains(t, a->id, b->id, b->subId);
  } else if (cur.type == POINT && sv.type == SIMPLE_LINE) {
    auto p = I32Point(cur.val, cur.loY);
    auto ts = TIME();
    auto b = _simpleLineCache.get(sv.id, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    _stats[t].lineCmps++;
    _stats[t].lineLenSum += util::geo::dist(b->a, b->b);

    _stats[t].anchorSum += 2;

    if (util::geo::contains(p, LineSegment<int32_t>(b->a, b->b))) {
      auto ts = TIME();
      auto a = _pointCache.get(cur.id, sv.large ? -1 : t);
      _stats[t].timeGeoCacheRetrievalPoint += TOOK(ts);
      writeIntersect(t, a->id, b->id);

      writeCovers(t, b->id, a->id, a->subId);

      if (p != b->a && p != b->b) {
        writeContains(t, b->id, a->id, a->subId);

        writeNotTouches(t, a->id, a->subId, b->id, 0);
      } else {
        writeTouches(t, a->id, a->subId, b->id, 0);
      }
    }
  } else if (cur.type == POINT && sv.type == LINE) {
    auto ts = TIME();
    auto a = I32Point(cur.val, cur.loY);
    auto b = _lineCache.get(sv.id, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    _stats[t].lineCmps++;
    _stats[t].lineLenSum += b->length;

    _stats[t].anchorSum += b->geom.size() / 2;

    _stats[t].totalComps++;
    auto totTime = TIME();

    auto res = check(a, b.get(), t);

    _stats[t].timeHisto(b->geom.rawLine().size(), TOOK(totTime));

    if (std::get<0>(res)) {
      auto ts = TIME();
      auto a = _pointCache.get(cur.id, cur.large ? -1 : t);
      _stats[t].timeGeoCacheRetrievalPoint += TOOK(ts);

      if (a->id == b->id) return;  // no self-checks in multigeometries

      writeIntersect(t, a->id, b->id);

      writeCovers(t, b->id, a->id, a->subId);

      if (std::get<1>(res)) {
        writeContains(t, b->id, a->id, a->subId);
        writeNotTouches(t, a->id, a->subId, b->id, b->subId);
      } else {
        writeTouches(t, a->id, a->subId, b->id, b->subId);
      }

      if (b->length == 0) {
        // zero length line, point also covers line

        writeCovers(t, a->id, b->id, b->subId);
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
      bl = areaFromSimpleArea(
          _simpleAreaCache.get(sv.id, sv.large ? -1 : t).get());
      b = &bl;
    } else {
      bsp = _areaCache.get(sv.id, sv.large ? -1 : t);
      b = bsp.get();
    }
    _stats[t].timeGeoCacheRetrievalArea += TOOK(ts);

    _stats[t].totalComps++;
    auto totTime = TIME();

    auto res = check(a, b, t);

    _stats[t].timeHisto(b->geom.getOuter().rawRing().size(), TOOK(totTime));

    if (res.second) {
      auto ts = TIME();
      auto a = _pointCache.get(cur.id, cur.large ? -1 : t);
      _stats[t].timeGeoCacheRetrievalPoint += TOOK(ts);

      writeCovers(t, b->id, a->id, a->subId);
      writeIntersect(t, a->id, b->id);

      if (res.first) {
        writeContains(t, b->id, a->id, a->subId);

        if (_refs.count(a->id) || a->subId != 0) {
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
  if (_cfg.writeRelCb) return;

  if (_outMode == COUT) {
    for (size_t i = 0; i < _cfg.numThreads + 1; i++) {
      _outBuffers[i][_outBufPos[i]] = 0;
      fputs(reinterpret_cast<const char*>(_outBuffers[i]), stdout);
    }
  }
  if (_outMode == BZ2 || _outMode == GZ || _outMode == PLAIN) {
    if (_outMode == BZ2) {
#ifndef SPATIALJOIN_NO_BZIP2
      for (size_t i = 0; i < _cfg.numThreads + 1; i++) {
        int err = 0;
        BZ2_bzWrite(&err, _bzFiles[i], _outBuffers[i], _outBufPos[i]);
        if (err == BZ_IO_ERROR) {
          BZ2_bzWriteClose(&err, _bzFiles[i], 0, 0, 0);
          std::string fname = _cache + "/.rels" + std::to_string(getpid()) +
                              "-" + std::to_string(i);
          std::stringstream ss;
          ss << "Could not write spatial relation to temporary bzip2 file '"
             << fname << "':\n";
          ss << strerror(errno) << std::endl;
          throw std::runtime_error(ss.str());
        }
        BZ2_bzWriteClose(&err, _bzFiles[i], 0, 0, 0);
        fclose(_rawFiles[i]);
      }
#endif
    } else if (_outMode == GZ) {
#ifndef SPATIALJOIN_NO_ZLIB
      for (size_t i = 0; i < _cfg.numThreads + 1; i++) {
        int r = gzwrite(_gzFiles[i], _outBuffers[i], _outBufPos[i]);
        if (r != (int)_outBufPos[i]) {
          gzclose(_gzFiles[i]);
          std::string fname = _cache + "/.rels" + std::to_string(getpid()) +
                              "-" + std::to_string(i);
          std::stringstream ss;
          ss << "Could not write spatial relation to temporary gzip file '"
             << fname << "':\n";
          ss << strerror(errno) << std::endl;
          throw std::runtime_error(ss.str());
        }
        gzclose(_gzFiles[i]);
      }
#endif
    } else {
      for (size_t i = 0; i < _cfg.numThreads + 1; i++) {
        size_t r =
            fwrite(_outBuffers[i], sizeof(char), _outBufPos[i], _rawFiles[i]);
        if (r != _outBufPos[i]) {
          std::string fname = _cache + "/.rels" + std::to_string(getpid()) +
                              "-" + std::to_string(i);
          std::stringstream ss;
          ss << "Could not write spatial relation to temporary file '" << fname
             << "':\n";
          ss << strerror(errno) << std::endl;
          throw std::runtime_error(ss.str());
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
      if (ifCur.peek() != std::ifstream::traits_type::eof())
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
  if (_cfg.writeRelCb) return;

  if (_outMode == BZ2) {
#ifndef SPATIALJOIN_NO_BZIP2
    for (size_t i = 0; i < _cfg.numThreads + 1; i++) {
      std::string fname = _cache + "/.rels" + std::to_string(getpid()) + "-" +
                          std::to_string(i);
      _rawFiles[i] = fopen(fname.c_str(), "w");

      if (_rawFiles[i] == NULL) {
        std::stringstream ss;
        ss << "Could not open temporary bzip2 file '" << fname
           << "' for writing:\n";
        ss << strerror(errno) << std::endl;
        throw std::runtime_error(ss.str());
      }

      int err = 0;
      _bzFiles[i] = BZ2_bzWriteOpen(&err, _rawFiles[i], 6, 0, 30);
      if (err != BZ_OK) {
        std::stringstream ss;
        ss << "Could not open temporary bzip2 file '" << fname
           << "' for writing:\n";
        ss << strerror(errno) << std::endl;
        throw std::runtime_error(ss.str());
      }
      _outBuffers[i] = new unsigned char[BUFFER_S_PAIRS];
    }
#endif
  } else if (_outMode == GZ) {
#ifndef SPATIALJOIN_NO_ZLIB
    for (size_t i = 0; i < _cfg.numThreads + 1; i++) {
      std::string fname = _cache + "/.rels" + std::to_string(getpid()) + "-" +
                          std::to_string(i);
      _gzFiles[i] = gzopen(fname.c_str(), "w");
      if (_gzFiles[i] == Z_NULL) {
        std::stringstream ss;
        ss << "Could not open temporary gzip file '" << fname
           << "' for writing:\n";
        ss << strerror(errno) << std::endl;
        throw std::runtime_error(ss.str());
      }
      _outBuffers[i] = new unsigned char[BUFFER_S_PAIRS];
    }
#endif
  } else if (_outMode == PLAIN) {
    for (size_t i = 0; i < _cfg.numThreads + 1; i++) {
      std::string fname = _cache + "/.rels" + std::to_string(getpid()) + "-" +
                          std::to_string(i);
      _rawFiles[i] = fopen(fname.c_str(), "w");

      if (_rawFiles[i] == NULL) {
        std::stringstream ss;
        ss << "Could not open temporary file '" << fname << "' for writing:\n";
        ss << strerror(errno) << std::endl;
        throw std::runtime_error(ss.str());
      }

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
  try {
    JobBatch batch;
    while ((batch = _jobs.get()).size()) {
      for (const auto& job : batch) {
        if (job.multiOut.empty()) {
          if (_cfg.withinDist >= 0) {
            doDistCheck(job.boxVal, job.sweepVal, t);
          } else {
            doCheck(job.boxVal, job.sweepVal, t);
          }
        } else {
          multiOut(t, job.multiOut);
        }
      }
    }
  } catch (const std::runtime_error& e) {
    std::stringstream ss;
    ss << "libspatialjoin: " << e.what();
    std::cerr << ss.str() << std::endl;
    std::exit(1);
  }

  _atomicCurX[t] = _curX[t];
}

// _____________________________________________________________________________
void Sweeper::fillBatch(
    JobBatch* batch, const util::geo::IntervalIdx<int32_t, SweepVal>* actives,
    const BoxVal* cur) const {
  const auto& overlaps = actives->overlap_find_all({cur->loY, cur->upY});

  for (const auto& p : overlaps) {
    if (_cfg.useDiagBox && !util::geo::intersects(p.v.b45, cur->b45)) continue;

    batch->push_back({*cur, p.v, ""});
  }
}

// _____________________________________________________________________________
void Sweeper::writeOverlaps(size_t t, const std::string& a, size_t aSub,
                            const std::string& b, size_t bSub) {
  if (a != b) {
    if (aSub == 0 && bSub == 0) {
      _relStats[t].overlaps++;
      writeRel(t, a, b, _cfg.sepOverlaps);
      _relStats[t].overlaps++;
      writeRel(t, b, a, _cfg.sepOverlaps);
    } else {
      std::unique_lock<std::mutex> lock(_mutsOverlaps[t]);

      if (bSub != 0) _subOverlaps[t][b].insert(a);
      if (aSub != 0) _subOverlaps[t][a].insert(b);
    }
  }

  if (_refs.size() == 0) return;

  // handle references

  auto referersA = _refs.find(a);
  auto referersB = _refs.find(b);

  if (referersB != _refs.end()) {
    for (const auto& idB : referersB->second) {
      writeOverlaps(t, a, aSub, idB.first, idB.second);
    }
  }

  if (referersA != _refs.end()) {
    for (const auto& idA : referersA->second) {
      writeOverlaps(t, idA.first, idA.second, b, bSub);
    }
  }
}

// _____________________________________________________________________________
void Sweeper::writeNotOverlaps(size_t t, const std::string& a, size_t aSub,
                               const std::string& b, size_t bSub) {
  if (a == b) return;

  if ((aSub != 0 || bSub != 0) && !refRelated(a, b)) {
    std::unique_lock<std::mutex> lock(_mutsNotOverlaps[t]);

    if (bSub != 0) _subNotOverlaps[t][b].insert(a);
    if (aSub != 0) _subNotOverlaps[t][a].insert(b);
  }

  if (_refs.size() == 0) return;

  // handle references

  auto referersA = _refs.find(a);
  auto referersB = _refs.find(b);

  if (referersB != _refs.end()) {
    for (const auto& idB : referersB->second) {
      writeNotOverlaps(t, a, aSub, idB.first, idB.second);
    }
  }

  if (referersA != _refs.end()) {
    for (const auto& idA : referersA->second) {
      writeNotOverlaps(t, idA.first, idA.second, b, bSub);
    }
  }
}

// _____________________________________________________________________________
void Sweeper::writeCrosses(size_t t, const std::string& a, size_t aSub,
                           const std::string& b, size_t bSub) {
  if (a == b) return;

  if (aSub == 0 && bSub == 0) {
    _relStats[t].crosses++;
    writeRel(t, a, b, _cfg.sepCrosses);
    _relStats[t].crosses++;
    writeRel(t, b, a, _cfg.sepCrosses);
  } else {
    std::unique_lock<std::mutex> lock(_mutsCrosses[t]);

    if (bSub != 0) _subCrosses[t][b].insert(a);
    if (aSub != 0) _subCrosses[t][a].insert(b);
  }

  if (_refs.size() == 0) return;

  // handle references

  auto referersA = _refs.find(a);
  auto referersB = _refs.find(b);

  if (referersB != _refs.end()) {
    for (const auto& idB : referersB->second) {
      writeCrosses(t, a, aSub, idB.first, idB.second);
    }
  }

  if (referersA != _refs.end()) {
    for (const auto& idA : referersA->second) {
      writeCrosses(t, idA.first, idA.second, b, bSub);
    }
  }
}

// _____________________________________________________________________________
void Sweeper::writeNotCrosses(size_t t, const std::string& a, size_t aSub,
                              const std::string& b, size_t bSub) {
  if (a == b) return;
  if ((aSub != 0 || bSub != 0) && !refRelated(a, b)) {
    std::unique_lock<std::mutex> lock(_mutsNotCrosses[t]);

    if (bSub != 0) _subNotCrosses[t][b].insert(a);
    if (aSub != 0) _subNotCrosses[t][a].insert(b);
  }

  if (_refs.size() == 0) return;

  // handle references

  auto referersA = _refs.find(a);
  auto referersB = _refs.find(b);

  if (referersB != _refs.end()) {
    for (const auto& idB : referersB->second) {
      writeNotCrosses(t, a, aSub, idB.first, idB.second);
    }
  }

  if (referersA != _refs.end()) {
    for (const auto& idA : referersA->second) {
      writeNotCrosses(t, idA.first, idA.second, b, bSub);
    }
  }
}

// _____________________________________________________________________________
void Sweeper::writeTouches(size_t t, const std::string& a, size_t aSub,
                           const std::string& b, size_t bSub) {
  if (a == b) return;

  if (aSub == 0 && bSub == 0) {
    _relStats[t].touches++;
    writeRel(t, a, b, _cfg.sepTouches);
    _relStats[t].touches++;
    writeRel(t, b, a, _cfg.sepTouches);
  } else {
    std::unique_lock<std::mutex> lock(_mutsTouches[t]);

    if (bSub != 0) _subTouches[t][b].insert(a);
    if (aSub != 0) _subTouches[t][a].insert(b);
  }

  if (_refs.size() == 0) return;

  // handle references

  auto referersA = _refs.find(a);
  auto referersB = _refs.find(b);

  if (referersB != _refs.end()) {
    for (const auto& idB : referersB->second) {
      writeTouches(t, a, aSub, idB.first, idB.second);
    }
  }

  if (referersA != _refs.end()) {
    for (const auto& idA : referersA->second) {
      writeTouches(t, idA.first, idA.second, b, bSub);
    }
  }
}

// _____________________________________________________________________________
void Sweeper::writeNotTouches(size_t t, const std::string& a, size_t aSub,
                              const std::string& b, size_t bSub) {
  if (a == b) return;

  if ((aSub != 0 || bSub != 0) && !refRelated(a, b)) {
    std::unique_lock<std::mutex> lock(_mutsNotTouches[t]);

    if (bSub != 0) _subNotTouches[t][b].insert(a);
    if (aSub != 0) _subNotTouches[t][a].insert(b);
  }

  if (_refs.size() == 0) return;

  // handle references

  auto referersA = _refs.find(a);
  auto referersB = _refs.find(b);

  if (referersB != _refs.end()) {
    for (const auto& idB : referersB->second) {
      writeNotTouches(t, a, aSub, idB.first, idB.second);
    }
  }

  if (referersA != _refs.end()) {
    for (const auto& idA : referersA->second) {
      writeNotTouches(t, idA.first, idA.second, b, bSub);
    }
  }
}

// _____________________________________________________________________________
void Sweeper::writeEquals(size_t t, const std::string& a, size_t aSub,
                          const std::string& b, size_t bSub) {
  if (a != b) {
    if (aSub == 0 && bSub == 0) {
      writeRel(t, a, b, _cfg.sepEquals);
      _relStats[t].equals++;
      writeRel(t, b, a, _cfg.sepEquals);
      _relStats[t].equals++;
    } else if (aSub == 0 || bSub == 0) {
      writeNotOverlaps(t, a, aSub, b, bSub);
    } else if (_subSizes[a] != _subSizes[b]) {
    } else {
      std::unique_lock<std::mutex> lock(_mutsEquals[t]);

      _subEquals[t][b][a].insert(aSub);
      _subEquals[t][a][b].insert(bSub);
    }
  }

  if (_refs.size() == 0) return;

  // handle references

  auto referersA = _refs.find(a);
  auto referersB = _refs.find(b);

  if (referersB != _refs.end()) {
    for (const auto& idB : referersB->second) {
      writeEquals(t, a, aSub, idB.first, idB.second);
    }
  }

  if (referersA != _refs.end()) {
    for (const auto& idA : referersA->second) {
      writeEquals(t, idA.first, idA.second, b, bSub);
    }
  }
}

// _____________________________________________________________________________
void Sweeper::writeCovers(size_t t, const std::string& a, const std::string& b,
                          size_t bSub) {
  if (a != b) {
    if (bSub > 0) {
      {
        std::unique_lock<std::mutex> lock(_mutsCovers[t]);
        _subCovered[t][b][a].insert(bSub);
      }
    } else {
      writeRel(t, a, b, _cfg.sepCovers);
      _relStats[t].covers++;
    }
  }

  if (_refs.size() == 0) return;

  // handle references

  auto referersA = _refs.find(a);
  auto referersB = _refs.find(b);

  if (referersB != _refs.end()) {
    for (const auto& idB : referersB->second) {
      writeCovers(t, a, idB.first, idB.second);
    }
  }

  if (referersA != _refs.end()) {
    for (const auto& idA : referersA->second) {
      writeCovers(t, idA.first, b, bSub);
    }
  }
}

// _____________________________________________________________________________
void Sweeper::writeContains(size_t t, const std::string& a,
                            const std::string& b, size_t bSub) {
  if (a != b) {
    if (bSub > 0) {
      std::unique_lock<std::mutex> lock(_mutsContains[t]);
      _subContains[t][b][a].insert(bSub);
    } else {
      writeRel(t, a, b, _cfg.sepContains);
      _relStats[t].contains++;
    }
  }

  if (_refs.size() == 0) return;

  // handle references

  auto referersA = _refs.find(a);
  auto referersB = _refs.find(b);

  if (referersB != _refs.end()) {
    for (const auto& idB : referersB->second) {
      writeContains(t, a, idB.first, idB.second);
    }
  }

  if (referersA != _refs.end()) {
    for (const auto& idA : referersA->second) {
      writeContains(t, idA.first, b, bSub);
    }
  }
}

// _____________________________________________________________________________
bool Sweeper::notCrosses(const std::string& a, const std::string& b) {
  if (refRelated(a, b)) return true;

  for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
    std::unique_lock<std::mutex> lock(_mutsNotCrosses[t]);
    auto i = _subNotCrosses[t].find(a);
    if (i != _subNotCrosses[t].end() && i->second.count(b)) return true;
  }

  return false;
}

// _____________________________________________________________________________
bool Sweeper::notTouches(const std::string& a, const std::string& b) {
  if (refRelated(a, b)) return true;

  for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
    std::unique_lock<std::mutex> lock(_mutsNotTouches[t]);
    auto i = _subNotTouches[t].find(a);
    if (i != _subNotTouches[t].end() && i->second.count(b)) return true;
  }

  return false;
}

// _____________________________________________________________________________
bool Sweeper::notOverlaps(const std::string& a, const std::string& b) {
  if (refRelated(a, b)) return true;

  for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
    std::unique_lock<std::mutex> lock(_mutsNotOverlaps[t]);
    auto i = _subNotOverlaps[t].find(a);
    if (i != _subNotOverlaps[t].end() && i->second.find(b) != i->second.end())
      return true;
  }

  return false;
}

// _____________________________________________________________________________
void Sweeper::log(const std::string& msg) {
  if (_cfg.logCb) _cfg.logCb(msg);
}

// _____________________________________________________________________________
bool Sweeper::refRelated(const std::string& a, const std::string& b) const {
  auto i = _refs.find(a);
  if (i != _refs.end() && i->second.find(b) != i->second.end()) return true;

  auto j = _refs.find(b);
  if (j != _refs.end() && j->second.find(a) != j->second.end()) return true;

  return false;
}

// _____________________________________________________________________________
std::string Sweeper::intToBase126(uint64_t id) {
  if (id == 0) return std::string("\1");

  std::string ret;
  ret.reserve(::log(id) / ::log(126) + 1);

  div_t d = {id, 0};
  uint64_t pos = 126;

  do {
    d = div(d.quot, pos);
    ret.push_back(d.rem + 1);  // avoid 0 bytes, although std::string allows it
    pos *= 126;
  } while (d.quot);

  return ret;
}

// _____________________________________________________________________________
double Sweeper::getMaxScaleFactor(const I32Box& bbox) const {
  double invScaleFactor = std::min(
      util::geo::webMercDistFactor(I32Point{bbox.getLowerLeft().getX() / PREC,
                                            bbox.getLowerLeft().getY() / PREC}),
      util::geo::webMercDistFactor(
          I32Point{bbox.getUpperRight().getX() / PREC,
                   bbox.getUpperRight().getY() / PREC}));

  return 1.0 / invScaleFactor;
}

// _____________________________________________________________________________
double Sweeper::getMaxScaleFactor(const I32Point& p) const {
  return 1.0 / util::geo::webMercDistFactor(
                   I32Point{p.getX() / PREC, p.getY() / PREC});
}

// _____________________________________________________________________________
uint64_t Sweeper::base126ToInt(const std::string& id) {
  uint64_t ret = 0;
  uint64_t pos = 1;
  for (size_t i = 0; i < id.size(); i++) {
    ret += (id[i] - 1) * pos;
    pos *= 126;
  }
  return ret;
}

// _____________________________________________________________________________
double Sweeper::meterDist(const I32Point& p1, const I32Point& p2) {
  return util::geo::webMercMeterDist(FPoint{(p1.getX() * 1.0) / (PREC * 1.0),
                                            (p1.getY() * 1.0) / (PREC * 1.0)},
                                     FPoint{(p2.getX() * 1.0) / (PREC * 1.0),
                                            (p2.getY() * 1.0) / (PREC * 1.0)});
}

// _____________________________________________________________________________
double Sweeper::distCheck(const I32Point& a, const Area* b, size_t t) const {
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect({{1, 0}, {getBoxId(a), 0}}, b->boxIds);
    _stats[t].timeBoxIdIsectAreaPoint += TOOK(ts);

    // all boxes of a are fully contained in b, we are contained
    if (r.first) return 0;
  }

  auto ts = TIME();
  double scaleFactor =
      std::max(getMaxScaleFactor(a), getMaxScaleFactor(b->box)) * PREC;

  auto dist =
      util::geo::withinDist<int32_t>(a, b->geom, _cfg.withinDist * scaleFactor,
                                     _cfg.withinDist, &Sweeper::meterDist);

  _stats[t].timeFullGeoCheckAreaPoint += TOOK(ts);
  _stats[t].fullGeoChecksAreaPoint++;

  return dist;
}

// _____________________________________________________________________________
double Sweeper::distCheck(const I32Point& a, const Line* b, size_t t) const {
  auto ts = TIME();
  double scaleFactor =
      std::max(getMaxScaleFactor(a), getMaxScaleFactor(b->box)) * PREC;

  auto dist =
      util::geo::withinDist<int32_t>(a, b->geom, _cfg.withinDist * scaleFactor,
                                     _cfg.withinDist, &Sweeper::meterDist);
  _stats[t].timeFullGeoCheckLinePoint += TOOK(ts);
  _stats[t].fullGeoChecksLinePoint++;

  return dist;
}

// _____________________________________________________________________________
double Sweeper::distCheck(const I32Point& a, const SimpleLine* b,
                          size_t t) const {
  auto ts = TIME();

  auto p2 = projectOn(b->a, a, b->b);

  auto dist = Sweeper::meterDist(a, p2);

  _stats[t].timeFullGeoCheckLinePoint += TOOK(ts);
  _stats[t].fullGeoChecksLinePoint++;

  return dist;
}

// _____________________________________________________________________________
double Sweeper::distCheck(const SimpleLine* a, const SimpleLine* b,
                          size_t t) const {
  auto ts = TIME();

  auto dist = util::geo::dist<int32_t>(LineSegment<int32_t>(a->a, a->b),
                                       LineSegment<int32_t>(b->a, b->b),
                                       &Sweeper::meterDist);

  _stats[t].timeFullGeoCheckLineLine += TOOK(ts);
  _stats[t].fullGeoChecksLineLine++;

  return dist;
}

// _____________________________________________________________________________
double Sweeper::distCheck(const SimpleLine* a, const Line* b, size_t t) const {
  auto ts = TIME();
  double scaleFactor =
      std::max(std::max(getMaxScaleFactor(a->b), getMaxScaleFactor(a->a)),
               getMaxScaleFactor(b->box));

  auto dist = util::geo::withinDist<int32_t>(
      I32XSortedLine(LineSegment<int32_t>(a->a, a->b)), b->geom,
      getBoundingBox(LineSegment<int32_t>(a->a, a->b)), b->box,
      _cfg.withinDist * scaleFactor * PREC,
      _cfg.withinDist * scaleFactor * PREC, _cfg.withinDist,
      &Sweeper::meterDist);

  _stats[t].timeFullGeoCheckAreaLine += TOOK(ts);
  _stats[t].fullGeoChecksAreaLine++;

  return dist;
}

// _____________________________________________________________________________
double Sweeper::distCheck(const Line* a, const Line* b, size_t t) const {
  auto ts = TIME();
  double scaleFactor =
      std::max(getMaxScaleFactor(a->box), getMaxScaleFactor(b->box));

  auto dist = util::geo::withinDist<int32_t>(
      a->geom, b->geom, a->box, b->box, _cfg.withinDist * scaleFactor * PREC,
      _cfg.withinDist * scaleFactor * PREC, _cfg.withinDist,
      &Sweeper::meterDist);

  _stats[t].timeFullGeoCheckLineLine += TOOK(ts);
  _stats[t].fullGeoChecksLineLine++;

  return dist;
}

// _____________________________________________________________________________
double Sweeper::distCheck(const SimpleLine* a, const Area* b, size_t t) const {
  auto ts = TIME();

  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect({{1, 0}, {getBoxId(a->a), 0}}, b->boxIds);
    _stats[t].timeBoxIdIsectAreaLine += TOOK(ts);

    if (r.first) return 0;
  }

  double scaleFactor =
      std::max(getMaxScaleFactor(b->box),
               std::max(getMaxScaleFactor(a->b), getMaxScaleFactor(a->a)));

  auto dist = util::geo::withinDist<int32_t>(
      I32XSortedLine(LineSegment<int32_t>(a->a, a->b)), b->geom,
      getBoundingBox(LineSegment<int32_t>(a->a, a->b)), b->box,
      _cfg.withinDist * scaleFactor * PREC,
      _cfg.withinDist * scaleFactor * PREC, _cfg.withinDist,
      &Sweeper::meterDist);

  _stats[t].timeFullGeoCheckAreaLine += TOOK(ts);
  _stats[t].fullGeoChecksAreaLine++;

  return dist;
}

// _____________________________________________________________________________
double Sweeper::distCheck(const Line* a, const Area* b, size_t t) const {
  auto ts = TIME();

  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect(a->boxIds, b->boxIds);
    _stats[t].timeBoxIdIsectAreaLine += TOOK(ts);

    // all boxes of a are fully contained in b, we intersect and we are
    // contained
    if (r.first) return 0;
  }

  double scaleFactor =
      std::max(getMaxScaleFactor(a->box), getMaxScaleFactor(b->box));

  auto dist = util::geo::withinDist<int32_t>(
      a->geom, b->geom, a->box, b->box, _cfg.withinDist * scaleFactor * PREC,
      _cfg.withinDist * scaleFactor * PREC, _cfg.withinDist,
      &Sweeper::meterDist);

  _stats[t].timeFullGeoCheckAreaLine += TOOK(ts);
  _stats[t].fullGeoChecksAreaLine++;

  return dist;
}

// _____________________________________________________________________________
double Sweeper::distCheck(const Area* a, const Area* b, size_t t) const {
  auto ts = TIME();

  // cheap equivalence check
  if (a->box == b->box && a->area == b->area && a->geom == b->geom) {
    // equivalent!
    return 0;
  }

  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect(a->boxIds, b->boxIds);
    _stats[t].timeBoxIdIsectAreaArea += TOOK(ts);

    // all boxes of a are fully contained in b, we intersect and we are
    // contained and we do not touch or overlap
    if (r.first) return 0;
  }

  double scaleFactor =
      std::max(getMaxScaleFactor(a->box), getMaxScaleFactor(b->box));

  auto dist = util::geo::withinDist<int32_t>(
      a->geom, b->geom, a->box, b->box, _cfg.withinDist * scaleFactor * PREC,
      _cfg.withinDist * scaleFactor * PREC, _cfg.withinDist,
      &Sweeper::meterDist);

  _stats[t].timeFullGeoCheckAreaArea += TOOK(ts);
  _stats[t].fullGeoChecksAreaArea++;

  return dist;
}
