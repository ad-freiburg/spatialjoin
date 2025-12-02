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

using sj::GeomCheckRes;
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
using util::geo::DE9IM;
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
I32Box Sweeper::add(const I32MultiPoint& a, const std::string& gid,
                    size_t subid, bool side, WriteBatch& batch) const {
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
I32Box Sweeper::add(const I32Polygon& poly, const std::string& gidR,
                    size_t subid, bool side, WriteBatch& batch) const {
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

  if (_cfg.useBoxIds) {
    boxIds = packBoxIds(getBoxIds(spoly, rawBox, outerAreaSize));
  }

  I32Box box45;
  if (_cfg.useDiagBox) {
    auto polyR = util::geo::rotateSinCos(poly, sin45, cos45, I32Point(0, 0));
    box45 = getPaddedBoundingBox(polyR, rawBox);
  }

  cur.subid = subid;
  cur.gid = gid;

  if (poly.getInners().size() == 0 && subid == 0 && gid.size() < 8 &&
      (!_cfg.useBoxIds || boxIds.front().first == 1) &&
      area(rawBox) == areaSize) {
    cur.boxvalIn = {0,  // placeholder, will be overwritten later on
                    box.getLowerLeft().getY(),
                    box.getUpperRight().getY(),
                    box.getLowerLeft().getX(),
                    false,
                    FOLDED_BOX_POLYGON,
                    areaSize,
                    box.getUpperRight(),
                    box45,
                    side,
                    false};
    cur.boxvalOut = {0,  // placeholder, will be overwritten later on
                     box.getLowerLeft().getY(),
                     box.getUpperRight().getY(),
                     box.getUpperRight().getX(),
                     true,
                     FOLDED_BOX_POLYGON,
                     areaSize,
                     box.getLowerLeft(),
                     box45,
                     side,
                     false};
    batch.foldedBoxAreas.emplace_back(cur);
  } else if (poly.getInners().size() == 0 && poly.getOuter().size() < 10 &&
             subid == 0 && (!_cfg.useBoxIds || boxIds.front().first == 1)) {
    std::stringstream str;
    _simpleAreaCache.writeTo({poly.getOuter(), gid}, str);
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
    _areaCache.writeTo(
        {std::move(spoly), box, gid, subid, areaSize,
         _cfg.useArea ? outerAreaSize : 0, boxIds, obb, inner, innerBox,
         innerOuterAreaSize, outer, outerBox, outerOuterAreaSize},
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

  if (_cfg.useBoxIds) {
    boxIds = packBoxIds(getBoxIds(line, rawBox));
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

    cur.boxvalIn = {
        0,  // placeholder, will be overwritten later on
        box.getLowerLeft().getY(),
        box.getUpperRight().getY(),
        box.getLowerLeft().getX(),
        false,
        SIMPLE_LINE,
        len,
        line.front().getX() < line.back().getX() ? line.back() : line.front(),
        box45,
        side,
        false};
    cur.boxvalOut = {
        0,  // placeholder, will be overwritten later on,
        box.getLowerLeft().getY(),
        box.getUpperRight().getY(),
        box.getUpperRight().getX(),
        true,
        SIMPLE_LINE,
        len,
        line.front().getX() < line.back().getX() ? line.front() : line.back(),
        box45,
        side,
        false};

    // check if we can fold the gid into the offset id, because the gid is all
    // we store in the cache for points
    if (subid == 0 && gid.size() < 8) {
      cur.boxvalIn.type = FOLDED_SIMPLE_LINE;
      cur.boxvalOut.type = FOLDED_SIMPLE_LINE;
      batch.foldedSimpleLines.emplace_back(cur);
    } else {
      std::stringstream str;
      _simpleLineCache.writeTo({gid}, str);

      cur.raw = str.str();

      batch.simpleLines.emplace_back(cur);
    }
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
    _lineCache.writeTo({std::move(sline), box, gid, subid, len, boxIds, obb},
                       str);
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
I32Box Sweeper::add(const I32Point& point, const std::string& gidR,
                    size_t subid, bool side, WriteBatch& batch) const {
  std::string gid = (side ? ("B" + gidR) : ("A" + gidR));

  WriteCand cur;

  const auto& rawBox = getBoundingBox(point);
  const auto& box = getPaddedBoundingBox(rawBox);

  cur.subid = subid;

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

  cur.gid = gid;

  // check if we can fold the gid into the offset id, because the gid is all
  // we store in the cache for points
  if (subid == 0 && gid.size() < 8) {
    cur.boxvalIn.type = FOLDED_POINT;
    cur.boxvalOut.type = FOLDED_POINT;
    batch.foldedPoints.emplace_back(cur);
  } else {
    std::stringstream str;
    _pointCache.writeTo({gid, subid}, str);

    cur.raw = str.str();

    batch.points.emplace_back(cur);
  }

  return box;
}

// _____________________________________________________________________________
void Sweeper::addBatch(WriteBatch& cands) {
  {
    for (auto& cand : cands.foldedPoints) {
      if (cand.boxvalIn.side) _numSides = 2;
      cand.boxvalIn.id = foldString(cand.gid);
      cand.boxvalOut.id = cand.boxvalIn.id;
    }
  }

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
    for (auto& cand : cands.foldedSimpleLines) {
      if (cand.boxvalIn.side) _numSides = 2;
      cand.boxvalIn.id = foldString(cand.gid);
      cand.boxvalOut.id = cand.boxvalIn.id;
    }
  }

  {
    for (auto& cand : cands.foldedBoxAreas) {
      if (cand.boxvalIn.side) _numSides = 2;
      cand.boxvalIn.id = foldString(cand.gid);
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
    for (const auto& cand : cands.foldedPoints) {
      diskAdd(cand.boxvalIn);
      diskAdd(cand.boxvalOut);
      if (_curSweepId / 2 % 1000000 == 0)
        log("@ " + std::to_string(_curSweepId / 2));
    }
    for (const auto& cand : cands.points) {
      diskAdd(cand.boxvalIn);
      diskAdd(cand.boxvalOut);
      if (_curSweepId / 2 % 1000000 == 0)
        log("@ " + std::to_string(_curSweepId / 2));
    }
    for (const auto& cand : cands.foldedSimpleLines) {
      diskAdd(cand.boxvalIn);
      diskAdd(cand.boxvalOut);
      if (_curSweepId / 2 % 1000000 == 0)
        log("@ " + std::to_string(_curSweepId / 2));
    }
    for (const auto& cand : cands.foldedBoxAreas) {
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
  // collect dist, if requested
  if (_cfg.withinDist >= 0) {
    std::map<std::string, double> subDistance;
    for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
      std::unique_lock<std::mutex> lock(_mutsDistance[t]);
      auto i = _subDistance[t].find(gidA);
      if (i != _subDistance[t].end()) {
        for (const auto& a : i->second) {
          if (subDistance.find(a.first) == subDistance.end())
            subDistance[a.first] = a.second;
          else if (subDistance[a.first] > a.second)
            subDistance[a.first] = a.second;
        }
        _subDistance[t].erase(i);
      }
    }

    for (const auto& a : subDistance) {
      writeRel(tOut, gidA, a.first, "\t" + std::to_string(a.second) + "\t");
      writeRel(tOut, a.first, gidA, "\t" + std::to_string(a.second) + "\t");

      for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
        auto j = _subDistance[t].find(a.first);
        if (j != _subDistance[t].end()) {
          auto k = j->second.find(gidA);
          if (k != j->second.end()) {
            j->second.erase(gidA);
          }
        }
      }
    }
    return;
  }

  // collect DE9IM, if requested
  if (_cfg.computeDE9IM) {
    std::map<std::string, util::geo::DE9IMatrix> subDE9IM;

    for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
      std::unique_lock<std::mutex> lock(_mutsDE9IM[t]);
      auto i = _subDE9IM[t].find(gidA);
      if (i != _subDE9IM[t].end()) {
        for (const auto& a : i->second) {
          subDE9IM[a.first] += a.second;

          for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
            auto j = _subDE9IM[t].find(a.first);
            if (j != _subDE9IM[t].end()) {
              auto k = j->second.find(gidA);
              if (k != j->second.end()) {
                j->second.erase(gidA);
              }
            }
          }
        }
        _subDE9IM[t].erase(i);
      }
    }

    for (const auto& a : subDE9IM) {
      writeRel(tOut, gidA, a.first, "\t" + a.second.toString() + "\t");
      _relStats[tOut].de9im++;
      writeRel(tOut, a.first, gidA,
               "\t" + a.second.transpose().toString() + "\t");
      _relStats[tOut].de9im++;
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

  log("Sorting events...");

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

  log("...done");
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

  _cancelled = false;

  const size_t batchSize = 100000;
  JobBatch curBatch;

  const size_t RBUF_SIZE = 100000;
  unsigned char* buf = new unsigned char[sizeof(BoxVal) * RBUF_SIZE];

  util::geo::IntervalIdx<int32_t, SweepVal> actives[2];

  _stats.resize(_cfg.numThreads + 1);
  _relStats.resize(_cfg.numThreads + 1);
  _checks.resize(_cfg.numThreads);
  _curX.resize(_cfg.numThreads);
  _subEquals.resize(_cfg.numThreads + 1);
  _subCovered.resize(_cfg.numThreads + 1);
  _subContains.resize(_cfg.numThreads + 1);
  _subDistance.resize(_cfg.numThreads + 1);
  _subDE9IM.resize(_cfg.numThreads + 1);
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
  _mutsDE9IM = std::vector<std::mutex>(_cfg.numThreads + 1);
  _atomicCurX = std::vector<std::atomic<int32_t>>(_cfg.numThreads + 1);

  size_t counts = 0, totalCheckCount = 0, jj = 0, checkPairs = 0;
  auto t = TIME();

  // fire up worker threads for geometry checking
  std::vector<std::thread> thrds(_cfg.numThreads);
  for (size_t i = 0; i < thrds.size(); i++)
    thrds[i] = std::thread(&Sweeper::processQueue, this, i);

  ssize_t len;

  try {
    while ((len = util::readAll(_file, buf, sizeof(BoxVal) * RBUF_SIZE)) != 0) {
      if (len < 0) {
        std::stringstream ss;
        ss << "Could not read from events file '" << _fname << "'\n";
        ss << strerror(errno) << std::endl;
        throw std::runtime_error(ss.str());
      }

      for (ssize_t i = 0; i < len; i += sizeof(BoxVal)) {
        auto cur = reinterpret_cast<const BoxVal*>(buf + i);

        if (_cfg.sweepCancellationCb && jj % 10000 == 0) {
          _cfg.sweepCancellationCb();
        }

        jj++;

        if (!cur->out && cur->loY == 1 && cur->upY == 0 && cur->type == POINT) {
          // special multi-IN
          _activeMultis[cur->side].insert(cur->id);
        } else if (!cur->out) {
          // IN event
          actives[cur->side].insert(
              {cur->loY, cur->upY},
              {cur->id,
               cur->type,
               cur->b45,
               cur->point,
               {cur->val, cur->point.getY() == cur->loY ? cur->upY : cur->loY},
               cur->side,
               cur->large});

          if (jj % 500000 == 0) {
            auto lon =
                webMercToLatLng<double>((1.0 * cur->val) / PREC, 0).getX();
            totalCheckCount += checkPairs;

            auto cacheSizePoint = _pointCache.size();
            auto cacheSizeArea = _areaCache.size();
            auto cacheSizeSimpleArea = _simpleAreaCache.size();
            auto cacheSizeSimpleLine = _simpleLineCache.size();
            auto cacheSizeLine = _lineCache.size();

            log(std::to_string(jj / 2) + " / " +
                std::to_string(_curSweepId / 2) + " (" +
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
                               _activeMultis[1].size()) +
                ", |C|=" +
                std::to_string(cacheSizePoint.first + cacheSizeArea.first +
                               cacheSizeSimpleArea.first +
                               cacheSizeSimpleLine.first +
                               cacheSizeLine.first) +
                " (" +
                util::readableSize(
                    cacheSizePoint.second + cacheSizeArea.second +
                    cacheSizeSimpleArea.second + cacheSizeSimpleLine.second +
                    cacheSizeLine.second) +
                ")");
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
          if (_refs.size()) curBatch.push_back({*cur, *cur, ""});

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
  } catch (...) {
    // graceful handling of an exception during sweep

    delete[] buf;

    // set the cancelled variable to true
    _cancelled = true;

    // the DONE element on the job queue to signal all threads to shut down
    _jobs.add({});

    // again wait for all workers to finish
    for (auto& thr : thrds)
      if (thr.joinable()) thr.join();

    // rethrow exception
    throw;
  }

  delete[] buf;

  if (!_cfg.noGeometryChecks && curBatch.size()) _jobs.add(std::move(curBatch));

  // the DONE element on the job queue to signal all threads to shut down
  _jobs.add({});

  // wait for all workers to finish
  for (auto& thr : thrds)
    if (thr.joinable()) thr.join();

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
  for (auto& thr : thrds)
    if (thr.joinable()) thr.join();

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

  return {std::move(spoly),
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
          0,
          {},
          {},
          0};
}

// _____________________________________________________________________________
util::geo::DE9IMatrix Sweeper::DE9IMCheck(const Area* a, const Area* b,
                                          size_t t) const {
  _stats[t].totalComps++;
  // cheap equivalence check
  if (a->box == b->box && a->area == b->area && a->geom == b->geom) {
    // equivalent!
    return util::geo::M2FFF1FFF2;
  }

  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect(a->boxIds, b->boxIds);
    _stats[t].timeBoxIdIsectAreaArea += TOOK(ts);

    // all boxes of a are fully contained in b, a is fully contained in b
    if (r.first == a->boxIds.front().first) return util::geo::M2FF1FF212;

    // no box shared, we cannot have any spatial relation
    if (r.first + r.second == 0) return util::geo::MFF2FF1212;

    if (_dontNeedFullDE9IM) {
      // at least one box is fully contained, so we intersect
      // but the number of fully and partially contained boxes is smaller
      // than the number of boxes of A, so we cannot possible be contained
      if (r.first + r.second < a->boxIds.front().first && r.first > 0) {
        // we surely overlap if the area of b is greater than the area of a
        // or if the bounding box of b is not in a
        // otherwise, we cannot be sure
        if (b->area > a->area || !util::geo::contains(b->box, a->box))
          // this is an incomplete matrix by design, specified to return true
          // for only intersects() and overlaps02()
          return util::geo::M2F2FFF2F2;
      }
    }
  }

  if (_cfg.useOBB) {
    if (!a->obb.empty() && !b->obb.empty()) {
      auto ts = TIME();
      auto r = util::geo::intersectsContainsCovers(a->obb, b->obb);
      _stats[t].timeOBBIsectAreaArea += TOOK(ts);
      if (!std::get<0>(r)) return util::geo::MFF2FF1212;
    }
  }

  if (_cfg.useInnerOuter && !a->outer.empty() && !b->outer.empty()) {
    auto ts = TIME();
    auto r = util::geo::intersectsContainsCovers(
        a->outer, a->outerBox, a->outerOuterArea, b->outer, b->outerBox,
        b->outerOuterArea);
    _stats[t].timeInnerOuterCheckAreaArea += TOOK(ts);
    _stats[t].innerOuterChecksAreaArea++;
    if (!std::get<0>(r)) return util::geo::MFF2FF1212;
  }

  if (_cfg.useInnerOuter && !a->outer.empty() && !b->inner.empty()) {
    auto ts = TIME();
    auto r = util::geo::intersectsContainsCovers(
        a->outer, a->outerBox, a->outerOuterArea, b->inner, b->innerBox,
        b->innerOuterArea);
    _stats[t].timeInnerOuterCheckAreaArea += TOOK(ts);
    _stats[t].innerOuterChecksAreaArea++;
    if (std::get<1>(r)) return util::geo::M2FF1FF212;
  }

  if (_cfg.useInnerOuter && a->outer.empty() && !b->outer.empty()) {
    auto ts = TIME();
    auto r = util::geo::intersectsContainsCovers(a->geom, a->box, a->outerArea,
                                                 b->outer, b->outerBox,
                                                 b->outerOuterArea);
    _stats[t].timeInnerOuterCheckAreaArea += TOOK(ts);
    _stats[t].innerOuterChecksAreaArea++;
    if (!std::get<0>(r)) return util::geo::MFF2FF1212;
  }

  if (_cfg.useInnerOuter && a->outer.empty() && !b->inner.empty()) {
    auto ts = TIME();
    auto r = util::geo::intersectsContainsCovers(a->geom, a->box, a->outerArea,
                                                 b->inner, b->innerBox,
                                                 b->innerOuterArea);
    _stats[t].timeInnerOuterCheckAreaArea += TOOK(ts);
    _stats[t].innerOuterChecksAreaArea++;
    if (std::get<1>(r)) return util::geo::M2FF1FF212;
  }

  auto ts = TIME();
  auto res =
      DE9IM(a->geom, a->box, a->outerArea, b->geom, b->box, b->outerArea);
  _stats[t].timeFullGeoCheckAreaArea += TOOK(ts);
  _stats[t].fullGeoChecksAreaArea++;
  return res;
}

// _____________________________________________________________________________
util::geo::DE9IMatrix Sweeper::DE9IMCheck(const Line* a, const Area* b,
                                          size_t t) const {
  _stats[t].totalComps++;
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect(a->boxIds, b->boxIds);
    _stats[t].timeBoxIdIsectAreaLine += TOOK(ts);

    // all boxes of a are fully contained in b, we intersect and we are
    // contained
    if (r.first == a->boxIds.front().first) return util::geo::M1FF0FF212;

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return util::geo::MFF1FF0212;

    // at least one box is fully contained, so we intersect
    // but the number of fully and partially contained boxes is smaller
    // than the number of boxes of A, so we cannot possible by contained, but
    // we cross
    if (_dontNeedFullDE9IM && r.first + r.second < a->boxIds.front().first &&
        r.first > 0) {
      // this is an incomplete matrix by design, specified to return true
      // for only intersects() and crosses1vs2()
      return util::geo::M1F1FFFFF2;
    }
  }

  if (_cfg.useOBB) {
    if (!a->obb.empty() && !b->obb.empty()) {
      auto ts = TIME();
      auto r = intersectsContainsCovers(a->obb, b->obb);
      _stats[t].timeOBBIsectAreaLine += TOOK(ts);
      if (!std::get<0>(r)) return util::geo::MFF1FF0212;
    }
  }

  if (_cfg.useInnerOuter && !b->outer.empty()) {
    auto ts = TIME();
    auto r = util::geo::intersectsContainsCovers(a->geom, a->box, b->outer,
                                                 b->outerBox);
    _stats[t].timeInnerOuterCheckAreaLine += TOOK(ts);
    _stats[t].innerOuterChecksAreaLine++;
    if (!std::get<0>(r)) return util::geo::MFF1FF0212;
  }

  if (_cfg.useInnerOuter && !b->inner.empty()) {
    auto ts = TIME();
    auto r = util::geo::intersectsContainsCovers(a->geom, a->box, b->inner,
                                                 b->innerBox);
    _stats[t].timeInnerOuterCheckAreaLine += TOOK(ts);
    _stats[t].innerOuterChecksAreaLine++;
    if (std::get<1>(r)) return util::geo::M1FF0FF212;
  }

  auto ts = TIME();
  auto res = DE9IM(a->geom, a->box, b->geom, b->box);
  _stats[t].timeFullGeoCheckAreaLine += TOOK(ts);
  _stats[t].fullGeoChecksAreaLine++;

  return res;
}

// _____________________________________________________________________________
util::geo::DE9IMatrix Sweeper::DE9IMCheck(const Line* a, const Line* b,
                                          size_t t) const {
  _stats[t].totalComps++;
  // cheap equivalence check
  if (a->box == b->box && a->geom == b->geom) {
    // equivalent!
    return util::geo::M10FF0FFF2;
  }

  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect(a->boxIds, b->boxIds);
    _stats[t].timeBoxIdIsectLineLine += TOOK(ts);

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return util::geo::MFF1FF0102;
  }

  if (_cfg.useOBB) {
    if (!a->obb.empty() && !b->obb.empty()) {
      auto ts = TIME();
      auto r = util::geo::intersectsContainsCovers(a->obb, b->obb);
      _stats[t].timeOBBIsectLineLine += TOOK(ts);
      if (!std::get<0>(r)) return util::geo::MFF1FF0102;
    }
  }

  auto ts = TIME();
  auto res = DE9IM(a->geom, b->geom, a->box, b->box);
  _stats[t].timeFullGeoCheckLineLine += TOOK(ts);
  _stats[t].fullGeoChecksLineLine++;

  return res;
}

// _____________________________________________________________________________
GeomCheckRes Sweeper::check(const Line* a, const Line* b, size_t t) const {
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
util::geo::DE9IMatrix Sweeper::DE9IMCheck(const LineSegment<int32_t>& a,
                                          const Area* b, size_t t) const {
  _stats[t].totalComps++;
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect({{1, 0}, {getBoxId(a.first), 0}}, b->boxIds);
    _stats[t].timeBoxIdIsectAreaLine += TOOK(ts);

    // all boxes of a are fully contained in b, we intersect and we are
    // contained
    if (r.first == 1) return util::geo::M1FF0FF212;

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return util::geo::MFF1FF0212;
  }

  if (_cfg.useOBB && !b->obb.empty()) {
    auto ts = TIME();
    auto r = intersectsContainsCovers(I32XSortedLine(a), b->obb);
    _stats[t].timeOBBIsectAreaLine += TOOK(ts);
    if (!std::get<0>(r)) return util::geo::MFF1FF0212;
  }

  if (_cfg.useInnerOuter && !b->outer.empty()) {
    auto ts = TIME();
    auto r = util::geo::intersectsContainsCovers(
        I32XSortedLine(a), getBoundingBox(a), b->outer, b->outerBox);
    _stats[t].timeInnerOuterCheckAreaLine += TOOK(ts);
    _stats[t].innerOuterChecksAreaLine++;
    if (!std::get<0>(r)) return util::geo::MFF1FF0212;
  }

  if (_cfg.useInnerOuter && !b->inner.empty()) {
    auto ts = TIME();
    auto r = util::geo::intersectsContainsCovers(
        I32XSortedLine(a), getBoundingBox(a), b->inner, b->box);
    _stats[t].timeInnerOuterCheckAreaLine += TOOK(ts);
    _stats[t].innerOuterChecksAreaLine++;
    if (std::get<1>(r)) return util::geo::M1FF0FF212;
  }

  auto ts = TIME();
  auto res = DE9IM(I32XSortedLine(a), getBoundingBox(a), b->geom, b->box);
  _stats[t].timeFullGeoCheckAreaLine += TOOK(ts);
  _stats[t].fullGeoChecksAreaLine++;
  return res;
}

// _____________________________________________________________________________
GeomCheckRes Sweeper::check(const LineSegment<int32_t>& a, const Area* b,
                            size_t t) const {
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect({{1, 0}, {getBoxId(a.first), 0}}, b->boxIds);
    _stats[t].timeBoxIdIsectAreaLine += TOOK(ts);

    // all boxes of a are fully contained in b, we intersect and we are
    // contained
    if (r.first == 1) return {1, 1, 1, 0, 0};

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return {0, 0, 0, 0, 0};
  }

  if (_cfg.useOBB && !b->obb.empty()) {
    auto ts = TIME();
    auto r = intersectsContainsCovers(I32XSortedLine(a), b->obb);
    _stats[t].timeOBBIsectAreaLine += TOOK(ts);
    if (!std::get<0>(r)) return {0, 0, 0, 0, 0};
  }

  if (_cfg.useInnerOuter && !b->outer.empty()) {
    auto ts = TIME();
    auto r = util::geo::intersectsContainsCovers(
        I32XSortedLine(a), getBoundingBox(a), b->outer, b->outerBox);
    _stats[t].timeInnerOuterCheckAreaLine += TOOK(ts);
    _stats[t].innerOuterChecksAreaLine++;
    if (!std::get<0>(r)) return {0, 0, 0, 0, 0};
  }

  if (_cfg.useInnerOuter && !b->inner.empty()) {
    auto ts = TIME();
    auto r = util::geo::intersectsContainsCovers(
        I32XSortedLine(a), getBoundingBox(a), b->inner, b->box);
    _stats[t].timeInnerOuterCheckAreaLine += TOOK(ts);
    _stats[t].innerOuterChecksAreaLine++;
    if (std::get<1>(r)) return {1, 1, 1, 0, 0};
  }

  auto ts = TIME();
  auto res = intersectsContainsCovers(I32XSortedLine(a), getBoundingBox(a),
                                      b->geom, b->box);
  _stats[t].timeFullGeoCheckAreaLine += TOOK(ts);
  _stats[t].fullGeoChecksAreaLine++;
  return res;
}

// _____________________________________________________________________________
util::geo::DE9IMatrix Sweeper::DE9IMCheck(const LineSegment<int32_t>& a,
                                          const LineSegment<int32_t>& b,
                                          size_t t) const {
  _stats[t].totalComps++;
  auto ts = TIME();

  // no need to do a full sweep for two simple lines with all the required
  // datastructures, just unroll the individual checks here

  auto r = util::geo::IntersectorLine<int32_t>::check(
      a, 32767, true, 32767, true, b, 32767, true, 32767, true);

  const bool weakIntersect = r;
  const bool strictIntersect = (r >> 0) & 1;
  const bool overlaps = (r >> 1) & 1;
  const bool aFirstInB = (r >> 6) & 1;
  const bool aSecondInB = (r >> 7) & 1;
  const bool crosses = (r >> 4) & 1;
  const bool strictIntersect2 = (r >> 5) & 1;
  const bool bFirstInA = (r >> 2) & 1;
  const bool bSecondInA = (r >> 3) & 1;

  const bool aInB = !crosses && !strictIntersect && weakIntersect;
  const bool bInA = !crosses && !strictIntersect2 && weakIntersect;

  char ii = overlaps ? '1' : (crosses ? '0' : 'F');
  char ib = ((bFirstInA && b.first != a.first && b.first != a.second) ||
             (bSecondInA && b.second != a.first && b.second != a.second))
                ? '0'
                : 'F';
  char ie = aInB ? 'F' : '1';
  char bi = ((aFirstInB && a.first != b.first && a.first != b.second) ||
             (aSecondInB && a.second != b.first && a.second != b.second))
                ? '0'
                : 'F';
  char bb = (a.first == b.first || a.second == b.first ||
             a.second == b.second || a.first == b.second)
                ? '0'
                : 'F';
  char be = !(aFirstInB && aSecondInB) ? '0' : 'F';
  char ei = bInA ? 'F' : '1';
  char eb = !(bFirstInA && bSecondInA) ? '0' : 'F';
  char ee = '2';

  _stats[t].timeFullGeoCheckLineLine += TOOK(ts);
  _stats[t].fullGeoChecksLineLine++;

  return std::string{ii, ib, ie, bi, bb, be, ei, eb, ee};
}

// _____________________________________________________________________________
GeomCheckRes Sweeper::check(const LineSegment<int32_t>& a,
                            const LineSegment<int32_t>& b, size_t t) const {
  auto ts = TIME();

  // no need to do a full sweep for two simple lines with all the required
  // datastructures, just unroll the individual checks here

  auto r = util::geo::IntersectorLine<int32_t>::check(
      a, 32767, true, 32767, true, b, 32767, true, 32767, true);

  bool weakIntersect = r;
  bool strictIntersect = (r >> 0) & 1;
  bool overlaps = (r >> 1) & 1;
  const bool bFirstInA = (r >> 2) & 1;
  const bool bSecondInA = (r >> 3) & 1;
  const bool aFirstInB = (r >> 6) & 1;
  const bool aSecondInB = (r >> 7) & 1;
  bool touches = bFirstInA || bSecondInA || aFirstInB || aSecondInB;

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
util::geo::DE9IMatrix Sweeper::DE9IMCheck(const LineSegment<int32_t>& a,
                                          const Line* b, size_t t) const {
  _stats[t].totalComps++;
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect({{1, 0}, {getBoxId(a.first), 0}}, b->boxIds);
    _stats[t].timeBoxIdIsectLineLine += TOOK(ts);

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return util::geo::MFF1FF0102;
  }

  if (_cfg.useOBB && !b->obb.empty()) {
    auto ts = TIME();
    auto r = intersectsContainsCovers(I32XSortedLine(a), b->obb);
    _stats[t].timeOBBIsectLineLine += TOOK(ts);
    if (!std::get<0>(r)) return util::geo::MFF1FF0102;
  }

  auto ts = TIME();
  auto res = DE9IM(I32XSortedLine(a), b->geom, getBoundingBox(a), b->box);
  _stats[t].timeFullGeoCheckLineLine += TOOK(ts);
  _stats[t].fullGeoChecksLineLine++;
  return res;
}

// _____________________________________________________________________________
GeomCheckRes Sweeper::check(const LineSegment<int32_t>& a, const Line* b,
                            size_t t) const {
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect({{1, 0}, {getBoxId(a.first), 0}}, b->boxIds);
    _stats[t].timeBoxIdIsectLineLine += TOOK(ts);

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return {0, 0, 0, 0, 0};
  }

  if (_cfg.useOBB && !b->obb.empty()) {
    auto ts = TIME();
    auto r = intersectsContainsCovers(I32XSortedLine(a), b->obb);
    _stats[t].timeOBBIsectLineLine += TOOK(ts);
    if (!std::get<0>(r)) return {0, 0, 0, 0, 0};
  }

  auto ts = TIME();
  auto res =
      intersectsCovers(I32XSortedLine(a), b->geom, getBoundingBox(a), b->box);
  _stats[t].timeFullGeoCheckLineLine += TOOK(ts);
  _stats[t].fullGeoChecksLineLine++;
  return res;
}

// _____________________________________________________________________________
GeomCheckRes Sweeper::check(const Line* a, const LineSegment<int32_t>& b,
                            size_t t) const {
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect(a->boxIds, {{1, 0}, {getBoxId(b.first), 0}});
    _stats[t].timeBoxIdIsectLineLine += TOOK(ts);

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return {0, 0, 0, 0, 0};
  }

  if (_cfg.useOBB && !a->obb.empty()) {
    auto ts = TIME();
    auto r = intersectsContainsCovers(I32XSortedLine(b), a->obb);
    _stats[t].timeOBBIsectLineLine += TOOK(ts);
    if (!std::get<0>(r)) return {0, 0, 0, 0, 0};
  }

  auto ts = TIME();
  auto res =
      intersectsCovers(a->geom, I32XSortedLine(b), a->box, getBoundingBox(b));
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
util::geo::DE9IMatrix Sweeper::DE9IMCheck(const I32Point& a, const Line* b,
                                          size_t t) const {
  _stats[t].totalComps++;
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect({{1, 0}, {getBoxId(a), 0}}, b->boxIds);
    _stats[t].timeBoxIdIsectLinePoint += TOOK(ts);

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return util::geo::MFF0FFF102;
  }

  auto ts = TIME();
  auto res = util::geo::DE9IM(a, b->geom);
  _stats[t].timeFullGeoCheckLinePoint += TOOK(ts);
  _stats[t].fullGeoChecksLinePoint++;

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
  }

  auto ts = TIME();
  auto res = util::geo::intersectsContains(a, b->geom);
  _stats[t].timeFullGeoCheckLinePoint += TOOK(ts);
  _stats[t].fullGeoChecksLinePoint++;

  return res;
}

// ____________________________________________________________________________
void Sweeper::writeRel(size_t t, const std::string& a, const std::string& b,
                       const std::string& pred) {
  if (!_cfg.writeRelCb) return;

  auto ts = TIME();

  if (_numSides == 2 && (a[0] != 'A' || a[0] == b[0])) return;

  _cfg.writeRelCb(t, a.c_str() + 1, a.size() - 1, b.c_str() + 1, b.size() - 1,
                  pred.c_str(), pred.size());

  _stats[t].timeWrite += TOOK(ts);
}

// ____________________________________________________________________________
void Sweeper::writeDE9IM(size_t t, const std::string& a, size_t aSub,
                         const std::string& b, size_t bSub,
                         util::geo::DE9IMatrix de9im) {
  if (a != b) {
    if (aSub > 0 && bSub == 0 && de9im.covers()) {
      // no need to lock and track the multigeometry here, we can directly
      // write that a contains b
      writeRel(t, a, b, "\t" + de9im.toString() + "\t");
      _relStats[t].de9im++;
      writeRel(t, b, a, "\t" + de9im.transpose().toString() + "\t");
      _relStats[t].de9im++;
    } else if (bSub > 0 && aSub == 0 && de9im.transpose().covers()) {
      // no need to lock and track the multigeometry here, we can directly
      // write that b contains a
      writeRel(t, a, b, "\t" + de9im.toString() + "\t");
      _relStats[t].de9im++;
      writeRel(t, b, a, "\t" + de9im.transpose().toString() + "\t");
      _relStats[t].de9im++;
    } else if ((bSub > 0 || aSub > 0)) {
      std::unique_lock<std::mutex> lock(_mutsDE9IM[t]);
      if (bSub > 0) {
        if (_subDE9IM[t][b].find(a) == _subDE9IM[t][b].end()) {
          _subDE9IM[t][b][a] = de9im.transpose();
        } else {
          _subDE9IM[t][b][a] += de9im.transpose();
        }
      }
      if (aSub > 0) {
        if (_subDE9IM[t][a].find(b) == _subDE9IM[t][a].end()) {
          _subDE9IM[t][a][b] = de9im;
        } else {
          _subDE9IM[t][a][b] += de9im;
        }
      }
    } else {
      _relStats[t].de9im++;
      _relStats[t].de9im++;
      writeRel(t, a, b, "\t" + de9im.toString() + "\t");
      writeRel(t, b, a, "\t" + de9im.transpose().toString() + "\t");
    }
  }

  // TODO: handle references!
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
      const auto& dStr = util::formatFloat(dist, 4);
      writeRel(t, a, b, "\t" + dStr + "\t");
      writeRel(t, b, a, "\t" + dStr + "\t");
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
void Sweeper::selfCheck(const JobVal cur, size_t t) {
  if (cur.type == FOLDED_POINT) {
    auto ts = TIME();
    auto gid = unfoldString(cur.id);
    _stats[t].timeGeoCacheRetrievalPoint += TOOK(ts);

    writeIntersect(t, gid, gid);
    writeEquals(t, gid, 0, gid, 0);
    writeCovers(t, gid, gid, 0);
  } else if (cur.type == POINT) {
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
  } else if (isSimpleLine(cur.type)) {
    auto a = getSimpleLine(cur, cur.large ? -1 : t);

    writeIntersect(t, a->id, a->id);
    writeEquals(t, a->id, 0, a->id, 0);
    writeCovers(t, a->id, a->id, 0);
  } else if (isArea(cur.type)) {
    auto a = getArea(cur, cur.large ? -1 : t);

    writeIntersect(t, a->id, a->id);
    writeEquals(t, a->id, a->subId, a->id, a->subId);
    writeCovers(t, a->id, a->id, a->subId);
  }
}

// ____________________________________________________________________________
void Sweeper::doDE9IMCheck(const JobVal cur, const JobVal sv, size_t t) {
  _checks[t]++;
  _curX[t] = cur.val;

  // every 10000 checks, update our position
  if (_checks[t] % 10000 == 0) _atomicCurX[t] = _curX[t];

  if (isPoint(cur.type) && isPoint(sv.type)) {
    auto p1 = cur.point;
    auto p2 = sv.point;
    _stats[t].totalComps++;

    auto de9im = DE9IM(p1, p2);

    if (!de9im.disjoint()) {
      auto a = getPoint(cur.id, cur.type, cur.large ? -1 : t);
      auto b = getPoint(sv.id, sv.type, sv.large ? -1 : t);

      if (a->id == b->id) return;  // no self-checks in multigeometries

      writeDE9IM(t, a->id, a->subId, b->id, b->subId, de9im);
    }
  } else if (isPoint(cur.type) && isArea(sv.type)) {
    auto p = cur.point;

    std::shared_ptr<Area> a = getArea(sv, sv.large ? -1 : t);

    auto de9im = DE9IMCheck(p, a.get(), t);

    if (!de9im.disjoint()) {
      auto b = getPoint(cur.id, cur.type, cur.large ? -1 : t);

      if (a->id == b->id) return;  // no self-checks in multigeometries

      writeDE9IM(t, b->id, b->subId, a->id, a->subId, de9im);
    }
  } else if (isArea(cur.type) && isPoint(sv.type)) {
    auto p = sv.point;
    std::shared_ptr<Area> a = getArea(cur, cur.large ? -1 : t);

    auto de9im = DE9IMCheck(p, a.get(), t).transpose();

    if (!de9im.disjoint()) {
      auto b = getPoint(sv.id, sv.type, sv.large ? -1 : t);

      if (a->id == b->id) return;  // no self-checks in multigeometries

      writeDE9IM(t, a->id, a->subId, b->id, b->subId, de9im);
    }
  } else if (isLine(cur.type) && isPoint(sv.type)) {
    auto p = sv.point;

    if (isSimpleLine(cur.type)) {
      auto de9im = DE9IMCheck(p, {cur.point, cur.point2}, t);

      if (!de9im.disjoint()) {
        auto a = getPoint(sv.id, sv.type, sv.large ? -1 : t);

        auto ts = TIME();
        auto b = getSimpleLine(cur, cur.large ? -1 : t);
        _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);

        if (a->id == b->id) return;  // no self-checks in multigeometries

        writeDE9IM(t, a->id, a->subId, b->id, 0, de9im);
      }
    } else {
      auto ts = TIME();
      auto b = _lineCache.get(cur.id, cur.large ? -1 : t);
      _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);
      auto de9im = DE9IMCheck(p, b.get(), t);

      if (!de9im.disjoint()) {
        auto a = getPoint(sv.id, sv.type, sv.large ? -1 : t);

        if (a->id == b->id) return;  // no self-checks in multigeometries

        writeDE9IM(t, a->id, a->subId, b->id, b->subId, de9im);
      }
    }
  } else if (isLine(sv.type) && isPoint(cur.type)) {
    auto p = cur.point;

    if (isSimpleLine(sv.type)) {
      auto de9im = DE9IMCheck(p, {sv.point, sv.point2}, t);

      if (!de9im.disjoint()) {
        auto a = getPoint(cur.id, cur.type, cur.large ? -1 : t);

        auto ts = TIME();
        auto b = getSimpleLine(sv, sv.large ? -1 : t);
        _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);

        if (a->id == b->id) return;  // no self-checks in multigeometries

        writeDE9IM(t, a->id, a->subId, b->id, 0, de9im);
      }
    } else {
      auto ts = TIME();
      auto b = _lineCache.get(sv.id, sv.large ? -1 : t);
      _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);
      auto de9im = DE9IMCheck(p, b.get(), t);

      if (!de9im.disjoint()) {
        auto a = getPoint(cur.id, cur.type, cur.large ? -1 : t);

        if (a->id == b->id) return;  // no self-checks in multigeometries

        writeDE9IM(t, a->id, a->subId, b->id, b->subId, de9im);
      }
    }
  } else if (sv.type == LINE && cur.type == LINE) {
    auto ts = TIME();
    auto a = _lineCache.get(sv.id, sv.large ? -1 : t);
    auto b = _lineCache.get(cur.id, cur.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    auto de9im = DE9IMCheck(a.get(), b.get(), t);

    if (!de9im.disjoint()) {
      writeDE9IM(t, a->id, a->subId, b->id, b->subId, de9im);
    }
  } else if (isSimpleLine(sv.type) && isSimpleLine(cur.type)) {
    auto de9im = DE9IMCheck({sv.point, sv.point2}, {cur.point, cur.point2}, t);

    if (!de9im.disjoint()) {
      auto ts = TIME();
      auto a = getSimpleLine(sv, sv.large ? -1 : t);
      auto b = getSimpleLine(cur, cur.large ? -1 : t);
      _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);
      if (a->id == b->id) return;  // no self-checks in multigeometries
      writeDE9IM(t, a->id, 0, b->id, 0, de9im);
    }
  } else if (isSimpleLine(sv.type) && cur.type == LINE) {
    auto ts = TIME();
    auto a = getSimpleLine(sv, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);
    ts = TIME();
    auto b = _lineCache.get(cur.id, cur.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    auto de9im = DE9IMCheck({sv.point, sv.point2}, b.get(), t);

    if (!de9im.disjoint()) {
      writeDE9IM(t, a->id, 0, b->id, b->subId, de9im);
    }
  } else if (sv.type == LINE && isSimpleLine(cur.type)) {
    auto ts = TIME();
    auto a = _lineCache.get(sv.id, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);
    ts = TIME();
    auto b = getSimpleLine(cur, cur.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    auto de9im = DE9IMCheck({cur.point, cur.point2}, a.get(), t);

    if (!de9im.disjoint()) {
      writeDE9IM(t, a->id, a->subId, b->id, 0, de9im.transpose());
    }
  } else if (isArea(sv.type) && isArea(cur.type)) {
    std::shared_ptr<Area> a = getArea(cur, cur.large ? -1 : t);
    std::shared_ptr<Area> b = getArea(sv, sv.large ? -1 : t);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    auto de9im = DE9IMCheck(a.get(), b.get(), t);

    if (!de9im.disjoint()) {
      writeDE9IM(t, a->id, a->subId, b->id, b->subId, de9im);
    }
  } else if (sv.type == LINE && isArea(cur.type)) {
    auto ts = TIME();
    auto a = _lineCache.get(sv.id, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    std::shared_ptr<Area> b = getArea(cur, cur.large ? -1 : t);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    auto de9im = DE9IMCheck(a.get(), b.get(), t);

    if (!de9im.disjoint()) {
      writeDE9IM(t, a->id, a->subId, b->id, b->subId, de9im);
    }
  } else if (isArea(sv.type) && cur.type == LINE) {
    std::shared_ptr<Area> a = getArea(sv, sv.large ? -1 : t);

    auto ts = TIME();
    auto b = _lineCache.get(cur.id, cur.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    auto de9im = DE9IMCheck(b.get(), a.get(), t);

    if (!de9im.disjoint()) {
      writeDE9IM(t, a->id, a->subId, b->id, b->subId, de9im.transpose());
    }
  } else if (isSimpleLine(sv.type) && isArea(cur.type)) {
    auto ts = TIME();
    auto a = getSimpleLine(sv, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);

    std::shared_ptr<Area> b = getArea(cur, cur.large ? -1 : t);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    auto de9im = DE9IMCheck({sv.point, sv.point2}, b.get(), t);

    if (!de9im.disjoint()) {
      writeDE9IM(t, a->id, 0, b->id, b->subId, de9im);
    }
  } else if (isArea(sv.type) && isSimpleLine(cur.type)) {
    std::shared_ptr<Area> a = getArea(sv, sv.large ? -1 : t);

    auto ts = TIME();
    auto b = getSimpleLine(cur, cur.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    auto de9im = DE9IMCheck({cur.point, cur.point2}, a.get(), t);

    if (!de9im.disjoint()) {
      writeDE9IM(t, a->id, a->subId, b->id, 0, de9im.transpose());
    }
  }
}

// ____________________________________________________________________________
void Sweeper::doDistCheck(const JobVal cur, const JobVal sv, size_t t) {
  _checks[t]++;
  _curX[t] = cur.val;

  // every 10000 checks, update our position
  if (_checks[t] % 10000 == 0) _atomicCurX[t] = _curX[t];

  if (isPoint(cur.type) && isPoint(sv.type)) {
    auto p1 = cur.point;
    auto p2 = sv.point;

    auto dist = meterDist(p1, p2);

    if (dist <= _cfg.withinDist) {
      auto a = getPoint(cur.id, cur.type, cur.large ? -1 : t);
      auto b = getPoint(sv.id, sv.type, sv.large ? -1 : t);

      writeDist(t, a->id, a->subId, b->id, b->subId, dist);
    }
  } else if (isPoint(cur.type) && isArea(sv.type)) {
    auto p = cur.point;

    std::shared_ptr<Area> a = getArea(sv, sv.large ? -1 : t);

    double dist = distCheck(p, a.get(), t);

    if (dist <= _cfg.withinDist) {
      auto b = getPoint(cur.id, cur.type, cur.large ? -1 : t);
      writeDist(t, a->id, a->subId, b->id, b->subId, dist);
    }
  } else if (isArea(cur.type) && isPoint(sv.type)) {
    auto p = sv.point;

    std::shared_ptr<Area> a = getArea(cur, cur.large ? -1 : t);

    double dist = distCheck(p, a.get(), t);

    if (dist <= _cfg.withinDist) {
      auto b = getPoint(sv.id, sv.type, sv.large ? -1 : t);
      writeDist(t, a->id, a->subId, b->id, b->subId, dist);
    }
  } else if (isLine(cur.type) && isPoint(sv.type)) {
    auto p = sv.point;

    double dist = std::numeric_limits<double>::max();

    if (isSimpleLine(cur.type)) {
      dist = distCheck(p, {cur.point, cur.point2}, t);

      if (dist <= _cfg.withinDist) {
        auto a = getPoint(sv.id, sv.type, sv.large ? -1 : t);
        auto b = getSimpleLine(cur, cur.large ? -1 : t);
        writeDist(t, a->id, a->subId, b->id, 0, dist);
      }
    } else {
      auto b = _lineCache.get(cur.id, cur.large ? -1 : t);
      dist = distCheck(p, b.get(), t);

      if (dist <= _cfg.withinDist) {
        auto a = getPoint(sv.id, sv.type, sv.large ? -1 : t);
        writeDist(t, a->id, a->subId, b->id, b->subId, dist);
      }
    }
  } else if (isLine(sv.type) && isPoint(cur.type)) {
    auto p = cur.point;

    double dist = std::numeric_limits<double>::max();

    if (isSimpleLine(sv.type)) {
      dist = distCheck(p, {sv.point, sv.point2}, t);

      if (dist <= _cfg.withinDist) {
        auto a = getPoint(cur.id, cur.type, cur.large ? -1 : t);
        auto b = getSimpleLine(sv, sv.large ? -1 : t);
        writeDist(t, a->id, a->subId, b->id, 0, dist);
      }
    } else {
      auto b = _lineCache.get(sv.id, sv.large ? -1 : t);
      dist = distCheck(p, b.get(), t);

      if (dist <= _cfg.withinDist) {
        auto a = getPoint(cur.id, cur.type, cur.large ? -1 : t);
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
  } else if (isSimpleLine(sv.type) && isSimpleLine(cur.type)) {
    auto dist = distCheck({sv.point, sv.point2}, {cur.point, cur.point2}, t);

    if (dist <= _cfg.withinDist) {
      auto a = getSimpleLine(sv, sv.large ? -1 : t);
      auto b = getSimpleLine(cur, cur.large ? -1 : t);
      writeDist(t, a->id, 0, b->id, 0, dist);
    }
  } else if (isSimpleLine(sv.type) && cur.type == LINE) {
    auto a = getSimpleLine(sv, sv.large ? -1 : t);
    auto b = _lineCache.get(cur.id, cur.large ? -1 : t);
    auto dist = distCheck({sv.point, sv.point2}, b.get(), t);

    if (dist <= _cfg.withinDist) {
      writeDist(t, a->id, 0, b->id, b->subId, dist);
    }
  } else if (sv.type == LINE && isSimpleLine(cur.type)) {
    auto a = _lineCache.get(sv.id, sv.large ? -1 : t);
    auto dist = distCheck({cur.point, cur.point2}, a.get(), t);

    if (dist <= _cfg.withinDist) {
      auto b = getSimpleLine(cur, cur.large ? -1 : t);
      writeDist(t, a->id, a->subId, b->id, 0, dist);
    }
  } else if (isArea(sv.type) && isArea(cur.type)) {
    std::shared_ptr<Area> a = getArea(sv, sv.large ? -1 : t);
    std::shared_ptr<Area> b = getArea(cur, cur.large ? -1 : t);

    auto dist = distCheck(a.get(), b.get(), t);

    if (dist <= _cfg.withinDist) {
      writeDist(t, a->id, a->subId, b->id, b->subId, dist);
    }
  } else if (sv.type == LINE && isArea(cur.type)) {
    auto a = _lineCache.get(sv.id, sv.large ? -1 : t);

    std::shared_ptr<Area> b = getArea(cur, cur.large ? -1 : t);

    auto dist = distCheck(a.get(), b.get(), t);

    if (dist <= _cfg.withinDist) {
      writeDist(t, a->id, a->subId, b->id, b->subId, dist);
    }
  } else if (isArea(sv.type) && cur.type == LINE) {
    std::shared_ptr<Area> a = getArea(sv, sv.large ? -1 : t);

    auto b = _lineCache.get(cur.id, cur.large ? -1 : t);
    auto dist = distCheck(b.get(), a.get(), t);

    if (dist <= _cfg.withinDist) {
      writeDist(t, a->id, a->subId, b->id, b->subId, dist);
    }
  } else if (isSimpleLine(sv.type) && isArea(cur.type)) {
    std::shared_ptr<Area> b = getArea(cur, cur.large ? -1 : t);

    auto dist = distCheck({sv.point, sv.point2}, b.get(), t);

    if (dist <= _cfg.withinDist) {
      auto a = getSimpleLine(sv, sv.large ? -1 : t);
      writeDist(t, a->id, 0, b->id, b->subId, dist);
    }
  } else if (isArea(sv.type) && isSimpleLine(cur.type)) {
    std::shared_ptr<Area> a = getArea(sv, sv.large ? -1 : t);

    auto dist = distCheck({cur.point, cur.point2}, a.get(), t);

    if (dist <= _cfg.withinDist) {
      auto b = getSimpleLine(cur, cur.large ? -1 : t);
      writeDist(t, a->id, a->subId, b->id, 0, dist);
    }
  }
}

// ____________________________________________________________________________
void Sweeper::doCheck(const JobVal cur, const JobVal sv, size_t t) {
  _checks[t]++;
  _curX[t] = cur.val;

  if (cur.type == sv.type && cur.id == sv.id) return selfCheck(cur, t);

  // every 10000 checks, update our position
  if (_checks[t] % 10000 == 0) _atomicCurX[t] = _curX[t];

  if (isArea(cur.type) && isArea(sv.type)) {
    std::shared_ptr<Area> a = getArea(cur, cur.large ? -1 : t);
    std::shared_ptr<Area> b = getArea(sv, sv.large ? -1 : t);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    _stats[t].areaCmps++;
    _stats[t].areaSizeSum += std::max(a->area, b->area);

    _stats[t].anchorSum += std::max(a->geom.size() / 2, b->geom.size() / 2);

    _stats[t].totalComps++;
    auto totTime = TIME();

    auto res = DE9IMCheck(a.get(), b.get(), t);

    _stats[t].timeHisto(std::max(a->geom.getOuter().rawRing().size(),
                                 b->geom.getOuter().rawRing().size()),
                        TOOK(totTime));

    // intersects
    if (res.intersects()) {
      writeIntersect(t, a->id, b->id);
    }

    // contained
    if (res.within()) {
      writeContains(t, b->id, a->id, a->subId);
    }

    // covered
    if (res.coveredBy()) {
      writeCovers(t, b->id, a->id, a->subId);

      if (fabs(a->area - b->area) < util::geo::EPSILON) {
        // both areas were equivalent
        writeEquals(t, a->id, a->subId, b->id, b->subId);

        // covers in other direction
        writeCovers(t, a->id, b->id, b->subId);

        // contains in other direction
        writeContains(t, a->id, b->id, b->subId);
      }
    }

    // touches
    if (res.touches()) {
      writeTouches(t, a->id, a->subId, b->id, b->subId);
    } else if (res.intersects()) {
      // if a is not a multi-geom, and is completey covered, we wont
      // be finding a touch as we assume non-self-intersecting geoms
      if (_refs.count(a->id) || !(a->subId == 0 && res.coveredBy())) {
        writeNotTouches(t, a->id, a->subId, b->id, b->subId);
      }
    }

    // overlaps
    if (res.overlaps02()) {
      _relStats[t].overlaps++;
      writeRel(t, a->id, b->id, _cfg.sepOverlaps);
      _relStats[t].overlaps++;
      writeRel(t, b->id, a->id, _cfg.sepOverlaps);
    }
  } else if (cur.type == LINE && isArea(sv.type)) {
    std::shared_ptr<Area> b = getArea(sv, sv.large ? -1 : t);

    auto ts = TIME();
    auto a = _lineCache.get(cur.id, cur.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    _stats[t].areaCmps++;
    _stats[t].areaSizeSum += b->area;

    _stats[t].lineCmps++;
    _stats[t].lineLenSum += a->length;

    _stats[t].anchorSum += std::max(a->geom.size() / 2, b->geom.size() / 2);

    _stats[t].totalComps++;
    auto totTime = TIME();

    auto res = DE9IMCheck(a.get(), b.get(), t);

    _stats[t].timeHisto(
        std::max(a->geom.rawLine().size(), b->geom.getOuter().rawRing().size()),
        TOOK(totTime));

    // intersects
    if (res.intersects()) {
      writeIntersect(t, a->id, b->id);
    }

    // contains
    if (res.within()) {
      writeContains(t, b->id, a->id, a->subId);
    }

    // covers
    if (res.coveredBy()) {
      writeCovers(t, b->id, a->id, a->subId);
    }

    // touches
    if (res.touches()) {
      writeTouches(t, a->id, a->subId, b->id, b->subId);
    } else if (res.intersects()) {
      // if a is not a multi-geom, and is completey covered, we wont
      // be finding a touch as we assume non-self-intersecting geoms
      if (_refs.count(a->id) || !(a->subId == 0 && res.coveredBy())) {
        writeNotTouches(t, a->id, a->subId, b->id, b->subId);
      }
    }

    // crosses
    if (res.crosses1vs2()) {
      _relStats[t].crosses++;
      writeRel(t, a->id, b->id, _cfg.sepCrosses);
    }
  } else if (isSimpleLine(cur.type) && isArea(sv.type)) {
    std::shared_ptr<Area> b = getArea(sv, sv.large ? -1 : t);

    auto ts = TIME();
    auto a = getSimpleLine(cur, cur.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    _stats[t].areaCmps++;
    _stats[t].areaSizeSum += b->area;

    _stats[t].lineCmps++;
    _stats[t].lineLenSum += util::geo::dist(cur.point, cur.point2);

    _stats[t].anchorSum += std::max((size_t)2, b->geom.size() / 2);

    _stats[t].totalComps++;
    auto totTime = TIME();

    auto res = check({cur.point, cur.point2}, b.get(), t);

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
  } else if (isArea(cur.type) && sv.type == LINE) {
    std::shared_ptr<Area> a = getArea(cur, cur.large ? -1 : t);

    auto ts = TIME();
    auto b = _lineCache.get(sv.id, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    _stats[t].areaCmps++;
    _stats[t].areaSizeSum += a->area;

    _stats[t].lineCmps++;
    _stats[t].lineLenSum += b->length;

    _stats[t].anchorSum += std::max(a->geom.size() / 2, b->geom.size() / 2);

    _stats[t].totalComps++;
    auto totTime = TIME();

    auto res = DE9IMCheck(b.get(), a.get(), t);

    _stats[t].timeHisto(
        std::max(a->geom.getOuter().rawRing().size(), b->geom.rawLine().size()),
        TOOK(totTime));

    // intersects
    if (res.intersects()) {
      writeIntersect(t, a->id, b->id);
    }

    // contains
    if (res.within()) {
      writeContains(t, a->id, b->id, b->subId);
    }

    // covers
    if (res.coveredBy()) {
      writeCovers(t, a->id, b->id, b->subId);
    }

    // touches
    if (res.touches()) {
      writeTouches(t, a->id, a->subId, b->id, b->subId);
    } else if (res.intersects()) {
      // if b is not a multi-geom, and is completey covered, we wont
      // be finding a touch as we assume non-self-intersecting geoms
      if (_refs.count(a->id) || !(b->subId == 0 && res.coveredBy())) {
        writeNotTouches(t, a->id, a->subId, b->id, b->subId);
      }
    }

    // crosses
    if (res.crosses1vs2()) {
      _relStats[t].crosses++;
      writeRel(t, b->id, a->id, _cfg.sepCrosses);
    }
  } else if (isArea(cur.type) && isSimpleLine(sv.type)) {
    std::shared_ptr<Area> a = getArea(cur, cur.large ? -1 : t);

    auto ts = TIME();
    auto b = getSimpleLine(sv, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);

    _stats[t].areaCmps++;
    _stats[t].areaSizeSum += a->area;

    _stats[t].lineCmps++;
    _stats[t].lineLenSum += util::geo::dist(sv.point, sv.point2);

    _stats[t].anchorSum += std::max(a->geom.size() / 2, (size_t)2);

    _stats[t].totalComps++;
    auto totTime = TIME();

    auto res = check({sv.point, sv.point2}, a.get(), t);

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
  } else if (cur.type == LINE && isSimpleLine(sv.type)) {
    auto ts = TIME();
    auto a = _lineCache.get(cur.id, cur.large ? -1 : t);
    auto b = getSimpleLine(sv, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);

    _stats[t].lineCmps++;
    _stats[t].lineLenSum +=
        std::max(a->length, util::geo::dist(sv.point, sv.point2));

    _stats[t].anchorSum += std::max(a->geom.size() / 2, (size_t)2);

    _stats[t].totalComps++;
    auto totTime = TIME();

    auto res = check(a.get(), {sv.point, sv.point2}, t);

    _stats[t].timeHisto(a->geom.rawLine().size(), TOOK(totTime));

    // intersects
    if (std::get<0>(res)) {
      writeIntersect(t, a->id, b->id);
    }

    // covers
    if (std::get<1>(res)) {
      writeNotCrosses(t, a->id, a->subId, b->id, 0);

      writeCovers(t, b->id, a->id, a->subId);

      if (fabs(a->length - util::geo::len(LineSegment<int32_t>(
                               sv.point, sv.point2))) < util::geo::EPSILON) {
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
  } else if (isSimpleLine(cur.type) && sv.type == LINE) {
    auto ts = TIME();
    auto a = getSimpleLine(cur, cur.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);
    ts = TIME();
    auto b = _lineCache.get(sv.id, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    _stats[t].lineCmps++;
    _stats[t].lineLenSum +=
        std::max(b->length, util::geo::dist(cur.point, cur.point2));

    _stats[t].anchorSum += std::max(b->geom.size() / 2, (size_t)2);

    _stats[t].totalComps++;
    auto totTime = TIME();

    auto res = check({cur.point, cur.point2}, b.get(), t);

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

      if (fabs(util::geo::len(LineSegment<int32_t>(cur.point, cur.point2)) -
               b->length) < util::geo::EPSILON) {
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
  } else if (isSimpleLine(cur.type) && isSimpleLine(sv.type)) {
    auto ts = TIME();
    auto a = getSimpleLine(cur, cur.large ? -1 : t);
    auto b = getSimpleLine(sv, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);

    _stats[t].lineCmps++;
    _stats[t].lineLenSum += std::max(util::geo::dist(cur.point, cur.point2),
                                     util::geo::dist(sv.point, sv.point2));

    _stats[t].anchorSum += 2;

    _stats[t].totalComps++;
    auto totTime = TIME();

    auto res = check({cur.point, cur.point2}, {sv.point, sv.point2}, t);

    _stats[t].timeHisto(2, TOOK(totTime));

    if (std::get<0>(res)) {
      writeIntersect(t, a->id, b->id);
    }

    if (std::get<1>(res)) {
      writeCovers(t, b->id, a->id, 0);

      if (fabs(util::geo::len(LineSegment<int32_t>(cur.point, cur.point2)) -
               util::geo::len(LineSegment<int32_t>(sv.point, sv.point2))) <
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
  } else if (isPoint(cur.type) && isPoint(sv.type)) {
    // point/point: trivial intersect & cover & contains

    auto a = getPoint(cur.id, cur.type, cur.large ? -1 : t);
    auto b = getPoint(sv.id, sv.type, sv.large ? -1 : t);

    _stats[t].anchorSum += 1;

    if (a->id == b->id)
      return;  // no self-checks in multigeometries
               //
    _stats[t].totalComps++;

    writeIntersect(t, a->id, b->id);
    writeEquals(t, a->id, a->subId, b->id, b->subId);

    writeCovers(t, b->id, a->id, a->subId);
    writeContains(t, b->id, a->id, a->subId);

    writeCovers(t, a->id, b->id, b->subId);
    writeContains(t, a->id, b->id, b->subId);
  } else if (isPoint(cur.type) && isSimpleLine(sv.type)) {
    auto p = cur.point;

    _stats[t].lineCmps++;
    _stats[t].lineLenSum += util::geo::dist(sv.point, sv.point2);

    _stats[t].anchorSum += 2;

    if (util::geo::contains(p, LineSegment<int32_t>(sv.point, sv.point2))) {
      auto a = getPoint(cur.id, cur.type, cur.large ? -1 : t);
      auto ts = TIME();
      auto b = getSimpleLine(sv, sv.large ? -1 : t);
      _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);
      writeIntersect(t, a->id, b->id);

      writeCovers(t, b->id, a->id, a->subId);

      if (p != sv.point && p != sv.point2) {
        writeContains(t, b->id, a->id, a->subId);

        writeNotTouches(t, a->id, a->subId, b->id, 0);
      } else {
        writeTouches(t, a->id, a->subId, b->id, 0);
      }
    }
  } else if (isPoint(cur.type) && sv.type == LINE) {
    auto ts = TIME();
    auto a = cur.point;
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
      auto a = getPoint(cur.id, cur.type, cur.large ? -1 : t);

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
  } else if (isPoint(cur.type) && isArea(sv.type)) {
    std::shared_ptr<Area> b = getArea(sv, sv.large ? -1 : t);
    auto a = cur.point;

    _stats[t].totalComps++;
    auto totTime = TIME();

    auto res = check(a, b.get(), t);

    _stats[t].timeHisto(b->geom.getOuter().rawRing().size(), TOOK(totTime));

    if (res.second) {
      auto a = getPoint(cur.id, cur.type, cur.large ? -1 : t);

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
void Sweeper::processQueue(size_t t) {
  try {
    JobBatch batch;
    while ((batch = _jobs.get()).size()) {
      for (const auto& job : batch) {
        if (_cancelled) break;

        if (job.multiOut.empty()) {
          if (_cfg.computeDE9IM) {
            doDE9IMCheck(job.boxVal, job.sweepVal, t);
          } else if (_cfg.withinDist >= 0) {
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
    // check if diagonal boxes intersect, if not, ignore this pair
    if (_cfg.useDiagBox && !util::geo::intersects(p.v.b45, cur->b45)) continue;

    JobVal a(*cur);
    JobVal b(p.v);

    // for simple lines, already check if the lines intersect, if not,
    // ignore
    if (isSimpleLine(a.type) && isSimpleLine(b.type) &&
        !util::geo::IntersectorLine<int32_t>::check(
            LineSegment<int32_t>(a.point, a.point2), 32767, true, 32767, true,
            LineSegment<int32_t>(b.point, b.point2), 32767, true, 32767, true))
      continue;

    batch->push_back({a, b, ""});
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
      std::unique_lock<std::mutex> lock(_mutsCovers[t]);
      _subCovered[t][b][a].insert(bSub);
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
util::geo::DE9IMatrix Sweeper::DE9IMCheck(const I32Point& a, const Area* b,
                                          size_t t) const {
  _stats[t].totalComps++;
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect({{1, 0}, {getBoxId(a), 0}}, b->boxIds);
    _stats[t].timeBoxIdIsectAreaPoint += TOOK(ts);

    // all boxes of a are fully contained in b, we are contained
    if (r.first) return util::geo::M0FFFFF212;

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return util::geo::MFF0FFF212;
  }

  if (_cfg.useOBB && b->obb.getOuter().rawRing().size()) {
    auto ts = TIME();
    auto r = containsCovers(a, b->obb);
    _stats[t].timeOBBIsectAreaPoint += TOOK(ts);
    if (!std::get<1>(r)) return util::geo::MFF0FFF212;
  }

  if (_cfg.useInnerOuter && !b->outer.empty()) {
    auto ts = TIME();
    auto r = containsCovers(a, b->outer);
    _stats[t].timeInnerOuterCheckAreaPoint += TOOK(ts);
    _stats[t].innerOuterChecksAreaPoint++;
    if (!std::get<1>(r)) return util::geo::MFF0FFF212;
  }

  if (_cfg.useInnerOuter && !b->inner.empty()) {
    auto ts = TIME();
    auto r = containsCovers(a, b->inner);
    _stats[t].timeInnerOuterCheckAreaPoint += TOOK(ts);
    _stats[t].innerOuterChecksAreaPoint++;
    if (std::get<1>(r)) return util::geo::M0FFFFF212;
  }

  auto ts = TIME();
  auto res = util::geo::DE9IM(a, b->geom);
  _stats[t].timeFullGeoCheckAreaPoint += TOOK(ts);
  _stats[t].fullGeoChecksAreaPoint++;

  return res;
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
util::geo::DE9IMatrix Sweeper::DE9IMCheck(const I32Point& a,
                                          const LineSegment<int32_t>& b,
                                          size_t t) const {
  _stats[t].totalComps++;
  auto ts = TIME();
  auto res = util::geo::DE9IM(a, util::geo::XSortedLine<int32_t>(b));
  _stats[t].timeFullGeoCheckLinePoint += TOOK(ts);
  _stats[t].fullGeoChecksLinePoint++;

  return res;
}

// _____________________________________________________________________________
double Sweeper::distCheck(const I32Point& a, const LineSegment<int32_t>& b,
                          size_t t) const {
  auto ts = TIME();

  auto p2 = projectOn(b.first, a, b.second);

  auto dist = Sweeper::meterDist(a, p2);

  _stats[t].timeFullGeoCheckLinePoint += TOOK(ts);
  _stats[t].fullGeoChecksLinePoint++;

  return dist;
}

// _____________________________________________________________________________
double Sweeper::distCheck(const LineSegment<int32_t>& a,
                          const LineSegment<int32_t>& b, size_t t) const {
  auto ts = TIME();

  auto dist = util::geo::dist<int32_t>(a, b, &Sweeper::meterDist);

  _stats[t].timeFullGeoCheckLineLine += TOOK(ts);
  _stats[t].fullGeoChecksLineLine++;

  return dist;
}

// _____________________________________________________________________________
double Sweeper::distCheck(const LineSegment<int32_t>& a, const Line* b,
                          size_t t) const {
  auto ts = TIME();
  double scaleFactor = std::max(
      std::max(getMaxScaleFactor(a.second), getMaxScaleFactor(a.first)),
      getMaxScaleFactor(b->box));

  auto dist = util::geo::withinDist<int32_t>(
      I32XSortedLine(a), b->geom, getBoundingBox(a), b->box,
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
double Sweeper::distCheck(const LineSegment<int32_t>& a, const Area* b,
                          size_t t) const {
  auto ts = TIME();

  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect({{1, 0}, {getBoxId(a.first), 0}}, b->boxIds);
    _stats[t].timeBoxIdIsectAreaLine += TOOK(ts);

    if (r.first) return 0;
  }

  double scaleFactor = std::max(
      getMaxScaleFactor(b->box),
      std::max(getMaxScaleFactor(a.second), getMaxScaleFactor(a.first)));

  auto dist = util::geo::withinDist<int32_t>(
      I32XSortedLine(a), b->geom, getBoundingBox(a), b->box,
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

// _____________________________________________________________________________
std::shared_ptr<sj::Point> Sweeper::getPoint(size_t id, GeomType gt,
                                             size_t t) const {
  std::shared_ptr<sj::Point> ret;
  auto ts = TIME();
  if (gt == sj::FOLDED_POINT) {
    ret = std::make_shared<sj::Point>(sj::Point{unfoldString(id), 0});
  } else {
    ret = _pointCache.get(id, t);
  }
  _stats[t].timeGeoCacheRetrievalPoint += TOOK(ts);

  return ret;
}

// _____________________________________________________________________________
std::shared_ptr<sj::SimpleLine> Sweeper::getSimpleLine(const JobVal& cur,
                                                       size_t t) const {
  if (cur.type == sj::FOLDED_SIMPLE_LINE) {
    return std::make_shared<sj::SimpleLine>(
        sj::SimpleLine{unfoldString(cur.id)});
  }

  return _simpleLineCache.get(cur.id, t);
}

// _____________________________________________________________________________
std::shared_ptr<sj::Area> Sweeper::getArea(const JobVal& sv, size_t t) const {
  auto ts = TIME();
  std::shared_ptr<Area> asp;

  if (sv.type == SIMPLE_POLYGON) {
    auto p = _simpleAreaCache.get(sv.id, sv.large ? -1 : t);
    asp = std::make_shared<sj::Area>(sj::Area(areaFromSimpleArea(p.get())));
  } else if (sv.type == FOLDED_BOX_POLYGON) {
    SimpleArea sa;
    sa.id = unfoldString(sv.id);
    sa.geom = util::geo::Polygon<int32_t>(
                  getBoundingBox(util::geo::Line<int32_t>{sv.point, sv.point2}))
                  .getOuter();
    asp = std::make_shared<sj::Area>(sj::Area(areaFromSimpleArea(&sa)));
  } else {
    asp = _areaCache.get(sv.id, sv.large ? -1 : t);
  }

  _stats[t].timeGeoCacheRetrievalArea += TOOK(ts);

  return asp;
}
