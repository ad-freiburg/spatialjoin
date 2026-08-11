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
#include "GeometryCacheManager.h"
#include "util/Misc.h"
#include "util/geo/IntervalIdx.h"
#include "util/log/Log.h"

using sj::GeomCheckRes;
using sj::GeometryCacheManager;
using sj::GeomType;
using sj::boxids::boxIdIsect;
using sj::boxids::BoxIdList;
using sj::boxids::getBoxId;
using sj::boxids::getBoxIds;
using sj::boxids::packBoxIds;
using util::preadAll;
using util::pwriteAll;
using util::readAll;
using util::writeAll;
using util::geo::area;
using util::geo::DE9IM;
using util::geo::DPoint;
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
using util::geo::Point;
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
I32Box GeometryCacheManager::add(const I32MultiPolygon& a,
                                 const std::string& gid, bool side,
                                 WriteBatch& batch) const {
  size_t subid = 0;  // a subid of 0 means "single polygon"
  if (a.size() > 1) subid = 1;

  return add(a, gid, subid, side, batch);
}

// _____________________________________________________________________________
I32Box GeometryCacheManager::add(const I32MultiLine& a, const std::string& gid,
                                 bool side, WriteBatch& batch) const {
  size_t subid = 0;  // a subid of 0 means "single line"
  if (a.size() > 1) subid = 1;

  return add(a, gid, subid, side, batch);
}

// _____________________________________________________________________________
I32Box GeometryCacheManager::add(const I32MultiPoint& a, const std::string& gid,
                                 bool side, WriteBatch& batch) const {
  size_t subid = 0;  // a subid of 0 means "single point"
  if (a.size() > 1) subid = 1;

  return add(a, gid, subid, side, batch);
}

// _____________________________________________________________________________
I32Box GeometryCacheManager::add(const I32MultiPolygon& a,
                                 const std::string& gid, size_t subId,
                                 bool side, WriteBatch& batch) const {
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
I32Box GeometryCacheManager::add(const I32MultiLine& a, const std::string& gid,
                                 size_t subId, bool side,
                                 WriteBatch& batch) const {
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
I32Box GeometryCacheManager::add(const I32MultiPoint& a, const std::string& gid,
                                 size_t subid, bool side,
                                 WriteBatch& batch) const {
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
void GeometryCacheManager::multiAdd(const std::string& gid, bool side,
                                    int32_t xLeft, int32_t xRight,
                                    const I32Point& pointRight) {
  auto i = _multiGidToId[side].find(gid);

  if (i == _multiGidToId[side].end()) {
    _multiIds[side].push_back(gid);
    _multiRightX[side].push_back(xRight);
    _multiRightPoint[gid] = pointRight;
    _multiLeftX[side].push_back(xLeft);
    _multiGidToId[side][gid] = _multiIds[side].size() - 1;
    _subSizes[gid] = 1;
  } else {
    size_t id = _multiGidToId[side][gid];
    if (xRight > _multiRightX[side][id]) _multiRightX[side][id] = xRight;
    if (pointRight.getX() > _multiRightPoint[gid].getX())
      _multiRightPoint[gid] = pointRight;
    if (xLeft < _multiLeftX[side][id]) _multiLeftX[side][id] = xLeft;
    _subSizes[gid] = _subSizes[gid] + 1;
  }
}

// _____________________________________________________________________________
void GeometryCacheManager::add(const std::string& parentR,
                               const util::geo::I32Box& box,
                               const std::string& gidR, size_t subid, bool side,
                               WriteBatch& batch) const {
  // NOTE: referencing atm *only* works if the referenced geometry is a non-
  // multi geometry. If a multi geometry is referenced, the behavior is
  // undefined.
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
I32Box GeometryCacheManager::add(const I32Polygon& poly, const std::string& gid,
                                 bool side, WriteBatch& batch) const {
  return add(poly, gid, 0, side, batch);
}

// _____________________________________________________________________________
I32Box GeometryCacheManager::add(const I32Polygon& poly,
                                 const std::string& gidR, size_t subid,
                                 bool side, WriteBatch& batch) const {
  if (subid == 0 && _cfg.de9imFilter != util::geo::FANY) {
    // drop certain geometries if we can be sure that they will never match
    // the given DE-9IM filter
    if (_cfg.de9imFilter.minBoundaryDim() > 1) return {};
    if (_cfg.de9imFilter.maxInteriorDim() < 2) return {};
    if (side && _cfg.de9imFilter.maxRightInteriorDim() < 2) return {};
    if (side && _cfg.de9imFilter.minRightBoundaryDim() > 1) return {};
    if (_numSides > 1 && !side && _cfg.de9imFilter.maxLeftInteriorDim() < 2)
      return {};
    if (_numSides > 1 && !side && _cfg.de9imFilter.minLeftBoundaryDim() > 1)
      return {};
  }

  std::string gid = (side ? ("B" + gidR) : ("A" + gidR));

  WriteCand cur;
  I32XSortedPolygon spoly(poly);
  const auto& rawBox = spoly.boundingBox();
  const auto& box = getPaddedBoundingBox(rawBox);
  if (!util::geo::intersects(box, _filterBox)) return {};

  if (spoly.empty()) return box;

  size_t polySize = poly.size();
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
                    4,
                    box45,
                    side,
                    false,
                    0};
    cur.boxvalOut = {0,  // placeholder, will be overwritten later on
                     box.getLowerLeft().getY(),
                     box.getUpperRight().getY(),
                     box.getUpperRight().getX(),
                     true,
                     FOLDED_BOX_POLYGON,
                     areaSize,
                     box.getLowerLeft(),
                     4,
                     box45,
                     side,
                     false,
                     0};
    batch.foldedBoxAreas.emplace_back(cur);
  } else if (poly.getInners().size() == 0 && poly.getOuter().size() < 10 &&
             subid == 0 && (!_cfg.useBoxIds || boxIds.front().first == 1)) {
    std::stringstream str;
    _simpleAreaCache.writeTo({poly.getOuter(), gid}, str);
    cur.raw = str.str();

    auto rightPoint = poly.getOuter().front();

    for (const auto& p : poly.getOuter()) {
      if (p.getX() > rightPoint.getX()) rightPoint = p;
    }

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
                    poly.size(),
                    box45,
                    side,
                    estimatedSize > GEOM_LARGENESS_THRESHOLD,
                    0};
    cur.boxvalOut = {0,  // placeholder, will be overwritten later on
                     box.getLowerLeft().getY(),
                     box.getUpperRight().getY(),
                     box.getUpperRight().getX(),
                     true,
                     SIMPLE_POLYGON,
                     areaSize,
                     rightPoint,
                     poly.size(),
                     box45,
                     side,
                     estimatedSize > GEOM_LARGENESS_THRESHOLD,
                     0};
    batch.simpleAreas.emplace_back(cur);
  } else {
    if (!_cfg.useFastSweepSkip) {
      spoly.setInnerMaxSegLen(std::numeric_limits<int32_t>::max());
      spoly.getOuter().setMaxSegLen(std::numeric_limits<int32_t>::max());
      for (auto& inner : spoly.getInners()) {
        inner.setMaxSegLen(std::numeric_limits<int32_t>::max());
      }
    }

    util::geo::I32Polygon obb;

    if (_cfg.useOBB && poly.getOuter().size() >= OBB_MIN_SIZE) {
      obb = util::geo::convexHull(
          util::geo::pad(util::geo::getOrientedEnvelope(poly), 10));

      // drop redundant oriented bbox
      if (obb.getOuter().size() >= poly.getOuter().size()) obb = {};
    }

    // careful, assign this before move below
    auto rightPoint = spoly.getOuter().rawRing().back().p;

    std::stringstream str;
    _areaCache.writeTo({std::move(spoly), gid, subid, boxIds, obb}, str);
    ;

    size_t estimatedSize = spoly.getOuter().rawRing().size() *
                           sizeof(util::geo::XSortedTuple<int32_t>);
    for (const auto& p : spoly.getInners()) {
      estimatedSize +=
          p.rawRing().size() * sizeof(util::geo::XSortedTuple<int32_t>);
    }

    cur.raw = str.str();

    int32_t polySizeCapped = polySize < std::numeric_limits<int32_t>::max()
                                 ? static_cast<int32_t>(polySize)
                                 : std::numeric_limits<int32_t>::max();

    cur.boxvalIn = {0,  // placeholder, will be overwritten later on
                    box.getLowerLeft().getY(),
                    box.getUpperRight().getY(),
                    box.getLowerLeft().getX(),
                    false,
                    POLYGON,
                    areaSize,
                    {},
                    polySize,
                    box45,
                    side,
                    estimatedSize > GEOM_LARGENESS_THRESHOLD,
                    polySizeCapped};
    cur.boxvalOut = {0,  // placeholder, will be overwritten later on
                     box.getLowerLeft().getY(),
                     box.getUpperRight().getY(),
                     box.getUpperRight().getX(),
                     true,
                     POLYGON,
                     areaSize,
                     rightPoint,
                     polySize,
                     box45,
                     side,
                     estimatedSize > GEOM_LARGENESS_THRESHOLD,
                     polySizeCapped};
    batch.areas.emplace_back(cur);
  }

  return box;
}

// _____________________________________________________________________________
I32Box GeometryCacheManager::add(const I32Line& line, const std::string& gid,
                                 bool side, WriteBatch& batch) const {
  return add(line, gid, 0, side, batch);
}

// _____________________________________________________________________________
I32Box GeometryCacheManager::add(const I32Line& line, const std::string& gidR,
                                 size_t subid, bool side,
                                 WriteBatch& batch) const {
  if (line.size() < 2) return {};

  if (subid == 0 && _cfg.de9imFilter != util::geo::FANY) {
    // drop certain geometries if we can be sure that they will never match
    // the given DE-9IM filter
    if (_cfg.de9imFilter.minInteriorDim() > 1) return {};
    if (_cfg.de9imFilter.minBoundaryDim() > 0) return {};
    if (_cfg.de9imFilter.maxInteriorDim() < 1) return {};
    if (side && _cfg.de9imFilter.minRightInteriorDim() > 1) return {};
    if (side && _cfg.de9imFilter.minRightBoundaryDim() > 0) return {};
    if (side && _cfg.de9imFilter.maxRightInteriorDim() < 1) return {};
    if (_numSides > 1 && !side && _cfg.de9imFilter.minLeftInteriorDim() > 1)
      return {};
    if (_numSides > 1 && !side && _cfg.de9imFilter.maxLeftInteriorDim() < 1)
      return {};
    if (_numSides > 1 && !side && _cfg.de9imFilter.minLeftBoundaryDim() > 0)
      return {};
  }
  if (_cfg.de9imFilter.maxExteriorDim() < 2) return {};

  std::string gid = (side ? ("B" + gidR) : ("A" + gidR));

  WriteCand cur;

  I32XSortedLine sline(line);

  const auto& rawBox = sline.boundingBox();
  const auto& box = getPaddedBoundingBox(rawBox);

  if (!util::geo::intersects(box, _filterBox)) return {};
  BoxIdList boxIds;

  if (_cfg.useBoxIds) {
    boxIds = packBoxIds(getBoxIds(line, rawBox));
  }

  const double len = util::geo::len(line);
  size_t lineSize = line.size();

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
        2,
        box45,
        side,
        false,
        0};
    cur.boxvalOut = {
        0,  // placeholder, will be overwritten later on,
        box.getLowerLeft().getY(),
        box.getUpperRight().getY(),
        box.getUpperRight().getX(),
        true,
        SIMPLE_LINE,
        len,
        line.front().getX() < line.back().getX() ? line.front() : line.back(),
        2,
        box45,
        side,
        false,
        0};

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
    if (line.empty()) return {};
    if (sline.rawLine().empty()) return {};
    auto rightPoint = sline.rawLine().back().p;
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
    _lineCache.writeTo({std::move(sline), gid, subid, boxIds, obb}, str);
    cur.raw = str.str();

    size_t estimatedSize =
        line.size() * sizeof(util::geo::XSortedTuple<int32_t>);

    int32_t lineSizeCapped = lineSize < std::numeric_limits<int32_t>::max()
                                 ? static_cast<int32_t>(lineSize)
                                 : std::numeric_limits<int32_t>::max();

    cur.boxvalIn = {0,  // placeholder, will be overwritten later on
                    box.getLowerLeft().getY(),
                    box.getUpperRight().getY(),
                    box.getLowerLeft().getX(),
                    false,
                    LINE,
                    len,
                    {},
                    lineSize,
                    box45,
                    side,
                    estimatedSize > GEOM_LARGENESS_THRESHOLD,
                    lineSizeCapped};
    cur.boxvalOut = {0,  // placeholder, will be overwritten later on
                     box.getLowerLeft().getY(),
                     box.getUpperRight().getY(),
                     box.getUpperRight().getX(),
                     true,
                     LINE,
                     len,
                     rightPoint,
                     lineSize,
                     box45,
                     side,
                     estimatedSize > GEOM_LARGENESS_THRESHOLD,
                     lineSizeCapped};
    batch.lines.emplace_back(cur);
  }

  return box;
}

// _____________________________________________________________________________
I32Box GeometryCacheManager::add(const I32Point& point, const std::string& gid,
                                 bool side, WriteBatch& batch) const {
  return add(point, gid, 0, side, batch);
}

// _____________________________________________________________________________
I32Box GeometryCacheManager::add(const I32Point& point, const std::string& gidR,
                                 size_t subid, bool side,
                                 WriteBatch& batch) const {
  if (subid == 0 && _cfg.de9imFilter != util::geo::FANY) {
    // drop certain geometries if we can be sure that they will never match
    // the given DE-9IM filter
    if (_cfg.de9imFilter.minInteriorDim() > 0) return {};
    if (_cfg.de9imFilter.minBoundaryDim() >= 0) return {};
    if (_cfg.de9imFilter.maxInteriorDim() < 0) return {};
    if (side && _cfg.de9imFilter.minRightInteriorDim() > 0) return {};
    if (side && _cfg.de9imFilter.maxRightInteriorDim() < 0) return {};
    if (side && _cfg.de9imFilter.minRightBoundaryDim() >= 0) return {};
    if (_numSides > 1 && !side && _cfg.de9imFilter.minLeftInteriorDim() > 0)
      return {};
    if (_numSides > 1 && !side && _cfg.de9imFilter.maxLeftInteriorDim() < 0)
      return {};
    if (_numSides > 1 && !side && _cfg.de9imFilter.minLeftBoundaryDim() >= 0)
      return {};
  }

  if (_cfg.de9imFilter.maxExteriorDim() < 2) return {};

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
                  1,
                  getPaddedBoundingBox(pointR, rawBox),
                  side,
                  false,
                  0};
  cur.boxvalOut = {0,  // placeholder, will be overwritten later on
                   box.getLowerLeft().getY(),
                   box.getUpperRight().getY(),
                   box.getUpperRight().getX(),
                   true,
                   POINT,
                   0,
                   point,
                   1,
                   getPaddedBoundingBox(pointR, rawBox),
                   side,
                   false,
                   0};

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
void GeometryCacheManager::addBatch(WriteBatch& cands) {
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
               cand.boxvalOut.val, cand.boxvalOut.point);
    }
  }

  for (const auto& cand : cands.simpleLines) {
    if (cand.subid > 0) {
      std::unique_lock<std::mutex> lock(_multiAddMtx);
      multiAdd(cand.gid, cand.boxvalIn.side, cand.boxvalIn.val,
               cand.boxvalOut.val, cand.boxvalOut.point);
    }
  }

  for (const auto& cand : cands.lines) {
    if (cand.subid > 0) {
      std::unique_lock<std::mutex> lock(_multiAddMtx);
      multiAdd(cand.gid, cand.boxvalIn.side, cand.boxvalIn.val,
               cand.boxvalOut.val, cand.boxvalOut.point);
    }
  }

  for (const auto& cand : cands.simpleAreas) {
    if (cand.subid > 0) {
      std::unique_lock<std::mutex> lock(_multiAddMtx);
      multiAdd(cand.gid, cand.boxvalIn.side, cand.boxvalIn.val,
               cand.boxvalOut.val, cand.boxvalOut.point);
    }
  }

  for (const auto& cand : cands.areas) {
    if (cand.subid > 0) {
      std::unique_lock<std::mutex> lock(_multiAddMtx);
      multiAdd(cand.gid, cand.boxvalIn.side, cand.boxvalIn.val,
               cand.boxvalOut.val, cand.boxvalOut.point);
    }
  }

  for (const auto& cand : cands.refs) {
    if (cand.subid > 0) {
      std::unique_lock<std::mutex> lock(_multiAddMtx);
      multiAdd(cand.gid, cand.boxvalIn.side, cand.boxvalIn.val,
               cand.boxvalOut.val, cand.boxvalOut.point);
    }
  }

  {
    std::unique_lock<std::mutex> lock(_sweepEventWriteMtx);
    for (const auto& cand : cands.foldedPoints) {
      _events.add(cand.boxvalIn);
      _events.add(cand.boxvalOut);
      if (_events.numObjects() % 1000000 == 0)
        log("@ " + std::to_string(_events.numObjects()));
    }
    for (const auto& cand : cands.points) {
      _events.add(cand.boxvalIn);
      _events.add(cand.boxvalOut);
      if (_events.numObjects() % 1000000 == 0)
        log("@ " + std::to_string(_events.numObjects()));
    }
    for (const auto& cand : cands.foldedSimpleLines) {
      _events.add(cand.boxvalIn);
      _events.add(cand.boxvalOut);
      if (_events.numObjects() % 1000000 == 0)
        log("@ " + std::to_string(_events.numObjects()));
    }
    for (const auto& cand : cands.foldedBoxAreas) {
      _events.add(cand.boxvalIn);
      _events.add(cand.boxvalOut);
      if (_events.numObjects() % 1000000 == 0)
        log("@ " + std::to_string(_events.numObjects()));
    }
    for (const auto& cand : cands.simpleLines) {
      _events.add(cand.boxvalIn);
      _events.add(cand.boxvalOut);
      if (_events.numObjects() % 1000000 == 0)
        log("@ " + std::to_string(_events.numObjects()));
    }
    for (const auto& cand : cands.lines) {
      _events.add(cand.boxvalIn);
      _events.add(cand.boxvalOut);
      if (_events.numObjects() % 1000000 == 0)
        log("@ " + std::to_string(_events.numObjects()));
    }
    for (const auto& cand : cands.simpleAreas) {
      _events.add(cand.boxvalIn);
      _events.add(cand.boxvalOut);
      if (_events.numObjects() % 1000000 == 0)
        log("@ " + std::to_string(_events.numObjects()));
    }
    for (const auto& cand : cands.areas) {
      _events.add(cand.boxvalIn);
      _events.add(cand.boxvalOut);
      if (_events.numObjects() % 1000000 == 0)
        log("@ " + std::to_string(_events.numObjects()));
    }
    for (const auto& cand : cands.refs) {
      _refs[cand.raw][0][cand.gid] = cand.subid;
      _selfCheckBounds[cand.raw] = util::geo::getBoundingBox(
          I32Point{cand.boxvalIn.val, cand.boxvalIn.loY});
      if (_events.numObjects() % 1000000 == 0)
        log("@ " + std::to_string(_events.numObjects()));
    }
  }
}

// _____________________________________________________________________________
void GeometryCacheManager::flush() {
  if (_numSides > 1) log("(Non-self join between 2 datasets)");

  log(std::to_string(_multiIds[0].size() + _multiIds[1].size()) +
      " multi geometries");

  for (const auto& ref : _refs) {
    for (const auto& sub : ref.second) {
      _selfChecks.push_back({ref.first, sub.first});

      _events.add({_selfChecks.size() - 1,
               1,
               0,
               _selfCheckBounds[ref.first].getLowerLeft().getX(),
               false,
               SELF_CHECK,
               0.0,
               {},
               0,
               {},
               false,
               false,
               0});
    }
  }

  for (size_t side = 0; side < 2; side++) {
    for (size_t i = 0; i < _multiIds[side].size(); i++) {
      _events.add({i,
               1,
               0,
               _multiLeftX[side][i] - 1,
               false,
               POINT,
               0.0,
               {},
               0,
               {},
               static_cast<bool>(side),
               false,
               0});
    }
  }

  _pointCache.flush();
  _areaCache.flush();
  _simpleAreaCache.flush();
  _lineCache.flush();
  _simpleLineCache.flush();

  log("Sorting events...");

  _events.flush();

  log("...done");

  // TODO!
  // duplicatesToReferences();

  log(std::to_string(_refs.size()) + " reference geometries");
}

// _____________________________________________________________________________
void GeometryCacheManager::duplicatesToReferences() {
  // start at beginning of _file
  lseek(_file, 0, SEEK_SET);

  const size_t RBUF_SIZE = 100000;
  unsigned char* buf = new unsigned char[sizeof(BoxVal) * RBUF_SIZE];

  std::unordered_set<size_t> deleted;
  std::unordered_set<size_t> referenced;

  log("Removing duplicates...");

  ssize_t len;
  size_t jj = 0;

  int32_t curX = 0;

  std::unordered_map<uint64_t, std::pair<size_t, bool>> duplicatePolys,
      duplicateLines;

  size_t pos = 0;

  try {
    while ((len = preadAll(_file, buf, sizeof(BoxVal) * RBUF_SIZE, pos)) != 0) {
      size_t posOld = pos;
      pos += len;
      if (len < 0) {
        std::stringstream ss;
        ss << "Could not read from events file \n";
        ss << strerror(errno) << std::endl;
        throw std::runtime_error(ss.str());
      }

      if (len % sizeof(BoxVal))
        throw std::runtime_error("Corrupted events file");

      bool updated = false;

      for (ssize_t i = 0; i < len; i += sizeof(BoxVal)) {
        auto cur = reinterpret_cast<BoxVal*>(buf + i);

        if (_cfg.sweepCancellationCb && jj % 10000 == 0) {
          _cfg.sweepCancellationCb();
        }

        jj++;

        if (cur->out) {
          if ((cur->type == POLYGON || cur->type == LINE) &&
              deleted.erase(cur->id)) {
            // erase it if present, to avoid unnecessary memory consumption
            cur->type = DELETED;
            updated = true;
          }
          referenced.erase(cur->id);
          continue;
        }

        if (curX != cur->val) {
          // new equal-X block
          duplicatePolys = {};
          duplicateLines = {};
          curX = cur->val;
        }

        if (cur->type == POLYGON && cur->size >= DUPLICATE_REMOVAL_MIN_SIZE) {
          size_t h = cur->numAnchors;
          const auto& existing = duplicatePolys.find(h);

          if (existing != duplicatePolys.end()) {
            auto a = _areaCache.get(cur->id, cur->large ? -1 : 0);
            auto b = _areaCache.get(existing->second.first,
                                    existing->second.second ? -1 : 0);

            if (a->geom == b->geom) {
              deleted.insert(cur->id);
              if (referenced.insert(existing->second.first).second) {
                // for the first element referencing this, modify this
                // event to the self check of the referenced geom
                cur->type = SELF_CHECK_AREA;
                _selfChecks.push_back({b->id, b->subId});
                cur->id = _selfChecks.size() - 1;
              } else {
                cur->type = DELETED;
              }

              updated = true;
              _refs[b->id][b->subId][a->id] = a->subId;
            }
          } else {
            duplicatePolys[h] = {(size_t)cur->id, cur->large};
          }
        }

        if (cur->type == LINE && cur->size >= DUPLICATE_REMOVAL_MIN_SIZE) {
          size_t h = cur->numAnchors;
          const auto& existing = duplicateLines.find(h);

          if (existing != duplicateLines.end()) {
            auto a = _lineCache.get(cur->id, cur->large ? -1 : 0);
            auto b = _lineCache.get(existing->second.first,
                                    existing->second.second ? -1 : 0);

            if (a->geom == b->geom) {
              deleted.insert(cur->id);
              if (referenced.insert(existing->second.first).second) {
                // for the first element referencing this, modify this
                // event to the self check of the referenced geom
                cur->type = SELF_CHECK_LINE;
                _selfChecks.push_back({b->id, b->subId});
                cur->id = _selfChecks.size() - 1;
              } else {
                cur->type = DELETED;
              }
              updated = true;
              _refs[b->id][b->subId][a->id] = a->subId;
            }
          } else {
            duplicateLines[h] = {(size_t)cur->id, cur->large};
          }
        }
      }

      // if we changed something in this buffer, write it back
      if (updated) pwriteAll(_file, buf, len, posOld);
    }
  } catch (...) {
    // graceful handling of an exception during sweep

    delete[] buf;

    // set the cancelled variable to true
    _cancelled = true;

    // rethrow exception
    throw;
  }

  delete[] buf;

  log("...done");
}

// _____________________________________________________________________________
void GeometryCacheManager::log(const std::string& msg) {
  if (_cfg.logCb) _cfg.logCb(msg);
}

// _____________________________________________________________________________
template <template <typename> class G1, template <typename> class G2,
          typename T>
util::geo::I32Box GeometryCacheManager::getPaddedBoundingBox(
    const G1<T>& geom, const G2<T>& refGeom) const {
  auto bbox = util::geo::getBoundingBox(geom);

  if (_cfg.withinDist >= 0) {
    if (_cfg.euclideanDist && !_cfg.haversineApprox)
      return util::geo::pad(bbox, _cfg.withinDist * PREC / 2.0);

    auto a = (reinterpret_cast<const void*>(&geom) ==
                      reinterpret_cast<const void*>(&refGeom)
                  ? bbox
                  : util::geo::getBoundingBox(refGeom));

    // convert distanceUpperBound (meters) to maximum latitude padding (degrees)
    // we have to "pad" the box by -dy and dy because the distance path could
    // be within that padded box, and thus the distortions have to be computed
    // based on that path
    double dLat =
        _cfg.withinDist / util::geo::MIN_METERS_PER_LAT_RAD * util::geo::IRAD;
    auto upper = util::geo::webMercToLatLng<double>(
        0.0, a.getUpperRight().getY() * 1.0 / PREC);
    auto lower = util::geo::webMercToLatLng<double>(
        0.0, a.getLowerLeft().getY() * 1.0 / PREC);

    double scaleLatUp =
        cos(std::min(90.0 - util::geo::EPSILON, (upper.getY() + dLat)) *
            util::geo::RAD);
    double scaleLatLow =
        cos(std::max(-90.0 + util::geo::EPSILON, (lower.getY() - dLat)) *
            util::geo::RAD);
    double scaleFactor = std::min(scaleLatUp, scaleLatLow);

    double pad = (_cfg.withinDist / 2.0) / scaleFactor * PREC;

    double llx = bbox.getLowerLeft().getX();
    double lly = bbox.getLowerLeft().getY();
    double urx = bbox.getUpperRight().getX();
    double ury = bbox.getUpperRight().getY();

    double m = sj::boxids::WORLD_W / 2.0;

    // restrict padding to world extent
    T llxt = -m;
    T llyt = -m;
    T urxt = m;
    T uryt = m;

    if (llx - pad > -m) {
      llxt = llx - pad;
    }

    if (lly - pad > -m) {
      llyt = lly - pad;
    }

    if (urx + pad < m) {
      urxt = urx + pad;
    }

    if (ury + pad < m) {
      uryt = ury + pad;
    }

    return {{llxt, llyt}, {urxt, uryt}};
  }

  return bbox;
}

// _____________________________________________________________________________
std::pair<size_t, size_t> GeometryCacheManager::size() const {
  auto cacheSizePoint = _pointCache.size();
  auto cacheSizeArea = _areaCache.size();
  auto cacheSizeSimpleArea = _simpleAreaCache.size();
  auto cacheSizeSimpleLine = _simpleLineCache.size();
  auto cacheSizeLine = _lineCache.size();

  return {cacheSizePoint.first + cacheSizeArea.first +
              cacheSizeSimpleArea.first + cacheSizeSimpleLine.first +
              cacheSizeLine.first,
          cacheSizePoint.second + cacheSizeArea.second +
              cacheSizeSimpleArea.second + cacheSizeSimpleLine.second +
              cacheSizeLine.second};
}

// _____________________________________________________________________________
size_t GeometryCacheManager::foldString(const std::string& s) {
  size_t ret = 0;
  for (size_t i = 0; i < std::min((size_t)7, s.size()); i++) {
    size_t tmp = static_cast<unsigned char>(s[i]);
    ret |= tmp << (i * 8);
  }

  // highest byte stores the length
  ret |= (s.size() << 56);

  return ret;
};
