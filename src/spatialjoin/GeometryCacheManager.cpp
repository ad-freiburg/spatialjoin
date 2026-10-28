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
                                    int32_t xRight,
                                    const I32Point& pointRight) {
  auto i = _multiGidToId[side].find(gid);

  if (i == _multiGidToId[side].end()) {
    _multiIds[side].push_back(gid);
    _multiRightX[side].push_back(xRight);
    _multiRightPoint[gid] = pointRight;
    _multiGidToId[side][gid] = _multiIds[side].size() - 1;
    _subSizes[gid] = 1;
  } else {
    size_t id = _multiGidToId[side][gid];
    if (xRight > _multiRightX[side][id]) _multiRightX[side][id] = xRight;
    if (pointRight.getX() > _multiRightPoint[gid].getX())
      _multiRightPoint[gid] = pointRight;
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
  const auto& densePoly = util::geo::densifyX(poly, 500 * PREC);
  I32XSortedPolygon spoly(densePoly);
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
      poly.getOuter().size() == densePoly.getOuter().size() &&
      (!_cfg.useBoxIds || boxIds.front().first == 1) &&
      area(rawBox) == areaSize) {
    cur.boxvalIn = {typedId(0, FOLDED_BOX_POLYGON),  // placeholder ID
                    box.getLowerLeft().getY(),
                    box.getUpperRight().getY(),
                    box.getLowerLeft().getX(),
                    false,
                    areaSize,
                    rawBox.getUpperRight(),
                    packPoint(rawBox.getLowerLeft()),
                    box45,
                    side};
    cur.boxvalOut = {typedId(0, FOLDED_BOX_POLYGON),  // placeholder ID
                     box.getLowerLeft().getY(),
                     box.getUpperRight().getY(),
                     box.getUpperRight().getX(),
                     true,
                     areaSize,
                     rawBox.getLowerLeft(),
                     packPoint(rawBox.getUpperRight()),
                     box45,
                     side};
    batch.foldedBoxAreas.emplace_back(cur);
  } else if (poly.getInners().size() == 0 && poly.getOuter().size() < 10 &&
             poly.getOuter().size() == densePoly.getOuter().size() &&
             subid == 0 && (!_cfg.useBoxIds || boxIds.front().first == 1)) {
    std::stringstream str;
    _simpleAreaStore.writeTo({poly.getOuter(), gid}, str);
    cur.raw = str.str();

    auto rightPoint = poly.getOuter().front();

    for (const auto& p : poly.getOuter()) {
      if (p.getX() > rightPoint.getX()) rightPoint = p;
    }

    cur.boxvalIn = {typedId(0, SIMPLE_POLYGON),  // placeholder ID
                    box.getLowerLeft().getY(),
                    box.getUpperRight().getY(),
                    box.getLowerLeft().getX(),
                    false,
                    areaSize,
                    {},
                    poly.size(),
                    box45,
                    side};
    cur.boxvalOut = {typedId(0, SIMPLE_POLYGON),  // placeholder ID
                     box.getLowerLeft().getY(),
                     box.getUpperRight().getY(),
                     box.getUpperRight().getX(),
                     true,
                     areaSize,
                     rightPoint,
                     poly.size(),
                     box45,
                     side};
    batch.simpleAreas.emplace_back(cur);
  } else {
    if (!_cfg.useFastSweepSkip) {
      spoly.setInnerMaxSegLen(std::numeric_limits<int32_t>::max());
      spoly.getOuter().setMaxSegLen(std::numeric_limits<int32_t>::max());
      for (auto& inner : spoly.getInners()) {
        inner.setMaxSegLen(std::numeric_limits<int32_t>::max());
      }
    }

    // careful, assign this before move below
    auto rightPoint = spoly.getOuter().rawRing().back().p;

    std::stringstream str;
    _areaStore.writeTo({std::move(spoly), gid, subid, boxIds}, str);
    ;

    cur.raw = str.str();

    cur.boxvalIn = {typedId(0, POLYGON),  // placeholder ID
                    box.getLowerLeft().getY(),
                    box.getUpperRight().getY(),
                    box.getLowerLeft().getX(),
                    false,
                    areaSize,
                    {},
                    polySize,
                    box45,
                    side};
    cur.boxvalOut = {typedId(0, POLYGON),  // placeholder ID
                     box.getLowerLeft().getY(),
                     box.getUpperRight().getY(),
                     box.getUpperRight().getX(),
                     true,
                     areaSize,
                     rightPoint,
                     polySize,
                     box45,
                     side};
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

  const auto& denseLine = util::geo::densifyX(line, 500 * PREC);
  I32XSortedLine sline(denseLine);

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

  if (line.size() == 2 && denseLine.size() == line.size() &&
      (!_cfg.useBoxIds || boxIds.front().first == 1) && subid == 0) {
    // simple line

    cur.boxvalIn = {
        typedId(0, SIMPLE_LINE),  // placeholder ID
        box.getLowerLeft().getY(),
        box.getUpperRight().getY(),
        box.getLowerLeft().getX(),
        false,
        len,
        line.front().getX() < line.back().getX() ? line.back() : line.front(),
        packPoint(line.front().getX() < line.back().getX() ? line.front()
                                                           : line.back()),
        box45,
        side};
    cur.boxvalOut = {
        typedId(0, SIMPLE_LINE),  // placeholder ID
        box.getLowerLeft().getY(),
        box.getUpperRight().getY(),
        box.getUpperRight().getX(),
        true,
        len,
        line.front().getX() < line.back().getX() ? line.front() : line.back(),
        packPoint(line.front().getX() < line.back().getX() ? line.back()
                                                           : line.front()),
        box45,
        side};

    // check if we can fold the gid into the offset id, because the gid is all
    // we store in the cache for points
    if (subid == 0 && gid.size() < 8) {
      cur.boxvalIn.setType(FOLDED_SIMPLE_LINE);
      cur.boxvalOut.setType(FOLDED_SIMPLE_LINE);
      batch.foldedSimpleLines.emplace_back(cur);
    } else {
      std::stringstream str;
      _simpleLineStore.writeTo({gid}, str);

      cur.raw = str.str();

      batch.simpleLines.emplace_back(cur);
    }
  } else {
    // normal line
    if (line.empty()) return {};
    if (sline.rawLine().empty()) return {};
    auto rightPoint = sline.rawLine().back().p;

    if (!_cfg.useFastSweepSkip) {
      sline.setMaxSegLen(std::numeric_limits<int32_t>::max());
    }

    std::stringstream str;
    _lineStore.writeTo({std::move(sline), gid, subid, boxIds}, str);
    cur.raw = str.str();

    cur.boxvalIn = {typedId(0, LINE),  // placeholder ID
                    box.getLowerLeft().getY(),
                    box.getUpperRight().getY(),
                    box.getLowerLeft().getX(),
                    false,
                    len,
                    {},
                    lineSize,
                    box45,
                    side};
    cur.boxvalOut = {typedId(0, LINE),  // placeholder ID
                     box.getLowerLeft().getY(),
                     box.getUpperRight().getY(),
                     box.getUpperRight().getX(),
                     true,
                     len,
                     rightPoint,
                     lineSize,
                     box45,
                     side};
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
  // THIS NO LONGER WORKS IF THE CACHE IS BUILD INDEPENDENT OF THE JOINPREDICATE
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
  cur.boxvalIn = {typedId(0, POINT),  // placeholder ID
                  box.getLowerLeft().getY(),
                  box.getUpperRight().getY(),
                  box.getLowerLeft().getX(),
                  false,
                  0,
                  point,
                  1,
                  getPaddedBoundingBox(pointR, rawBox),
                  side};
  cur.boxvalOut = {typedId(0, POINT),  // placeholder ID
                   box.getLowerLeft().getY(),
                   box.getUpperRight().getY(),
                   box.getUpperRight().getX(),
                   true,
                   0,
                   point,
                   1,
                   getPaddedBoundingBox(pointR, rawBox),
                   side};

  cur.gid = gid;

  // check if we can fold the gid into the offset id, because the gid is all
  // we store in the cache for points
  if (subid == 0 && gid.size() < 8) {
    cur.boxvalIn.setType(FOLDED_POINT);
    cur.boxvalOut.setType(FOLDED_POINT);
    batch.foldedPoints.emplace_back(cur);
  } else {
    std::stringstream str;
    _pointStore.writeTo({gid, subid}, str);

    cur.raw = str.str();

    batch.points.emplace_back(cur);
  }

  return box;
}

// _____________________________________________________________________________
void GeometryCacheManager::addBatch(WriteBatch& cands,
                                    sj::SweepEventList* events) {
  {
    for (auto& cand : cands.foldedPoints) {
      if (cand.boxvalIn.side) _numSides = 2;
      cand.boxvalIn.id =
          typedId(foldString(cand.gid), cand.boxvalIn.getType());
      cand.boxvalOut.id = cand.boxvalIn.id;
    }
  }

  {
    std::unique_lock<std::mutex> lock(_pointGeomCacheWriteMtx);
    for (auto& cand : cands.points) {
      if (cand.boxvalIn.side) _numSides = 2;
      cand.boxvalIn.id = _pointStore.add(cand.raw);
      cand.boxvalOut.id = cand.boxvalIn.id;
    }
  }

  {
    std::unique_lock<std::mutex> lock(_lineGeomCacheWriteMtx);
    for (auto& cand : cands.lines) {
      if (cand.boxvalIn.side) _numSides = 2;
      cand.boxvalIn.id = _lineStore.add(cand.raw);
      cand.boxvalOut.id = cand.boxvalIn.id;
    }
  }

  {
    std::unique_lock<std::mutex> lock(_simpleLineGeomCacheWriteMtx);
    for (auto& cand : cands.simpleLines) {
      if (cand.boxvalIn.side) _numSides = 2;
      cand.boxvalIn.id = _simpleLineStore.add(cand.raw);
      cand.boxvalOut.id = cand.boxvalIn.id;
    }
  }

  {
    for (auto& cand : cands.foldedSimpleLines) {
      if (cand.boxvalIn.side) _numSides = 2;
      cand.boxvalIn.id =
          typedId(foldString(cand.gid), cand.boxvalIn.getType());
      cand.boxvalOut.id = cand.boxvalIn.id;
    }
  }

  {
    for (auto& cand : cands.foldedBoxAreas) {
      if (cand.boxvalIn.side) _numSides = 2;
      cand.boxvalIn.id =
          typedId(foldString(cand.gid), cand.boxvalIn.getType());
      cand.boxvalOut.id = cand.boxvalIn.id;
    }
  }

  {
    std::unique_lock<std::mutex> lock(_simpleAreaGeomCacheWriteMtx);
    for (auto& cand : cands.simpleAreas) {
      if (cand.boxvalIn.side) _numSides = 2;
      cand.boxvalIn.id = _simpleAreaStore.add(cand.raw);
      cand.boxvalOut.id = cand.boxvalIn.id;
    }
  }

  {
    std::unique_lock<std::mutex> lock(_areaGeomCacheWriteMtx);
    for (auto& cand : cands.areas) {
      if (cand.boxvalIn.side) _numSides = 2;
      cand.boxvalIn.id = _areaStore.add(cand.raw);
      cand.boxvalOut.id = cand.boxvalIn.id;
    }
  }

  for (const auto& cand : cands.points) {
    if (cand.subid > 0) {
      std::unique_lock<std::mutex> lock(_multiAddMtx);
      multiAdd(cand.gid, cand.boxvalIn.side, cand.boxvalOut.val,
               cand.boxvalOut.point);
    }
  }

  for (const auto& cand : cands.simpleLines) {
    if (cand.subid > 0) {
      std::unique_lock<std::mutex> lock(_multiAddMtx);
      multiAdd(cand.gid, cand.boxvalIn.side, cand.boxvalOut.val,
               cand.boxvalOut.point);
    }
  }

  for (const auto& cand : cands.lines) {
    if (cand.subid > 0) {
      std::unique_lock<std::mutex> lock(_multiAddMtx);
      multiAdd(cand.gid, cand.boxvalIn.side, cand.boxvalOut.val,
               cand.boxvalOut.point);
    }
  }

  for (const auto& cand : cands.simpleAreas) {
    if (cand.subid > 0) {
      std::unique_lock<std::mutex> lock(_multiAddMtx);
      multiAdd(cand.gid, cand.boxvalIn.side, cand.boxvalOut.val,
               cand.boxvalOut.point);
    }
  }

  for (const auto& cand : cands.areas) {
    if (cand.subid > 0) {
      std::unique_lock<std::mutex> lock(_multiAddMtx);
      multiAdd(cand.gid, cand.boxvalIn.side, cand.boxvalOut.val,
               cand.boxvalOut.point);
    }
  }

  for (const auto& cand : cands.refs) {
    if (cand.subid > 0) {
      std::unique_lock<std::mutex> lock(_multiAddMtx);
      multiAdd(cand.gid, cand.boxvalIn.side, cand.boxvalOut.val,
               cand.boxvalOut.point);
    }
  }

  {
    std::unique_lock<std::mutex> lock(_sweepEventWriteMtx);
    for (const auto& cand : cands.foldedPoints) {
      events->add(cand.boxvalIn);
      events->add(cand.boxvalOut);
      if (events->numObjects() % 1000000 == 0)
        log("@ " + std::to_string(events->numObjects()));
    }
    for (const auto& cand : cands.points) {
      events->add(cand.boxvalIn);
      events->add(cand.boxvalOut);
      if (events->numObjects() % 1000000 == 0)
        log("@ " + std::to_string(events->numObjects()));
    }
    for (const auto& cand : cands.foldedSimpleLines) {
      events->add(cand.boxvalIn);
      events->add(cand.boxvalOut);
      if (events->numObjects() % 1000000 == 0)
        log("@ " + std::to_string(events->numObjects()));
    }
    for (const auto& cand : cands.foldedBoxAreas) {
      events->add(cand.boxvalIn);
      events->add(cand.boxvalOut);
      if (events->numObjects() % 1000000 == 0)
        log("@ " + std::to_string(events->numObjects()));
    }
    for (const auto& cand : cands.simpleLines) {
      events->add(cand.boxvalIn);
      events->add(cand.boxvalOut);
      if (events->numObjects() % 1000000 == 0)
        log("@ " + std::to_string(events->numObjects()));
    }
    for (const auto& cand : cands.lines) {
      events->add(cand.boxvalIn);
      events->add(cand.boxvalOut);
      if (events->numObjects() % 1000000 == 0)
        log("@ " + std::to_string(events->numObjects()));
    }
    for (const auto& cand : cands.simpleAreas) {
      events->add(cand.boxvalIn);
      events->add(cand.boxvalOut);
      if (events->numObjects() % 1000000 == 0)
        log("@ " + std::to_string(events->numObjects()));
    }
    for (const auto& cand : cands.areas) {
      events->add(cand.boxvalIn);
      events->add(cand.boxvalOut);
      if (events->numObjects() % 1000000 == 0)
        log("@ " + std::to_string(events->numObjects()));
    }
    for (const auto& cand : cands.refs) {
      _refs[cand.raw][0][cand.gid] = cand.subid;
      _selfCheckBounds[cand.raw] = util::geo::getBoundingBox(
          I32Point{cand.boxvalIn.val, cand.boxvalIn.loY});
      if (events->numObjects() % 1000000 == 0)
        log("@ " + std::to_string(events->numObjects()));
    }
  }
}

// _____________________________________________________________________________
void GeometryCacheManager::flush(sj::SweepEventList* events) {
  if (_numSides > 1) log("(Non-self join between 2 datasets)");

  log(std::to_string(_multiIds[0].size() + _multiIds[1].size()) +
      " multi geometries");

  for (const auto& ref : _refs) {
    for (const auto& sub : ref.second) {
      _selfChecks.push_back({ref.first, sub.first});

      events->add({typedId(_selfChecks.size() - 1, SELF_CHECK),
                   1,
                   0,
                   _selfCheckBounds[ref.first].getLowerLeft().getX(),
                   false,
                   0.0,
                   {},
                   0,
                   {},
                   false});
    }
  }

  // special multi-OUT event, at the right x of the multi geometry
  for (size_t side = 0; side < 2; side++) {
    for (size_t i = 0; i < _multiIds[side].size(); i++) {
      events->add({typedId(i, POINT),
                   1,
                   0,
                   _multiRightX[side][i],
                   true,
                   0.0,
                   {},
                   0,
                   {},
                   static_cast<bool>(side)});
    }
  }

  _pointStore.flush();
  _areaStore.flush();
  _simpleAreaStore.flush();
  _lineStore.flush();
  _simpleLineStore.flush();

  // TODO!!!!
  // duplicatesToReferences();

  log(std::to_string(_refs.size()) + " reference geometries");
}

// _____________________________________________________________________________
void GeometryCacheManager::duplicatesToReferences() {
  // start at beginning of _file
  lseek(_file, 0, SEEK_SET);

  const size_t RBUF_SIZE = 100000;
  unsigned char* buf = new unsigned char[sizeof(BoxVal) * RBUF_SIZE];

  std::unordered_set<size_t> deletedLines;
  std::unordered_set<size_t> referencedLines;
  std::unordered_set<size_t> deletedPolys;
  std::unordered_set<size_t> referencedPolys;

  log("Removing duplicates...");

  ssize_t len;
  size_t jj = 0;

  int32_t curX = 0;

  std::unordered_map<uint64_t, size_t> duplicatePolys, duplicateLines;

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
          if (cur->getType() == POLYGON) {
            if (deletedPolys.erase(getId(*cur))) {
              // erase it if present, to avoid unnecessary memory consumption
              cur->setType(DELETED);
              updated = true;
            }
            referencedPolys.erase(getId(*cur));
          }

          if (cur->getType() == LINE) {
            if (deletedLines.erase(getId(*cur))) {
              // erase it if present, to avoid unnecessary memory consumption
              cur->setType(DELETED);
              updated = true;
            }
            referencedLines.erase(getId(*cur));
          }

          continue;
        }

        if (curX != cur->val) {
          // new equal-X block
          duplicatePolys = {};
          duplicateLines = {};
          curX = cur->val;
        }

        if (cur->getType() == POLYGON &&
            cur->numAnchors >= DUPLICATE_REMOVAL_MIN_SIZE) {
          size_t h = cur->numAnchors;
          const auto& existing = duplicatePolys.find(h);

          if (existing != duplicatePolys.end()) {
            auto a = get(_areaStore, _areaCache, getId(*cur), 0);
            auto b = get(_areaStore, _areaCache, existing->second, 0);

            if (a->geom == b->geom) {
              deletedPolys.insert(getId(*cur));
              if (referencedPolys.insert(existing->second).second) {
                // for the first element referencing this, modify this
                // event to the self check of the referenced geom
                cur->setType(SELF_CHECK_AREA);
                _selfChecks.push_back({b->id, b->subId});
                cur->id = typedId(_selfChecks.size() - 1, cur->getType());
              } else {
                cur->setType(DELETED);
              }

              updated = true;
              _refs[b->id][b->subId][a->id] = a->subId;
            }
          } else {
            duplicatePolys[h] = getId(*cur);
          }
        }

        if (cur->getType() == LINE &&
            cur->numAnchors >= DUPLICATE_REMOVAL_MIN_SIZE) {
          size_t h = cur->numAnchors;
          const auto& existing = duplicateLines.find(h);

          if (existing != duplicateLines.end()) {
            auto a = get(_lineStore, _lineCache, getId(*cur), 0);
            auto b = get(_lineStore, _lineCache, existing->second, 0);

            if (a->geom == b->geom) {
              deletedLines.insert(getId(*cur));
              if (referencedLines.insert(existing->second).second) {
                // for the first element referencing this, modify this
                // event to the self check of the referenced geom
                cur->setType(SELF_CHECK_LINE);
                _selfChecks.push_back({b->id, b->subId});
                cur->id = typedId(_selfChecks.size() - 1, cur->getType());
              } else {
                cur->setType(DELETED);
              }
              updated = true;
              _refs[b->id][b->subId][a->id] = a->subId;
            }
          } else {
            duplicateLines[h] = getId(*cur);
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
template <typename W>
std::shared_ptr<W> GeometryCacheManager::get(const GeometryStore<W>& store,
                                             GeomLRUCache<W>& cache, size_t off,
                                             size_t tid) const {
  // completely circumvent cache system
  if (store.inMemory()) return store.getFromMem(off);

  tid = tid % _cfg.numCacheThreads;

  std::unique_lock<std::mutex> lock(cache.mutexes[tid]);

  // check if value is in cache
  auto it = cache.idMap[tid].find(off);

  if (it == cache.idMap[tid].end()) {
    // if not, load, cache and return
    const auto& val = store.getFrom(off, tid);
    return this->cache(cache, off, val.second, val.first, tid);
  }

  // if in cache, move to front of list and return
  // splice only changes pointers in the linked list, no copying here
  cache.vals[tid].splice(cache.vals[tid].begin(), cache.vals[tid], it->second);

  return it->second->second.val;
}

template std::shared_ptr<sj::Point> GeometryCacheManager::get(
    const sj::GeometryStore<sj::Point>&, sj::GeomLRUCache<sj::Point>&, size_t,
    size_t) const;
template std::shared_ptr<sj::Area> GeometryCacheManager::get(
    const sj::GeometryStore<sj::Area>&, sj::GeomLRUCache<sj::Area>&, size_t,
    size_t) const;
template std::shared_ptr<sj::SimpleArea> GeometryCacheManager::get(
    const sj::GeometryStore<sj::SimpleArea>&, sj::GeomLRUCache<sj::SimpleArea>&,
    size_t, size_t) const;
template std::shared_ptr<sj::Line> GeometryCacheManager::get(
    const sj::GeometryStore<sj::Line>&, sj::GeomLRUCache<sj::Line>&, size_t,
    size_t) const;
template std::shared_ptr<sj::SimpleLine> GeometryCacheManager::get(
    const sj::GeometryStore<sj::SimpleLine>&, sj::GeomLRUCache<sj::SimpleLine>&,
    size_t, size_t) const;

// _____________________________________________________________________________
template <typename W>
std::shared_ptr<W> GeometryCacheManager::cache(GeomLRUCache<W>& cache,
                                               size_t off, const W& val,
                                               size_t estSize,
                                               size_t tid) const {
  // if cache is too large, pop last elements until we have space
  while (cache.maxSize > 0 && cache.vals[tid].size() > 0 &&
         cache.valSizes[tid] > cache.maxSize) {
    auto last = cache.vals[tid].rbegin();
    cache.valSizes[tid] -= last->second.estimatedSize;
    cache.idMap[tid].erase(last->first);
    cache.vals[tid].pop_back();
  }

  // push value to front
  cache.vals[tid].push_front({off, {estSize, std::make_shared<W>(val)}});

  // if cache has too many elements, pop last element
  if (cache.maxNumElements > 0 &&
      cache.vals[tid].size() > cache.maxNumElements) {
    auto last = cache.vals[tid].rbegin();
    cache.valSizes[tid] -= last->second.estimatedSize;
    cache.idMap[tid].erase(last->first);
    cache.vals[tid].pop_back();
  }

  // set map to front iterator
  cache.idMap[tid][off] = cache.vals[tid].begin();

  // increase total estimated size
  cache.valSizes[tid] += estSize;

  return cache.vals[tid].front().second.val;
}

// _____________________________________________________________________________
template <typename W>
std::pair<size_t, size_t> GeometryCacheManager::size(
    const GeomLRUCache<W>& cache) {
  size_t numEntries = 0;
  size_t bytes = 0;
  for (size_t tid = 0; tid < cache.vals.size(); tid++) {
    std::unique_lock<std::mutex> lock(cache.mutexes[tid]);
    numEntries += cache.vals[tid].size();
    bytes += cache.valSizes[tid];
  }
  return {numEntries, bytes};
}

// _____________________________________________________________________________
std::pair<size_t, size_t> GeometryCacheManager::size() const {
  auto cacheSizePoint = size(_pointCache);
  auto cacheSizeArea = size(_areaCache);
  auto cacheSizeSimpleArea = size(_simpleAreaCache);
  auto cacheSizeSimpleLine = size(_simpleLineCache);
  auto cacheSizeLine = size(_lineCache);

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
}
