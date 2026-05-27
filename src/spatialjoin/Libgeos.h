// Copyright 2025, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_LIBGEOS_H_
#define SPATIALJOINS_LIBGEOS_H_

#include <geos_c.h>
#include <stdarg.h>
#include <stdio.h>

#include "util/geo/Geo.h"
#include "util/log/Log.h"

using util::geo::I32Line;
using util::geo::I32Point;
using util::geo::I32Polygon;
using util::geo::LineSegment;
using util::geo::Ring;

namespace sj {

const static size_t GEOS_PREP_THRESHOLD = 100;

static void GEOSMsgHandler(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
}

// _____________________________________________________________________________
inline GEOSGeometry* makeGeosPoint(GEOSContextHandle_t geosHndl,
                                   const I32Point& p) {
  double coordBuf[2] = {static_cast<double>(p.getX()),
                        static_cast<double>(p.getY())};

  auto seq = GEOSCoordSeq_copyFromBuffer_r(geosHndl, coordBuf, 1, 0, 0);

  return GEOSGeom_createPoint_r(geosHndl, seq);
}

// _____________________________________________________________________________
inline GEOSGeometry* makeGeosLinestring(GEOSContextHandle_t geosHndl,
                                        const I32Line& line) {
  double* coordBuf = new double[line.size() * 2];
  for (size_t i = 0; i < line.size(); i++) {
    coordBuf[i * 2] = line[i].getX();
    coordBuf[i * 2 + 1] = line[i].getY();
  }

  auto seq =
      GEOSCoordSeq_copyFromBuffer_r(geosHndl, coordBuf, line.size(), 0, 0);
  delete[] coordBuf;

  return GEOSGeom_createLineString_r(geosHndl, seq);
}

// _____________________________________________________________________________
inline GEOSGeometry* makeGeosLinestring(GEOSContextHandle_t geosHndl,
                                        const LineSegment<int32_t>& ls) {
  double coordBuf[4];
  coordBuf[0] = ls.first.getX();
  coordBuf[1] = ls.first.getY();
  coordBuf[2] = ls.second.getX();
  coordBuf[3] = ls.second.getY();
  auto seq = GEOSCoordSeq_copyFromBuffer_r(geosHndl, coordBuf, 2, 0, 0);

  return GEOSGeom_createLineString_r(geosHndl, seq);
}

// _____________________________________________________________________________
inline GEOSGeometry* makeGeosPolygon(GEOSContextHandle_t geosHndl,
                                     const Ring<int32_t>& ring) {
  if (ring.size() == 0) return GEOSGeom_createEmptyPolygon_r(geosHndl);

  double* coordBufOuter = new double[(ring.size() + 1) * 2];
  for (size_t i = 0; i < ring.size(); i++) {
    coordBufOuter[i * 2] = ring[i].getX();
    coordBufOuter[i * 2 + 1] = ring[i].getY();
  }

  coordBufOuter[ring.size() * 2] = ring[0].getX();
  coordBufOuter[ring.size() * 2 + 1] = ring[0].getY();

  auto seqOuter = GEOSCoordSeq_copyFromBuffer_r(geosHndl, coordBufOuter,
                                                ring.size() + 1, 0, 0);
  delete[] coordBufOuter;
  auto outer = GEOSGeom_createLinearRing_r(geosHndl, seqOuter);

  GEOSGeometry** innerRings = new GEOSGeometry*[0];

  auto geosPoly = GEOSGeom_createPolygon_r(geosHndl, outer, innerRings, 0);

  delete[] innerRings;
  return geosPoly;
}

// _____________________________________________________________________________
inline GEOSGeometry* makeGeosPolygon(GEOSContextHandle_t geosHndl,
                                     const I32Polygon& poly) {
  if (poly.getOuter().size() == 0)
    return GEOSGeom_createEmptyPolygon_r(geosHndl);

  double* coordBufOuter = new double[(poly.getOuter().size() + 1) * 2];
  for (size_t i = 0; i < poly.getOuter().size(); i++) {
    coordBufOuter[i * 2] = poly.getOuter()[i].getX();
    coordBufOuter[i * 2 + 1] = poly.getOuter()[i].getY();
  }

  coordBufOuter[poly.getOuter().size() * 2] = poly.getOuter()[0].getX();
  coordBufOuter[poly.getOuter().size() * 2 + 1] = poly.getOuter()[0].getY();

  auto seqOuter = GEOSCoordSeq_copyFromBuffer_r(
      geosHndl, coordBufOuter, poly.getOuter().size() + 1, 0, 0);

  delete[] coordBufOuter;
  auto outer = GEOSGeom_createLinearRing_r(geosHndl, seqOuter);

  GEOSGeometry** innerRings = new GEOSGeometry*[poly.getInners().size()];
  size_t realInners = 0;

  for (size_t j = 0; j < poly.getInners().size(); j++) {
    if (poly.getInners()[j].size() == 0) continue;
    double* coordBuf = new double[(poly.getInners()[j].size() + 1) * 2];
    for (size_t i = 0; i < poly.getInners()[j].size(); i++) {
      coordBuf[i * 2] = poly.getInners()[j][i].getX();
      coordBuf[i * 2 + 1] = poly.getInners()[j][i].getY();
    }
    coordBuf[poly.getInners()[j].size() * 2] = poly.getInners()[j][0].getX();
    coordBuf[poly.getInners()[j].size() * 2 + 1] =
        poly.getInners()[j][0].getY();

    auto seqInner = GEOSCoordSeq_copyFromBuffer_r(
        geosHndl, coordBuf, poly.getInners()[j].size() + 1, 0, 0);
    delete[] coordBuf;
    innerRings[realInners] = GEOSGeom_createLinearRing_r(geosHndl, seqInner);
    realInners++;
  }

  auto geosPoly =
      GEOSGeom_createPolygon_r(geosHndl, outer, innerRings, realInners);
  delete[] innerRings;
  return geosPoly;
}

class GEOSLineString {
 public:
  GEOSLineString() : _geom(0), _prepGeom(0), _geosHndlDestroy(0), _size(0){};
  GEOSLineString(GEOSGeometry* geom, GEOSContextHandle_t destroyHndl)
      : _geom(geom), _prepGeom(0), _geosHndlDestroy(destroyHndl), _size(0){};
  GEOSLineString(const GEOSLineString& other) = delete;
  GEOSLineString(GEOSLineString&& other)
      : _geom(other._geom),
        _prepGeom(other._prepGeom),
        _geosHndlDestroy(other._geosHndlDestroy),
        _size(other._size) {
    other._geom = 0;
    other._prepGeom = 0;
    other._size = 0;
  }
  GEOSLineString(GEOSContextHandle_t geosHndl, const I32Line& line)
      : _geom(makeGeosLinestring(geosHndl, line)),
        _prepGeom(0),
        _geosHndlDestroy(geosHndl),
        _size(line.size()) {}
  GEOSLineString(GEOSContextHandle_t geosHndl, const LineSegment<int32_t>& ls)
      : _geom(makeGeosLinestring(geosHndl, ls)),
        _prepGeom(0),
        _geosHndlDestroy(geosHndl),
        _size(2) {}
  ~GEOSLineString() {
    if (_prepGeom) GEOSPreparedGeom_destroy_r(_geosHndlDestroy, _prepGeom);
    if (_geom) GEOSGeom_destroy_r(_geosHndlDestroy, _geom);
  }

  GEOSLineString& operator=(const GEOSLineString& other) = delete;
  GEOSLineString& operator=(GEOSLineString&& other) {
    _geom = other._geom;
    _prepGeom = other._prepGeom;
    _size = other._size;
    other._geom = 0;
    other._prepGeom = 0;
    other._size = 0;
    _geosHndlDestroy = other._geosHndlDestroy;
    return *this;
  }

  void prepare(GEOSContextHandle_t hndl) {
    _prepGeom = GEOSPrepare_r(hndl, _geom);
  }

  GEOSGeometry* getGEOSGeom() const { return _geom; }
  size_t getSize() const { return _size; }
  void setSize(size_t size) { _size = size; }
  const GEOSPreparedGeometry* getPrepGEOSGeom() const { return _prepGeom; }

 private:
  GEOSGeometry* _geom;
  const GEOSPreparedGeometry* _prepGeom;
  GEOSContextHandle_t _geosHndlDestroy;
  size_t _size;
};

class GEOSPolygon {
 public:
  GEOSPolygon() : _geom(0), _prepGeom(0), _geosHndlDestroy(0), _size(0){};
  GEOSPolygon(GEOSGeometry* geom, GEOSContextHandle_t destroyHndl)
      : _geom(geom), _prepGeom(0), _geosHndlDestroy(destroyHndl), _size(0){};
  GEOSPolygon(const GEOSPolygon& other) = delete;
  GEOSPolygon(GEOSPolygon&& other)
      : _geom(other._geom),
        _prepGeom(other._prepGeom),
        _geosHndlDestroy(other._geosHndlDestroy),
        _size(other._size) {
    other._geom = 0;
    other._prepGeom = 0;
    other._size = 0;
  };
  GEOSPolygon(GEOSContextHandle_t geosHndl, const I32Polygon& poly)
      : _geom(makeGeosPolygon(geosHndl, poly)),
        _prepGeom(0),
        _geosHndlDestroy(geosHndl),
        _size(0) {
    _size = poly.getSize();
  }
  GEOSPolygon(GEOSContextHandle_t geosHndl, const Ring<int32_t>& ring)
      : _geom(makeGeosPolygon(geosHndl, ring)),
        _prepGeom(0),
        _geosHndlDestroy(geosHndl),
        _size(0) {
    _size = ring.size();
  }

  ~GEOSPolygon() {
    // destroy prep'ed geom first, has internal reference to original
    if (_prepGeom) GEOSPreparedGeom_destroy_r(_geosHndlDestroy, _prepGeom);
    if (_geom) GEOSGeom_destroy_r(_geosHndlDestroy, _geom);
  }

  void prepare(GEOSContextHandle_t hndl) {
    _prepGeom = GEOSPrepare_r(hndl, _geom);
  }

  size_t getSize() const { return _size; }
  void setSize(size_t size) { _size = size; }
  const GEOSPreparedGeometry* getPrepGEOSGeom() const { return _prepGeom; }

  GEOSPolygon& operator=(const GEOSPolygon& other) = delete;
  GEOSPolygon& operator=(GEOSPolygon&& other) {
    _geom = other._geom;
    _prepGeom = other._prepGeom;
    _size = other._size;
    _geosHndlDestroy = other._geosHndlDestroy;
    other._geom = 0;
    other._prepGeom = 0;
    other._size = 0;
    return *this;
  }

  GEOSGeometry* getGEOSGeom() const { return _geom; }
  bool empty(GEOSContextHandle_t hndl) const {
    return _geom == 0 || GEOSisEmpty_r(hndl, _geom);
  }

 private:
  GEOSGeometry* _geom;
  const GEOSPreparedGeometry* _prepGeom;
  GEOSContextHandle_t _geosHndlDestroy;
  size_t _size;
};

// wrappers which may automatically select prepared geoms
inline bool GEOSContains_r(GEOSContextHandle_t h, const GEOSPolygon& a,
                           const GEOSPolygon& b) {
  if ((a.getPrepGEOSGeom() && !b.getPrepGEOSGeom()) ||
      (a.getPrepGEOSGeom() && b.getPrepGEOSGeom() &&
       a.getSize() > b.getSize())) {
    return GEOSPreparedContains_r(h, a.getPrepGEOSGeom(), b.getGEOSGeom());
  } else if ((b.getPrepGEOSGeom() && !a.getPrepGEOSGeom()) ||
             (b.getPrepGEOSGeom() && a.getPrepGEOSGeom() &&
              a.getSize() < b.getSize())) {
    return GEOSPreparedWithin_r(h, b.getPrepGEOSGeom(), a.getGEOSGeom());
  }
  return GEOSContains_r(h, b.getGEOSGeom(), a.getGEOSGeom());
}

inline bool GEOSIntersects_r(GEOSContextHandle_t h, const GEOSPolygon& a,
                             const GEOSPolygon& b) {
  if ((a.getPrepGEOSGeom() && !b.getPrepGEOSGeom()) ||
      (a.getPrepGEOSGeom() && b.getPrepGEOSGeom() &&
       a.getSize() > b.getSize())) {
    return GEOSPreparedIntersects_r(h, a.getPrepGEOSGeom(), b.getGEOSGeom());
  } else if ((b.getPrepGEOSGeom() && !a.getPrepGEOSGeom()) ||
             (b.getPrepGEOSGeom() && a.getPrepGEOSGeom() &&
              a.getSize() < b.getSize())) {
    return GEOSPreparedIntersects_r(h, b.getPrepGEOSGeom(), a.getGEOSGeom());
  }
  return GEOSIntersects_r(h, b.getGEOSGeom(), a.getGEOSGeom());
}

inline void GEOSDistance_r(GEOSContextHandle_t h,
                                          const GEOSLineString& a,
                                          const GEOSLineString& b, double* ret) {
  if ((a.getPrepGEOSGeom() && !b.getPrepGEOSGeom()) ||
      (a.getPrepGEOSGeom() && b.getPrepGEOSGeom() &&
       a.getSize() > b.getSize())) {
    GEOSPreparedDistance_r(h, a.getPrepGEOSGeom(), b.getGEOSGeom(), ret);
		return;
  } else if ((b.getPrepGEOSGeom() && !a.getPrepGEOSGeom()) ||
             (b.getPrepGEOSGeom() && a.getPrepGEOSGeom() &&
              a.getSize() < b.getSize())) {
    GEOSPreparedDistance_r(h, b.getPrepGEOSGeom(), a.getGEOSGeom(), ret);
		return;
  }

  GEOSDistance_r(h, b.getGEOSGeom(), a.getGEOSGeom(), ret);
}

inline void GEOSDistanceWithin_r(GEOSContextHandle_t h,
                                          const GEOSPolygon& a,
                                          const GEOSLineString& b, double, double* ret) {
  if ((a.getPrepGEOSGeom() && !b.getPrepGEOSGeom()) ||
      (a.getPrepGEOSGeom() && b.getPrepGEOSGeom() &&
       a.getSize() > b.getSize())) {
    GEOSPreparedDistance_r(h, a.getPrepGEOSGeom(), b.getGEOSGeom(), ret);
		return;
  } else if ((b.getPrepGEOSGeom() && !a.getPrepGEOSGeom()) ||
             (b.getPrepGEOSGeom() && a.getPrepGEOSGeom() &&
              a.getSize() < b.getSize())) {
    GEOSPreparedDistance_r(h, b.getPrepGEOSGeom(), a.getGEOSGeom(), ret);
		return;
  }

  GEOSDistance_r(h, b.getGEOSGeom(), a.getGEOSGeom(), ret);
}

inline void GEOSDistanceWithin_r(GEOSContextHandle_t h,
                                          const GEOSLineString& a,
                                          const GEOSLineString& b, double, double* ret) {
  if ((a.getPrepGEOSGeom() && !b.getPrepGEOSGeom()) ||
      (a.getPrepGEOSGeom() && b.getPrepGEOSGeom() &&
       a.getSize() > b.getSize())) {
    GEOSPreparedDistance_r(h, a.getPrepGEOSGeom(), b.getGEOSGeom(), ret);
		return;
  } else if ((b.getPrepGEOSGeom() && !a.getPrepGEOSGeom()) ||
             (b.getPrepGEOSGeom() && a.getPrepGEOSGeom() &&
              a.getSize() < b.getSize())) {
    GEOSPreparedDistance_r(h, b.getPrepGEOSGeom(), a.getGEOSGeom(), ret);
		return;
  }

  GEOSDistance_r(h, b.getGEOSGeom(), a.getGEOSGeom(), ret);
}

inline void GEOSDistanceWithin_r(GEOSContextHandle_t h,
                                          const GEOSPolygon& a,
                                          const GEOSPolygon& b, double, double* ret) {
  if ((a.getPrepGEOSGeom() && !b.getPrepGEOSGeom()) ||
      (a.getPrepGEOSGeom() && b.getPrepGEOSGeom() &&
       a.getSize() > b.getSize())) {
    GEOSPreparedDistance_r(h, a.getPrepGEOSGeom(), b.getGEOSGeom(), ret);
		return;
  } else if ((b.getPrepGEOSGeom() && !a.getPrepGEOSGeom()) ||
             (b.getPrepGEOSGeom() && a.getPrepGEOSGeom() &&
              a.getSize() < b.getSize())) {
    GEOSPreparedDistance_r(h, b.getPrepGEOSGeom(), a.getGEOSGeom(), ret);
		return;
  }

  GEOSDistance_r(h, b.getGEOSGeom(), a.getGEOSGeom(), ret);
}

inline util::geo::DE9IMatrix GEOSRelate_r(GEOSContextHandle_t h,
                                          const GEOSPolygon& a,
                                          const GEOSPolygon& b) {
#if GEOS_VERSION_MAJOR > 3 || GEOS_VERSION_MINOR > 12
  if ((a.getPrepGEOSGeom() && !b.getPrepGEOSGeom()) ||
      (a.getPrepGEOSGeom() && b.getPrepGEOSGeom() &&
       a.getSize() > b.getSize())) {
    char* de9im = GEOSPreparedRelate_r(h, a.getPrepGEOSGeom(), b.getGEOSGeom());
    auto res = util::geo::DE9IMatrix(de9im);
    GEOSFree_r(h, de9im);
    return res;
  } else if ((b.getPrepGEOSGeom() && !a.getPrepGEOSGeom()) ||
             (b.getPrepGEOSGeom() && a.getPrepGEOSGeom() &&
              a.getSize() < b.getSize())) {
    char* de9im = GEOSPreparedRelate_r(h, b.getPrepGEOSGeom(), a.getGEOSGeom());
    auto res = util::geo::DE9IMatrix(de9im);
    GEOSFree_r(h, de9im);
    return res.transpose();
  }
#endif

  char* de9im = GEOSRelate_r(h, a.getGEOSGeom(), b.getGEOSGeom());
  auto res = util::geo::DE9IMatrix(de9im);
  GEOSFree_r(h, de9im);
  return res;
}

inline bool GEOSContains_r(GEOSContextHandle_t h, const GEOSPolygon& a,
                           const GEOSLineString& b) {
  if ((a.getPrepGEOSGeom() && !b.getPrepGEOSGeom()) ||
      (a.getPrepGEOSGeom() && b.getPrepGEOSGeom() &&
       a.getSize() > b.getSize())) {
    return GEOSPreparedContains_r(h, a.getPrepGEOSGeom(), b.getGEOSGeom());
  } else if ((b.getPrepGEOSGeom() && !a.getPrepGEOSGeom()) ||
             (b.getPrepGEOSGeom() && a.getPrepGEOSGeom() &&
              a.getSize() < b.getSize())) {
    return GEOSPreparedWithin_r(h, b.getPrepGEOSGeom(), a.getGEOSGeom());
  }
  return GEOSContains_r(h, b.getGEOSGeom(), a.getGEOSGeom());
}

inline bool GEOSIntersects_r(GEOSContextHandle_t h, const GEOSLineString& a,
                             const GEOSPolygon& b) {
  if ((a.getPrepGEOSGeom() && !b.getPrepGEOSGeom()) ||
      (a.getPrepGEOSGeom() && b.getPrepGEOSGeom() &&
       a.getSize() > b.getSize())) {
    return GEOSPreparedIntersects_r(h, a.getPrepGEOSGeom(), b.getGEOSGeom());
  } else if ((b.getPrepGEOSGeom() && !a.getPrepGEOSGeom()) ||
             (b.getPrepGEOSGeom() && a.getPrepGEOSGeom() &&
              a.getSize() < b.getSize())) {
    return GEOSPreparedIntersects_r(h, b.getPrepGEOSGeom(), a.getGEOSGeom());
  }
  return GEOSIntersects_r(h, b.getGEOSGeom(), a.getGEOSGeom());
}

inline util::geo::DE9IMatrix GEOSRelate_r(GEOSContextHandle_t h,
                                          const GEOSLineString& a,
                                          const GEOSPolygon& b) {
#if GEOS_VERSION_MAJOR > 3 || GEOS_VERSION_MINOR > 12
  if ((a.getPrepGEOSGeom() && !b.getPrepGEOSGeom()) ||
      (a.getPrepGEOSGeom() && b.getPrepGEOSGeom() &&
       a.getSize() > b.getSize())) {
    char* de9im = GEOSPreparedRelate_r(h, a.getPrepGEOSGeom(), b.getGEOSGeom());
    auto res = util::geo::DE9IMatrix(de9im);
    GEOSFree_r(h, de9im);
    return res;
  } else if ((b.getPrepGEOSGeom() && !a.getPrepGEOSGeom()) ||
             (b.getPrepGEOSGeom() && a.getPrepGEOSGeom() &&
              a.getSize() < b.getSize())) {
    char* de9im = GEOSPreparedRelate_r(h, b.getPrepGEOSGeom(), a.getGEOSGeom());
    auto res = util::geo::DE9IMatrix(de9im);
    GEOSFree_r(h, de9im);
    return res.transpose();
  }
#endif

  char* de9im = GEOSRelate_r(h, a.getGEOSGeom(), b.getGEOSGeom());
  auto res = util::geo::DE9IMatrix(de9im);
  GEOSFree_r(h, de9im);
  return res;
}

inline bool GEOSIntersects_r(GEOSContextHandle_t h, const GEOSLineString& a,
                             const GEOSLineString& b) {
  if ((a.getPrepGEOSGeom() && !b.getPrepGEOSGeom()) ||
      (a.getPrepGEOSGeom() && b.getPrepGEOSGeom() &&
       a.getSize() > b.getSize())) {
    return GEOSPreparedIntersects_r(h, a.getPrepGEOSGeom(), b.getGEOSGeom());
  } else if ((b.getPrepGEOSGeom() && !a.getPrepGEOSGeom()) ||
             (b.getPrepGEOSGeom() && a.getPrepGEOSGeom() &&
              a.getSize() < b.getSize())) {
    return GEOSPreparedIntersects_r(h, b.getPrepGEOSGeom(), a.getGEOSGeom());
  }
  return GEOSIntersects_r(h, b.getGEOSGeom(), a.getGEOSGeom());
}

inline bool GEOSContains_r(GEOSContextHandle_t h, const GEOSLineString& a,
                           const GEOSLineString& b) {
  if ((a.getPrepGEOSGeom() && !b.getPrepGEOSGeom()) ||
      (a.getPrepGEOSGeom() && b.getPrepGEOSGeom() &&
       a.getSize() > b.getSize())) {
    return GEOSPreparedContains_r(h, a.getPrepGEOSGeom(), b.getGEOSGeom());
  } else if ((b.getPrepGEOSGeom() && !a.getPrepGEOSGeom()) ||
             (b.getPrepGEOSGeom() && a.getPrepGEOSGeom() &&
              a.getSize() < b.getSize())) {
    return GEOSPreparedWithin_r(h, b.getPrepGEOSGeom(), a.getGEOSGeom());
  }
  return GEOSContains_r(h, b.getGEOSGeom(), a.getGEOSGeom());
}

inline util::geo::DE9IMatrix GEOSRelate_r(GEOSContextHandle_t h,
                                          const GEOSLineString& a,
                                          const GEOSLineString& b) {
#if GEOS_VERSION_MAJOR > 3 || GEOS_VERSION_MINOR > 12
  if ((a.getPrepGEOSGeom() && !b.getPrepGEOSGeom()) ||
      (a.getPrepGEOSGeom() && b.getPrepGEOSGeom() &&
       a.getSize() > b.getSize())) {
    char* de9im = GEOSPreparedRelate_r(h, a.getPrepGEOSGeom(), b.getGEOSGeom());
    auto res = util::geo::DE9IMatrix(de9im);
    GEOSFree_r(h, de9im);
    return res;
  } else if ((b.getPrepGEOSGeom() && !a.getPrepGEOSGeom()) ||
             (b.getPrepGEOSGeom() && a.getPrepGEOSGeom() &&
              a.getSize() < b.getSize())) {
    char* de9im = GEOSPreparedRelate_r(h, b.getPrepGEOSGeom(), a.getGEOSGeom());
    auto res = util::geo::DE9IMatrix(de9im);
    GEOSFree_r(h, de9im);
    return res.transpose();
  }
#endif

  char* de9im = GEOSRelate_r(h, a.getGEOSGeom(), b.getGEOSGeom());
  auto res = util::geo::DE9IMatrix(de9im);
  GEOSFree_r(h, de9im);
  return res;
}

inline util::geo::DE9IMatrix GEOSRelate_r(GEOSContextHandle_t h,
                                          const I32Point& p,
                                          const GEOSLineString& b) {
  auto point = makeGeosPoint(h, p);
  char* de9im;
#if GEOS_VERSION_MAJOR > 3 || GEOS_VERSION_MINOR > 12
  if (b.getPrepGEOSGeom())
    de9im = GEOSPreparedRelate_r(h, b.getPrepGEOSGeom(), point);
  else
    de9im = GEOSRelate_r(h, b.getGEOSGeom(), point);
#else
  de9im = GEOSRelate_r(h, b.getGEOSGeom(), point);
#endif
  auto res = util::geo::DE9IMatrix(de9im);
  GEOSFree_r(h, de9im);
  GEOSGeom_destroy_r(h, point);
  return res.transpose();
}

inline bool GEOSContains_r(GEOSContextHandle_t h, const GEOSPolygon& b,
                           const I32Point& p) {
  auto point = makeGeosPoint(h, p);
  bool res;
  if (b.getPrepGEOSGeom())
    res = GEOSPreparedContains_r(h, b.getPrepGEOSGeom(), point);
  else
    res = GEOSContains_r(h, b.getGEOSGeom(), point);
  GEOSGeom_destroy_r(h, point);
  return res;
}

inline void GEOSDistanceWithin_r(GEOSContextHandle_t h,
                                          const I32Point& p,
                                          const GEOSPolygon& b, double, double* ret) {
  auto point = makeGeosPoint(h, p);
  if (b.getPrepGEOSGeom())
    GEOSPreparedDistance_r(h, b.getPrepGEOSGeom(), point, ret);
  else
    GEOSDistance_r(h, b.getGEOSGeom(), point, ret);
  GEOSGeom_destroy_r(h, point);
}

inline void GEOSDistanceWithin_r(GEOSContextHandle_t h,
                                          const I32Point& p,
                                          const GEOSLineString& b, double, double* ret) {
  auto point = makeGeosPoint(h, p);
  if (b.getPrepGEOSGeom())
    GEOSPreparedDistance_r(h, b.getPrepGEOSGeom(), point, ret);
  else
    GEOSDistance_r(h, b.getGEOSGeom(), point, ret);
  GEOSGeom_destroy_r(h, point);
}

inline util::geo::DE9IMatrix GEOSRelate_r(GEOSContextHandle_t h,
                                          const I32Point& p,
                                          const GEOSPolygon& b) {
  auto point = makeGeosPoint(h, p);
  char* de9im;
#if GEOS_VERSION_MAJOR > 3 || GEOS_VERSION_MINOR > 12
  if (b.getPrepGEOSGeom())
    de9im = GEOSPreparedRelate_r(h, b.getPrepGEOSGeom(), point);
  else
    de9im = GEOSRelate_r(h, b.getGEOSGeom(), point);
#else
  de9im = GEOSRelate_r(h, b.getGEOSGeom(), point);
#endif
  auto res = util::geo::DE9IMatrix(de9im);
  GEOSFree_r(h, de9im);
  GEOSGeom_destroy_r(h, point);
  return res.transpose();
}
}  // namespace sj

#endif  // SPATIALJOINS_LIBGEOS_H_
