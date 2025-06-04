// Copyright 2025, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_LIBGEOS_H_
#define SPATIALJOINS_LIBGEOS_H_

#include <stdarg.h>
#include <stdio.h>
#include <geos_c.h>

#include "util/geo/Geo.h"
#include "util/log/Log.h"

using util::geo::I32Line;
using util::geo::I32Point;
using util::geo::I32Polygon;
using util::geo::LineSegment;
using util::geo::Ring;

namespace sj {

static void GEOSMsgHandler(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
}

// _____________________________________________________________________________
inline GEOSGeometry* makeGeosPoint(GEOSContextHandle_t geosHndl,
                                   const I32Point& p) {
  double coordBuf[2] = {p.getX(), p.getY()};

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
  double* coordBufOuter = new double[ring.size() * 2];
  for (size_t i = 0; i < ring.size(); i++) {
    coordBufOuter[i * 2] = ring[i].getX();
    coordBufOuter[i * 2 + 1] = ring[i].getY();
  }

  auto seqOuter =
      GEOSCoordSeq_copyFromBuffer_r(geosHndl, coordBufOuter, ring.size(), 0, 0);
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

  auto seqOuter = GEOSCoordSeq_copyFromBuffer_r(geosHndl, coordBufOuter,
                                                poly.getOuter().size(), 0, 0);

  delete[] coordBufOuter;
  auto outer = GEOSGeom_createLinearRing_r(geosHndl, seqOuter);

  GEOSGeometry** innerRings = new GEOSGeometry*[poly.getInners().size()];

  for (size_t j = 0; j < poly.getInners().size(); j++) {
    double* coordBuf = new double[poly.getInners()[j].size() * 2];
    for (size_t i = 0; i < poly.getInners()[j].size(); i++) {
      coordBuf[i * 2] = poly.getInners()[j][i].getX();
      coordBuf[i * 2 + 1] = poly.getInners()[j][i].getY();
    }
    auto seqInner = GEOSCoordSeq_copyFromBuffer_r(
        geosHndl, coordBuf, poly.getInners()[j].size(), 0, 0);
    delete[] coordBuf;
    innerRings[j] = GEOSGeom_createLinearRing_r(geosHndl, seqInner);
  }

  auto geosPoly = GEOSGeom_createPolygon_r(geosHndl, outer, innerRings,
                                           poly.getInners().size());
  delete[] innerRings;
  return geosPoly;
}

class GEOSLineString {
 public:
  GEOSLineString() : _geom(0){};
  GEOSLineString(GEOSGeometry* geom, GEOSContextHandle_t destroyHndl)
      : _geom(geom), _geosHndlDestroy(destroyHndl){};
  GEOSLineString(const GEOSLineString& other) = delete;
  GEOSLineString(GEOSLineString&& other)
      : _geom(other._geom), _geosHndlDestroy(other._geosHndlDestroy) {
    other._geom = 0;
  }
  GEOSLineString(GEOSContextHandle_t geosHndl, const I32Line& line)
      : _geom(makeGeosLinestring(geosHndl, line)), _geosHndlDestroy(geosHndl) {}
  GEOSLineString(GEOSContextHandle_t geosHndl, const LineSegment<int32_t>& ls)
      : _geom(makeGeosLinestring(geosHndl, ls)) , _geosHndlDestroy(geosHndl){}
  ~GEOSLineString() { GEOSGeom_destroy_r(_geosHndlDestroy, _geom); }

  GEOSLineString& operator=(const GEOSLineString& other) = delete;
  GEOSLineString& operator=(GEOSLineString&& other) {
    _geom = other._geom;
    other._geom = 0;
    _geosHndlDestroy = other._geosHndlDestroy;
    return *this;
  }

  GEOSGeometry* getGEOSGeom() const { return _geom; }

 private:
  GEOSGeometry* _geom;
  GEOSContextHandle_t _geosHndlDestroy;
};

class GEOSPolygon {
 public:
  GEOSPolygon() : _geom(0), _geosHndlDestroy(0){};
  GEOSPolygon(GEOSGeometry* geom, GEOSContextHandle_t destroyHndl)
      : _geom(geom), _geosHndlDestroy(destroyHndl){};
  GEOSPolygon(const GEOSPolygon& other) = delete;
  GEOSPolygon(GEOSPolygon&& other)
      : _geom(other._geom), _geosHndlDestroy(other._geosHndlDestroy) {
    other._geom = 0;
  };
  GEOSPolygon(GEOSContextHandle_t geosHndl, const I32Polygon& poly)
      : _geom(makeGeosPolygon(geosHndl, poly)), _geosHndlDestroy(geosHndl) {}
  GEOSPolygon(GEOSContextHandle_t geosHndl, const Ring<int32_t>& ring)
      : _geom(makeGeosPolygon(geosHndl, ring)), _geosHndlDestroy(geosHndl) {}

  ~GEOSPolygon() { GEOSGeom_destroy_r(_geosHndlDestroy, _geom); }

  GEOSPolygon& operator=(const GEOSPolygon& other) = delete;
  GEOSPolygon& operator=(GEOSPolygon&& other) {
    _geom = other._geom;
    _geosHndlDestroy = other._geosHndlDestroy;
    other._geom = 0;
    return *this;
  }

  GEOSGeometry* getGEOSGeom() const { return _geom; }

 private:
  GEOSGeometry* _geom;
  GEOSContextHandle_t _geosHndlDestroy;
};

}  // namespace sj

#endif  // SPATIALJOINS_LIBGEOS_H_
