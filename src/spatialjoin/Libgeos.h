// Copyright 2025, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_LIBGEOS_H_
#define SPATIALJOINS_LIBGEOS_H_

#include <geos_c.h>

#include "util/geo/Geo.h"

using util::geo::I32Line;
using util::geo::I32Point;
using util::geo::I32Polygon;
using util::geo::Ring;

namespace sj {

// _____________________________________________________________________________
inline GEOSGeometry* makeGeosPoint(const I32Point& p) {
  double coordBuf[2] = {p.getX(), p.getY()};

  auto seq = GEOSCoordSeq_copyFromBuffer(coordBuf, 1, 0, 0);

  return GEOSGeom_createPoint(seq);
}

// _____________________________________________________________________________
inline GEOSGeometry* makeGeosLinestring(const I32Line& line) {
  double* coordBuf = new double[line.size() * 2];
  for (size_t i = 0; i < line.size(); i++) {
    coordBuf[i * 2] = line[i].getX();
    coordBuf[i * 2 + 1] = line[i].getY();
  }

  auto seq = GEOSCoordSeq_copyFromBuffer(coordBuf, line.size(), 0, 0);
  delete[] coordBuf;

  return GEOSGeom_createLineString(seq);
}

// _____________________________________________________________________________
inline GEOSGeometry* makeGeosPolygon(const Ring<int32_t>& ring) {
  double* coordBufOuter = new double[ring.size() * 2];
  for (size_t i = 0; i < ring.size(); i++) {
    coordBufOuter[i * 2] = ring[i].getX();
    coordBufOuter[i * 2 + 1] = ring[i].getY();
  }

  auto seqOuter =
      GEOSCoordSeq_copyFromBuffer(coordBufOuter, ring.size(), 0, 0);
  delete[] coordBufOuter;
  auto outer = GEOSGeom_createLinearRing(seqOuter);

  GEOSGeometry** innerRings = new GEOSGeometry*[0];

  auto geosPoly =
      GEOSGeom_createPolygon(outer, innerRings, 0);

  delete[] innerRings;
  return geosPoly;
}

// _____________________________________________________________________________
inline GEOSGeometry* makeGeosPolygon(const I32Polygon& poly) {
  if (poly.getOuter().size() == 0) return GEOSGeom_createEmptyPolygon();

  double* coordBufOuter = new double[(poly.getOuter().size() + 1) * 2];
  for (size_t i = 0; i < poly.getOuter().size(); i++) {
    coordBufOuter[i * 2] = poly.getOuter()[i].getX();
    coordBufOuter[i * 2 + 1] = poly.getOuter()[i].getY();
  }

  coordBufOuter[poly.getOuter().size() * 2] = poly.getOuter()[0].getX();
  coordBufOuter[poly.getOuter().size() * 2 + 1] = poly.getOuter()[0].getY();

  auto seqOuter =
      GEOSCoordSeq_copyFromBuffer(coordBufOuter, poly.getOuter().size(), 0, 0);

  delete[] coordBufOuter;
  auto outer = GEOSGeom_createLinearRing(seqOuter);

  GEOSGeometry** innerRings = new GEOSGeometry*[poly.getInners().size()];

  for (size_t j = 0; j < poly.getInners().size(); j++) {
    double* coordBuf = new double[poly.getInners()[j].size() * 2];
    for (size_t i = 0; i < poly.getInners()[j].size(); i++) {
      coordBuf[i * 2] = poly.getInners()[j][i].getX();
      coordBuf[i * 2 + 1] = poly.getInners()[j][i].getY();
    }
    auto seqInner =
        GEOSCoordSeq_copyFromBuffer(coordBuf, poly.getInners()[j].size(), 0, 0);
    delete[] coordBuf;
    innerRings[j] = GEOSGeom_createLinearRing(seqInner);
  }

  auto geosPoly =
      GEOSGeom_createPolygon(outer, innerRings, poly.getInners().size());
  delete[] innerRings;
  return geosPoly;
}

class GEOSLineString {
 public:
  GEOSLineString() : _geom(GEOSGeom_createEmptyLineString()) {};
  GEOSLineString(GEOSGeometry* geom) : _geom(geom) {};
  GEOSLineString(const GEOSLineString& other) {
    _geom = GEOSGeom_clone(other._geom);
  }
  explicit GEOSLineString(const I32Line& line) : _geom(makeGeosLinestring(line)) {}
  ~GEOSLineString() {
    GEOSGeom_destroy(_geom);
  }

  GEOSLineString& operator=(const GEOSLineString& other) {
    if (other._geom) _geom = GEOSGeom_clone(other._geom);
    else _geom = 0;
    return *this;
  }
  GEOSLineString& operator=(GEOSLineString&& other) {
    _geom = other._geom;
    other._geom = 0;
    return *this;
  }

  GEOSGeometry* getGEOSGeom() const { return _geom; }

 private:
  GEOSGeometry* _geom;
};

class GEOSPolygon {
 public:
  GEOSPolygon() : _geom(GEOSGeom_createEmptyPolygon()) {};
  GEOSPolygon(GEOSGeometry* geom) : _geom(geom) {};
  GEOSPolygon(const GEOSPolygon& other) {
    _geom = GEOSGeom_clone(other._geom);
  }
  GEOSPolygon(GEOSPolygon&& other) : _geom(other._geom) {
    other._geom = 0;
  };
  explicit GEOSPolygon(const I32Polygon& poly) : _geom(makeGeosPolygon(poly)) {
  }
  explicit GEOSPolygon(const Ring<int32_t>& ring) : _geom(makeGeosPolygon(ring)) {}
  ~GEOSPolygon() {
    GEOSGeom_destroy(_geom);
  }

  GEOSPolygon& operator=(const GEOSPolygon& other) {
    if (other._geom) _geom = GEOSGeom_clone(other._geom);
    else _geom = 0;
    return *this;
  }
  GEOSPolygon& operator=(GEOSPolygon&& other) {
    _geom = other._geom;
    other._geom = 0;
    return *this;
  }

  GEOSGeometry* getGEOSGeom() const { return _geom; }
  const GEOSGeometry* getRing() const { return GEOSGetExteriorRing(_geom); }

 private:
  GEOSGeometry* _geom;
};

}  // namespace sj

#endif  // SPATIALJOINS_LIBGEOS_H_
