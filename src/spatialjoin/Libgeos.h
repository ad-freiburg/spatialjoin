// Copyright 2025, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_LIBGEOS_H_
#define SPATIALJOINS_LIBGEOS_H_

#include <geos_c.h>

#include "util/geo/Geo.h"

using util::geo::I32Line;
using util::geo::I32Point;
using util::geo::I32Polygon;

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
inline GEOSGeometry* makeGeosPolygon(const I32Polygon& poly) {
  double* coordBufOuter = new double[poly.getOuter().size() * 2];
  for (size_t i = 0; i < poly.getOuter().size(); i++) {
    coordBufOuter[i * 2] = poly.getOuter()[i].getX();
    coordBufOuter[i * 2 + 1] = poly.getOuter()[i].getY();
  }

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
}  // namespace sj

#endif  // SPATIALJOINS_LIBGEOS_H_
