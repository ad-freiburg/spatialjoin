// Copyright 2026, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_SPATIALJOINCFG_H_
#define SPATIALJOINS_SPATIALJOINCFG_H_

#include <functional>
#include <string>
#include <thread>

#include "util/geo/Geo.h"

namespace sj {

// the defaults are those of the spatialjoin command line tool
struct SpatialJoinCfg {
  size_t numThreads = std::thread::hardware_concurrency();
  size_t numCacheBuckets = std::thread::hardware_concurrency();
  size_t geomCacheMaxSize = 100 * 1000 * 1000;  // in bytes
  size_t geomCacheMaxNumElements = 10000;
  std::string sepIsect = " intersects ";
  std::string sepContains = " contains ";
  std::string sepCovers = " covers ";
  std::string sepTouches = " touches ";
  std::string sepEquals = " equals ";
  std::string sepOverlaps = " overlaps ";
  std::string sepCrosses = " crosses ";
  bool useBoxIds = true;
  bool useDiagBox = true;
  bool noGeometryChecks = false;
  double withinDist = -1;  // < 0: no distance join
  bool euclideanDist = false;
  bool haversineApprox = false;
  bool computeDE9IM = false;
  util::geo::DE9IMFilter de9imFilter;
  std::function<void(size_t t, const char* a, size_t an, const char* b,
                     size_t bn, const char* pred, size_t predn)>
      writeRelCb;
  std::function<void(const std::string&)> logCb;
  std::function<void(const std::string&)> statsCb;
  std::function<void(size_t)> sweepProgressCb;
  std::function<void()> sweepCancellationCb;
};

}  // namespace sj

#endif
