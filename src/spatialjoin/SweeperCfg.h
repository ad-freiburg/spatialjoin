// Copyright 2026, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_SWEEPERCFG_H_
#define SPATIALJOINS_SWEEPERCFG_H_

#include <functional>
#include <string>

#include "util/geo/Geo.h"

namespace sj {

struct SweeperCfg {
  size_t numThreads;
  size_t numCacheThreads;
  size_t geomCacheMaxSize;
  size_t geomCacheMaxNumElements;
  std::string sepIsect;
  std::string sepContains;
  std::string sepCovers;
  std::string sepTouches;
  std::string sepEquals;
  std::string sepOverlaps;
  std::string sepCrosses;
  bool useBoxIds;
  bool useArea;
  bool useDiagBox;
  bool useFastSweepSkip;
  bool noGeometryChecks;
  double withinDist;
  bool euclideanDist;
  bool haversineApprox;
  bool computeDE9IM;
  util::geo::DE9IMFilter de9imFilter;
  bool forceTwoSided;
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
