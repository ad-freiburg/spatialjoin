// Copyright 2024, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_STATS_H_
#define SPATIALJOINS_STATS_H_

namespace sj {
struct Stats {
  uint64_t timeGeoCacheRetrievalArea = 0;
  uint64_t timeGeoCacheRetrievalLine = 0;
  uint64_t timeGeoCacheRetrievalPoint = 0;

  uint64_t timeWrite = 0;

  uint64_t timeBoxIdIsectAreaArea = 0;
  uint64_t timeBoxIdIsectAreaLine = 0;
  uint64_t timeBoxIdIsectAreaPoint = 0;
  uint64_t timeBoxIdIsectLineLine = 0;
  uint64_t timeBoxIdIsectLinePoint = 0;

  uint64_t timeOBBIsectAreaArea = 0;
  uint64_t timeOBBIsectAreaLine = 0;
  uint64_t timeOBBIsectAreaPoint = 0;
  uint64_t timeOBBIsectLineLine = 0;

  uint64_t timeFullGeoCheckAreaArea = 0;
  uint64_t timeFullGeoCheckAreaLine = 0;
  uint64_t timeFullGeoCheckAreaPoint = 0;
  uint64_t timeFullGeoCheckLineLine = 0;
  uint64_t timeFullGeoCheckLinePoint = 0;

  uint64_t timeCutoutGeoCheckAreaArea = 0;
  uint64_t timeCutoutGeoCheckAreaLine = 0;
  uint64_t timeCutoutGeoCheckAreaPoint = 0;
  uint64_t timeCutoutGeoCheckLineLine = 0;
  uint64_t timeCutoutGeoCheckLinePoint = 0;

  uint64_t timeInnerOuterCheckAreaArea = 0;
  uint64_t timeInnerOuterCheckAreaLine = 0;
  uint64_t timeInnerOuterCheckAreaPoint = 0;

  size_t fullGeoChecksAreaArea = 0;
  size_t fullGeoChecksAreaLine = 0;
  size_t fullGeoChecksAreaPoint = 0;
  size_t fullGeoChecksLineLine = 0;
  size_t fullGeoChecksLinePoint = 0;

  size_t cutoutGeoChecksAreaArea = 0;
  size_t cutoutGeoChecksAreaLine = 0;
  size_t cutoutGeoChecksAreaPoint = 0;
  size_t cutoutGeoChecksLineLine = 0;
  size_t cutoutGeoChecksLinePoint = 0;

  size_t innerOuterChecksAreaArea = 0;
  size_t innerOuterChecksAreaLine = 0;
  size_t innerOuterChecksAreaPoint = 0;

  size_t totalComps = 0;

  uint64_t timeSums[7] = {0, 0, 0, 0, 0, 0, 0};
  uint64_t timeSegLenSum[7] = {0, 0, 0, 0, 0, 0, 0};

  std::string toString();
  void timeHisto(size_t numPoints, uint64_t time);
  void timeWeightedHisto(uint64_t maxSegLen, uint64_t time);
};

inline std::string Stats::toString() {
  double sum =
      double(timeGeoCacheRetrievalArea + timeGeoCacheRetrievalLine +
             timeGeoCacheRetrievalPoint + timeWrite + timeBoxIdIsectAreaArea +
             timeBoxIdIsectAreaLine + timeOBBIsectAreaArea +
             timeOBBIsectAreaLine + timeOBBIsectAreaPoint +
             timeOBBIsectLineLine + timeBoxIdIsectAreaPoint +
             timeBoxIdIsectLineLine + timeBoxIdIsectLinePoint +
             timeFullGeoCheckAreaArea + timeFullGeoCheckAreaLine +
             timeFullGeoCheckAreaPoint + timeFullGeoCheckLineLine +
             timeFullGeoCheckLinePoint + timeCutoutGeoCheckAreaArea +
             timeCutoutGeoCheckAreaLine + timeCutoutGeoCheckAreaPoint +
             timeCutoutGeoCheckLineLine + timeCutoutGeoCheckLinePoint +
             timeInnerOuterCheckAreaArea + timeInnerOuterCheckAreaLine) /
      1000000000.0;

  std::stringstream ss;

  double t = double(timeGeoCacheRetrievalArea) / 1000000000.0;
  ss << "time for geo cache retrieval of AREAS: " << t << " s ("
     << ((t / sum) * 100.0) << "%)\n";

  t = double(timeGeoCacheRetrievalLine) / 1000000000.0;
  ss << "time for geo cache retrieval of LINES: " << t << " s ("
     << ((t / sum) * 100.0) << "%)\n";

  t = double(timeGeoCacheRetrievalPoint) / 1000000000.0;
  ss << "time for geo cache retrieval of POINTS: " << t << " s ("
     << ((t / sum) * 100.0) << "%)\n";

  t = double(timeBoxIdIsectAreaArea) / 1000000000.0;
  ss << "time for box ID intersections AREA/AREA: " << t << " s ("
     << ((t / sum) * 100.0) << "%)\n";

  t = double(timeBoxIdIsectAreaLine) / 1000000000.0;
  ss << "time for box ID intersections AREA/LINE: " << t << " s ("
     << ((t / sum) * 100.0) << "%)\n";

  t = double(timeBoxIdIsectAreaPoint) / 1000000000.0;
  ss << "time for box ID intersections AREA/POINT: " << t << " s ("
     << ((t / sum) * 100.0) << "%)\n";

  t = double(timeBoxIdIsectLineLine) / 1000000000.0;
  ss << "time for box ID intersections LINE/LINE: " << t << " s ("
     << ((t / sum) * 100.0) << "%)\n";

  t = double(timeBoxIdIsectLinePoint) / 1000000000.0;
  ss << "time for box ID intersections LINE/POINT: " << t << " s ("
     << ((t / sum) * 100.0) << "%)\n";

  t = double(timeOBBIsectAreaArea) / 1000000000.0;
  ss << "time for obb intersections AREA/AREA: " << t << " s ("
     << ((t / sum) * 100.0) << "%)\n";

  t = double(timeOBBIsectAreaLine) / 1000000000.0;
  ss << "time for obb intersections AREA/LINE: " << t << " s ("
     << ((t / sum) * 100.0) << "%)\n";

  t = double(timeOBBIsectAreaPoint) / 1000000000.0;
  ss << "time for obb intersections AREA/POINT: " << t << " s ("
     << ((t / sum) * 100.0) << "%)\n";

  t = double(timeOBBIsectLineLine) / 1000000000.0;
  ss << "time for obb intersections LINE/LINE: " << t << " s ("
     << ((t / sum) * 100.0) << "%)\n";

  t = double(timeFullGeoCheckAreaArea) / 1000000000.0;
  ss << "time for " << fullGeoChecksAreaArea
     << " full geom checks AREA/AREA: " << t << " s (" << ((t / sum) * 100.0)
     << "%)\n";

  t = double(timeFullGeoCheckAreaLine) / 1000000000.0;
  ss << "time for " << fullGeoChecksAreaLine
     << " full geom checks AREA/LINE: " << t << " s (" << ((t / sum) * 100.0)
     << "%)\n";

  t = double(timeFullGeoCheckAreaPoint) / 1000000000.0;
  ss << "time for " << fullGeoChecksAreaPoint
     << " full geom checks AREA/POINT: " << t << " s (" << ((t / sum) * 100.0)
     << "%)\n";

  t = double(timeFullGeoCheckLineLine) / 1000000000.0;
  ss << "time for " << fullGeoChecksLineLine
     << " full geom checks LINE/LINE: " << t << " s (" << ((t / sum) * 100.0)
     << "%)\n";

  t = double(timeFullGeoCheckLinePoint) / 1000000000.0;
  ss << "time for " << fullGeoChecksLinePoint
     << " full geom checks LINE/POINT: " << t << " s (" << ((t / sum) * 100.0)
     << "%)\n";

  t = double(timeCutoutGeoCheckAreaArea) / 1000000000.0;
  ss << "time for " << cutoutGeoChecksAreaArea
     << " cutout geom checks AREA/AREA: " << t << " s (" << ((t / sum) * 100.0)
     << "%)\n";

  t = double(timeCutoutGeoCheckAreaLine) / 1000000000.0;
  ss << "time for " << cutoutGeoChecksAreaLine
     << " cutout geom checks AREA/LINE: " << t << " s (" << ((t / sum) * 100.0)
     << "%)\n";

  t = double(timeCutoutGeoCheckAreaPoint) / 1000000000.0;
  ss << "time for " << cutoutGeoChecksAreaPoint
     << " cutout geom checks AREA/POINT: " << t << " s (" << ((t / sum) * 100.0)
     << "%)\n";

  t = double(timeCutoutGeoCheckLineLine) / 1000000000.0;
  ss << "time for " << cutoutGeoChecksLineLine
     << " cutout geom checks LINE/LINE: " << t << " s (" << ((t / sum) * 100.0)
     << "%)\n";

  t = double(timeCutoutGeoCheckLinePoint) / 1000000000.0;
  ss << "time for " << cutoutGeoChecksLinePoint
     << " cutout geom checks LINE/POINT: " << t << " s (" << ((t / sum) * 100.0)
     << "%)\n";

  t = double(timeInnerOuterCheckAreaArea) / 1000000000.0;
  ss << "time for " << innerOuterChecksAreaArea
     << " inner/outer checks AREA/AREA: " << t << " s (" << ((t / sum) * 100.0)
     << "%)\n";

  t = double(timeInnerOuterCheckAreaLine) / 1000000000.0;
  ss << "time for " << innerOuterChecksAreaLine
     << " inner/outer checks AREA/LINE: " << t << " s (" << ((t / sum) * 100.0)
     << "%)\n";

  t = double(timeInnerOuterCheckAreaPoint) / 1000000000.0;
  ss << "time for " << innerOuterChecksAreaPoint
     << " inner/outer checks AREA/POINT: " << t << " s (" << ((t / sum) * 100.0)
     << "%)\n";

  t = double(timeWrite) / 1000000000.0;
  ss << "time for output writing: " << t << " s (" << ((t / sum) * 100.0)
     << "%)\n";

  double histoSum = ((timeSums[0] + timeSums[1] + timeSums[2] + timeSums[3] +
                      timeSums[4] + timeSums[5] + timeSums[6]) *
                     1.0) /
                    1000000000.0;

  t = (timeSums[6] * 1.0) / 1000000000.0;
  ss << "comparisons inv. > 1000000 points on one side: " << t << " s ("
     << ((t / histoSum) * 100.0) << "%)\n";

  t = (timeSums[5] * 1.0) / 1000000000.0;
  ss << "comparisons inv. > 100000 points on one side: " << t << " s ("
     << ((t / histoSum) * 100.0) << "%)\n";

  t = (timeSums[4] * 1.0) / 1000000000.0;
  ss << "comparisons inv. > 10000 points on one side: " << t << " s ("
     << ((t / histoSum) * 100.0) << "%)\n";

  t = (timeSums[3] * 1.0) / 1000000000.0;
  ss << "comparisons inv. > 1000 points on one side: " << t << " s ("
     << ((t / histoSum) * 100.0) << "%)\n";

  t = (timeSums[2] * 1.0) / 1000000000.0;
  ss << "comparisons inv. > 100 points on one side: " << t << " s ("
     << ((t / histoSum) * 100.0) << "%)\n";

  t = (timeSums[1] * 1.0) / 1000000000.0;
  ss << "comparisons inv. > 10 points on one side: " << t << " s ("
     << ((t / histoSum) * 100.0) << "%)\n";

  t = (timeSums[0] * 1.0) / 1000000000.0;
  ss << "comparisons inv. > 1 points on one side: " << t << " s ("
     << ((t / histoSum) * 100.0) << "%)\n";


  double histoSegLenSum = ((timeSegLenSum[0] + timeSegLenSum[1] + timeSegLenSum[2] + timeSegLenSum[3] +
      timeSegLenSum[4] + timeSegLenSum[5] + timeSegLenSum[6]) *
      1.0) /
      1000000000.0;

  t = (timeSegLenSum[6] * 1.0) / 1000000000.0;
  ss << "comparisons inv. seg len > 1000000 on one side: " << t << " s ("
     << ((t / histoSegLenSum) * 100.0) << "%)\n";

  t = (timeSegLenSum[5] * 1.0) / 1000000000.0;
  ss << "comparisons inv. seg len > 100000 on one side: " << t << " s ("
     << ((t / histoSegLenSum) * 100.0) << "%)\n";

  t = (timeSegLenSum[4] * 1.0) / 1000000000.0;
  ss << "comparisons inv. seg len > 10000 on one side: " << t << " s ("
     << ((t / histoSegLenSum) * 100.0) << "%)\n";

  t = (timeSegLenSum[3] * 1.0) / 1000000000.0;
  ss << "comparisons inv. seg len > 1000 on one side: " << t << " s ("
     << ((t / histoSegLenSum) * 100.0) << "%)\n";

  t = (timeSegLenSum[2] * 1.0) / 1000000000.0;
  ss << "comparisons inv. seg len > 100 on one side: " << t << " s ("
     << ((t / histoSegLenSum) * 100.0) << "%)\n";

  t = (timeSegLenSum[1] * 1.0) / 1000000000.0;
  ss << "comparisons inv. seg len > 10 on one side: " << t << " s ("
     << ((t / histoSegLenSum) * 100.0) << "%)\n";

  t = (timeSegLenSum[0] * 1.0) / 1000000000.0;
  ss << "comparisons inv. seg len > 1 on one side: " << t << " s ("
     << ((t / histoSegLenSum) * 100.0) << "%)\n";

  ss << "\n    SUM: " << sum << " s\n";
  ss << "    TOTAL COMPARISONS: " << totalComps << "\n";
  return ss.str();
}

inline void Stats::timeHisto(size_t numPoints, uint64_t time) {
  if (numPoints > 1000000) {
    timeSums[6] += time;
    return;
  }
  if (numPoints > 100000) {
    timeSums[5] += time;
    return;
  }
  if (numPoints > 10000) {
    timeSums[4] += time;
    return;
  }
  if (numPoints > 1000) {
    timeSums[3] += time;
    return;
  }
  if (numPoints > 100) {
    timeSums[2] += time;
    return;
  }
  if (numPoints > 10) {
    timeSums[1] += time;
    return;
  }

  timeSums[0] += time;
  return;
}

inline void Stats::timeWeightedHisto(uint64_t maxSegLen, uint64_t time) {
  if (maxSegLen > 1000000) {
    timeSegLenSum[6] += time;
    return;
  }
  if (maxSegLen > 100000) {
    timeSegLenSum[5] += time;
    return;
  }
  if (maxSegLen > 10000) {
    timeSegLenSum[4] += time;
    return;
  }
  if (maxSegLen > 1000) {
    timeSegLenSum[3] += time;
    return;
  }
  if (maxSegLen > 100) {
    timeSegLenSum[2] += time;
    return;
  }
  if (maxSegLen > 10) {
    timeSegLenSum[1] += time;
    return;
  }

  timeSegLenSum[0] += time;
  return;
}

inline Stats operator+(const Stats& a, const Stats& b) {
  return Stats{a.timeGeoCacheRetrievalArea + b.timeGeoCacheRetrievalArea,
               a.timeGeoCacheRetrievalLine + b.timeGeoCacheRetrievalLine,
               a.timeGeoCacheRetrievalPoint + b.timeGeoCacheRetrievalPoint,
               a.timeWrite + b.timeWrite,
               a.timeBoxIdIsectAreaArea + b.timeBoxIdIsectAreaArea,
               a.timeBoxIdIsectAreaLine + b.timeBoxIdIsectAreaLine,
               a.timeBoxIdIsectAreaPoint + b.timeBoxIdIsectAreaPoint,
               a.timeBoxIdIsectLineLine + b.timeBoxIdIsectLineLine,
               a.timeBoxIdIsectLinePoint + b.timeBoxIdIsectLinePoint,
               a.timeOBBIsectAreaArea + b.timeOBBIsectAreaArea,
               a.timeOBBIsectAreaLine + b.timeOBBIsectAreaLine,
               a.timeOBBIsectAreaPoint + b.timeOBBIsectAreaPoint,
               a.timeOBBIsectLineLine + b.timeOBBIsectLineLine,
               a.timeFullGeoCheckAreaArea + b.timeFullGeoCheckAreaArea,
               a.timeFullGeoCheckAreaLine + b.timeFullGeoCheckAreaLine,
               a.timeFullGeoCheckAreaPoint + b.timeFullGeoCheckAreaPoint,
               a.timeFullGeoCheckLineLine + b.timeFullGeoCheckLineLine,
               a.timeFullGeoCheckLinePoint + b.timeFullGeoCheckLinePoint,
               a.timeCutoutGeoCheckAreaArea + b.timeCutoutGeoCheckAreaArea,
               a.timeCutoutGeoCheckAreaLine + b.timeCutoutGeoCheckAreaLine,
               a.timeCutoutGeoCheckAreaPoint + b.timeCutoutGeoCheckAreaPoint,
               a.timeCutoutGeoCheckLineLine + b.timeCutoutGeoCheckLineLine,
               a.timeCutoutGeoCheckLinePoint + b.timeCutoutGeoCheckLinePoint,
               a.timeInnerOuterCheckAreaArea + b.timeInnerOuterCheckAreaArea,
               a.timeInnerOuterCheckAreaLine + b.timeInnerOuterCheckAreaLine,
               a.timeInnerOuterCheckAreaPoint + b.timeInnerOuterCheckAreaPoint,
               a.fullGeoChecksAreaArea + b.fullGeoChecksAreaArea,
               a.fullGeoChecksAreaLine + b.fullGeoChecksAreaLine,
               a.fullGeoChecksAreaPoint + b.fullGeoChecksAreaPoint,
               a.fullGeoChecksLineLine + b.fullGeoChecksLineLine,
               a.fullGeoChecksLinePoint + b.fullGeoChecksLinePoint,
               a.cutoutGeoChecksAreaArea + b.cutoutGeoChecksAreaArea,
               a.cutoutGeoChecksAreaLine + b.cutoutGeoChecksAreaLine,
               a.cutoutGeoChecksAreaPoint + b.cutoutGeoChecksAreaPoint,
               a.cutoutGeoChecksLineLine + b.cutoutGeoChecksLineLine,
               a.cutoutGeoChecksLinePoint + b.cutoutGeoChecksLinePoint,
               a.innerOuterChecksAreaArea + b.innerOuterChecksAreaArea,
               a.innerOuterChecksAreaLine + b.innerOuterChecksAreaLine,
               a.innerOuterChecksAreaPoint + b.innerOuterChecksAreaPoint,
               a.totalComps + b.totalComps,
               {a.timeSums[0] + b.timeSums[0], a.timeSums[1] + b.timeSums[1],
                a.timeSums[2] + b.timeSums[2], a.timeSums[3] + b.timeSums[3],
                a.timeSums[4] + b.timeSums[4], a.timeSums[5] + b.timeSums[5],
                a.timeSums[6] + b.timeSums[6]},
               {a.timeSegLenSum[0] + b.timeSegLenSum[0], a.timeSegLenSum[1] + b.timeSegLenSum[1],
                a.timeSegLenSum[2] + b.timeSegLenSum[2], a.timeSegLenSum[3] + b.timeSegLenSum[3],
                a.timeSegLenSum[4] + b.timeSegLenSum[4], a.timeSegLenSum[5] + b.timeSegLenSum[5],
                a.timeSegLenSum[6] + b.timeSegLenSum[6]}};
}

inline void operator+=(Stats& a, const Stats& b) { a = a + b; }
}  // namespace sj

#endif
