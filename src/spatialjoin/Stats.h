// Copyright 2024, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_STATS_H_
#define SPATIALJOINS_STATS_H_

namespace sj {
struct Stats {
  uint64_t timeGeoCacheRetrievalArea = 0;
  uint64_t timeGeoCacheRetrievalLine = 0;
  uint64_t timeGeoCacheRetrievalSimpleLine = 0;
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

  uint64_t timeInnerOuterCheckAreaArea = 0;
  uint64_t timeInnerOuterCheckAreaLine = 0;
  uint64_t timeInnerOuterCheckAreaPoint = 0;

  size_t fullGeoChecksAreaArea = 0;
  size_t fullGeoChecksAreaLine = 0;
  size_t fullGeoChecksAreaPoint = 0;
  size_t fullGeoChecksLineLine = 0;
  size_t fullGeoChecksLinePoint = 0;

  size_t innerOuterChecksAreaArea = 0;
  size_t innerOuterChecksAreaLine = 0;
  size_t innerOuterChecksAreaPoint = 0;

  size_t totalComps = 0;

  double areaSizeSum = 0;
  size_t areaCmps = 0;

  double lineLenSum = 0;
  size_t lineCmps = 0;

  size_t anchorSum = 0;

  std::string toString();
};

inline std::string Stats::toString() {
  double sum =
      double(timeGeoCacheRetrievalArea + timeGeoCacheRetrievalLine +
             timeGeoCacheRetrievalSimpleLine + timeGeoCacheRetrievalPoint +
             timeWrite + timeBoxIdIsectAreaArea + timeBoxIdIsectAreaLine +
             timeOBBIsectAreaArea + timeOBBIsectAreaLine +
             timeOBBIsectAreaPoint + timeOBBIsectLineLine +
             timeBoxIdIsectAreaPoint + timeBoxIdIsectLineLine +
             timeBoxIdIsectLinePoint + timeFullGeoCheckAreaArea +
             timeFullGeoCheckAreaLine + timeFullGeoCheckAreaPoint +
             timeFullGeoCheckLineLine + timeFullGeoCheckLinePoint +
             timeInnerOuterCheckAreaArea + timeInnerOuterCheckAreaLine) /
      1000000000.0;

  std::stringstream ss;

  double t = double(timeGeoCacheRetrievalArea) / 1000000000.0;
  ss << "time for geo cache retrieval of AREAS: " << t << " s ("
     << ((t / sum) * 100.0) << "%)\n";

  t = double(timeGeoCacheRetrievalSimpleLine) / 1000000000.0;
  ss << "time for geo cache retrieval of SIMPLE LINES: " << t << " s ("
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

  ss << "\n";

  ss << "    Avg. max surface area between cmps: " << std::fixed
     << (areaSizeSum / (areaCmps * 1.0)) / 100.0 << " (map units)^2\n";
  ss << "    Avg. max line length between cmps: " << std::fixed
     << (lineLenSum / (lineCmps * 1.0)) / 10.0 << " map units\n";

  ss << "    Avg. max num anchor points between cmps: " << std::fixed
     << (anchorSum * 1.0) / (totalComps * 1.0) << "\n";

  ss << "\n    SUM: " << sum << " s\n";
  ss << "    TOTAL COMPARISONS (after bbox / diag box filter): " << totalComps;
  return ss.str();
}

inline Stats operator+(const Stats& a, const Stats& b) {
  return Stats{
      a.timeGeoCacheRetrievalArea + b.timeGeoCacheRetrievalArea,
      a.timeGeoCacheRetrievalLine + b.timeGeoCacheRetrievalLine,
      a.timeGeoCacheRetrievalSimpleLine + b.timeGeoCacheRetrievalSimpleLine,
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
      a.timeInnerOuterCheckAreaArea + b.timeInnerOuterCheckAreaArea,
      a.timeInnerOuterCheckAreaLine + b.timeInnerOuterCheckAreaLine,
      a.timeInnerOuterCheckAreaPoint + b.timeInnerOuterCheckAreaPoint,
      a.fullGeoChecksAreaArea + b.fullGeoChecksAreaArea,
      a.fullGeoChecksAreaLine + b.fullGeoChecksAreaLine,
      a.fullGeoChecksAreaPoint + b.fullGeoChecksAreaPoint,
      a.fullGeoChecksLineLine + b.fullGeoChecksLineLine,
      a.fullGeoChecksLinePoint + b.fullGeoChecksLinePoint,
      a.innerOuterChecksAreaArea + b.innerOuterChecksAreaArea,
      a.innerOuterChecksAreaLine + b.innerOuterChecksAreaLine,
      a.innerOuterChecksAreaPoint + b.innerOuterChecksAreaPoint,
      a.totalComps + b.totalComps,
      a.areaSizeSum + b.areaSizeSum,
      a.areaCmps + b.areaCmps,
      a.lineLenSum + b.lineLenSum,
      a.lineCmps + b.lineCmps,
      a.anchorSum + b.anchorSum};
}

inline void operator+=(Stats& a, const Stats& b) { a = a + b; }
}  // namespace sj
   //

struct RelStats {
  size_t intersects = 0;
  size_t equals = 0;
  size_t covers = 0;
  size_t contains = 0;
  size_t overlaps = 0;
  size_t crosses = 0;
  size_t touches = 0;
  size_t de9im = 0;

  std::string toString() {
    std::stringstream ss;
    ss << intersects << " intersection, " << equals << " equals, " << covers
       << " covers, " << contains << " contains, " << overlaps << " overlaps, "
       << crosses << " crosses, " << touches << " touches, " << de9im
       << " de9im relations" << std::endl;
    return ss.str();
  }
};

inline RelStats operator+(const RelStats& a, const RelStats& b) {
  return {a.intersects + b.intersects, a.equals + b.equals,
          a.covers + b.covers,         a.contains + b.contains,
          a.overlaps + b.overlaps,     a.crosses + b.crosses,
          a.touches + b.touches,       a.de9im + b.de9im};
}

inline void operator+=(RelStats& a, const RelStats& b) { a = a + b; }

#endif
