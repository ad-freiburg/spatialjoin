// Copyright 2024, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_STATS_H_
#define SPATIALJOINS_STATS_H_

namespace sj {
struct Stats {
  uint64_t timeGeoCacheRetrievalArea = 0;
  uint64_t timeGeoCacheRetrievalLine = 0;

  uint64_t timeWrite = 0;

  uint64_t timeBoxIdIsectAreaArea = 0;
  uint64_t timeBoxIdIsectAreaLine = 0;
  uint64_t timeBoxIdIsectAreaPoint = 0;
  uint64_t timeBoxIdIsectLineLine = 0;

  uint64_t timeOBBIsectAreaArea = 0;
  uint64_t timeOBBIsectAreaLine = 0;
  uint64_t timeOBBIsectAreaPoint = 0;
  uint64_t timeOBBIsectLineLine = 0;

  uint64_t timeFullGeoCheckAreaArea = 0;
  uint64_t timeFullGeoCheckAreaLine = 0;
  uint64_t timeFullGeoCheckAreaPoint = 0;
  uint64_t timeFullGeoCheckLineLine = 0;

  size_t fullGeoChecksAreaArea = 0;
  size_t fullGeoChecksAreaLine = 0;
  size_t fullGeoChecksAreaPoint = 0;
  size_t fullGeoChecksLineLine = 0;

  std::string toString();
};

inline std::string Stats::toString() {
  double sum =
      double(timeGeoCacheRetrievalArea + timeGeoCacheRetrievalLine + timeWrite +
             timeBoxIdIsectAreaArea + timeBoxIdIsectAreaLine +
             timeBoxIdIsectAreaPoint + timeBoxIdIsectLineLine +
	     timeOBBIsectAreaArea + timeOBBIsectAreaLine +
	     timeOBBIsectAreaPoint + timeOBBIsectLineLine +
             timeFullGeoCheckAreaArea + timeFullGeoCheckAreaLine +
             timeFullGeoCheckAreaPoint + timeFullGeoCheckLineLine) /
      1000000000.0;

  std::stringstream ss;

  double t = double(timeGeoCacheRetrievalArea) / 1000000000.0;
  ss << "time for geo cache retrievel of AREAS: " << t << " s ("
     << ((t / sum) * 100.0) << "%)\n";

  t = double(timeGeoCacheRetrievalLine) / 1000000000.0;
  ss << "time for geo cache retrievel of LINES: " << t << " s ("
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

  t = double(timeWrite) / 1000000000.0;
  ss << "time for output writing: " << t << " s (" << ((t / sum) * 100.0)
     << "%)\n";

  ss << "\n    SUM: " << sum << " s\n";
  return ss.str();
}

inline Stats operator+(const Stats& a, const Stats& b) {
  return Stats{a.timeGeoCacheRetrievalArea + b.timeGeoCacheRetrievalArea,
               a.timeGeoCacheRetrievalLine + b.timeGeoCacheRetrievalLine,
               a.timeWrite + b.timeWrite,
               a.timeBoxIdIsectAreaArea + b.timeBoxIdIsectAreaArea,
               a.timeBoxIdIsectAreaLine + b.timeBoxIdIsectAreaLine,
               a.timeBoxIdIsectAreaPoint + b.timeBoxIdIsectAreaPoint,
               a.timeBoxIdIsectLineLine + b.timeBoxIdIsectLineLine,
               a.timeOBBIsectAreaArea + b.timeOBBIsectAreaArea,
               a.timeOBBIsectAreaLine + b.timeOBBIsectAreaLine,
               a.timeOBBIsectAreaPoint + b.timeOBBIsectAreaPoint,
               a.timeOBBIsectLineLine + b.timeOBBIsectLineLine,
               a.timeFullGeoCheckAreaArea + b.timeFullGeoCheckAreaArea,
               a.timeFullGeoCheckAreaLine + b.timeFullGeoCheckAreaLine,
               a.timeFullGeoCheckAreaPoint + b.timeFullGeoCheckAreaPoint,
               a.timeFullGeoCheckLineLine + b.timeFullGeoCheckLineLine,
               a.fullGeoChecksAreaArea + b.fullGeoChecksAreaArea,
               a.fullGeoChecksAreaLine + b.fullGeoChecksAreaLine,
               a.fullGeoChecksAreaPoint + b.fullGeoChecksAreaPoint,
               a.fullGeoChecksLineLine + b.fullGeoChecksLineLine};
}

inline void operator+=(Stats& a, const Stats& b) { a = a + b; }
}  // namespace sj

#endif
