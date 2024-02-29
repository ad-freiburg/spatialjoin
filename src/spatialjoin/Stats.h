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

  uint64_t timeFullGeoCheckAreaArea = 0;
  uint64_t timeFullGeoCheckAreaLine = 0;
  uint64_t timeFullGeoCheckAreaPoint = 0;
  uint64_t timeFullGeoCheckLineLine = 0;

  std::string toString();
};

inline std::string Stats::toString() {
  double sum =
      double(timeGeoCacheRetrievalArea + timeGeoCacheRetrievalLine + timeWrite +
             timeBoxIdIsectAreaArea + timeBoxIdIsectAreaLine +
             timeBoxIdIsectAreaPoint + timeBoxIdIsectLineLine +
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

  t = double(timeFullGeoCheckAreaArea) / 1000000000.0;
  ss << "time for geom check AREA/AREA: " << t << " s (" << ((t / sum) * 100.0)
     << "%)\n";

  t = double(timeFullGeoCheckAreaLine) / 1000000000.0;
  ss << "time for geom check AREA/LINE: " << t << " s (" << ((t / sum) * 100.0)
     << "%)\n";

  t = double(timeFullGeoCheckAreaPoint) / 1000000000.0;
  ss << "time for geom check AREA/POINT: " << t << " s (" << ((t / sum) * 100.0)
     << "%)\n";

  t = double(timeFullGeoCheckLineLine) / 1000000000.0;
  ss << "time for geom check LINE/LINE: " << t << " s (" << ((t / sum) * 100.0)
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
               a.timeFullGeoCheckAreaArea + b.timeFullGeoCheckAreaArea,
               a.timeFullGeoCheckAreaLine + b.timeFullGeoCheckAreaLine,
               a.timeFullGeoCheckAreaPoint + b.timeFullGeoCheckAreaPoint,
               a.timeFullGeoCheckLineLine + b.timeFullGeoCheckLineLine};
}

inline void operator+=(Stats& a, const Stats& b) { a = a + b; }
}  // namespace sj

#endif
