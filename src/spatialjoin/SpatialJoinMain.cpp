// Copyright 2023, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#include <iostream>

#include "BoxIds.h"
#include "Sweeper.h"
#include "util/Misc.h"
#include "util/geo/Geo.h"
#include "util/http/Server.h"
#include "util/log/Log.h"

using sj::Sweeper;
using util::geo::DLine;
using util::geo::DPoint;
using util::geo::I32Line;
using util::geo::I32MultiPolygon;
using util::geo::I32Point;
using util::geo::I32Polygon;

// _____________________________________________________________________________
void printHelp(int argc, char** argv) {
  UNUSED(argc);
  std::cout << "Usage: " << argv[0] << " [--help] [-h] <input>"
            << "\n";
}

// _____________________________________________________________________________
util::geo::I32Line parseLineString(const std::string& a, size_t p) {
  util::geo::I32Line line;
  line.reserve(2);
  auto end = memchr(a.c_str() + p, ')', a.size() - p);
  assert(end);

  while (true) {
    while (*(a.c_str() + p) && isspace(*(a.c_str() + p))) p++;
    double x = util::atof(a.c_str() + p, 10);
    double y = util::atof(
        static_cast<const char*>(memchr(a.c_str() + p, ' ', a.size() - p)) + 1,
        10);
    auto projPoint = latLngToWebMerc(DPoint(x, y));

    line.push_back(I32Point{projPoint.getX() * PREC, projPoint.getY() * PREC});

    auto n = memchr(a.c_str() + p, ',', a.size() - p);
    if (!n || n > end) break;
    p = static_cast<const char*>(n) - a.c_str() + 1;
  }

  return util::geo::simplify(line, 0);
}

// _____________________________________________________________________________
util::geo::I32Point parsePoint(const std::string& a, size_t p) {
  auto point = latLngToWebMerc(DPoint(
      util::atof(a.c_str() + p, 10),
      util::atof(
          static_cast<const char*>(memchr(a.c_str() + p, ' ', a.size() - p)) +
              1,
          10)));

  return {point.getX() * PREC, point.getY() * PREC};
}

// _____________________________________________________________________________
void parse(const char* c, size_t size, std::string& dangling, size_t* gid,
           Sweeper& idx) {
  const char* start = c;
  while (c < start + size) {
    if (*c == '\t' || *c == '\n') {
      (*gid)++;

      auto p = dangling.rfind("POINT(", 2);

      if (p != std::string::npos) {
        p += 6;
        auto point = parsePoint(dangling, p);
        idx.add(point, *gid);
      } else if ((p = dangling.rfind("LINESTRING(", 2)) !=

                 std::string::npos) {
        p += 11;
        const auto& line = parseLineString(dangling, p);
        if (line.size() != 0) {
          idx.add(line, *gid);
        }
      } else if ((p = dangling.rfind("MULTILINESTRING(", 2)) !=
                 std::string::npos) {
        p += 16;
        size_t i = 0;
        while ((p = dangling.find("(", p + 1)) != std::string::npos) {
          const auto& line = parseLineString(dangling, p + 1);
          if (line.size() != 0) {
            // TODO, i is the line number
          }
          i++;
        }
      } else if ((p = dangling.rfind("POLYGON(", 2)) != std::string::npos) {
        p += 7;
        size_t i = 0;
        I32Polygon poly;
        while ((p = dangling.find("(", p + 1)) != std::string::npos) {
          const auto& line = parseLineString(dangling, p + 1);
          if (i == 0) {
            // outer
            poly.getOuter() = line;
          } else {
            poly.getInners().push_back(line);
          }
          i++;
        }
        idx.add(poly, *gid);
      } else if ((p = dangling.rfind("MULTIPOLYGON(", 2)) !=
                 std::string::npos) {
        p += 12;
        size_t i = 0;
        I32MultiPolygon mp;
        while (p != std::string::npos &&
               (p = dangling.find("(", p + 1)) != std::string::npos) {
          I32Polygon poly;
          while ((p = dangling.find("(", p + 1)) != std::string::npos) {
            const auto& line = parseLineString(dangling, p + 1);
            if (i == 0) {
              // outer
              poly.getOuter() = line;
            } else {
              poly.getInners().push_back(line);
            }
            i++;
          }
          mp.push_back(poly);
        }
        idx.add(mp, *gid);
      }

      dangling.clear();
      c++;
      continue;
    }

    dangling += toupper(*c);

    c++;
  }
}

// _____________________________________________________________________________
int main(int argc, char** argv) {
  // disable output buffering for standard output
  setbuf(stdout, NULL);

  // initialize randomness
  srand(time(NULL) + rand());  // NOLINT

  bool useCache = false;

  for (int i = 1; i < argc; i++) {
    std::string cur = argv[i];
    if (cur == "-h" || cur == "--help") {
      printHelp(argc, argv);
      exit(0);
    }
    if (cur == "-c") {
      useCache = true;
    }
  }

  char* buf = new char[1024 * 1024 * 100];

  size_t len;

  std::string dangling;

  size_t NUM_THREADS = std::thread::hardware_concurrency();

  Sweeper sweeper(
      std::max(1, (int)NUM_THREADS - std::max(2, (int)NUM_THREADS / 4)),
      std::max(2, (int)NUM_THREADS / 4), "", " intersects ", " contains ", "\n",
      useCache);

  size_t gid = 0;

  if (!useCache) {
    LOG(INFO) << "Parsing input geometries...";

    while ((len = read(0, buf, 1024 * 1024 * 100)) > 0) {
      parse(buf, len, dangling, &gid, sweeper);
    }

    sweeper.flush();
  }

  LOG(INFO) << "done.";

  LOG(INFO) << "Sweeping...";
  sweeper.sweepLine();
  LOG(INFO) << "done.";
}
