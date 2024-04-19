#ifndef SPATIALJOINS_WKTPARSE_H_
#define SPATIALJOINS_WKTPARSE_H_

#include "Sweeper.h"
#include "util/geo/Geo.h"
#include "util/log/Log.h"

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
    auto projPoint = latLngToWebMerc(util::geo::DPoint(x, y));

    line.push_back(util::geo::I32Point{projPoint.getX() * PREC, projPoint.getY() * PREC});

    auto n = memchr(a.c_str() + p, ',', a.size() - p);
    if (!n || n > end) break;
    p = static_cast<const char*>(n) - a.c_str() + 1;
  }

  return util::geo::simplify(line, 0);
}

// _____________________________________________________________________________
util::geo::I32Point parsePoint(const std::string& a, size_t p) {
  auto point = latLngToWebMerc(util::geo::DPoint(
      util::atof(a.c_str() + p, 10),
      util::atof(
          static_cast<const char*>(memchr(a.c_str() + p, ' ', a.size() - p)) +
              1,
          10)));

  return {point.getX() * PREC, point.getY() * PREC};
}

// _____________________________________________________________________________
void parse(const char* c, size_t size, std::string& dangling, size_t* gid,
           sj::Sweeper& idx) {
  const char* start = c;
  while (c < start + size) {
    if (*c == '\n') {
      (*gid)++;

      auto idp = dangling.find("\t");

      std::string id = std::to_string(*gid);

      size_t start = 2;

      if (idp != std::string::npos) {
        id = dangling.substr(0, idp);
        start = idp + 2;
      }

      auto p = dangling.rfind("POINT(", start);

      if (p != std::string::npos) {
        p += 6;
        auto point = parsePoint(dangling, p);
        idx.add(point, id);
      } else if ((p = dangling.rfind("LINESTRING(", start)) !=

                 std::string::npos) {
        p += 11;
        const auto& line = parseLineString(dangling, p);
        if (line.size() != 0) {
          idx.add(line, id);
        }
      } else if ((p = dangling.rfind("MULTILINESTRING(", start)) !=
                 std::string::npos) {
        util::geo::I32MultiLine ml;
        p += 16;
        while ((p = dangling.find("(", p)) != std::string::npos) {
          const auto& line = parseLineString(dangling, p + 1);
          if (line.size() != 0) {
            ml.push_back(line);
          }
          p += 1;
        }
        idx.add(ml, id);
      } else if ((p = dangling.rfind("POLYGON(", start)) != std::string::npos) {
        p += 7;
        size_t i = 0;
        util::geo::I32Polygon poly;
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
        idx.add(poly, id);
      } else if ((p = dangling.rfind("MULTIPOLYGON(", start)) !=
                 std::string::npos) {
        p += 12;
        util::geo::I32MultiPolygon mp;
        while (p != std::string::npos &&
               (p = dangling.find("(", p + 1)) != std::string::npos) {
          util::geo::I32Polygon poly;
          size_t i = 0;
          while ((p = dangling.find("(", p + 1)) != std::string::npos) {
            const auto& line = parseLineString(dangling, p + 1);
            if (i == 0) {
              // outer
              poly.getOuter() = line;
            } else {
              poly.getInners().push_back(line);
            }

            // check if multipolygon is closed
            auto q = dangling.find(
                ")", p + 1);  // this is the closing of the linestring
            auto q2 = dangling.find(")", q + 1);
            auto q3 = dangling.find(",", q + 1);
            if (q2 != std::string::npos && q3 != std::string::npos && q2 < q3) {
              p = q3;
              break;
            }

            i++;
          }
          mp.push_back(poly);
        }
        idx.add(mp, id);
      }

      dangling.clear();
      c++;
      continue;
    }

    dangling += *c;

    c++;
  }
}

#endif
