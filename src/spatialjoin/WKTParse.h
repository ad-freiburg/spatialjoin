#ifndef SPATIALJOINS_WKTPARSE_H_
#define SPATIALJOINS_WKTPARSE_H_

#include "Sweeper.h"
#include "util/geo/Geo.h"
#include "util/log/Log.h"

// _____________________________________________________________________________
util::geo::I32Line parseLineString(const char* c, size_t res) {
  util::geo::I32Line line;
  auto end = strchr(c, ')');
  if (!end) return line;

  line.reserve(res);

  while (true) {
    while (*c && *c != ')' && isspace(*c)) c++;

    double x = util::atof(c, 10);

    const char* next = strchr(c, ' ');

    if (!next || next >= end) return {};
    double y = util::atof(next + 1, 10);
    auto projPoint = latLngToWebMerc(util::geo::DPoint(x, y));

    line.push_back({projPoint.getX() * PREC, projPoint.getY() * PREC});

    auto n = strchr(next, ',');
    if (!n || n > end) break;
    c = n + 1;
  }

  return line;
}

// _____________________________________________________________________________
util::geo::I32Point parsePoint(const char* c) {
  double x = util::atof(c, 10);
  const char* next = strchr(c, ' ');
  if (!next) return {0, 0};  // TODO!
  double y = util::atof(next + 1, 10);
  auto point = latLngToWebMerc(util::geo::DPoint(x, y));

  return {point.getX() * PREC, point.getY() * PREC};
}

// _____________________________________________________________________________
void parseLine(char* c, size_t len, size_t* gid, sj::Sweeper& idx) {
  (*gid)++;

  char* idp = reinterpret_cast<char*>(strchr(c, '\t'));

  std::string id;

  if (idp) {
    *idp = 0;
    id = c;
    c = idp + 1;
  } else {
    id = std::to_string(*gid);
  }

  if (len > 6 && memcmp(c, "POINT(", 6) == 0) {
    c += 6;
    auto point = parsePoint(c);
    idx.add(point, id);
    return;
  } else if (len > 11 && memcmp(c, "MULTIPOINT(", 11) == 0) {
    c += 11;
    const auto& mp = parseLineString(c, (len - 11) / 20);
    if (mp.size() != 0) idx.addMp(mp, id);
    return;
  } else if (len > 11 && memcmp(c, "LINESTRING(", 11) == 0) {
    c += 11;
    const auto& line = parseLineString(c, (len - 11) / 20);
    if (line.size() > 1) idx.add(line, id);
  } else if (len > 16 && memcmp(c, "MULTILINESTRING(", 16) == 0) {
    util::geo::I32MultiLine ml;
    c += 16;
    while ((c = strchr(c, '('))) {
      c++;
      const auto& line = parseLineString(c, 10);
      if (line.size() != 0) ml.push_back(std::move(line));
    }
    idx.add(ml, id);
  } else if (len > 8 && memcmp(c, "POLYGON(", 8) == 0) {
    c += 7;
    size_t i = 0;
    util::geo::I32Polygon poly;
    while ((c = strchr(c + 1, '('))) {
      c++;
      const auto& line = parseLineString(c, (len - 8) / 20);
      if (i == 0)
        poly.getOuter() = line;
      else
        poly.getInners().push_back(std::move(line));
      i++;
    }
    if (poly.getOuter().size() > 1) idx.add(poly, id);
  } else if (len > 13 && memcmp(c, "MULTIPOLYGON(", 13) == 0) {
    c += 12;
    util::geo::I32MultiPolygon mp;
    while (c && (c = strchr(c + 1, '('))) {
      util::geo::I32Polygon poly;
      size_t i = 0;
      while ((c = strchr(c + 1, '('))) {
        c++;
        const auto& line = parseLineString(c, (len - 12) / 20);
        if (i == 0)
          poly.getOuter() = line;
        else
          poly.getInners().push_back(std::move(line));

        // check if multipolygon is closed
        auto q = strchr(c, ')');  // this is the closing of the linestring
        auto q2 = strchr(q + 1, ')');
        auto q3 = strchr(q + 1, ',');
        if (q2 && q2 < q3) {
          c = q3;
          break;
        }

        i++;
      }
      mp.push_back(std::move(poly));
    }
    idx.add(mp, id);
  }
}

// _____________________________________________________________________________
void parse(char* c, size_t size, std::string& dang, size_t* gid,
           sj::Sweeper& idx) {
  size_t p = 0;

  while (true) {
    char* newLine = reinterpret_cast<char*>(memchr(c + p, '\n', size - p));

    if (newLine) {
      size_t pNext = newLine - c;
      c[pNext] = 0;

      if (dang.size()) {
        dang += (c + p);  // guaranteed to be null terminated
        parseLine(const_cast<char*>(dang.c_str()), dang.size(), gid, idx);
        dang.clear();
      } else {
        parseLine(c + p, pNext - p, gid, idx);
      }

      p = pNext + 1;
    } else {
      size_t oldSize = dang.size();
      dang.resize(oldSize + (size - p));
      memcpy(const_cast<char*>(&dang.data()[oldSize]), (c + p), size - p);
      return;
    }
  }
}

#endif
