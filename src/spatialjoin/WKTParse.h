#ifndef SPATIALJOINS_WKTPARSE_H_
#define SPATIALJOINS_WKTPARSE_H_

#include "Sweeper.h"
#include "util/geo/Geo.h"
#include "util/log/Log.h"

struct ParseJob {
  std::string str;
  size_t line;
  bool side;
};

inline bool operator==(const ParseJob& a, const ParseJob& b) {
  return a.line == b.line && a.str == b.str && a.side == b.side;
}

typedef std::vector<ParseJob> ParseBatch;

// _____________________________________________________________________________
inline util::geo::I32Line parseLineString(const char* c, const char** endr) {
  util::geo::I32Line line;
  auto end = strchr(c, ')');
  if (endr) (*endr) = end;
  if (!end) return line;

  line.reserve((end - c) / 20);

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
inline util::geo::I32Point parsePoint(const char* c) {
  double x = util::atof(c, 10);
  const char* next = strchr(c, ' ');
  if (!next) return {0, 0};  // TODO!
  double y = util::atof(next + 1, 10);
  auto point = latLngToWebMerc(util::geo::DPoint(x, y));

  return {point.getX() * PREC, point.getY() * PREC};
}

// _____________________________________________________________________________
inline util::geo::I32Polygon parsePolygon(const char* c, const char** endr) {
  c += 1;
  size_t i = 0;
  util::geo::I32Polygon poly;
  while ((c = strchr(c, '('))) {
    c++;
    const char* end  = 0;
    const auto& line = parseLineString(c, &end);

    if (!end) return {};  // parse error

    c = end;

    if (i == 0)
      poly.getOuter() = line;
    else
      poly.getInners().push_back(std::move(line));
    i++;


    auto q = strchr(c + 1, ')');
    auto cc = strchr(c + 1, '(');

    if ((!cc && q) || (q && cc && q < cc)) {
      // polygon closes at q
      if (endr) (*endr) = q;
      return poly;
    }

    if (cc) c = cc;
  }

  return poly;
}

// _____________________________________________________________________________
inline util::geo::I32MultiLine parseMultiLineString(const char* c, const char** endr) {
  util::geo::I32MultiLine ml;
  while ((c = strchr(c, '('))) {
    c++;
    const char* end = 0;
    const auto& line = parseLineString(c, &end);
    if (line.size() != 0) ml.push_back(std::move(line));

    auto nextComma = strchr(end + 1, ',');
    auto nextCloseBracket = strchr(end + 1, ')');

    if (!nextComma || (nextComma && nextCloseBracket && nextComma > nextCloseBracket)) {
      if (endr) (*endr) = nextCloseBracket;
      return ml;
    }

    c = nextComma;
  }

  return ml;
}

// _____________________________________________________________________________
inline util::geo::I32MultiPolygon parseMultiPolygon(const char* c, const char** endr) {
  util::geo::I32MultiPolygon mp;
  do {
    c = strchr(c, '(');
    if (!c) break;
    const char* end = 0;
    const auto& poly = parsePolygon(c, &end);

    if (!end) break;

    if (poly.getOuter().size() > 1) mp.push_back(std::move(poly));

    auto nextComma = strchr(end + 1, ',');
    auto nextCloseBracket = strchr(end + 1, ')');

    if (!nextComma || (nextComma && nextCloseBracket && nextComma > nextCloseBracket)) {
      if (endr) (*endr) = nextCloseBracket;
      return mp;
    }

    c = nextComma;
  } while (c);

  return mp;
}

// _____________________________________________________________________________
inline void parseLine(char* c, size_t len, size_t gid, sj::Sweeper* sweeper,
                      sj::WriteBatch& batch, bool side) {
  char* idp = reinterpret_cast<char*>(strchr(c, '\t'));

  std::string id;

  if (idp) {
    *idp = 0;
    id = c;
    len -= (idp - c) + 1;
    c = idp + 1;
  } else {
    id = std::to_string(gid);
  }

  idp = reinterpret_cast<char*>(strchr(c, '\t'));

  if (idp) {
    *idp = 0;
    side = atoi(c);
    len -= (idp - c) + 1;
    c = idp + 1;
  }

  if (len > 2 && *c == '<') {
    char* end = strchr(c, ',');
    size_t subId = 0;
    c += 1;
    if (end) {
      subId = 1;
      do {
        *end = 0;
        sweeper->add(c,

                     util::geo::I32Box({std::numeric_limits<int32_t>::min(),
                                        std::numeric_limits<int32_t>::min()},
                                       {std::numeric_limits<int32_t>::max(),
                                        std::numeric_limits<int32_t>::max()}),
                     id, subId, side, batch);
        len -= (end - c) + 1;
        c = end + 1;
      } while (len > 0 && (end = strchr(c, ',')));
    }

    c[len - 2] = 0;
    sweeper->add(c,

                 util::geo::I32Box({std::numeric_limits<int32_t>::min(),
                                    std::numeric_limits<int32_t>::min()},
                                   {std::numeric_limits<int32_t>::max(),
                                    std::numeric_limits<int32_t>::max()}),
                 id, subId, side, batch);
  } else if (len > 6 && memcmp(c, "POINT(", 6) == 0) {
    c += 6;
    const auto& point = parsePoint(c);
    sweeper->add(point, id, side, batch);
    return;
  } else if (len > 11 && memcmp(c, "MULTIPOINT(", 11) == 0) {
    c += 11;
    const auto& mp = parseLineString(c, 0);
    if (mp.size() != 0) sweeper->addMp(mp, id, side, batch);
    return;
  } else if (len > 11 && memcmp(c, "LINESTRING(", 11) == 0) {
    c += 11;
    const auto& line = parseLineString(c, 0);
    if (line.size() > 1) sweeper->add(line, id, side, batch);
  } else if (len > 16 && memcmp(c, "MULTILINESTRING(", 16) == 0) {
    c += 16;
    const auto& ml = parseMultiLineString(c, 0);
    sweeper->add(ml, id, side, batch);
  } else if (len > 8 && memcmp(c, "POLYGON(", 8) == 0) {
    c += 7;
    const auto& poly = parsePolygon(c, 0);
    if (poly.getOuter().size() > 1) sweeper->add(poly, id, side, batch);
  } else if (len > 13 && memcmp(c, "MULTIPOLYGON(", 13) == 0) {
    c += 13;
    const auto& mp = parseMultiPolygon(c, 0);
    if (mp.size()) sweeper->add(mp, id, side, batch);
  } else if (len > 19 && memcmp(c, "GEOMETRYCOLLECTION(", 19) == 0) {
    c += 18;
    size_t subId = 1;

    do {
      c++;
      while (*c == ' ') c++;  // skip possible whitespace

      if (memcmp(c, "POINT(", 6) == 0) {
        c += 6;
        const char* end = strchr(c, ')');
        const auto& point = parsePoint(c);

        if (!end) break;

        sweeper->add(point, id, subId, side, batch);
        subId++;
        c = const_cast<char*>(strchr(end, ','));
      } else if (memcmp(c, "POLYGON(", 8) == 0) {
        c += 7;
        const char* end = 0;
        const auto& poly = parsePolygon(c, &end);

        if (!end) break;
        if (poly.getOuter().size() > 1) {
          sweeper->add(poly, id, subId, side, batch);
          subId++;
        }
        c = const_cast<char*>(strchr(end, ','));
      } else if (memcmp(c, "LINESTRING(", 11) == 0) {
        c += 11;
        const char* end = 0;
        const auto& line = parseLineString(c, &end);

        if (!end) break;
        if (line.size() > 1) {
          sweeper->add(line, id, subId, side, batch);
          subId++;
        }
        c = const_cast<char*>(strchr(end, ','));
      } else if (memcmp(c, "MULTIPOLYGON(", 13) == 0) {
        c += 13;
        const char* end = 0;
        const auto& mp = parseMultiPolygon(c, &end);

        if (!end) break;
        if (mp.size()) {
          sweeper->add(mp, id, subId, side, batch);
          subId++;
        }
        c = const_cast<char*>(strchr(end, ','));
      } else if (memcmp(c, "MULTILINESTRING(", 16) == 0) {
        c += 16;
        const char* end = 0;
        const auto& ml = parseMultiLineString(c, &end);

        if (!end) break;
        if (ml.size()) {
          sweeper->add(ml, id, subId, side, batch);
          subId++;
        }
        c = const_cast<char*>(strchr(end, ','));
      }
    } while (c && *c);
  }
}

// _____________________________________________________________________________
inline void processQueue(util::JobQueue<ParseBatch>* jobs, size_t,
                         sj::Sweeper* idx) {
  ParseBatch batch;
  while ((batch = jobs->get()).size()) {
    sj::WriteBatch w;
    for (const auto& job : batch) {
      parseLine(const_cast<char*>(job.str.c_str()), job.str.size(), job.line,
                idx, w, job.side);
    }

    idx->addBatch(w);
  }
}

// _____________________________________________________________________________
inline void parse(char* c, size_t size, std::string& dang, size_t* gid,
                  util::JobQueue<ParseBatch>& jobs, bool side) {
  size_t p = 0;

  ParseBatch curBatch;
  curBatch.reserve(1000);

  while (true) {
    char* newLine = reinterpret_cast<char*>(memchr(c + p, '\n', size - p));

    if (newLine) {
      size_t pNext = newLine - c;
      c[pNext] = 0;

      if (dang.size()) {
        dang += (c + p);  // guaranteed to be null terminated
        curBatch.push_back({dang, *gid, side});
        dang.clear();
      } else {
        curBatch.push_back({c + p, *gid, side});

        if (curBatch.size() > 1000) {
          jobs.add(std::move(curBatch));
          curBatch.clear();  // std doesnt guarantee that after move
          curBatch.reserve(1000);
        }
      }

      p = pNext + 1;

      (*gid)++;
    } else {
      size_t oldSize = dang.size();
      dang.resize(oldSize + (size - p));
      memcpy(const_cast<char*>(&dang.data()[oldSize]), (c + p), size - p);

      if (curBatch.size()) jobs.add(std::move(curBatch));

      return;
    }
  }

  if (curBatch.size()) jobs.add(std::move(curBatch));
}

#endif
