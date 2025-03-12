// Copyright 2024, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#include "WKTParse.h"

using sj::WKTParser;

// _____________________________________________________________________________
WKTParser::WKTParser(sj::Sweeper* sweeper, size_t numThreads) : _sweeper(sweeper), _jobs(1000), _thrds(numThreads), _bboxes(numThreads) {
  for (size_t i = 0; i < _thrds.size(); i++) {
    _thrds[i] = std::thread(&WKTParser::processQueue, this, i);
  }
}

// _____________________________________________________________________________
void WKTParser::done() {
  if (_curBatch.size()) {
    _jobs.add(std::move(_curBatch));
    _curBatch.clear();
  }

  // end event
  _jobs.add({});
  // wait for all workers to finish
  for (auto& thr : _thrds) thr.join();

  // collect bounding box of parsed geometries
  for (const auto& box : _bboxes) {
    _bbox = util::geo::extendBox(box, _bbox);
  }
}

// _____________________________________________________________________________
 util::geo::I32Line WKTParser::parseLineString(const char* c,
                                                     const char** endr) {
  util::geo::I32Line line;

  c = strchr(c, '(');
  if (!c) return line;  // parse error
  c++;

  auto end = strchr(c, ')');
  if (endr) (*endr) = end;
  if (!end) return line;  // parse error

  line.reserve((end - c) / 20);

  while (true) {
    while (*c && *c != ')' && isspace(*c)) c++;

    double x = util::atof(c, 10);

    const char* next = strchr(c, ' ');

    if (!next || next >= end) return {};  // parse error

    while (*next && *next != ')' && isspace(*next)) next++;

    double y = util::atof(next, 10);
    auto projPoint = latLngToWebMerc(util::geo::DPoint(x, y));

    line.push_back({static_cast<int>(projPoint.getX() * PREC),
                    static_cast<int>(projPoint.getY() * PREC)});

    auto n = strchr(next, ',');
    if (!n || n > end) break;
    c = n + 1;
  }

  return line;
}

// _____________________________________________________________________________
 util::geo::I32Point WKTParser::parsePoint(const char* c) {
  c = strchr(c, '(');
  if (!c) return {0, 0};  // TODO!

  c += 1;
  while (*c && *c != ')' && isspace(*c)) c++;

  double x = util::atof(c, 10);
  const char* next = strchr(c, ' ');
  if (!next) return {0, 0};  // TODO!
  while (*next && *next != ')' && isspace(*next)) next++;
  double y = util::atof(next, 10);
  auto point = latLngToWebMerc(util::geo::DPoint(x, y));

  return {static_cast<int>(point.getX() * PREC),
          static_cast<int>(point.getY() * PREC)};
}

// _____________________________________________________________________________
 util::geo::I32Polygon WKTParser::parsePolygon(const char* c,
                                                     const char** endr) {
  c = strchr(c, '(');
  if (!c) return {};  // parse error
  c += 1;

  size_t i = 0;
  util::geo::I32Polygon poly;
  while ((c = strchr(c, '('))) {
    const char* end = 0;
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
 util::geo::I32MultiLine WKTParser::parseMultiLineString(
    const char* c, const char** endr) {
  c = strchr(c, '(');
  if (!c) return {};  // parse error
  c += 1;

  util::geo::I32MultiLine ml;
  while ((c = strchr(c, '('))) {
    const char* end = 0;
    const auto& line = parseLineString(c, &end);
    if (line.size() != 0) ml.push_back(std::move(line));

    auto nextComma = strchr(end + 1, ',');
    auto nextCloseBracket = strchr(end + 1, ')');

    if (!nextComma ||
        (nextComma && nextCloseBracket && nextComma > nextCloseBracket)) {
      if (endr) (*endr) = nextCloseBracket;
      return ml;
    }

    c = nextComma;
  }

  return ml;
}

// _____________________________________________________________________________
 util::geo::I32MultiPolygon WKTParser::parseMultiPolygon(
    const char* c, const char** endr) {
  c = strchr(c, '(');
  if (!c) return {};  // parse error
  c += 1;

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

    if (!nextComma ||
        (nextComma && nextCloseBracket && nextComma > nextCloseBracket)) {
      if (endr) (*endr) = nextCloseBracket;
      return mp;
    }

    c = nextComma;
  } while (c);

  return mp;
}

// _____________________________________________________________________________
 std::pair<util::geo::I32Collection, size_t>
WKTParser::parseGeometryCollection(const char* c) {
  util::geo::I32Collection col;

  c = strchr(c, '(');
  if (!c) return {col, 0};
  size_t numGeoms = 0;

  do {
    c++;
    while (*c == ' ') c++;  // skip possible whitespace

    if (memcmp(c, "POINT", 5) == 0) {
      c += 5;
      const char* end = strchr(c, ')');
      const auto& point = parsePoint(c);

      if (!end) break;

      col.push_back(point);
      numGeoms++;
      c = const_cast<char*>(strchr(end, ','));
    } else if (memcmp(c, "POLYGON", 7) == 0) {
      c += 7;
      const char* end = 0;
      const auto& poly = parsePolygon(c, &end);

      if (!end) break;
      if (poly.getOuter().size() > 1) {
        col.push_back(poly);
        numGeoms++;
      }
      c = const_cast<char*>(strchr(end, ','));
    } else if (memcmp(c, "LINESTRING", 10) == 0) {
      c += 10;
      const char* end = 0;
      const auto& line = parseLineString(c, &end);

      if (!end) break;
      if (line.size() > 1) {
        col.push_back(line);
        numGeoms++;
      }
      c = const_cast<char*>(strchr(end, ','));
    } else if (memcmp(c, "MULTIPOINT", 10) == 0) {
      c += 10;
      const char* end = 0;
      const auto& line = parseLineString(c, &end);

      if (!end) break;
      if (line.size() > 0) {
        col.push_back(util::geo::I32MultiPoint(std::move(line)));
        numGeoms += line.size();
      }
      c = const_cast<char*>(strchr(end, ','));
    } else if (memcmp(c, "MULTIPOLYGON", 12) == 0) {
      c += 12;
      const char* end = 0;
      const auto& mp = parseMultiPolygon(c, &end);

      if (!end) break;
      if (mp.size()) {
        col.push_back(mp);
        numGeoms += mp.size();
      }
      c = const_cast<char*>(strchr(end, ','));
    } else if (memcmp(c, "MULTILINESTRING", 15) == 0) {
      c += 15;

      const char* end = 0;
      const auto& ml = parseMultiLineString(c, &end);

      if (!end) break;
      if (ml.size()) {
        col.push_back(ml);
        numGeoms += ml.size();
      }
      c = const_cast<char*>(strchr(end, ','));
    }
  } while (c && *c);

  return {col, numGeoms};
}

// _____________________________________________________________________________
 void WKTParser::parseLine(char* c, size_t len, size_t gid, size_t t,
                                 sj::WriteBatch& batch,
                                 bool side) {
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
        _sweeper->add(c,
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
    _sweeper->add(c,
                 util::geo::I32Box({std::numeric_limits<int32_t>::min(),
                                    std::numeric_limits<int32_t>::min()},
                                   {std::numeric_limits<int32_t>::max(),
                                    std::numeric_limits<int32_t>::max()}),
                 id, subId, side, batch);
  } else if (len > 5 && memcmp(c, "POINT", 5) == 0) {
    c += 5;
    const auto& point = parsePoint(c);
    _bboxes[t] = util::geo::extendBox(_sweeper->add(point, id, side, batch), _bboxes[t]);
  } else if (len > 10 && memcmp(c, "MULTIPOINT", 10) == 0) {
    c += 10;
    const auto& mp = util::geo::I32MultiPoint(parseLineString(c, 0));
    if (mp.size() != 0) _bboxes[t] = util::geo::extendBox(_sweeper->add(mp, id, side, batch), _bboxes[t]);
  } else if (len > 10 && memcmp(c, "LINESTRING", 10) == 0) {
    c += 10;
    const auto& line = parseLineString(c, 0);
    if (line.size() > 1) _bboxes[t] = util::geo::extendBox(_sweeper->add(line, id, side, batch), _bboxes[t]);
  } else if (len > 15 && memcmp(c, "MULTILINESTRING", 15) == 0) {
    c += 15;
    const auto& ml = parseMultiLineString(c, 0);
    _bboxes[t] = util::geo::extendBox(_sweeper->add(ml, id, side, batch), _bboxes[t]);
  } else if (len > 7 && memcmp(c, "POLYGON", 7) == 0) {
    c += 7;
    const auto& poly = parsePolygon(c, 0);
    if (poly.getOuter().size() > 1) _bboxes[t] = util::geo::extendBox(_sweeper->add(poly, id, side, batch), _bboxes[t]);
  } else if (len > 12 && memcmp(c, "MULTIPOLYGON", 12) == 0) {
    c += 12;
    const auto& mp = parseMultiPolygon(c, 0);
    if (mp.size()) _bboxes[t] = util::geo::extendBox(_sweeper->add(mp, id, side, batch), _bboxes[t]);
  } else if (len > 18 && memcmp(c, "GEOMETRYCOLLECTION", 18) == 0) {
    c += 18;

    const auto& p = parseGeometryCollection(c);
    const auto& col = p.first;

    size_t subId = p.second > 1 ? 1 : 0;

    for (const auto& a : col) {
      if (a.getType() == 0) _bboxes[t] = util::geo::extendBox(_sweeper->add(a.getPoint(), id, subId, side, batch), _bboxes[t]);
      if (a.getType() == 1) _bboxes[t] = util::geo::extendBox(_sweeper->add(a.getLine(), id, subId, side, batch), _bboxes[t]);
      if (a.getType() == 2)
        _bboxes[t] = util::geo::extendBox(_sweeper->add(a.getPolygon(), id, subId, side, batch), _bboxes[t]);
      if (a.getType() == 3)
        _bboxes[t] = util::geo::extendBox(_sweeper->add(a.getMultiLine(), id, subId, side, batch), _bboxes[t]);
      if (a.getType() == 4)
        _bboxes[t] = util::geo::extendBox(_sweeper->add(a.getMultiPolygon(), id, subId, side, batch), _bboxes[t]);
      if (a.getType() == 6)
        _bboxes[t] = util::geo::extendBox(_sweeper->add(a.getMultiPoint(), id, subId, side, batch), _bboxes[t]);
      subId++;
    }
  }
}

// _____________________________________________________________________________
 void WKTParser::processQueue(size_t t) {
  ParseBatch batch;
  while ((batch = _jobs.get()).size()) {
    sj::WriteBatch w;
    for (const auto& job : batch) {
      if (job.str.size()) {
        parseLine(const_cast<char*>(job.str.c_str()), job.str.size(), job.line, t,
                   w, job.side);
      } else {
        // parse point directly
        auto mercPoint = latLngToWebMerc(job.point);

        util::geo::I32Point addPoint{static_cast<int>(mercPoint.getX() * PREC),
                static_cast<int>(mercPoint.getY() * PREC)};
        _bboxes[t] = util::geo::extendBox(_sweeper->add(addPoint, std::to_string(job.line), job.side, w), _bboxes[t]);
      }
    }

    _sweeper->addBatch(w);
  }
}

// _____________________________________________________________________________
 void WKTParser::parseWKT(const char* c, size_t id, bool side) {
   // dont parse empty strings
   if (*c == 0) return;

  _curBatch.reserve(10000);
  _curBatch.push_back({std::string(c), id, side, {0, 0}});

  if (_curBatch.size() > 10000) {
    _jobs.add(std::move(_curBatch));
    _curBatch.clear();
  }
}

// _____________________________________________________________________________
 void WKTParser::parsePoint(util::geo::DPoint point, size_t id, bool side) {
  _curBatch.reserve(10000);
  _curBatch.push_back({"", id, side, point});

  if (_curBatch.size() > 10000) {
    _jobs.add(std::move(_curBatch));
    _curBatch.clear();
  }
}

// _____________________________________________________________________________
 void WKTParser::parse(char* c, size_t size, bool side) {
  size_t p = 0;

  while (true) {
    char* newLine = reinterpret_cast<char*>(memchr(c + p, '\n', size - p));

    if (newLine) {
      size_t pNext = newLine - c;
      c[pNext] = 0;

      if (_dangling.size()) {
        _dangling += (c + p);  // guaranteed to be null terminated
        _curBatch.push_back({_dangling, _gid, side, {0, 0}});
        _dangling.clear();
      } else {
        if (*(c+p)) _curBatch.push_back({c + p, _gid, side, {0, 0}});

        if (_curBatch.size() > 1000) {
          _jobs.add(std::move(_curBatch));
          _curBatch.clear();  // std doesnt guarantee that after move
          _curBatch.reserve(1000);
        }
      }

      p = pNext + 1;

      _gid++;
    } else {
      size_t oldSize = _dangling.size();
      _dangling.resize(oldSize + (size - p));
      memcpy(const_cast<char*>(&_dangling.data()[oldSize]), (c + p), size - p);

      if (_curBatch.size()) {
        _jobs.add(std::move(_curBatch));
        _curBatch.clear();  // std doesnt guarantee that after move
        _curBatch.reserve(1000);
      }

      return;
    }
  }

  if (_curBatch.size()) {
    _jobs.add(std::move(_curBatch));
    _curBatch.clear();  // std doesnt guarantee that after move
    _curBatch.reserve(1000);
  }
}
