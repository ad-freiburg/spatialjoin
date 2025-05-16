// Copyright 2024, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#include "WKTParse.h"
#include "Libgeos.h"

#ifdef __cpp_lib_string_view
#include <string_view>
#endif

using sj::WKTParser;
using util::geo::collectionFromWKT;
using util::geo::getWKTType;
using util::geo::lineFromWKT;
using util::geo::multiLineFromWKT;
using util::geo::multiPointFromWKT;
using util::geo::multiPolygonFromWKT;
using util::geo::pointFromWKT;
using util::geo::polygonFromWKT;

// _____________________________________________________________________________
WKTParser::WKTParser(sj::Sweeper* sweeper, size_t numThreads)
    : _sweeper(sweeper),
      _jobs(1000),
      _thrds(numThreads),
      _bboxes(numThreads),
      _cancelled(false) {
  for (size_t i = 0; i < _thrds.size(); i++) {
    _thrds[i] = std::thread(&WKTParser::processQueue, this, i);
  }
}

// _____________________________________________________________________________
WKTParser::~WKTParser() {
  // graceful shutdown of all threads, should they be still running
  _cancelled = true;

  // end event
  _jobs.add({});

  // wait for all workers to finish
  for (auto& thr : _thrds) if (thr.joinable()) thr.join();
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
void WKTParser::parseLine(char* c, size_t len, size_t gid, size_t t,
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
  } else {
    auto wktType = getWKTType(c, const_cast<const char**>(&c));
    if (wktType == util::geo::WKTType::POINT) {
      const auto& point = pointFromWKT<int32_t>(c, 0, &projFunc);
      _bboxes[t] = util::geo::extendBox(_sweeper->add(point, id, side, batch),
                                        _bboxes[t]);
    } else if (wktType == util::geo::WKTType::MULTIPOINT) {
      const auto& mp = multiPointFromWKT<int32_t>(c, 0, &projFunc);
      if (mp.size() != 0)
        _bboxes[t] = util::geo::extendBox(_sweeper->add(mp, id, side, batch),
                                          _bboxes[t]);
    } else if (wktType == util::geo::WKTType::LINESTRING) {
      const auto& line = lineFromWKT<int32_t>(c, 0, &projFunc);
      if (line.size() > 1)
        _bboxes[t] = util::geo::extendBox(_sweeper->add(line, id, side, batch),
                                          _bboxes[t]);
    } else if (wktType == util::geo::WKTType::MULTILINESTRING) {
      const auto& ml = multiLineFromWKT<int32_t>(c, 0, &projFunc);
      _bboxes[t] =
          util::geo::extendBox(_sweeper->add(ml, id, side, batch), _bboxes[t]);
    } else if (wktType == util::geo::WKTType::POLYGON) {
      const auto& poly = polygonFromWKT<int32_t>(c, 0, &projFunc);
      if (poly.getOuter().size() > 1)
        _bboxes[t] = util::geo::extendBox(_sweeper->add(poly, id, side, batch),
                                          _bboxes[t]);
    } else if (wktType == util::geo::WKTType::MULTIPOLYGON) {
      const auto& mp = multiPolygonFromWKT<int32_t>(c, 0, &projFunc);
      if (mp.size())
        _bboxes[t] = util::geo::extendBox(_sweeper->add(mp, id, side, batch),
                                          _bboxes[t]);
    } else if (wktType == util::geo::WKTType::COLLECTION) {
      const auto& col = collectionFromWKT<int32_t>(c, 0, &projFunc);

      size_t numGeoms = 0;
      for (const auto& a : col) {
        if (a.getType() == 0) numGeoms++;
        if (a.getType() == 1) numGeoms++;
        if (a.getType() == 2) numGeoms++;
        if (a.getType() == 3) numGeoms += a.getMultiLine().size();
        if (a.getType() == 4) numGeoms += a.getMultiPolygon().size();
        if (a.getType() == 6) numGeoms += a.getMultiPoint().size();
      }

      size_t subId = numGeoms > 1 ? 1 : 0;

      for (const auto& a : col) {
        if (a.getType() == 0)
          _bboxes[t] = util::geo::extendBox(
              _sweeper->add(a.getPoint(), id, subId, side, batch), _bboxes[t]);
        if (a.getType() == 1)
          _bboxes[t] = util::geo::extendBox(
              _sweeper->add(a.getLine(), id, subId, side, batch), _bboxes[t]);
        if (a.getType() == 2)
          _bboxes[t] = util::geo::extendBox(
              _sweeper->add(a.getPolygon(), id, subId, side, batch),
              _bboxes[t]);
        if (a.getType() == 3)
          _bboxes[t] = util::geo::extendBox(
              _sweeper->add(a.getMultiLine(), id, subId, side, batch),
              _bboxes[t]);
        if (a.getType() == 4)
          _bboxes[t] = util::geo::extendBox(
              _sweeper->add(a.getMultiPolygon(), id, subId, side, batch),
              _bboxes[t]);
        if (a.getType() == 6)
          _bboxes[t] = util::geo::extendBox(
              _sweeper->add(a.getMultiPoint(), id, subId, side, batch),
              _bboxes[t]);
        subId++;
      }
    }
  }
}

// _____________________________________________________________________________
void WKTParser::processQueue(size_t t) {
  ParseBatch batch;

  auto geosHndl = initGEOS_r(GEOSMsgHandler, GEOSMsgHandler);

  while ((batch = _jobs.get()).size()) {
    sj::WriteBatch w;
    w.geosHndl = geosHndl;
    for (const auto& job : batch) {
      if (_cancelled) break;

      if (job.str.size()) {
        parseLine(const_cast<char*>(job.str.c_str()), job.str.size(), job.line,
                  t, w, job.side);
      } else {
        // parse point directly
        auto mercPoint = latLngToWebMerc(job.point);

        util::geo::I32Point addPoint{static_cast<int>(mercPoint.getX() * PREC),
                                     static_cast<int>(mercPoint.getY() * PREC)};
        _bboxes[t] = util::geo::extendBox(
            _sweeper->add(addPoint, std::to_string(job.line), job.side, w),
            _bboxes[t]);
      }
    }

    _sweeper->addBatch(w);
  }

  GEOS_finish_r(geosHndl);
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
void WKTParser::parseWKT(const std::string& str, size_t id, bool side) {
  if (str.empty()) return;

  _curBatch.reserve(10000);
  _curBatch.push_back({str, id, side, {0, 0}});

  if (_curBatch.size() > 10000) {
    _jobs.add(std::move(_curBatch));
    _curBatch.clear();
  }
}

#ifdef __cpp_lib_string_view
// _____________________________________________________________________________
void WKTParser::parseWKT(const std::string_view str, size_t id, bool side) {
  // dont parse empty strings
  if (str.empty()) return;

  _curBatch.reserve(10000);
  _curBatch.push_back({std::string{str}, id, side, {0, 0}});

  if (_curBatch.size() > 10000) {
    _jobs.add(std::move(_curBatch));
    _curBatch.clear();
  }
}

#endif

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
        if (*(c + p)) _curBatch.push_back({c + p, _gid, side, {0, 0}});

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
