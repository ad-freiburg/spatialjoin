// Copyright 2024, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#ifndef SPATIALJOINS_WKTPARSE_H_
#define SPATIALJOINS_WKTPARSE_H_

#include "Sweeper.h"
#include "util/geo/Geo.h"
#include "util/log/Log.h"
#include <atomic>

#ifdef __cpp_lib_string_view
#include <string_view>
#endif

namespace sj {

struct ParseJob {
  std::string str;
  size_t line;
  bool side;
  util::geo::DPoint point;
};

inline bool operator==(const ParseJob &a, const ParseJob &b) {
  return a.line == b.line && a.str == b.str && a.side == b.side;
}

typedef std::vector<ParseJob> ParseBatch;

template <typename ParseJobT> class WKTParserBase {
public:
  WKTParserBase(sj::Sweeper *sweeper, size_t numThreads)
      : _sweeper(sweeper), _jobs(1000), _thrds(numThreads), _bboxes(numThreads),
        _cancelled(false) {
    for (size_t i = 0; i < _thrds.size(); i++) {
      _thrds[i] = std::thread(&WKTParserBase<ParseJobT>::processQueue, this, i);
    }
  };
  ~WKTParserBase() {
    // graceful shutdown of all threads, should they be still running
    _cancelled = true;

    // end event
    _jobs.add({});

    // wait for all workers to finish
    for (auto &thr : _thrds)
      if (thr.joinable())
        thr.join();
  };

  util::geo::I32Box getBoundingBox() const { return _bbox; }

  void done() {
    if (_curBatch.size()) {
      _jobs.add(std::move(_curBatch));
      _curBatch.clear();
    }

    // end event
    _jobs.add({});
    // wait for all workers to finish
    for (auto &thr : _thrds)
      thr.join();

    // collect bounding box of parsed geometries
    for (const auto &box : _bboxes) {
      _bbox = util::geo::extendBox(box, _bbox);
    }
  };

  static util::geo::I32Point projFunc(const util::geo::DPoint &p) {
    auto projPoint = latLngToWebMerc(p);
    return {static_cast<int>(projPoint.getX() * PREC),
            static_cast<int>(projPoint.getY() * PREC)};
  }

protected:
  void parseLine(char *c, size_t len, size_t gid, size_t t,
                 sj::WriteBatch &batch, bool side) {
    using namespace util::geo;

    char *idp = reinterpret_cast<char *>(strchr(c, '\t'));

    std::string id;

    if (idp) {
      *idp = 0;
      id = c;
      len -= (idp - c) + 1;
      c = idp + 1;
    } else {
      id = std::to_string(gid);
    }

    idp = reinterpret_cast<char *>(strchr(c, '\t'));

    if (idp) {
      *idp = 0;
      side = atoi(c);
      len -= (idp - c) + 1;
      c = idp + 1;
    }

    if (len > 2 && *c == '<') {
      char *end = strchr(c, ',');
      size_t subId = 0;
      c += 1;
      if (end) {
        subId = 1;
        do {
          *end = 0;
          _sweeper->add(
              c,
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
      auto wktType = getWKTType(c, const_cast<const char **>(&c));
      if (wktType == util::geo::WKTType::POINT) {
        const auto &point = pointFromWKT<int32_t>(c, 0, &projFunc);
        _bboxes[t] = util::geo::extendBox(_sweeper->add(point, id, side, batch),
                                          _bboxes[t]);
      } else if (wktType == util::geo::WKTType::MULTIPOINT) {
        const auto &mp = multiPointFromWKT<int32_t>(c, 0, &projFunc);
        if (mp.size() != 0)
          _bboxes[t] = util::geo::extendBox(_sweeper->add(mp, id, side, batch),
                                            _bboxes[t]);
      } else if (wktType == util::geo::WKTType::LINESTRING) {
        const auto &line = lineFromWKT<int32_t>(c, 0, &projFunc);
        if (line.size() > 1)
          _bboxes[t] = util::geo::extendBox(
              _sweeper->add(line, id, side, batch), _bboxes[t]);
      } else if (wktType == util::geo::WKTType::MULTILINESTRING) {
        const auto &ml = multiLineFromWKT<int32_t>(c, 0, &projFunc);
        _bboxes[t] = util::geo::extendBox(_sweeper->add(ml, id, side, batch),
                                          _bboxes[t]);
      } else if (wktType == util::geo::WKTType::POLYGON) {
        const auto &poly = polygonFromWKT<int32_t>(c, 0, &projFunc);
        if (poly.getOuter().size() > 1)
          _bboxes[t] = util::geo::extendBox(
              _sweeper->add(poly, id, side, batch), _bboxes[t]);
      } else if (wktType == util::geo::WKTType::MULTIPOLYGON) {
        const auto &mp = multiPolygonFromWKT<int32_t>(c, 0, &projFunc);
        if (mp.size())
          _bboxes[t] = util::geo::extendBox(_sweeper->add(mp, id, side, batch),
                                            _bboxes[t]);
      } else if (wktType == util::geo::WKTType::COLLECTION) {
        const auto &col = collectionFromWKT<int32_t>(c, 0, &projFunc);

        size_t numGeoms = 0;
        for (const auto &a : col) {
          if (a.getType() == 0)
            numGeoms++;
          if (a.getType() == 1)
            numGeoms++;
          if (a.getType() == 2)
            numGeoms++;
          if (a.getType() == 3)
            numGeoms += a.getMultiLine().size();
          if (a.getType() == 4)
            numGeoms += a.getMultiPolygon().size();
          if (a.getType() == 6)
            numGeoms += a.getMultiPoint().size();
        }

        size_t subId = numGeoms > 1 ? 1 : 0;

        for (const auto &a : col) {
          if (a.getType() == 0)
            _bboxes[t] = util::geo::extendBox(
                _sweeper->add(a.getPoint(), id, subId, side, batch),
                _bboxes[t]);
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
  };
  virtual void processQueue(size_t t) = 0;
  size_t _gid = 1;
  std::string _dangling;

  sj::Sweeper *_sweeper;

  std::vector<ParseJobT> _curBatch;

  util::geo::I32Box _bbox;

  util::JobQueue<std::vector<ParseJobT>> _jobs;
  std::vector<std::thread> _thrds;

  std::vector<util::geo::I32Box> _bboxes;

  std::atomic<bool> _cancelled;
};

class WKTParser : public WKTParserBase<ParseJob> {
public:
  WKTParser(sj::Sweeper *sweeper, size_t numThreads);
  void parse(char *c, size_t size, bool side);
  void parseWKT(const char *c, size_t id, bool side);
  void parseWKT(const std::string &str, size_t id, bool side);
#ifdef __cpp_lib_string_view
  void parseWKT(const std::string_view str, size_t id, bool side);
#endif
  void parsePoint(util::geo::DPoint point, size_t id, bool side);

protected:
  void processQueue(size_t t) override;
};

} // namespace sj

#endif
