// Copyright 2024, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#ifndef SPATIALJOINS_WKTPARSE_H_
#define SPATIALJOINS_WKTPARSE_H_

#include <atomic>

#include "Sweeper.h"
#include "util/geo/Geo.h"
#include "util/log/Log.h"

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

template <typename ParseJobT>
class WKTParserBase {
 public:
  WKTParserBase(sj::Sweeper *sweeper, size_t numThreads)
      : _sweeper(sweeper),
        _jobs(1000),
        _thrds(numThreads),
        _bboxes(numThreads),
        _cancelled(false){};
  ~WKTParserBase() {
    // graceful shutdown of all threads, should they be still running
    _cancelled = true;

    // end event
    _jobs.add({});

    // wait for all workers to finish
    for (auto &thr : _thrds)
      if (thr.joinable()) thr.join();
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
    for (auto &thr : _thrds) thr.join();

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
  void parseLine(const char *c, size_t len, size_t gid, size_t t,
                 sj::WriteBatch &batch, bool side) {
    using namespace util::geo;

    const char *lastC = c + len;

    // search for first occurance of tab
    const char *idp = strchr(c, '\t');

    std::string id;

    if (idp) {
      // if we have a tab, set id
      id.assign(c, idp - c);
      c = idp + 1;
    } else {
      id = std::to_string(gid);
    }

    // search for next tab occurance
    idp = strchr(c, '\t');

    if (idp) {
      side = atoi(c);
      c = idp + 1;
    }

    if (lastC - c > 2 && *c == '<') {
      // handle reference geometries
      const char *end = strchr(c, ',');
      size_t subId = 0;
      c += 1;

      if (end) subId = 1;

      if (!end) end = strrchr(c, '>');

      do {
        c += std::strspn(c, " \f\n\r\t\v");
        if (!end) {
          return;  // erroneous line, ignore
        }

        std::string referenceId;
        referenceId.assign(c, end - c);

        if (!referenceId.empty()) {
          _sweeper->add(
              referenceId,
              util::geo::I32Box({std::numeric_limits<int32_t>::min(),
                                 std::numeric_limits<int32_t>::min()},
                                {std::numeric_limits<int32_t>::max(),
                                 std::numeric_limits<int32_t>::max()}),
              id, subId, side, batch);
        }
        c = end + 1;
      } while (c < lastC && ((end = strchr(c, ',')) || (end = strchr(c, '>'))));
    } else {
      auto wktType = getWKTType(c, &c);
      if (wktType == util::geo::WKTType::POINT) {
        const auto &point = pointFromWKTProj<int32_t>(c, 0, &projFunc);
        _bboxes[t] = util::geo::extendBox(_sweeper->add(point, id, side, batch),
                                          _bboxes[t]);
      } else if (wktType == util::geo::WKTType::MULTIPOINT) {
        const auto &mp = multiPointFromWKTProj<int32_t>(c, 0, &projFunc);
        if (mp.size() != 0)
          _bboxes[t] = util::geo::extendBox(_sweeper->add(mp, id, side, batch),
                                            _bboxes[t]);
      } else if (wktType == util::geo::WKTType::LINESTRING) {
        const auto &line = lineFromWKTProj<int32_t>(c, 0, &projFunc);
        if (line.size() > 1)
          _bboxes[t] = util::geo::extendBox(
              _sweeper->add(line, id, side, batch), _bboxes[t]);
      } else if (wktType == util::geo::WKTType::MULTILINESTRING) {
        const auto &ml = multiLineFromWKTProj<int32_t>(c, 0, &projFunc);
        _bboxes[t] = util::geo::extendBox(_sweeper->add(ml, id, side, batch),
                                          _bboxes[t]);
      } else if (wktType == util::geo::WKTType::POLYGON) {
        const auto &poly = polygonFromWKTProj<int32_t>(c, 0, &projFunc);
        if (poly.getOuter().size() > 1)
          _bboxes[t] = util::geo::extendBox(
              _sweeper->add(poly, id, side, batch), _bboxes[t]);
      } else if (wktType == util::geo::WKTType::MULTIPOLYGON) {
        const auto &mp = multiPolygonFromWKTProj<int32_t>(c, 0, &projFunc);
        if (mp.size())
          _bboxes[t] = util::geo::extendBox(_sweeper->add(mp, id, side, batch),
                                            _bboxes[t]);
      } else if (wktType == util::geo::WKTType::COLLECTION) {
        const auto &col = collectionFromWKTProj<int32_t>(c, 0, &projFunc);

        size_t numGeoms = 0;
        for (const auto &a : col) {
          if (a.getType() == 0) numGeoms++;
          if (a.getType() == 1) numGeoms++;
          if (a.getType() == 2) numGeoms++;
          if (a.getType() == 3) numGeoms += a.getMultiLine().size();
          if (a.getType() == 4) numGeoms += a.getMultiPolygon().size();
          if (a.getType() == 6) numGeoms += a.getMultiPoint().size();
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

}  // namespace sj

#endif
