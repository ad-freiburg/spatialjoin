// Copyright 2024, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#ifndef SPATIALJOINS_WKTPARSE_H_
#define SPATIALJOINS_WKTPARSE_H_

#include <atomic>

#include "GeometryStore.h"
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

inline bool operator==(const ParseJob& a, const ParseJob& b) {
  return a.line == b.line && a.str == b.str && a.side == b.side;
}

inline std::string intInString(uint64_t oid) {
  unsigned char id[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int a = 0;
  uint64_t tmp;

  while (a < 8 && (oid >> (a * 8))) {
    tmp = (oid & (0xFFLL << (a * 8)));
    id[8 - a] = tmp >> (a * 8);
    a++;
  }

  a--;

  return std::string{reinterpret_cast<char*>(id + (8 - a)),
                     static_cast<size_t>(a + 1)};
}

inline std::string idEnhance(const std::string& id) {
  bool digitId = true;
  for (size_t i = 0; i < id.size(); i++) {
    if (!isdigit(id[i])) {
      digitId = false;
      break;
    }
  }

  if (digitId) {
    size_t i = std::atoll(id.c_str());
    return std::string("d") + intInString(i);
  } else {
    return std::string("s") + id;
  }
}

typedef std::vector<ParseJob> ParseBatch;

template <typename ParseJobT>
class WKTParserBase {
 public:
  WKTParserBase(sj::GeometryStore* store,
                sj::SweepEventList* events, size_t numThreads)
      : _store(store),
        _events(events),
        _jobs(1000),
        _thrds(numThreads),
        _bboxes(numThreads),
        _numInvalid(numThreads),
        _twoSided(numThreads),
        _cancelled(false){};
  ~WKTParserBase() {
    // graceful shutdown of all threads, should they be still running
    _cancelled = true;

    // end event
    _jobs.add({});

    // wait for all workers to finish
    for (auto& thr : _thrds)
      if (thr.joinable()) thr.join();
  };

  util::geo::I32Box getBoundingBox() const { return _bbox; }

  // Number of geometries skipped because they were invalid or had an
  // unsupported CRS in strict mode. Only valid after `done()`.
  size_t getNumInvalid() const {
    size_t ret = 0;
    for (auto n : _numInvalid) ret += n;
    return ret;
  }

  bool isTwoSided() const {
    bool ret = false;
    for (auto t : _twoSided) ret += t;
    return ret;
  }

  void done() {
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

    if (isTwoSided()) _store->setNumSides(2);
  };

  static util::geo::I32Point projFunc(const util::geo::DPoint& p,
                                      util::geo::CRSType sourceCRS) {
    auto projPoint = projectToWebMerc(p, sourceCRS);
    return {static_cast<int>(projPoint.getX() * PREC),
            static_cast<int>(projPoint.getY() * PREC)};
  }

 protected:
  void parseLine(const char* c, size_t gid, size_t t,
                 sj::WriteBatch& batch, bool side, bool useIdEnhance,
                 bool strict = false) {
    using namespace util::geo;

    // search for first occurance of tab
    const char* idp = strchr(c, '\t');

    std::string id;

    if (idp) {
      // if we have a tab, set id
      id.assign(c, idp - c);
      c = idp + 1;
    } else {
      id = std::to_string(gid);
    }

    if (useIdEnhance) id = idEnhance(id);

    // search for next tab occurance
    idp = strchr(c, '\t');

    if (idp) {
      side = atoi(c);
      c = idp + 1;
    }

    if (side) _twoSided[t] = true;

    parseWKTGeometry(c, id, t, batch, side, strict);
  };

  // Parse a single WKT geometry (with an optional CRS IRI prefix) and add it to
  // the sweeper under `id` and `side`. Unlike `parseLine`, `c` is only the WKT:
  // there is no tab-separated id or side prefix.
  // In strict mode, invalid WKT (or WKT with an unsupported CRS) is skipped
  // and counted in `_numInvalid`, instead of being parsed partially.
  void parseWKTGeometry(const char* c, const std::string& id, size_t t,
                        sj::WriteBatch& batch, bool side, bool strict) {
    using namespace util::geo;

    auto crsType = getCRSType(c, &c);

    // unsupported crs type, ignore
    if (crsType == util::geo::CRSType::UNSUPPORTED) {
      if (strict) _numInvalid[t]++;
      return;
    }

    auto wktType = getWKTType(c, &c, strict);
    if (strict && wktType == util::geo::WKTType::NONE) {
      _numInvalid[t]++;
      return;
    }

    // All parsing happens before anything is added to the sweeper, so on an
    // exception, nothing of this geometry has been added.
    const char* end = nullptr;
    // `checkWKTEnd(nullptr)` is a no-op, so trailing input is only rejected in
    // strict mode.
    const char** endr = strict ? &end : nullptr;
    try {
      if (wktType == util::geo::WKTType::POINT) {
        const auto& point =
            pointFromWKTProj<int32_t>(c, endr, &projFunc, crsType, strict);
        checkWKTEnd(end);
        _bboxes[t] = util::geo::extendBox(
            _store->add(point, id, side, batch), _bboxes[t]);
      } else if (wktType == util::geo::WKTType::MULTIPOINT) {
        const auto& mp =
            multiPointFromWKTProj<int32_t>(c, endr, &projFunc, crsType, strict);
        checkWKTEnd(end);
        if (mp.size() != 0)
          _bboxes[t] = util::geo::extendBox(
              _store->add(mp, id, side, batch), _bboxes[t]);
      } else if (wktType == util::geo::WKTType::LINESTRING) {
        const auto& line =
            lineFromWKTProj<int32_t>(c, endr, &projFunc, crsType, strict);
        checkWKTEnd(end);
        if (line.size() > 1)
          _bboxes[t] = util::geo::extendBox(
              _store->add(line, id, side, batch), _bboxes[t]);
      } else if (wktType == util::geo::WKTType::MULTILINESTRING) {
        const auto& ml =
            multiLineFromWKTProj<int32_t>(c, endr, &projFunc, crsType, strict);
        checkWKTEnd(end);
        _bboxes[t] = util::geo::extendBox(
            _store->add(ml, id, side, batch), _bboxes[t]);
      } else if (wktType == util::geo::WKTType::POLYGON) {
        const auto& poly =
            polygonFromWKTProj<int32_t>(c, endr, &projFunc, crsType, strict);
        checkWKTEnd(end);
        if (poly.getOuter().size() > 1)
          _bboxes[t] = util::geo::extendBox(
              _store->add(poly, id, side, batch), _bboxes[t]);
      } else if (wktType == util::geo::WKTType::MULTIPOLYGON) {
        const auto& mp = multiPolygonFromWKTProj<int32_t>(c, endr, &projFunc,
                                                          crsType, strict);
        checkWKTEnd(end);
        if (mp.size())
          _bboxes[t] = util::geo::extendBox(
              _store->add(mp, id, side, batch), _bboxes[t]);
      } else if (wktType == util::geo::WKTType::COLLECTION) {
        const auto& col =
            collectionFromWKTProj<int32_t>(c, endr, &projFunc, crsType, strict);
        checkWKTEnd(end);

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

        // the members of a multi collection are kept together
        sj::WriteBatch members;
        auto& target = numGeoms > 1 ? members : batch;
        util::geo::I32Box colBox;

        for (const auto& a : col) {
          if (a.getType() == 0)
            colBox = util::geo::extendBox(
                _store->add(a.getPoint(), id, subId, side, target), colBox);
          if (a.getType() == 1)
            colBox = util::geo::extendBox(
                _store->add(a.getLine(), id, subId, side, target), colBox);
          if (a.getType() == 2)
            colBox = util::geo::extendBox(
                _store->add(a.getPolygon(), id, subId, side, target), colBox);
          if (a.getType() == 3)
            colBox = util::geo::extendBox(
                _store->add(a.getMultiLine(), id, subId, side, target),
                colBox);
          if (a.getType() == 4)
            colBox = util::geo::extendBox(
                _store->add(a.getMultiPolygon(), id, subId, side, target),
                colBox);
          if (a.getType() == 6)
            colBox = util::geo::extendBox(
                _store->add(a.getMultiPoint(), id, subId, side, target),
                colBox);

          // the members of a multi element each take their own sub ID
          if (a.getType() == 3)
            subId += a.getMultiLine().size();
          else if (a.getType() == 4)
            subId += a.getMultiPolygon().size();
          else if (a.getType() == 6)
            subId += a.getMultiPoint().size();
          else
            subId++;
        }

        _bboxes[t] = util::geo::extendBox(colBox, _bboxes[t]);
        if (numGeoms > 1)
          _store->addMulti(std::move(members), colBox, id, side, batch);
      }
    } catch (const util::geo::WKTParseException&) {
      if (!strict) throw;
      _numInvalid[t]++;
    }
  };
  virtual void processQueue(size_t t) = 0;
  size_t _gid = 1;
  std::string _dangling;

  sj::GeometryStore* _store;
  sj::SweepEventList* _events;

  std::vector<ParseJobT> _curBatch;

  util::geo::I32Box _bbox;

  util::JobQueue<std::vector<ParseJobT>> _jobs;
  std::vector<std::thread> _thrds;

  std::vector<util::geo::I32Box> _bboxes;
  std::vector<size_t> _numInvalid;
  std::vector<bool> _twoSided;

  std::atomic<bool> _cancelled;
};

class WKTParser : public WKTParserBase<ParseJob> {
 public:
  WKTParser(sj::GeometryStore* store, sj::SweepEventList* events,
            size_t numThreads, bool strict = false);
  void parse(char* c, size_t size, bool side);
  void parseWKT(const char* c, size_t id, bool side);
  void parseWKT(const std::string& str, size_t id, bool side);
#ifdef __cpp_lib_string_view
  void parseWKT(const std::string_view str, size_t id, bool side);
#endif
  void parsePoint(util::geo::DPoint point, size_t id, bool side);

 protected:
  void processQueue(size_t t) override;

 private:
  bool _strict;
};

}  // namespace sj

#endif
