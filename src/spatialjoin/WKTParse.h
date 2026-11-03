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
  util::geo::DPoint point;
};

inline bool operator==(const ParseJob& a, const ParseJob& b) {
  return a.line == b.line && a.str == b.str;
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
  WKTParserBase(sj::GeometryStore* store, sj::PreparedGeomMetaCb metaCb,
                size_t numThreads)
      : _store(store),
        _metaCb(std::move(metaCb)),
        _jobs(1000),
        _thrds(numThreads),
        _bboxes(numThreads),
        _numInvalid(numThreads),
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

  // the gid given to the next parsed line without an explicit id, used to
  // continue the line numbering across several parsers
  size_t getNextGid() const { return _gid; }
  void setNextGid(size_t gid) { _gid = gid; }

  // Number of geometries skipped because they were invalid or had an
  // unsupported CRS in strict mode. Only valid after done().
  size_t getNumInvalid() const {
    size_t ret = 0;
    for (auto n : _numInvalid) ret += n;
    return ret;
  }

  void done() {
    if (_done) return;
    _done = true;

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
  };

  static util::geo::I32Point projFunc(const util::geo::DPoint& p,
                                      util::geo::CRSType sourceCRS) {
    auto projPoint = projectToWebMerc(p, sourceCRS);
    return {static_cast<int>(projPoint.getX() * PREC),
            static_cast<int>(projPoint.getY() * PREC)};
  }

 protected:
  void parseLine(const char* c, size_t gid, size_t t, sj::WriteBatch& batch,
                 bool useIdEnhance, bool strict = false) {
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

    parseWKTGeometry(c, id, t, batch, strict);
  };

  // Parse a single WKT geometry (with an optional CRS IRI prefix) and add it to
  // the store under `id`. Unlike `parseLine`, `c` is only the WKT: there is
  // no tab-separated id prefix.
  // In strict mode, invalid WKT (or WKT with an unsupported CRS) is skipped
  // and counted in `_numInvalid`, instead of being parsed partially.
  void parseWKTGeometry(const char* c, const std::string& id, size_t t,
                        sj::WriteBatch& batch, bool strict) {
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

    // All parsing happens before anything is added to the store, so on an
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
        _bboxes[t] =
            util::geo::extendBox(_store->add(point, id, batch), _bboxes[t]);
      } else if (wktType == util::geo::WKTType::MULTIPOINT) {
        const auto& mp =
            multiPointFromWKTProj<int32_t>(c, endr, &projFunc, crsType, strict);
        checkWKTEnd(end);
        if (mp.size() != 0)
          _bboxes[t] =
              util::geo::extendBox(_store->add(mp, id, batch), _bboxes[t]);
      } else if (wktType == util::geo::WKTType::LINESTRING) {
        const auto& line =
            lineFromWKTProj<int32_t>(c, endr, &projFunc, crsType, strict);
        checkWKTEnd(end);
        if (line.size() > 1)
          _bboxes[t] =
              util::geo::extendBox(_store->add(line, id, batch), _bboxes[t]);
      } else if (wktType == util::geo::WKTType::MULTILINESTRING) {
        const auto& ml =
            multiLineFromWKTProj<int32_t>(c, endr, &projFunc, crsType, strict);
        checkWKTEnd(end);
        _bboxes[t] =
            util::geo::extendBox(_store->add(ml, id, batch), _bboxes[t]);
      } else if (wktType == util::geo::WKTType::POLYGON) {
        const auto& poly =
            polygonFromWKTProj<int32_t>(c, endr, &projFunc, crsType, strict);
        checkWKTEnd(end);
        if (poly.getOuter().size() > 1)
          _bboxes[t] =
              util::geo::extendBox(_store->add(poly, id, batch), _bboxes[t]);
      } else if (wktType == util::geo::WKTType::MULTIPOLYGON) {
        const auto& mp = multiPolygonFromWKTProj<int32_t>(c, endr, &projFunc,
                                                          crsType, strict);
        checkWKTEnd(end);
        if (mp.size())
          _bboxes[t] =
              util::geo::extendBox(_store->add(mp, id, batch), _bboxes[t]);
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
                _store->add(a.getPoint(), id, subId, target), colBox);
          if (a.getType() == 1)
            colBox = util::geo::extendBox(
                _store->add(a.getLine(), id, subId, target), colBox);
          if (a.getType() == 2)
            colBox = util::geo::extendBox(
                _store->add(a.getPolygon(), id, subId, target), colBox);
          if (a.getType() == 3)
            colBox = util::geo::extendBox(
                _store->add(a.getMultiLine(), id, subId, target), colBox);
          if (a.getType() == 4)
            colBox = util::geo::extendBox(
                _store->add(a.getMultiPolygon(), id, subId, target), colBox);
          if (a.getType() == 6)
            colBox = util::geo::extendBox(
                _store->add(a.getMultiPoint(), id, subId, target), colBox);

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
          _store->addMulti(std::move(members), colBox, id, batch);
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
  sj::PreparedGeomMetaCb _metaCb;

  std::vector<ParseJobT> _curBatch;

  util::geo::I32Box _bbox;

  util::JobQueue<std::vector<ParseJobT>> _jobs;
  std::vector<std::thread> _thrds;

  std::vector<util::geo::I32Box> _bboxes;
  std::vector<size_t> _numInvalid;

  std::atomic<bool> _cancelled;

  // whether done() was called
  bool _done = false;
};

class WKTParser : public WKTParserBase<ParseJob> {
 public:
  WKTParser(sj::GeometryStore* store, sj::PreparedGeomMetaCb metaCb,
            size_t numThreads, bool strict = false);
  void parse(char* c, size_t size);

  void parseFile(const std::string& path);
  void parseGzFile(const std::string& path);
  void parseBz2File(const std::string& path);
  void parseFd(int fd);

  void parseWKT(const char* c, size_t id);
  void parseWKT(const std::string& str, size_t id);
#ifdef __cpp_lib_string_view
  void parseWKT(const std::string_view str, size_t id);
#endif
  void parsePoint(util::geo::DPoint point, size_t id);

 protected:
  void processQueue(size_t t) override;

 private:
  bool _strict;
};

}  // namespace sj

#endif
