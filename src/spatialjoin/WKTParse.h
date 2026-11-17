// Copyright 2024, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#ifndef SPATIALJOINS_WKTPARSE_H_
#define SPATIALJOINS_WKTPARSE_H_

#include <atomic>
#include <limits>

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
  bool digitId = !id.empty() && !(id[0] == '0' && id.size() > 1);
  uint64_t num = 0;
  for (size_t i = 0; digitId && i < id.size(); i++) {
    if (!isdigit(id[i])) {
      digitId = false;
      break;
    }
    uint64_t digit = id[i] - '0';
    if (num > (std::numeric_limits<uint64_t>::max() - digit) / 10) {
      // would overflow
      digitId = false;
      break;
    }
    num = num * 10 + digit;
  }

  if (digitId) {
    return std::string("d") + intInString(num);
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

  // the gid given to the next parsed line without an explicit id, used to
  // continue the line numbering across several parsers
  size_t getNextGid() const { return _gid; }
  void setNextGid(size_t gid) { _gid = gid; }

  // full bounding box of all geometries parsed by this
  util::geo::I32Box getBoundingBox() const { return _bbox; }

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
    for (const auto& box : _bboxes) _bbox = util::geo::extendBox(box, _bbox);
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
        addBox(_store->add(point, id, batch), t);
      } else if (wktType == util::geo::WKTType::MULTIPOINT) {
        const auto& mp =
            multiPointFromWKTProj<int32_t>(c, endr, &projFunc, crsType, strict);
        checkWKTEnd(end);
        addBox(_store->add(mp, id, batch), t);
      } else if (wktType == util::geo::WKTType::LINESTRING) {
        const auto& line =
            lineFromWKTProj<int32_t>(c, endr, &projFunc, crsType, strict);
        checkWKTEnd(end);
        addBox(_store->add(line, id, batch), t);
      } else if (wktType == util::geo::WKTType::MULTILINESTRING) {
        const auto& ml =
            multiLineFromWKTProj<int32_t>(c, endr, &projFunc, crsType, strict);
        checkWKTEnd(end);
        addBox(_store->add(ml, id, batch), t);
      } else if (wktType == util::geo::WKTType::POLYGON) {
        const auto& poly =
            polygonFromWKTProj<int32_t>(c, endr, &projFunc, crsType, strict);
        checkWKTEnd(end);
        addBox(_store->add(poly, id, batch), t);
      } else if (wktType == util::geo::WKTType::MULTIPOLYGON) {
        const auto& mp = multiPolygonFromWKTProj<int32_t>(c, endr, &projFunc,
                                                          crsType, strict);
        checkWKTEnd(end);
        addBox(_store->add(mp, id, batch), t);
      } else if (wktType == util::geo::WKTType::COLLECTION) {
        const auto& col =
            collectionFromWKTProj<int32_t>(c, endr, &projFunc, crsType, strict);
        checkWKTEnd(end);

        addBox(_store->add(col, id, batch), t);
      }
    } catch (const util::geo::WKTParseException&) {
      if (!strict) throw;
      _numInvalid[t]++;
    }
  };

  void addBox(const util::geo::I32Box& box, size_t t) {
    if (!box.isNull()) _bboxes[t] = util::geo::extendBox(box, _bboxes[t]);
  }

  virtual void processQueue(size_t t) = 0;
  size_t _gid = 1;
  std::string _dangling;

  sj::GeometryStore* _store;
  sj::PreparedGeomMetaCb _metaCb;

  std::vector<ParseJobT> _curBatch;

  util::JobQueue<std::vector<ParseJobT>> _jobs;
  std::vector<std::thread> _thrds;

  // the bounding boxes of the geometries parsed by each thread, and their
  // union, computed in done()
  std::vector<util::geo::I32Box> _bboxes;
  util::geo::I32Box _bbox;

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
  WKTParser(sj::GeometryStore* store, sj::PreparedGeomMetaCb metaCb,
            size_t numThreads, bool strict, bool encodeIds);

  void processQueue(size_t t) override;

 private:
  bool _strict;
  bool _encodeIds;
};

// A WKTParser which encodes the geometry ids with idEnhance()
class IDEncodingWKTParser : public WKTParser {
 public:
  IDEncodingWKTParser(sj::GeometryStore* store, sj::PreparedGeomMetaCb metaCb,
                      size_t numThreads, bool strict = false)
      : WKTParser(store, std::move(metaCb), numThreads, strict, true) {}
};

}  // namespace sj

#endif
