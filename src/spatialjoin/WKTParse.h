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
  WKTParserBase(sj::Sweeper *sweeper, size_t numThreads);
  ~WKTParserBase();

  util::geo::I32Box getBoundingBox() const { return _bbox; }

  void done();

  static util::geo::I32Point projFunc(const util::geo::DPoint &p) {
    auto projPoint = latLngToWebMerc(p);
    return {static_cast<int>(projPoint.getX() * PREC),
            static_cast<int>(projPoint.getY() * PREC)};
  }

protected:
  void parseLine(char *c, size_t len, size_t gid, size_t t,
                 sj::WriteBatch &batch, bool side);
  virtual void processQueue(size_t t);
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
};

} // namespace sj

#endif
