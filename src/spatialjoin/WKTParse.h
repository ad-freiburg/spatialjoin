// Copyright 2024, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#ifndef SPATIALJOINS_WKTPARSE_H_
#define SPATIALJOINS_WKTPARSE_H_

#include "Sweeper.h"
#include "util/geo/Geo.h"
#include "util/log/Log.h"

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

typedef std::vector<ParseJob> ParseBatch;

class WKTParser {
 public:
  WKTParser(sj::Sweeper* sweeper, size_t numThreads);
  void parse(char* c, size_t size, bool side);
  void parseWKT(const char* c, size_t id, bool side);
  void parsePoint(util::geo::DPoint point, size_t id, bool side);

  util::geo::I32Box getBoundingBox() const { return _bbox;}

  void done();

 private:
  util::geo::I32Line parseLineString(const char* c, const char** endr);
  util::geo::I32Point parsePoint(const char* c);
  util::geo::I32Polygon parsePolygon(const char* c, const char** endr);
  util::geo::I32MultiLine parseMultiLineString(const char* c,
                                                      const char** endr);
  util::geo::I32MultiPolygon parseMultiPolygon(const char* c,
                                                      const char** endr);
  std::pair<util::geo::I32Collection, size_t> parseGeometryCollection(
      const char* c);
  void parseLine(char* c, size_t len, size_t gid, size_t t,
                                 sj::WriteBatch& batch,
                                 bool side);
  void processQueue(size_t t);
  size_t _gid = 1;
  std::string _dangling;

  sj::Sweeper* _sweeper;

  ParseBatch _curBatch;

  util::geo::I32Box _bbox;

  util::JobQueue<ParseBatch> _jobs;
  std::vector<std::thread> _thrds;

  std::vector<util::geo::I32Box> _bboxes;

};

}  // namespace sj

#endif
