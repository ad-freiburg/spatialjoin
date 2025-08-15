// Copyright 2024, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#include "WKTParse.h"

#ifdef __cpp_lib_string_view
#include <string_view>
#endif

using sj::WKTParser;
using sj::WKTParserBase;

// _____________________________________________________________________________
WKTParser::WKTParser(sj::Sweeper *sweeper, size_t numThreads)
    : WKTParserBase<ParseJob>(sweeper, numThreads) {
  for (size_t i = 0; i < _thrds.size(); i++) {
    _thrds[i] = std::thread(&WKTParser::processQueue, this, i);
  }
}

void WKTParser::processQueue(size_t t) {
  ParseBatch batch;
  while ((batch = _jobs.get()).size()) {
    sj::WriteBatch w;
    for (const auto &job : batch) {
      if (_cancelled) break;

      if (job.str.size()) {
        parseLine(job.str.c_str(), job.str.size(), job.line, t, w, job.side);
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
}

// _____________________________________________________________________________
void WKTParser::parseWKT(const char *c, size_t id, bool side) {
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
void WKTParser::parseWKT(const std::string &str, size_t id, bool side) {
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
void WKTParser::parse(char *c, size_t size, bool side) {
  size_t p = 0;

  while (true) {
    char *newLine = reinterpret_cast<char *>(memchr(c + p, '\n', size - p));

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
      memcpy(&_dangling[oldSize], (c + p), size - p);

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

namespace sj {
template class WKTParserBase<ParseJob>;
}
