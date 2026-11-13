// Copyright 2024, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#include "WKTParse.h"

#include <fcntl.h>
#include <unistd.h>
#ifndef SPATIALJOIN_NO_ZLIB
#include <zlib.h>
#endif
#ifndef SPATIALJOIN_NO_BZIP2
#include <bzlib.h>
#endif

#include <stdexcept>
#include <string>
#include <vector>
#ifdef __cpp_lib_string_view
#include <string_view>
#endif

#include "util/Misc.h"

// size of the read buffer used by the parseFile() methods
static const size_t READ_BUFF_SIZE = 1024 * 1024;
static const size_t MAX_BATCH_BYTES = 16 * 1024;

using sj::WKTParser;
using sj::WKTParserBase;

// _____________________________________________________________________________
WKTParser::WKTParser(sj::GeometryStore* store, sj::PreparedGeomMetaCb metaCb,
                     size_t numThreads, bool strict)
    : WKTParser(store, std::move(metaCb), numThreads, strict, false) {}

// _____________________________________________________________________________
WKTParser::WKTParser(sj::GeometryStore* store, sj::PreparedGeomMetaCb metaCb,
                     size_t numThreads, bool strict, bool encodeIds)
    : WKTParserBase<ParseJob>(store, std::move(metaCb), numThreads),
      _strict(strict),
      _encodeIds(encodeIds) {
  for (size_t i = 0; i < _thrds.size(); i++) {
    _thrds[i] = std::thread(&WKTParser::processQueue, this, i);
  }
}

// _____________________________________________________________________________
void WKTParser::processQueue(size_t t) {
  ParseBatch batch;
  while ((batch = _jobs.get()).size()) {
    sj::WriteBatch w;
    for (const auto& job : batch) {
      if (_cancelled) break;

      if (job.str.size()) {
        parseLine(job.str.c_str(), job.line, t, w, _encodeIds, _strict);
      } else {
        // parse point directly
        auto mercPoint = latLngToWebMerc(job.point);

        util::geo::I32Point addPoint{static_cast<int>(mercPoint.getX() * PREC),
                                     static_cast<int>(mercPoint.getY() * PREC)};
        std::string id = std::to_string(job.line);
        if (_encodeIds) id = idEnhance(id);
        addBox(_store->add(addPoint, id, w), t);
      }
    }

    _store->addBatch(w, t, _metaCb);
  }
}

// _____________________________________________________________________________
void WKTParser::parseWKT(const char* c, size_t id) {
  // dont parse empty strings
  if (*c == 0) return;

  _curBatch.reserve(10000);
  _curBatch.push_back({std::string(c), id, {0, 0}});

  if (_curBatch.size() > 10000) {
    _jobs.add(std::move(_curBatch));
    _curBatch.clear();
  }
}

// _____________________________________________________________________________
void WKTParser::parseWKT(const std::string& str, size_t id) {
  if (str.empty()) return;

  _curBatch.reserve(10000);
  _curBatch.push_back({str, id, {0, 0}});

  if (_curBatch.size() > 10000) {
    _jobs.add(std::move(_curBatch));
    _curBatch.clear();
  }
}

#ifdef __cpp_lib_string_view
// _____________________________________________________________________________
void WKTParser::parseWKT(const std::string_view str, size_t id) {
  // dont parse empty strings
  if (str.empty()) return;

  _curBatch.reserve(10000);
  _curBatch.push_back({std::string{str}, id, {0, 0}});

  if (_curBatch.size() > 10000) {
    _jobs.add(std::move(_curBatch));
    _curBatch.clear();
  }
}

#endif

// _____________________________________________________________________________
void WKTParser::parsePoint(util::geo::DPoint point, size_t id) {
  _curBatch.reserve(10000);
  _curBatch.push_back({"", id, point});

  if (_curBatch.size() > 10000) {
    _jobs.add(std::move(_curBatch));
    _curBatch.clear();
  }
}

// _____________________________________________________________________________
void WKTParser::parseFd(int fd) {
  std::vector<unsigned char, util::no_init_allocator<unsigned char>> buf(
      READ_BUFF_SIZE);
  ssize_t len;

  while ((len = util::readAll(fd, buf.data(), buf.size())) > 0) {
    parse(reinterpret_cast<char*>(buf.data()), len);
  }

  if (len < 0) throw std::runtime_error("Could not read input");
}

// _____________________________________________________________________________
void WKTParser::parseFile(const std::string& path) {
  if (util::endsWith(path, ".bz2")) {
    parseBz2File(path);
  } else if (util::endsWith(path, ".gz")) {
    parseGzFile(path);
  } else {
    int f = open(path.c_str(), O_RDONLY);
    if (f < 0) throw std::runtime_error("Could not open input file " + path);

    try {
      parseFd(f);
    } catch (const std::runtime_error&) {
      close(f);
      throw std::runtime_error("Could not read input file " + path);
    }

    close(f);
  }

  done();
}

// _____________________________________________________________________________
void WKTParser::parseGzFile(const std::string& path) {
#ifndef SPATIALJOIN_NO_ZLIB
  gzFile f = gzopen(path.c_str(), "r");
  if (f == Z_NULL) {
    throw std::runtime_error("Could not open input file " + path);
  }

  std::vector<unsigned char, util::no_init_allocator<unsigned char>> buf(
      READ_BUFF_SIZE);
  ssize_t len;

  while ((len = util::zreadAll(f, buf.data(), buf.size())) > 0) {
    parse(reinterpret_cast<char*>(buf.data()), len);
  }

  gzclose(f);

  if (len < 0) throw std::runtime_error("Could not read input file " + path);
#else
  throw std::runtime_error("Could not open input file " + path +
                           ", spatialjoin was compiled without gzip support");
#endif
}

// _____________________________________________________________________________
void WKTParser::parseBz2File(const std::string& path) {
#ifndef SPATIALJOIN_NO_BZIP2
  auto fh = fopen(path.c_str(), "r");
  if (!fh) throw std::runtime_error("Could not open input file " + path);

  int err;
  BZFILE* f = BZ2_bzReadOpen(&err, fh, 0, 0, NULL, 0);
  if (!f || err != BZ_OK) {
    fclose(fh);
    throw std::runtime_error("Could not open input file " + path);
  }

  std::vector<unsigned char, util::no_init_allocator<unsigned char>> buf(
      READ_BUFF_SIZE);
  ssize_t len;

  while ((len = util::bz2readAll(f, buf.data(), buf.size())) > 0) {
    parse(reinterpret_cast<char*>(buf.data()), len);
  }

  BZ2_bzReadClose(&err, f);
  fclose(fh);

  if (len < 0) throw std::runtime_error("Could not read input file " + path);
#else
  throw std::runtime_error("Could not open input file " + path +
                           ", spatialjoin was compiled without BZip2 support");
#endif
}

// _____________________________________________________________________________
void WKTParser::parse(char* c, size_t size) {
  size_t p = 0;
  size_t batchStart = 0;

  while (true) {
    char* newLine = reinterpret_cast<char*>(memchr(c + p, '\n', size - p));

    if (newLine) {
      size_t pNext = newLine - c;
      c[pNext] = 0;

      if (_dangling.size()) {
        _dangling += (c + p);  // guaranteed to be null terminated
        _curBatch.push_back({_dangling, _gid, {0, 0}});
        _dangling.clear();
      } else {
        if (*(c + p)) _curBatch.push_back({c + p, _gid, {0, 0}});

        if (_curBatch.size() > 1000 || pNext - batchStart > MAX_BATCH_BYTES) {
          _jobs.add(std::move(_curBatch));
          _curBatch.clear();  // std doesnt guarantee that after move
          _curBatch.reserve(1000);
          batchStart = pNext + 1;
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
}

namespace sj {
template class WKTParserBase<ParseJob>;
}
