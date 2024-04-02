#include <bzlib.h>
#include <stdio.h>
#include <unistd.h>

#include <algorithm>
#include <cassert>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <set>

#include "BoxIds.h"
#include "IntervalIdx.h"
#include "Sweeper.h"
#include "util/Misc.h"
#include "util/log/Log.h"

using sj::GeomType;
using sj::Sweeper;
using sj::boxids::boxIdIsect;
using sj::boxids::getBoxId;
using sj::boxids::getBoxIds;
using sj::boxids::packBoxIds;
using sj::boxids::BoxIdList;
using util::geo::I32XSortedPolygon;
using util::geo::webMercToLatLng;

// _____________________________________________________________________________
void Sweeper::add(const util::geo::I32MultiPolygon& a, const std::string& gid) {
  uint16_t subid = 0;  // a subid of 0 means "single polygon"
  if (a.size() > 1) subid = 1;

  add(a, gid, subid);
}

// _____________________________________________________________________________
void Sweeper::add(const util::geo::I32MultiLine& a, const std::string& gid) {
  uint16_t subid = 0;  // a subid of 0 means "single line"
  if (a.size() > 1) subid = 1;

  add(a, gid, subid);
}

// _____________________________________________________________________________
void Sweeper::add(const util::geo::I32MultiPoint& a, const std::string& gid) {
  uint16_t subid = 0;  // a subid of 0 means "single point"
  if (a.size() > 1) subid = 1;

  add(a, gid, subid);
}

// _____________________________________________________________________________
size_t Sweeper::add(const util::geo::I32MultiPolygon& a, const std::string& gid,
                    size_t subid) {
  if (subid > 0) _subSizes[gid] = _subSizes[gid] + a.size();
  for (const auto& poly : a) {
    add(poly, gid, subid);
    subid++;
  }
  return subid;
}

// _____________________________________________________________________________
size_t Sweeper::add(const util::geo::I32MultiLine& a, const std::string& gid,
                    size_t subid) {
  if (subid > 0) _subSizes[gid] = _subSizes[gid] + a.size();
  for (const auto& line : a) {
    add(line, gid, subid);
    subid++;
  }
  return subid;
}

// _____________________________________________________________________________
size_t Sweeper::add(const util::geo::I32MultiPoint& a, const std::string& gid,
                    size_t subid) {
  if (subid > 0) _subSizes[gid] = _subSizes[gid] + a.size();
  for (const auto& point : a) {
    add(point, gid, subid);
    subid++;
  }
  return subid;
}

// _____________________________________________________________________________
void Sweeper::add(const util::geo::I32Polygon& poly, const std::string& gid) {
  add(poly, gid, 0);
}

// _____________________________________________________________________________
void Sweeper::add(const util::geo::I32Polygon& poly, const std::string& gid,
                  size_t subid) {
  const auto& box = util::geo::getBoundingBox(poly);
  const I32XSortedPolygon spoly(poly);
  double areaSize = util::geo::area(poly);
  BoxIdList boxIds;
  if (_cfg.useBoxIds) boxIds = packBoxIds(getBoxIds(spoly, poly, box, areaSize));

  if (!_cfg.useArea) areaSize = 0;

  size_t id = _areaCache.add(Area{
      spoly,
      box,
      gid,
      subid,
      areaSize,
      boxIds,
  });

  diskAdd({id, box.getLowerLeft().getY(), box.getUpperRight().getY(),
           box.getLowerLeft().getX(), false, POLYGON});
  diskAdd({id, box.getLowerLeft().getY(), box.getUpperRight().getY(),
           box.getUpperRight().getX(), true, POLYGON});
  _curSweepId++;

  if (_curSweepId % 1000000 == 0) LOGTO(INFO, std::cerr) << "@ " << _curSweepId;
}

// _____________________________________________________________________________
void Sweeper::add(const util::geo::I32Line& line, const std::string& gid) {
  add(line, gid, 0);
}

// _____________________________________________________________________________
void Sweeper::add(const util::geo::I32Line& line, const std::string& gid,
                  size_t subid) {
  const auto& box = util::geo::getBoundingBox(line);
  BoxIdList boxIds;
  if (_cfg.useBoxIds) boxIds = packBoxIds(getBoxIds(line, box));

  if (line.size() == 2 && (!_cfg.useBoxIds || boxIds.front().first == 1) && subid == 0) {
    // simple line
    size_t id =
        _simpleLineCache.add(SimpleLine{line.front(), line.back(), gid});

    diskAdd({id, box.getLowerLeft().getY(), box.getUpperRight().getY(),
             box.getLowerLeft().getX(), false, SIMPLE_LINE});
    diskAdd({id, box.getLowerLeft().getY(), box.getUpperRight().getY(),
             box.getUpperRight().getX(), true, SIMPLE_LINE});
  } else {
    const util::geo::I32XSortedLine sline(line);

    size_t id = _lineCache.add(Line{
        sline, box, gid, subid, boxIds,  //{}  // dummy
    });

    diskAdd({id, box.getLowerLeft().getY(), box.getUpperRight().getY(),
             box.getLowerLeft().getX(), false, LINE});
    diskAdd({id, box.getLowerLeft().getY(), box.getUpperRight().getY(),
             box.getUpperRight().getX(), true, LINE});
  }
  _curSweepId++;

  if (_curSweepId % 1000000 == 0) LOGTO(INFO, std::cerr) << "@ " << _curSweepId;
}

// _____________________________________________________________________________
void Sweeper::add(const util::geo::I32Point& point, const std::string& gid) {
  add(point, gid, 0);
}

// _____________________________________________________________________________
void Sweeper::add(const util::geo::I32Point& point, const std::string& gid,
                  size_t subid) {
  size_t id = _pointCache.add(Point{gid, subid});
  diskAdd({id, point.getY(), point.getY(), point.getX(), false, POINT});
  diskAdd({id, point.getY(), point.getY(), point.getX(), true, POINT});
  _curSweepId++;

  if (_curSweepId % 1000000 == 0) LOGTO(INFO, std::cerr) << "@ " << _curSweepId;
}

// _____________________________________________________________________________
void Sweeper::flush() {
  write(_file, _outBuffer, _obufpos);

  _pointCache.flush();
  _areaCache.flush();
  _lineCache.flush();
  _simpleLineCache.flush();
  LOGTO(INFO, std::cerr) << "Sorting...";

  // now the individial parts are sorted
  std::string newFName = _cache + "/.sortTmp";
  int newFile = open(newFName.c_str(), O_RDWR | O_CREAT, 0666);

  if (newFile < 0) {
    throw std::runtime_error("Could not open temporary file " + newFName);
    exit(1);
  }

#ifdef __unix__
  posix_fadvise(newFile, 0, 0, POSIX_FADV_SEQUENTIAL);
#endif
  util::externalSort(_file, newFile, sizeof(BoxVal), _curSweepId * 2, boxCmp);

  // remove old file
  std::remove((_cache + "/events").c_str());
  std::rename((_cache + "/.sortTmp").c_str(), (_cache + "/events").c_str());

  _file = open((_cache + "/events").c_str(), O_RDONLY);

#ifdef __unix__
  posix_fadvise(newFile, 0, 0, POSIX_FADV_SEQUENTIAL);
#endif

  LOGTO(INFO, std::cerr) << "...done";
}

// _____________________________________________________________________________
void Sweeper::diskAdd(const BoxVal& bv) {
  memcpy(_outBuffer + _obufpos, &bv, sizeof(BoxVal));
  _obufpos += sizeof(BoxVal);

  if (_obufpos >= BUFFER_S) {
    write(_file, _outBuffer, BUFFER_S);
    _obufpos = 0;
  }
}

// _____________________________________________________________________________
void Sweeper::sweep() {
  // start at beginning of _file
  lseek(_file, 0, SEEK_SET);

  const size_t batchSize = 100000;
  JobBatch curBatch;

  const size_t RBUF_SIZE = 100000;
  char* buf = new char[sizeof(BoxVal) * RBUF_SIZE];

  sj::IntervalIdx<int32_t, SweepVal> actives;

  _rawFiles = {}, _files = {}, _outBufPos = {}, _outBuffers = {};

  _files.resize(_cfg.numThreads);
  _rawFiles.resize(_cfg.numThreads);
  _outBufPos.resize(_cfg.numThreads);
  _outBuffers.resize(_cfg.numThreads);
  _stats.resize(_cfg.numThreads);

  size_t counts = 0, checkCount = 0, jj = 0, checkPairs = 0;
  auto t = TIME();

  prepareOutputFiles();

  // fire up worker threads for geometry checking
  std::vector<std::thread> thrds(_cfg.numThreads);
  for (size_t i = 0; i < thrds.size(); i++)
    thrds[i] = std::thread(&Sweeper::processQueue, this, i);

  size_t len;
  while ((len = read(_file, buf, sizeof(BoxVal) * RBUF_SIZE)) != 0) {
    for (size_t i = 0; i < len; i += sizeof(BoxVal)) {
      auto cur = reinterpret_cast<const BoxVal*>(buf + i);

      if (!cur->out) {
        // IN event

        // dont add POINTs to active - immediately treat them as "out" (keeps
        // active set smaller)
        if (cur->type != POINT) {
          actives.insert({cur->loY, cur->upY}, {cur->id, cur->type});
        }

        if (++jj % 100000 == 0) {
          auto lon = webMercToLatLng<double>((1.0 * cur->val) / PREC, 0).getX();
          checkCount += checkPairs;
          LOGTO(INFO, std::cerr)
              << jj << " / " << _curSweepId << " ("
              << (((1.0 * jj) / (1.0 * _curSweepId)) * 100) << "%, "
              << (100000.0 / double(TOOK(t))) * 1000000000.0 << " geoms/s, "
              << (checkPairs / double(TOOK(t))) * 1000000000.0
              << " pairs/s), avg. " << ((1.0 * checkCount) / (1.0 * counts))
              << " checks/geom, sweepLon=" << lon << "°, |A|=" << actives.size()
              << ", |JQ|=" << _jobs.size() << " (x" << batchSize << ")";
          t = TIME();
          checkPairs = 0;
        }
      } else {
        // OUT event

        // POINTs are never put onto active, so we don't have to remove them
        if (cur->type != POINT) {
          actives.erase({cur->loY, cur->upY}, {cur->id, cur->type});
        }

        counts++;

        fillBatch(&curBatch, &actives, cur);

        if (curBatch.size() > batchSize) {
          checkPairs += curBatch.size();
          _jobs.add(std::move(curBatch));
          curBatch.clear();  // std doesnt guarantee that after move
          curBatch.reserve(batchSize + 100);
        }
      }
    }
  }

  delete[] buf;

  if (curBatch.size()) _jobs.add(std::move(curBatch));

  // the DONE element on the job queue to signal all threads to shut down
  _jobs.add({});

  // wait for all workers to finish
  for (auto& thr : thrds) thr.join();

  flushOutputFiles();

  // aggregate total stats
  Stats sum;
  for (auto s : _stats) sum += s;

  std::cerr << sum.toString() << std::endl;
}

// _____________________________________________________________________________
std::pair<bool, bool> Sweeper::check(const Area* a, const Area* b,
                                     size_t t) const {
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect(a->boxIds, b->boxIds);
    _stats[t].timeBoxIdIsectAreaArea += TOOK(ts);

    // all boxes of a are fully contained in b, we intersect and we are
    // contained
    if (r.first == a->boxIds.front().first) return {1, 1};

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return {0, 0};

    // at least one box is fully contained, so we intersect
    // but the number of fully and partially contained boxed is smaller
    // than the number of boxes of A, so we cannot possible by contained
    if (r.first + r.second < a->boxIds.front().first && r.second > 0) {
      return {1, 0};
    }
  }

  auto ts = TIME();
  auto res = util::geo::intersectsContains(a->geom, a->box, a->area, b->geom,
                                           b->box, b->area);
  _stats[t].timeFullGeoCheckAreaArea += TOOK(ts);
  _stats[t].fullGeoChecksAreaArea++;
  return res;
}

// _____________________________________________________________________________
std::pair<bool, bool> Sweeper::check(const Line* a, const Area* b,
                                     size_t t) const {
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect(a->boxIds, b->boxIds);
    _stats[t].timeBoxIdIsectAreaLine += TOOK(ts);

    // all boxes of a are fully contained in b, we intersect and we are
    // contained
    if (r.first == a->boxIds.front().first) return {1, 1};

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return {0, 0};

    // at least one box is fully contained, so we intersect
    // but the number of fully and partially contained boxed is smaller
    // than the number of boxes of A, so we cannot possible by contained
    if (r.first + r.second < a->boxIds.front().first && r.second > 0) {
      return {1, 0};
    }
  }

  auto ts = TIME();
  auto res = util::geo::intersectsContains(a->geom, a->box, b->geom, b->box);
  _stats[t].timeFullGeoCheckAreaLine += TOOK(ts);
  _stats[t].fullGeoChecksAreaLine++;

  return res;
}

// _____________________________________________________________________________
bool Sweeper::check(const Line* a, const Line* b, size_t t) const {
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect(a->boxIds, b->boxIds);
    _stats[t].timeBoxIdIsectLineLine += TOOK(ts);

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return false;
  }

  auto ts = TIME();
  auto res = util::geo::intersects(a->geom, b->geom, a->box, b->box);
  _stats[t].timeFullGeoCheckLineLine += TOOK(ts);
  _stats[t].fullGeoChecksLineLine++;

  return res;
}

// _____________________________________________________________________________
std::pair<bool, bool> Sweeper::check(const SimpleLine* a, const Area* b,
                                     size_t t) const {
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect({{1, 0}, {getBoxId(a->a), 0}}, b->boxIds);
    _stats[t].timeBoxIdIsectAreaLine += TOOK(ts);

    // all boxes of a are fully contained in b, we intersect and we are
    // contained
    if (r.first == 1) return {1, 1};

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return {0, 0};
  }

  auto ts = TIME();
  auto res = util::geo::intersectsContains(
      util::geo::I32XSortedLine(util::geo::LineSegment<int32_t>(a->a, a->b)),
      util::geo::getBoundingBox(util::geo::LineSegment<int32_t>(a->a, a->b)),
      b->geom, b->box);
  _stats[t].timeFullGeoCheckAreaLine += TOOK(ts);
  _stats[t].fullGeoChecksAreaLine++;
  return res;
}

// _____________________________________________________________________________
bool Sweeper::check(const Line* a, const SimpleLine* b, size_t t) const {
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect(a->boxIds, {{1, 0}, {getBoxId(b->a), 0}});
    _stats[t].timeBoxIdIsectLineLine += TOOK(ts);

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return false;
  }

  auto ts = TIME();
  auto res = util::geo::intersects(
      a->geom,
      util::geo::I32XSortedLine(util::geo::LineSegment<int32_t>(b->a, b->b)),
      a->box,
      util::geo::getBoundingBox(util::geo::LineSegment<int32_t>(b->a, b->b)));
  _stats[t].timeFullGeoCheckLineLine += TOOK(ts);
  _stats[t].fullGeoChecksLineLine++;
  return res;
}

// _____________________________________________________________________________
bool Sweeper::check(const util::geo::I32Point& a, const Area* b,
                    size_t t) const {
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect({{1, 0}, {getBoxId(a), 0}}, b->boxIds);
    _stats[t].timeBoxIdIsectAreaPoint += TOOK(ts);

    // all boxes of a are fully contained in b, we are contained
    if (r.first) return true;

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return false;
  }

  auto ts = TIME();
  auto res = util::geo::contains(a, b->geom);
  _stats[t].timeFullGeoCheckAreaPoint += TOOK(ts);
  _stats[t].fullGeoChecksAreaPoint++;

  return res;
}

// ____________________________________________________________________________
void Sweeper::writeContains(size_t t, const std::string& a,
                            const std::string& b) {
  writeRel(t, a, b, _cfg.sepContains);
}

// ____________________________________________________________________________
void Sweeper::writeRel(size_t t, const std::string& a, const std::string& b,
                       const std::string& pred) {
  auto ts = TIME();
  std::string out = _cfg.pairStart + a + pred + b + _cfg.pairEnd;

  if (_out.size()) {
    if (_outBufPos[t] + out.size() >= BUFFER_S_PAIRS) {
      int err = 0;
      BZ2_bzWrite(&err, _files[t], _outBuffers[t], _outBufPos[t]);
      if (err == BZ_IO_ERROR) {
        BZ2_bzWriteClose(&err, _files[t], 0, 0, 0);
        throw std::runtime_error("Could not write to file.");
      }
      _outBufPos[t] = 0;
    }

    memcpy(_outBuffers[t] + _outBufPos[t], out.c_str(), out.size());
    _outBufPos[t] += out.size();
  } else {
    fputs(out.c_str(), stdout);
  }

  _stats[t].timeWrite += TOOK(ts);
}

// ____________________________________________________________________________
void Sweeper::writeIntersect(size_t t, const std::string& a,
                             const std::string& b) {
  writeRel(t, a, b, _cfg.sepIsect);
}

// ____________________________________________________________________________
void Sweeper::doCheck(const BoxVal cur, const SweepVal sv, size_t t) {
  if (cur.type == POLYGON && sv.type == POLYGON) {
    auto ts = TIME();

    auto a = _areaCache.get(cur.id, t);
    auto b = _areaCache.get(sv.id, t);

    _stats[t].timeGeoCacheRetrievalArea += TOOK(ts);

    auto res = check(a.get(), b.get(), t);

    if (res.first) {
      writeIntersect(t, a->id, b->id);
      writeIntersect(t, b->id, a->id);

      // check for contains in other direction
      // NOTE: this is not necessary if we assume that "lying on the border"
      // does not mean contain, then we can always be sure that if a came out
      // first of the sweep line process, b cannot be contained in a
      // auto resOther = check(b.get(), a.get());
      // if (res.second) {
      // writeContains(t, b->id, a->id);
      // }
    }

    if (res.second) {
      if (a->subId > 0) {
        // a is a multigeometry, *all* its parts must be contained.
        // we cache them, and write them as soon as we know that yes,
        // they are all contained
        writeContainsMulti(t, b->id, a->id, a->subId);
      } else {
        writeContains(t, b->id, a->id);
      }
    }
  } else if (cur.type == LINE && sv.type == POLYGON) {
    auto ts = TIME();
    auto a = _lineCache.get(cur.id, t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);
    ts = TIME();
    auto b = _areaCache.get(sv.id, t);
    _stats[t].timeGeoCacheRetrievalArea += TOOK(ts);

    auto res = check(a.get(), b.get(), t);

    if (res.first) {
      writeIntersect(t, a->id, b->id);
      writeIntersect(t, b->id, a->id);
    }

    if (res.second) {
      writeContains(t, b->id, a->id);
    }
  } else if (cur.type == SIMPLE_LINE && sv.type == POLYGON) {
    auto ts = TIME();
    auto a = _simpleLineCache.get(cur.id, t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);
    ts = TIME();
    auto b = _areaCache.get(sv.id, t);
    _stats[t].timeGeoCacheRetrievalArea += TOOK(ts);

    auto res = check(a.get(), b.get(), t);

    if (res.first) {
      writeIntersect(t, a->id, b->id);
      writeIntersect(t, b->id, a->id);
    }

    if (res.second) {
      writeContains(t, b->id, a->id);
    }
  } else if (cur.type == POLYGON && sv.type == LINE) {
    auto ts = TIME();
    auto a = _areaCache.get(cur.id, t);
    _stats[t].timeGeoCacheRetrievalArea += TOOK(ts);
    ts = TIME();
    auto b = _lineCache.get(sv.id, t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    auto res = check(b.get(), a.get(), t);

    if (res.first) {
      writeIntersect(t, a->id, b->id);
      writeIntersect(t, b->id, a->id);
    }

    if (res.second) {
      writeContains(t, a->id, b->id);
    }
  } else if (cur.type == POLYGON && sv.type == SIMPLE_LINE) {
    auto ts = TIME();
    auto a = _areaCache.get(cur.id, t);
    _stats[t].timeGeoCacheRetrievalArea += TOOK(ts);
    ts = TIME();
    auto b = _simpleLineCache.get(sv.id, t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    auto res = check(b.get(), a.get(), t);

    if (res.first) {
      writeIntersect(t, a->id, b->id);
      writeIntersect(t, b->id, a->id);
    }

    if (res.second) {
      writeContains(t, a->id, b->id);
    }
  } else if (cur.type == LINE && sv.type == LINE) {
    auto ts = TIME();
    auto a = _lineCache.get(cur.id, t);
    auto b = _lineCache.get(sv.id, t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    auto res = check(a.get(), b.get(), t);

    if (res) {
      writeIntersect(t, a->id, b->id);
      writeIntersect(t, b->id, a->id);
    }
  } else if (cur.type == LINE && sv.type == SIMPLE_LINE) {
    auto ts = TIME();
    auto a = _lineCache.get(cur.id, t);
    auto b = _simpleLineCache.get(sv.id, t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    auto res = check(a.get(), b.get(), t);

    if (res) {
      writeIntersect(t, a->id, b->id);
      writeIntersect(t, b->id, a->id);
    }
  } else if (cur.type == SIMPLE_LINE && sv.type == LINE) {
    auto ts = TIME();
    auto a = _simpleLineCache.get(cur.id, t);
    auto b = _lineCache.get(sv.id, t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    auto res = check(b.get(), a.get(), t);

    if (res) {
      writeIntersect(t, a->id, b->id);
      writeIntersect(t, b->id, a->id);
    }
  } else if (cur.type == SIMPLE_LINE && sv.type == SIMPLE_LINE) {
    auto ts = TIME();
    auto a = _simpleLineCache.get(cur.id, t);
    auto b = _simpleLineCache.get(sv.id, t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    if (util::geo::intersects(util::geo::LineSegment<int32_t>(a->a, a->b),
                              util::geo::LineSegment<int32_t>(b->a, b->b))) {
      writeIntersect(t, a->id, b->id);
      writeIntersect(t, b->id, a->id);
    }
  } else if (cur.type == POINT && sv.type == POLYGON) {
    auto a = util::geo::I32Point(cur.val, cur.loY);
    auto ts = TIME();
    auto b = _areaCache.get(sv.id, t);
    _stats[t].timeGeoCacheRetrievalArea += TOOK(ts);

    auto res = check(a, b.get(), t);

    if (res) {
      auto a = _pointCache.get(cur.id, t);
      writeContains(t, b->id, a->id);
      writeIntersect(t, a->id, b->id);
      writeIntersect(t, b->id, a->id);
    }
  }
}

// _____________________________________________________________________________
void Sweeper::flushOutputFiles() {
  if (_out.size()) {
    for (size_t i = 0; i < _cfg.numThreads; i++) {
      int err = 0;
      BZ2_bzWrite(&err, _files[i], _outBuffers[i], _outBufPos[i]);
      if (err == BZ_IO_ERROR) {
        BZ2_bzWriteClose(&err, _files[i], 0, 0, 0);
        throw std::runtime_error("Could not write to file.");
      }
      BZ2_bzWriteClose(&err, _files[i], 0, 0, 0);
      fclose(_rawFiles[i]);
    }

    // merge files into first file
    std::ofstream out(_cache + "/.rels0", std::ios_base::binary |
                                              std::ios_base::app |
                                              std::ios_base::ate);
    for (size_t i = 1; i < _cfg.numThreads; i++) {
      std::string fName = _cache + "/.rels" + std::to_string(i);
      std::ifstream ifCur(fName, std::ios_base::binary);
      out << ifCur.rdbuf();
      std::remove(fName.c_str());
    }

    // move first file to output file
    std::rename((_cache + "/.rels0").c_str(), _out.c_str());
  }
}

// _____________________________________________________________________________
void Sweeper::prepareOutputFiles() {
  if (_out.size()) {
    for (size_t i = 0; i < _cfg.numThreads; i++) {
      _rawFiles[i] =
          fopen((_cache + "/.rels" + std::to_string(i)).c_str(), "w");
      int err = 0;
      _files[i] = BZ2_bzWriteOpen(&err, _rawFiles[i], 6, 0, 30);
      if (err != BZ_OK) {
        throw std::runtime_error("Could not open bzip file for writing.");
      }
      _outBuffers[i] = new unsigned char[BUFFER_S_PAIRS];
    }
  }
}

// _____________________________________________________________________________
void Sweeper::processQueue(size_t t) {
  JobBatch batch;
  while ((batch = _jobs.get()).size()) {
    for (const auto& job : batch) doCheck(job.first, job.second, t);
  }
}

// _____________________________________________________________________________
void Sweeper::fillBatch(JobBatch* batch,
                        const IntervalIdx<int32_t, SweepVal>* actives,
                        const BoxVal* cur) const {
  batch->reserve(20);

  const auto& overlaps = actives->overlap_find_all({cur->loY, cur->upY});

  for (auto p : overlaps) batch->push_back({*cur, p.v});
}

// _____________________________________________________________________________
void Sweeper::writeContainsMulti(size_t t, const std::string& a,
                                 const std::string& b, size_t bSub) {
  {
    std::unique_lock<std::mutex> lock(_mut);

    _subContains[b][a].insert(bSub);

    if (_subContains[b][a].size() != _subSizes[b]) return;

    _subContains[b].erase(a);
  }

  writeContains(t, a, b);
}
