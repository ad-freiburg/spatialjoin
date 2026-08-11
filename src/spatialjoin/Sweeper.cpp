#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#ifndef SPATIALJOIN_NO_ZLIB
#include <zlib.h>
#endif

#ifndef SPATIALJOIN_NO_BZIP2
#include <bzlib.h>
#endif

#include <algorithm>
#include <cassert>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <set>
#include <sstream>

#include "BoxIds.h"
#include "SweepEventList.h"
#include "Sweeper.h"
#include "util/Misc.h"
#include "util/geo/IntervalIdx.h"
#include "util/log/Log.h"

using sj::GeomCheckRes;
using sj::GeomType;
using sj::Sweeper;
using sj::boxids::boxIdIsect;
using sj::boxids::BoxIdList;
using sj::boxids::getBoxId;
using sj::boxids::getBoxIds;
using sj::boxids::packBoxIds;
using util::preadAll;
using util::pwriteAll;
using util::readAll;
using util::writeAll;
using util::geo::area;
using util::geo::DE9IM;
using util::geo::DPoint;
using util::geo::FPoint;
using util::geo::getBoundingBox;
using util::geo::I32Box;
using util::geo::I32Line;
using util::geo::I32MultiLine;
using util::geo::I32MultiPoint;
using util::geo::I32MultiPolygon;
using util::geo::I32Point;
using util::geo::I32Polygon;
using util::geo::I32XSortedLine;
using util::geo::I32XSortedPolygon;
using util::geo::intersectsContainsCovers;
using util::geo::intersectsCovers;
using util::geo::LineSegment;
using util::geo::Point;
using util::geo::webMercToLatLng;
using util::LogLevel::DEBUG;
using util::LogLevel::ERROR;
using util::LogLevel::INFO;
using util::LogLevel::VDEBUG;
using util::LogLevel::WARN;

const static size_t OBB_MIN_SIZE = 100;

const static double sin45 = 1.0 / sqrt(2);
const static double cos45 = 1.0 / sqrt(2);

// _____________________________________________________________________________
void Sweeper::clearMultis(bool force) {
  JobBatch curBatch;
  size_t batchSize = 1000;
  int32_t curMinThreadX = std::numeric_limits<int32_t>::max();

  for (size_t i = 0; i < _cfg.numThreads; i++) {
    if (_atomicCurX[i] < curMinThreadX) curMinThreadX = _atomicCurX[i];
  }

  for (size_t i = 0; i < 2; i++) {
    for (auto a = _activeMultis[i].begin(); a != _activeMultis[i].end();) {
      size_t mid = *a;
      if (mid >= _cacheManager->numMultis(i)) {
        LOG(WARN) << "Invalid multi ID " << mid << " detected!";
        a++;
        continue;
      }
      const std::string& gid = _cacheManager->multiId(i, mid);
      int32_t rightX = _cacheManager->multiRightX(i, mid);
      if (force || rightX < curMinThreadX) {
        curBatch.push_back({{}, {}, gid});
        a = _activeMultis[i].erase(a);
      } else {
        a++;
      }

      if (curBatch.size() > batchSize) {
        _jobs.add(std::move(curBatch));
        curBatch.clear();  // std doesnt guarantee that after move
        curBatch.reserve(batchSize);
      }
    }
  }

  if (curBatch.size()) _jobs.add(std::move(curBatch));
}

// _____________________________________________________________________________
void Sweeper::multiOut(size_t tOut, const std::string& gidA) {
  // collect dist, if requested
  if (_cfg.withinDist >= 0) {
    std::map<std::string, double> subDistance;
    for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
      std::unique_lock<std::mutex> lock(_mutsDistance[t]);
      auto i = _subDistance[t].find(gidA);
      if (i != _subDistance[t].end()) {
        for (const auto& a : i->second) {
          if (subDistance.find(a.first) == subDistance.end())
            subDistance[a.first] = a.second;
          else if (subDistance[a.first] > a.second)
            subDistance[a.first] = a.second;
        }
        _subDistance[t].erase(i);
      }
    }

    for (const auto& a : subDistance) {
      writeRel(tOut, gidA, a.first, "\t" + std::to_string(a.second) + "\t");
      writeRel(tOut, a.first, gidA, "\t" + std::to_string(a.second) + "\t");

      for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
        std::unique_lock<std::mutex> lock(_mutsDistance[t]);
        auto j = _subDistance[t].find(a.first);
        if (j != _subDistance[t].end()) {
          auto k = j->second.find(gidA);
          if (k != j->second.end()) {
            j->second.erase(gidA);
          }
        }
      }
    }
    return;
  }

  // collect DE9IM, if requested
  if (_cfg.computeDE9IM) {
    std::map<std::string, util::geo::DE9IMatrix> subDE9IM;

    for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
      std::unique_lock<std::mutex> lock(_mutsDE9IM[t]);
      auto i = _subDE9IM[t].find(gidA);
      if (i != _subDE9IM[t].end()) {
        for (const auto& a : i->second) {
          subDE9IM[a.first] += a.second;
        }
        _subDE9IM[t].erase(i);
      }
    }

    for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
      std::unique_lock<std::mutex> lock(_mutsDE9IM[t]);
      for (const auto& a : subDE9IM) {
        auto j = _subDE9IM[t].find(a.first);
        if (j != _subDE9IM[t].end()) {
          auto k = j->second.find(gidA);
          if (k != j->second.end()) {
            j->second.erase(gidA);
          }
        }
      }
    }

    for (const auto& a : subDE9IM) {
      if (_cfg.de9imFilter.matches(a.second)) {
        writeRel(tOut, gidA, a.first, "\t" + a.second.toString() + "\t");
        _relStats[tOut].de9im++;
      }
      if (_cfg.de9imFilter.matches(a.second.transpose())) {
        writeRel(tOut, a.first, gidA,
                 "\t" + a.second.transpose().toString() + "\t");
        _relStats[tOut].de9im++;
      }
    }
    return;
  }

  std::unordered_map<std::string, size_t> subContains, subCovered;

  std::unordered_map<std::string, std::unordered_map<std::string, size_t>>
      subEquals;

  // collect equals
  for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
    std::unique_lock<std::mutex> lock(_mutsEquals[t]);
    auto i = _subEquals[t].find(gidA);
    if (i != _subEquals[t].end()) {
      for (const auto& a : i->second) {
        subEquals[gidA][a.first] += a.second.size();
        auto j = _subEquals[t].find(a.first);
        if (j != _subEquals[t].end()) {
          auto k = j->second.find(gidA);
          if (k != j->second.end()) {
            subEquals[a.first][gidA] += k->second.size();
            j->second.erase(gidA);
          }
        }
      }
      _subEquals[t].erase(i);
    }
  }

  // collect contains
  for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
    std::unique_lock<std::mutex> lock(_mutsContains[t]);
    auto i = _subContains[t].find(gidA);
    if (i != _subContains[t].end()) {
      for (const auto& a : i->second) {
        subContains[a.first] += a.second.size();
      }
      _subContains[t].erase(i);
    }
  }

  // collect covers
  for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
    std::unique_lock<std::mutex> lock(_mutsCovers[t]);
    auto i = _subCovered[t].find(gidA);
    if (i != _subCovered[t].end()) {
      for (const auto& a : i->second) {
        subCovered[a.first] += a.second.size();
      }
      _subCovered[t].erase(i);
    }
  }

  // write equals
  for (auto i : subEquals[gidA]) {
    if (i.second == _cacheManager->subSize(gidA) &&
        subEquals[i.first][gidA] == _cacheManager->subSize(i.first)) {
      writeRel(tOut, i.first, gidA, _cfg.sepEquals);
      _relStats[tOut].equals++;
      writeRel(tOut, gidA, i.first, _cfg.sepEquals);
      _relStats[tOut].equals++;
    }
  }

  // write contains
  for (auto i : subContains) {
    if (i.second == _cacheManager->subSize(gidA)) {
      writeRel(tOut, i.first, gidA, _cfg.sepContains);
      _relStats[tOut].contains++;
    }
  }

  // write covers
  for (auto i : subCovered) {
    if (i.second == _cacheManager->subSize(gidA)) {
      writeNotOverlaps(tOut, i.first, _cacheManager->isMulti(i.first) ? 1 : 0,
                       gidA, 1);
      writeRel(tOut, i.first, gidA, _cfg.sepCovers);
      _relStats[tOut].covers++;
    }
  }

  // write touches, aggregate first to avoid locking during I/O
  std::vector<std::pair<std::string, std::string>> touchesTmp;

  for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
    {
      std::unique_lock<std::mutex> lock(_mutsTouches[t]);
      auto i = _subTouches[t].find(gidA);
      if (i != _subTouches[t].end()) {
        for (const auto& b : i->second) {
          auto gidB = b;
          if (!notTouches(gidA, gidB)) touchesTmp.push_back({gidA, gidB});

          {
            std::unique_lock<std::mutex> lock2(_mutsNotTouches[t]);
            auto j = _subNotTouches[t].find(gidB);
            if (j != _subNotTouches[t].end()) j->second.erase(gidA);
          }

          auto k = _subTouches[t].find(gidB);
          if (k != _subTouches[t].end()) k->second.erase(gidA);
        }

        _subTouches[t].erase(i);
      }
    }

    std::unique_lock<std::mutex> lock2(_mutsNotTouches[t]);
    _subNotTouches[t].erase(gidA);
  }

  for (const auto& p : touchesTmp) {
    _relStats[tOut].touches++;
    writeRel(tOut, p.first, p.second, _cfg.sepTouches);
    _relStats[tOut].touches++;
    writeRel(tOut, p.second, p.first, _cfg.sepTouches);
  }

  // write crosses, aggregate first to avoid locking during I/O
  std::vector<std::pair<std::string, std::string>> crossesTmp;
  for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
    {
      std::unique_lock<std::mutex> lock(_mutsCrosses[t]);
      auto i = _subCrosses[t].find(gidA);
      if (i != _subCrosses[t].end()) {
        for (const auto& b : i->second) {
          auto gidB = b;
          if (!notCrosses(gidA, gidB)) crossesTmp.push_back({gidA, gidB});

          {
            std::unique_lock<std::mutex> lock2(_mutsNotCrosses[t]);
            auto j = _subNotCrosses[t].find(gidB);
            if (j != _subNotCrosses[t].end()) j->second.erase(gidA);
          }

          auto k = _subCrosses[t].find(gidB);
          if (k != _subCrosses[t].end()) k->second.erase(gidA);
        }

        _subCrosses[t].erase(i);
      }
    }

    std::unique_lock<std::mutex> lock2(_mutsNotCrosses[t]);
    _subNotCrosses[t].erase(gidA);
  }

  for (const auto& p : crossesTmp) {
    _relStats[tOut].crosses++;
    writeRel(tOut, p.first, p.second, _cfg.sepCrosses);
    _relStats[tOut].crosses++;
    writeRel(tOut, p.second, p.first, _cfg.sepCrosses);
  }

  // write overlaps caused by incomplete covers
  {
    for (const auto& b : subCovered) {
      auto gidB = b.first;
      if (b.second == _cacheManager->subSize(gidA)) continue;

      if (!notOverlaps(gidA, gidB)) {
        _relStats[tOut].overlaps++;
        writeRel(tOut, gidA, gidB, _cfg.sepOverlaps);
        _relStats[tOut].overlaps++;
        writeRel(tOut, gidB, gidA, _cfg.sepOverlaps);
      }
    }
  }

  // write overlaps, aggregate first to avoid locking during I/O
  std::vector<std::pair<std::string, std::string>> overlapsTmp;
  for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
    {
      std::unique_lock<std::mutex> lock(_mutsOverlaps[t]);
      auto i = _subOverlaps[t].find(gidA);
      if (i != _subOverlaps[t].end()) {
        for (const auto& b : i->second) {
          auto gidB = b;
          if (!notOverlaps(gidA, gidB)) overlapsTmp.push_back({gidA, gidB});

          {
            std::unique_lock<std::mutex> lock2(_mutsNotOverlaps[t]);
            auto j = _subNotOverlaps[t].find(gidB);
            if (j != _subNotOverlaps[t].end()) j->second.erase(gidA);
          }

          auto k = _subOverlaps[t].find(gidB);
          if (k != _subOverlaps[t].end()) k->second.erase(gidA);
        }

        _subOverlaps[t].erase(i);
      }
    }
    std::unique_lock<std::mutex> lock2(_mutsNotOverlaps[t]);
    _subNotOverlaps[t].erase(gidA);
  }

  for (const auto& p : overlapsTmp) {
    _relStats[tOut].overlaps++;
    writeRel(tOut, p.first, p.second, _cfg.sepOverlaps);
    _relStats[tOut].overlaps++;
    writeRel(tOut, p.second, p.first, _cfg.sepOverlaps);
  }
}

// _____________________________________________________________________________
RelStats Sweeper::sweep(const SweepEventList& events) {
  // reads from the beginning of the event list
  auto reader = events.newReader();

  _cancelled = false;

  const size_t batchSize = 100000;
  JobBatch curBatch;

  util::geo::IntervalIdx<int32_t, SweepVal> actives[2];

  _stats.resize(_cfg.numThreads + 1);
  _relStats.resize(_cfg.numThreads + 1);
  _checks.resize(_cfg.numThreads);
  _curX.resize(_cfg.numThreads);
  _subEquals.resize(_cfg.numThreads + 1);
  _subCovered.resize(_cfg.numThreads + 1);
  _subContains.resize(_cfg.numThreads + 1);
  _subDistance.resize(_cfg.numThreads + 1);
  _subDE9IM.resize(_cfg.numThreads + 1);
  _subNotOverlaps.resize(_cfg.numThreads + 1);
  _subOverlaps.resize(_cfg.numThreads + 1);
  _subNotTouches.resize(_cfg.numThreads + 1);
  _subNotCrosses.resize(_cfg.numThreads + 1);
  _subCrosses.resize(_cfg.numThreads + 1);
  _subTouches.resize(_cfg.numThreads + 1);

  _mutsEquals = std::vector<std::mutex>(_cfg.numThreads + 1);
  _mutsCovers = std::vector<std::mutex>(_cfg.numThreads + 1);
  _mutsContains = std::vector<std::mutex>(_cfg.numThreads + 1);
  _mutsOverlaps = std::vector<std::mutex>(_cfg.numThreads + 1);
  _mutsNotOverlaps = std::vector<std::mutex>(_cfg.numThreads + 1);
  _mutsNotTouches = std::vector<std::mutex>(_cfg.numThreads + 1);
  _mutsTouches = std::vector<std::mutex>(_cfg.numThreads + 1);
  _mutsNotCrosses = std::vector<std::mutex>(_cfg.numThreads + 1);
  _mutsCrosses = std::vector<std::mutex>(_cfg.numThreads + 1);
  _mutsDistance = std::vector<std::mutex>(_cfg.numThreads + 1);
  _mutsDE9IM = std::vector<std::mutex>(_cfg.numThreads + 1);
  _atomicCurX = std::vector<std::atomic<int32_t>>(_cfg.numThreads + 1);

  size_t counts = 0, totalCheckCount = 0, jj = 0, checkPairs = 0;
  auto t = TIME();

  // fire up worker threads for geometry checking
  std::vector<std::thread> thrds(_cfg.numThreads);
  for (size_t i = 0; i < thrds.size(); i++)
    thrds[i] = std::thread(&Sweeper::processQueue, this, i);

  const BoxVal* cur = 0;

  try {
    while ((cur = reader.next()) != 0) {
      if (_cfg.sweepCancellationCb && jj % 10000 == 0) {
        _cfg.sweepCancellationCb();
      }

      jj++;

      if (jj % 200000 == 0) clearMultis(false);

      if (cur->type == DELETED) {
        continue;
      } else if (cur->type == SELF_CHECK || cur->type == SELF_CHECK_AREA ||
                 cur->type == SELF_CHECK_LINE ||
                 cur->type == SELF_CHECK_POINT) {
        // self checks, required if we have reference geoms
        curBatch.push_back({*cur, *cur, ""});
      } else if (!cur->out && cur->loY == 1 && cur->upY == 0 &&
                 cur->type == POINT) {
        // special multi-IN
        _activeMultis[cur->side].insert(cur->id);
      } else if (!cur->out) {
        // IN event
        actives[cur->side].insert(
            {cur->loY, cur->upY},
            {cur->id,
             cur->type,
             cur->b45,
             cur->point,
             {cur->val, cur->point.getY() == cur->loY ? cur->upY : cur->loY},
             cur->side,
             cur->large});

        if (jj % 500000 == 0) {
          auto lon = webMercToLatLng<double>((1.0 * cur->val) / PREC, 0).getX();
          totalCheckCount += checkPairs;

          auto cacheSize = _cacheManager->size();

          log(std::to_string(jj / 2) + " / " +
              std::to_string(events.numObjects()) + " (" +
              std::to_string(
                  (((1.0 * jj) / (1.0 * events.numEvents())) * 100)) +
              "%, " +
              std::to_string((500000.0 / double(TOOK(t))) * 1000000000.0) +
              " geoms/s, " +
              std::to_string((checkPairs / double(TOOK(t))) * 1000000000.0) +
              " pairs/s), avg. " +
              std::to_string(((1.0 * totalCheckCount) / (1.0 * counts))) +
              " checks/geom, sweepLon=" + std::to_string(lon) + "°, |A|=" +
              std::to_string(actives[0].size() + actives[1].size()) +
              ", |JQ|=" + std::to_string(_jobs.size()) + " (x" +
              std::to_string(batchSize) + "), |A_mult|=" +
              std::to_string(_activeMultis[0].size() +
                             _activeMultis[1].size()) +
              ", |C|=" + std::to_string(cacheSize.first) + " (" +
              util::readableSize(cacheSize.second) + ")");
          t = TIME();
          checkPairs = 0;
        }

        if ((jj % 100 == 0) && _cfg.sweepProgressCb)
          _cfg.sweepProgressCb(jj / 2);
      } else {
        // OUT event
        actives[cur->side].erase({cur->loY, cur->upY}, {cur->id, cur->type});

        counts++;

        int sideB = ((int)(cur->side) + 1) % _cacheManager->numSides();

        fillBatch(&curBatch, &actives[sideB], cur);

        if (curBatch.size() > batchSize) {
          checkPairs += curBatch.size();
          if (!_cfg.noGeometryChecks) _jobs.add(std::move(curBatch));
          curBatch.clear();  // std doesnt guarantee that after move
          curBatch.reserve(batchSize + 100);
        }
      }
    }
  } catch (...) {
    // graceful handling of an exception during sweep

    // set the cancelled variable to true
    _cancelled = true;

    // the DONE element on the job queue to signal all threads to shut down
    _jobs.add({});

    // again wait for all workers to finish
    for (auto& thr : thrds)
      if (thr.joinable()) thr.join();

    // rethrow exception
    throw;
  }

  if (!_cfg.noGeometryChecks && curBatch.size()) _jobs.add(std::move(curBatch));

  // the DONE element on the job queue to signal all threads to shut down
  _jobs.add({});

  // wait for all workers to finish
  for (auto& thr : thrds)
    if (thr.joinable()) thr.join();

  // empty job queue
  _jobs.reset();
  // fire up new workers to clear multis
  for (size_t i = 0; i < thrds.size(); i++)
    thrds[i] = std::thread(&Sweeper::processQueue, this, i);

  // now also clear the multis
  clearMultis(true);
  // the DONE element on the job queue to signal all threads to shut down
  _jobs.add({});

  // again wait for all workers to finish
  for (auto& thr : thrds)
    if (thr.joinable()) thr.join();

  // final check count aggregation
  totalCheckCount += checkPairs;

  // aggregate total stats
  Stats sum;
  for (auto s : _stats) sum += s;

  // aggregate total stats
  RelStats sumRel;
  for (auto s : _relStats) sumRel += s;

  if (_cfg.statsCb) {
    _cfg.statsCb(sum.toString() + "\n\n");
    _cfg.statsCb(sumRel.toString() + "\n");
  }

  return sumRel;
}

// _____________________________________________________________________________
sj::Area Sweeper::areaFromSimpleArea(const SimpleArea* sa) const {
  auto spoly = I32XSortedPolygon(sa->geom);

  if (!_cfg.useFastSweepSkip) {
    spoly.getOuter().setMaxSegLen(std::numeric_limits<int32_t>::max());
  }

  return {std::move(spoly),
          sa->id,
          0,
          (_cfg.useBoxIds ? BoxIdList{{1, 0}, {-getBoxId(sa->geom.front()), 0}}
                          : BoxIdList{}),
          {}};
}

// _____________________________________________________________________________
util::geo::DE9IMatrix Sweeper::DE9IMCheck(const Area* a, const Area* b,
                                          size_t t) const {
  _stats[t].totalComps++;
  // cheap equivalence check
  if (a->geom == b->geom) {
    // equivalent!
    return util::geo::M2FFF1FFF2;
  }

  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect(a->boxIds, b->boxIds);
    _stats[t].timeBoxIdIsectAreaArea += TOOK(ts);

    // all boxes of a are fully contained in b, a is fully contained in b
    if (r.first == a->boxIds.front().first) return util::geo::M2FF1FF212;

    // no box shared, we cannot have any spatial relation
    if (r.first + r.second == 0) return util::geo::MFF2FF1212;

    if (_dontNeedFullDE9IM) {
      // at least one box is fully contained, so we intersect
      // but the number of fully and partially contained boxes is smaller
      // than the number of boxes of A, so we cannot possible be contained
      if (r.first + r.second < a->boxIds.front().first && r.first > 0) {
        // we surely overlap if the area of b is greater than the area of a
        // or if the bounding box of b is not in a
        // otherwise, we cannot be sure
        if (b->geom.area() > a->geom.area() ||
            !util::geo::contains(b->geom.boundingBox(), a->geom.boundingBox()))
          // this is an incomplete matrix by design, specified to return true
          // for only intersects() and overlaps02()
          return util::geo::M2F2FFF2F2;
      }
    }
  }

  if (_cfg.useOBB) {
    if (!a->obb.empty() && !b->obb.empty()) {
      auto ts = TIME();
      auto r = util::geo::intersectsContainsCovers(a->obb, b->obb);
      _stats[t].timeOBBIsectAreaArea += TOOK(ts);
      if (!std::get<0>(r)) return util::geo::MFF2FF1212;
    }
  }

  auto ts = TIME();
  auto res = DE9IM(b->geom, a->geom).transpose();

  _stats[t].timeFullGeoCheckAreaArea += TOOK(ts);
  _stats[t].fullGeoChecksAreaArea++;
  return res;
}

// _____________________________________________________________________________
util::geo::DE9IMatrix Sweeper::DE9IMCheck(const Line* a, const Area* b,
                                          size_t t) const {
  _stats[t].totalComps++;
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect(a->boxIds, b->boxIds);
    _stats[t].timeBoxIdIsectAreaLine += TOOK(ts);

    // all boxes of a are fully contained in b, we intersect and we are
    // contained
    if (r.first == a->boxIds.front().first) return util::geo::M1FF0FF212;

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return util::geo::MFF1FF0212;

    // at least one box is fully contained, so we intersect
    // but the number of fully and partially contained boxes is smaller
    // than the number of boxes of A, so we cannot possible by contained, but
    // we cross
    if (_dontNeedFullDE9IM && r.first + r.second < a->boxIds.front().first &&
        r.first > 0) {
      // this is an incomplete matrix by design, specified to return true
      // for only intersects() and crosses1vs2()
      return util::geo::M1F1FFFFF2;
    }
  }

  if (_cfg.useOBB) {
    if (!a->obb.empty() && !b->obb.empty()) {
      auto ts = TIME();
      auto r = intersectsContainsCovers(a->obb, b->obb);
      _stats[t].timeOBBIsectAreaLine += TOOK(ts);
      if (!std::get<0>(r)) return util::geo::MFF1FF0212;
    }
  }

  auto ts = TIME();
  auto res = DE9IM(a->geom, b->geom);
  _stats[t].timeFullGeoCheckAreaLine += TOOK(ts);
  _stats[t].fullGeoChecksAreaLine++;

  return res;
}

// _____________________________________________________________________________
util::geo::DE9IMatrix Sweeper::DE9IMCheck(const Line* a, const Line* b,
                                          size_t t) const {
  _stats[t].totalComps++;
  // cheap equivalence check
  if (a->geom == b->geom) {
    // equivalent!
    return util::geo::M10FF0FFF2;
  }

  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect(a->boxIds, b->boxIds);
    _stats[t].timeBoxIdIsectLineLine += TOOK(ts);

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return util::geo::MFF1FF0102;
  }

  if (_cfg.useOBB) {
    if (!a->obb.empty() && !b->obb.empty()) {
      auto ts = TIME();
      auto r = util::geo::intersectsContainsCovers(a->obb, b->obb);
      _stats[t].timeOBBIsectLineLine += TOOK(ts);
      if (!std::get<0>(r)) return util::geo::MFF1FF0102;
    }
  }

  auto ts = TIME();
  auto res = DE9IM(a->geom, b->geom);
  _stats[t].timeFullGeoCheckLineLine += TOOK(ts);
  _stats[t].fullGeoChecksLineLine++;

  return res;
}

// _____________________________________________________________________________
util::geo::DE9IMatrix Sweeper::DE9IMCheck(const LineSegment<int32_t>& a,
                                          const Area* b, size_t t) const {
  _stats[t].totalComps++;
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect({{1, 0}, {getBoxId(a.first), 0}}, b->boxIds);
    _stats[t].timeBoxIdIsectAreaLine += TOOK(ts);

    // all boxes of a are fully contained in b, we intersect and we are
    // contained
    if (r.first == 1) return util::geo::M1FF0FF212;

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return util::geo::MFF1FF0212;
  }

  if (_cfg.useOBB && !b->obb.empty()) {
    auto ts = TIME();
    auto r = intersectsContainsCovers(I32XSortedLine(a), b->obb);
    _stats[t].timeOBBIsectAreaLine += TOOK(ts);
    if (!std::get<0>(r)) return util::geo::MFF1FF0212;
  }

  auto ts = TIME();
  auto res = DE9IM(I32XSortedLine(a), b->geom);
  _stats[t].timeFullGeoCheckAreaLine += TOOK(ts);
  _stats[t].fullGeoChecksAreaLine++;
  return res;
}

// _____________________________________________________________________________
util::geo::DE9IMatrix Sweeper::DE9IMCheck(const LineSegment<int32_t>& a,
                                          const LineSegment<int32_t>& b,
                                          size_t t) const {
  _stats[t].totalComps++;
  auto ts = TIME();

  // no need to do a full sweep for two simple lines with all the required
  // datastructures, just unroll the individual checks here

  auto r = util::geo::IntersectorLine<int32_t>::check(
      a, 32767, true, 32767, true, b, 32767, true, 32767, true);

  const bool weakIntersect = r;

  const bool strictIntersect = (r >> 0) & 1;
  const bool overlaps = (r >> 1) & 1;
  const bool crosses = (r >> 4) & 1;
  const bool strictIntersect2 = (r >> 5) & 1;
  const bool bFirstInA = (r >> 2) & 1;
  const bool bLastInA = (r >> 3) & 1;
  const bool aFirstInB = (r >> 6) & 1;
  const bool aLastInB = (r >> 7) & 1;

  const bool aInB = !crosses && !strictIntersect && weakIntersect;
  const bool bInA = !crosses && !strictIntersect2 && weakIntersect;

  bool aFirstOnBoundary = a.first == b.first || a.first == b.second;
  bool aLastOnBoundary = a.second == b.first || a.second == b.second;

  bool bFirstOnBoundary = b.first == a.first || b.first == a.second;
  bool bLastOnBoundary = b.second == a.first || b.second == a.second;

  uint16_t ii =
      overlaps ? util::geo::D1 : (crosses ? util::geo::D0 : util::geo::F);
  uint16_t ib = (bFirstInA || bLastInA) ? util::geo::D0 : util::geo::F;
  uint16_t ie = aInB ? util::geo::F : util::geo::D1;
  uint16_t bi =
      ((aFirstInB && !aFirstOnBoundary) || (aLastInB && !aLastOnBoundary))
          ? util::geo::D0
          : util::geo::F;
  uint16_t bb = (a.first == b.first || a.second == b.first ||
                 a.second == b.second || a.first == b.second)
                    ? util::geo::D0
                    : util::geo::F;
  uint16_t be =
      ((aFirstInB || aFirstOnBoundary) && (aLastInB || aLastOnBoundary))
          ? util::geo::F
          : util::geo::D0;
  uint16_t ei = bInA ? util::geo::F : util::geo::D1;
  uint16_t eb =
      ((bFirstInA || bFirstOnBoundary) && (bLastInA || bLastOnBoundary))
          ? util::geo::F
          : util::geo::D0;
  _stats[t].timeFullGeoCheckLineLine += TOOK(ts);
  _stats[t].fullGeoChecksLineLine++;

  return (ii << 0) | (ib << 2) | (ie << 4) | (bi << 6) | (bb << 8) |
         (be << 10) | (ei << 12) | (eb << 14);
}

// _____________________________________________________________________________
util::geo::DE9IMatrix Sweeper::DE9IMCheck(const LineSegment<int32_t>& a,
                                          const Line* b, size_t t) const {
  _stats[t].totalComps++;
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect({{1, 0}, {getBoxId(a.first), 0}}, b->boxIds);
    _stats[t].timeBoxIdIsectLineLine += TOOK(ts);

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return util::geo::MFF1FF0102;
  }

  if (_cfg.useOBB && !b->obb.empty()) {
    auto ts = TIME();
    auto r = intersectsContainsCovers(I32XSortedLine(a), b->obb);
    _stats[t].timeOBBIsectLineLine += TOOK(ts);
    if (!std::get<0>(r)) return util::geo::MFF1FF0102;
  }

  auto ts = TIME();
  auto res = DE9IM(I32XSortedLine(a), b->geom);
  _stats[t].timeFullGeoCheckLineLine += TOOK(ts);
  _stats[t].fullGeoChecksLineLine++;
  return res;
}

// _____________________________________________________________________________
util::geo::DE9IMatrix Sweeper::DE9IMCheck(const I32Point& a, const Line* b,
                                          size_t t) const {
  _stats[t].totalComps++;
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect({{1, 0}, {getBoxId(a), 0}}, b->boxIds);
    _stats[t].timeBoxIdIsectLinePoint += TOOK(ts);

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return util::geo::MFF0FFF102;
  }

  auto ts = TIME();
  auto res = util::geo::DE9IM(a, b->geom);
  _stats[t].timeFullGeoCheckLinePoint += TOOK(ts);
  _stats[t].fullGeoChecksLinePoint++;

  return res;
}

// ____________________________________________________________________________
void Sweeper::writeRel(size_t t, const std::string& a, const std::string& b,
                       const std::string& pred) {
  if (!_cfg.writeRelCb) return;

  auto ts = TIME();

  if (_cacheManager->numSides() == 2 && (a[0] != 'A' || a[0] == b[0])) return;

  _cfg.writeRelCb(t, a.c_str() + 1, a.size() - 1, b.c_str() + 1, b.size() - 1,
                  pred.c_str(), pred.size());

  _stats[t].timeWrite += TOOK(ts);
}

// ____________________________________________________________________________
void Sweeper::writeDE9IM(size_t t, const std::string& a, size_t aSub,
                         const std::string& b, size_t bSub,
                         util::geo::DE9IMatrix de9im) {
  if (a != b) {
    if (aSub > 0 && bSub == 0 && de9im.covers()) {
      // no need to lock and track the multigeometry here, we can directly
      // write that a contains b
      if (_cfg.de9imFilter.matches(de9im)) {
        _relStats[t].de9im++;
        writeRel(t, a, b, "\t" + de9im.toString() + "\t");
      }
      if (_cfg.de9imFilter.matches(de9im.transpose())) {
        _relStats[t].de9im++;
        writeRel(t, b, a, "\t" + de9im.transpose().toString() + "\t");
      }
    } else if (bSub > 0 && aSub == 0 && de9im.transpose().covers()) {
      // no need to lock and track the multigeometry here, we can directly
      // write that b contains a
      if (_cfg.de9imFilter.matches(de9im)) {
        _relStats[t].de9im++;
        writeRel(t, a, b, "\t" + de9im.toString() + "\t");
      }
      if (_cfg.de9imFilter.matches(de9im.transpose())) {
        _relStats[t].de9im++;
        writeRel(t, b, a, "\t" + de9im.transpose().toString() + "\t");
      }
    } else if ((bSub > 0 || aSub > 0)) {
      std::unique_lock<std::mutex> lock(_mutsDE9IM[t]);
      if (bSub > 0) {
        if (_subDE9IM[t][b].find(a) == _subDE9IM[t][b].end()) {
          _subDE9IM[t][b][a] = de9im.transpose();
        } else {
          _subDE9IM[t][b][a] += de9im.transpose();
        }
      }
      if (aSub > 0) {
        if (_subDE9IM[t][a].find(b) == _subDE9IM[t][a].end()) {
          _subDE9IM[t][a][b] = de9im;
        } else {
          _subDE9IM[t][a][b] += de9im;
        }
      }
    } else {
      if (_cfg.de9imFilter.matches(de9im)) {
        _relStats[t].de9im++;
        writeRel(t, a, b, "\t" + de9im.toString() + "\t");
      }
      if (_cfg.de9imFilter.matches(de9im.transpose())) {
        _relStats[t].de9im++;
        writeRel(t, b, a, "\t" + de9im.transpose().toString() + "\t");
      }
    }
  }

  // handle references

  if (!_cacheManager->hasRefs()) return;

  const auto* referersA = _cacheManager->getRefs(a, aSub);
  const auto* referersB = _cacheManager->getRefs(b, bSub);

  if (referersB) {
    for (const auto& idB : *referersB) {
      writeDE9IM(t, a, aSub, idB.first, idB.second, de9im);
    }
  }

  if (referersA) {
    for (const auto& idA : *referersA) {
      writeDE9IM(t, idA.first, idA.second, b, bSub, de9im);
    }
  }
}

// ____________________________________________________________________________
void Sweeper::writeDist(size_t t, const std::string& a, size_t aSub,
                        const std::string& b, size_t bSub, double dist) {
  if (a != b) {
    if (bSub > 0 || aSub > 0) {
      std::unique_lock<std::mutex> lock(_mutsDistance[t]);
      if (bSub > 0 && (_subDistance[t][b].find(a) == _subDistance[t][b].end() ||
                       _subDistance[t][b][a] > dist))
        _subDistance[t][b][a] = dist;
      if (aSub > 0 && (_subDistance[t][a].find(b) == _subDistance[t][a].end() ||
                       _subDistance[t][a][b] > dist))
        _subDistance[t][a][b] = dist;
    } else {
      const auto& dStr = util::formatFloat(dist, 4);
      writeRel(t, a, b, "\t" + dStr + "\t");
      writeRel(t, b, a, "\t" + dStr + "\t");
    }
  }

  // handle references

  if (!_cacheManager->hasRefs()) return;

  const auto* referersA = _cacheManager->getRefs(a, aSub);
  const auto* referersB = _cacheManager->getRefs(b, bSub);

  if (referersB) {
    for (const auto& idB : *referersB) {
      writeDist(t, a, aSub, idB.first, idB.second, dist);
    }
  }

  // no need to check exactly the same direction again
  if (a != b || aSub != bSub) {
    if (referersA) {
      for (const auto& idA : *referersA) {
        writeDist(t, idA.first, idA.second, b, bSub, dist);
      }
    }
  }
}

// ____________________________________________________________________________
void Sweeper::writeIntersect(size_t t, const std::string& a, size_t aSub,
                             const std::string& b, size_t bSub) {
  if (a != b) {
    _relStats[t].intersects++;
    _relStats[t].intersects++;
    writeRel(t, a, b, _cfg.sepIsect);
    writeRel(t, b, a, _cfg.sepIsect);
  }

  if (!_cacheManager->hasRefs()) return;

  // handle references

  const auto* referersA = _cacheManager->getRefs(a, aSub);
  const auto* referersB = _cacheManager->getRefs(b, bSub);

  if (referersB) {
    for (const auto& idB : *referersB) {
      writeIntersect(t, a, aSub, idB.first, idB.second);
    }
  }

  // no need to check exactly the same direction again
  if (a != b || aSub != bSub) {
    if (referersA) {
      for (const auto& idA : *referersA) {
        writeIntersect(t, idA.first, idA.second, b, bSub);
      }
    }
  }
}

// ____________________________________________________________________________
void Sweeper::selfCheck(const std::string& a, size_t subId, GeomType type,
                        size_t t) {
  if (_cfg.computeDE9IM) {
    if (type == SELF_CHECK_LINE)
      writeDE9IM(t, a, subId, a, subId, util::geo::M10FF0FFF2);
    else if (type == SELF_CHECK_AREA)
      writeDE9IM(t, a, subId, a, subId, util::geo::M2FFF1FFF2);
    else if (type == SELF_CHECK_POINT)
      writeDE9IM(t, a, subId, a, subId, util::geo::M0FFFFFFF2);
    else {
      // LOG(WARN)
      // << "Input reference geometries not supported with DE-9IM computation";
    }
  } else if (_cfg.withinDist >= 0) {
    writeDist(t, a, subId, a, subId, 0);
  } else {
    writeIntersect(t, a, subId, a, subId);
    writeEquals(t, a, subId, a, subId);
    writeCovers(t, a, subId, a, subId);
    writeContains(t, a, subId, a, subId);
    writeNotCrosses(t, a, subId, a, subId);
  }
}

// ____________________________________________________________________________
void Sweeper::doDE9IMCheck(const JobVal cur, const JobVal sv, size_t t) {
  _checks[t]++;
  _curX[t] = cur.val;

  // every 10000 checks, update our position
  if (_checks[t] % 10000 == 0) _atomicCurX[t] = _curX[t];

  if (cur.type == SELF_CHECK || cur.type == SELF_CHECK_AREA ||
      cur.type == SELF_CHECK_LINE || cur.type == SELF_CHECK_POINT) {
    auto sc = _cacheManager->selfCheck(cur.id);
    return selfCheck(sc.first, sc.second, cur.type, t);
  }

  if (cur.type == sv.type && cur.id == sv.id) return;

  if (isPoint(cur.type) && isPoint(sv.type)) {
    auto p1 = cur.point;
    auto p2 = sv.point;
    _stats[t].totalComps++;

    auto de9im = DE9IM(p1, p2);

    if (!de9im.disjoint()) {
      auto a = getPoint(cur.id, cur.type, cur.large ? -1 : t);
      auto b = getPoint(sv.id, sv.type, sv.large ? -1 : t);

      if (a->id == b->id) return;  // no self-checks in multigeometries

      writeDE9IM(t, a->id, a->subId, b->id, b->subId, de9im);
    }
  } else if (isPoint(cur.type) && isArea(sv.type)) {
    auto p = cur.point;

    std::shared_ptr<Area> a = getArea(sv, sv.large ? -1 : t);

    auto de9im = DE9IMCheck(p, a.get(), t);

    if (!de9im.disjoint()) {
      auto b = getPoint(cur.id, cur.type, cur.large ? -1 : t);

      if (a->id == b->id) return;  // no self-checks in multigeometries

      writeDE9IM(t, b->id, b->subId, a->id, a->subId, de9im);
    }
  } else if (isArea(cur.type) && isPoint(sv.type)) {
    auto p = sv.point;
    std::shared_ptr<Area> a = getArea(cur, cur.large ? -1 : t);

    auto de9im = DE9IMCheck(p, a.get(), t).transpose();

    if (!de9im.disjoint()) {
      auto b = getPoint(sv.id, sv.type, sv.large ? -1 : t);

      if (a->id == b->id) return;  // no self-checks in multigeometries

      writeDE9IM(t, a->id, a->subId, b->id, b->subId, de9im);
    }
  } else if (isLine(cur.type) && isPoint(sv.type)) {
    auto p = sv.point;

    if (isSimpleLine(cur.type)) {
      auto de9im = DE9IMCheck(p, {cur.point, cur.point2}, t);

      if (!de9im.disjoint()) {
        auto a = getPoint(sv.id, sv.type, sv.large ? -1 : t);

        auto ts = TIME();
        auto b = getSimpleLine(cur, cur.large ? -1 : t);
        _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);

        if (a->id == b->id) return;  // no self-checks in multigeometries

        writeDE9IM(t, a->id, a->subId, b->id, 0, de9im);
      }
    } else {
      auto ts = TIME();
      auto b = _cacheManager->getLine(cur.id, cur.large ? -1 : t);
      _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);
      auto de9im = DE9IMCheck(p, b.get(), t);

      if (!de9im.disjoint()) {
        auto a = getPoint(sv.id, sv.type, sv.large ? -1 : t);

        if (a->id == b->id) return;  // no self-checks in multigeometries

        writeDE9IM(t, a->id, a->subId, b->id, b->subId, de9im);
      }
    }
  } else if (isLine(sv.type) && isPoint(cur.type)) {
    auto p = cur.point;

    if (isSimpleLine(sv.type)) {
      auto de9im = DE9IMCheck(p, {sv.point, sv.point2}, t);

      if (!de9im.disjoint()) {
        auto a = getPoint(cur.id, cur.type, cur.large ? -1 : t);

        auto ts = TIME();
        auto b = getSimpleLine(sv, sv.large ? -1 : t);
        _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);

        if (a->id == b->id) return;  // no self-checks in multigeometries

        writeDE9IM(t, a->id, a->subId, b->id, 0, de9im);
      }
    } else {
      auto ts = TIME();
      auto b = _cacheManager->getLine(sv.id, sv.large ? -1 : t);
      _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);
      auto de9im = DE9IMCheck(p, b.get(), t);

      if (!de9im.disjoint()) {
        auto a = getPoint(cur.id, cur.type, cur.large ? -1 : t);

        if (a->id == b->id) return;  // no self-checks in multigeometries

        writeDE9IM(t, a->id, a->subId, b->id, b->subId, de9im);
      }
    }
  } else if (sv.type == LINE && cur.type == LINE) {
    auto ts = TIME();
    auto a = _cacheManager->getLine(sv.id, sv.large ? -1 : t);
    auto b = _cacheManager->getLine(cur.id, cur.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    auto de9im = DE9IMCheck(a.get(), b.get(), t);

    if (!de9im.disjoint()) {
      writeDE9IM(t, a->id, a->subId, b->id, b->subId, de9im);
    }
  } else if (isSimpleLine(sv.type) && isSimpleLine(cur.type)) {
    auto de9im = DE9IMCheck({sv.point, sv.point2}, {cur.point, cur.point2}, t);

    if (!de9im.disjoint()) {
      auto ts = TIME();
      auto a = getSimpleLine(sv, sv.large ? -1 : t);
      auto b = getSimpleLine(cur, cur.large ? -1 : t);
      _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);
      if (a->id == b->id) return;  // no self-checks in multigeometries
      writeDE9IM(t, a->id, 0, b->id, 0, de9im);
    }
  } else if (isSimpleLine(sv.type) && cur.type == LINE) {
    auto ts = TIME();
    auto a = getSimpleLine(sv, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);
    ts = TIME();
    auto b = _cacheManager->getLine(cur.id, cur.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    auto de9im = DE9IMCheck({sv.point, sv.point2}, b.get(), t);

    if (!de9im.disjoint()) {
      writeDE9IM(t, a->id, 0, b->id, b->subId, de9im);
    }
  } else if (sv.type == LINE && isSimpleLine(cur.type)) {
    auto ts = TIME();
    auto a = _cacheManager->getLine(sv.id, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);
    ts = TIME();
    auto b = getSimpleLine(cur, cur.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    auto de9im = DE9IMCheck({cur.point, cur.point2}, a.get(), t);

    if (!de9im.disjoint()) {
      writeDE9IM(t, a->id, a->subId, b->id, 0, de9im.transpose());
    }
  } else if (isArea(sv.type) && isArea(cur.type)) {
    std::shared_ptr<Area> a = getArea(cur, cur.large ? -1 : t);
    std::shared_ptr<Area> b = getArea(sv, sv.large ? -1 : t);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    auto de9im = DE9IMCheck(a.get(), b.get(), t);

    if (!de9im.disjoint()) {
      writeDE9IM(t, a->id, a->subId, b->id, b->subId, de9im);
    }
  } else if (sv.type == LINE && isArea(cur.type)) {
    auto ts = TIME();
    auto a = _cacheManager->getLine(sv.id, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    std::shared_ptr<Area> b = getArea(cur, cur.large ? -1 : t);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    auto de9im = DE9IMCheck(a.get(), b.get(), t);

    if (!de9im.disjoint()) {
      writeDE9IM(t, a->id, a->subId, b->id, b->subId, de9im);
    }
  } else if (isArea(sv.type) && cur.type == LINE) {
    std::shared_ptr<Area> a = getArea(sv, sv.large ? -1 : t);

    auto ts = TIME();
    auto b = _cacheManager->getLine(cur.id, cur.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    auto de9im = DE9IMCheck(b.get(), a.get(), t);

    if (!de9im.disjoint()) {
      writeDE9IM(t, a->id, a->subId, b->id, b->subId, de9im.transpose());
    }
  } else if (isSimpleLine(sv.type) && isArea(cur.type)) {
    auto ts = TIME();
    auto a = getSimpleLine(sv, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);

    std::shared_ptr<Area> b = getArea(cur, cur.large ? -1 : t);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    auto de9im = DE9IMCheck({sv.point, sv.point2}, b.get(), t);

    if (!de9im.disjoint()) {
      writeDE9IM(t, a->id, 0, b->id, b->subId, de9im);
    }
  } else if (isArea(sv.type) && isSimpleLine(cur.type)) {
    std::shared_ptr<Area> a = getArea(sv, sv.large ? -1 : t);

    auto ts = TIME();
    auto b = getSimpleLine(cur, cur.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    auto de9im = DE9IMCheck({cur.point, cur.point2}, a.get(), t);

    if (!de9im.disjoint()) {
      writeDE9IM(t, a->id, a->subId, b->id, 0, de9im.transpose());
    }
  }
}

// ____________________________________________________________________________
void Sweeper::doDistCheck(const JobVal cur, const JobVal sv, size_t t) {
  _checks[t]++;
  _curX[t] = cur.val;

  // every 10000 checks, update our position
  if (_checks[t] % 10000 == 0) _atomicCurX[t] = _curX[t];

  if (cur.type == SELF_CHECK || cur.type == SELF_CHECK_AREA ||
      cur.type == SELF_CHECK_LINE || cur.type == SELF_CHECK_POINT) {
    auto sc = _cacheManager->selfCheck(cur.id);
    return selfCheck(sc.first, sc.second, cur.type, t);
  }

  if (cur.type == sv.type && cur.id == sv.id) return;

  if (isPoint(cur.type) && isPoint(sv.type)) {
    auto p1 = cur.point;
    auto p2 = sv.point;

    auto dist = meterDist(p1, p2, _cfg.withinDist);

    if (dist <= _cfg.withinDist) {
      auto a = getPoint(cur.id, cur.type, cur.large ? -1 : t);
      auto b = getPoint(sv.id, sv.type, sv.large ? -1 : t);

      // no self checks
      if (a->id == b->id) return;

      writeDist(t, a->id, a->subId, b->id, b->subId, dist);
    }
  } else if (isPoint(cur.type) && isArea(sv.type)) {
    auto p = cur.point;

    std::shared_ptr<Area> a = getArea(sv, sv.large ? -1 : t);
    auto b = getPoint(cur.id, cur.type, cur.large ? -1 : t);

    double dist = distCheck(p, b.get(), a.get(), t);

    if (dist <= _cfg.withinDist) {
      writeDist(t, a->id, a->subId, b->id, b->subId, dist);
    }
  } else if (isArea(cur.type) && isPoint(sv.type)) {
    auto p = sv.point;

    std::shared_ptr<Area> a = getArea(cur, cur.large ? -1 : t);
    auto b = getPoint(sv.id, sv.type, sv.large ? -1 : t);

    double dist = distCheck(p, b.get(), a.get(), t);

    if (dist <= _cfg.withinDist) {
      writeDist(t, a->id, a->subId, b->id, b->subId, dist);
    }
  } else if (isLine(cur.type) && isPoint(sv.type)) {
    auto p = sv.point;

    double dist = std::numeric_limits<double>::max();

    if (isSimpleLine(cur.type)) {
      dist = distCheck(p, {cur.point, cur.point2}, t);

      if (dist <= _cfg.withinDist) {
        auto a = getPoint(sv.id, sv.type, sv.large ? -1 : t);
        auto b = getSimpleLine(cur, cur.large ? -1 : t);
        writeDist(t, a->id, a->subId, b->id, 0, dist);
      }
    } else {
      auto b = _cacheManager->getLine(cur.id, cur.large ? -1 : t);
      auto a = getPoint(sv.id, sv.type, sv.large ? -1 : t);
      dist = distCheck(p, a.get(), b.get(), t);

      if (dist <= _cfg.withinDist) {
        writeDist(t, a->id, a->subId, b->id, b->subId, dist);
      }
    }
  } else if (isLine(sv.type) && isPoint(cur.type)) {
    auto p = cur.point;

    double dist = std::numeric_limits<double>::max();

    if (isSimpleLine(sv.type)) {
      dist = distCheck(p, {sv.point, sv.point2}, t);

      if (dist <= _cfg.withinDist) {
        auto a = getPoint(cur.id, cur.type, cur.large ? -1 : t);
        auto b = getSimpleLine(sv, sv.large ? -1 : t);
        writeDist(t, a->id, a->subId, b->id, 0, dist);
      }
    } else {
      auto b = _cacheManager->getLine(sv.id, sv.large ? -1 : t);
      auto a = getPoint(cur.id, cur.type, cur.large ? -1 : t);

      dist = distCheck(p, a.get(), b.get(), t);

      if (dist <= _cfg.withinDist) {
        writeDist(t, a->id, a->subId, b->id, b->subId, dist);
      }
    }
  } else if (sv.type == LINE && cur.type == LINE) {
    auto a = _cacheManager->getLine(sv.id, sv.large ? -1 : t);
    auto b = _cacheManager->getLine(cur.id, cur.large ? -1 : t);

    // no expensive self checks for multi geoms
    if (a->id == b->id) return;

    auto dist = distCheck(a.get(), b.get(), t);

    if (dist <= _cfg.withinDist) {
      writeDist(t, a->id, a->subId, b->id, b->subId, dist);
    }
  } else if (isSimpleLine(sv.type) && isSimpleLine(cur.type)) {
    auto dist = distCheck({sv.point, sv.point2}, {cur.point, cur.point2}, t);

    if (dist <= _cfg.withinDist) {
      auto a = getSimpleLine(sv, sv.large ? -1 : t);
      auto b = getSimpleLine(cur, cur.large ? -1 : t);
      writeDist(t, a->id, 0, b->id, 0, dist);
    }
  } else if (isSimpleLine(sv.type) && cur.type == LINE) {
    auto a = getSimpleLine(sv, sv.large ? -1 : t);
    auto b = _cacheManager->getLine(cur.id, cur.large ? -1 : t);
    auto dist = distCheck({sv.point, sv.point2}, b.get(), t);

    if (dist <= _cfg.withinDist) {
      writeDist(t, a->id, 0, b->id, b->subId, dist);
    }
  } else if (sv.type == LINE && isSimpleLine(cur.type)) {
    auto a = _cacheManager->getLine(sv.id, sv.large ? -1 : t);

    auto dist = distCheck({cur.point, cur.point2}, a.get(), t);

    if (dist <= _cfg.withinDist) {
      auto b = getSimpleLine(cur, cur.large ? -1 : t);
      writeDist(t, a->id, a->subId, b->id, 0, dist);
    }
  } else if (isArea(sv.type) && isArea(cur.type)) {
    std::shared_ptr<Area> a = getArea(cur, cur.large ? -1 : t);
    std::shared_ptr<Area> b = getArea(sv, sv.large ? -1 : t);

    // no expensive self checks for multi geoms
    if (a->id == b->id) return;

    auto dist = distCheck(a.get(), b.get(), t);

    if (dist <= _cfg.withinDist) {
      writeDist(t, a->id, a->subId, b->id, b->subId, dist);
    }
  } else if (sv.type == LINE && isArea(cur.type)) {
    auto a = _cacheManager->getLine(sv.id, sv.large ? -1 : t);

    std::shared_ptr<Area> b = getArea(cur, cur.large ? -1 : t);

    // no expensive self checks for multi geoms
    if (a->id == b->id) return;

    auto dist = distCheck(a.get(), b.get(), t);

    if (dist <= _cfg.withinDist) {
      writeDist(t, a->id, a->subId, b->id, b->subId, dist);
    }
  } else if (isArea(sv.type) && cur.type == LINE) {
    std::shared_ptr<Area> a = getArea(sv, sv.large ? -1 : t);

    auto b = _cacheManager->getLine(cur.id, cur.large ? -1 : t);

    // no expensive self checks for multi geoms
    if (a->id == b->id) return;

    auto dist = distCheck(b.get(), a.get(), t);

    if (dist <= _cfg.withinDist) {
      writeDist(t, a->id, a->subId, b->id, b->subId, dist);
    }
  } else if (isSimpleLine(sv.type) && isArea(cur.type)) {
    std::shared_ptr<Area> b = getArea(cur, cur.large ? -1 : t);

    auto dist = distCheck({sv.point, sv.point2}, b.get(), t);

    if (dist <= _cfg.withinDist) {
      auto a = getSimpleLine(sv, sv.large ? -1 : t);
      writeDist(t, a->id, 0, b->id, b->subId, dist);
    }
  } else if (isArea(sv.type) && isSimpleLine(cur.type)) {
    std::shared_ptr<Area> a = getArea(sv, sv.large ? -1 : t);

    auto dist = distCheck({cur.point, cur.point2}, a.get(), t);

    if (dist <= _cfg.withinDist) {
      auto b = getSimpleLine(cur, cur.large ? -1 : t);
      writeDist(t, a->id, a->subId, b->id, 0, dist);
    }
  }
}

// ____________________________________________________________________________
void Sweeper::doCheck(const JobVal cur, const JobVal sv, size_t t) {
  _checks[t]++;
  _curX[t] = cur.val;

  // every 10000 checks, update our position
  if (_checks[t] % 10000 == 0) _atomicCurX[t] = _curX[t];

  if (cur.type == SELF_CHECK || cur.type == SELF_CHECK_AREA ||
      cur.type == SELF_CHECK_LINE || cur.type == SELF_CHECK_POINT) {
    auto sc = _cacheManager->selfCheck(cur.id);
    return selfCheck(sc.first, sc.second, cur.type, t);
  }

  if (cur.type == sv.type && cur.id == sv.id) return;

  if (isArea(cur.type) && isArea(sv.type)) {
    std::shared_ptr<Area> a = getArea(cur, cur.large ? -1 : t);
    std::shared_ptr<Area> b = getArea(sv, sv.large ? -1 : t);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    _stats[t].areaCmps++;
    _stats[t].areaSizeSum += std::max(a->geom.area(), b->geom.area());

    _stats[t].anchorSum += std::max(a->geom.size() / 2, b->geom.size() / 2);

    _stats[t].totalComps++;

    auto res = DE9IMCheck(a.get(), b.get(), t);

    // intersects
    if (res.intersects()) {
      writeIntersect(t, a->id, a->subId, b->id, b->subId);
    }

    // contained
    if (res.within()) {
      writeContains(t, b->id, b->subId, a->id, a->subId);
    }

    // covered
    if (res.coveredBy()) {
      writeCovers(t, b->id, b->subId, a->id, a->subId);

      if (fabs(a->geom.area() - b->geom.area()) < util::geo::EPSILON) {
        // both areas were equivalent
        writeEquals(t, a->id, a->subId, b->id, b->subId);

        // covers in other direction
        writeCovers(t, a->id, a->subId, b->id, b->subId);

        // contains in other direction
        writeContains(t, a->id, a->subId, b->id, b->subId);
      }
    }

    // touches
    if (res.touches()) {
      writeTouches(t, a->id, a->subId, b->id, b->subId);
    } else if (res.intersects()) {
      // if a is not a multi-geom, and is completey covered, we wont
      // be finding a touch as we assume non-self-intersecting geoms
      if (_cacheManager->isRefed(a->id) ||
          !(a->subId == 0 && res.coveredBy())) {
        writeNotTouches(t, a->id, a->subId, b->id, b->subId);
      }
    }

    // overlaps
    if (res.overlaps02()) {
      writeOverlaps(t, a->id, a->subId, b->id, b->subId);
    }
  } else if (cur.type == LINE && isArea(sv.type)) {
    std::shared_ptr<Area> b = getArea(sv, sv.large ? -1 : t);

    auto ts = TIME();
    auto a = _cacheManager->getLine(cur.id, cur.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    _stats[t].areaCmps++;
    _stats[t].areaSizeSum += b->geom.area();

    _stats[t].lineCmps++;
    _stats[t].lineLenSum += a->geom.length();

    _stats[t].anchorSum += std::max(a->geom.size() / 2, b->geom.size() / 2);

    _stats[t].totalComps++;

    auto res = DE9IMCheck(a.get(), b.get(), t);

    // intersects
    if (res.intersects()) {
      writeIntersect(t, b->id, b->subId, a->id, a->subId);
    }

    // contains
    if (res.within()) {
      writeContains(t, b->id, b->subId, a->id, a->subId);
    }

    // covers
    if (res.coveredBy()) {
      writeCovers(t, b->id, b->subId, a->id, a->subId);
    }

    // touches
    if (res.touches()) {
      writeTouches(t, a->id, a->subId, b->id, b->subId);
    } else if (res.intersects()) {
      // if a is not a multi-geom, and is completey covered, we wont
      // be finding a touch as we assume non-self-intersecting geoms
      if (_cacheManager->isRefed(a->id) ||
          !(a->subId == 0 && res.coveredBy())) {
        writeNotTouches(t, a->id, a->subId, b->id, b->subId);
      }
    }

    // crosses
    if (res.crosses1vs2()) {
      _relStats[t].crosses++;
      writeRel(t, a->id, b->id, _cfg.sepCrosses);
    }
  } else if (isSimpleLine(cur.type) && isArea(sv.type)) {
    std::shared_ptr<Area> b = getArea(sv, sv.large ? -1 : t);

    auto ts = TIME();
    auto a = getSimpleLine(cur, cur.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    _stats[t].areaCmps++;
    _stats[t].areaSizeSum += b->geom.area();

    _stats[t].lineCmps++;
    _stats[t].lineLenSum += util::geo::dist(cur.point, cur.point2);

    _stats[t].anchorSum += std::max((size_t)2, b->geom.size() / 2);

    _stats[t].totalComps++;

    auto res = DE9IMCheck({cur.point, cur.point2}, b.get(), t);

    // intersects
    if (res.intersects()) {
      writeIntersect(t, a->id, 0, b->id, b->subId);
    }

    // contains
    if (res.within()) {
      writeContains(t, b->id, b->subId, a->id, 0);
    }

    // covers
    if (res.coveredBy()) {
      writeCovers(t, b->id, 0, a->id, 0);
    }

    // touches
    if (res.touches()) {
      writeTouches(t, a->id, 0, b->id, b->subId);
    } else if (res.intersects()) {
      if (_cacheManager->isRefed(a->id) || !(res.coveredBy())) {
        writeNotTouches(t, a->id, 0, b->id, b->subId);
      }
    }

    // crosses
    if (res.crosses1vs2()) {
      _relStats[t].crosses++;
      writeRel(t, a->id, b->id, _cfg.sepCrosses);
    }
  } else if (isArea(cur.type) && sv.type == LINE) {
    std::shared_ptr<Area> a = getArea(cur, cur.large ? -1 : t);

    auto ts = TIME();
    auto b = _cacheManager->getLine(sv.id, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    _stats[t].areaCmps++;
    _stats[t].areaSizeSum += a->geom.area();

    _stats[t].lineCmps++;
    _stats[t].lineLenSum += b->geom.length();

    _stats[t].anchorSum += std::max(a->geom.size() / 2, b->geom.size() / 2);

    _stats[t].totalComps++;

    auto res = DE9IMCheck(b.get(), a.get(), t);

    // intersects
    if (res.intersects()) {
      writeIntersect(t, a->id, a->subId, b->id, b->subId);
    }

    // contains
    if (res.within()) {
      writeContains(t, a->id, a->subId, b->id, b->subId);
    }

    // covers
    if (res.coveredBy()) {
      writeCovers(t, a->id, a->subId, b->id, b->subId);
    }

    // touches
    if (res.touches()) {
      writeTouches(t, a->id, a->subId, b->id, b->subId);
    } else if (res.intersects()) {
      // if b is not a multi-geom, and is completey covered, we wont
      // be finding a touch as we assume non-self-intersecting geoms
      if (_cacheManager->isRefed(a->id) ||
          !(b->subId == 0 && res.coveredBy())) {
        writeNotTouches(t, a->id, a->subId, b->id, b->subId);
      }
    }

    // crosses
    if (res.crosses1vs2()) {
      _relStats[t].crosses++;
      writeRel(t, b->id, a->id, _cfg.sepCrosses);
    }
  } else if (isArea(cur.type) && isSimpleLine(sv.type)) {
    std::shared_ptr<Area> a = getArea(cur, cur.large ? -1 : t);

    auto ts = TIME();
    auto b = getSimpleLine(sv, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);

    _stats[t].areaCmps++;
    _stats[t].areaSizeSum += a->geom.area();

    _stats[t].lineCmps++;
    _stats[t].lineLenSum += util::geo::dist(sv.point, sv.point2);

    _stats[t].anchorSum += std::max(a->geom.size() / 2, (size_t)2);

    _stats[t].totalComps++;

    auto res = DE9IMCheck({sv.point, sv.point2}, a.get(), t);

    // intersects
    if (res.intersects()) {
      writeIntersect(t, a->id, a->subId, b->id, 0);
    }

    // contains
    if (res.within()) {
      writeContains(t, a->id, a->subId, b->id, 0);
    }

    // covers
    if (res.coveredBy()) {
      writeCovers(t, a->id, a->subId, b->id, 0);
    }

    // touches
    if (res.touches()) {
      writeTouches(t, a->id, a->subId, b->id, 0);
    } else if (res.intersects()) {
      if (_cacheManager->isRefed(a->id) || !res.coveredBy()) {
        writeNotTouches(t, a->id, a->subId, b->id, 0);
      }
    }

    // crosses
    if (res.crosses1vs2()) {
      _relStats[t].crosses++;
      writeRel(t, b->id, a->id, _cfg.sepCrosses);
    }
  } else if (cur.type == LINE && sv.type == LINE) {
    auto ts = TIME();
    auto a = _cacheManager->getLine(cur.id, cur.large ? -1 : t);
    auto b = _cacheManager->getLine(sv.id, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    if (a->id == b->id) return;  // no self-checks in multigeometries

    _stats[t].lineCmps++;
    _stats[t].lineLenSum += std::max(a->geom.length(), b->geom.length());

    _stats[t].anchorSum += std::max(a->geom.size() / 2, b->geom.size() / 2);

    _stats[t].totalComps++;

    auto res = DE9IMCheck(a.get(), b.get(), t);

    // intersects
    if (res.intersects()) {
      writeIntersect(t, a->id, a->subId, b->id, b->subId);
    }

    // covers
    if (res.coveredBy()) {
      writeNotCrosses(t, a->id, a->subId, b->id, b->subId);
      if (a->subId == 0) writeNotOverlaps(t, a->id, a->subId, b->id, b->subId);

      writeCovers(t, b->id, b->subId, a->id, a->subId);

      if (fabs(a->geom.length() - b->geom.length()) < util::geo::EPSILON) {
        // both lines were equivalent
        writeEquals(t, a->id, a->subId, b->id, b->subId);

        writeCovers(t, a->id, a->subId, b->id, b->subId);
      }
    }

    // touches
    if (res.touches()) {
      writeTouches(t, a->id, a->subId, b->id, b->subId);
    } else if (res.intersects()) {
      writeNotTouches(t, a->id, a->subId, b->id, b->subId);
    }

    // crosses
    if (res.crosses1vs1()) {
      writeNotOverlaps(t, a->id, a->subId, b->id, b->subId);
      writeCrosses(t, a->id, a->subId, b->id, b->subId);
    }

    // overlaps
    if (res.overlaps1()) {
      if (!res.coveredBy())
        writeNotCrosses(t, a->id, a->subId, b->id, b->subId);
      writeOverlaps(t, a->id, a->subId, b->id, b->subId);
    }
  } else if (cur.type == LINE && isSimpleLine(sv.type)) {
    auto ts = TIME();
    auto a = _cacheManager->getLine(cur.id, cur.large ? -1 : t);
    auto b = getSimpleLine(sv, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);

    _stats[t].lineCmps++;
    _stats[t].lineLenSum +=
        std::max(a->geom.length(), util::geo::dist(sv.point, sv.point2));

    _stats[t].anchorSum += std::max(a->geom.size() / 2, (size_t)2);

    _stats[t].totalComps++;

    auto res = DE9IMCheck({sv.point, sv.point2}, a.get(), t).transpose();

    // intersects
    if (res.intersects()) {
      writeIntersect(t, a->id, a->subId, b->id, 0);
    }

    // covers
    if (res.coveredBy()) {
      writeCovers(t, b->id, 0, a->id, a->subId);
      writeNotCrosses(t, a->id, a->subId, b->id, 0);

      if (fabs(a->geom.length() -
               util::geo::len(LineSegment<int32_t>(sv.point, sv.point2))) <
          util::geo::EPSILON) {
        // both lines were equivalent
        writeCovers(t, a->id, a->subId, b->id, 0);

        writeEquals(t, a->id, a->subId, b->id, 0);
      }
    }

    // touches
    if (res.touches()) {
      writeTouches(t, a->id, a->subId, b->id, 0);
    } else if (res.intersects()) {
      writeNotTouches(t, a->id, a->subId, b->id, 0);
    }

    // crosses
    if (res.crosses1vs1()) {
      writeNotOverlaps(t, a->id, a->subId, b->id, 0);
      writeCrosses(t, a->id, a->subId, b->id, 0);
    }

    // overlaps
    if (res.overlaps1()) {
      if (!res.coveredBy()) writeNotCrosses(t, a->id, a->subId, b->id, 0);
      writeOverlaps(t, a->id, a->subId, b->id, 0);
    }
  } else if (isSimpleLine(cur.type) && sv.type == LINE) {
    auto ts = TIME();
    auto a = getSimpleLine(cur, cur.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);
    ts = TIME();
    auto b = _cacheManager->getLine(sv.id, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    _stats[t].lineCmps++;
    _stats[t].lineLenSum +=
        std::max(b->geom.length(), util::geo::dist(cur.point, cur.point2));

    _stats[t].anchorSum += std::max(b->geom.size() / 2, (size_t)2);

    _stats[t].totalComps++;

    auto res = DE9IMCheck({cur.point, cur.point2}, b.get(), t);

    // intersects
    if (res.intersects()) {
      writeIntersect(t, a->id, 0, b->id, b->subId);
    }

    // covers
    if (res.coveredBy()) {
      writeNotCrosses(t, a->id, 0, b->id, b->subId);
      writeCovers(t, b->id, b->subId, a->id, 0);

      writeNotOverlaps(t, a->id, 0, b->id, b->subId);

      if (fabs(util::geo::len(LineSegment<int32_t>(cur.point, cur.point2)) -
               b->geom.length()) < util::geo::EPSILON) {
        writeEquals(t, a->id, 0, b->id, b->subId);

        writeCovers(t, a->id, 0, b->id, b->subId);
      }
    }

    // touches
    if (res.touches()) {
      writeTouches(t, a->id, 0, b->id, b->subId);
    } else if (res.intersects()) {
      writeNotTouches(t, a->id, 0, b->id, b->subId);
    }

    // crosses
    if (res.crosses1vs1()) {
      writeNotOverlaps(t, a->id, 0, b->id, b->subId);
      writeCrosses(t, a->id, 0, b->id, b->subId);
    }

    // overlaps
    if (res.overlaps1()) {
      if (!res.coveredBy()) writeNotCrosses(t, a->id, 0, b->id, b->subId);
      writeOverlaps(t, a->id, 0, b->id, b->subId);
    }
  } else if (isSimpleLine(cur.type) && isSimpleLine(sv.type)) {
    auto ts = TIME();
    auto a = getSimpleLine(cur, cur.large ? -1 : t);
    auto b = getSimpleLine(sv, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);

    _stats[t].lineCmps++;
    _stats[t].lineLenSum += std::max(util::geo::dist(cur.point, cur.point2),
                                     util::geo::dist(sv.point, sv.point2));

    _stats[t].anchorSum += 2;

    _stats[t].totalComps++;

    auto res = DE9IMCheck({cur.point, cur.point2}, {sv.point, sv.point2}, t);

    if (res.intersects()) {
      writeIntersect(t, a->id, 0, b->id, 0);
    }

    if (res.coveredBy()) {
      writeCovers(t, b->id, 0, a->id, 0);

      if (fabs(util::geo::len(LineSegment<int32_t>(cur.point, cur.point2)) -
               util::geo::len(LineSegment<int32_t>(sv.point, sv.point2))) <
          util::geo::EPSILON) {
        writeEquals(t, a->id, 0, b->id, 0);
        writeCovers(t, a->id, 0, b->id, 0);
      }
    }

    // touches
    if (res.touches()) {
      writeTouches(t, a->id, 0, b->id, 0);
    }

    // crosses
    if (res.crosses1vs1()) {
      writeCrosses(t, a->id, 0, b->id, 0);
    }

    // overlaps
    if (res.overlaps1()) {
      writeOverlaps(t, a->id, 0, b->id, 0);
    }
  } else if (isPoint(cur.type) && isPoint(sv.type)) {
    // point/point: trivial intersect & cover & contains

    auto a = getPoint(cur.id, cur.type, cur.large ? -1 : t);
    auto b = getPoint(sv.id, sv.type, sv.large ? -1 : t);

    _stats[t].anchorSum += 1;

    if (a->id == b->id)
      return;  // no self-checks in multigeometries
               //
    _stats[t].totalComps++;

    writeIntersect(t, a->id, a->subId, b->id, b->subId);
    writeEquals(t, a->id, a->subId, b->id, b->subId);

    writeCovers(t, b->id, b->subId, a->id, a->subId);
    writeContains(t, b->id, b->subId, a->id, a->subId);

    writeCovers(t, a->id, a->subId, b->id, b->subId);
    writeContains(t, a->id, a->subId, b->id, b->subId);
  } else if (isPoint(cur.type) && isSimpleLine(sv.type)) {
    auto p = cur.point;

    _stats[t].lineCmps++;
    _stats[t].lineLenSum += util::geo::dist(sv.point, sv.point2);

    _stats[t].anchorSum += 2;

    if (util::geo::contains(p, LineSegment<int32_t>(sv.point, sv.point2))) {
      auto a = getPoint(cur.id, cur.type, cur.large ? -1 : t);
      auto ts = TIME();
      auto b = getSimpleLine(sv, sv.large ? -1 : t);
      _stats[t].timeGeoCacheRetrievalSimpleLine += TOOK(ts);
      writeIntersect(t, a->id, a->subId, b->id, 0);

      writeCovers(t, b->id, 0, a->id, a->subId);

      if (p != sv.point && p != sv.point2) {
        writeContains(t, b->id, 0, a->id, a->subId);

        writeNotTouches(t, a->id, a->subId, b->id, 0);
      } else {
        writeTouches(t, a->id, a->subId, b->id, 0);
      }
    }
  } else if (isPoint(cur.type) && sv.type == LINE) {
    auto ts = TIME();
    auto a = cur.point;
    auto b = _cacheManager->getLine(sv.id, sv.large ? -1 : t);
    _stats[t].timeGeoCacheRetrievalLine += TOOK(ts);

    _stats[t].lineCmps++;
    _stats[t].lineLenSum += b->geom.length();

    _stats[t].anchorSum += b->geom.size() / 2;

    _stats[t].totalComps++;

    auto res = DE9IMCheck(a, b.get(), t);

    if (res.intersects()) {
      auto a = getPoint(cur.id, cur.type, cur.large ? -1 : t);

      if (a->id == b->id) return;  // no self-checks in multigeometries

      writeIntersect(t, a->id, a->subId, b->id, b->subId);

      writeCovers(t, b->id, b->subId, a->id, a->subId);

      if (res.within()) {
        writeContains(t, b->id, b->subId, a->id, a->subId);
        writeNotTouches(t, a->id, a->subId, b->id, b->subId);
      } else {
        writeTouches(t, a->id, a->subId, b->id, b->subId);
      }

      if (b->geom.length() == 0) {
        // zero length line, point also covers line

        writeCovers(t, a->id, a->subId, b->id, b->subId);
      }
    }
  } else if (isPoint(cur.type) && isArea(sv.type)) {
    std::shared_ptr<Area> b = getArea(sv, sv.large ? -1 : t);
    auto a = cur.point;

    _stats[t].totalComps++;

    auto res = DE9IMCheck(a, b.get(), t);

    if (res.coveredBy()) {
      auto a = getPoint(cur.id, cur.type, cur.large ? -1 : t);

      writeCovers(t, b->id, b->subId, a->id, a->subId);
      writeIntersect(t, a->id, a->subId, b->id, b->subId);

      if (res.within()) {
        writeContains(t, b->id, b->subId, a->id, a->subId);

        if (_cacheManager->isRefed(a->id) || a->subId != 0) {
          writeNotTouches(t, a->id, a->subId, b->id, b->subId);
        }
      } else {
        writeTouches(t, a->id, a->subId, b->id, b->subId);
      }
    }
  }
}

// _____________________________________________________________________________
void Sweeper::processQueue(size_t t) {
  try {
    JobBatch batch;
    while ((batch = _jobs.get()).size()) {
      for (const auto& job : batch) {
        if (_cancelled) break;

        if (job.multiOut.empty()) {
          if (_cfg.computeDE9IM) {
            doDE9IMCheck(job.boxVal, job.sweepVal, t);
          } else if (_cfg.withinDist >= 0) {
            doDistCheck(job.boxVal, job.sweepVal, t);
          } else {
            doCheck(job.boxVal, job.sweepVal, t);
          }
        } else {
          multiOut(t, job.multiOut);
        }
      }
    }
  } catch (const std::runtime_error& e) {
    std::stringstream ss;
    ss << "libspatialjoin: " << e.what();
    std::cerr << ss.str() << std::endl;
    std::exit(1);
  }

  _atomicCurX[t] = _curX[t];
}

// _____________________________________________________________________________
void Sweeper::fillBatch(
    JobBatch* batch, const util::geo::IntervalIdx<int32_t, SweepVal>* actives,
    const BoxVal* cur) const {
  const auto& overlaps = actives->overlap_find_all({cur->loY, cur->upY});

  for (const auto& p : overlaps) {
    // check if diagonal boxes intersect, if not, ignore this pair
    if (_cfg.useDiagBox && !util::geo::intersects(p.v.b45, cur->b45)) continue;

    JobVal a(*cur);
    JobVal b(p.v);

    // for simple lines, already check if the lines intersect, if not,
    // ignore
    if (isSimpleLine(a.type) && isSimpleLine(b.type) &&
        !util::geo::IntersectorLine<int32_t>::check(
            LineSegment<int32_t>(a.point, a.point2), 32767, true, 32767, true,
            LineSegment<int32_t>(b.point, b.point2), 32767, true, 32767, true))
      continue;

    batch->push_back({a, b, ""});
  }
}

// _____________________________________________________________________________
void Sweeper::writeOverlaps(size_t t, const std::string& a, size_t aSub,
                            const std::string& b, size_t bSub) {
  if (a != b) {
    if (aSub == 0 && bSub == 0) {
      _relStats[t].overlaps++;
      writeRel(t, a, b, _cfg.sepOverlaps);
      _relStats[t].overlaps++;
      writeRel(t, b, a, _cfg.sepOverlaps);
    } else {
      std::unique_lock<std::mutex> lock(_mutsOverlaps[t]);

      if (bSub != 0) _subOverlaps[t][b].insert(a);
      if (aSub != 0) _subOverlaps[t][a].insert(b);
    }
  }

  if (!_cacheManager->hasRefs()) return;

  // handle references

  const auto* referersA = _cacheManager->getRefs(a, aSub);
  const auto* referersB = _cacheManager->getRefs(b, bSub);

  if (referersB) {
    for (const auto& idB : *referersB) {
      writeOverlaps(t, a, aSub, idB.first, idB.second);
    }
  }

  // no need to check exactly the same direction again
  if (a != b || aSub != bSub) {
    if (referersA) {
      for (const auto& idA : *referersA) {
        writeOverlaps(t, idA.first, idA.second, b, bSub);
      }
    }
  }
}

// _____________________________________________________________________________
void Sweeper::writeNotOverlaps(size_t t, const std::string& a, size_t aSub,
                               const std::string& b, size_t bSub) {
  if (a != b && (aSub != 0 || bSub != 0)) {
    std::unique_lock<std::mutex> lock(_mutsNotOverlaps[t]);

    if (bSub != 0) _subNotOverlaps[t][b].insert(a);
    if (aSub != 0) _subNotOverlaps[t][a].insert(b);
  }

  if (!_cacheManager->hasRefs()) return;

  // handle references

  const auto* referersA = _cacheManager->getRefs(a, aSub);
  const auto* referersB = _cacheManager->getRefs(b, bSub);

  if (referersB) {
    for (const auto& idB : *referersB) {
      writeNotOverlaps(t, a, aSub, idB.first, idB.second);
    }
  }

  if (referersA) {
    for (const auto& idA : *referersA) {
      writeNotOverlaps(t, idA.first, idA.second, b, bSub);
    }
  }
}

// _____________________________________________________________________________
void Sweeper::writeCrosses(size_t t, const std::string& a, size_t aSub,
                           const std::string& b, size_t bSub) {
  if (a == b) return;

  if (aSub == 0 && bSub == 0) {
    _relStats[t].crosses++;
    writeRel(t, a, b, _cfg.sepCrosses);
    _relStats[t].crosses++;
    writeRel(t, b, a, _cfg.sepCrosses);
  } else {
    std::unique_lock<std::mutex> lock(_mutsCrosses[t]);

    if (bSub != 0) _subCrosses[t][b].insert(a);
    if (aSub != 0) _subCrosses[t][a].insert(b);
  }

  if (!_cacheManager->hasRefs()) return;

  // handle references

  const auto* referersA = _cacheManager->getRefs(a, aSub);
  const auto* referersB = _cacheManager->getRefs(b, bSub);

  if (referersB) {
    for (const auto& idB : *referersB) {
      writeCrosses(t, a, aSub, idB.first, idB.second);
    }
  }

  // no need to check exactly the same direction again
  if (a != b || aSub != bSub) {
    if (referersA) {
      for (const auto& idA : *referersA) {
        writeCrosses(t, idA.first, idA.second, b, bSub);
      }
    }
  }
}

// _____________________________________________________________________________
void Sweeper::writeNotCrosses(size_t t, const std::string& a, size_t aSub,
                              const std::string& b, size_t bSub) {
  if (a != b && (aSub != 0 || bSub != 0)) {
    std::unique_lock<std::mutex> lock(_mutsNotCrosses[t]);

    if (bSub != 0) _subNotCrosses[t][b].insert(a);
    if (aSub != 0) _subNotCrosses[t][a].insert(b);
  }

  if (!_cacheManager->hasRefs()) return;

  // handle references

  const auto* referersA = _cacheManager->getRefs(a, aSub);
  const auto* referersB = _cacheManager->getRefs(b, bSub);

  if (referersB) {
    for (const auto& idB : *referersB) {
      writeNotCrosses(t, a, aSub, idB.first, idB.second);
    }
  }

  // no need to check exactly the same direction again
  if (a != b || aSub != bSub) {
    if (referersA) {
      for (const auto& idA : *referersA) {
        writeNotCrosses(t, idA.first, idA.second, b, bSub);
      }
    }
  }
}

// _____________________________________________________________________________
void Sweeper::writeTouches(size_t t, const std::string& a, size_t aSub,
                           const std::string& b, size_t bSub) {
  if (a == b) return;

  if (aSub == 0 && bSub == 0) {
    _relStats[t].touches++;
    writeRel(t, a, b, _cfg.sepTouches);
    _relStats[t].touches++;
    writeRel(t, b, a, _cfg.sepTouches);
  } else {
    std::unique_lock<std::mutex> lock(_mutsTouches[t]);

    if (bSub != 0) _subTouches[t][b].insert(a);
    if (aSub != 0) _subTouches[t][a].insert(b);
  }

  if (!_cacheManager->hasRefs()) return;

  // handle references

  const auto* referersA = _cacheManager->getRefs(a, aSub);
  const auto* referersB = _cacheManager->getRefs(b, bSub);

  if (referersB) {
    for (const auto& idB : *referersB) {
      writeTouches(t, a, aSub, idB.first, idB.second);
    }
  }

  // no need to check exactly the same direction again
  if (a != b || aSub != bSub) {
    if (referersA) {
      for (const auto& idA : *referersA) {
        writeTouches(t, idA.first, idA.second, b, bSub);
      }
    }
  }
}

// _____________________________________________________________________________
void Sweeper::writeNotTouches(size_t t, const std::string& a, size_t aSub,
                              const std::string& b, size_t bSub) {
  if (a != b && (aSub != 0 || bSub != 0)) {
    std::unique_lock<std::mutex> lock(_mutsNotTouches[t]);

    if (bSub != 0) _subNotTouches[t][b].insert(a);
    if (aSub != 0) _subNotTouches[t][a].insert(b);
  }

  if (!_cacheManager->hasRefs()) return;

  // handle references

  const auto* referersA = _cacheManager->getRefs(a, aSub);
  const auto* referersB = _cacheManager->getRefs(b, bSub);

  if (referersB) {
    for (const auto& idB : *referersB) {
      writeNotTouches(t, a, aSub, idB.first, idB.second);
    }
  }

  // no need to check exactly the same direction again
  if (a != b || aSub != bSub) {
    if (referersA) {
      for (const auto& idA : *referersA) {
        writeNotTouches(t, idA.first, idA.second, b, bSub);
      }
    }
  }
}

// _____________________________________________________________________________
void Sweeper::writeEquals(size_t t, const std::string& a, size_t aSub,
                          const std::string& b, size_t bSub) {
  if (a != b) {
    if (aSub == 0 && bSub == 0) {
      writeRel(t, a, b, _cfg.sepEquals);
      _relStats[t].equals++;
      writeRel(t, b, a, _cfg.sepEquals);
      _relStats[t].equals++;
    } else if (aSub == 0 || bSub == 0) {
      writeNotOverlaps(t, a, aSub, b, bSub);
    } else if (_cacheManager->subSize(a) != _cacheManager->subSize(b)) {
    } else {
      std::unique_lock<std::mutex> lock(_mutsEquals[t]);

      _subEquals[t][b][a].insert(aSub);
      _subEquals[t][a][b].insert(bSub);
    }
  }

  if (!_cacheManager->hasRefs()) return;

  // handle references

  const auto* referersA = _cacheManager->getRefs(a, aSub);
  const auto* referersB = _cacheManager->getRefs(b, bSub);

  if (referersB) {
    for (const auto& idB : *referersB) {
      writeEquals(t, a, aSub, idB.first, idB.second);
    }
  }

  // no need to check exactly the same direction again
  if (a != b || aSub != bSub) {
    if (referersA) {
      for (const auto& idA : *referersA) {
        writeEquals(t, idA.first, idA.second, b, bSub);
      }
    }
  }
}

// _____________________________________________________________________________
void Sweeper::writeCovers(size_t t, const std::string& a, size_t aSub,
                          const std::string& b, size_t bSub) {
  if (a != b) {
    if (bSub > 0) {
      std::unique_lock<std::mutex> lock(_mutsCovers[t]);
      _subCovered[t][b][a].insert(bSub);
    } else {
      writeRel(t, a, b, _cfg.sepCovers);
      _relStats[t].covers++;
    }
  }

  if (!_cacheManager->hasRefs()) return;

  // handle references

  const auto* referersA = _cacheManager->getRefs(a, aSub);
  const auto* referersB = _cacheManager->getRefs(b, bSub);

  if (referersB) {
    for (const auto& idB : *referersB) {
      writeCovers(t, a, aSub, idB.first, idB.second);
    }
  }

  if (referersA) {
    for (const auto& idA : *referersA) {
      writeCovers(t, idA.first, idA.second, b, bSub);
    }
  }
}

// _____________________________________________________________________________
void Sweeper::writeContains(size_t t, const std::string& a, size_t aSub,
                            const std::string& b, size_t bSub) {
  if (a != b) {
    if (bSub > 0) {
      std::unique_lock<std::mutex> lock(_mutsContains[t]);
      _subContains[t][b][a].insert(bSub);
    } else {
      writeRel(t, a, b, _cfg.sepContains);
      _relStats[t].contains++;
    }
  }

  if (!_cacheManager->hasRefs()) return;

  // handle references
  const auto* referersA = _cacheManager->getRefs(a, aSub);
  const auto* referersB = _cacheManager->getRefs(b, bSub);

  if (referersB) {
    for (const auto& idB : *referersB) {
      writeContains(t, a, aSub, idB.first, idB.second);
    }
  }

  if (referersA) {
    for (const auto& idA : *referersA) {
      writeContains(t, idA.first, idA.second, b, bSub);
    }
  }
}

// _____________________________________________________________________________
bool Sweeper::notCrosses(const std::string& a, const std::string& b) {
  for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
    std::unique_lock<std::mutex> lock(_mutsNotCrosses[t]);
    auto i = _subNotCrosses[t].find(a);
    if (i != _subNotCrosses[t].end() && i->second.count(b)) return true;
  }

  return false;
}

// _____________________________________________________________________________
bool Sweeper::notTouches(const std::string& a, const std::string& b) {
  for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
    std::unique_lock<std::mutex> lock(_mutsNotTouches[t]);
    auto i = _subNotTouches[t].find(a);
    if (i != _subNotTouches[t].end() && i->second.count(b)) return true;
  }

  return false;
}

// _____________________________________________________________________________
bool Sweeper::notOverlaps(const std::string& a, const std::string& b) {
  for (size_t t = 0; t < _cfg.numThreads + 1; t++) {
    std::unique_lock<std::mutex> lock(_mutsNotOverlaps[t]);
    auto i = _subNotOverlaps[t].find(a);
    if (i != _subNotOverlaps[t].end() && i->second.find(b) != i->second.end())
      return true;
  }

  return false;
}

// _____________________________________________________________________________
void Sweeper::log(const std::string& msg) {
  if (_cfg.logCb) _cfg.logCb(msg);
}

// _____________________________________________________________________________
std::pair<double, double> Sweeper::getMinMaxLocalScaleFactors(
    const I32Box& boxA, const I32Box& boxB, double distanceUpperBound) {
  auto withinBox = util::geo::extendBox(boxA, boxB);

  // convert distanceUpperBound (meters) to maximum latitude padding (degrees)
  // we have to "pad" each box by -dy and dy because the distance path could
  // be within that padded box, and thus the distortions have to be computed
  // based on that path
  double dLat =
      distanceUpperBound / util::geo::MIN_METERS_PER_LAT_RAD * util::geo::IRAD;

  // the lower extrema of the bounding boxes are padded by dLat
  auto withinLow = util::geo::webMercToLatLng<double>(
      0.0, withinBox.getLowerLeft().getY() * 1.0 / PREC);
  auto aLow = util::geo::webMercToLatLng<double>(
      0.0, boxA.getLowerLeft().getY() * 1.0 / PREC);
  aLow.setY(aLow.getY() - dLat);
  auto bLow = util::geo::webMercToLatLng<double>(
      0.0, boxB.getLowerLeft().getY() * 1.0 / PREC);
  bLow.setY(bLow.getY() - dLat);

  // the upper extrema of the bounding boxes are padded by dLat
  auto withinUp = util::geo::webMercToLatLng<double>(
      0.0, withinBox.getUpperRight().getY() * 1.0 / PREC);
  auto aUp = util::geo::webMercToLatLng<double>(
      0.0, boxA.getUpperRight().getY() * 1.0 / PREC);
  aUp.setY(aUp.getY() + dLat);
  auto bUp = util::geo::webMercToLatLng<double>(
      0.0, boxB.getUpperRight().getY() * 1.0 / PREC);
  bUp.setY(bUp.getY() + dLat);

  double yRangeMin = std::min(
      90.0 - util::geo::EPSILON,
      std::max(withinLow.getY() * 1.0, std::max(aLow.getY(), bLow.getY())));
  double yRangeMax = std::max(
      -90.0 - util::geo::EPSILON,
      std::min(withinUp.getY() * 1.0, std::min(aUp.getY(), bUp.getY())));

  double a = cos(yRangeMin * util::geo::RAD);
  double b = cos(yRangeMax * util::geo::RAD);

  // if we crossed the pole, we encountered a scale factor of 1!
  if (withinLow.getY() < 0 && withinUp.getY() > 0) {
    return {std::min(a, b), std::max(1.0, std::max(a, b))};
  }

  return {std::min(a, b), std::max(a, b)};
}

// _____________________________________________________________________________
double Sweeper::noSearchPadding(double euclideanDistanceUpperBound, double,
                                const I32Box&, const I32Box&) {
  return euclideanDistanceUpperBound;
}

// _____________________________________________________________________________
double Sweeper::localSearchPadding(double euclideanDistanceUpperBound,
                                   double distanceUpperBound,
                                   const I32Box& boxA, const I32Box& boxB) {
  auto minScaleALow =
      getMinMaxLocalScaleFactors(util::geo::getBoundingBox(boxA.getLowerLeft()),
                                 boxB, distanceUpperBound)
          .second;
  auto minScaleAUp = getMinMaxLocalScaleFactors(
                         util::geo::getBoundingBox(boxA.getUpperRight()), boxB,
                         distanceUpperBound)
                         .second;
  auto minScaleBLow =
      getMinMaxLocalScaleFactors(util::geo::getBoundingBox(boxB.getLowerLeft()),
                                 boxA, distanceUpperBound)
          .second;
  auto minScaleBUp = getMinMaxLocalScaleFactors(
                         util::geo::getBoundingBox(boxB.getUpperRight()), boxA,
                         distanceUpperBound)
                         .second;

  double min = std::min(std::min(minScaleALow, minScaleAUp),
                        std::min(minScaleBLow, minScaleBUp));
  auto a = getMinMaxLocalScaleFactors(boxA, boxB, distanceUpperBound);
  double max = a.second;

  double factorNew2 = max / min;

  double minEuclideanXDist = util::geo::dist(
      LineSegment<int32_t>{I32Point{boxA.getLowerLeft().getX(), 0},
                           I32Point{boxA.getUpperRight().getX(), 0}},
      LineSegment<int32_t>{I32Point{boxB.getLowerLeft().getX(), 0},
                           I32Point{boxB.getUpperRight().getX(), 0}});
  double minEuclideanYDist = util::geo::dist(
      LineSegment<int32_t>{I32Point{0, boxA.getLowerLeft().getY()},
                           I32Point{0, boxA.getUpperRight().getY()}},
      LineSegment<int32_t>{I32Point{0, boxB.getLowerLeft().getY()},
                           I32Point{0, boxB.getUpperRight().getY()}});

  double padding = factorNew2 * euclideanDistanceUpperBound;
  auto xPadding = (sqrt(std::max(
      0.0, padding * padding - minEuclideanYDist * minEuclideanYDist)));
  auto yPadding = (sqrt(std::max(
      0.0, padding * padding - minEuclideanXDist * minEuclideanXDist)));

  auto paddedA = util::geo::pad(boxA, xPadding, yPadding);

  auto boxBStar = util::geo::intersection(paddedA, boxB);

  double min2 = std::numeric_limits<double>::infinity();
  double max2 = 0;

  std::vector<I32Point> cornerA = {boxA.getLowerLeft(), boxA.getLowerRight(),
                                   boxA.getUpperRight(), boxA.getUpperLeft()};
  std::vector<I32Point> cornerB = {
      boxBStar.getLowerLeft(), boxBStar.getLowerRight(),
      boxBStar.getUpperRight(), boxBStar.getUpperLeft()};

  for (size_t i = 0; i < cornerA.size(); i++) {
    for (size_t j = 0; j < cornerB.size(); j++) {
      double eucD = util::geo::dist(cornerA[i], cornerB[j]);
      double mD = meterDist(cornerA[i], cornerB[j],
                            std::numeric_limits<double>::infinity());
      if (mD / eucD < min2) min2 = mD / eucD;
      if (mD / eucD > max2) max2 = mD / eucD;
    }
  }

  double factorNew3 = max2 / min2;

  if (factorNew2 < factorNew3) return factorNew2 * euclideanDistanceUpperBound;

  return factorNew3 * euclideanDistanceUpperBound;
}

// _____________________________________________________________________________
double Sweeper::getMaxScaleFactor(const I32Box& bbox) const {
  double invScaleFactor = std::min(
      util::geo::webMercDistFactor(I32Point{bbox.getLowerLeft().getX() / PREC,
                                            bbox.getLowerLeft().getY() / PREC}),
      util::geo::webMercDistFactor(
          I32Point{bbox.getUpperRight().getX() / PREC,
                   bbox.getUpperRight().getY() / PREC}));

  return 1.0 / invScaleFactor;
}

// _____________________________________________________________________________
double Sweeper::getMaxScaleFactor(const I32Point& p) const {
  return 1.0 / util::geo::webMercDistFactor(
                   I32Point{p.getX() / PREC, p.getY() / PREC});
}

// _____________________________________________________________________________
double Sweeper::euclideanDist(const I32Point& p1, const I32Point& p2, double) {
  return util::geo::dist(p1, p2) / PREC;
}

// _____________________________________________________________________________
double Sweeper::meterDist(const I32Point& p1, const I32Point& p2,
                          double maxDist) {
  auto fp1 = FPoint{static_cast<float>((p1.getX() * 1.0) / (PREC * 1.0)),
                    static_cast<float>((p1.getY() * 1.0) / (PREC * 1.0))};
  auto fp2 = FPoint{static_cast<float>((p2.getX() * 1.0) / (PREC * 1.0)),
                    static_cast<float>((p2.getY() * 1.0) / (PREC * 1.0))};

  double dX = fp2.getX() - fp1.getX();
  double dY = fp2.getY() - fp1.getY();

  // get max absolute latitude (distance distortion)
  double distFactor = std::min(util::geo::webMercDistFactor(fp1),
                               util::geo::webMercDistFactor(fp2));

  // early abort
  if (std::abs(dY) > maxDist / distFactor)
    return std::numeric_limits<double>::max();
  if (std::abs(dX) > maxDist / distFactor)
    return std::numeric_limits<double>::max();
  if ((dX * dX + dY * dY) > (maxDist / distFactor) * (maxDist / distFactor))
    return std::numeric_limits<double>::max();

  return util::geo::webMercMeterDist(fp1, fp2);
}

// _____________________________________________________________________________
double Sweeper::distCheck(const I32Point& a, const Point* aMeta, const Area* b,
                          size_t t) {
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect({{1, 0}, {getBoxId(a), 0}}, b->boxIds);
    _stats[t].timeBoxIdIsectAreaPoint += TOOK(ts);

    // all boxes of a are fully contained in b, we are contained
    if (r.first) return 0;
  }

  auto ts = TIME();
  double maxD = _cfg.withinDist;

  maxD = std::min(maxD,
                  getMaxMultiDist(aMeta->id, aMeta->subId, a, b->id, b->subId,
                                  b->geom.getOuter().rawRing().front().p, t));

  auto scale = _cfg.euclideanDist && !_cfg.haversineApprox
                   ? std::pair<double, double>{1, 1}
                   : getMinMaxLocalScaleFactors(util::geo::getBoundingBox(a),
                                                b->geom.boundingBox(), maxD);

  double maxEuclideanDist = maxD / scale.first * PREC;

  auto dist = util::geo::withinDist<int32_t>(
      a, b->geom, maxD,
      _cfg.euclideanDist ? &Sweeper::noSearchPadding
                         : &Sweeper::localSearchPadding,
      maxEuclideanDist,
      _cfg.euclideanDist && !_cfg.haversineApprox ? &Sweeper::euclideanDist
                                                  : &Sweeper::meterDist);

  _stats[t].timeFullGeoCheckAreaPoint += TOOK(ts);
  _stats[t].fullGeoChecksAreaPoint++;

  return dist;
}

// _____________________________________________________________________________
util::geo::DE9IMatrix Sweeper::DE9IMCheck(const I32Point& a, const Area* b,
                                          size_t t) const {
  _stats[t].totalComps++;
  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect({{1, 0}, {getBoxId(a), 0}}, b->boxIds);
    _stats[t].timeBoxIdIsectAreaPoint += TOOK(ts);

    // all boxes of a are fully contained in b, we are contained
    if (r.first) return util::geo::M0FFFFF212;

    // no box shared, we cannot contain or intersect
    if (r.first + r.second == 0) return util::geo::MFF0FFF212;
  }

  if (_cfg.useOBB && b->obb.getOuter().rawRing().size()) {
    auto ts = TIME();
    auto r = containsCovers(a, b->obb);
    _stats[t].timeOBBIsectAreaPoint += TOOK(ts);
    if (!std::get<1>(r)) return util::geo::MFF0FFF212;
  }

  auto ts = TIME();
  auto res = util::geo::DE9IM(a, b->geom);
  _stats[t].timeFullGeoCheckAreaPoint += TOOK(ts);
  _stats[t].fullGeoChecksAreaPoint++;

  return res;
}

// _____________________________________________________________________________
double Sweeper::distCheck(const I32Point& a, const Point* aMeta, const Line* b,
                          size_t t) {
  auto ts = TIME();
  double maxD = _cfg.withinDist;

  maxD =
      std::min(maxD, getMaxMultiDist(aMeta->id, aMeta->subId, a, b->id,
                                     b->subId, b->geom.rawLine().front().p, t));

  auto scale = _cfg.euclideanDist && !_cfg.haversineApprox
                   ? std::pair<double, double>{1.0, 1.0}
                   : getMinMaxLocalScaleFactors(util::geo::getBoundingBox(a),
                                                b->geom.boundingBox(), maxD);

  double maxEuclideanDist = maxD / scale.first * PREC;

  auto dist = util::geo::withinDist<int32_t>(
      a, b->geom, maxD,
      _cfg.euclideanDist ? &Sweeper::noSearchPadding
                         : &Sweeper::localSearchPadding,
      maxEuclideanDist,
      _cfg.euclideanDist && !_cfg.haversineApprox ? &Sweeper::euclideanDist
                                                  : &Sweeper::meterDist);

  _stats[t].timeFullGeoCheckLinePoint += TOOK(ts);
  _stats[t].fullGeoChecksLinePoint++;

  return dist;
}

// _____________________________________________________________________________
util::geo::DE9IMatrix Sweeper::DE9IMCheck(const I32Point& a,
                                          const LineSegment<int32_t>& b,
                                          size_t t) const {
  _stats[t].totalComps++;
  auto ts = TIME();
  auto res = util::geo::DE9IM(a, util::geo::XSortedLine<int32_t>(b));
  _stats[t].timeFullGeoCheckLinePoint += TOOK(ts);
  _stats[t].fullGeoChecksLinePoint++;

  return res;
}

// _____________________________________________________________________________
double Sweeper::distCheck(const I32Point& a, const LineSegment<int32_t>& b,
                          size_t t) {
  auto ts = TIME();

  auto p2 = projectOn(b.first, a, b.second);

  auto dist = _cfg.euclideanDist && !_cfg.haversineApprox
                  ? Sweeper::euclideanDist(a, p2, _cfg.withinDist)
                  : Sweeper::meterDist(a, p2, _cfg.withinDist);

  _stats[t].timeFullGeoCheckLinePoint += TOOK(ts);
  _stats[t].fullGeoChecksLinePoint++;

  return dist;
}

// _____________________________________________________________________________
double Sweeper::distCheck(const LineSegment<int32_t>& a,
                          const LineSegment<int32_t>& b, size_t t) {
  auto ts = TIME();

  auto dist = util::geo::dist<int32_t>(
      a, b,
      _cfg.euclideanDist && !_cfg.haversineApprox ? &Sweeper::euclideanDist
                                                  : &Sweeper::meterDist);

  _stats[t].timeFullGeoCheckLineLine += TOOK(ts);
  _stats[t].fullGeoChecksLineLine++;

  return dist;
}

// _____________________________________________________________________________
double Sweeper::distCheck(const LineSegment<int32_t>& a, const Line* b,
                          size_t t) {
  auto ts = TIME();
  auto scale =
      _cfg.euclideanDist && !_cfg.haversineApprox
          ? std::pair<double, double>{1.0, 1.0}
          : getMinMaxLocalScaleFactors(util::geo::getBoundingBox(a),
                                       b->geom.boundingBox(), _cfg.withinDist);

  double maxEuclideanDist = _cfg.withinDist / scale.first * PREC;

  auto dist = util::geo::withinDist<int32_t>(
      I32XSortedLine(a), b->geom, _cfg.withinDist,
      _cfg.euclideanDist ? &Sweeper::noSearchPadding
                         : &Sweeper::localSearchPadding,
      maxEuclideanDist,
      _cfg.euclideanDist && !_cfg.haversineApprox ? &Sweeper::euclideanDist
                                                  : &Sweeper::meterDist);

  _stats[t].timeFullGeoCheckAreaLine += TOOK(ts);
  _stats[t].fullGeoChecksAreaLine++;

  return dist;
}

// _____________________________________________________________________________
double Sweeper::distCheck(const Line* a, const Line* b, size_t t) {
  auto ts = TIME();
  if (a == b) return 0;

  double maxD = _cfg.withinDist;

  maxD = std::min(
      maxD, getMaxMultiDist(a->id, a->subId, a->geom.rawLine().front().p, b->id,
                            b->subId, b->geom.rawLine().front().p, t));

  auto scale = _cfg.euclideanDist && !_cfg.haversineApprox
                   ? std::pair<double, double>{1.0, 1.0}
                   : getMinMaxLocalScaleFactors(a->geom.boundingBox(),
                                                b->geom.boundingBox(), maxD);

  double maxEuclideanDist = maxD / scale.first * PREC;

  auto dist = util::geo::withinDist<int32_t>(
      a->geom, b->geom, maxD,
      _cfg.euclideanDist ? &Sweeper::noSearchPadding
                         : &Sweeper::localSearchPadding,
      maxEuclideanDist,
      _cfg.euclideanDist && !_cfg.haversineApprox ? &Sweeper::euclideanDist
                                                  : &Sweeper::meterDist);

  _stats[t].timeFullGeoCheckLineLine += TOOK(ts);
  _stats[t].fullGeoChecksLineLine++;

  return dist;
}

// _____________________________________________________________________________
double Sweeper::distCheck(const LineSegment<int32_t>& a, const Area* b,
                          size_t t) {
  auto ts = TIME();

  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect({{1, 0}, {getBoxId(a.first), 0}}, b->boxIds);
    _stats[t].timeBoxIdIsectAreaLine += TOOK(ts);

    if (r.first) return 0;
  }

  auto scale =
      _cfg.euclideanDist && !_cfg.haversineApprox
          ? std::pair<double, double>{1.0, 1.0}
          : getMinMaxLocalScaleFactors(util::geo::getBoundingBox(a),
                                       b->geom.boundingBox(), _cfg.withinDist);

  double maxEuclideanDist = _cfg.withinDist / scale.first * PREC;

  auto dist = util::geo::withinDist<int32_t>(
      I32XSortedLine(a), b->geom, _cfg.withinDist,
      _cfg.euclideanDist ? &Sweeper::noSearchPadding
                         : &Sweeper::localSearchPadding,
      maxEuclideanDist,
      _cfg.euclideanDist && !_cfg.haversineApprox ? &Sweeper::euclideanDist
                                                  : &Sweeper::meterDist);

  _stats[t].timeFullGeoCheckAreaLine += TOOK(ts);
  _stats[t].fullGeoChecksAreaLine++;

  return dist;
}

// _____________________________________________________________________________
double Sweeper::distCheck(const Line* a, const Area* b, size_t t) {
  auto ts = TIME();

  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect(a->boxIds, b->boxIds);
    _stats[t].timeBoxIdIsectAreaLine += TOOK(ts);

    // all boxes of a are fully contained in b, we intersect and we are
    // contained
    if (r.first) return 0;
  }

  double maxD = _cfg.withinDist;

  maxD = std::min(
      maxD,
      getMaxMultiDist(a->id, a->subId, a->geom.rawLine().front().p, b->id,
                      b->subId, b->geom.getOuter().rawRing().front().p, t));

  auto scale = _cfg.euclideanDist && !_cfg.haversineApprox
                   ? std::pair<double, double>{1.0, 1.0}
                   : getMinMaxLocalScaleFactors(a->geom.boundingBox(),
                                                b->geom.boundingBox(), maxD);

  double maxEuclideanDist = maxD / scale.first * PREC;

  auto dist = util::geo::withinDist<int32_t>(
      a->geom, b->geom, maxD,
      _cfg.euclideanDist ? &Sweeper::noSearchPadding
                         : &Sweeper::localSearchPadding,
      maxEuclideanDist,
      _cfg.euclideanDist && !_cfg.haversineApprox ? &Sweeper::euclideanDist
                                                  : &Sweeper::meterDist);

  _stats[t].timeFullGeoCheckAreaLine += TOOK(ts);
  _stats[t].fullGeoChecksAreaLine++;

  return dist;
}

// _____________________________________________________________________________
double Sweeper::distCheck(const Area* a, const Area* b, size_t t) {
  auto ts = TIME();

  // cheap equivalence check
  if (a->geom == b->geom) {
    return 0;
  }

  if (_cfg.useBoxIds) {
    auto ts = TIME();
    auto r = boxIdIsect(a->boxIds, b->boxIds);
    _stats[t].timeBoxIdIsectAreaArea += TOOK(ts);

    // all boxes of a are fully contained in b, we intersect and we are
    // contained and we do not touch or overlap
    if (r.first) return 0;
  }

  double maxD = _cfg.withinDist;

  maxD = std::min(
      maxD, getMaxMultiDist(
                a->id, a->subId, a->geom.getOuter().rawRing().front().p, b->id,
                b->subId, b->geom.getOuter().rawRing().front().p, t));

  auto scale = _cfg.euclideanDist && !_cfg.haversineApprox
                   ? std::pair<double, double>{1.0, 1.0}
                   : getMinMaxLocalScaleFactors(a->geom.boundingBox(),
                                                b->geom.boundingBox(), maxD);

  double maxEuclideanDist = maxD / scale.first * PREC;

  auto dist = util::geo::withinDist<int32_t>(
      a->geom, b->geom, maxD,
      _cfg.euclideanDist ? &Sweeper::noSearchPadding
                         : &Sweeper::localSearchPadding,
      maxEuclideanDist,
      _cfg.euclideanDist && !_cfg.haversineApprox ? &Sweeper::euclideanDist
                                                  : &Sweeper::meterDist);
  _stats[t].timeFullGeoCheckAreaArea += TOOK(ts);
  _stats[t].fullGeoChecksAreaArea++;

  return dist;
}

// _____________________________________________________________________________
std::shared_ptr<sj::Point> Sweeper::getPoint(size_t id, GeomType gt,
                                             size_t t) const {
  std::shared_ptr<sj::Point> ret;
  auto ts = TIME();
  if (gt == sj::FOLDED_POINT) {
    ret = std::make_shared<sj::Point>(sj::Point{unfoldString(id), 0});
  } else {
    ret = _cacheManager->getPoint(id, t);
  }
  _stats[t].timeGeoCacheRetrievalPoint += TOOK(ts);

  return ret;
}

// _____________________________________________________________________________
std::shared_ptr<sj::SimpleLine> Sweeper::getSimpleLine(const JobVal& cur,
                                                       size_t t) const {
  if (cur.type == sj::FOLDED_SIMPLE_LINE) {
    return std::make_shared<sj::SimpleLine>(
        sj::SimpleLine{unfoldString(cur.id)});
  }

  return _cacheManager->getSimpleLine(cur.id, t);
}

// _____________________________________________________________________________
std::shared_ptr<sj::Area> Sweeper::getArea(const JobVal& sv, size_t t) const {
  auto ts = TIME();
  std::shared_ptr<Area> asp;

  if (sv.type == SIMPLE_POLYGON) {
    auto p = _cacheManager->getSimpleArea(sv.id, sv.large ? -1 : t);
    asp = std::make_shared<sj::Area>(sj::Area(areaFromSimpleArea(p.get())));
  } else if (sv.type == FOLDED_BOX_POLYGON) {
    SimpleArea sa;
    sa.id = unfoldString(sv.id);
    sa.geom = util::geo::Polygon<int32_t>(
                  getBoundingBox(util::geo::Line<int32_t>{sv.point, sv.point2}))
                  .getOuter();
    asp = std::make_shared<sj::Area>(sj::Area(areaFromSimpleArea(&sa)));
  } else {
    asp = _cacheManager->getArea(sv.id, sv.large ? -1 : t);
  }

  _stats[t].timeGeoCacheRetrievalArea += TOOK(ts);

  return asp;
}

// _____________________________________________________________________________
double Sweeper::getMaxMultiDist(const std::string& idA, size_t aSub,
                                const I32Point& leftAPoint,
                                const std::string& idB, size_t bSub,
                                const I32Point& leftBPoint, size_t t) {
  double maxD = _cfg.withinDist;
  // for multigeometries, we may already have a minimum distance above which we
  // are not required to search
  if (aSub > 0) {
    const auto& rightPointA = _cacheManager->multiRightPoint(idA);
    double d = _cfg.euclideanDist && !_cfg.haversineApprox
                   ? Sweeper::euclideanDist(rightPointA, leftBPoint, maxD)
                   : Sweeper::meterDist(rightPointA, leftBPoint, maxD);
    maxD = std::min(maxD, d);
    std::unique_lock<std::mutex> lock(_mutsDistance[t]);
    if (_subDistance[t][idA].find(idB) != _subDistance[t][idA].end()) {
      maxD = std::min(maxD, _subDistance[t][idA][idB]);
    }
  }
  if (bSub > 0) {
    const auto& rightPointB = _cacheManager->multiRightPoint(idB);
    double d = _cfg.euclideanDist && !_cfg.haversineApprox
                   ? Sweeper::euclideanDist(rightPointB, leftAPoint, maxD)
                   : Sweeper::meterDist(rightPointB, leftAPoint, maxD);
    maxD = std::min(maxD, d);
    std::unique_lock<std::mutex> lock(_mutsDistance[t]);
    if (_subDistance[t][idB].find(idA) != _subDistance[t][idB].end()) {
      maxD = std::min(maxD, _subDistance[t][idB][idA]);
    }
  }

  return maxD;
}

// _____________________________________________________________________________
std::string Sweeper::unfoldString(size_t folded) {
  // shift by 7 bytes to get size
  size_t n = folded >> 56;

  std::string ret;
  ret.reserve(n);

  for (size_t i = 0; i < n; i++) {
    ret.push_back(static_cast<char>(folded >> (i * 8)) & 0xFF);
  }

  return ret;
};
