// Copyright 2023, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_SWEEPER_H_
#define SPATIALJOINS_SWEEPER_H_

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#if __cplusplus >= 201402L
#include <shared_mutex>
#endif
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include "GeometryCache.h"
#include "GeometryStore.h"
#include "Stats.h"
#include "util/JobQueue.h"
#include "util/geo/Geo.h"
#include "util/geo/IntervalIdx.h"

#ifndef POSIX_FADV_SEQUENTIAL
#define POSIX_FADV_SEQUENTIAL 2
#endif

namespace sj {

// if available, use a shared_timed_mutex, if not, fall back to a normal mutex
#if __cplusplus >= 201402L
typedef std::shared_timed_mutex SharedMutex;
typedef std::shared_lock<SharedMutex> SharedLock;
#else
typedef std::mutex SharedMutex;
typedef std::unique_lock<SharedMutex> SharedLock;
#endif
typedef std::unique_lock<SharedMutex> ExclusiveLock;

// a position in the sweep events of the multi geometries merged in during the
// sweep, as (slot of the multi geometry, position of the event)
typedef std::pair<size_t, size_t> MultiEventPos;

struct MultiEventCmp {
  const std::vector<std::vector<BoxVal>>* events;

  bool operator()(const MultiEventPos& a, const MultiEventPos& b) const {
    return boxCmp(&(*events)[a.first][a.second],
                  &(*events)[b.first][b.second]) > 0;
  }
};

typedef std::priority_queue<MultiEventPos, std::vector<MultiEventPos>,
                            MultiEventCmp>
    MultiEventQueue;

class Sweeper {
 public:
  Sweeper(SweeperCfg cfg, const GeometryCache* geomCache)
      : _cfg(cfg),
        _obufpos(0),
        _geomCache(geomCache),
        _geomStore(&geomCache->getStore()),
        _jobs(100),
        _dontNeedFullDE9IM(!_cfg.computeDE9IM &&
                           _cfg.de9imFilter == util::geo::FANY) {
    _outBuffer = new unsigned char[BUFFER_S];
  };

  void log(const std::string& msg);

  RelStats sweep(const SweepEventList* eventsA,
                 const SweepEventList* eventsB = 0);

  size_t numElements() const { return _curSweepId / 2; }

  static std::string unfoldString(size_t folded);

 private:
  // metadata of the multi geometries, available after their multi-IN event
  size_t subSize(const std::string& gid) const {
    SharedLock lock(_multiMetaMtx);
    return _subSizes.at(gid);
  }

  bool isMulti(const std::string& gid) const {
    SharedLock lock(_multiMetaMtx);
    return _subSizes.find(gid) != _subSizes.end();
  }

  util::geo::I32Point multiRightPoint(const std::string& gid) const {
    SharedLock lock(_multiMetaMtx);
    auto i = _multiRightPoint.find(gid);
    if (i == _multiRightPoint.end()) return {};
    return i->second;
  }

  // fills the metadata of the multi geometry of multiIn, and returns the
  // sorted sweep events of its members
  std::vector<BoxVal> multiIn(const BoxVal& multiIn);

  Area areaFromSimpleArea(const SimpleArea* sa) const;
  Line lineFromSimpleLine(const SimpleLine* sl) const;

  double distCheck(const util::geo::I32Point& a, const Point* aMeta,
                   const Area* b, size_t t);
  double distCheck(const util::geo::I32Point& a, const Point* aMeta,
                   const Line* b, size_t t);
  double distCheck(const util::geo::I32Point& a,
                   const util::geo::LineSegment<int32_t>& b, size_t t);
  double distCheck(const util::geo::LineSegment<int32_t>& a,
                   const util::geo::LineSegment<int32_t>& b, size_t t);
  double distCheck(const util::geo::LineSegment<int32_t>& a, const Line* b,
                   size_t t);
  double distCheck(const util::geo::LineSegment<int32_t>& a, const Area* b,
                   size_t t);
  double distCheck(const Line* a, const Line* b, size_t t);
  double distCheck(const Area* a, const Area* b, size_t t);
  double distCheck(const Line* a, const Area* b, size_t t);

  double getMaxMultiDist(const std::string& idA, size_t aSub,
                         const util::geo::I32Point& leftAPoint,
                         const std::string& idB, size_t bSub,
                         const util::geo::I32Point& leftBPoint, size_t t);

  util::geo::DE9IMatrix DE9IMCheck(const util::geo::I32Point& a, const Area* b,
                                   size_t t) const;
  util::geo::DE9IMatrix DE9IMCheck(const util::geo::I32Point& a, const Line* b,
                                   size_t t) const;
  util::geo::DE9IMatrix DE9IMCheck(const util::geo::I32Point& a,
                                   const util::geo::LineSegment<int32_t>& b,
                                   size_t t) const;
  util::geo::DE9IMatrix DE9IMCheck(const util::geo::LineSegment<int32_t>& a,
                                   const util::geo::LineSegment<int32_t>& b,
                                   size_t t) const;
  util::geo::DE9IMatrix DE9IMCheck(const util::geo::LineSegment<int32_t>& a,
                                   const Line* b, size_t t) const;
  util::geo::DE9IMatrix DE9IMCheck(const util::geo::LineSegment<int32_t>& a,
                                   const Area* b, size_t t) const;
  util::geo::DE9IMatrix DE9IMCheck(const Line* a, const Line* b,
                                   size_t t) const;
  util::geo::DE9IMatrix DE9IMCheck(const Area* a, const Area* b,
                                   size_t t) const;
  util::geo::DE9IMatrix DE9IMCheck(const Line* a, const Area* b,
                                   size_t t) const;

  double getMaxScaleFactor(const util::geo::I32Box& geom) const;
  static std::pair<double, double> getMinMaxLocalScaleFactors(
      const util::geo::I32Box& boxA, const util::geo::I32Box& boxB,
      double distanceUpperBound);
  double getMaxScaleFactor(const util::geo::I32Point& geom) const;

  void diskAdd(const BoxVal& bv);

  void multiOut(size_t t, const std::string& gid);
  void clearMultis(bool force);

  void writeIntersect(size_t t, const std::string& a, size_t aSub,
                      const std::string& b, size_t bSub, bool expandB = true,
                      bool selfExp = false);
  void writeRel(size_t t, const std::string& a, const std::string& b,
                const std::string& pred);
  void writeContains(size_t t, const std::string& a, size_t aSub,
                     const std::string& b, size_t bSub, bool expandB = true);
  void writeCovers(size_t t, const std::string& a, size_t aSub,
                   const std::string& b, size_t bSub, bool expandB = true);
  void writeEquals(size_t t, const std::string& a, size_t aSub,
                   const std::string& b, size_t bSub, bool expandB = true,
                   bool selfExp = false);
  void writeDE9IM(size_t t, const std::string& a, size_t aSub,
                  const std::string& b, size_t bSub,
                  util::geo::DE9IMatrix de9im, bool expandB = true,
                  bool selfExp = false);
  void writeDist(size_t t, const std::string& a, size_t aSub,
                 const std::string& b, size_t bSub, double dist,
                 bool expandB = true, bool selfExp = false);
  void writeTouches(size_t t, const std::string& a, size_t aSub,
                    const std::string& b, size_t bSub, bool expandB = true);
  void writeNotTouches(size_t t, const std::string& a, size_t aSub,
                       const std::string& b, size_t bSub, bool expandB = true);

  void writeOverlaps(size_t t, const std::string& a, size_t aSub,
                     const std::string& b, size_t bSub, bool expandB = true);
  void writeNotOverlaps(size_t t, const std::string& a, size_t aSub,
                        const std::string& b, size_t bSub, bool expandB = true,
                        bool expandRefs = true);

  void writeCrosses(size_t t, const std::string& a, size_t aSub,
                    const std::string& b, size_t bSub, bool expandB = true);
  void writeNotCrosses(size_t t, const std::string& a, size_t aSub,
                       const std::string& b, size_t bSub, bool expandB = true);
  void writeCrossesOneWay(size_t t, const std::string& a, size_t aSub,
                          const std::string& b, size_t bSub,
                          bool expandB = true);

  void doCheck(JobVal cur, JobVal sv, size_t t);
  void doDistCheck(JobVal cur, JobVal sv, size_t t);
  void doDE9IMCheck(JobVal cur, JobVal sv, size_t t);
  void selfCheck(const std::string& a, size_t subId, GeomType type, size_t t);
  void processQueue(size_t t);

  bool notOverlaps(const std::string& a, const std::string& b);
  bool coversAll(const std::string& a, const std::string& b);
  bool notTouches(const std::string& a, const std::string& b);
  bool notCrosses(const std::string& a, const std::string& b);

  std::shared_ptr<sj::Point> getPoint(size_t id, GeomType gt, size_t t) const;
  static bool isPoint(GeomType gt) { return gt == POINT || gt == FOLDED_POINT; }

  static bool isArea(GeomType gt) {
    return gt == POLYGON || gt == SIMPLE_POLYGON || gt == FOLDED_BOX_POLYGON;
  }

  std::shared_ptr<sj::Area> getArea(const JobVal& j, size_t) const;

  std::shared_ptr<sj::SimpleLine> getSimpleLine(const JobVal& cur,
                                                size_t t) const;
  static bool isLine(GeomType gt) {
    return gt == LINE || gt == SIMPLE_LINE || gt == FOLDED_SIMPLE_LINE;
  }

  static bool isSimpleLine(GeomType gt) {
    return gt == SIMPLE_LINE || gt == FOLDED_SIMPLE_LINE;
  }

  static double meterDist(const util::geo::I32Point& p1,
                          const util::geo::I32Point& p2, double maxDist);

  static double euclideanDist(const util::geo::I32Point& p1,
                              const util::geo::I32Point& p2, double maxDist);

  static const util::geo::DistFunc<int32_t> EUCLIDEAN_DIST_FUNC;
  static const util::geo::DistFunc<int32_t> METER_DIST_FUNC;

  static double localSearchPadding(double euclideanDistanceUpperBound,
                                   double distanceUpperBound,
                                   const util::geo::I32Box& aBox,
                                   const util::geo::I32Box& bBox);

  static double noSearchPadding(double euclideanDistanceUpperBound,
                                double distanceUpperBound,
                                const util::geo::I32Box& aBox,
                                const util::geo::I32Box& bBox);

  void fillBatch(JobBatch* batch,
                 const util::geo::IntervalIdx<int32_t, SweepVal>* actives,
                 const BoxVal* cur) const;

  const SweeperCfg _cfg;
  size_t _curSweepId = 0;
  unsigned char* _outBuffer;
  ssize_t _obufpos;

  std::vector<size_t> _checks;
  std::vector<int32_t> _curX;
  std::vector<std::atomic<int32_t>> _atomicCurX;
  std::atomic<bool> _cancelled;

  std::vector<RelStats> _relStats;

  mutable std::vector<Stats> _stats;

  std::vector<std::unordered_map<std::string, std::map<std::string, double>>>
      _subDistance;
  std::vector<std::unordered_map<std::string,
                                 std::map<std::string, util::geo::DE9IMatrix>>>
      _subDE9IM;
  std::vector<
      std::unordered_map<std::string, std::map<std::string, std::set<size_t>>>>
      _subContains;
  std::vector<
      std::unordered_map<std::string, std::map<std::string, std::set<size_t>>>>
      _subCovered;
  std::vector<
      std::unordered_map<std::string, std::map<std::string, std::set<size_t>>>>
      _subEquals;
  std::vector<std::unordered_map<std::string, std::set<std::string>>>
      _subTouches;
  std::vector<std::unordered_map<std::string, std::set<std::string>>>
      _subNotTouches;
  std::vector<std::unordered_map<std::string, std::set<std::string>>>
      _subCrosses;
  std::vector<std::unordered_map<std::string, std::set<std::string>>>
      _subNotCrosses;
  std::vector<std::unordered_map<std::string, std::set<std::string>>>
      _subOverlaps;
  std::vector<std::unordered_map<std::string, std::set<std::string>>>
      _subNotOverlaps;

  // multi geometries whose OUT event has been processed
  std::queue<BoxVal> _finishedMultis;

  std::unordered_map<size_t, std::string> _openMultiIDs;

  std::unordered_map<std::string, size_t> _subSizes;
  std::unordered_map<std::string, util::geo::I32Point> _multiRightPoint;
  mutable SharedMutex _multiMetaMtx;

  const GeometryCache* _geomCache;
  const GeometryStore* _geomStore;

  util::JobQueue<JobBatch> _jobs;

  std::vector<std::mutex> _mutsEquals;
  std::vector<std::mutex> _mutsCovers;
  std::vector<std::mutex> _mutsContains;
  std::vector<std::mutex> _mutsTouches;
  std::vector<std::mutex> _mutsNotTouches;
  std::vector<std::mutex> _mutsCrosses;
  std::vector<std::mutex> _mutsNotCrosses;
  std::vector<std::mutex> _mutsOverlaps;
  std::vector<std::mutex> _mutsNotOverlaps;
  std::vector<std::mutex> _mutsDistance;
  std::vector<std::mutex> _mutsDE9IM;

  bool _dontNeedFullDE9IM;
};

}  // namespace sj

#endif
