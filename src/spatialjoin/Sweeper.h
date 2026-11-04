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
#include "SweepEventList.h"
#include "util/JobQueue.h"
#include "util/geo/Geo.h"
#include "util/geo/IntervalIdx.h"

#ifndef POSIX_FADV_SEQUENTIAL
#define POSIX_FADV_SEQUENTIAL 2
#endif

namespace sj {

struct SweepVal {
  SweepVal(size_t id) : id(id), side(false), storeId(0) {}
  SweepVal(size_t id, util::geo::I32Box b45, util::geo::I32Point point,
           util::geo::I32Point point2, bool side, uint8_t storeId)
      : id(id),
        b45(b45),
        point(point),
        point2(point2),
        side(side),
        storeId(storeId) {}
  SweepVal() : id(typedId(0, POLYGON)), side(false), storeId(0) {}
  size_t id;  // typed ID, see typedId()
  util::geo::I32Box b45;
  util::geo::I32Point point, point2;
  // the join side, given by the SweepEventList the geometry came from
  bool side;
  // the store the geometry is stored in, see GeometryCache::addStore()
  uint8_t storeId;

  size_t getId() const { return untypedId(id); }
  GeomType getType() const { return idType(id); }
};

// where a geometry in a sweep comes from: its join side (given by the
// SweepEventList it came from), and the store it is stored in
struct GeomOrigin {
  bool side;
  uint8_t storeId;
};

struct JobVal {
  size_t id;  // typed ID, see typedId()
  util::geo::I32Point point, point2;
  int32_t val;
  bool side;
  uint8_t storeId;

  JobVal() : id(typedId(0, POLYGON)), val(0), side(false), storeId(0) {}
  JobVal(const BoxVal& bv, bool side)
      : id(bv.id),
        point(bv.point),
        point2(getOtherPoint(bv)),
        val(bv.val),
        side(side),
        storeId(bv.storeId){};
  JobVal(const SweepVal& sv)
      : id(sv.id),
        point(sv.point),
        point2(sv.point2),
        val(0),
        side(sv.side),
        storeId(sv.storeId){};

  size_t getId() const { return untypedId(id); }
  GeomType getType() const { return idType(id); }
  GeomOrigin origin() const { return {side, storeId}; }
};

inline bool operator==(const JobVal& a, const JobVal& b) {
  return a.id == b.id;
}

inline bool operator==(const SweepVal& a, const SweepVal& b) {
  return a.id == b.id;
}

inline bool operator<(const SweepVal& a, const SweepVal& b) {
  return a.id < b.id;
}

struct Job {
  JobVal boxVal, sweepVal;
  std::string multiOut;
};

inline bool operator==(const Job& a, const Job& b) {
  return a.boxVal == b.boxVal && a.sweepVal == b.sweepVal &&
         a.multiOut == b.multiOut;
}

typedef std::vector<Job> JobBatch;

// intersects, contains, covers, touches, crosses / overlaps
typedef std::tuple<bool, bool, bool, bool, bool> GeomCheckRes;

static const size_t MAX_OUT_LINE_LENGTH = 1000;

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
  Sweeper(SpatialJoinCfg cfg, const GeometryCache* geomCache)
      : _cfg(cfg),
        _geomCache(geomCache),
        _jobs(100),
        _dontNeedFullDE9IM(!_cfg.computeDE9IM &&
                           _cfg.de9imFilter == util::geo::FANY) {}

  void log(const std::string& msg);

  // with eventsB == 0, a self join over eventsA, otherwise a non-self join
  // between eventsA (the left side) and eventsB (the right side). Each event
  // references its geometry in the store given by BoxVal::storeId, so a single
  // event list may hold geometries from several stores, and both event lists
  // may hold geometries from the same store. The gids of the geometries of
  // each side have to be unique.
  RelStats sweep(const SweepEventList* eventsA,
                 const SweepEventList* eventsB = 0);

  size_t numElements() const { return _curSweepId / 2; }

  static std::string unfoldString(size_t folded);

 private:
  // metadata of the multi geometries, available after their multi-IN event
  size_t subSize(const std::string& gid, bool side) const {
    SharedLock lock(_multiMetaMtx);
    return _subSizes[side].at(gid);
  }

  bool isMulti(const std::string& gid, bool side) const {
    SharedLock lock(_multiMetaMtx);
    return _subSizes[side].find(gid) != _subSizes[side].end();
  }

  util::geo::I32Point multiRightPoint(const std::string& gid, bool side) const {
    SharedLock lock(_multiMetaMtx);
    auto i = _multiRightPoint[side].find(gid);
    if (i == _multiRightPoint[side].end()) return {};
    return i->second;
  }

  // whether the event lists of a or b hold references
  bool hasRefs(GeomOrigin a, GeomOrigin b) const {
    return _events[a.side]->hasRefs() || _events[b.side]->hasRefs();
  }

  // whether gid a on aSide and gid b on bSide denote the same geometry
  static bool sameGeom(const std::string& a, bool aSide, const std::string& b,
                       bool bSide) {
    return aSide == bSide && a == b;
  }

  // the store with id storeId
  const GeometryStore& store(uint8_t storeId) const {
    return _geomCache->getStore(storeId);
  }

  // fills the metadata of the multi geometry of multiIn on side, and returns
  // the sorted sweep events of its members
  std::vector<BoxVal> multiIn(const BoxVal& multiIn, bool side);

  // the side the partners of a geometry on side are on
  bool otherSide(bool side) const { return _numSides == 2 ? !side : side; }

  Area areaFromSimpleArea(const SimpleArea* sa) const;
  Line lineFromSimpleLine(const SimpleLine* sl) const;

  double distCheck(const util::geo::I32Point& a, const Point* aMeta,
                   const Area* b, GeomOrigin aOrig, GeomOrigin bOrig, size_t t);
  double distCheck(const util::geo::I32Point& a, const Point* aMeta,
                   const Line* b, GeomOrigin aOrig, GeomOrigin bOrig, size_t t);
  double distCheck(const util::geo::I32Point& a,
                   const util::geo::LineSegment<int32_t>& b, size_t t);
  double distCheck(const util::geo::LineSegment<int32_t>& a,
                   const util::geo::LineSegment<int32_t>& b, size_t t);
  double distCheck(const util::geo::LineSegment<int32_t>& a, const Line* b,
                   size_t t);
  double distCheck(const util::geo::LineSegment<int32_t>& a, const Area* b,
                   size_t t);
  double distCheck(const Line* a, const Line* b, GeomOrigin aOrig,
                   GeomOrigin bOrig, size_t t);
  double distCheck(const Area* a, const Area* b, GeomOrigin aOrig,
                   GeomOrigin bOrig, size_t t);
  double distCheck(const Line* a, const Area* b, GeomOrigin aOrig,
                   GeomOrigin bOrig, size_t t);

  double getMaxMultiDist(const std::string& idA, size_t aSub, GeomOrigin aOrig,
                         const util::geo::I32Point& leftAPoint,
                         const std::string& idB, size_t bSub, GeomOrigin bOrig,
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

  void multiOut(size_t t, const std::string& gid, GeomOrigin origin);
  void clearMultis(bool force);

  void writeIntersect(size_t t, const std::string& a, size_t aSub,
                      GeomOrigin aOrig, const std::string& b, size_t bSub,
                      GeomOrigin bOrig, bool expandB = true,
                      bool selfExp = false);
  void writeRel(size_t t, const std::string& a, bool aSide,
                const std::string& b, bool bSide, const std::string& pred);
  void writeContains(size_t t, const std::string& a, size_t aSub,
                     GeomOrigin aOrig, const std::string& b, size_t bSub,
                     GeomOrigin bOrig, bool expandB = true);
  void writeCovers(size_t t, const std::string& a, size_t aSub,
                   GeomOrigin aOrig, const std::string& b, size_t bSub,
                   GeomOrigin bOrig, bool expandB = true);
  void writeEquals(size_t t, const std::string& a, size_t aSub,
                   GeomOrigin aOrig, const std::string& b, size_t bSub,
                   GeomOrigin bOrig, bool expandB = true, bool selfExp = false);
  void writeDE9IM(size_t t, const std::string& a, size_t aSub, GeomOrigin aOrig,
                  const std::string& b, size_t bSub, GeomOrigin bOrig,
                  util::geo::DE9IMatrix de9im, bool expandB = true,
                  bool selfExp = false);
  void writeDist(size_t t, const std::string& a, size_t aSub, GeomOrigin aOrig,
                 const std::string& b, size_t bSub, GeomOrigin bOrig,
                 double dist, bool expandB = true, bool selfExp = false);
  void writeTouches(size_t t, const std::string& a, size_t aSub,
                    GeomOrigin aOrig, const std::string& b, size_t bSub,
                    GeomOrigin bOrig, bool expandB = true);
  void writeNotTouches(size_t t, const std::string& a, size_t aSub,
                       GeomOrigin aOrig, const std::string& b, size_t bSub,
                       GeomOrigin bOrig, bool expandB = true);

  void writeOverlaps(size_t t, const std::string& a, size_t aSub,
                     GeomOrigin aOrig, const std::string& b, size_t bSub,
                     GeomOrigin bOrig, bool expandB = true);
  void writeNotOverlaps(size_t t, const std::string& a, size_t aSub,
                        GeomOrigin aOrig, const std::string& b, size_t bSub,
                        GeomOrigin bOrig, bool expandB = true,
                        bool expandRefs = true);

  void writeCrosses(size_t t, const std::string& a, size_t aSub,
                    GeomOrigin aOrig, const std::string& b, size_t bSub,
                    GeomOrigin bOrig, bool expandB = true);
  void writeNotCrosses(size_t t, const std::string& a, size_t aSub,
                       GeomOrigin aOrig, const std::string& b, size_t bSub,
                       GeomOrigin bOrig, bool expandB = true);
  void writeCrossesOneWay(size_t t, const std::string& a, size_t aSub,
                          GeomOrigin aOrig, const std::string& b, size_t bSub,
                          GeomOrigin bOrig, bool expandB = true);

  void doCheck(JobVal cur, JobVal sv, size_t t);
  void doDistCheck(JobVal cur, JobVal sv, size_t t);
  void doDE9IMCheck(JobVal cur, JobVal sv, size_t t);
  // the self check of the referenced geometry of self check event cur
  void selfCheck(const JobVal& cur, size_t t);
  void selfCheck(const std::string& a, size_t subId, GeomOrigin origin,
                 GeomType type, size_t t);
  void processQueue(size_t t);

  bool notOverlaps(const std::string& a, bool aSide, const std::string& b);
  bool coversAll(const std::string& a, const std::string& b, bool bSide);
  bool notTouches(const std::string& a, bool aSide, const std::string& b);
  bool notCrosses(const std::string& a, bool aSide, const std::string& b);

  std::shared_ptr<sj::Point> getPoint(const JobVal& j, size_t t) const;
  static bool isPoint(GeomType gt) { return gt == POINT || gt == FOLDED_POINT; }

  static bool isArea(GeomType gt) {
    return gt == POLYGON || gt == SIMPLE_POLYGON || gt == FOLDED_BOX_POLYGON;
  }

  std::shared_ptr<sj::Area> getArea(const JobVal& j, size_t) const;

  std::shared_ptr<sj::SimpleLine> getSimpleLine(const JobVal& cur,
                                                size_t t) const;
  static bool isLine(GeomType gt) { return gt == LINE || isSimpleLine(gt); }

  static bool isSimpleLine(GeomType gt) { return sj::isSimpleLine(gt); }

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
                 const BoxVal* cur, bool side) const;

  const SpatialJoinCfg _cfg;
  size_t _curSweepId = 0;

  // 1 for a self join, 2 for a non-self join, set in sweep()
  size_t _numSides = 1;

  const SweepEventList* _events[2] = {0, 0};

  std::vector<size_t> _checks;
  std::vector<int32_t> _curX;
  std::vector<std::atomic<int32_t>> _atomicCurX;
  std::atomic<bool> _cancelled;

  std::vector<RelStats> _relStats;

  mutable std::vector<Stats> _stats;

  std::vector<std::unordered_map<std::string, std::map<std::string, double>>>
      _subDistance[2];
  std::vector<std::unordered_map<std::string,
                                 std::map<std::string, util::geo::DE9IMatrix>>>
      _subDE9IM[2];
  std::vector<
      std::unordered_map<std::string, std::map<std::string, std::set<size_t>>>>
      _subContains[2];
  std::vector<
      std::unordered_map<std::string, std::map<std::string, std::set<size_t>>>>
      _subCovered[2];
  std::vector<
      std::unordered_map<std::string, std::map<std::string, std::set<size_t>>>>
      _subEquals[2];
  std::vector<std::unordered_map<std::string, std::set<std::string>>>
      _subTouches[2];
  std::vector<std::unordered_map<std::string, std::set<std::string>>>
      _subNotTouches[2];
  std::vector<std::unordered_map<std::string, std::set<std::string>>>
      _subCrosses[2];
  std::vector<std::unordered_map<std::string, std::set<std::string>>>
      _subNotCrosses[2];
  std::vector<std::unordered_map<std::string, std::set<std::string>>>
      _subOverlaps[2];
  std::vector<std::unordered_map<std::string, std::set<std::string>>>
      _subNotOverlaps[2];

  // multi geometries whose OUT event has been processed, with their side
  std::queue<std::pair<BoxVal, bool>> _finishedMultis;

  // per side, the gids of the multi geometries whose multi-IN event was
  // processed, keyed by their store and their offset in its multi store
  std::map<std::pair<uint8_t, size_t>, std::string> _openMultiIDs[2];

  std::unordered_map<std::string, size_t> _subSizes[2];
  std::unordered_map<std::string, util::geo::I32Point> _multiRightPoint[2];
  mutable SharedMutex _multiMetaMtx;

  const GeometryCache* _geomCache;

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
