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
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include "GeometryCache.h"
#include "GeometryCacheManager.h"
#include "Stats.h"
#include "util/JobQueue.h"
#include "util/geo/Geo.h"
#include "util/geo/IntervalIdx.h"

#ifndef POSIX_FADV_SEQUENTIAL
#define POSIX_FADV_SEQUENTIAL 2
#endif

namespace sj {

class Sweeper {
 public:
  Sweeper(SweeperCfg cfg,
          const GeometryCacheManager* cacheManager)
      : _cfg(cfg),
        _obufpos(0),
        _cacheManager(cacheManager),
        _jobs(100),
        _dontNeedFullDE9IM(!_cfg.computeDE9IM &&
                           _cfg.de9imFilter == util::geo::FANY) {
    _outBuffer = new unsigned char[BUFFER_S];
  };

  void log(const std::string& msg);

  RelStats sweep(const SweepEventList& events);

  size_t numElements() const { return _curSweepId / 2; }

  static std::string unfoldString(size_t folded);

 private:
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
                      const std::string& b, size_t bSub);
  void writeRel(size_t t, const std::string& a, const std::string& b,
                const std::string& pred);
  void writeContains(size_t t, const std::string& a, size_t aSub,
                     const std::string& b, size_t bSub);
  void writeCovers(size_t t, const std::string& a, size_t aSub,
                   const std::string& b, size_t bSub);
  void writeEquals(size_t t, const std::string& a, size_t aSub,
                   const std::string& b, size_t bSub);
  void writeDE9IM(size_t t, const std::string& a, size_t aSub,
                  const std::string& b, size_t bSub,
                  util::geo::DE9IMatrix de9im);
  void writeDist(size_t t, const std::string& a, size_t aSub,
                 const std::string& b, size_t bSub, double dist);
  void writeTouches(size_t t, const std::string& a, size_t aSub,
                    const std::string& b, size_t bSub);
  void writeNotTouches(size_t t, const std::string& a, size_t aSub,
                       const std::string& b, size_t bSub);

  void writeOverlaps(size_t t, const std::string& a, size_t aSub,
                     const std::string& b, size_t bSub);
  void writeNotOverlaps(size_t t, const std::string& a, size_t aSub,
                        const std::string& b, size_t bSub);

  void writeCrosses(size_t t, const std::string& a, size_t aSub,
                    const std::string& b, size_t bSub);
  void writeNotCrosses(size_t t, const std::string& a, size_t aSub,
                       const std::string& b, size_t bSub);

  void doCheck(JobVal cur, JobVal sv, size_t t);
  void doDistCheck(JobVal cur, JobVal sv, size_t t);
  void doDE9IMCheck(JobVal cur, JobVal sv, size_t t);
  void selfCheck(const std::string& a, size_t subId, GeomType type, size_t t);
  void processQueue(size_t t);

  bool notOverlaps(const std::string& a, const std::string& b);
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

  static int boxCmp(const void* a, const void* b) {
    const auto& boxa = static_cast<const BoxVal*>(a);
    const auto& boxb = static_cast<const BoxVal*>(b);
    if (boxa->val < boxb->val) return -1;
    if (boxa->val > boxb->val) return 1;

    if (!boxa->out && boxb->out) return -1;
    if (boxa->out && !boxb->out) return 1;

    // everything before a polygon
    if (boxa->type != POLYGON && boxa->type != SIMPLE_POLYGON &&
        boxa->type != FOLDED_BOX_POLYGON &&
        (boxb->type == POLYGON || boxb->type == SIMPLE_POLYGON ||
         boxb->type == FOLDED_BOX_POLYGON))
      return -1;
    if ((boxa->type == POLYGON || boxa->type == SIMPLE_POLYGON ||
         boxa->type == FOLDED_BOX_POLYGON) &&
        boxb->type != POLYGON && boxb->type != SIMPLE_POLYGON &&
        boxb->type != FOLDED_BOX_POLYGON)
      return 1;

    // points before lines
    if ((boxa->type == POINT || boxa->type == FOLDED_POINT) &&
        (boxb->type == SIMPLE_LINE || boxb->type == LINE ||
         boxb->type == FOLDED_SIMPLE_LINE))
      return -1;
    if ((boxb->type == POINT || boxb->type == FOLDED_POINT) &&
        (boxa->type == SIMPLE_LINE || boxa->type == LINE ||
         boxa->type == FOLDED_SIMPLE_LINE))
      return 1;

    // smaller polygons before larger
    if ((boxa->type == POLYGON || boxa->type == SIMPLE_POLYGON ||
         boxa->type == FOLDED_BOX_POLYGON) &&
        (boxb->type == POLYGON || boxb->type == SIMPLE_POLYGON ||
         boxb->type == FOLDED_BOX_POLYGON) &&
        boxa->areaOrLen < boxb->areaOrLen)
      return -1;
    if ((boxa->type == POLYGON || boxa->type == SIMPLE_POLYGON ||
         boxa->type == FOLDED_BOX_POLYGON) &&
        (boxb->type == POLYGON || boxb->type == SIMPLE_POLYGON ||
         boxb->type == FOLDED_BOX_POLYGON) &&
        boxa->areaOrLen > boxb->areaOrLen)
      return 1;

    // shorter lines before longer
    if ((boxa->type == LINE || boxa->type == SIMPLE_LINE ||
         boxa->type == FOLDED_SIMPLE_LINE) &&
        (boxb->type == LINE || boxb->type == SIMPLE_LINE ||
         boxb->type == FOLDED_SIMPLE_LINE) &&
        boxa->areaOrLen < boxb->areaOrLen)
      return -1;
    if ((boxa->type == LINE || boxa->type == SIMPLE_LINE ||
         boxa->type == FOLDED_SIMPLE_LINE) &&
        (boxb->type == LINE || boxb->type == SIMPLE_LINE ||
         boxb->type == FOLDED_SIMPLE_LINE) &&
        boxa->areaOrLen > boxb->areaOrLen)
      return 1;

    return 0;
  }

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

  std::vector<std::map<std::string, std::map<std::string, double>>>
      _subDistance;
  std::vector<
      std::map<std::string, std::map<std::string, util::geo::DE9IMatrix>>>
      _subDE9IM;
  std::vector<std::map<std::string, std::map<std::string, std::set<size_t>>>>
      _subContains;
  std::vector<std::map<std::string, std::map<std::string, std::set<size_t>>>>
      _subCovered;
  std::vector<std::map<std::string, std::map<std::string, std::set<size_t>>>>
      _subEquals;
  std::vector<std::map<std::string, std::set<std::string>>> _subTouches;
  std::vector<std::map<std::string, std::set<std::string>>> _subNotTouches;
  std::vector<std::map<std::string, std::set<std::string>>> _subCrosses;
  std::vector<std::map<std::string, std::set<std::string>>> _subNotCrosses;
  std::vector<std::map<std::string, std::set<std::string>>> _subOverlaps;
  std::vector<std::map<std::string, std::set<std::string>>> _subNotOverlaps;

  std::set<size_t> _activeMultis[2];

  const GeometryCacheManager* _cacheManager;

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

  mutable std::mutex _multiAddMtx;
  mutable std::mutex _sweepEventWriteMtx;
  mutable std::mutex _pointGeomCacheWriteMtx;
  mutable std::mutex _lineGeomCacheWriteMtx;
  mutable std::mutex _simpleLineGeomCacheWriteMtx;
  mutable std::mutex _areaGeomCacheWriteMtx;
  mutable std::mutex _simpleAreaGeomCacheWriteMtx;

  bool _dontNeedFullDE9IM;
};

}  // namespace sj

#endif
