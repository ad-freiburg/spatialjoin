// Copyright 2026, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_GEOMETRYCACHEMANAGER_H_
#define SPATIALJOINS_GEOMETRYCACHEMANAGER_H_

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
#include "SweepEventList.h"
#include "Stats.h"
#include "util/JobQueue.h"
#include "util/geo/Geo.h"
#include "util/geo/IntervalIdx.h"

#ifndef POSIX_FADV_SEQUENTIAL
#define POSIX_FADV_SEQUENTIAL 2
#endif

namespace sj {

inline std::string toString(const BoxVal& bv) {
  std::stringstream ret;

  ret << "(id=" << bv.id;
  ret << " loY=" << bv.loY;
  ret << " upY=" << bv.upY;
  ret << " val=" << bv.val;
  ret << " out=" << bv.out;
  ret << " type=" << (int)bv.type;
  ret << " point=" << util::geo::getWKT(bv.point);
  // ret << " b45=" << util::geo::getWKT(bv.b45);
  ret << " side=" << bv.side;
  ret << " large=" << bv.large;
  ret << ")";

  return ret.str();
}

struct WriteCand {
  std::string raw;
  std::string gid;
  BoxVal boxvalIn;
  BoxVal boxvalOut;
  size_t subid;
};

struct WriteBatch {
  std::vector<WriteCand> points;
  std::vector<WriteCand> foldedPoints;
  std::vector<WriteCand> simpleLines;
  std::vector<WriteCand> foldedSimpleLines;
  std::vector<WriteCand> lines;
  std::vector<WriteCand> simpleAreas;
  std::vector<WriteCand> foldedBoxAreas;
  std::vector<WriteCand> areas;
  std::vector<WriteCand> refs;

  size_t size() const {
    return points.size() + foldedSimpleLines.size() + foldedPoints.size() +
           simpleLines.size() + lines.size() + simpleAreas.size() +
           areas.size() + refs.size();
  }
};

inline bool operator==(const BoxVal& a, const BoxVal& b) {
  return a.id == b.id && a.loY == b.loY && a.upY == b.upY && a.type == b.type;
}

struct SweepVal {
  SweepVal(size_t id, GeomType type)
      : id(id), type(type), side(false), large(false) {}
  SweepVal(size_t id, GeomType type, util::geo::I32Box b45,
           util::geo::I32Point point, util::geo::I32Point point2, bool side,
           bool large)
      : id(id),
        type(type),
        b45(b45),
        point(point),
        point2(point2),
        side(side),
        large(large) {}
  SweepVal() : id(0), type(POLYGON) {}
  size_t id;
  GeomType type : 4;
  util::geo::I32Box b45;
  util::geo::I32Point point, point2;
  bool side;
  bool large;
};

struct JobVal {
  size_t id;
  GeomType type : 4;
  util::geo::I32Point point, point2;
  bool large;
  int32_t val;

  JobVal() : id(0), type(POLYGON) {}
  JobVal(const BoxVal& bv)
      : id(bv.id),
        type(bv.type),
        point(bv.point),
        point2(bv.val, bv.point.getY() == bv.loY ? bv.upY : bv.loY),
        large(bv.large),
        val(bv.val){};
  JobVal(const SweepVal& sv)
      : id(sv.id),
        type(sv.type),
        point(sv.point),
        point2(sv.point2),
        large(sv.large),
        val(0){};
};

inline bool operator==(const JobVal& a, const JobVal& b) {
  return a.id == b.id && a.type == b.type;
}

inline bool operator==(const SweepVal& a, const SweepVal& b) {
  return a.id == b.id && a.type == b.type;
}

inline bool operator<(const SweepVal& a, const SweepVal& b) {
  return a.id < b.id || (a.id == b.id && a.type < b.type);
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

struct SweeperCfg {
  size_t numThreads;
  size_t numCacheThreads;
  size_t geomCacheMaxSize;
  size_t geomCacheMaxNumElements;
  std::string sepIsect;
  std::string sepContains;
  std::string sepCovers;
  std::string sepTouches;
  std::string sepEquals;
  std::string sepOverlaps;
  std::string sepCrosses;
  bool useBoxIds;
  bool useArea;
  bool useOBB;
  bool useDiagBox;
  bool useFastSweepSkip;
  bool noGeometryChecks;
  double withinDist;
  bool euclideanDist;
  bool haversineApprox;
  bool computeDE9IM;
  util::geo::DE9IMFilter de9imFilter;
  bool forceTwoSided;
  std::function<void(size_t t, const char* a, size_t an, const char* b,
                     size_t bn, const char* pred, size_t predn)>
      writeRelCb;
  std::function<void(const std::string&)> logCb;
  std::function<void(const std::string&)> statsCb;
  std::function<void(size_t)> sweepProgressCb;
  std::function<void()> sweepCancellationCb;
};

static const size_t MAX_OUT_LINE_LENGTH = 1000;

static const size_t POINT_CACHE_MAX_ELEMENTS = 10000;
static const size_t SIMPLE_LINE_CACHE_MAX_ELEMENTS = 10000;

// only use large geom cache for extreme geometries
static const size_t GEOM_LARGENESS_THRESHOLD = 1024 * 1024 * 1024;

class GeometryCacheManager {
 public:
  GeometryCacheManager(SweeperCfg cfg, const std::string& cache)
      : GeometryCacheManager(cfg, cache, ".spatialjoin") {}
  GeometryCacheManager(SweeperCfg cfg, const std::string& cache,
                       const std::string& tmpPrefix)
      : _cfg(cfg),
        _events(cache, tmpPrefix, _cfg.numThreads),
        _obufpos(0),
        _pointCache({cfg.useOBB}, cfg.geomCacheMaxSize,
                    POINT_CACHE_MAX_ELEMENTS, cfg.numCacheThreads, cache,
                    tmpPrefix),
        _areaCache({cfg.useOBB}, cfg.geomCacheMaxSize,
                   cfg.geomCacheMaxNumElements, cfg.numCacheThreads, cache,
                   tmpPrefix),
        _simpleAreaCache({cfg.useOBB}, cfg.geomCacheMaxSize,
                         cfg.geomCacheMaxNumElements, cfg.numCacheThreads,
                         cache, tmpPrefix),
        _lineCache({cfg.useOBB}, cfg.geomCacheMaxSize,
                   cfg.geomCacheMaxNumElements, cfg.numCacheThreads, cache,
                   tmpPrefix),
        _simpleLineCache({cfg.useOBB}, cfg.geomCacheMaxSize,
                         SIMPLE_LINE_CACHE_MAX_ELEMENTS, cfg.numCacheThreads,
                         cache, tmpPrefix),
        _cache(cache),
        _numSides(1) {
    if (_cfg.forceTwoSided) _numSides = 2;
  };

  void log(const std::string& msg);

  util::geo::I32Box add(const util::geo::I32MultiPolygon& a,
                        const std::string& gid, bool side,
                        WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32MultiPolygon& a,
                        const std::string& gid, size_t, bool side,
                        WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32Polygon& a, const std::string& gid,
                        bool side, WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32Polygon& a, const std::string& gid,
                        size_t subId, bool side, WriteBatch& batch) const;

  util::geo::I32Box add(const util::geo::I32MultiLine& a,
                        const std::string& gid, size_t, bool side,
                        WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32MultiLine& a,
                        const std::string& gid, bool side,
                        WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32Line& a, const std::string& gid,
                        bool side, WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32Line& a, const std::string& gid,
                        size_t subid, bool side, WriteBatch& batch) const;

  util::geo::I32Box add(const util::geo::I32Point& a, const std::string& gid,
                        bool side, WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32Point& a, const std::string& gid,
                        size_t subid, bool side, WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32MultiPoint& a,
                        const std::string& gid, size_t, bool side,
                        WriteBatch& batch) const;
  util::geo::I32Box add(const util::geo::I32MultiPoint& a,
                        const std::string& gid, bool side,
                        WriteBatch& batch) const;

  void add(const std::string& a, const util::geo::I32Box& box,
           const std::string& gid, size_t subid, bool side,
           WriteBatch& batch) const;
  void add(const std::string& a, size_t parentSubId,
           const util::geo::I32Box& box, const std::string& gid, size_t subid,
           bool side, WriteBatch& batch) const;

  void addBatch(WriteBatch& cands);

  void flush();

  const SweepEventList& events() const { return _events; }

  size_t numReferences() const {
    size_t ret = 0;
    for (const auto& subs : _refs) {
      for (const auto& refd : subs.second) {
        ret += refd.second.size();
      }
    }
    return ret;
  }

  std::shared_ptr<sj::Line> getLine(size_t id, size_t tid) const {
    return _lineCache.get(id, tid);
  }
  std::shared_ptr<sj::SimpleLine> getSimpleLine(size_t id, size_t tid) const {
    return _simpleLineCache.get(id, tid);
  }

  std::shared_ptr<sj::Area> getArea(size_t id, size_t tid) const {
    return _areaCache.get(id, tid);
  }

  std::shared_ptr<sj::SimpleArea> getSimpleArea(size_t id, size_t tid) const {
    return _simpleAreaCache.get(id, tid);
  }

  std::shared_ptr<sj::Point> getPoint(size_t id, size_t tid) const {
    return _pointCache.get(id, tid);
  }

  std::pair<size_t, size_t> size() const;

  size_t numSides() const { return _numSides; }

  size_t subSize(const std::string& id) const {
    return _subSizes.at(id);
  }

  bool isMulti(const std::string& id) const {
    return _subSizes.find(id) != _subSizes.end();
  }

  size_t numMultis(bool side) const { return _multiIds[side].size(); }

  const std::string& multiId(bool side, size_t id) const {
    return _multiIds[side][id];
  }

  int32_t multiRightX(bool side, size_t id) const {
    return _multiRightX[side][id];
  }

  util::geo::I32Point multiRightPoint(const std::string& gid) const {
    auto i = _multiRightPoint.find(gid);
    if (i == _multiRightPoint.end()) return {};
    return i->second;
  }

  bool hasRefs() const { return _refs.size() != 0; }

  bool isRefed(const std::string& gid) const { return _refs.count(gid); }

  const std::unordered_map<std::string, size_t>* getRefs(
      const std::string& gid, size_t subId) const {
    auto i = _refs.find(gid);
    if (i == _refs.end()) return nullptr;
    auto j = i->second.find(subId);
    if (j == i->second.end()) return nullptr;
    return &j->second;
  }

  std::pair<std::string, size_t> selfCheck(size_t id) const {
    return _selfChecks[id];
  }

  void setFilterBox(const util::geo::I32Box& filterBox) {
    _filterBox = filterBox;
  }

  template <template <typename> class G, typename T>
  util::geo::I32Box getPaddedBoundingBox(const G<T>& geom) const {
    return getPaddedBoundingBox(geom, geom);
  }

  template <template <typename> class G1, template <typename> class G2,
            typename T>
  util::geo::I32Box getPaddedBoundingBox(const G1<T>& geom,
                                         const G2<T>& refGeom) const;
  static size_t foldString(const std::string& s);

  double DUPLICATE_REMOVAL_MIN_SIZE = 500;

 private:
  double getMaxMultiDist(const std::string& idA, size_t aSub,
                         const util::geo::I32Point& leftAPoint,
                         const std::string& idB, size_t bSub,
                         const util::geo::I32Point& leftBPoint, size_t t);

  void diskAdd(const BoxVal& bv);

  void multiAdd(const std::string& gid, bool side, int32_t xLeft,
                int32_t xRight, const util::geo::I32Point& pointRight);

  void duplicatesToReferences();

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
  int _file;
  SweepEventList _events;
  ssize_t _obufpos;

  std::vector<size_t> _checks;
  std::vector<int32_t> _curX;
  std::vector<std::atomic<int32_t>> _atomicCurX;
  std::atomic<bool> _cancelled;

  std::vector<RelStats> _relStats;

  mutable std::vector<Stats> _stats;

  GeometryCache<Point> _pointCache;
  GeometryCache<Area> _areaCache;
  GeometryCache<SimpleArea> _simpleAreaCache;
  GeometryCache<Line> _lineCache;
  GeometryCache<SimpleLine> _simpleLineCache;

  // these are written during the geometry add phase
  std::vector<std::string> _multiIds[2];
  std::vector<int32_t> _multiRightX[2];
  std::map<std::string, util::geo::I32Point> _multiRightPoint;
  std::vector<int32_t> _multiLeftX[2];
  std::map<std::string, size_t> _multiGidToId[2];
  std::map<std::string, size_t> _subSizes;

  std::string _cache;

  std::atomic<uint8_t> _numSides;

  mutable std::mutex _multiAddMtx;
  mutable std::mutex _sweepEventWriteMtx;
  mutable std::mutex _pointGeomCacheWriteMtx;
  mutable std::mutex _lineGeomCacheWriteMtx;
  mutable std::mutex _simpleLineGeomCacheWriteMtx;
  mutable std::mutex _areaGeomCacheWriteMtx;
  mutable std::mutex _simpleAreaGeomCacheWriteMtx;

  std::unordered_map<std::string, util::geo::I32Box> _selfCheckBounds;

  std::unordered_map<
      std::string,
      std::unordered_map<size_t, std::unordered_map<std::string, size_t>>>
      _refs;

  std::vector<std::pair<std::string, size_t>> _selfChecks;

  util::geo::I32Box _filterBox = {{std::numeric_limits<int32_t>::lowest(),
                                   std::numeric_limits<int32_t>::lowest()},
                                  {std::numeric_limits<int32_t>::max(),
                                   std::numeric_limits<int32_t>::max()}};
};

}  // namespace sj

#endif
