// Copyright 2024
// Author: Patrick Brosi

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <regex>
#include <set>
#include <string>
#include <vector>

#include "spatialjoin/BoxIds.h"
#include "spatialjoin/GeometryCache.h"
#include "spatialjoin/OutputWriter.h"
#include "spatialjoin/Sweeper.h"
#include "spatialjoin/WKTParse.h"
#include "spatialjoin/tests/SweepEventListTest.h"
#include "spatialjoin/tests/TypedGeometryStoreTest.h"
#include "util/Test.h"
#include "util/geo/Geo.h"
#include "util/log/Log.h"

using sj::GeometryCache;
using sj::ParseBatch;
using sj::Sweeper;
using sj::SweepEventList;

size_t NUM_THREADS = 1;

struct RunStats {
  size_t numReferences;
  size_t numInvalid;
};

// _____________________________________________________________________________
std::string fullRun(const std::vector<std::string>& files,
                    sj::SpatialJoinCfg cfg, RunStats* stats,
                    double dupRemovalMinSize = 0, bool strict = false,
                    bool sharedStore = false, bool singleList = false,
                    size_t eventBufSize = 10 * sizeof(sj::BoxVal),
                    bool reload = false) {
  // with reload, the geometry stores and the event lists are persisted to disk,
  // and the sweep runs on stores and event lists read back from these files
  auto storePrefix = [](size_t i) {
    return ".spatialjoin-store" + std::to_string(i);
  };
  auto eventsPrefix = [](size_t i) {
    return ".spatialjoin-list" + std::to_string(i);
  };

  {
    sj::OutputWriter outWriter(cfg.numThreads, "$", "$\n", ".resTmp", ".");
    cfg.writeRelCb = [&outWriter](size_t t, const char* a, size_t an,
                                  const char* b, size_t bn, const char* pred,
                                  size_t predn) {
      outWriter.writeRelCb(t, a, an, b, bn, pred, predn);
    };
    GeometryCache geomCache(cfg);

    std::vector<std::unique_ptr<sj::GeometryStore>> geomStores;
    std::vector<std::unique_ptr<SweepEventList>> sweepEvents;
    size_t storeId = 0;

    // very small buffer size 1 here for test purposes to force buffer overflows
    // during parsing
    const static size_t BUFF_SIZE = 100;
    char* buf = new char[BUFF_SIZE];

    size_t len = 0;
    size_t nextGid = 1;
    stats->numInvalid = 0;

    for (const auto& file : files) {
      if (geomStores.empty() || !sharedStore) {
        geomStores.emplace_back(new sj::GeometryStore(
            cfg, ".", reload ? storePrefix(geomStores.size()) : ".spatialjoin",
            reload));
        geomStores.back()->DUPLICATE_REMOVAL_MIN_SIZE = dupRemovalMinSize;
        storeId = geomCache.addStore(geomStores.back().get());
      }

      if (sweepEvents.empty() || !singleList) {
        sweepEvents.emplace_back(new SweepEventList(
            cfg, ".",
            reload ? eventsPrefix(sweepEvents.size()) : ".spatialjoin",
            eventBufSize, reload));
      }

      int f = open(file.c_str(), O_RDONLY);
      TEST(f >= 0);

      sj::WKTParser parser(geomStores.back().get(),
                           sweepEvents.back()->addCb(storeId), 1, strict);

      // the gids within a store and within an event list must be unique, so
      // continue the line numbering
      if (sharedStore || singleList) parser.setNextGid(nextGid);

      while ((len = read(f, buf, BUFF_SIZE)) > 0) {
        parser.parse(buf, len);
      }
      parser.done();

      nextGid = parser.getNextGid();
      stats->numInvalid += parser.getNumInvalid();

      close(f);
    }

    delete[] buf;

    for (auto& store : geomStores) store->flush();

    for (auto& events : sweepEvents) events->finalize(1);

    for (auto& events : sweepEvents) events->duplicatesToReferences(geomCache);

    if (reload) {
      // read back the persisted stores and event lists, keeping the store ids
      GeometryCache loadedCache(cfg);
      std::vector<std::unique_ptr<sj::GeometryStore>> loadedStores;
      std::vector<std::unique_ptr<SweepEventList>> loadedEvents;

      for (size_t i = 0; i < geomStores.size(); i++) {
        loadedStores.emplace_back(
            new sj::GeometryStore(cfg, ".", storePrefix(i)));
        TEST(loadedCache.addStore(loadedStores.back().get()), ==, i);
      }

      for (size_t i = 0; i < sweepEvents.size(); i++) {
        loadedEvents.emplace_back(
            new SweepEventList("./" + eventsPrefix(i) + "events"));
        TEST(loadedEvents.back()->numEvents(), ==,
             sweepEvents[i]->numEvents());
      }

      Sweeper sweeper(cfg, &loadedCache);
      sweeper.sweep(loadedEvents[0].get(),
                    loadedEvents.size() > 1 ? loadedEvents[1].get() : 0);

      stats->numReferences = 0;
      for (size_t i = 0; i < loadedEvents.size(); i++) {
        TEST(loadedEvents[i]->numReferences(), ==,
             sweepEvents[i]->numReferences());
        stats->numReferences += loadedEvents[i]->numReferences();
      }

      for (size_t i = 0; i < geomStores.size(); i++) {
        for (const auto& fName :
             {sj::TypedGeometryStore<sj::Point>::persistFName(".",
                                                              storePrefix(i)),
              sj::TypedGeometryStore<sj::Area>::persistFName(".",
                                                             storePrefix(i)),
              sj::TypedGeometryStore<sj::SimpleArea>::persistFName(
                  ".", storePrefix(i)),
              sj::TypedGeometryStore<sj::Line>::persistFName(".",
                                                             storePrefix(i)),
              sj::TypedGeometryStore<sj::SimpleLine>::persistFName(
                  ".", storePrefix(i)),
              sj::TypedGeometryStore<sj::MultiGeom>::persistFName(
                  ".", storePrefix(i))}) {
          TEST(unlink(fName.c_str()), ==, 0);
        }
      }
      for (size_t i = 0; i < sweepEvents.size(); i++) {
        TEST(unlink(("./" + eventsPrefix(i) + "events").c_str()), ==, 0);
        // only written if duplicates were replaced by references
        unlink(("./" + eventsPrefix(i) + "eventsrefs").c_str());
      }
    } else {
      Sweeper sweeper(cfg, &geomCache);
      sweeper.sweep(sweepEvents[0].get(),
                    sweepEvents.size() > 1 ? sweepEvents[1].get() : 0);

      stats->numReferences = 0;
      for (auto& events : sweepEvents)
        stats->numReferences += events->numReferences();
    }
  }

  std::string ret;

  std::stringstream ss;
  std::ifstream ifs(".resTmp");
  ss << ifs.rdbuf();

  ifs.close();
  unlink(".resTmp");

  return ss.str();
}

// _____________________________________________________________________________
std::string fullRun(const std::string& file, sj::SpatialJoinCfg cfg,
                    RunStats* stats, double dupRemovalMinSize = 0,
                    bool strict = false) {
  return fullRun(std::vector<std::string>{file}, cfg, stats, dupRemovalMinSize,
                 strict);
}

// _____________________________________________________________________________
std::vector<std::string> sortedRels(const std::string& res) {
  auto rels = util::split(res, '\n');
  std::sort(rels.begin(), rels.end());

  return rels;
}

// _____________________________________________________________________________
size_t countMatches(const std::string& res, const std::string& pattern) {
  std::regex re(pattern);
  return std::distance(std::sregex_iterator(res.begin(), res.end(), re),
                       std::sregex_iterator());
}

// _____________________________________________________________________________
void testNoContradictions(const std::string& res) {
  std::set<std::pair<std::string, std::string>> coveringContains, overlapping;

  for (auto r : sortedRels(res)) {
    r = util::trim(r, "$");
    auto parts = util::split(r, ' ');

    TEST(parts.size(), ==, 3);

    if (parts[1] == "covers") coveringContains.insert({parts[0], parts[2]});
    if (parts[1] == "contains") coveringContains.insert({parts[0], parts[2]});
    if (parts[1] == "overlaps") overlapping.insert({parts[0], parts[2]});
  }

  for (const auto& o : overlapping) {
    TEST(!(coveringContains.count(o) ||
           coveringContains.count({o.second, o.first})));
  }
}

// _____________________________________________________________________________
void writeMultiFlushDataset(const std::string& path) {
  // write a dataset triggering
  // https://github.com/ad-freiburg/spatialjoin/issues/28, see there for details
  std::ofstream ofs(path);
  ofs << "a\tPOLYGON((-10 -10, -1 -10, -1 -1, -10 -1, -10 -10))\n";
  ofs << "b\tMULTIPOINT((-5 -5), (-4 -4), (-3 -3))\n";

  for (size_t i = 0; i < 100000; i++) {
    ofs << "f" << i << "\tPOINT(" << (10.0 + i * 0.0008) << " 50)\n";
  }
}

// _____________________________________________________________________________
int main(int, char**) {
  sweepEventListTest();
  typedGeometryStoreTest();

  {
    // bounding box of the geometries added by a WKTParser
    using util::geo::extendBox;
    using util::geo::I32Box;
    using util::geo::I32Point;

    auto proj = [](double x, double y) {
      return sj::WKTParser::projFunc({x, y}, util::geo::CRSType::WGS84);
    };
    const I32Box lineBox(proj(1, 1), proj(2, 2));
    const I32Box pointBox(proj(3, 3), proj(3, 3));

    for (double withinDist : {-1.0, 1000.0}) {
      for (bool filter : {false, true}) {
        sj::SpatialJoinCfg cfg;
        cfg.numThreads = 2;
        cfg.withinDist = withinDist;
        sj::GeometryStore store(cfg, ".", ".spatialjointest", false);

        // with the filter, the point is not added
        if (filter) store.setFilterBox(lineBox);

        sj::WKTParser parser(
            &store,
            [](size_t, const std::string&, const sj::PreparedGeomMeta&) {}, 2,
            true);
        TEST(parser.getBoundingBox().isNull());
        parser.parseWKT("LINESTRING(1 1, 2 2)", 1);
        parser.parseWKT("LINESTRING(1 1, 2 a)", 2);  // invalid
        parser.parsePoint({3, 3}, 3);                 // added directly
        parser.done();

        TEST(parser.getNumInvalid(), ==, 1);

        // the union of the padded boxes of the added geometries
        I32Box expected = store.getPaddedBoundingBox(lineBox);
        if (!filter) {
          expected = extendBox(store.getPaddedBoundingBox(pointBox), expected);
        }

        TEST(parser.getBoundingBox() == expected);
        if (withinDist > 0) {
          // the padded box is larger than the unpadded one
          TEST(util::geo::contains(lineBox, parser.getBoundingBox()));
          TEST(!util::geo::contains(parser.getBoundingBox(), lineBox));
        } else {
          TEST(parser.getBoundingBox() ==
               (filter ? lineBox : extendBox(pointBox, lineBox)));
        }
        store.flush();
      }
    }
  }

  {
    using sj::boxids::GRID_H;
    using sj::boxids::GRID_W;
    using sj::boxids::inCellWithPadding;
    using sj::boxids::WORLD_H;
    using sj::boxids::WORLD_W;
    using util::geo::I32Box;

    int cx = std::ceil(1000 * GRID_W - WORLD_W / 2.0);
    int cy = std::ceil(2000 * GRID_H - WORLD_H / 2.0);

    I32Box box({cx + 100, cy + 100}, {cx + 200, cy + 200});
    TEST(inCellWithPadding(box, 0));
    TEST(inCellWithPadding(box, 50));
    TEST(inCellWithPadding(box, 98));
    TEST(!inCellWithPadding(box, 101));

    I32Box box2({cx - 10, cy + 100}, {cx + 10, cy + 200});
    TEST(!inCellWithPadding(box2, 0));

    int ux = std::floor(1001 * GRID_W - WORLD_W / 2.0);
    int uy = std::floor(2001 * GRID_H - WORLD_H / 2.0);
    I32Box box3({ux - 200, uy - 200}, {ux - 100, uy - 100});
    TEST(inCellWithPadding(box3, 98));
    TEST(!inCellWithPadding(box3, 101));
    I32Box box4({ux - 200, uy - 200}, {ux + 10, uy - 100});
    TEST(!inCellWithPadding(box4, 0));

    I32Box box5({cx + 4000, cy + 4000}, {cx + 4000, cy + 4000});
    TEST(inCellWithPadding(box5, 3000));
    TEST(!inCellWithPadding(box5, 4000));
    TEST(!inCellWithPadding(box5, GRID_W));
  }

  {
    using sj::boxids::boxIdIsectPadded;
    using sj::boxids::BoxIdList;
    using sj::boxids::GRID_W;
    using sj::boxids::NUM_GRID_CELLS;

    const int32_t N = NUM_GRID_CELLS;

    // cell (1000, 2000)
    int32_t c = 2000 * N + 1000 + 1;
    BoxIdList a{{1, 0}, {c, 0}};

    // padding by 1 cell
    TEST(boxIdIsectPadded(a, {{1, 0}, {c, 0}}, 100));
    TEST(boxIdIsectPadded(a, {{1, 0}, {c + 1, 0}}, 100));
    TEST(boxIdIsectPadded(a, {{1, 0}, {c - 1, 0}}, 100));
    TEST(boxIdIsectPadded(a, {{1, 0}, {c + N + 1, 0}}, 100));
    TEST(boxIdIsectPadded(a, {{1, 0}, {-(c - N - 1), 0}}, 100));
    TEST(!boxIdIsectPadded(a, {{1, 0}, {c + 2, 0}}, 100));
    TEST(!boxIdIsectPadded(a, {{1, 0}, {c - 2, 0}}, 100));
    TEST(!boxIdIsectPadded(a, {{1, 0}, {c + 2 * N, 0}}, 100));
    TEST(!boxIdIsectPadded(a, {{1, 0}, {-(c - 2 * N + 1), 0}}, 100));

    TEST(!boxIdIsectPadded(a, {{4, 0}, {c - 5, 3}}, 100));
    TEST(boxIdIsectPadded(a, {{5, 0}, {c - 5, 4}}, 100));
    TEST(!boxIdIsectPadded(a, {{4, 0}, {-(c + N + 2), 3}}, 100));
    TEST(boxIdIsectPadded(a, {{4, 0}, {-(c + N - 4), 3}}, 100));

    // padding by 2 cells
    TEST(boxIdIsectPadded(a, {{1, 0}, {c + 2, 0}}, GRID_W * 1.5));
    TEST(boxIdIsectPadded(a, {{1, 0}, {c - 2 * N - 2, 0}}, GRID_W * 1.5));
    TEST(!boxIdIsectPadded(a, {{1, 0}, {c + 3, 0}}, GRID_W * 1.5));
    TEST(!boxIdIsectPadded(a, {{1, 0}, {c + 3 * N, 0}}, GRID_W * 1.5));

    // undtermined
    TEST(boxIdIsectPadded(a, {{1, 0}, {c + 100, 0}}, GRID_W * 10));
    TEST(boxIdIsectPadded(a, {{0, 0}}, 100));
  }

  {
    // the metaCb of GeometryStore::addBatch() is called once for each
    // geometry, also for multi geometries
    sj::SpatialJoinCfg cfg;
    cfg.numThreads = cfg.numCacheBuckets = 1;
    sj::GeometryStore store(cfg, ".", ".spatialjoin", false);
    SweepEventList events(cfg, ".", ".spatialjoin");

    util::geo::I32Polygon poly1(
        {{0, 0}, {100, 0}, {100, 100}, {0, 100}, {0, 0}});
    util::geo::I32Polygon poly2(
        {{500, 300}, {700, 300}, {600, 900}, {500, 300}});
    util::geo::I32Polygon poly3(
        {{-50, -20}, {10, -20}, {10, 30}, {-40, 60}, {-50, -20}});

    sj::WriteBatch batch;
    store.add(util::geo::I32Point{5, 5}, "point", batch);
    store.add(util::geo::I32Line{{0, 0}, {50, 70}, {90, 10}}, "line", batch);
    store.add(poly1, "poly", batch);
    auto multiBox =
        store.add(util::geo::I32MultiPolygon{poly2, poly3}, "multi", batch);

    std::vector<sj::PreparedGeomMeta> metas;
    std::vector<std::string> gids;
    store.addBatch(batch, 3,
                   [&metas, &gids, &events](size_t t, const std::string& gid,
                                            const sj::PreparedGeomMeta& m) {
                     // the thread index passed to addBatch()
                     TEST(t, ==, 3);
                     metas.push_back(m);
                     gids.push_back(gid);
                     events.add(m, 0);
                   });

    TEST(metas.size(), ==, 4);

    // the gids given to add() are passed through
    auto sortedGids = gids;
    std::sort(sortedGids.begin(), sortedGids.end());
    TEST(sortedGids ==
         std::vector<std::string>({"line", "multi", "point", "poly"}));
    TEST(events.numObjects(), ==, metas.size());

    size_t numMultis = 0;
    for (size_t i = 0; i < metas.size(); i++) {
      const auto& m = metas[i];
      if (m.getType() != sj::MULTI) continue;
      numMultis++;
      TEST(gids[i], ==, "multi");
      TEST(m.bbox == multiBox);
      TEST(m.box45.isNull() == false);
      TEST(m.numAnchors, ==, poly2.getOuter().size() + poly3.getOuter().size());
    }
    TEST(numMultis, ==, 1);

    // the multi-IN and multi-OUT events added for the multi geometry are at
    // its left and right x
    events.finalize(1);
    auto reader = events.newReader();
    size_t numMultiEvents = 0;
    while (const auto* ev = reader.next()) {
      if (ev->type() != sj::MULTI) continue;
      numMultiEvents++;
      TEST(ev->val(), ==,
           ev->out() ? multiBox.getUpperRight().getX()
                     : multiBox.getLowerLeft().getX());
    }
    TEST(numMultiEvents, ==, 2);
  }

  auto mkCfg = [](bool useBoxIds, bool useDiagBox) {
    sj::SpatialJoinCfg cfg;
    cfg.numThreads = NUM_THREADS;
    cfg.numCacheBuckets = NUM_THREADS;
    cfg.geomCacheMaxSize = 1000;
    cfg.geomCacheMaxNumElements = 1000;
    cfg.useBoxIds = useBoxIds;
    cfg.useDiagBox = useDiagBox;
    return cfg;
  };

  sj::SpatialJoinCfg baseline = mkCfg(false, false);
  sj::SpatialJoinCfg all = mkCfg(true, true);
  sj::SpatialJoinCfg noBoxIds = mkCfg(false, true);
  sj::SpatialJoinCfg noDiagBox = mkCfg(true, false);

  std::vector<sj::SpatialJoinCfg> cfgs{baseline, all, noBoxIds, noDiagBox};

  for (auto cfg : cfgs) {
    {
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/freiburg", cfg, &stats);

      TEST(stats.numReferences, ==, 3);

      TEST(res.find("$freiburg1 covers freiburg2$") != std::string::npos);
      TEST(res.find("$freiburg1 covers freiburg2$") != std::string::npos);
      TEST(res.find("$freiburg2 covers freiburg1$") != std::string::npos);
      TEST(res.find("$freiburg1 equals freiburg2$") != std::string::npos);
      TEST(res.find("$freiburg2 equals freiburg1$") != std::string::npos);
      TEST(res.find("$freiburg1 contains freiburg2$") != std::string::npos);
      TEST(res.find("$freiburg2 contains freiburg1$") != std::string::npos);
      TEST(res.find("$freiburg1 intersects freiburg2$") != std::string::npos);
      TEST(res.find("$freiburg2 intersects freiburg1$") != std::string::npos);
      TEST(res.find("$freiburg2 overlaps freiburg1$") == std::string::npos);
      TEST(res.find("$freiburg2 touches freiburg1$") == std::string::npos);

      TEST(res.find("$freiburg1 covers grenzpart$") != std::string::npos);
      TEST(res.find("$freiburg1 contains grenzpart$") == std::string::npos);
      TEST(res.find("$grenzpart covers freiburg1$") == std::string::npos);

      TEST(res.find("$freiburg1 intersects grenz$") != std::string::npos);
      TEST(res.find("$freiburg2 intersects grenz$") != std::string::npos);
      TEST(res.find("$freiburg1 covers grenz$") == std::string::npos);
      TEST(res.find("$freiburg2 covers grenz$") == std::string::npos);
      TEST(res.find("$freiburg1 contains grenz$") == std::string::npos);
      TEST(res.find("$freiburg2 contains grenz$") == std::string::npos);

      TEST(res.find("$freiburg1 intersects a$") != std::string::npos);
      TEST(res.find("$freiburg2 intersects a$") != std::string::npos);
      TEST(res.find("$freiburg1 covers a$") == std::string::npos);
      TEST(res.find("$freiburg2 covers a$") == std::string::npos);
      TEST(res.find("$freiburg1 contains a$") == std::string::npos);
      TEST(res.find("$freiburg2 contains a$") == std::string::npos);

      TEST(res.find("$freiburg1 intersects b$") != std::string::npos);
      TEST(res.find("$freiburg2 intersects b$") != std::string::npos);
      TEST(res.find("$freiburg1 covers b$") == std::string::npos);
      TEST(res.find("$freiburg2 covers b$") == std::string::npos);
      TEST(res.find("$freiburg1 contains b$") == std::string::npos);
      TEST(res.find("$freiburg2 contains b$") == std::string::npos);

      TEST(res.find("$freiburg1 intersects c$") != std::string::npos);
      TEST(res.find("$freiburg2 intersects c$") != std::string::npos);
      TEST(res.find("$freiburg1 covers c$") == std::string::npos);
      TEST(res.find("$freiburg2 covers c$") == std::string::npos);
      TEST(res.find("$freiburg1 contains c$") == std::string::npos);
      TEST(res.find("$freiburg2 contains c$") == std::string::npos);

      TEST(res.find("$freiburg1 intersects d$") != std::string::npos);
      TEST(res.find("$freiburg2 intersects d$") != std::string::npos);
      TEST(res.find("$freiburg1 covers d$") == std::string::npos);
      TEST(res.find("$freiburg2 covers d$") == std::string::npos);
      TEST(res.find("$freiburg1 contains d$") == std::string::npos);
      TEST(res.find("$freiburg2 contains d$") == std::string::npos);

      TEST(res.find("$freiburg1 intersects grenzpunkt$") != std::string::npos);
      TEST(res.find("$freiburg2 intersects grenzpunkt$") != std::string::npos);
      TEST(res.find("$freiburg1 covers grenzpunkt$") != std::string::npos);
      TEST(res.find("$freiburg2 covers grenzpunkt$") != std::string::npos);
      TEST(res.find("$freiburg1 contains grenzpunkt$") == std::string::npos);
      TEST(res.find("$freiburg2 contains grenzpunkt$") == std::string::npos);

      TEST(res.find("$freiburg1 intersects grenzpunkt2$") != std::string::npos);
      TEST(res.find("$freiburg2 intersects grenzpunkt2$") != std::string::npos);
      TEST(res.find("$freiburg1 covers grenzpunkt2$") != std::string::npos);
      TEST(res.find("$freiburg2 covers grenzpunkt2$") != std::string::npos);
      TEST(res.find("$freiburg1 contains grenzpunkt2$") == std::string::npos);
      TEST(res.find("$freiburg2 contains grenzpunkt2$") == std::string::npos);

      TEST(res.find("$freiburg1 covers Umkirch$") == std::string::npos);

      TEST(res.find("$freiburg1 covers Sankt-Georgen$") != std::string::npos);
      TEST(res.find("$freiburg1 intersects Sankt-Georgen$") !=
           std::string::npos);
      TEST(res.find("$freiburg1 contains Sankt-Georgen$") != std::string::npos);

      TEST(res.find("$freiburg1 covers Haslach$") != std::string::npos);

      TEST(res.find("$freiburg1 covers Günterstal$") != std::string::npos);
      TEST(res.find("$freiburg1 intersects Günterstal$") != std::string::npos);
      TEST(res.find("$freiburg1 contains Günterstal$") != std::string::npos);

      TEST(res.find("$freiburg1 covers Kappel$") != std::string::npos);
      TEST(res.find("$freiburg1 intersects Kappel$") != std::string::npos);
      TEST(res.find("$freiburg1 contains Kappel$") != std::string::npos);

      TEST(res.find("$freiburg1 covers Littenweiler$") != std::string::npos);
      TEST(res.find("$freiburg1 intersects Littenweiler$") !=
           std::string::npos);
      TEST(res.find("$freiburg1 contains Littenweiler$") != std::string::npos);

      TEST(res.find("$freiburg1 covers Waldsee$") != std::string::npos);
      TEST(res.find("$freiburg1 intersects Waldsee$") != std::string::npos);
      TEST(res.find("$freiburg1 contains Waldsee$") != std::string::npos);

      TEST(res.find("$freiburg1 covers Wiehre$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Waltershofen$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Hochdorf$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Opfingen$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Betzenhausen$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Brühl$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Landwasser$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Lehen$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Mooswald$") != std::string::npos);

      TEST(res.find("$freiburg1 covers Munzingen$") != std::string::npos);
      TEST(res.find("$freiburg1 intersects Munzingen$") != std::string::npos);
      TEST(res.find("$freiburg1 contains Munzingen$") != std::string::npos);

      TEST(res.find("$freiburg1 covers Tiengen$") != std::string::npos);
      TEST(res.find("$freiburg1 intersects Tiengen$") != std::string::npos);
      TEST(res.find("$freiburg1 contains Tiengen$") != std::string::npos);

      TEST(res.find("$freiburg1 covers Mundenhof$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Rieselfeld$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Weingarten$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Altstadt$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Stühlinger$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Neuburg$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Herdern$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Zähringen$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Ebnet$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Oberau$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Sankt-Georgen-Süd$") !=
           std::string::npos);
      TEST(res.find("$freiburg1 covers Sankt-Georgen-Nord$") !=
           std::string::npos);
      TEST(res.find("$Sankt-Georgen covers Sankt-Georgen-Süd$") !=
           std::string::npos);
      TEST(res.find("$Sankt-Georgen covers Sankt-Georgen-Nord$") !=
           std::string::npos);
      TEST(res.find("$Sankt-Georgen-Süd touches Sankt-Georgen-Nord$") !=
           std::string::npos);
      TEST(res.find("$Sankt-Georgen-Nord touches Sankt-Georgen-Süd$") !=
           std::string::npos);
      TEST(res.find("$freiburg1 covers Haslach-Haid$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Haslach-Gartenstadt$") !=
           std::string::npos);
      TEST(res.find("$Haslach covers Haslach-Haid$") != std::string::npos);
      TEST(res.find("$Haslach covers Haslach-Gartenstadt$") !=
           std::string::npos);
      TEST(res.find("$Haslach covers Haslach-Schildacker$") !=
           std::string::npos);
      TEST(res.find("$Haslach covers Haslach-Egerten$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Haslach-Schildacker$") !=
           std::string::npos);
      TEST(res.find("$freiburg1 covers Haslach-Egerten$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Unterwiehre-Nord$") !=
           std::string::npos);
      TEST(res.find("$freiburg1 covers Unterwiehre-Süd$") != std::string::npos);
      TEST(res.find("$Wiehre covers Unterwiehre-Nord$") != std::string::npos);
      TEST(res.find("$Wiehre covers Unterwiehre-Süd$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Altstadt-Ring$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Alt-Stühlinger$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Betzenhausen-Bischofslinde$") !=
           std::string::npos);
      TEST(res.find("$freiburg1 covers Stühlinger-Eschholz$") !=
           std::string::npos);
      TEST(res.find("$freiburg1 covers Mooswald-Ost$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Brühl-Beurbarung$") !=
           std::string::npos);
      TEST(res.find("$freiburg1 covers Brühl-Industriegebiet$") !=
           std::string::npos);
      TEST(res.find("$freiburg1 covers Altstadt-Mitte$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Mittelwiehre$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Oberwiehre$") != std::string::npos);
      TEST(res.find("$Wiehre covers Mittelwiehre$") != std::string::npos);
      TEST(res.find("$Wiehre contains Mittelwiehre$") != std::string::npos);
      TEST(res.find("$Wiehre covers Oberwiehre$") != std::string::npos);
      TEST(res.find("$Wiehre contains Oberwiehre$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Alt-Betzenhausen$") !=
           std::string::npos);
      TEST(res.find("$freiburg1 covers Mooswald-West$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Brühl-Güterbahnhof$") !=
           std::string::npos);
      TEST(res.find("$freiburg1 covers Herdern-Süd$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Herdern-Nord$") != std::string::npos);
      TEST(res.find("$Herdern covers Herdern-Süd$") != std::string::npos);
      TEST(res.find("$Herdern covers Herdern-Nord$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Vauban$") != std::string::npos);

      TEST(res.find("$someway intersects someotherway$") != std::string::npos);
      TEST(res.find("$someway covers someotherway$") == std::string::npos);
      TEST(res.find("$someway contains someotherway$") == std::string::npos);

      TEST(res.find("$someway covers someway2$") != std::string::npos);
      TEST(res.find("$someway intersects someway2$") != std::string::npos);

      TEST(res.find("$someway2 covers someway$") != std::string::npos);
      TEST(res.find("$someway2 intersects someway$") != std::string::npos);

      TEST(res.find("$someotherway intersects someway$") != std::string::npos);
      TEST(res.find("$someotherway covers someway$") == std::string::npos);
      TEST(res.find("$someotherway contains someway$") == std::string::npos);
    }

    {
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/brandenburg_test", cfg, &stats);
      TEST(stats.numReferences, ==, 0);
      TEST(res.find("$Brandenburg-Point intersects Brandenburg-Way$") !=
           std::string::npos);
    }

    {
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/brandenburg", cfg, &stats);
      // TODO: duplicates in multi geometries are not yet removed
      // TEST(stats.numReferences, ==, 7);
      TEST(res.find("$Brandenburg covers Brandenburg2$") != std::string::npos);
      TEST(res.find("$Brandenburg intersects Brandenburg-Way$") !=
           std::string::npos);
      TEST(res.find("$Brandenburg equals Brandenburg2$") != std::string::npos);
      TEST(res.find("$Brandenburg2 equals Brandenburg$") != std::string::npos);
      TEST(res.find("$Brandenburg intersects Grenzpart$") != std::string::npos);
      TEST(res.find("$Brandenburg covers Grenzpart$") != std::string::npos);
      TEST(res.find("$Brandenburg contains Grenzpart$") == std::string::npos);

      TEST(res.find("$Brandenburg intersects Berlin$") != std::string::npos);
      TEST(res.find("$Brandenburg touches Berlin$") != std::string::npos);
      TEST(res.find("$Brandenburg overlaps Berlin$") == std::string::npos);
      TEST(res.find("$Berlin touches Brandenburg$") != std::string::npos);
      TEST(res.find("$Berlin overlaps Brandenburg$") == std::string::npos);
      TEST(res.find("$Brandenburg contains Berlin$") == std::string::npos);
      TEST(res.find("$Brandenburg covers Berlin$") == std::string::npos);

      TEST(res.find("$Berlin intersects Grenzpart$") != std::string::npos);
      TEST(res.find("$Berlin covers Grenzpart$") != std::string::npos);
      TEST(res.find("$Berlin contains Grenzpart$") == std::string::npos);

      TEST(res.find("$Haus overlaps Brandenburg$") != std::string::npos);
      TEST(res.find("$Haus-Way intersects Brandenburg$") != std::string::npos);
      TEST(res.find("$Haus intersects Brandenburg-Way$") != std::string::npos);
      TEST(res.find("$Haus-Way intersects Brandenburg-Way$") !=
           std::string::npos);

      TEST(res.find("$Brandenburg-Point intersects Brandenburg-Way$") !=
           std::string::npos);
      TEST(res.find("$Brandenburg-Point intersects Brandenburg$") !=
           std::string::npos);
    }

    {
      RunStats stats;
      auto res = fullRun(std::vector<std::string>{TEST_DATASET_DIR
                                                  "/brandenburg_nonself-side1",
                                                  TEST_DATASET_DIR
                                                  "/brandenburg_nonself-side2"},
                         cfg, &stats);

      // a non-self join on a single, shared store gives the same result
      RunStats statsShared;
      auto resShared = fullRun(
          std::vector<std::string>{
              TEST_DATASET_DIR "/brandenburg_nonself-side1",
              TEST_DATASET_DIR "/brandenburg_nonself-side2"},
          cfg, &statsShared, 0, false, true);
      TEST(sortedRels(resShared) == sortedRels(res));

      // TODO: duplicates in multi geometries are not yet removed
      // TEST(stats.numReferences, ==, 7);
      TEST(res.find("$Brandenburg covers Brandenburg2$") == std::string::npos);
      TEST(res.find("$Brandenburg intersects Brandenburg-Way$") ==
           std::string::npos);
      TEST(res.find("$Brandenburg equals Brandenburg2$") == std::string::npos);
      TEST(res.find("$Brandenburg2 equals Brandenburg$") == std::string::npos);
      TEST(res.find("$Brandenburg intersects Grenzpart$") == std::string::npos);
      TEST(res.find("$Grenzpart intersects Brandenburg$") != std::string::npos);
      TEST(res.find("$Brandenburg covers Grenzpart$") == std::string::npos);
      TEST(res.find("$Brandenburg contains Grenzpart$") == std::string::npos);

      TEST(res.find("$Brandenburg intersects Berlin$") == std::string::npos);
      TEST(res.find("$Berlin intersects Brandenburg$") != std::string::npos);
      TEST(res.find("$Brandenburg touches Berlin$") == std::string::npos);
      TEST(res.find("$Berlin touches Brandenburg$") != std::string::npos);
      TEST(res.find("$Brandenburg overlaps Berlin$") == std::string::npos);
      TEST(res.find("$Berlin touches Brandenburg$") != std::string::npos);
      TEST(res.find("$Berlin overlaps Brandenburg$") == std::string::npos);
      TEST(res.find("$Brandenburg contains Berlin$") == std::string::npos);
      TEST(res.find("$Brandenburg covers Berlin$") == std::string::npos);

      TEST(res.find("$Berlin intersects Grenzpart$") == std::string::npos);
      TEST(res.find("$Berlin covers Grenzpart$") == std::string::npos);
      TEST(res.find("$Berlin contains Grenzpart$") == std::string::npos);

      TEST(res.find("$Haus overlaps Brandenburg$") != std::string::npos);
      TEST(res.find("$Haus-Way intersects Brandenburg$") != std::string::npos);
      TEST(res.find("$Haus intersects Brandenburg-Way$") != std::string::npos);
      TEST(res.find("$Haus-Way intersects Brandenburg-Way$") !=
           std::string::npos);

      TEST(res.find("$Brandenburg-Point intersects Brandenburg-Way$") !=
           std::string::npos);
      TEST(res.find("$Brandenburg-Point intersects Brandenburg$") !=
           std::string::npos);
    }

    {
      RunStats stats;
      for (const auto& file :
           {TEST_DATASET_DIR "/brandenburg", TEST_DATASET_DIR "/freiburg"}) {
        auto onDisk = sortedRels(fullRun(file, cfg, &stats));
        auto inMemory =
            sortedRels(fullRun(std::vector<std::string>{file}, cfg, &stats, 0,
                               false, false, false, sj::DEFAULT_BUFFER_S));
        TEST(onDisk.size() > 0);
        TEST(inMemory == onDisk);
      }
    }

    {
      // a sweep over geometry stores and event lists read back from disk gives
      // the same result, in memory and on disk, with and without duplicates
      // replaced by references
      for (const auto& files : std::vector<std::vector<std::string>>{
               {TEST_DATASET_DIR "/brandenburg"},
               {TEST_DATASET_DIR "/duplicate-references"},
               {TEST_DATASET_DIR "/multitests"},
               {TEST_DATASET_DIR "/freiburg"},
               {TEST_DATASET_DIR "/brandenburg_nonself-side1",
                TEST_DATASET_DIR "/brandenburg_nonself-side2"}}) {
        for (size_t bufSize : {10 * sizeof(sj::BoxVal), sj::DEFAULT_BUFFER_S}) {
          RunStats stats, statsReloaded;
          auto res = sortedRels(fullRun(files, cfg, &stats, 0, false, false,
                                        false, bufSize));
          auto reloaded = sortedRels(fullRun(files, cfg, &statsReloaded, 0,
                                             false, false, false, bufSize,
                                             true));
          TEST(res.size() > 0);
          TEST(reloaded == res);
          TEST(statsReloaded.numReferences, ==, stats.numReferences);
        }
      }
    }

    {
      // a self join over a single event list holding the geometries of two
      // stores gives the same result as if they were in the same store
      RunStats stats;
      std::vector<std::string> files{
          TEST_DATASET_DIR "/brandenburg_nonself-side1",
          TEST_DATASET_DIR "/brandenburg_nonself-side2"};
      auto shared =
          sortedRels(fullRun(files, cfg, &stats, 0, false, true, true));
      auto separate =
          sortedRels(fullRun(files, cfg, &stats, 0, false, false, true));
      TEST(shared.size() > 0);
      TEST(separate == shared);
      TEST(std::find(separate.begin(), separate.end(),
                     "$Brandenburg covers Brandenburg2$") != separate.end());
    }

    {
      RunStats stats;
      auto self =
          sortedRels(fullRun(TEST_DATASET_DIR "/brandenburg", cfg, &stats));
      auto nonSelf = sortedRels(
          fullRun(std::vector<std::string>{TEST_DATASET_DIR "/brandenburg",
                                           TEST_DATASET_DIR "/brandenburg"},
                  cfg, &stats));

      TEST(std::includes(nonSelf.begin(), nonSelf.end(), self.begin(),
                         self.end()));
      TEST(std::find(nonSelf.begin(), nonSelf.end(),
                     "$Brandenburg equals Brandenburg$") != nonSelf.end());
      TEST(std::find(nonSelf.begin(), nonSelf.end(),
                     "$Brandenburg covers Brandenburg$") != nonSelf.end());
      TEST(std::find(self.begin(), self.end(),
                     "$Brandenburg equals Brandenburg$") == self.end());
      TEST(std::find(nonSelf.begin(), nonSelf.end(),
                     "$Brandenburg-Point equals Brandenburg-Point$") !=
           nonSelf.end());
    }

    {
      // the members of multi geometries inside a collection must not share
      // sub IDs with the following collection elements
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/collectionsubids", cfg, &stats);

      TEST(res.find("$colp equals mp$") != std::string::npos);
      TEST(res.find("$mp equals colp$") != std::string::npos);
      TEST(res.find("$mp covers colp$") != std::string::npos);
      TEST(res.find("$coll equals ml$") != std::string::npos);
      TEST(res.find("$ml equals coll$") != std::string::npos);
      TEST(res.find("$ml covers coll$") != std::string::npos);
    }

    {
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/collectiontests", cfg, &stats);
      // without box IDs, a polygon is converted into a box polygon
      if (cfg.useBoxIds) {
        // TODO: duplicates in multi geometries are not yet removed
        // TEST(stats.numReferences, ==, 11);
      } else {
        // TODO: duplicates in multi geometries are not yet removed
        // TEST(stats.numReferences, ==, 10);
      }

      TEST(res.find("$28 covers 27$") != std::string::npos);

      TEST(res.find("$27 overlaps 28$") == std::string::npos);
      TEST(res.find("$28 overlaps 27$") == std::string::npos);

      TEST(res.find("$17 touches 18$") != std::string::npos);
      TEST(res.find("$19 touches 18$") != std::string::npos);
      TEST(res.find("$17 touches 16$") != std::string::npos);

      TEST(res.find("$1 equals 2$") != std::string::npos);
      TEST(res.find("$1 overlaps 2$") == std::string::npos);
      TEST(res.find("$2 equals 1$") != std::string::npos);
      TEST(res.find("$3 equals 1$") == std::string::npos);
      TEST(res.find("$3 equals 2$") == std::string::npos);
      TEST(res.find("$3 equals 4$") != std::string::npos);
      TEST(res.find("$3 equals 5$") != std::string::npos);
      TEST(res.find("$4 equals 3$") != std::string::npos);
      TEST(res.find("$5 equals 3$") != std::string::npos);
      TEST(res.find("$5 equals 4$") != std::string::npos);
      TEST(res.find("$4 equals 5$") != std::string::npos);
      TEST(res.find("$4 overlaps 5$") == std::string::npos);
      TEST(res.find("$1 touches 6$") != std::string::npos);
      TEST(res.find("$6 touches 1$") != std::string::npos);
      TEST(res.find("$2 touches 6$") != std::string::npos);
      TEST(res.find("$6 touches 2$") != std::string::npos);
      TEST(res.find("$3 touches 6$") != std::string::npos);
      TEST(res.find("$6 touches 3$") != std::string::npos);
      TEST(res.find("$4 touches 6$") != std::string::npos);
      TEST(res.find("$6 touches 4$") != std::string::npos);
      TEST(res.find("$6 touches 7$") == std::string::npos);
      TEST(res.find("$7 touches 6$") == std::string::npos);

      TEST(res.find("$8 touches 9$") != std::string::npos);
      TEST(res.find("$9 touches 8$") != std::string::npos);

      TEST(res.find("$8 touches 11$") == std::string::npos);

      TEST(res.find("$9 touches 10$") == std::string::npos);
      TEST(res.find("$8 touches 10$") == std::string::npos);
      TEST(res.find("$12 touches 10$") != std::string::npos);
      TEST(res.find("$13 equals 10$") != std::string::npos);
      TEST(res.find("$13 equals 53$") != std::string::npos);
      TEST(res.find("$10 equals 53$") != std::string::npos);

      TEST(res.find("$28 equals 54$") != std::string::npos);

      TEST(res.find("$6 equals 14$") != std::string::npos);
      TEST(res.find("$15 touches 16$") != std::string::npos);
      TEST(res.find("$17 touches 6$") == std::string::npos);
      TEST(res.find("$17 intersects 16$") != std::string::npos);
      TEST(res.find("$17 intersects 18$") != std::string::npos);

      TEST(res.find("$8 overlaps 20$") != std::string::npos);
      TEST(res.find("$20 overlaps 8$") != std::string::npos);

      TEST(res.find("$17 crosses 21$") != std::string::npos);
      TEST(res.find("$21 crosses 17$") != std::string::npos);

      TEST(res.find("$17 crosses 22$") == std::string::npos);
      TEST(res.find("$22 crosses 17$") == std::string::npos);

      TEST(res.find("$17 overlaps 22$") == std::string::npos);
      TEST(res.find("$22 overlaps 17$") == std::string::npos);

      TEST(res.find("$24 covers 25$") != std::string::npos);

      TEST(res.find("$25 overlaps 24$") == std::string::npos);
      TEST(res.find("$24 overlaps 25$") == std::string::npos);

      TEST(res.find("$23 overlaps 24$") == std::string::npos);
      TEST(res.find("$24 overlaps 23$") == std::string::npos);

      TEST(res.find("$26 overlaps 24$") != std::string::npos);
      TEST(res.find("$24 overlaps 26$") != std::string::npos);

      TEST(res.find("$26 overlaps 27$") != std::string::npos);
      TEST(res.find("$27 overlaps 26$") != std::string::npos);

      TEST(res.find("$26 overlaps 28$") != std::string::npos);
      TEST(res.find("$28 overlaps 26$") != std::string::npos);

      TEST(res.find("8 overlaps 29$") == std::string::npos);
      TEST(res.find("8 overlaps 29$") == std::string::npos);
      TEST(res.find("30 covers 8$") != std::string::npos);
      TEST(res.find("8 overlaps 30$") == std::string::npos);
      TEST(res.find("29 overlaps 30$") == std::string::npos);
      TEST(res.find("30 overlaps 29") == std::string::npos);

      TEST(res.find("31 overlaps 30$") != std::string::npos);
      TEST(res.find("30 overlaps 31") != std::string::npos);
    }

    {
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/multitests", cfg, &stats);

      // without box IDs, a polygon is converted into a box polygon
      if (cfg.useBoxIds) {
        // TODO: duplicates in multi geometries are not yet removed
        // TEST(stats.numReferences, ==, 7);
      } else {
        // TODO: duplicates in multi geometries are not yet removed
        // TEST(stats.numReferences, ==, 6);
      }

      TEST(res.find("$28 covers 27$") != std::string::npos);

      TEST(res.find("$27 overlaps 28$") == std::string::npos);
      TEST(res.find("$28 overlaps 27$") == std::string::npos);

      TEST(res.find("$17 touches 18$") != std::string::npos);
      TEST(res.find("$19 touches 18$") != std::string::npos);
      TEST(res.find("$17 touches 16$") != std::string::npos);

      TEST(res.find("$1 equals 2$") != std::string::npos);
      TEST(res.find("$1 overlaps 2$") == std::string::npos);
      TEST(res.find("$2 equals 1$") != std::string::npos);
      TEST(res.find("$3 equals 1$") == std::string::npos);
      TEST(res.find("$3 equals 2$") == std::string::npos);
      TEST(res.find("$3 equals 4$") != std::string::npos);
      TEST(res.find("$3 equals 5$") != std::string::npos);
      TEST(res.find("$4 equals 3$") != std::string::npos);
      TEST(res.find("$5 equals 3$") != std::string::npos);
      TEST(res.find("$5 equals 4$") != std::string::npos);
      TEST(res.find("$4 equals 5$") != std::string::npos);
      TEST(res.find("$4 overlaps 5$") == std::string::npos);
      TEST(res.find("$1 touches 6$") != std::string::npos);
      TEST(res.find("$6 touches 1$") != std::string::npos);
      TEST(res.find("$2 touches 6$") != std::string::npos);
      TEST(res.find("$6 touches 2$") != std::string::npos);
      TEST(res.find("$3 touches 6$") != std::string::npos);
      TEST(res.find("$6 touches 3$") != std::string::npos);
      TEST(res.find("$4 touches 6$") != std::string::npos);
      TEST(res.find("$6 touches 4$") != std::string::npos);
      TEST(res.find("$6 touches 7$") == std::string::npos);
      TEST(res.find("$7 touches 6$") == std::string::npos);

      TEST(res.find("$8 touches 9$") != std::string::npos);
      TEST(res.find("$9 touches 8$") != std::string::npos);

      TEST(res.find("$8 touches 11$") == std::string::npos);

      TEST(res.find("$9 touches 10$") == std::string::npos);
      TEST(res.find("$8 touches 10$") == std::string::npos);
      TEST(res.find("$12 touches 10$") != std::string::npos);
      TEST(res.find("$13 equals 10$") != std::string::npos);

      TEST(res.find("$6 equals 14$") != std::string::npos);
      TEST(res.find("$15 touches 16$") != std::string::npos);
      TEST(res.find("$17 touches 6$") == std::string::npos);
      TEST(res.find("$17 intersects 16$") != std::string::npos);
      TEST(res.find("$17 intersects 18$") != std::string::npos);

      TEST(res.find("$8 overlaps 20$") != std::string::npos);
      TEST(res.find("$20 overlaps 8$") != std::string::npos);

      TEST(res.find("$17 crosses 21$") != std::string::npos);
      TEST(res.find("$21 crosses 17$") != std::string::npos);

      TEST(res.find("$17 crosses 22$") == std::string::npos);
      TEST(res.find("$22 crosses 17$") == std::string::npos);

      TEST(res.find("$17 overlaps 22$") == std::string::npos);
      TEST(res.find("$22 overlaps 17$") == std::string::npos);

      TEST(res.find("$24 covers 25$") != std::string::npos);

      TEST(res.find("$25 overlaps 24$") == std::string::npos);
      TEST(res.find("$24 overlaps 25$") == std::string::npos);

      TEST(res.find("$23 overlaps 24$") == std::string::npos);
      TEST(res.find("$24 overlaps 23$") == std::string::npos);

      TEST(res.find("$26 overlaps 24$") != std::string::npos);
      TEST(res.find("$24 overlaps 26$") != std::string::npos);

      TEST(res.find("$26 overlaps 27$") != std::string::npos);
      TEST(res.find("$27 overlaps 26$") != std::string::npos);

      TEST(res.find("$26 overlaps 28$") != std::string::npos);
      TEST(res.find("$28 overlaps 26$") != std::string::npos);

      TEST(res.find("8 overlaps 29$") == std::string::npos);
      TEST(res.find("8 overlaps 29$") == std::string::npos);
      TEST(res.find("30 covers 8$") != std::string::npos);
      TEST(res.find("8 overlaps 30$") == std::string::npos);
      TEST(res.find("29 overlaps 30$") == std::string::npos);
      TEST(res.find("30 overlaps 29") == std::string::npos);

      TEST(res.find("31 overlaps 30$") != std::string::npos);
      TEST(res.find("30 overlaps 31") != std::string::npos);
    }

    {
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/coverfail", cfg, &stats);
      TEST(stats.numReferences, ==, 0);
      TEST(res.find("$1 intersects 2$") != std::string::npos);
      TEST(res.find("$2 intersects 1$") != std::string::npos);
      TEST(res.find("$1 contains 2$") == std::string::npos);
      TEST(res.find("$2 contains 1$") == std::string::npos);
      TEST(res.find("$1 covers 2$") == std::string::npos);
      TEST(res.find("$2 covers 1$") == std::string::npos);
      TEST(res.find("$1 touches 2$") != std::string::npos);
      TEST(res.find("$2 touches 1$") != std::string::npos);
    }

    {
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/touchfail", cfg, &stats);
      TEST(stats.numReferences, ==, 0);
      TEST(res.find("$1 intersects 2$") != std::string::npos);
      TEST(res.find("$2 intersects 1$") != std::string::npos);
      TEST(res.find("$1 overlaps 2$") != std::string::npos);
      TEST(res.find("$2 overlaps 1$") != std::string::npos);
      TEST(res.find("$1 touches 2$") == std::string::npos);
      TEST(res.find("$2 touches 1$") == std::string::npos);
    }

    {
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/touchwayfail", cfg, &stats);
      TEST(stats.numReferences, ==, 0);
      TEST(res.find("$1 touches 2$") != std::string::npos);
      TEST(res.find("$2 touches 1$") != std::string::npos);
    }

    {
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/simpleareafail", cfg, &stats);
      TEST(stats.numReferences, ==, 0);
      TEST(res.find("$1 intersects 2$") != std::string::npos);
      TEST(res.find("$2 intersects 1$") != std::string::npos);
      TEST(res.find("$2 crosses 1$") != std::string::npos);
    }

    {
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/touchfail2", cfg, &stats);
      TEST(stats.numReferences, ==, 0);
      TEST(res.find("$1 covers 2$") != std::string::npos);
      TEST(res.find("$1 intersects 2$") != std::string::npos);
      TEST(res.find("$2 intersects 1$") != std::string::npos);
    }

    {
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/boxidfail", cfg, &stats);
      TEST(stats.numReferences, ==, 0);
      TEST(res.find("$osmway:312944635 intersects osmway:312944634$") !=
           std::string::npos);
      TEST(res.find("$osmway:312944634 intersects osmway:312944635$") !=
           std::string::npos);
    }
    {
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/boxidfail2", cfg, &stats);
      TEST(stats.numReferences, ==, 0);
      TEST(res.find("$osmway:205756242 intersects osmway:50218266$") !=
           std::string::npos);
      TEST(res.find("$osmway:50218266 intersects osmway:205756242$") !=
           std::string::npos);
    }
    {
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/boxidfail3", cfg, &stats);
      TEST(stats.numReferences, ==, 0);
      TEST(res.find("$osmway:901094335 intersects osmnode:8370757906$") !=
           std::string::npos);
      TEST(res.find("$osmnode:8370757906 intersects osmway:901094335$") !=
           std::string::npos);
    }

    {
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/issue-23", cfg, &stats);
      TEST(stats.numReferences, ==, 4);

      size_t a = res.find("$lsa equals lsb$");
      TEST(a != std::string::npos);
      // no second time!
      TEST(res.find("$lsa\tequals\tlsb$", a + 1) == std::string::npos);

      size_t aa = res.find("$lsb equals lsc$");
      TEST(aa != std::string::npos);
      // no second time!
      TEST(res.find("$lsb equals lsc$", aa + 1) == std::string::npos);
    }

    for (std::string dataset :
         {"duplicate-references", "multitests", "collectiontests", "freiburg",
          "issue-23"}) {
      RunStats withDups, withoutDups;

      const auto& withDupRemoval =
          fullRun(TEST_DATASET_DIR "/" + dataset, cfg, &withDups);
      const auto& withoutDupRemoval =
          fullRun(TEST_DATASET_DIR "/" + dataset, cfg, &withoutDups,
                  std::numeric_limits<double>::max());

      // TODO: duplicates in multi geometries are not yet removed
      if (dataset != "multitests" && dataset != "collectiontests")
        TEST(withDups.numReferences > withoutDups.numReferences);
      TEST(sortedRels(withDupRemoval) == sortedRels(withoutDupRemoval));

      // this tests for contradicting relations (two geometries contain AND
      // overlap each other)
      testNoContradictions(withDupRemoval);
      testNoContradictions(withoutDupRemoval);
    }

    {
      // geometries with different CRS IRIs
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/crs_iris", cfg, &stats);

      TEST(res.find("$TestA crosses TestB$") != std::string::npos);
      TEST(res.find("$TestB crosses TestA$") != std::string::npos);
      TEST(res.find("$TestD1 equals TestD2$") != std::string::npos);
      TEST(res.find("$TestD2 equals TestD1$") != std::string::npos);
      TEST(res.find("$TestD1 intersects TestD2$") != std::string::npos);
      TEST(res.find("$TestD2 intersects TestD1$") != std::string::npos);
      TEST(res.find("$TestD1 covers TestD2$") != std::string::npos);
      TEST(res.find("$TestD1 contains TestD2$") != std::string::npos);
      TEST(res.find("$TestD2 covers TestD1$") != std::string::npos);
      TEST(res.find("$TestD2 contains TestD1$") != std::string::npos);
      TEST(res.find("$TestD3 covers TestD2$") != std::string::npos);
      TEST(res.find("$TestD3 contains TestD2$") != std::string::npos);
      TEST(res.find("$TestD3 covers TestD1$") != std::string::npos);
      TEST(res.find("$TestD3 contains TestD1$") != std::string::npos);
      TEST(res.find("$TestD1 intersects TestD3$") != std::string::npos);
      TEST(res.find("$TestD2 intersects TestD3$") != std::string::npos);
      TEST(res.find("$TestD3 intersects TestD1$") != std::string::npos);
      TEST(res.find("$TestD3 intersects TestD2$") != std::string::npos);

      TEST(res.find("$TestP1 touches TestD1$") != std::string::npos);
      TEST(res.find("$TestP1 touches TestD2$") != std::string::npos);
      TEST(res.find("$TestP1 touches TestD3$") != std::string::npos);
      TEST(res.find("$TestP1 touches TestD4$") != std::string::npos);
      TEST(res.find("$TestD1 touches TestP1$") != std::string::npos);
      TEST(res.find("$TestD2 touches TestP1$") != std::string::npos);
      TEST(res.find("$TestD3 touches TestP1$") != std::string::npos);
      TEST(res.find("$TestD4 touches TestP1$") != std::string::npos);

      TEST(res.find("$TestP1 intersects TestD1$") != std::string::npos);
      TEST(res.find("$TestP1 intersects TestD2$") != std::string::npos);
      TEST(res.find("$TestP1 intersects TestD3$") != std::string::npos);
      TEST(res.find("$TestP1 intersects TestD4$") != std::string::npos);
      TEST(res.find("$TestD1 intersects TestP1$") != std::string::npos);
      TEST(res.find("$TestD2 intersects TestP1$") != std::string::npos);
      TEST(res.find("$TestD3 intersects TestP1$") != std::string::npos);
      TEST(res.find("$TestD4 intersects TestP1$") != std::string::npos);

      TEST(res.find("$unsupported$") == std::string::npos);
      TEST(res.find("$unsupported2$") == std::string::npos);
    }

    {
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/bawue", cfg, &stats);
      // TODO: duplicates in multi geometries are not yet removed
      // TEST(stats.numReferences, ==, 1);
      TEST(res.find("$germany covers bawue$") != std::string::npos);
      TEST(res.find("$germany contains bawue$") != std::string::npos);
    }

    {
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/boxidborder", cfg, &stats);

      TEST(res.find("$gridpoly covers onedge$") != std::string::npos);
      TEST(res.find("$gridpoly touches onedge$") != std::string::npos);
      TEST(res.find("$gridpoly contains onedge$") == std::string::npos);

      TEST(res.find("$gridpoly covers onvertex$") != std::string::npos);
      TEST(res.find("$gridpoly touches onvertex$") != std::string::npos);
      TEST(res.find("$gridpoly contains onvertex$") == std::string::npos);

      TEST(res.find("$gridpoly covers edgeline$") != std::string::npos);
      TEST(res.find("$gridpoly touches edgeline$") != std::string::npos);
      TEST(res.find("$gridpoly contains edgeline$") == std::string::npos);

      // geometries which really are in the interior are unaffected
      TEST(res.find("$gridpoly covers insidepoint$") != std::string::npos);
      TEST(res.find("$gridpoly contains insidepoint$") != std::string::npos);
      TEST(res.find("$gridpoly touches insidepoint$") == std::string::npos);

      TEST(res.find("$gridpoly covers insideline$") != std::string::npos);
      TEST(res.find("$gridpoly contains insideline$") != std::string::npos);
      TEST(res.find("$gridpoly touches insideline$") == std::string::npos);
    }

    {
      // a line which is covered by another line does not overlap it, but a
      // multi geometry with a member identical to that line might
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/overlapsreference", cfg, &stats);

      TEST(res.find("$big covers sub$") != std::string::npos);
      TEST(res.find("$big overlaps sub$") == std::string::npos);
      TEST(res.find("$multi overlaps big$") != std::string::npos);
      TEST(res.find("$big overlaps multi$") != std::string::npos);
    }
  }

  {
    auto cfg = all;

    for (std::string dataset :
         {"duplicate-references", "multitests", "collectiontests", "freiburg",
          "issue-23", "multi-threaded"}) {
      RunStats singleStats;
      const auto& single =
          fullRun(TEST_DATASET_DIR "/" + dataset, cfg, &singleStats);

      cfg.numThreads = 4;
      cfg.numCacheBuckets = 4;

      for (size_t i = 0; i < 3; i++) {
        cfg.numThreads = 4 - i;
        cfg.numCacheBuckets = 4 - i;
        RunStats multiStats;
        TEST(sortedRels(single) ==
             sortedRels(
                 fullRun(TEST_DATASET_DIR "/" + dataset, cfg, &multiStats)));
      }
    }
  }

  // DE9IM
  for (auto cfg : cfgs) {
    cfg.computeDE9IM = true;
    {
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/freiburg", cfg, &stats);
      TEST(stats.numReferences, ==, 3);
      TEST(res.find("$freiburg1\t2FFF1FFF2\tfreiburg2$") != std::string::npos);
      TEST(res.find("$freiburg2\t2FFF1FFF2\tfreiburg1$") != std::string::npos);
      TEST(res.find("$freiburg1\t2FFF1FFF2\tfreiburg2$") != std::string::npos);
      TEST(res.find("$freiburg2\t2FFF1FFF2\tfreiburg1$") != std::string::npos);

      TEST(res.find("$freiburg1\tFF210FFF2\tgrenzpart$") != std::string::npos);
      TEST(res.find("$grenzpart\tF1FF0F2F2\tfreiburg1$") != std::string::npos);

      TEST(res.find("$Umkirch\tFF2F11212\tfreiburg1$") != std::string::npos);
      TEST(res.find("$freiburg1\t212FF1FF2\tAltstadt$") != std::string::npos);
    }
    {
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/multide9im", cfg, &stats);

      TEST(countMatches(res, "\\$coll\\t[0-9F]{9}\\tway\\$"), ==, 1);
      TEST(countMatches(res, "\\$way\\t[0-9F]{9}\\tcoll\\$"), ==, 1);
      TEST(countMatches(res, "\\$coll\\t[0-9F]{9}\\tpoint\\$"), ==, 1);
      TEST(countMatches(res, "\\$point\\t[0-9F]{9}\\tcoll\\$"), ==, 1);

      TEST(res.find("$coll\t1F1F00FF2\tway$") != std::string::npos);
      TEST(res.find("$way\t1FFF0F102\tcoll$") != std::string::npos);
      TEST(res.find("$coll\tFF10F0FF2\tpoint$") != std::string::npos);
      TEST(res.find("$point\tF0FFFF102\tcoll$") != std::string::npos);
    }
    {
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/issue-23", cfg, &stats);
      TEST(stats.numReferences, ==, 4);

      size_t a = res.find("$lsa\t1FFF0FFF2\tlsb$");
      TEST(a != std::string::npos);
      // no second time!
      TEST(res.find("$lsa\t1FFF0FFF2\tlsb$", a + 1) == std::string::npos);

      size_t aa = res.find("$lsb\t1FFF0FFF2\tlsc$");
      TEST(aa != std::string::npos);
      // no second time!
      TEST(res.find("$lsb\t1FFF0FFF2\tlsc$", aa + 1) == std::string::npos);

      size_t b = res.find("$lsb\t1FFF0FFF2\tlsa$");
      TEST(b != std::string::npos);
      // no second time!
      TEST(res.find("$lsb\t1FFF0FFF2\tlsa$", b + 1) == std::string::npos);

      size_t c = res.find("$polya\t2FFF1FFF2\tpolyb$");
      TEST(c != std::string::npos);
      // no second time!
      TEST(res.find("$polya\t2FFF1FFF2\tpolyb$", c + 1) == std::string::npos);

      size_t d = res.find("$polyb\t2FFF1FFF2\tpolya$");
      TEST(d != std::string::npos);
      // no second time!
      TEST(res.find("$polyb\t2FFF1FFF2\tpolya$", d + 1) == std::string::npos);

      size_t e = res.find("$polyc\t2FFF1FFF2\tpolyb$");
      TEST(d != std::string::npos);
      // no second time!
      TEST(res.find("$polyc\t2FFF1FFF2\tpolyb$", e + 1) == std::string::npos);
    }
    for (std::string dataset :
         {"duplicate-references", "multitests", "collectiontests", "freiburg",
          "issue-23"}) {
      RunStats withDups, withoutDups;

      const auto& withDupRemoval =
          fullRun(TEST_DATASET_DIR "/" + dataset, cfg, &withDups);
      const auto& withoutDupRemoval =
          fullRun(TEST_DATASET_DIR "/" + dataset, cfg, &withoutDups,
                  std::numeric_limits<double>::max());

      // TODO: duplicates in multi geometries are not yet removed
      if (dataset != "multitests" && dataset != "collectiontests")
        TEST(withDups.numReferences > withoutDups.numReferences);
      TEST(sortedRels(withDupRemoval) == sortedRels(withoutDupRemoval));
    }

    {
      auto cfg = all;

      for (std::string dataset :
           {"duplicate-references", "multitests", "collectiontests", "freiburg",
            "issue-23", "multi-threaded"}) {
        RunStats singleStats;
        const auto& single =
            fullRun(TEST_DATASET_DIR "/" + dataset, cfg, &singleStats);

        cfg.numThreads = 4;
        cfg.numCacheBuckets = 4;

        for (size_t i = 0; i < 3; i++) {
          cfg.numThreads = 4 - i;
          cfg.numCacheBuckets = 4 - i;
          RunStats multiStats;
          TEST(sortedRels(single) ==
               sortedRels(
                   fullRun(TEST_DATASET_DIR "/" + dataset, cfg, &multiStats)));
        }
      }
    }
  }

  {
    // multi geometries with x < 0 (left of the meridian) must not be
    // multiOut`ed before their candidates are checked, see #28
    RunStats stats;
    writeMultiFlushDataset(".multiFlushTmp");
    auto res = fullRun(".multiFlushTmp", all, &stats);
    unlink(".multiFlushTmp");

    TEST(res.find("$a intersects b$") != std::string::npos);
    TEST(res.find("$a contains b$") != std::string::npos);
    TEST(res.find("$a covers b$") != std::string::npos);
  }

  {
    // strict parsing skips and counts invalid WKT, lenient parsing parses it
    // partially (here: `c`, `d` and `e` as a point at (-5 -5))

    RunStats stats;
    auto res = fullRun(TEST_DATASET_DIR "/wkt-fail", all, &stats, 0, true);
    TEST(stats.numInvalid, ==, 9);
    TEST(res.find("$a contains g$") == std::string::npos);
    TEST(res.find("$a contains h$") == std::string::npos);
    TEST(res.find("$a contains u$") == std::string::npos);
    TEST(res.find("$a contains b$") != std::string::npos);
    TEST(res.find("$a contains c$") == std::string::npos);
    TEST(res.find("$a contains d$") == std::string::npos);
    TEST(res.find("$a contains e$") == std::string::npos);

    res = fullRun(TEST_DATASET_DIR "/wkt-fail", all, &stats);
    TEST(stats.numInvalid, ==, 0);
    TEST(res.find("$a contains c$") != std::string::npos);
    TEST(res.find("$a contains g$") != std::string::npos);
    TEST(res.find("$a contains u$") == std::string::npos);
  }

  {
    // in the test file, a size 1 side of a non-self join is empty because the
    // WKT is invalid. This should not lead to a self join over the remaining
    // side
    RunStats stats;
    auto res = fullRun(
        std::vector<std::string>{TEST_DATASET_DIR "/self-join-regression-side1",
                                 TEST_DATASET_DIR
                                 "/self-join-regression-side2"},
        all, &stats, 0, false);
    TEST(res, ==, "");
  }

  // distance
  for (auto cfg : cfgs) {
    cfg.withinDist = 5000;
    {
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/freiburg", cfg, &stats);
      TEST(stats.numReferences, ==, 3);
      TEST(res.find("$freiburg1\t0\tfreiburg2$") != std::string::npos);
      TEST(res.find("$freiburg2\t0\tfreiburg1$") != std::string::npos);
      TEST(res.find("$freiburg1\t0\tfreiburg2$") != std::string::npos);
      TEST(res.find("$freiburg2\t0\tfreiburg1$") != std::string::npos);

      TEST(res.find("$freiburg1\t0\tgrenzpart$") != std::string::npos);
      TEST(res.find("$grenzpart\t0\tfreiburg1$") != std::string::npos);

      TEST(res.find("$Umkirch\t0\tfreiburg1$") != std::string::npos);
      TEST(res.find("$freiburg1\t0\tAltstadt$") != std::string::npos);

      TEST(res.find("$Kappel\t0\tKappel2$") != std::string::npos);
      TEST(res.find("$Kappel\t0\tKappel$") == std::string::npos);
      TEST(res.find("$Kappel2\t0\tKappel2$") == std::string::npos);

      std::regex pattern("\\$Kappel\\t3306.55\\d*\\\tHerdern\\$");
      TEST(std::regex_search(res, pattern));
      std::regex pattern2("\\$Herdern\\t3306.55\\d*\\\tKappel\\$");
      TEST(std::regex_search(res, pattern2));

      std::regex pattern3("\\$Kappel2\\t3306.55\\d*\\\tHerdern\\$");
      TEST(std::regex_search(res, pattern3));
      std::regex pattern4("\\$Herdern\\t3306.55\\d*\\\tKappel2\\$");
      TEST(std::regex_search(res, pattern4));
    }

    cfg.withinDist = 100;
    {
      RunStats stats;
      auto res =
          fullRun(TEST_DATASET_DIR "/brokenlinepointdistance", cfg, &stats);

      std::regex pattern1("\\$point2\\t9.6\\d*\\\tway\\$");
      TEST(std::regex_search(res, pattern1));

      std::regex pattern2("\\$way\\t9.6\\d*\\\tpoint2\\$");
      TEST(std::regex_search(res, pattern2));

      std::regex pattern3("\\$point\\t.*\\tway\\$");
      TEST(!std::regex_search(res, pattern3));

      std::regex pattern4("\\$way\\t.*\\tpoint\\$");
      TEST(!std::regex_search(res, pattern4));
    }

    cfg.withinDist = 1000000;
    {
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/util-issue-13", cfg, &stats);

      std::regex pattern1("\\$germany\\t426521\\.\\d*\\tlondon\\$");
      TEST(std::regex_search(res, pattern1));
      std::regex pattern2("\\$london\\t426521\\.\\d*\\tgermany\\$");
      TEST(std::regex_search(res, pattern2));
      std::regex pattern3("\\$germany\\t314975\\.\\d*\\teiffel\\$");
      TEST(std::regex_search(res, pattern3));
      std::regex pattern4("\\$eiffel\\t314975\\.\\d*\\tgermany\\$");
      TEST(std::regex_search(res, pattern4));
      std::regex pattern5("\\$eiffel\\t340875\\.\\d*\\tlondon\\$");
      TEST(std::regex_search(res, pattern5));
      std::regex pattern6("\\$london\\t340875\\.\\d*\\teiffel\\$");
      TEST(std::regex_search(res, pattern6));
    }

    cfg.withinDist = 10;
    cfg.euclideanDist = true;
    {
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/simplelinepadding", cfg, &stats);

      TEST(countMatches(res, "\\$line\\t7\\.62\\d*\\tpoly\\$"), ==, 1);
      TEST(countMatches(res, "\\$poly\\t7\\.62\\d*\\tline\\$"), ==, 1);
    }
    cfg.euclideanDist = false;

    {
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/multimeterprecision", cfg, &stats);

      TEST(countMatches(res, "\\$coll\\t5\\.929\\d*\\tpoly\\$"), ==, 1);
      TEST(countMatches(res, "\\$poly\\t5\\.929\\d*\\tcoll\\$"), ==, 1);
    }

    cfg.withinDist = 20;
    {
      // a pair of two multi geometries must be written exactly once in each
      // dir
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/multimultidist", cfg, &stats);

      TEST(countMatches(res, "\\$a\\t11\\.1\\d*\\tb\\$"), ==, 1);
      TEST(countMatches(res, "\\$b\\t11\\.1\\d*\\ta\\$"), ==, 1);
    }

    cfg.withinDist = 30;
    {
      cfg.euclideanDist = true;
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/foldedboxpadding", cfg, &stats);

      TEST(countMatches(res, "\\$house\\t11\\.1\\d*\\tlake\\$"), ==, 1);
      TEST(countMatches(res, "\\$lake\\t11\\.1\\d*\\thouse\\$"), ==, 1);
    }

    {
      cfg.euclideanDist = false;
      for (double maxDist : {50.0, 100.0, 1000.0}) {
        cfg.withinDist = maxDist;
        RunStats stats;
        auto res = fullRun(TEST_DATASET_DIR "/simplelinedistance", cfg, &stats);

        std::regex parkRiver("\\$park\\t79\\.6\\d*\\triver\\$");
        std::regex riverPark("\\$river\\t79\\.6\\d*\\tpark\\$");
        std::regex pointRiver("\\$point\\t61\\.7\\d*\\triver\\$");
        std::regex riverPoint("\\$river\\t61\\.7\\d*\\tpoint\\$");
        std::regex riverRiver("\\$river\\t14\\.3\\d*\\triver2\\$");

        TEST(std::regex_search(res, riverRiver));

        if (maxDist < 61.75) {
          TEST(!std::regex_search(res, riverPoint));
          TEST(!std::regex_search(res, pointRiver));
          TEST(!std::regex_search(res, parkRiver));
          TEST(!std::regex_search(res, riverPark));
        } else {
          TEST(std::regex_search(res, parkRiver));
          TEST(std::regex_search(res, riverPark));
          TEST(std::regex_search(res, pointRiver));
          TEST(std::regex_search(res, riverPoint));
        }
      }
    }

    // the distance to a line which is deduplicated against a member of a
    // multi geometry must not be bounded by the multi geometry's distance
    {
      cfg.withinDist = 1000;
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/multirefdistance", cfg, &stats);

      std::regex farPoly("\\$far\\t745\\.76\\d*\\tpoly\\$");
      std::regex polyFar("\\$poly\\t745\\.76\\d*\\tfar\\$");
      std::regex multiPoly("\\$multi\\t30\\.86\\d*\\tpoly\\$");
      std::regex polyMulti("\\$poly\\t30\\.86\\d*\\tmulti\\$");

      TEST(std::regex_search(res, farPoly));
      TEST(std::regex_search(res, polyFar));
      TEST(std::regex_search(res, multiPoly));
      TEST(std::regex_search(res, polyMulti));
    }

    // the distance to an axis-aligned rectangle (a "folded box polygon") must
    // not depend on the maximum distance, which the bounding box is padded with
    {
      for (double maxDist : {500.0, 2000.0, 5000.0}) {
        cfg.withinDist = maxDist;
        RunStats stats;
        auto res = fullRun(TEST_DATASET_DIR "/foldedboxdistance", cfg, &stats);

        std::regex boxPoint("\\$box\\t1001\\.88\\d*\\tpoint\\$");
        std::regex pointBox("\\$point\\t1001\\.88\\d*\\tbox\\$");

        if (maxDist < 1001.88) {
          TEST(res, ==, "");
        } else {
          TEST(std::regex_search(res, boxPoint));
          TEST(std::regex_search(res, pointBox));
        }
      }
    }

    cfg.withinDist = 50;
    {
      RunStats stats;
      auto res = fullRun(TEST_DATASET_DIR "/empty", cfg, &stats);

      TEST(res, ==, "");
    }
  }
}
