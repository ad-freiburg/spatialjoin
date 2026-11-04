// Copyright 2026, University of Fr
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

// Integration example for a non-self join over three geometry stores: two
// stores persisted on disk before, which together form the left side, and one
// non-persisted store built on-the-fly, which forms the right side

#include <cstdio>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "spatialjoin/GeometryCache.h"
#include "spatialjoin/GeometryStore.h"
#include "spatialjoin/OutputWriter.h"
#include "spatialjoin/SpatialJoinCfg.h"
#include "spatialjoin/SweepEventList.h"
#include "spatialjoin/Sweeper.h"
#include "spatialjoin/WKTParse.h"

// _____________________________________________________________________________
std::vector<sj::PreparedGeomMeta> writePersistedStorage(
    const sj::SpatialJoinCfg& cfg, const std::string& input,
    const std::string& dir, const std::string& prefix) {
  // note: persistOnDisk is true
  sj::GeometryStore store(cfg, dir, prefix, true);

  std::vector<sj::PreparedGeomMeta> metas;
  std::mutex metasMtx;
  sj::WKTParser parser(
      &store,
      [&metas, &metasMtx](size_t, const std::string&,
                          const sj::PreparedGeomMeta& meta) {
        std::lock_guard<std::mutex> lock(metasMtx);
        // metas are collected in a simple vector
        metas.push_back(meta);
      },
      cfg.numThreads);
  parser.parseFile(input);

  store.flush();  // necessary!

  return metas;
}

// _____________________________________________________________________________
int main(int, char**) {
  sj::SpatialJoinCfg cfg;

  std::vector<sj::PreparedGeomMeta> metasFr = writePersistedStorage(
      cfg, EXAMPLE_DATASET_DIR "/freiburg", ".", "spatialjoin-freiburg");
  std::vector<sj::PreparedGeomMeta> metasBuild =
      writePersistedStorage(cfg, EXAMPLE_DATASET_DIR "/freiburg-buildings", ".",
                            "spatialjoin-buildings");

  sj::OutputWriter out(cfg.numThreads, "", "\n", "", ".");

  // called by sweeper for each relation found (by thread t)
  cfg.writeRelCb = [&out](size_t t, const char* a, size_t an, const char* b,
                          size_t bn, const char* pred, size_t predn) {
    out.writeRelCb(t, a, an, b, bn, pred, predn);
  };

  {
    // use the persisted stores
    sj::GeometryStore storeFr(cfg, ".", "spatialjoin-freiburg");
    sj::GeometryStore storeBuild(cfg, ".", "spatialjoin-buildings");

    // a third, non-persisted store, built below
    sj::GeometryStore storeOnlyFr(cfg, ".", "spatialjoin-onlyfreiburg",
                                        false);

    // register all three stores
    sj::GeometryCache cache(cfg);
    const size_t storeIdFr = cache.addStore(&storeFr);
    const size_t storeIdBuild = cache.addStore(&storeBuild);
    const size_t storeIdOnlyFr = cache.addStore(&storeOnlyFr);

    // the left sweep event list
    sj::SweepEventList eventsLeft(cfg, ".", "spatialjoin-left");
    for (const auto& meta : metasFr) eventsLeft.add(meta, storeIdFr);
    for (const auto& meta : metasBuild)eventsLeft.add(meta, storeIdBuild);

    // the right sweep event list is filled during parsing
    sj::SweepEventList eventsRight(cfg, ".", "spatialjoin-right");
    sj::WKTParser parser(&storeOnlyFr,
                         eventsRight.addCb(storeIdOnlyFr),
                         cfg.numThreads);

    try {
      parser.parseFile(EXAMPLE_DATASET_DIR "/onlyfreiburg");
    } catch (const std::runtime_error& e) {
      std::cerr << e.what() << std::endl;
      return 1;
    }

    storeOnlyFr.flush();  // necessary!

    // necessary
    eventsLeft.finalize(cfg.numThreads);
    eventsRight.finalize(cfg.numThreads);

    // this is optional
    eventsLeft.duplicatesToReferences(cache);
    eventsRight.duplicatesToReferences(cache);

    sj::Sweeper sweeper(cfg, &cache);
    sweeper.sweep(&eventsLeft, &eventsRight);
  }

  // delete the persisted stores again
  for (const std::string prefix :
       {"spatialjoin-freiburg", "spatialjoin-buildings"}) {
    for (const std::string type : {"points", "areas", "simpleareas", "lines",
                                   "simplelines", "multis"}) {
      std::remove(("./" + prefix + type).c_str());
    }
  }
}
