// Copyright 2026, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

// Integration example for a non-self join on two geometry stores which were
// persisted on disk before

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
      [&metas, &metasMtx](size_t, const sj::PreparedGeomMeta& meta) {
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
int main(int argc, char** argv) {
  const std::string inputLeft =
      argc > 2 ? argv[1] : EXAMPLE_DATASET_DIR "/brandenburg_nonself-side1";
  const std::string inputRight =
      argc > 2 ? argv[2] : EXAMPLE_DATASET_DIR "/brandenburg_nonself-side2";

  sj::SpatialJoinCfg cfg;

  // one persisted store per side, each with its own prefix
  std::vector<sj::PreparedGeomMeta> metasLeft =
      writePersistedStorage(cfg, inputLeft, ".", "spatialjoin-left");
  std::vector<sj::PreparedGeomMeta> metasRight =
      writePersistedStorage(cfg, inputRight, ".", "spatialjoin-right");

  sj::OutputWriter out(cfg.numThreads, "", "\n", "", ".");

  // called by sweeper for each relation found (by thread t)
  cfg.writeRelCb = [&out](size_t t, const char* a, size_t an, const char* b,
                          size_t bn, const char* pred, size_t predn) {
    out.writeRelCb(t, a, an, b, bn, pred, predn);
  };

  {
    // use the persisted stores
    sj::GeometryStore storeLeft(cfg, ".", "spatialjoin-left");
    sj::GeometryStore storeRight(cfg, ".", "spatialjoin-right");

    // both stores are registered in a single cache, the returned store IDs
    // (here 0 and 1) tell the event lists where a geometry is stored
    sj::GeometryCache cache(cfg);
    const size_t storeIdLeft = cache.addStore(&storeLeft);
    const size_t storeIdRight = cache.addStore(&storeRight);

    // build the left and right sweep event lists from the metas
    sj::SweepEventList eventsLeft(cfg, ".", "spatialjoin-left");
    sj::SweepEventList eventsRight(cfg, ".", "spatialjoin-right");
    for (const auto& meta : metasLeft) eventsLeft.add(meta, storeIdLeft);
    for (const auto& meta : metasRight) eventsRight.add(meta, storeIdRight);

    // necessary
    eventsLeft.finalize(cfg.numThreads);
    eventsRight.finalize(cfg.numThreads);

    // this is optional
    eventsLeft.duplicatesToReferences(cache);
    eventsRight.duplicatesToReferences(cache);

    // only geometries from different lists are compared
    sj::Sweeper sweeper(cfg, &cache);
    sweeper.sweep(&eventsLeft, &eventsRight);
  }

  // delete the persisted stores again
  for (const std::string prefix : {"spatialjoin-left", "spatialjoin-right"}) {
    for (const std::string type : {"points", "areas", "simpleareas", "lines",
                                   "simplelines", "multis"}) {
      std::remove(("./" + prefix + type).c_str());
    }
  }
}
