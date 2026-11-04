// Copyright 2026, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

// Integration example for a simple self-join

#include <iostream>
#include <stdexcept>
#include <string>

#include "spatialjoin/GeometryCache.h"
#include "spatialjoin/GeometryStore.h"
#include "spatialjoin/OutputWriter.h"
#include "spatialjoin/SpatialJoinCfg.h"
#include "spatialjoin/SweepEventList.h"
#include "spatialjoin/Sweeper.h"
#include "spatialjoin/WKTParse.h"

// _____________________________________________________________________________
int main(int argc, char** argv) {
  const std::string input =
      argc > 1 ? argv[1] : EXAMPLE_DATASET_DIR "/freiburg";

  const std::string dir = ".";
  const std::string prefix = "spatialjoin";

  sj::SpatialJoinCfg cfg;

  sj::OutputWriter out(cfg.numThreads, "", "\n", "", ".");

  // called by sweeper for each relation found (by thread t)
  cfg.writeRelCb = [&out](size_t t, const char* a, size_t an, const char* b,
                          size_t bn, const char* pred, size_t predn) {
    out.writeRelCb(t, a, an, b, bn, pred, predn);
  };

  sj::GeometryStore store(cfg, dir, prefix, false);
  sj::SweepEventList events(cfg, dir, prefix);

  sj::WKTParser parser(&store, events.addCb(0), cfg.numThreads);

  // EQUIVALENT, with explicit callback:
  // std::mutex eventsMtx;
  // sj::WKTParser parser(
      // &store,
      // [&events, &eventsMtx](size_t, const std::string&,
      //                       const sj::PreparedGeomMeta& meta) {
        // // the PreparedGeomMetas may also be stored elsewhere :)
        // std::lock_guard<std::mutex> lock(eventsMtx);
        // events.add(meta, 0);
      // },
      // cfg.numThreads);

  sj::GeometryCache cache(cfg);
  cache.addStore(&store);

  try {
    parser.parseFile(input);
  } catch (const std::runtime_error& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  store.flush();  // necessary!
  events.finalize(cfg.numThreads);  // necessary, this sorts the event file
  events.duplicatesToReferences(cache);  // this is optional

  sj::Sweeper sweeper(cfg, &cache);
  sweeper.sweep(&events);
}
