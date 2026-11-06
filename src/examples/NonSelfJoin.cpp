// Copyright 2026, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

// Integration example for a simple non-self join

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
  const std::string inputLeft =
      argc > 2 ? argv[1] : EXAMPLE_DATASET_DIR "/brandenburg_nonself-side1";
  const std::string inputRight =
      argc > 2 ? argv[2] : EXAMPLE_DATASET_DIR "/brandenburg_nonself-side2";

  const std::string dir = ".";

  // each store has own file prefix
  const std::string prefixLeft = "spatialjoin-left";
  const std::string prefixRight = "spatialjoin-right";

  sj::SpatialJoinCfg cfg;

  sj::OutputWriter out(cfg.numThreads, "", "\n", "", ".");

  // called by sweeper for each relation found (by thread t)
  cfg.writeRelCb = [&out](size_t t, const char* a, size_t an, const char* b,
                          size_t bn, const char* pred, size_t predn) {
    out.writeRelCb(t, a, an, b, bn, pred, predn);
  };

  // one store and one event list per side
  sj::GeometryStore storeLeft(cfg, dir, prefixLeft, false);
  sj::GeometryStore storeRight(cfg, dir, prefixRight, false);
  sj::SweepEventList eventsLeft(cfg, dir, prefixLeft);
  sj::SweepEventList eventsRight(cfg, dir, prefixRight);

  sj::GeometryCache cache(cfg);
  const size_t storeIdLeft = cache.addStore(&storeLeft);
  const size_t storeIdRight = cache.addStore(&storeRight);

  sj::WKTParser parserLeft(&storeLeft, eventsLeft.addCb(storeIdLeft),
                           cfg.numThreads);
  sj::WKTParser parserRight(&storeRight, eventsRight.addCb(storeIdRight),
                            cfg.numThreads);

  try {
    parserLeft.parseFile(inputLeft);
    parserRight.parseFile(inputRight);
  } catch (const std::runtime_error& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  storeLeft.flush();   // necessary!
  storeRight.flush();  // necessary!

  // necessary, this sorts the event files
  eventsLeft.finalize(cfg.numThreads);
  eventsRight.finalize(cfg.numThreads);

  // this is optional
  eventsLeft.duplicatesToReferences(cache);
  eventsRight.duplicatesToReferences(cache);

  // only geometries from different lists are compared
  sj::Sweeper sweeper(cfg, &cache);
  sweeper.sweep(&eventsLeft, &eventsRight);
}
