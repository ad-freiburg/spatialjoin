// Copyright 2026, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#include "TypedGeometryStoreTest.h"

#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "spatialjoin/GeometryStore.h"
#include "util/Test.h"

using sj::Point;
using PointStore = sj::TypedGeometryStore<Point>;

namespace {

const size_t NUM_THREADS = 2;

// _____________________________________________________________________________
// the serialized point with id, as expected by PointStore::add()
std::string raw(const PointStore& store, const std::string& id) {
  std::stringstream ss;
  store.writeTo(Point{id, 0}, ss);
  return ss.str();
}

// _____________________________________________________________________________
// an id long enough that about 1000 points no longer fit into memory
std::string longId(size_t i) {
  return std::to_string(i) + std::string(60000, 'x');
}

// _____________________________________________________________________________
// adds points with long ids to store until it is written to disk, returns
// their offsets
std::vector<size_t> fillToDisk(PointStore* store) {
  std::vector<size_t> ret;
  while (store->inMemory())
    ret.push_back(store->add(raw(*store, longId(ret.size()))));

  // some more points, written directly to disk
  for (size_t i = 0; i < 10; i++)
    ret.push_back(store->add(raw(*store, longId(ret.size()))));

  return ret;
}

// _____________________________________________________________________________
// checks that the (flushed) on-disk store holds the points added by
// fillToDisk(), reading with every thread
void checkOnDisk(const PointStore& store, const std::vector<size_t>& offs) {
  TEST(!store.inMemory());
  for (size_t t = 0; t < NUM_THREADS; t++) {
    for (size_t i : {size_t(0), offs.size() / 2, offs.size() - 1}) {
      TEST(store.getFrom(offs[i], t).second.id == longId(i));
    }
  }
}

}  // namespace

// _____________________________________________________________________________
void typedGeometryStoreTest() {
  static_assert(!std::is_copy_constructible<PointStore>::value,
                "TypedGeometryStore must not be copyable");
  static_assert(std::is_move_constructible<PointStore>::value,
                "TypedGeometryStore must be move constructible");
  static_assert(std::is_move_assignable<PointStore>::value,
                "TypedGeometryStore must be move assignable");

  {
    // in memory: move construction and move assignment
    PointStore a(NUM_THREADS, ".");
    auto offA = a.add(raw(a, "a"));
    PointStore b(NUM_THREADS, ".");
    b.add(raw(b, "b"));

    b = std::move(a);
    TEST(b.inMemory());
    TEST(b.getFromMem(offA)->id == "a");

    PointStore c(std::move(b));
    TEST(c.inMemory());
    TEST(c.getFromMem(offA)->id == "a");

    // a moved-from store can be assigned to again
    a = std::move(c);
    TEST(a.getFromMem(offA)->id == "a");
  }

  {
    // a store moved while still in memory keeps its settings (directory,
    // number of read threads) for creating its file later
    PointStore a(NUM_THREADS, ".");
    PointStore b(std::move(a));
    auto offs = fillToDisk(&b);
    b.flush();
    checkOnDisk(b, offs);
  }

  {
    // on disk: move assignment onto an in-memory store before flushing, with
    // geometries still in the write buffer, then move construction
    PointStore a(NUM_THREADS, ".");
    auto offs = fillToDisk(&a);
    PointStore b(NUM_THREADS, ".");
    b.add(raw(b, "b"));

    b = std::move(a);
    b.flush();
    checkOnDisk(b, offs);

    PointStore c(std::move(b));
    checkOnDisk(c, offs);

    // move assignment of an on-disk store onto an on-disk store
    PointStore d(NUM_THREADS, ".");
    fillToDisk(&d);
    d.flush();
    d = std::move(c);
    checkOnDisk(d, offs);
  }
}
