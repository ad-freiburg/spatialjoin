// Copyright 2026, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#include "TypedGeometryStoreTest.h"

#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include <sstream>
#include <stdexcept>
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
std::string raw(const PointStore& store, const std::string& id) {
  std::stringstream ss;
  store.writeTo(Point{id, 0}, ss);
  return ss.str();
}

// _____________________________________________________________________________
std::string longId(size_t i) {
  return std::to_string(i) + std::string(60000, 'x');
}

// _____________________________________________________________________________
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
void checkOnDisk(const PointStore& store, const std::vector<size_t>& offs) {
  TEST(!store.inMemory());
  for (size_t t = 0; t < NUM_THREADS; t++) {
    for (size_t i : {size_t(0), offs.size() / 2, offs.size() - 1}) {
      TEST(store.getFrom(offs[i], t).second.id == longId(i));
    }
  }
}

// _____________________________________________________________________________
std::vector<std::string> files(const std::string& dir) {
  std::vector<std::string> ret;
  DIR* d = opendir(dir.c_str());
  TEST(d != 0);
  while (const dirent* entry = readdir(d)) {
    std::string name = entry->d_name;
    if (name != "." && name != "..") ret.push_back(dir + "/" + name);
  }
  closedir(d);
  return ret;
}

// _____________________________________________________________________________
void removeDir(const std::string& dir) {
  DIR* d = opendir(dir.c_str());
  if (!d) return;
  while (const dirent* entry = readdir(d)) {
    std::string name = entry->d_name;
    if (name != "." && name != "..") unlink((dir + "/" + name).c_str());
  }
  closedir(d);
  rmdir(dir.c_str());
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
    PointStore a(NUM_THREADS, ".", ".spatialjoin");
    auto offA = a.add(raw(a, "a"));
    PointStore b(NUM_THREADS, ".", ".spatialjoin");
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
    PointStore a(NUM_THREADS, ".", ".spatialjoin");
    PointStore b(std::move(a));
    auto offs = fillToDisk(&b);
    b.flush();
    checkOnDisk(b, offs);
  }

  {
    PointStore a(NUM_THREADS, ".", ".spatialjoin");
    auto offs = fillToDisk(&a);
    PointStore b(NUM_THREADS, ".", ".spatialjoin");
    b.add(raw(b, "b"));

    b = std::move(a);
    b.flush();
    checkOnDisk(b, offs);

    PointStore c(std::move(b));
    checkOnDisk(c, offs);

    // move assignment of an on-disk store onto an on-disk store
    PointStore d(NUM_THREADS, ".", ".spatialjoin");
    fillToDisk(&d);
    d.flush();
    d = std::move(c);
    checkOnDisk(d, offs);
  }

  {
    const std::string dir = "./.spatialjoin-test-persist-store";
    const std::string path = dir + "/.spatialjoinpoints";

    // left over from an aborted earlier run
    removeDir(dir);
    TEST(mkdir(dir.c_str(), 0777) == 0);

    TEST_THROWS(std::runtime_error, PointStore(NUM_THREADS, path));

    {
      // stores which are not persisted leave no file, in memory and on disk
      PointStore a(NUM_THREADS, dir, ".spatialjoin");
      a.add(raw(a, "a"));
      a.flush();
      PointStore b(NUM_THREADS, dir, ".spatialjoin");
      fillToDisk(&b);
      b.flush();
      TEST(files(dir).empty());
    }

    {
      // a store held in memory
      PointStore a(NUM_THREADS, dir, ".spatialjoin", true);
      auto offA = a.add(raw(a, "a"));
      auto offB = a.add(raw(a, "b"));
      a.flush();
      TEST(files(dir) == std::vector<std::string>{path});

      // the original stays in memory
      TEST(a.inMemory());
      TEST(a.getFromMem(offB)->id == "b");

      PointStore b(NUM_THREADS, path);
      TEST(!b.inMemory());
      for (size_t t = 0; t < NUM_THREADS; t++) {
        TEST(b.getFrom(offA, t).second.id == "a");
        TEST(b.getFrom(offB, t).second.id == "b");
      }
    }

    {
      // the file is kept after the store was destroyed
      PointStore b(NUM_THREADS, path);
      TEST(b.getFrom(0, 1).second.id == "a");
      unlink(path.c_str());
    }

    std::vector<size_t> offs;
    {
      // a store held on disk
      PointStore a(NUM_THREADS, dir, ".spatialjoin", true);
      offs = fillToDisk(&a);
      a.flush();
      TEST(files(dir) == std::vector<std::string>{path});
      checkOnDisk(a, offs);

      PointStore b(NUM_THREADS, path);
      checkOnDisk(b, offs);

      // a loaded store can be moved
      PointStore c(std::move(b));
      checkOnDisk(c, offs);
    }

    {
      // the file is kept after the store was destroyed
      PointStore b(NUM_THREADS, path);
      checkOnDisk(b, offs);
    }

    removeDir(dir);
  }
}
