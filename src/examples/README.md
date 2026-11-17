# `libspatialjoin` integration examples

This folder contains several integration examples of `libspatialjoin`.

## Overview

All examples use the following classes:

* `SpatialJoinCfg`: The overall config, with reasonable defaults.
* `GeometryStore(<config>, <dir>, <prefix>, <persistOnDisk>)`: holds the parsed geometries, preprocessed according to `<config>`. Fully in memory, will switch to a disk-based storaged in `<dir>`, with files prefixed by `<prefix>`, if memory usaged gets too high. If `<persistOnDisk>` is true, the files on disk are preserved, and the storage is always disk-based. Has a constructor to re-use a disk-persisted storage (see TODO).
* `WKTParser(<store>, <addCallback>, <numThreads>)`: offers parsing methods to parse WKT geoms into `<store>`, in parallel, using `<numThreads>` threads. The `<addCallback>` is called async with `PreparedGeomMeta` objects holding shallow meta information on the added geometry (most importantly the internal geometry ID and the bounding box).
* `SweepEventList(<config>, <dir>, <prefix>)`: A sweep event list, offering a method `.add(<preparedgeometrymeta>, <storeid>)`. The `<storeid>` tells the list from which store the geometry comes from (`<storeId>` is 8bit, if a single store is used, it is 0). Fully in memory, until a memory threshold is reached, then it will switch to a disk-based storage in `<dir>` with prefix `<prefix>`. Can optionally also be persisted on disk and re-read. Offers a convenience method `.addCb(<storeid>)` which returns a synchronized callback to add a `PreparedGeomMeta` from from store `<storeid>`.
* `GeometryCache(<config)`:  Offers a method `.addStore()` to register a `GeometryStore`. Is then used by the `Sweeper` to retrieve geometries from disk and cache them in a FIFO cache.
* `Sweeper(<config>, <geometrycache>)`: offers the `.sweep(<leftlist>, <rightList>)` method which joins the left `SweepEventList` with the right `SweepEventList`. If only a `<leftlist>` is given, computes a self-join. Calls asynchronously the `writeRelCb` from the cfg with result rows.

## `SelfJoin.cpp`

A single self join from a single dataset read from a file.

## `NonSelfJoin.cpp`

A single non-self join between two datasets read from two files into two storages.

## `NonSelfJoinSameStorage.cpp`

A single non-self join within a single dataset read from a file into a single storage. Left and right `SweepEventList` are than build manually.

## `SelfJoinStorageReuse.cpp`

A self join on a persisted storage.

## `NonSelfJoinStorageReuse.cpp`

A non-self join on two persisted storages.

## `ThreeStoresJoin.cpp`

A non-self join over three storages.
