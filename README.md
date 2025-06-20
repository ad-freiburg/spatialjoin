# spatialjoin

Compute a spatial self-join (on intersects, contains, covers, touches, crosses, overlaps and equals) on line-separated WKT geometries read from stdin. Also supports within-distance joins for arbitrary meter distances if the input geometries are given as WGS84 coordinates.

Relations are written to stdout (or to a BZ2/GZ/plain file specified with `-o`).

Can handle massive amounts of input data. For example, the full self-join on the complete ~1.5 B geometries of OpenStreetMap can be computed (excluding the time required for input parsing and output writing) in around 90 minutes on an AMD
Ryzen 9 7950X machine with 16 physical and 32 virtual cores, 128 GB of RAM (DDR5), and 7.7 TB of disk space (NVMe SSD).

## Reproducibility materials for SIGSPATIAL'25 submission 143

Additional materials required to do a full evaluation of our tool and a comparison against `libgeos` can be found [here](https://github.com/ad-freiburg/spatialjoin/tree/use-libgeos-option/sigspatial-reproducibility-143).

## Requirements

 * `cmake`
 * `gcc >= 5.0` (or `clang >= 3.9`)
 * `libbz2` (*optional*)

## Building and Installation

Fetch this repository and init submodules:

```
git clone --recurse-submodules https://github.com/ad-freiburg/spatialjoin
```

```
mkdir build && cd build
cmake ..
make -j
```

To install, type
```
make install
```

## Usage

```
$ cat example.txt
POLYGON((0 0, 10  0 ,10 10, 0 10, 0 0))
POLYGON((0 0, 10 0, 10 10, 0 10, 0 0), (1 1, 9 1, 9 9, 1 9, 1 1))
MULTIPOLYGON(((0 0, 10 0, 10 10, 0 10, 0 0), (1 1, 9 1, 9 9, 1 9, 1 1)))
POLYGON((4 4, 5 4, 5 5, 4 5, 4 4))
POLYGON((4 4, 5 4, 5 11, 4 11, 4 4))
LINESTRING(1 1, 1 2)
LINESTRING(0.5 1.5, 1.5 1.5)
LINESTRING(-10 1, 100 1)
POINT(0.5 0.5)
```

```
$ spatialjoin < example.txt
1 contains 9
9 intersects 1
[...]
```

### Custom IDs

You may specify a custom geometry string ID, outputted instead of the line number, before the WKT, separated by a tab:

```
$ cat example.txt
polygon1	POLYGON((0 0, 10  0 ,10 10, 0 10, 0 0))
polygon2	POLYGON((0 0, 10 0, 10 10, 0 10, 0 0), (1 1, 9 1, 9 9, 1 9, 1 1))
multipolygon3	MULTIPOLYGON(((0 0, 10 0, 10 10, 0 10, 0 0), (1 1, 9 1, 9 9, 1 9, 1 1)))
polygon4	POLYGON((4 4, 5 4, 5 5, 4 5, 4 4))
polygon5	POLYGON((4 4, 5 4, 5 11, 4 11, 4 4))
linestring6	LINESTRING(1 1, 1 2)
linestring7	LINESTRING(0.5 1.5, 1.5 1.5)
linestring8	LINESTRING(-10 1, 100 1)
point9	POINT(0.5 0.5)
```

```
$ spatialjoin < example.txt
polygon1 contains point9
point9 intersects polygon1
[...]
```

### Non-self joins

You may specify a "side" (either 0 or 1) per geometry, as an additional tab-separated field after the custom geometry ID. If sides are defined, only geometries from different sides are compared. Note that a custom geometry ID *must* be given, otherwise the side will be interpreted as the custom geometry ID. The default side is 0.

```
$ cat example.txt
polygon1	0	POLYGON((0 0, 10  0 ,10 10, 0 10, 0 0))
polygon2	0	POLYGON((0 0, 10 0, 10 10, 0 10, 0 0), (1 1, 9 1, 9 9, 1 9, 1 1))
multipolygon3	0	MULTIPOLYGON(((0 0, 10 0, 10 10, 0 10, 0 0), (1 1, 9 1, 9 9, 1 9, 1 1)))
polygon4	1	POLYGON((4 4, 5 4, 5 5, 4 5, 4 4))
polygon5	1	POLYGON((4 4, 5 4, 5 11, 4 11, 4 4))
linestring6	1	LINESTRING(1 1, 1 2)
linestring7	1	LINESTRING(0.5 1.5, 1.5 1.5)
linestring8	0	LINESTRING(-10 1, 100 1)
point9	1	POINT(0.5 0.5)
```

### `within-distance` queries

To compute a spatial join on a `within-distance` predicate, use the `--within-distance <METER>` option. `spatialjoin` will output all pairs of geometries which are within `<METER>` meters of each other. Note that your input geometries **must be** WGS 84 coordinates (longitude/latitude pairs) for correct meter distance computation.
