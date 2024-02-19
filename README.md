# Spatialjoins

Compute a spatial self-join (intersects and contains) on line-separated WKT geometries read from stdin.

Relations are written into a file `rels.out.bz2`.

## Example

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
$ mkdir build
$ cd build
$ cmake ..
$ make spatialjoin
$ ./spatialjoin < example.txt
```

```
$ bzcat rels.out.bz2
1 contains 9
9 intersects 1
[...]
```
