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

## Use with QLever and osm2rdf

One use case of `spatialjoin` is to add triples for the relations `contains` and
`intersects` to an RDF dataset with WKT literals. The following example shows
the process for the OSM data for germany.

### Step 1: Download PBF from Geofabrik and convert to RDF

```
NAME=osm-germany
wget -O ${NAME}.pbf https://download.geofabrik.de/europe/germany-latest.osm.pbf
osm2rdf ${NAME}.pbf -o ${NAME}.ttl --simplify-wkt 0 --write-ogc-geo-triples none
```

Note: `osm2rdf` by default computes and outputs the predicates `ogc:sfContains`
and `ogc:sfIntersects`. The `--write-ogc-geo-triples none` option disables
this. To have both the `osm2rdf` predicates *and* the `spatiajoin` predicates
(for comparison or debugging), just omit the option.

### Step 2: Build a QLever instance, start it, and download the geometries

```
PORT=7008
echo '{ "languages-internal": [], "prefixes-external": [""], "ascii-prefixes-only": false, "num-triples-per-batch": 1000000 }' > ${NAME}.settings.json
ulimit -Sn 1048576; bzcat ${NAME}.ttl.bz2 | IndexBuilderMain -F ttl -f - -i ${NAME} -s ${NAME}.settings.json --stxxl-memory 10G | tee ${NAME}.index-log.txt
ServerMain -i ${NAME} -j 8 -p ${PORT} -m 20G -c 10G -e 3G -k 200 -s 300s
curl -s localhost:${PORT} -H "Accept: text/tab-separated-values" -H "Content-type: application/sparql-query" --data "PREFIX geo: <http://www.opengis.net/ont/geosparql#> SELECT ?osm_id ?geometry WHERE { ?osm_id geo:hasGeometry/geo:asWKT ?geometry }" | sed -E 's#<https://www.openstreetmap.org/(rel|way|node)(ation)?/([0-9]+)>\t"(.+)"\^\^<http:.*wktLiteral>#osm\1:\3\t\4#g' | sed 1d > spatialjoin.input.tsv
```

Note: The `sed` command replaces the full IRIs by shorter prefixed IRIs. Also
note that we only get the WKT literals from `geo:gasGeometry/geo:asWKT` here.
It would be nicer to fetch all WKT literals in the datasets, no matter to which
predicate they belong (for example, the predicates `osm2rdfgeom:envelope` or
`osm2rdfgeom:convex_hull` also have WKT literals as objects)

### Step 3: Compute the spatial relations

```
cat spatialjoin.input.tsv | spatialjoin --contains ' ogc:_contains ' --intersects ' ogc:_intersects ' --suffix $' .\n'

```

Note that we could feed the geometries directly into `spatialjoin` as follows:

```
curl -s localhost:${PORT} -H "Accept: text/tab-separated-values" -H "Content-type: application/sparql-query" --data "PREFIX geo: <http://www.opengis.net/ont/geosparql#> SELECT ?osm_id ?geometry WHERE { ?osm_id geo:hasGeometry/geo:asWKT ?geometry }" | sed -E 's#<https://www.openstreetmap.org/(rel|way|node)(ation)?/([0-9]+)>\t"(.+)"\^\^<http:.*wktLiteral>#osm\1:\3\t\4#g' | sed 1d | spatialjoin --contains ' ogc:_contains ' --intersects ' ogc:_intersects ' --suffix $' .\n'
```

### Step 4: Rebuild the QLever index with the added triples

```
ulimit -Sn 1048576; bzcat ${NAME}.ttl.bz2 rels.out.bz2 | IndexBuilderMain -F ttl -f - -i ${NAME} -s ${NAME}.settings.json --stxxl-memory 10G | tee ${NAME}.index-log.txt
ServerMain -i ${NAME} -j 8 -p ${PORT} -m 20G -c 10G -e 3G -k 200 -s 300s
```
