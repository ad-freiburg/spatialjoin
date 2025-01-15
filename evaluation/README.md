# Evaluation instructions and results

We evaluated the performance of our spatial join and compared it against
PostgreSQL+PostGIS. In the following sections, we provide instructions and
results for the evaluation.

## Setup PostgreSQL and PostGIS on Ubuntu 24.04

We first install the required packages:


```
sudo apt update
sudo apt install postgresql postgresql-contrib postgis postgresql-16-postgis-3 
sudo apt-get install gdal-bin
```

Next, we create a new database in a directory of our choice.
postgresql-16-postgis-3
```
export POSTGIS_DIR=/local/data-ssd/postgis/spatialjoin
sudo mkdir -p ${POSTGIS_DIR} && sudo chown postgres:postgres ${POSTGIS_DIR}
sudo -u postgres /usr/lib/postgresql/16/bin/initdb -D ${POSTGIS_DIR}
sudo vim ${POSTGIS_DIR}/postgresql.conf
EDIT: work_mem = 4MB
EDIT: max_worker_processes = 8
EDIT: max_parallel_workers_per_gather = 2
EDIT: max_parallel_workers = 8
sudo su - postgres -c "/usr/lib/postgresql/16/bin/pg_ctl -D ${POSTGIS_DIR} -l logfile start"
psql -U postgres -c "SHOW data_directory;"
psql -U postgres -c "SHOW work_mem;"
```

Optionally, set up the `postgres` user and group (only neeeded on machines,
where the changes to `/etc/passwd` and `/etc/group` from the installation are
not persistent), and create a simple `.bashrc` file for the `postgres` user.

```
export POSTGRES_UID=$(stat -c %u ${POSTGIS_DIR})
export POSTGRES_GID=$(stat -c %g ${POSTGIS_DIR})
sudo groupadd -g ${POSTGRES_GID} postgres && sudo useradd -u ${POSTGRES_UID} -g ${POSTGRES_GID} -s /bin/bash -m -d ${POSTGIS_DIR} postgres
echo -e 'export PS1="\u@\h:\W$ "\nalias ll="ls -alh"' > .bashrc
ln -s .bashrc .bash_profile
```

## Create a database (if not already created), and list existing tables and indexes

Create a database `spatialjoin_db`, enable PostGIS, and list all public (that
is, not system) tables and indexes in the `spatialjoin_db` database (the `+`
provides additional information). The first two commands will do nothing if
the database and extension were already created before.


```
sudo su - postgres -c "createdb spatialjoin_db"
psql -U postgres -d spatialjoin_db -c "CREATE EXTENSION postgis;"
psql -U postgres -d spatialjoin_db -c "\dt+ public.*"
psql -U postgres -d spatialjoin_db -c "\di+ public.*"
```

To remove tables and indexes with a certain prefix, use the following commands:

```
psql -U postgres -d spatialjoin_db -c "DROP TABLE public.freiburg*;"
psql -U postgres -d spatialjoin_db -c "DROP INDEX public.freiburg*;"
```

## Get all OSM data for a region, load it into the database, and query it

The following `curl` command produces a TSV file from a SPARQL query that
contains all geometries that are contained in the region specified by the
`osmrel:...` relation. The table has two columns: `osm_id` and `geometry`.

```
export NAME=freiburg
curl -s https://qlever.cs.uni-freiburg.de/api/osm-planet -H "Accept: text/csv" -H "Content-type: application/sparql-query" --data "PREFIX geo: <http://www.opengis.net/ont/geosparql#> PREFIX ogc: <http://www.opengis.net/rdf#> PREFIX osmrel: <https://www.openstreetmap.org/relation/> SELECT ?osm_id ?geometry WHERE { osmrel:62768 ogc:sfContains ?osm_id . ?osm_id geo:hasGeometry/geo:asWKT ?geometry }" | sed 's/,/\t/;s|https://www.openstreetmap.org/|osm|;s|/|:|;s/"//g' > $NAME.tsv
```

The following commands load the data into the database, and create a spatial index.
This produces one table called `${NAME}` and two indexes called `${NAME}_pkey`
and `${NAME}_geom_idx`.


```
psql -U postgres -d spatialjoin_db -c "CREATE TABLE ${NAME} (id VARCHAR PRIMARY KEY, geom GEOMETRY);"
psql -U postgres -d spatialjoin_db -c "CREATE TABLE ${NAME}_loader (id VARCHAR, geom_text VARCHAR);"
psql -U postgres -d spatialjoin_db -c "\copy ${NAME}_loader FROM '$(pwd)/${NAME}.tsv' WITH (FORMAT csv, DELIMITER E'\t', HEADER true);"
psql -U postgres -d spatialjoin_db -c "INSERT INTO ${NAME} (id, geom) SELECT id, ST_GeomFromText(geom_text, 4326) FROM ${NAME}_loader;"
psql -U postgres -d spatialjoin_db -c "DROP table ${NAME}_loader;"
psql -U postgres -d spatialjoin_db -c "SELECT COUNT(*) FROM ${NAME};"
psql -U postgres -d spatialjoin_db -c "CREATE INDEX ${NAME}_geom_idx ON ${NAME} USING GIST (geom);"
psql -U postgres -d spatialjoin_db -c "\dt+ public.${NAME}*"
psql -U postgres -d spatialjoin_db -c "\di+ public.${NAME}*"
```

The following commands compute the complete spatial self-join, output its
size and reports the query time. The first command only computes the number of candidate pairs (that is,
the number of pairs of geometries that have a bounding box overlap). The second
command computes the exact number of pairs that intersect.

```
psql -U postgres -d spatialjoin_db -c "\timing" -c "SELECT COUNT(*) FROM ${NAME} AS a, ${NAME} AS b WHERE a.geom && b.geom;"
psql -U postgres -d spatialjoin_db -c "\timing" -c "SELECT COUNT(*) FROM ${NAME} AS a, ${NAME} AS b WHERE ST_Intersects(a.geom, b.geom);"
```

## Create table for complete OSM data

The following produces a TSV file for all OSM objects of a certain class. The
TSV file has four columns: the OSM id, the class name, the type, and the
geometry.

```
export CLASS=building
curl -s https://qlever.cs.uni-freiburg.de/api/osm-planet -H "Accept: text/tab-separated-values" -H "Content-type: application/sparql-query" --data "PREFIX osm: <https://www.openstreetmap.org/> PREFIX geo: <http://www.opengis.net/ont/geosparql#> PREFIX ogc: <http://www.opengis.net/rdf#> PREFIX osmrel: <https://www.openstreetmap.org/relation/> PREFIX osmkey: <https://www.openstreetmap.org/wiki/Key:> SELECT (REPLACE(REPLACE(STR(?osm_id_), STR(osm:), \"osm\"), \"/\", \":\") AS ?osm_id) (REPLACE(STR(osmkey:${CLASS}), STR(osmkey:), \"\") AS ?predicate) ?type ?geometry WHERE { { SELECT ?osm_id_ (SAMPLE(?type_) AS ?type) WHERE { ?osm_id_ osmkey:${CLASS} ?type_ } GROUP BY ?osm_id_ } ?osm_id_ geo:hasGeometry/geo:asWKT ?geometry }" | sed 's/"//g;s/\^\^<http[^\t]*>$//' > ${CLASS}.tsv
```

```
export NAME=osm-planet
psql -U postgres -d spatialjoin_db -c "CREATE TABLE ${NAME} (id VARCHAR PRIMARY KEY, class VARCHAR, type VARCHAR, geom GEOMETRY);"
psql -U postgres -d spatialjoin_db -c "CREATE TABLE ${NAME}_loader (id VARCHAR, class VARCHAR, type VARCHAR, geom_text VARCHAR);"
psql -U postgres -d spatialjoin_db -c "COPY ${NAME}_loader FROM '$(pwd)/building.tsv' WITH (FORMAT csv, DELIMITER E'\t', HEADER true);"
psql -U postgres -d spatialjoin_db -c "COPY ${NAME}_loader FROM '$(pwd)/highway.tsv' WITH (FORMAT csv, DELIMITER E'\t', HEADER true);"
psql -U postgres -d spatialjoin_db -c "INSERT INTO ${NAME} (id, class, type, geom) SELECT DISTINCT ON (id) id, class, type, ST_GeomFromText(geom_text, 4326) FROM ${NAME}_loader;"
psql -U postgres -d spatialjoin_db -c "DROP table ${NAME}_loader;"
psql -U postgres -d spatialjoin_db -c "SELECT COUNT(*) FROM ${NAME};"
psql -U postgres -d spatialjoin_db -c "CREATE INDEX ${NAME}_geom_idx ON ${NAME} USING GIST (geom);"
psql -U postgres -d spatialjoin_db -c "\dt+ public.${NAME}*"
psql -U postgres -d spatialjoin_db -c "\di+ public.${NAME}*"
```

## Compare variants of our own spatial join

First build the `spatialjoin` executable and include it in the `PATH`:

```
git clone --recurse-submodules https://github.com/ad-freiburg/spatialjoin
cd spatialjoin
mkdir build && cd build
cmake ..
make -j
cd ..
export PATH=PATH:$(pwd)/build:$(pwd)/scripts

```

Create a file `${NAME}.spatialjoin-input.tsv` with two columns (id and
geometry) and no header, following the instructions [above](#get-all-osm-data-for-a-region-load-it-into-the-database-and-query-it). The you can
und and evaluate variants of our spatial join as follows:

```
./spatialjoin-evaluation.py ${NAME} --combinations bcsdoi,Bcsdoi,BCsdoi,BCSdoi,BCSDoi,BCSdOi,BCSdoI 2>&1 | tee ${NAME}.spatialjoin-evaluation.tsv
./spatialjoin-evaluation.py ${NAME} --combinations bcsdoi,Bcsdoi,BCsdoi,BCSdoi,BCSDoi,BCSdOi,BCSdoI --analyze total --minutes
```

For the commands needed to produce the other results from our original paper,
see sigspatial-reproducibility/README.md.
