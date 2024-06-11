# Reproducibility materials for SIGSPATIAL'24 submission 192

Here is the link to the [PDF of the
paper](https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBKL_2024.pdf).

## Datasets

Here are the links to the datasets used in the Evaluation in Section 4. Each
dataset is provided as a TSV file with two columns, with one geometry per line
(ID in the first column, WKT in the second column).

Datasets from Section 4.2 (and Table 2):
[OHM](https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBKL_2024.materials/OHM.tsv),
[FIN](https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBKL_2024.materials/FIN.tsv),
[GER](https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBKL_2024.materials/GER.tsv),
[OSM](https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBKL_2024.materials/OSM.tsv.bz2).

Datasets from Section 4.3.2:
[restaurants](https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBKL_2024.materials/restaurants.tsv),
[transit stops](https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBKL_2024.materials/transit-stops.tsv),
[residential streets](https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBKL_2024.materials/residential-streets.tsv.bz2),
[administrative regions](https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBKL_2024.materials/administrative-regions.tsv),
[powerlines](https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBKL_2024.materials/powerlines.tsv).


## Comparison of our standalone tool (Section 4.2)

### Preparing data for spatialjoin

```
wget https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBKL_2024.materials/OHM.tsv -o OHM.spatialjoin-input.tsv
wget https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBKL_2024.materials/FIN.tsv -o FIN.spatialjoin-input.tsv
wget https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBKL_2024.materials/GER.tsv -o GET.spatialjoin-input.tsv
wget https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBKL_2024.materials/OSM.tsv.bz2 -o OSM.spatialjoin-input.tsv.bz2
bunzip2 OSM.spatialjoin.input.tsv.bz2
```

### Build and install spatialjoin

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

### Running the evalation and analyzing the results for Table 3

For each of the datasets `OHM, FIN, GER` and `OSM`, run the evaluation script in folder `scripts` as follows:

```
./scripts/spatialjoin-evaluation.py $DATASET --combinations bcsdoi,Bcsdoi,BCsdoi,BCSdoi,BCSDoi,BCSdOi,BCSdoI 2>&1 | tee $DATASET.spatialjoin-evaluation.tsv
./scripts/spatialjoin-evaluation.py $DATASET --combinations bcsdoi,Bcsdoi,BCsdoi,BCSdoi,BCSDoi,BCSdOi,BCSdoI --analyze total --minutes
```

## Comparison against PostgreSQL+PostGIS (Section 4.3)

### Preparing data for spatialjoin

To mark a non-selfjoin spatialjoin expects the data of the second side to be marked with a non-zero value.
To create the duplicate dataset we replace `\t` with `\t1\t` and generate `.1` variants of each dataset.

```
cat restaurants.tsv | sed 's/\t/\t1\t/' > restaurants.1.tsv
cat transit-stops.tsv | sed 's/\t/\t1\t/' > transit-stops.1.tsv
cat residential-streets.tsv | sed 's/\t/\t1\t/' > residential-streets.1.tsv
cat administrative-regions.tsv | sed 's/\t/\t1\t/' > administrative-regions.1.tsv
cat powerlines.tsv | sed 's/\t/\t1\t/' > powerlines.1.tsv
```

### Preparing data into PostgreSQL+PostGIS

#### Install PostgreSQL 16 with PostGIS 3.4.2

Ubuntu 22.04 (`jammy`) requires additional package sources.

##### PostgreSQL 16
```
apt install gnupg2 curl
echo "deb http://apt.postgresql.org/pub/repos/apt jammy-pgdg main" > /etc/apt/sources.list.d/pgdg.list
curl -fsSL https://www.postgresql.org/media/keys/ACCC4CF8.asc | gpg --dearmor -o /etc/apt/trusted.gpg.d/postgresql.gpg
apt update
apt install postgresql-16 postgresql-contrib-16
```

##### PostGIS 3.4.2
```
apt install software-properties-common
add-apt-repository ppa:ubuntugis/ppa
apt update
apt install postgresql-16-postgis-3
```

#### Create Database and enable PostGIS

```
createdb spatialjoin_db
psql -d spatialjoin_db -c "CREATE EXTENSION postgis;"
```

#### Create tables and geom index


```
psql -d spatialjoin_db -c "CREATE TABLE IF NOT EXISTS ohm_planet (id VARCHAR PRIMARY KEY, geom GEOMETRY);"
psql -d spatialjoin_db -c "CREATE TABLE IF NOT EXISTS ohm_planet_loader (id VARCHAR, geom_text VARCHAR);"
psql -d spatialjoin_db -c "\copy ohm_planet_loader FROM '/path/to/OHM.tsv' WITH (FORMAT csv, DELIMITER E'\t', HEADER false);"
psql -d spatialjoin_db -c "INSERT INTO ohm_planet (id, geom) SELECT id, ST_GeomFromText(geom_text, 4326) FROM ohm_planet_loader;"
psql -d spatialjoin_db -c "DROP table ohm_planet_loader;"
psql -d spatialjoin_db -c "CREATE INDEX ohm_planet_geom_idx ON ohm_planet USING GIST (geom);"
```
```
psql -d spatialjoin_db -c "CREATE TABLE IF NOT EXISTS osm_finland (id VARCHAR PRIMARY KEY, geom GEOMETRY);"
psql -d spatialjoin_db -c "CREATE TABLE IF NOT EXISTS osm_finland_loader (id VARCHAR, geom_text VARCHAR);"
psql -d spatialjoin_db -c "\copy osm_finland_loader FROM '/path/to/FIN.tsv' WITH (FORMAT csv, DELIMITER E'\t', HEADER false);"
psql -d spatialjoin_db -c "INSERT INTO osm_finland (id, geom) SELECT id, ST_GeomFromText(geom_text, 4326) FROM osm_finland_loader;"
psql -d spatialjoin_db -c "DROP table osm_finland_loader;"
psql -d spatialjoin_db -c "CREATE INDEX osm_finland_geom_idx ON osm_finland USING GIST (geom);"
```
```
psql -d spatialjoin_db -c "CREATE TABLE IF NOT EXISTS osm_germany (id VARCHAR PRIMARY KEY, geom GEOMETRY);"
psql -d spatialjoin_db -c "CREATE TABLE IF NOT EXISTS osm_germany_loader (id VARCHAR, geom_text VARCHAR);"
psql -d spatialjoin_db -c "\copy osm_germany_loader FROM '/path/to/GER.tsv' WITH (FORMAT csv, DELIMITER E'\t', HEADER false);"
psql -d spatialjoin_db -c "INSERT INTO osm_germany (id, geom) SELECT id, ST_GeomFromText(geom_text, 4326) FROM osm_germany_loader;"
psql -d spatialjoin_db -c "DROP table osm_germany_loader;"
psql -d spatialjoin_db -c "CREATE INDEX osm_germany_geom_idx ON osm_germany USING GIST (geom);"
```
```
psql -d spatialjoin_db -c "CREATE TABLE IF NOT EXISTS osm_planet (id VARCHAR PRIMARY KEY, geom GEOMETRY);"
psql -d spatialjoin_db -c "CREATE TABLE IF NOT EXISTS osm_planet_loader (id VARCHAR, geom_text VARCHAR);"
psql -d spatialjoin_db -c "\copy osm_planet_loader FROM '/path/to/OSM.tsv' WITH (FORMAT csv, DELIMITER E'\t', HEADER false);"
psql -d spatialjoin_db -c "INSERT INTO osm_planet (id, geom) SELECT id, ST_GeomFromText(geom_text, 4326) FROM osm_planet_loader;"
psql -d spatialjoin_db -c "DROP table osm_planet_loader;"
psql -d spatialjoin_db -c "CREATE INDEX osm_planet_geom_idx ON osm_planet USING GIST (geom);"
```
```
psql -d spatialjoin_db -c "CREATE TABLE IF NOT EXISTS restaurants (id VARCHAR PRIMARY KEY, geom GEOMETRY);"
psql -d spatialjoin_db -c "CREATE TABLE IF NOT EXISTS restaurants_loader (id VARCHAR, geom_text VARCHAR);"
psql -d spatialjoin_db -c "\copy restaurants_loader FROM '/path/to/restaurants.tsv' WITH (FORMAT csv, DELIMITER E'\t', HEADER false);"
psql -d spatialjoin_db -c "INSERT INTO restaurants (id, geom) SELECT id, ST_GeomFromText(geom_text, 4326) FROM restaurants_loader;"
psql -d spatialjoin_db -c "DROP table restaurants_loader;"
psql -d spatialjoin_db -c "CREATE INDEX restaurants_geom_idx ON restaurants USING GIST (geom);"
```
```
psql -d spatialjoin_db -c "CREATE TABLE IF NOT EXISTS transit_stops (id VARCHAR PRIMARY KEY, geom GEOMETRY);"
psql -d spatialjoin_db -c "CREATE TABLE IF NOT EXISTS transit_stops_loader (id VARCHAR, geom_text VARCHAR);"
psql -d spatialjoin_db -c "\copy transit_stops_loader FROM '/path/to/transit-stops.tsv' WITH (FORMAT csv, DELIMITER E'\t', HEADER false);"
psql -d spatialjoin_db -c "INSERT INTO transit_stops (id, geom) SELECT id, ST_GeomFromText(geom_text, 4326) FROM transit_stops_loader;"
psql -d spatialjoin_db -c "DROP table transit_stops_loader;"
psql -d spatialjoin_db -c "CREATE INDEX transit_stops_geom_idx ON transit_stops USING GIST (geom);"
```
```
psql -d spatialjoin_db -c "CREATE TABLE IF NOT EXISTS residential_streets (id VARCHAR PRIMARY KEY, geom GEOMETRY);"
psql -d spatialjoin_db -c "CREATE TABLE IF NOT EXISTS residential_streets_loader (id VARCHAR, geom_text VARCHAR);"
psql -d spatialjoin_db -c "\copy residential_streets_loader FROM '/path/to/residential-streets.tsv' WITH (FORMAT csv, DELIMITER E'\t', HEADER false);"
psql -d spatialjoin_db -c "INSERT INTO residential_streets (id, geom) SELECT id, ST_GeomFromText(geom_text, 4326) FROM residential_streets_loader;"
psql -d spatialjoin_db -c "DROP table residential_streets_loader;"
psql -d spatialjoin_db -c "CREATE INDEX restaurants_geom_idx ON residential_streets USING GIST (geom);"
```
```
psql -d spatialjoin_db -c "CREATE TABLE IF NOT EXISTS administrative_regions (id VARCHAR PRIMARY KEY, geom GEOMETRY);"
psql -d spatialjoin_db -c "CREATE TABLE IF NOT EXISTS administrative_regions_loader (id VARCHAR, geom_text VARCHAR);"
psql -d spatialjoin_db -c "\copy administrative_regions_loader FROM '/path/to/restaurants.tsv' WITH (FORMAT csv, DELIMITER E'\t', HEADER false);"
psql -d spatialjoin_db -c "INSERT INTO administrative_regions (id, geom) SELECT id, ST_GeomFromText(geom_text, 4326) FROM administrative_regions_loader;"
psql -d spatialjoin_db -c "DROP table administrative_regions_loader;"
psql -d spatialjoin_db -c "CREATE INDEX administrative_regions_geom_idx ON administrative_regions USING GIST (geom);"
```
```
psql -d spatialjoin_db -c "CREATE TABLE IF NOT EXISTS powerlines (id VARCHAR PRIMARY KEY, geom GEOMETRY);"
psql -d spatialjoin_db -c "CREATE TABLE IF NOT EXISTS powerlines_loader (id VARCHAR, geom_text VARCHAR);"
psql -d spatialjoin_db -c "\copy powerlines_loader FROM '/path/to/powerlines.tsv' WITH (FORMAT csv, DELIMITER E'\t', HEADER false);"
psql -d spatialjoin_db -c "INSERT INTO powerlines (id, geom) SELECT id, ST_GeomFromText(geom_text, 4326) FROM powerlines_loader;"
psql -d spatialjoin_db -c "DROP table powerlines_loader;"
psql -d spatialjoin_db -c "CREATE INDEX powerlines_geom_idx ON powerlines USING GIST (geom);"
```

### Queries used for data generation

#### Queries used for Table 2

##### PostgreSQL+PostGIS
```
# candidates
psql -d spatialjoin_db -c "\timing" -c "SELECT COUNT(*) FROM ohm_planet a, ohm_planet b WHERE a.geom && b.geom;"
# intersects
psql -d spatialjoin_db -c "\timing" -c "SELECT COUNT(*) FROM ohm_planet a, ohm_planet b WHERE ST_Intersects(a.geom, b.geom);"
```
```
# candidates
psql -d spatialjoin_db -c "\timing" -c "SELECT COUNT(*) FROM osm_finland a, osm_finland b WHERE a.geom && b.geom;"
# intersects
psql -d spatialjoin_db -c "\timing" -c "SELECT COUNT(*) FROM osm_finland a, osm_finland b WHERE ST_Intersects(a.geom, b.geom);"
```
```
# candidates
psql -d spatialjoin_db -c "\timing" -c "SELECT COUNT(*) FROM osm_germany a, osm_germany b WHERE a.geom && b.geom;"
# intersects
psql -d spatialjoin_db -c "\timing" -c "SELECT COUNT(*) FROM osm_germany a, osm_germany b WHERE ST_Intersects(a.geom, b.geom);"
```
```
# candidates
psql -d spatialjoin_db -c "\timing" -c "SELECT COUNT(*) FROM osm_planet a, osm_planet b WHERE a.geom && b.geom;"
# intersects
psql -d spatialjoin_db -c "\timing" -c "SELECT COUNT(*) FROM osm_planet a, osm_planet b WHERE ST_Intersects(a.geom, b.geom);"
```

##### spatialjoin
```
# candidates
spatialjoin --num-threads 2 --no-geometry-checks --no-diag-box -o /dev/null < /path/to/OHM.tsv
# all predicates
spatialjoin --num-threads 2 -o /dev/null < /path/to/OHM.tsv
```
```
# candidates
spatialjoin --num-threads 2 --no-geometry-checks --no-diag-box -o /dev/null < /path/to/FIN.tsv
# all predicates
spatialjoin --num-threads 2 -o /dev/null < /path/to/FIN.tsv
```
```
# candidates
spatialjoin --num-threads 2 --no-geometry-checks --no-diag-box -o /dev/null < /path/to/GER.tsv
# all predicates
spatialjoin --num-threads 2 -o /dev/null < /path/to/GER.tsv
```
```
# candidates
spatialjoin --num-threads 2 --no-geometry-checks --no-diag-box -o /dev/null < /path/to/OSM.tsv
# all predicates
spatialjoin --num-threads 2 -o /dev/null < /path/to/OSM.tsv
```

#### Queries used for Table 5

##### PostgreSQL+PostGIS
```
# candidates
psql -d spatialjoin_db -c "\timing" -c "SELECT COUNT(*) FROM restaurants a, transit_stops b WHERE a.geom && b.geom;"
# intersects
psql -d spatialjoin_db -c "\timing" -c "SELECT COUNT(*) FROM restaurants a, transit_stops b WHERE ST_Intersects(a.geom, b.geom);"
```
```
# candidates
psql -d spatialjoin_db -c "\timing" -c "SELECT COUNT(*) FROM residential_streets a, administrative_regions b WHERE a.geom && b.geom;"
# intersects
psql -d spatialjoin_db -c "\timing" -c "SELECT COUNT(*) FROM residential_streets a, administrative_regions b WHERE ST_Intersects(a.geom, b.geom);"
```
```
# candidates
psql -d spatialjoin_db -c "\timing" -c "SELECT COUNT(*) FROM residential_streets a, residential_streets b WHERE a.geom && b.geom;"
# intersects
psql -d spatialjoin_db -c "\timing" -c "SELECT COUNT(*) FROM residential_streets a, residential_streets b WHERE ST_Intersects(a.geom, b.geom);"
```
```
# candidates
psql -d spatialjoin_db -c "\timing" -c "SELECT COUNT(*) FROM powerlines a, residential_streets b WHERE a.geom && b.geom;"
# intersects
psql -d spatialjoin_db -c "\timing" -c "SELECT COUNT(*) FROM powerlines a, residential_streets b WHERE ST_Intersects(a.geom, b.geom);"
```

##### spatialjoin
```
# candidates
cat /path/to/restaurants.tsv /path/to/transit-stops.1.tsv | spatialjoin --num-threads 2 --no-geometry-checks --no-diag-box -o /dev/null
# all predicates
cat /path/to/restaurants.tsv /path/to/transit-stops.1.tsv | spatialjoin --num-threads 2 -o /dev/null 
```
```
# candidates
cat /path/to/residential-streets.tsv /path/to/administrative-regions.1.tsv | spatialjoin --num-threads 2 --no-geometry-checks --no-diag-box -o /dev/null
# all predicates
cat /path/to/residential-streets.tsv /path/to/administrative-regions.1.tsv | spatialjoin --num-threads 2 -o /dev/null 
```
```
# candidates
cat /path/to/residential-streets.tsv | spatialjoin --num-threads 2 --no-geometry-checks --no-diag-box -o /dev/null
# all predicates
cat /path/to/residential-streets.tsv | spatialjoin --num-threads 2 -o /dev/null 
```
```
# candidates
cat /path/to/powerlines.tsv /path/to/residential-streets.1.tsv | spatialjoin --num-threads 2 --no-geometry-checks --no-diag-box -o /dev/null
# all predicates
cat /path/to/powerlines.tsv /path/to/residential-streets.1.tsv | spatialjoin --num-threads 2 -o /dev/null 
```
