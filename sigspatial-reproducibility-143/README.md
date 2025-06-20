# Reproducibility materials for SIGSPATIAL'25 submission 143

## Datasets

Here are the links to the datasets used in the Evaluation in Section 4. Each
dataset is provided as a TSV file with two columns, with one geometry per line
(ID in the first column, WKT in the second column).

Datasets from Section 4.2 (and Tables 2 and 3):
[FIN](https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBK_2025.materials/FIN.spatialjoin-input.tsv),
[GER](https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBK_2025.materials/GER.spatialjoin-input.tsv),
[OHM](https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBK_2025.materials/OHM.spatialjoin-input.tsv),
[OSM](https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBK_2025.materials/OSM.spatialjoin-input.tsv).

Datasets for queries from Section 4.3.2 (and Table 4):
[Q1](https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBK_2025.materials/Q1.spatialjoin-input.tsv),
[Q2](https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBK_2025.materials/Q2.spatialjoin-input.tsv),
[Q3](https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBK_2025.materials/Q3.spatialjoin-input.tsv),
[Q4](https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBK_2025.materials/Q4.spatialjoin-input.tsv).


## Comparison of our standalone tool (Section 4.2)

### Build and install spatialjoin

Fetch the `use-libgeos-option` branch of this repository and init submodules:

```
git clone -b use-libgeos-option --recurse-submodules https://github.com/ad-freiburg/spatialjoin
```

Aftewards, build the tool:

```
cd spatialjoin
mkdir build && cd build
cmake ..
make -j
```

Then go into the sigspatial-reproducibilty-143 folder:
```
cd ..
cd sigspatial-reproducibilty-143
```

## Pre-flight checks

Run
```
make check
```
to check if your evaluation environment is correctly set up.


### Running the evaluation and analyzing the results for Table 3

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

For PostgreSQL, we use the official package for version 16 provided on [postgresql.org](https://postgresql.org), with the default configuration.

For PostGIS, we use the package for version 3.4.2 provided by UbuntuGIS, also with the default configuration.

#### Install PostgreSQL 16 with PostGIS 3.4.2

This installation guide is for Ubuntu 22.04. The process may be different for other distributions.

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

Change to user `postgres`:

```
su postgres
```

```
createdb spatialjoin_db
psql -d spatialjoin_db -c "CREATE EXTENSION postgis;"
```

#### Create tables and geom index

For each of the datasets `OHM`, `FIN`, `GER`, `OSM`, `restaurants`, `transit_stops`, `residential-streets`, `administrative-regions`, and `powerlines`, do the following, where `${DATASET}` is the dataset name, and `${DATASET/-/_}` is the dataset name with `-` replaced by `_`:


```
psql -d spatialjoin_db -c "CREATE TABLE IF NOT EXISTS ${DATASET/-/_} (id VARCHAR PRIMARY KEY, geom GEOMETRY);"
psql -d spatialjoin_db -c "CREATE TABLE IF NOT EXISTS ${DATASET/-/_}_loader (id VARCHAR, geom_text VARCHAR);"
psql -d spatialjoin_db -c "\copy ${DATASET/-/_}_loader FROM '${DATASET}.tsv' WITH (FORMAT csv, DELIMITER E'\t', HEADER false);"
psql -d spatialjoin_db -c "INSERT INTO ${DATASET/-/_} (id, geom) SELECT id, ST_GeomFromText(geom_text, 4326) FROM ${DATASET/-/_}_loader;"
psql -d spatialjoin_db -c "DROP table ${DATASET/-/_}_loader;"
psql -d spatialjoin_db -c "CREATE INDEX ${DATASET/-/_}_geom_idx ON ${DATASET/-/_} USING GIST (geom);"
```

### Queries used for table data generation

#### Queries used for Table 4

For each of the datasets `OHM`, `FIN`, `GER`, `OSM`, do the following, where `${DATASET}` is again the dataset name:

##### PostgreSQL+PostGIS

```
# Compute #candidates
psql -d spatialjoin_db -c "\timing" -c "SELECT COUNT(*) FROM ${DATASET} a, ${DATASET} b WHERE a.geom && b.geom;"

# Compute #results
psql -d spatialjoin_db -c "\timing" -c "SELECT COUNT(*) FROM ${DATASET} a, ${DATASET} b WHERE ST_Intersects(a.geom, b.geom);"
```

##### spatialjoin
```
# Compute #candidates
spatialjoin --num-threads 2 --no-geometry-checks --no-diag-box -o /dev/null < ${DATASET}.tsv
# Compute #results
spatialjoin --num-threads 2 -o /dev/null < ${DATASET}.tsv
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
cat restaurants.tsv transit-stops.1.tsv | spatialjoin --num-threads 2 --no-geometry-checks --no-diag-box -o /dev/null
# all predicates
cat restaurants.tsv transit-stops.1.tsv | spatialjoin --num-threads 2 -o /dev/null 
```
```
# candidates
cat residential-streets.tsv administrative-regions.1.tsv | spatialjoin --num-threads 2 --no-geometry-checks --no-diag-box -o /dev/null
# all predicates
cat residential-streets.tsv administrative-regions.1.tsv | spatialjoin --num-threads 2 -o /dev/null 
```
```
# candidates
cat residential-streets.tsv | spatialjoin --num-threads 2 --no-geometry-checks --no-diag-box -o /dev/null
# all predicates
cat residential-streets.tsv | spatialjoin --num-threads 2 -o /dev/null 
```
```
# candidates
cat powerlines.tsv residential-streets.1.tsv | spatialjoin --num-threads 2 --no-geometry-checks --no-diag-box -o /dev/null
# all predicates
cat powerlines.tsv residential-streets.1.tsv | spatialjoin --num-threads 2 -o /dev/null 
```
