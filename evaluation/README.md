# Evaluation instructions and results

We evaluated the performance of our spatial join and compared it against
PostgreSQL+PostGIS. In the following sections, we provide instructions and
results for the evaluation.

## Setup PostgreSQL and PostGIS on Ubuntu 24.04

Install the required packages:


```
sudo apt update
sudo apt install postgresql postgresql-contrib postgis postgresql-16-postgis-3 gdal-bin
```

Next, create a new database storage in a directory of your choice.
```
export POSTGIS_DIR=/local/data-ssd/postgis/spatialjoin
sudo mkdir -p ${POSTGIS_DIR} && sudo chown postgres:postgres ${POSTGIS_DIR}
sudo -u postgres /usr/lib/postgresql/16/bin/initdb -D ${POSTGIS_DIR}
sudo vim ${POSTGIS_DIR}/postgresql.conf
```
In the file `${POSTGIS_DIR}/postgresql.conf`, set the following:
```
work_mem = 4MB
max_worker_processes = 8
max_parallel_workers_per_gather = 2
max_parallel_workers = 8
```
Afterwards, restart Postgres with the selected database storage directory:

```
sudo su - postgres -c "/usr/lib/postgresql/16/bin/pg_ctl -D ${POSTGIS_DIR} -l logfile start"
```

## Create a database

Create a database `spatialjoin_db` and enable PostGIS.
```
sudo su - postgres -c "createdb spatialjoin_db"
psql -U postgres -d spatialjoin_db -c "CREATE EXTENSION postgis;"
```

## Install spatialjoin

Build the `spatialjoin` executable in this repository and include it in the `PATH`:

```
mkdir build && cd build
cmake ..
make -j
cd ..
export PATH=PATH:$(pwd)/build
```

## Run full evaluation using the provided Makefile

First, check if the PostgreSQL, PostGIS and spatialjoin installation works as expected:

```
make check
```

Afterwards, create the tables required for the evaluation. This will take a while. Note that this  will completely rebuild *all* tables every time.

```
make tables
```

Finally, run the complete evaluation:

```
make eval
```

You can change individual configuration parameters (listed on the top section of the Makefile) by setting them explicity, e.g. `make POSTGRES_USER=patrick POSTGRES_DB=eval tables`.

## Run individual evaluations using the provided Makefile

Run

```
make help
```

to get a list of available target.
