POSTGRES_USER = postgres
POSTGRES_DB = spatialjoin_db
SPATIALJOIN_EVAL_SCRIPT = spatialjoin-evaluation.py
SPATIALJOIN = spatialjoin
SPATIALJOIN_ARGS = --num-threads 2 --num-caches 2 --no-oriented-envelope # BCSDoi with 2 threads
POSTGRES_TIMEOUT = 10h

QUERY_1_POSTGRES = SELECT COUNT(*) FROM classes AS a, classes AS b WHERE a.class = 'highway' AND b.id = 'rel:2171347' AND ST_Contains(b.geom, a.geom)
QUERY_2_POSTGRES = SELECT COUNT(*) FROM classes AS a, classes AS b WHERE a.class = 'highway' AND b.id = 'rel:51477' AND ST_Contains(b.geom, a.geom)
QUERY_3_POSTGRES = SELECT COUNT(*) FROM classes AS a, classes AS b WHERE a.class = 'building' AND b.class = 'power' AND b.type = 'line' AND ST_Intersects(a.geom, b.geom)
QUERY_4_POSTGRES = SELECT COUNT(*) FROM classes AS a, classes AS b WHERE a.class = 'highway' AND b.class = 'highway' AND a.type = 'residential' AND b.type = 'residential' AND ST_Intersects(a.geom, b.geom)

# TODO: QUERY 5, Number of postboxes by country

.PHONY: eval help tables check

.PRECIOUS: %.tsv

help:
	@echo "spatialjoin evaluation script\n"
	@echo "Supported datasets"
	@echo "  region-freiburg, region-finland, region-germany, region-ohm-planet, region-osm-planet"
	@echo "\nGeneral targets\n"
	@echo "  make check\n    check PostgreSQL/PostGIS and spatialjoin installation"
	@echo "  make eval\n    run entire evaluation"
	@echo "\nIndividual targets\n"
	@echo "  make <DATASET>-table\n    prepare PostGIS table for <DATASET>"
	@echo "  make eval-self-join-<DATASET>-postgres\n    run self-join evaluation on <DATASET> for PostGIS"
	@echo "  make eval-self-join-<DATASET>-spatialjoin\n    run self-join evaluation on <DATASET> for spatialjoin"
	@echo "  make classes-table\n    create a table 'classes' containing all objects of predefined classes"
	@echo "  make eval-query-<QUERYID>\n    evaluate query QUERYID (1,2, 3, 4, 5) against Postgres"
	@echo "  make eval-combinations-<DATASET>-spatialjoin\n    run self-evaluation for spatialjoin on <DATASET>"

check:
	@echo -n "PostgreSQL user             "
	@echo $(POSTGRES_USER)
	@echo -n "PostgreSQL database         "
	@echo $(POSTGRES_DB)
	@echo -n "PostgreSQL query timeout:   "
	@echo $(POSTGRES_TIMEOUT)
	@echo -n "PostgreSQL data directory:  "
	@psql -U $(POSTGRES_USER) -tA -c "SHOW data_directory;"
	@echo -n "PostgreSQL working memory:  "
	@psql -U $(POSTGRES_USER) -tA -c "SHOW work_mem;"
	@echo -n "PostgreSQL max processess:  "
	@psql -U $(POSTGRES_USER) -tA -c "SHOW max_worker_processes;"
	@echo -n "PostgreSQL max workers:     "
	@psql -U $(POSTGRES_USER) -tA -c "SHOW max_parallel_workers;"
	@echo -n "PostgreSQL version:         "
	@psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "SELECT version();" | cut -d ' ' -f2
	@echo -n "PostGIS version:            "
	@psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "SELECT PostGIS_Version();"
	@echo -n "spatialjoin binary:         "
	@echo $(SPATIALJOIN)
	@echo -n "spatialjoin version:        "
	@$(SPATIALJOIN) --version
	@echo -n "spatialjoin eval script:    "
	@[ -f $(SPATIALJOIN_EVAL_SCRIPT) ] && realpath $(SPATIALJOIN_EVAL_SCRIPT) || echo " NOT FOUND"

region-osm-planet.tsv:
	curl -s https://qlever.cs.uni-freiburg.de/api/osm-planet -H "Accept: text/csv" -H "Content-type: application/sparql-query" --data "PREFIX geo: <http://www.opengis.net/ont/geosparql#> PREFIX ogc: <http://www.opengis.net/rdf#> PREFIX osmrel: <https://www.openstreetmap.org/relation/> SELECT ?osm_id ?geometry WHERE { ?osm_id geo:hasGeometry/geo:asWKT ?geometry }" | sed 's/,/\t/;s|https://www.openstreetmap.org/|osm|;s|/|:|;s/"//g' > $@

region-freiburg.tsv:
	curl -s https://qlever.cs.uni-freiburg.de/api/osm-planet -H "Accept: text/csv" -H "Content-type: application/sparql-query" --data "PREFIX geo: <http://www.opengis.net/ont/geosparql#> PREFIX ogc: <http://www.opengis.net/rdf#> PREFIX osmrel: <https://www.openstreetmap.org/relation/> SELECT ?osm_id ?geometry WHERE { osmrel:62768 ogc:sfContains ?osm_id . ?osm_id geo:hasGeometry/geo:asWKT ?geometry }" | sed 's/,/\t/;s|https://www.openstreetmap.org/|osm|;s|/|:|;s/"//g' > $@

region-finland.tsv:
	curl -s https://qlever.cs.uni-freiburg.de/api/osm-planet -H "Accept: text/csv" -H "Content-type: application/sparql-query" --data "PREFIX geo: <http://www.opengis.net/ont/geosparql#> PREFIX ogc: <http://www.opengis.net/rdf#> PREFIX osmrel: <https://www.openstreetmap.org/relation/> SELECT ?osm_id ?geometry WHERE { osmrel:54224 ogc:sfContains ?osm_id . ?osm_id geo:hasGeometry/geo:asWKT ?geometry }" | sed 's/,/\t/;s|https://www.openstreetmap.org/|osm|;s|/|:|;s/"//g' > $@

region-germany.tsv:
	curl -s https://qlever.cs.uni-freiburg.de/api/osm-planet -H "Accept: text/csv" -H "Content-type: application/sparql-query" --data "PREFIX geo: <http://www.opengis.net/ont/geosparql#> PREFIX ogc: <http://www.opengis.net/rdf#> PREFIX osmrel: <https://www.openstreetmap.org/relation/> SELECT ?osm_id ?geometry WHERE { osmrel:51477 ogc:sfContains ?osm_id . ?osm_id geo:hasGeometry/geo:asWKT ?geometry }" | sed 's/,/\t/;s|https://www.openstreetmap.org/|osm|;s|/|:|;s/"//g' > $@

region-ohm-planet.tsv:
	curl -s https://qlever.cs.uni-freiburg.de/api/ohm-planet -H "Accept: text/csv" -H "Content-type: application/sparql-query" --data "PREFIX geo: <http://www.opengis.net/ont/geosparql#> PREFIX ogc: <http://www.opengis.net/rdf#> PREFIX osmrel: <https://www.openstreetmap.org/relation/> SELECT ?osm_id ?geometry WHERE { ?osm_id geo:hasGeometry/geo:asWKT ?geometry }" | sed 's/,/\t/;s|https://www.openstreetmap.org/|osm|;s|/|:|;s/"//g' > $@

region-%-table: region-%.tsv
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "CREATE TABLE IF NOT EXISTS \"region-$*\" (id VARCHAR PRIMARY KEY, geom GEOMETRY);"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "CREATE TABLE IF NOT EXISTS \"region-$*_loader\" (id VARCHAR, geom_text VARCHAR);"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "DELETE FROM \"region-$*\";"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "DELETE FROM \"region-$*_loader\";"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "\copy \"region-$*_loader\" FROM '$(shell pwd)/$^' WITH (FORMAT csv, DELIMITER E'\t', HEADER true);"
	@# filter invalid single-point LINESTRINGs here, they are still present in the old OHM QLever instance
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "INSERT INTO \"region-$*\" (id, geom) SELECT id, ST_GeomFromText(geom_text, 4326) FROM \"region-$*_loader\" WHERE NOT starts_with(geom_text, 'LINESTRING') OR POSITION(',' IN geom_text) > 0;"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "DROP table \"region-$*_loader\";"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "SELECT COUNT(*) FROM \"region-$*\";"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "CREATE INDEX IF NOT EXISTS \"region-$*_geom_idx\" ON \"region-$*\" USING GIST (geom);"

class-%.tsv:
	curl -s https://qlever.cs.uni-freiburg.de/api/osm-planet -H "Accept: text/csv" -H "Content-type: application/sparql-query" --data "PREFIX osm: <https://www.openstreetmap.org/> PREFIX geo: <http://www.opengis.net/ont/geosparql#> PREFIX ogc: <http://www.opengis.net/rdf#> PREFIX osmrel: <https://www.openstreetmap.org/relation/> PREFIX osmkey: <https://www.openstreetmap.org/wiki/Key:> SELECT (REPLACE(REPLACE(STR(?osm_id_), STR(osm:), \"osm\"), \"/\", \":\") AS ?osm_id) (REPLACE(STR(osmkey:$*), STR(osmkey:), \"\") AS ?predicate) ?type ?geometry WHERE { { SELECT ?osm_id_ (SAMPLE(?type_) AS ?type) WHERE { ?osm_id_ osmkey:$* ?type_ } GROUP BY ?osm_id_ } ?osm_id_ geo:hasGeometry/geo:asWKT ?geometry }" | sed 's/,/\t/g;s|https://www.openstreetmap.org/|osm|;s|/|:|;s/"//g' | sed 's/"//g;s/\^\^<http[^\t]*>$$//' > $@

classes-table: class-building.tsv class-highway.tsv class-amenity.tsv class-power.tsv
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "CREATE TABLE classes (id VARCHAR PRIMARY KEY, class VARCHAR, type VARCHAR, geom GEOMETRY);"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "CREATE TABLE classes_loader (id VARCHAR, class VARCHAR, type VARCHAR, geom_text VARCHAR);"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "COPY classes_loader FROM '$(shell pwd)/class-building.tsv' WITH (FORMAT csv, DELIMITER E'\t', HEADER true);"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "COPY classes_loader FROM '$(shell pwd)/class-highway.tsv' WITH (FORMAT csv, DELIMITER E'\t', HEADER true);"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "COPY classes_loader FROM '$(shell pwd)/class-amenity.tsv' WITH (FORMAT csv, DELIMITER E'\t', HEADER true);"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "COPY classes_loader FROM '$(shell pwd)/class-power.tsv' WITH (FORMAT csv, DELIMITER E'\t', HEADER true);"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "INSERT INTO classes (id, class, type, geom) SELECT DISTINCT ON (id) id, class, type, ST_GeomFromText(geom_text, 4326) FROM classes_loader;"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "DROP table classes_loader;"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "SELECT COUNT(*) FROM classes;"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "CREATE INDEX classes_geom_idx ON classes USING GIST (geom);"

%-table:
	@echo ERROR: Not a supported dataset: $*;false


eval-self-join-%-postgres:
	@echo
	@echo ++ Starting postgres full self-join evaluation for \'$*\':
	@psql -q -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "SELECT FROM \"$*\" LIMIT 1" > /dev/null 2>&1 || (echo "ERROR: Table $* does not yet exist, run 'make $*-table' first\\n";false)
	@echo Postgres full self-join candidates for \'$*\':
	@psql -q -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "\timing" -c "SET statement_timeout = '$(POSTGRES_TIMEOUT)'; SELECT COUNT(*)::text || ' rows retrieved' FROM \"$*\" AS a, \"$*\" AS b WHERE a.geom && b.geom;" || true
	@echo Postgres full self-join on ST_Intersects for \'$*\':
	@psql -q -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "\timing" -c "SET statement_timeout = '$(POSTGRES_TIMEOUT)'; SELECT COUNT(*)::text || ' rows retrieved' FROM \"$*\" AS a, \"$*\" AS b WHERE ST_Intersects(a.geom, b.geom);" || true

eval-self-join-%-spatialjoin: %.spatialjoin-input.tsv
	@echo
	@echo ++ Starting spatialjoin full self-join evaluation for \'$*\':
	@echo spatialjoin full self-join candidates for \'$*\':
	@./$(SPATIALJOIN) --no-geometry-checks < $< > /dev/null 2> .spatialjoin-$*.log
	@grep "Done sweeping" .spatialjoin-$*.log | sed "s/.* Done sweeping (\([0-9s\.]*\))\./\1/g"
	@echo spatialjoin full self-join for \'$*\':
	@./$(SPATIALJOIN) < $< > /dev/null 2> .spatialjoin-$*.log
	@grep "Done sweeping" .spatialjoin-$*.log | sed "s/.* Done sweeping (\([0-9s\.]*\))\./\1/g"
	@rm .spatialjoin-$*.log

%.spatialjoin-input.tsv: %.tsv
	((head $< -n1 | wc -w | grep -q 4 && cut -d'	' -f 1,4 $< | tail -n +2 | head) || (tail -n +2 $<))  > $@

eval-combinations-%-spatialjoin: %.spatialjoin-input.tsv
	@echo
	@echo ++ Starting spatialjoin self-evaluation for \'$*\':
	@./$(SPATIALJOIN_EVAL_SCRIPT) $* --spatialjoin $(SPATIALJOIN) --combinations bcsdoi,Bcsdoi,BCsdoi,BCSdoi,BCSDoi,BCSdOi,BCSdoI | tee $*.spatialjoin-evaluation.tsv
	@echo
	@echo ++ Analyzing spatialjoin self-evaluation for \'$*\':
	@./$(SPATIALJOIN_EVAL_SCRIPT) $* --spatialjoin $(SPATIALJOIN) --combinations bcsdoi,Bcsdoi,BCsdoi,BCSdoi,BCSDoi,BCSdOi,BCSdoI --analyze total --minutes

eval-self-join-%: eval-self-join-%-spatialjoin eval-self-join-%-postgres
	@echo

eval-query-%:
	@echo
	@echo ++ Starting postgres evaluation for query $*:
	@echo "(Query is: '$(QUERY_$*_POSTGRES)' )"
	@psql -q -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "SELECT FROM classes LIMIT 1" > /dev/null 2>&1 || (echo "ERROR: Table classes does not yet exist, run 'make classes-table' first\\n";false)
	@echo Postgres result size and time:
	@psql -q -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "\timing" -c "SET statement_timeout = '$(POSTGRES_TIMEOUT)'; $(QUERY_$*_POSTGRES);" || true

tables: region-freiburg-table region-germany-table region-finland-table region-ohm-planet-table region-osm-planet-table classes-table

eval: eval-combinations-region-osm-planet eval-self-join-region-ohm-planet eval-selfjoin-region-finland eval-selfjoin-region-germany eval-selfjoin-region-osm-planet
