POSTGRES_USER = postgres
POSTGRES_DB = spatialjoin_db
SPATIALJOIN_EVAL_SCRIPT = spatialjoin-evaluation.py
SPATIALJOIN = spatialjoin
SPATIALJOIN_ARGS = --num-threads 2 --num-caches 2 --no-oriented-envelope # BCSDoi with 2 threads
POSTGRES_TIMEOUT = 10h

DATA_DIR = .

QUERY_1_POSTGRES = SELECT COUNT(*) FROM classes AS a, classes AS b WHERE a.class = 'highway' AND b.id = 'rel:2171347' AND ST_Contains(b.geom, a.geom)
QUERY_2_POSTGRES = SELECT COUNT(*) FROM classes AS a, classes AS b WHERE a.class = 'highway' AND b.id = 'rel:51477' AND ST_Contains(b.geom, a.geom)
QUERY_3_POSTGRES = SELECT COUNT(*) FROM classes AS a, classes AS b WHERE a.class = 'building' AND b.class = 'power' AND b.type = 'line' AND ST_Intersects(a.geom, b.geom)
QUERY_4_POSTGRES = SELECT COUNT(*) FROM classes AS a, classes AS b WHERE a.class = 'highway' AND b.class = 'highway' AND a.type = 'residential' AND b.type = 'residential' AND ST_Intersects(a.geom, b.geom)

# TODO: QUERY 5, Number of postboxes by country

.PHONY: eval help tables check

.PRECIOUS: %.tsv $.tsv.gz $(DATADIR)/%.tsv $(DATADIR)/$.tsv.gz

.SECONDEXPANSION:

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
	@echo -n "Data dir for exports:       "
	@realpath $(DATA_DIR)
	@echo -n "PostgreSQL user:            "
	@echo $(POSTGRES_USER)
	@echo -n "PostgreSQL database:        "
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
	@echo -n "spatialjoin eval script:    "
	@[ -f $(SPATIALJOIN_EVAL_SCRIPT) ] && realpath $(SPATIALJOIN_EVAL_SCRIPT) || echo " NOT FOUND"

$(DATA_DIR)/region-osm-planet.tsv.gz:
	curl -s https://qlever.cs.uni-freiburg.de/api/osm-planet -H "Accept: text/csv" -H "Content-type: application/sparql-query" --data "PREFIX geo: <http://www.opengis.net/ont/geosparql#> PREFIX ogc: <http://www.opengis.net/rdf#> PREFIX osmrel: <https://www.openstreetmap.org/relation/> SELECT ?osm_id ?geometry WHERE { ?osm_id geo:hasGeometry/geo:asWKT ?geometry }" | sed 's/,/\t/;s|https://www.openstreetmap.org/|osm|;s|/|:|;s/"//g' | gzip -1 > $@

$(DATA_DIR)/region-freiburg.tsv.gz:
	curl -s https://qlever.cs.uni-freiburg.de/api/osm-planet -H "Accept: text/csv" -H "Content-type: application/sparql-query" --data "PREFIX geo: <http://www.opengis.net/ont/geosparql#> PREFIX ogc: <http://www.opengis.net/rdf#> PREFIX osmrel: <https://www.openstreetmap.org/relation/> SELECT ?osm_id ?geometry WHERE { osmrel:62768 ogc:sfContains ?osm_id . ?osm_id geo:hasGeometry/geo:asWKT ?geometry }" | sed 's/,/\t/;s|https://www.openstreetmap.org/|osm|;s|/|:|;s/"//g' | gzip -1 > $@

$(DATA_DIR)/region-finland.tsv.gz:
	curl -s https://qlever.cs.uni-freiburg.de/api/osm-planet -H "Accept: text/csv" -H "Content-type: application/sparql-query" --data "PREFIX geo: <http://www.opengis.net/ont/geosparql#> PREFIX ogc: <http://www.opengis.net/rdf#> PREFIX osmrel: <https://www.openstreetmap.org/relation/> SELECT ?osm_id ?geometry WHERE { osmrel:54224 ogc:sfContains ?osm_id . ?osm_id geo:hasGeometry/geo:asWKT ?geometry }" | sed 's/,/\t/;s|https://www.openstreetmap.org/|osm|;s|/|:|;s/"//g' | gzip -1 > $@

$(DATA_DIR)/region-germany.tsv.gz:
	curl -s https://qlever.cs.uni-freiburg.de/api/osm-planet -H "Accept: text/csv" -H "Content-type: application/sparql-query" --data "PREFIX geo: <http://www.opengis.net/ont/geosparql#> PREFIX ogc: <http://www.opengis.net/rdf#> PREFIX osmrel: <https://www.openstreetmap.org/relation/> SELECT ?osm_id ?geometry WHERE { osmrel:51477 ogc:sfContains ?osm_id . ?osm_id geo:hasGeometry/geo:asWKT ?geometry }" | sed 's/,/\t/;s|https://www.openstreetmap.org/|osm|;s|/|:|;s/"//g' | gzip -1 > $@

$(DATA_DIR)/region-ohm-planet.tsv.gz:
	curl -s https://qlever.cs.uni-freiburg.de/api/ohm-planet -H "Accept: text/csv" -H "Content-type: application/sparql-query" --data "PREFIX geo: <http://www.opengis.net/ont/geosparql#> PREFIX ogc: <http://www.opengis.net/rdf#> PREFIX osmrel: <https://www.openstreetmap.org/relation/> SELECT ?osm_id ?geometry WHERE { ?osm_id geo:hasGeometry/geo:asWKT ?geometry }" | sed 's/,/\t/;s|https://www.openstreetmap.org/|osm|;s|/|:|;s/"//g' | gzip -1 > $@

region-%-table: $(DATA_DIR)/region-%.tsv.gz
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "CREATE TABLE IF NOT EXISTS \"region-$*\" (id VARCHAR PRIMARY KEY, geom GEOMETRY);"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "CREATE TABLE IF NOT EXISTS \"region-$*_loader\" (id VARCHAR, geom_text VARCHAR);"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "DELETE FROM \"region-$*\";"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "DELETE FROM \"region-$*_loader\";"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "\copy \"region-$*_loader\" FROM PROGRAM 'gzip -dc $(shell realpath $^)' WITH (FORMAT csv, DELIMITER E'\t', HEADER true);"
	@# filter invalid single-point LINESTRINGs here, they are still present in the old OHM QLever instance
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "INSERT INTO \"region-$*\" (id, geom) SELECT id, ST_GeomFromText(geom_text, 4326) FROM \"region-$*_loader\" WHERE NOT starts_with(geom_text, 'LINESTRING') OR POSITION(',' IN geom_text) > 0;"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "DROP table \"region-$*_loader\";"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "SELECT COUNT(*) FROM \"region-$*\";"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "CREATE INDEX IF NOT EXISTS \"region-$*_geom_idx\" ON \"region-$*\" USING GIST (geom);"

$(DATA_DIR)/class-%.tsv.gz:
	curl -s https://qlever.cs.uni-freiburg.de/api/osm-planet -H "Accept: text/csv" -H "Content-type: application/sparql-query" --data "PREFIX osm: <https://www.openstreetmap.org/> PREFIX geo: <http://www.opengis.net/ont/geosparql#> PREFIX ogc: <http://www.opengis.net/rdf#> PREFIX osmrel: <https://www.openstreetmap.org/relation/> PREFIX osmkey: <https://www.openstreetmap.org/wiki/Key:> SELECT (REPLACE(REPLACE(STR(?osm_id_), STR(osm:), \"osm\"), \"/\", \":\") AS ?osm_id) (REPLACE(STR(osmkey:$*), STR(osmkey:), \"\") AS ?predicate) ?type ?geometry WHERE { { SELECT ?osm_id_ (SAMPLE(?type_) AS ?type) WHERE { ?osm_id_ osmkey:$* ?type_ } GROUP BY ?osm_id_ } ?osm_id_ geo:hasGeometry/geo:asWKT ?geometry }" | sed 's/,/\t/g;s|https://www.openstreetmap.org/|osm|;s|/|:|;s/"//g' | sed 's/"//g;s/\^\^<http[^\t]*>$$//' | gzip -1 > $@

classes-table: $(DATA_DIR)/class-building.tsv.gz $(DATA_DIR)/class-highway.tsv.gz $(DATA_DIR)/class-amenity.tsv.gz $(DATA_DIR)/class-power.tsv.gz
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "CREATE TABLE classes (id VARCHAR PRIMARY KEY, class VARCHAR, type VARCHAR, geom GEOMETRY);"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "CREATE TABLE classes_loader (id VARCHAR, class VARCHAR, type VARCHAR, geom_text VARCHAR);"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "COPY classes_loader FROM PROGRAM 'gzip -dc $(shell realpath $(DATA_DIR)/class-building.tsv.gz)' WITH (FORMAT csv, DELIMITER E'\t', HEADER true);"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "COPY classes_loader FROM PROGRAM 'gzip -dc $(shell realpath $(DATA_DIR)/class-highway.tsv.gz)' WITH (FORMAT csv, DELIMITER E'\t', HEADER true);"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "COPY classes_loader FROM PROGRAM 'gzip -dc $(shell realpath $(DATA_DIR)/class-amenity.tsv.gz)' WITH (FORMAT csv, DELIMITER E'\t', HEADER true);"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "COPY classes_loader FROM PROGRAM 'gzip -dc $(shell realpath $(DATA_DIR)/class-power.tsv.gz)' WITH (FORMAT csv, DELIMITER E'\t', HEADER true);"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "INSERT INTO classes (id, class, type, geom) SELECT DISTINCT ON (id) id, class, type, ST_GeomFromText(geom_text, 4326) FROM classes_loader;"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "DROP table classes_loader;"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "SELECT COUNT(*) FROM classes;"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "CREATE INDEX classes_geom_idx ON classes USING GIST (geom);"

$(DATA_DIR)/static-residential-streets.tsv.gz:
	curl https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBKL_2024.materials/residential-streets.tsv.bz2 | bunzip2 -c | gzip -1 > $@

$(DATA_DIR)/static-%.1.tsv.gz:
	curl https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBKL_2024.materials/residential-streets.tsv.bz2 | bunzip2 -c | sed 's/\t/\t1\t/' | gzip -1 > $@

$(DATA_DIR)/static-%.tsv.gz:
	curl https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBKL_2024.materials/$*.tsv | gzip -1 > $@

$(DATA_DIR)/static-%.1.tsv.gz:
	curl https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBKL_2024.materials/$*.tsv | sed 's/\t/\t1\t/' | gzip -1 > $@

static-%-table: $(DATA_DIR)/static-%.tsv.gz
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "CREATE TABLE IF NOT EXISTS \"static-$*\" (id VARCHAR PRIMARY KEY, geom GEOMETRY);"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "CREATE TABLE IF NOT EXISTS \"static-$*_loader\" (id VARCHAR, geom_text VARCHAR);"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "DELETE FROM \"static-$*\";"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "DELETE FROM \"static-$*_loader\";"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "\copy \"static-$*_loader\" FROM PROGRAM 'gzip -dc $(shell realpath $^)' WITH (FORMAT csv, DELIMITER E'\t', HEADER true);"
	@# filter invalid single-point LINESTRINGs here, they are still present in the old OHM QLever instance
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "INSERT INTO \"static-$*\" (id, geom) SELECT id, ST_GeomFromText(geom_text, 4326) FROM \"static-$*_loader\" WHERE NOT starts_with(geom_text, 'LINESTRING') OR POSITION(',' IN geom_text) > 0;"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "DROP table \"static-$*_loader\";"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "SELECT COUNT(*) FROM \"static-$*\";"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "CREATE INDEX IF NOT EXISTS \"static-$*_geom_idx\" ON \"static-$*\" USING GIST (geom);"

eval-self-join-%-postgres:
	@echo
	@echo ++ Starting postgres full self-join evaluation for \'$*\':
	@psql -q -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "SELECT FROM \"$*\" LIMIT 1" > /dev/null 2>&1 || (echo "ERROR: Table $* does not yet exist, run 'make $*-table' first\\n";false)
	@echo Postgres full self-join candidates for \'$*\':
	@psql -q -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "\timing" -c "SET statement_timeout = '$(POSTGRES_TIMEOUT)'; SELECT COUNT(*)::text || ' rows retrieved' FROM \"$*\" AS a, \"$*\" AS b WHERE a.geom && b.geom;" || true
	@echo Postgres full self-join on ST_Intersects for \'$*\':
	@psql -q -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "\timing" -c "SET statement_timeout = '$(POSTGRES_TIMEOUT)'; SELECT COUNT(*)::text || ' rows retrieved' FROM \"$*\" AS a, \"$*\" AS b WHERE ST_Intersects(a.geom, b.geom);" || true

eval-self-join-%-spatialjoin: $(DATA_DIR)/%.spatialjoin-input.tsv
	@echo
	@echo ++ Starting spatialjoin full self-join evaluation for \'$*\':
	@echo spatialjoin full self-join candidates for \'$*\':
	@./$(SPATIALJOIN) $(SPATIALJOIN_ARGS) --no-geometry-checks < $< > /dev/null 2> .spatialjoin-$*.log
	@grep "Total predicate generation time" .spatialjoin-$*.log | sed "s/.* Total predicate generation time (without parsing): \([0-9s\.]*\)/\1/g"
	@echo spatialjoin full self-join for \'$*\':
	@./$(SPATIALJOIN) $(SPATIALJOIN_ARGS) < $< > /dev/null 2> .spatialjoin-$*.log
	@grep "Total predicate generation time" .spatialjoin-$*.log | sed "s/.* Total predicate generation time (without parsing): \([0-9s\.]*\)/\1/g"
	@rm .spatialjoin-$*.log

%.spatialjoin-input.tsv: $(DATA_DIR)/%.tsv.gz
	((gzip -dc $< | head -n1 | wc -w | grep -q 4 && gzip -dc $< | cut -d'	' -f 1,4 | tail -n +2 | head) || (gzip -dc $< | tail -n +2))  > $@

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

eval-non-self-join-%-postgres:
	@echo
	@echo ++ Starting postgres evaluation for non-self join $(word 1,$(subst _, ,$*)) vs $(word 2,$(subst _, ,$*))
	@psql -q -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "SELECT FROM \"static-$(word 1,$(subst _, ,$*))\" LIMIT 1" > /dev/null 2>&1 || (echo "ERROR: Table static-$(word 1,$(subst _, ,$*)) does not yet exist, run 'make $(word 1,$(subst _, ,$*))-table' first\\n";false)
	@psql -q -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "SELECT FROM \"static-$(word 2,$(subst _, ,$*))\" LIMIT 1" > /dev/null 2>&1 || (echo "ERROR: Table static-$(word 2,$(subst _, ,$*)) does not yet exist, run 'make $(word 2,$(subst _, ,$*))-table' first\\n";false)
	@echo Postgres candidates for non-self join  $(word 1,$(subst _, ,$*)) vs $(word 2,$(subst _, ,$*)):
	@psql -q -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "\timing" -c "SET statement_timeout = '$(POSTGRES_TIMEOUT)'; SELECT COUNT(*)::text || ' rows retrieved' FROM \"static-$(word 1,$(subst _, ,$*))\" AS a, \"static-$(word 2,$(subst _, ,$*))\" AS b WHERE a.geom && b.geom;" || true
	@echo Postgres full ST_Intersects for non-self join  $(word 1,$(subst _, ,$*)) vs $(word 2,$(subst _, ,$*)):
	@psql -q -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "\timing" -c "SET statement_timeout = '$(POSTGRES_TIMEOUT)'; SELECT COUNT(*)::text || ' rows retrieved' FROM \"static-$(word 1,$(subst _, ,$*))\" AS a, \"static-$(word 2,$(subst _, ,$*))\" AS b WHERE ST_Intersects(a.geom, b.geom);" || true

eval-non-self-join-%-spatialjoin: $(DATA_DIR)/static-$$(word 1,$$(subst _, , %)).tsv.gz $(DATA_DIR)/static-$$(word 2,$$(subst _, , %)).1.tsv.gz
	@echo
	@echo ++ Starting spatialjoin evaluation for non-self join $(word 1,$(subst _, ,$*)) vs $(word 2,$(subst _, ,$*))
	@echo spatialjoin candidates for non-self join  $(word 1,$(subst _, ,$*)) vs $(word 2,$(subst _, ,$*)):
	@zcat $^ | ./$(SPATIALJOIN) $(SPATIALJOIN_ARGS) --no-geometry-checks > /dev/null 2> .spatialjoin-$*.log
	@grep "Total predicate generation time" .spatialjoin-$*.log | sed "s/.* Total predicate generation time (without parsing): \([0-9s\.]*\)/\1/g"
	@echo spatialjoin full non-self join  $(word 1,$(subst _, ,$*)) vs $(word 2,$(subst _, ,$*)):
	@zcat $^ | ./$(SPATIALJOIN) $(SPATIALJOIN_ARGS) > /dev/null 2> .spatialjoin-$*.log
	@grep "Total predicate generation time" .spatialjoin-$*.log | sed "s/.* Total predicate generation time (without parsing): \([0-9s\.]*\)/\1/g"
	@rm .spatialjoin-$*.log

eval-non-self-join-%: eval-non-self-join-%-spatialjoin eval-non-self-join-%-postgres
	@#

tables: region-freiburg-table region-germany-table region-finland-table region-ohm-planet-table region-osm-planet-table classes-table static-restaurants-table static-residential-streets-table static-powerlines-tables static-administrative-regions-table

eval-queries: eval-query-1 eval-query-2 eval-query-3 eval-query-4

eval-self-joins: eval-self-join-region-ohm-planet eval-selfjoin-region-finland eval-selfjoin-region-germany eval-selfjoin-region-osm-planet

eval-non-self-joins: eval-non-self-join-restaurants_transit-stops eval-non-self-join-residential-streets_administrative-regions eval-non-self-join-residential-streets_residential-streets eval-non-self-join-powerlines_residential-streets

eval: eval-combinations-region-osm-planet-spatialjoin eval-self-joins eval-non-self-joins eval-queries
