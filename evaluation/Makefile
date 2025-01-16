POSTGRES_USER = postgres
POSTGRES_DB = spatialjoin_db
SPATIALJOIN_EVAL_SCRIPT = spatialjoin-evaluation.py
SPATIALJOIN = ../build/spatialjoin

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
	@echo "  make eval-self-join-<DATASET>-spatial\n    run self-join evaluation on <DATASET> for spatialjoin"
	@echo "  make table-class\n    create a table 'classes' containing all objects of predefined classes"

check:
	@echo -n "PostgreSQL data directory:  "
	@psql -U $(POSTGRES_USER) -tA -c "SHOW data_directory;"
	@echo -n "PostgreSQL working memory:  "
	@psql -U $(POSTGRES_USER) -tA -c "SHOW work_mem;"
	@echo -n "PostgreSQL max processess:  "
	@psql -U $(POSTGRES_USER) -tA -c "SHOW max_worker_processes;"
	@echo -n "PostgreSQL max workers:     "
	@psql -U $(POSTGRES_USER) -tA -c "SHOW max_parallel_workers;"
	@echo -n "PostgreSQL version:         "
	@psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "SELECT version();"
	@echo -n "PostGIS version:            "
	@psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "SELECT PostGIS_Version();"
	@echo -n "spatialjoin eval script:   "
	@[ -f $(SPATIALJOIN_EVAL_SCRIPT) ] && realpath $(SPATIALJOIN_EVAL_SCRIPT) || echo " NOT FOUND"
	@echo -n "spatialjoin version:   "
	@$(SPATIALJOIN) --version

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
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "CREATE TABLE IF NOT EXISTS region_$* (id VARCHAR PRIMARY KEY, geom GEOMETRY);"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "CREATE TABLE IF NOT EXISTS region_$*_loader (id VARCHAR, geom_text VARCHAR);"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "DELETE FROM region_$*;"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "DELETE FROM region_$*_loader;"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "\copy region_$*_loader FROM '$(shell pwd)/$^' WITH (FORMAT csv, DELIMITER E'\t', HEADER true);"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "INSERT INTO region_$* (id, geom) SELECT id, ST_GeomFromText(geom_text, 4326) FROM region_$*_loader;"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "DROP table region_$*_loader;"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "SELECT COUNT(*) FROM region_$*;"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "CREATE INDEX IF NOT EXISTS region_$*_geom_idx ON region_$* USING GIST (geom);"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "\dt+ public.region_$**"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "\di+ public.region_$**"

class-%.tsv:
	curl -s https://qlever.cs.uni-freiburg.de/api/osm-planet -H "Accept: text/csv" -H "Content-type: application/sparql-query" --data "PREFIX osm: <https://www.openstreetmap.org/> PREFIX geo: <http://www.opengis.net/ont/geosparql#> PREFIX ogc: <http://www.opengis.net/rdf#> PREFIX osmrel: <https://www.openstreetmap.org/relation/> PREFIX osmkey: <https://www.openstreetmap.org/wiki/Key:> SELECT (REPLACE(REPLACE(STR(?osm_id_), STR(osm:), \"osm\"), \"/\", \":\") AS ?osm_id) (REPLACE(STR(osmkey:$*), STR(osmkey:), \"\") AS ?predicate) ?type ?geometry WHERE { { SELECT ?osm_id_ (SAMPLE(?type_) AS ?type) WHERE { ?osm_id_ osmkey:$* ?type_ } GROUP BY ?osm_id_ } ?osm_id_ geo:hasGeometry/geo:asWKT ?geometry }" | sed 's/,/\t/g;s|https://www.openstreetmap.org/|osm|;s|/|:|;s/"//g' | sed 's/"//g;s/\^\^<http[^\t]*>$$//' > $@

table-class: class-building.tsv class-highway.tsv class-amenity.tsv class-power.tsv
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
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "\dt+ public.classes*"
	psql -U $(POSTGRES_USER) -d $(POSTGRES_DB) -c "\di+ public.classes*"

eval-self-join-%-postgres:
	@echo Starting postgres full self-join evaluation for \'$*\':
	@psql -q -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "SELECT FROM $* LIMIT 1" > /dev/null 2>&1 || echo Table $* does not yet exist, run make $*-table first
	@echo Postgres full self-join candidates for \'$*\':
	@psql -q -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "\timing" -c "SELECT COUNT(*)::text || ' rows retrieved' FROM $* AS a, $* AS b WHERE a.geom && b.geom;"
	@echo Postgres full self-join on ST_Intersects for \'$*\':
	@psql -q -U $(POSTGRES_USER) -d $(POSTGRES_DB) -tA -c "\timing" -c "SELECT COUNT(*)::text || ' rows retrieved' FROM $* AS a, $* AS b WHERE ST_Intersects(a.geom, b.geom);"
	@echo Finished postgres full self-join evaluation for \'$*\':

eval-self-join-%-spatialjoin: %.spatialjoin-input.tsv
	$(SPATIALJOIN) < $< > /dev/null

%.spatialjoin-input.tsv: %.tsv
	((head $< -n1 | wc -w | grep -q 4 && cut -d'	' -f 1,4 $< | tail -n +2 | head) || (tail -n +2 $<))  > $@

eval-combinations-%-spatialjoin: %.spatialjoin-input.tsv
	$(SPATIALJOIN_EVAL_SCRIPT) $* --spatialjoin $(SPATIALJOIN) --combinations bcsdoi,Bcsdoi,BCsdoi,BCSdoi,BCSDoi,BCSdOi,BCSdoI 2>&1 | tee ${NAME}.spatialjoin-evaluation.tsv
	$(SPATIALJOIN_EVAL_SCRIPT) $* --spatialjoin $(SPATIALJOIN) --combinations bcsdoi,Bcsdoi,BCsdoi,BCSdoi,BCSDoi,BCSdOi,BCSdoI --analyze total --minutes


eval-self-join-%: eval-self-join-%-spatialjoin eval-self-join-%-postgres

eval: eval-self-join-region-ohm-planet eval-selfjoin-region-finland eval-selfjoin-region-germany eval-selfjoin-region-osm-planet
