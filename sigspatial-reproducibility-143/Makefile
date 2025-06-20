SPATIALJOIN_EVAL_SCRIPT = ../scripts/spatialjoin-evaluation.py
SPATIALJOIN = ../build/spatialjoin
SPATIALJOIN_ARGS = --num-threads 2 --num-caches 2 --no-oriented-envelope # BCSDoi with 2 threads

DATA_DIR = .

.PHONY: eval help eval-table-2 check

.SECONDARY:

help:
	@echo "spatialjoin evaluation script\n"
	@echo "\nGeneral targets\n"
	@echo "  make check\n                check PostgreSQL/PostGIS and spatialjoin installation"
	@echo "  make eval\n                 run entire evaluation"
	@echo "\nIndividual targets (<DATASET> either FIN, GER, OHM, OSM, Q1, Q2, Q4, Q4\n"
	@echo "  make table-2\n              compute results for Table 2"
	@echo "  make table-3\n              compute results for Table 3"
	@echo "  make table-4\n              compute results for Table 4"
	@echo "  make table-2-<DATASET>\n    compute results for <DATASET> in Table 2"
	@echo "  make table-3-<DATASET>\n    compute results for <DATASET> in Table 2"
	@echo "  make table-4-<DATASET>\n    compute results for <DATASET> in Table 2"

check:
	@echo -n "Data dir for files:           "
	@[ -d $(DATA_DIR) ] && realpath $(DATA_DIR) || echo $(DATA_DIR) "(NOT FOUND)"
	@echo -n "spatialjoin binary:           "
	@[ -f $(SPATIALJOIN) ] && realpath $(SPATIALJOIN) || echo $(SPATIALJOIN) "(NOT FOUND)"
	@echo -n "spatialjoin use:             "
	@echo "POINT(0 0)" | $(SPATIALJOIN) > /dev/null 2> /dev/null && echo " OK " || echo " NOT WORKING"
	@echo -n "spatialjoin libgeos support: "
	@echo "POINT(0 0)" | $(SPATIALJOIN) --libgeos > /dev/null 2> /dev/null && echo " OK " || echo " NOT WORKING"
	@echo -n "spatialjoin eval script:      "
	@[ -f $(SPATIALJOIN_EVAL_SCRIPT) ] && realpath $(SPATIALJOIN_EVAL_SCRIPT) || echo $(SPATIALJOIN_EVAL_SCRIPT) "(NOT FOUND)"
	@echo -n "spatialjoin eval script use: "
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) dummy --combinations BSdoI --only-show-commands --log-file __dummy_log > /dev/null 2> /dev/null && rm __dummy_log && echo " OK " || " NOT WORKING"

$(DATA_DIR)/%.raw.spatialjoin-input.tsv:
	@echo Downloading $*.spatialjoin-input.tsv
	@curl "https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBK_2025.materials/$*.spatialjoin-input.tsv" > $@

$(DATA_DIR)/%-OURS.spatialjoin-input.tsv: $(DATA_DIR)/%.raw.spatialjoin-input.tsv
	@ln -s $(DATA_DIR)/$*.raw.spatialjoin-input.tsv $(DATA_DIR)/$*-OURS.spatialjoin-input.tsv

$(DATA_DIR)/%-GEOS.spatialjoin-input.tsv: $(DATA_DIR)/%.raw.spatialjoin-input.tsv
	@ln -s $(DATA_DIR)/$*.raw.spatialjoin-input.tsv $(DATA_DIR)/$*-GEOS.spatialjoin-input.tsv

%-OURS.spatialjoin-evaluation.tsv: $(DATA_DIR)/%-OURS.spatialjoin-input.tsv
	@echo "*** Evaluating dataset" $* "on OURS"
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) $*-OURS --combinations bsdoi,Bsdoi,BSdoi,BSDoi,BSdOi,BSdoI --core-library ours 2>&1 | tee $*-OURS.spatialjoin-evaluation.tsv || echo "Run failed, see $*.spatialjoin-evaluation.log for details"

%-GEOS.spatialjoin-evaluation.tsv: $(DATA_DIR)/%-GEOS.spatialjoin-input.tsv
	@echo "*** Evaluating dataset" $* "on GEOS"
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) $*-GEOS --combinations bsdoi,Bsdoi,BSdoi,BSDoi,BSdOi,BSdoI --core-library libgeos 2>&1 | tee $*-OURS.spatialjoin-evaluation.tsv || echo "Run failed, see $*.spatialjoin-evaluation.log for details"

Q%-OURS.spatialjoin-evaluation.tsv: $(DATA_DIR)/Q%-OURS.spatialjoin-input.tsv
	@echo "*** Evaluating dataset" Q$* "on OURS"
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) Q$*-OURS --combinations BSDoi --core-library ours 2>&1 | tee Q$*-OURS.spatialjoin-evaluation.tsv || echo "Run failed, see $*.spatialjoin-evaluation.log for details"

Q%-GEOS.spatialjoin-evaluation.tsv: $(DATA_DIR)/Q%-GEOS.spatialjoin-input.tsv
	@echo "*** Evaluating dataset" Q$* "on GEOS"
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) Q$*-GEOS --combinations BSdoI --core-library libgeos 2>&1 | tee Q$*-GEOS.spatialjoin-evaluation.tsv || echo "Run failed, see $*.spatialjoin-evaluation.log for details"

eval-table2-%: %-OURS.spatialjoin-evaluation.tsv
	@echo "*** Table 2 entries (OURS) on dataset" $*
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) $*-OURS --combinations bsdoi,Bsdoi,BSdoi,BSDoi,BSdOi,BSdoI --analyze total --minutes

eval-table3-%: %-GEOS.spatialjoin-evaluation.tsv
	@echo "*** Table 3 entries (GEOS) on dataset" $*
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) $*-GEOS --combinations bsdoi,Bsdoi,BSdoi,BSDoi,BSdOi,BSdoI --analyze total --minutes

eval-table4-ours-%: %-OURS.spatialjoin-evaluation.tsv
	@echo "*** Table 4 entries (OURS) on dataset" $*
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) $*-OURS --combinations BSDoi --analyze total --minutes

eval-table4-geos-%: %-GEOS.spatialjoin-evaluation.tsv
	@echo "*** Table 4 entries (GEOS) on dataset" $*
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) $*-GEOS --combinations BSdoI --analyze total --minutes

eval-table2: eval-table2-FIN eval-table2-GER eval-table2-OHM eval-table2-OSM
eval-table3: eval-table2-FIN eval-table2-GER eval-table2-OHM eval-table2-OSM
eval-table4-ours: eval-table4-ours-Q1 eval-table4-ours-Q2 eval-table4-ours-Q3 eval-table4-ours-Q4
eval-table4-geos: eval-table4-geos-Q1 eval-table4-geos-Q2 eval-table4-geos-Q3 eval-table4-geos-Q4

eval: eval-table2 eval-table3 eval-table4-ours eval-table4-geos

clean:
	rm -f *.spatialjoin-evaluation.tsv
	rm -f *.spatialjoin-evaluation.log
