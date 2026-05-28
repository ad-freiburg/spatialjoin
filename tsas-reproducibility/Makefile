SPATIALJOIN_EVAL_SCRIPT = ../scripts/spatialjoin-evaluation.py
SPATIALJOIN = ../build/spatialjoin
SPATIALJOIN_ARGS = --num-threads 2 --num-caches 2 --no-oriented-envelope # BCSDoi with 2 threads

DATA_DIR = .

.PHONY: eval help eval-table-2 check

.SECONDARY:

help:
	@echo "spatialjoin evaluation script\n"
	@echo "\nGeneral targets\n"
	@echo "  make check\n     check PostgreSQL/PostGIS and spatialjoin installation"
	@echo "  make eval\n     run entire evaluation"
	@echo "\nIndividual targets (<DATASET> either FIN, GER, OHM, OSM, Q1, Q2, Q4, Q4\n"
	@echo "  make eval-table2\n     compute results for Table 2 (heuristic effects on OURS)"
	@echo "  make eval-table3\n     compute results for Table 3 (heuristic effects on GEOS)"
	@echo "  make eval-table4\n     compute results for Table 4 (heuristic effects on GEOS-PREPARED)"
	@echo "  make eval-table5\n     compute results for Table 5 (dist heuristic effects on OURS)"
	@echo "  make eval-table6\n     compute results for Table 6 (dist heuristic effects on GEOS)"
	@echo "  make eval-table7\n     compute results for Table 7 (dist heuristic effects on GEOS-PREPARED)"
	@echo "  make eval-table2-<DATASET>\n     compute results for <DATASET> in Table 2"
	@echo "  make eval-table3-<DATASET>\n     compute results for <DATASET> in Table 3"
	@echo "  make eval-table8-ours-<DATASET>\n     compute results for <DATASET> and OURS in Table 8"
	@echo "  make eval-table8-geos-<DATASET>\n     compute results for <DATASET> and GEOS in Table 8"
	@echo "  make eval-table8-geos-prepared<DATASET>\n     compute results for <DATASET> and GEOS in Table 8"

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
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) dummy --combinations BdoI --only-show-commands --log-file __dummy_log > /dev/null 2> /dev/null && rm __dummy_log && echo " OK " || " NOT WORKING"

$(DATA_DIR)/%.raw.spatialjoin-input.tsv:
	@echo Downloading $*.spatialjoin-input.tsv
	@curl "https://ad-publications.cs.uni-freiburg.de/SIGSPATIAL_spatialjoin_BBK_2025.materials/$*.spatialjoin-input.tsv" > $@

$(DATA_DIR)/%-OURS.spatialjoin-input.tsv: $(DATA_DIR)/%.raw.spatialjoin-input.tsv
	@ln -s $(DATA_DIR)/$*.raw.spatialjoin-input.tsv $(DATA_DIR)/$*-OURS.spatialjoin-input.tsv

$(DATA_DIR)/%-GEOS.spatialjoin-input.tsv: $(DATA_DIR)/%.raw.spatialjoin-input.tsv
	@ln -s $(DATA_DIR)/$*.raw.spatialjoin-input.tsv $(DATA_DIR)/$*-GEOS.spatialjoin-input.tsv

$(DATA_DIR)/%-GEOS-PREPARED.spatialjoin-input.tsv: $(DATA_DIR)/%.raw.spatialjoin-input.tsv
	@ln -s $(DATA_DIR)/$*.raw.spatialjoin-input.tsv $(DATA_DIR)/$*-GEOS-PREPARED.spatialjoin-input.tsv

$(DATA_DIR)/%-MDIST-OURS.spatialjoin-input.tsv: $(DATA_DIR)/%.raw.spatialjoin-input.tsv
	@ln -s $(DATA_DIR)/$*.raw.spatialjoin-input.tsv $(DATA_DIR)/$*-MDIST-OURS.spatialjoin-input.tsv

$(DATA_DIR)/%-DIST-OURS.spatialjoin-input.tsv: $(DATA_DIR)/%.raw.spatialjoin-input.tsv
	@ln -s $(DATA_DIR)/$*.raw.spatialjoin-input.tsv $(DATA_DIR)/$*-DIST-OURS.spatialjoin-input.tsv

$(DATA_DIR)/%-DIST-GEOS.spatialjoin-input.tsv: $(DATA_DIR)/%.raw.spatialjoin-input.tsv
	@ln -s $(DATA_DIR)/$*.raw.spatialjoin-input.tsv $(DATA_DIR)/$*-DIST-GEOS.spatialjoin-input.tsv

$(DATA_DIR)/%-DIST-GEOS-PREPARED.spatialjoin-input.tsv: $(DATA_DIR)/%.raw.spatialjoin-input.tsv
	@ln -s $(DATA_DIR)/$*.raw.spatialjoin-input.tsv $(DATA_DIR)/$*-DIST-GEOS-PREPARED.spatialjoin-input.tsv

%-OURS.spatialjoin-evaluation.tsv: $(DATA_DIR)/%-OURS.spatialjoin-input.tsv
	@echo "*** Evaluating dataset" $* "on OURS"
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) $*-OURS --combinations bdoi,Bdoi,BDoi,BdOi,BdoI --core-library ours 2>&1 | tee $*-OURS.spatialjoin-evaluation.tsv || echo "Run failed, see $*-OURS.spatialjoin-evaluation.log for details"

%-GEOS.spatialjoin-evaluation.tsv: $(DATA_DIR)/%-GEOS.spatialjoin-input.tsv
	@echo "*** Evaluating dataset" $* "on GEOS"
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) $*-GEOS --combinations bdoi,Bdoi,Bdoi,BDoi,BdOi,BdoI --core-library libgeos 2>&1 | tee $*-GEOS.spatialjoin-evaluation.tsv || echo "Run failed, see $*-GEOS.spatialjoin-evaluation.log for details"

%-GEOS-PREPARED.spatialjoin-evaluation.tsv: $(DATA_DIR)/%-GEOS-PREPARED.spatialjoin-input.tsv
	@echo "*** Evaluating dataset" $* "on GEOS-PREPARED"
	python3 $(SPATIALJOIN_EVAL_SCRIPT) $*-GEOS-PREPARED --combinations bdoi,Bdoi,Bdoi,BDoi,BdOi,BdoI --core-library libgeos-prepared 2>&1 | tee $*-GEOS-PREPARED.spatialjoin-evaluation.tsv || echo "Run failed, see $*-GEOS-PREPARED.spatialjoin-evaluation.log for details"

%-MDIST-OURS.spatialjoin-evaluation.tsv: $(DATA_DIR)/%-MDIST-OURS.spatialjoin-input.tsv
	@echo "*** Evaluating dataset" $* "(distance) on OURS"
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) $*-MDIST-OURS --combinations bdoi,Bdoi,BDoi,BdOi,BdoI --core-library ours --use-de9im false --within-distance 50 --use-meter-distance true 2>&1 | tee $*-MDIST-OURS.spatialjoin-evaluation.tsv || echo "Run failed, see $*-MDIST-OURS.spatialjoin-evaluation.log for details"

%-DIST-OURS.spatialjoin-evaluation.tsv: $(DATA_DIR)/%-DIST-OURS.spatialjoin-input.tsv
	@echo "*** Evaluating dataset" $* "(distance) on OURS"
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) $*-DIST-OURS --combinations bdoi,Bdoi,BDoi,BdOi,BdoI --core-library ours --use-de9im false --within-distance 50 2>&1 | tee $*-DIST-OURS.spatialjoin-evaluation.tsv || echo "Run failed, see $*-DIST-OURS.spatialjoin-evaluation.log for details"

%-DIST-GEOS.spatialjoin-evaluation.tsv: $(DATA_DIR)/%-DIST-GEOS.spatialjoin-input.tsv
	@echo "*** Evaluating dataset" $* "(distance) on GEOS"
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) $*-DIST-GEOS --combinations bdoi,Bdoi,Bdoi,BDoi,BdOi,BdoI --core-library libgeos --use-de9im false --within-distance 50 2>&1 | tee $*-DIST-GEOS.spatialjoin-evaluation.tsv || echo "Run failed, see $*-DIST-GEOS.spatialjoin-evaluation.log for details"

%-DIST-GEOS-PREPARED.spatialjoin-evaluation.tsv: $(DATA_DIR)/%-DIST_GEOS-PREPARED.spatialjoin-input.tsv
	@echo "*** Evaluating dataset" $* "(distance) on GEOS-PREPARED"
	python3 $(SPATIALJOIN_EVAL_SCRIPT) $*-DIST-GEOS-PREPARED --combinations bdoi,Bdoi,Bdoi,BDoi,BdOi,BdoI --core-library libgeos-prepared --use-de9im false --within-distance 50 2>&1 | tee $*-DIST-GEOS-PREPARED.spatialjoin-evaluation.tsv || echo "Run failed, see $*-DIST-GEOS-PREPARED.spatialjoin-evaluation.log for details"

Q5-OURS.spatialjoin-evaluation.tsv: $(DATA_DIR)/Q5-OURS.spatialjoin-input.tsv
	@echo "*** Evaluating dataset" Q5 "on OURS"
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) Q5-OURS --combinations bdoi --use-de9im false --within-distance 10000000 --core-library ours 2>&1 | tee Q5-OURS.spatialjoin-evaluation.tsv || echo "Run failed, see Q5-OURS.spatialjoin-evaluation.log for details"

Q5-GEOS.spatialjoin-evaluation.tsv: $(DATA_DIR)/Q5-GEOS.spatialjoin-input.tsv
	@echo "*** Evaluating dataset" Q5 "on GEOS"
	python3 $(SPATIALJOIN_EVAL_SCRIPT) Q5-GEOS --combinations bdoi --use-de9im false --within-distance 10000000 --core-library libgeos 2>&1 | tee Q5-GEOS.spatialjoin-evaluation.tsv || echo "Run failed, see Q5-GEOS.spatialjoin-evaluation.log for details"

Q5-GEOS-PREPARED.spatialjoin-evaluation.tsv: $(DATA_DIR)/Q5-GEOS-PREPARED.spatialjoin-input.tsv
	@echo "*** Evaluating dataset" Q5 "on GEOS PREPARED"
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) Q5-GEOS-PREPARED --combinations bdoi --use-de9im false --within-distance 10000000 --core-library libgeos-prepared 2>&1 | tee Q5-GEOS-PREPARED.spatialjoin-evaluation.tsv || echo "Run failed, see Q5-GEOS-PREPARED.spatialjoin-evaluation.log for details"

Q%-OURS.spatialjoin-evaluation.tsv: $(DATA_DIR)/Q%-OURS.spatialjoin-input.tsv
	@echo "*** Evaluating dataset" Q$* "on OURS"
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) Q$*-OURS --combinations BDoi --core-library ours 2>&1 | tee Q$*-OURS.spatialjoin-evaluation.tsv || echo "Run failed, see $*-OURS.spatialjoin-evaluation.log for details"

Q%-GEOS.spatialjoin-evaluation.tsv: $(DATA_DIR)/Q%-GEOS.spatialjoin-input.tsv
	@echo "*** Evaluating dataset" Q$* "on GEOS"
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) Q$*-GEOS --combinations BdoI --core-library libgeos 2>&1 | tee Q$*-GEOS.spatialjoin-evaluation.tsv || echo "Run failed, see Q$*-GEOS.spatialjoin-evaluation.log for details"

Q%-GEOS-PREPARED.spatialjoin-evaluation.tsv: $(DATA_DIR)/Q%-GEOS-PREPARED.spatialjoin-input.tsv
	@echo "*** Evaluating dataset" Q$* "on GEOS PREPARED"
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) Q$*-GEOS-PREPARED --combinations BdoI --core-library libgeos-prepared 2>&1 | tee Q$*-GEOS-PREPARED.spatialjoin-evaluation.tsv || echo "Run failed, see Q$*-GEOS-PREPARED.spatialjoin-evaluation.log for details"

eval-table2-%: %-OURS.spatialjoin-evaluation.tsv
	@echo "*** Table 2 entries (OURS) on dataset" $*
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) $*-OURS --combinations bdoi,Bdoi,BDoi,BdOi,BdoI --analyze total --minutes

eval-table3-%: %-GEOS.spatialjoin-evaluation.tsv
	@echo "*** Table 3 entries (GEOS) on dataset" $*
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) $*-GEOS --combinations bdoi,Bdoi,BDoi,BdOi,BdoI --analyze total --core-library libgeos --minutes

eval-table4-%: %-GEOS-PREPARED.spatialjoin-evaluation.tsv
	@echo "*** Table 4 entries (GEOS-PREPARED) on dataset" $*
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) $*-GEOS --combinations bdoi,Bdoi,BDoi,BdOi,BdoI --analyze total --core-library libgeos-prepared --minutes

eval-table5-%: %-DIST-OURS.spatialjoin-evaluation.tsv
	@echo "*** Table 5 entries (OURS) on dataset" $*
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) $*-DIST-OURS --combinations bdoi,Bdoi,BDoi,BdOi,BdoI --analyze total --minutes

eval-table6-%: %-DIST-GEOS.spatialjoin-evaluation.tsv
	@echo "*** Table 6 entries (GEOS) on dataset" $*
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) $*-DIST-GEOS --combinations bdoi,Bdoi,BDoi,BdOi,BdoI --analyze total --minutes

eval-table7-%: %-DIST-GEOS-PREPARED.spatialjoin-evaluation.tsv
	@echo "*** Table 7 entries (GEOS-PREPARED) on dataset" $*
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) $*-DIST-GEOS-PREPARED --combinations bdoi,Bdoi,BDoi,BdOi,BdoI --analyze total --core-library libgeos-prepared --minutes

eval-table9-%: %-MDIST-OURS.spatialjoin-evaluation.tsv
	@echo "*** Table 9 entries (OURS) on dataset" $*
	python3 $(SPATIALJOIN_EVAL_SCRIPT) $*-MDIST-OURS --combinations bdoi,Bdoi,BDoi,BdOi,BdoI --analyze total --minutes

eval-table8-ours-Q5: Q5-OURS.spatialjoin-evaluation.tsv
	@echo "*** Table 8 entries (OURS) on dataset" Q5
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) Q5-OURS --combinations bdoi --analyze total --minutes

eval-table8-geos-Q5: Q5-GEOS.spatialjoin-evaluation.tsv
	@echo "*** Table 8 entries (GEOS) on dataset" $*
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) Q5-GEOS --combinations bdoi --analyze total --core-library libgeos  --minutes

eval-table8-geos-prepared-Q5: Q5-GEOS-PREPARED.spatialjoin-evaluation.tsv
	@echo "*** Table 7 entries (GEOS-PREPARED) on dataset" $*
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) Q5-GEOS-PREPARED --combinations bdoi --analyze total --core-library libgeos-prepared  --minutes

eval-table8-ours-%: %-OURS.spatialjoin-evaluation.tsv
	@echo "*** Table 8 entries (OURS) on dataset" $*
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) $*-OURS --combinations BDoi --analyze total --minutes

eval-table8-geos-%: %-GEOS.spatialjoin-evaluation.tsv
	@echo "*** Table 8 entries (GEOS) on dataset" $*
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) $*-GEOS --combinations BdoI --analyze total --core-library libgeos  --minutes

eval-table8-geos-prepared-%: %-GEOS-PREPARED.spatialjoin-evaluation.tsv
	@echo "*** Table 7 entries (GEOS-PREPARED) on dataset" $*
	@python3 $(SPATIALJOIN_EVAL_SCRIPT) $*-GEOS-PREPARED --combinations BdoI --analyze total --core-library libgeos-prepared  --minutes

eval-table2: eval-table2-FIN eval-table2-GER eval-table2-OHM eval-table2-OSM
eval-table3: eval-table3-FIN eval-table3-GER eval-table3-OHM eval-table3-OSM
eval-table4: eval-table4-FIN eval-table4-GER eval-table4-OHM eval-table4-OSM
eval-table5: eval-table5-FIN eval-table5-GER eval-table5-OHM eval-table5-OSM
eval-table6: eval-table6-FIN eval-table6-GER eval-table6-OHM eval-table6-OSM
eval-table7: eval-table7-FIN eval-table7-GER eval-table7-OHM eval-table7-OSM
eval-table8-ours: eval-table8-ours-Q1 eval-table8-ours-Q2 eval-table8-ours-Q3 eval-table8-ours-Q4 eval-table8-ours-Q5
eval-table8-geos: eval-table8-geos-Q1 eval-table8-geos-Q2 eval-table8-geos-Q3 eval-table8-geos-Q4 eval-table8-geos-Q5
eval-table8-geos-prepared: eval-table8-geos-prepared-Q1 eval-table8-geos-prepared-Q2 eval-table8-geos-prepared-Q3 eval-table8-geos-prepared-Q4 eval-table8-geos-prepared-Q5
eval-table9: eval-table9-FIN eval-table9-GER eval-table9-OHM eval-table9-OSM

eval: eval-table2 eval-table3 eval-table8-ours eval-table8-geos

clean:
	rm -f *.spatialjoin-evaluation.tsv
	rm -f *.spatialjoin-evaluation.log
