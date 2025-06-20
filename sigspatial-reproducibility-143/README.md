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


## Preparation

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

### Pre-flight checks

Run
```
make check
```
to check if your evaluation environment is correctly set up.

## Running the evaluation

Type
```
make eval
```
to run each evaluation. The entries for Tables 2, 3 and 4 will be printed to the console.

For more specific runs, type
```
make help
```
