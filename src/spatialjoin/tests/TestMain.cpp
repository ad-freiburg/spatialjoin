// Copyright 2024
// Author: Patrick Brosi

#include <iostream>
#include <string>

#include "spatialjoin/BoxIds.h"
#include "spatialjoin/OutputWriter.h"
#include "spatialjoin/Sweeper.h"
#include "spatialjoin/WKTParse.h"
#include "util/Test.h"
#include "util/geo/Geo.h"
#include "util/log/Log.h"

using sj::ParseBatch;
using sj::Sweeper;

size_t NUM_THREADS = 1;

// _____________________________________________________________________________
std::string fullRun(const std::string& file, sj::SweeperCfg cfg) {
  {
    sj::OutputWriter outWriter(NUM_THREADS, "$", "$\n", ".resTmp", ".");
    cfg.writeRelCb = [&outWriter](size_t t, const char* a, size_t an,
                                  const char* b, size_t bn, const char* pred,
                                  size_t predn) {
      outWriter.writeRelCb(t, a, an, b, bn, pred, predn);
    };
    Sweeper sweeper(cfg, ".");

    // small buffer size 1 here for test purposes to force buffer overflows
    // during parsing
    const static size_t BUFF_SIZE = 100;
    char* buf = new char[BUFF_SIZE];

    size_t len = 0;

    int f = open(file.c_str(), O_RDONLY);
    TEST(f >= 0);

    sj::WKTParser parser(&sweeper, 1);

    while ((len = read(f, buf, BUFF_SIZE)) > 0) {
      parser.parse(buf, len, 0);
    }
    parser.done();

    delete[] buf;

    sweeper.flush();

    sweeper.sweep();

    close(f);
  }

  std::string ret;

  std::stringstream ss;
  std::ifstream ifs(".resTmp");
  ss << ifs.rdbuf();

  ifs.close();
  unlink(".resTmp");

  return ss.str();
}

// _____________________________________________________________________________
int main(int, char**) {
  for (uint64_t i = 0; i < 9999; i++) {
    // TEST(i == Sweeper::base126ToInt(Sweeper::intToBase126(i)));
  }

  sj::SweeperCfg baseline{
      NUM_THREADS,
      NUM_THREADS,
      1000,
      1000,
      " intersects ",
      " contains ",
      " covers ",
      " touches ",
      " equals ",
      " overlaps ",
      " crosses ",
      false,
      false,
      false,
      false,
      false,
      false,
      false,
      -1,
      false,
      {},
      {},
      {},
      {},
      {},
      {}};

  sj::SweeperCfg all{
      NUM_THREADS,
      NUM_THREADS,
      1000,
      1000,
      " intersects ",
      " contains ",
      " covers ",
      " touches ",
      " equals ",
      " overlaps ",
      " crosses ",
      true,
      true,
      true,
      true,
      true,
      true,
      false,
      -1,
      false,
      {},
      {},
      {},
      {},
      {},
      {}};

  sj::SweeperCfg noSurfaceArea{
      NUM_THREADS,
      NUM_THREADS,
      1000,
      1000,
      " intersects ",
      " contains ",
      " covers ",
      " touches ",
      " equals ",
      " overlaps ",
      " crosses ",
      true,
      false,
      true,
      true,
      true,
      true,
      false,
      -1,
      false,
      {},
      {},
      {},
      {},
      {},
      {}};

  sj::SweeperCfg noBoxIds{
      NUM_THREADS,
      NUM_THREADS,
      1000,
      1000,
      " intersects ",
      " contains ",
      " covers ",
      " touches ",
      " equals ",
      " overlaps ",
      " crosses ",
      false,
      true,
      true,
      true,
      true,
      true,
      false,
      -1,
      false,
      {},
      {},
      {},
      {},
      {},
      {}};

  sj::SweeperCfg noObb{
      NUM_THREADS,
      NUM_THREADS,
      1000,
      1000,
      " intersects ",
      " contains ",
      " covers ",
      " touches ",
      " equals ",
      " overlaps ",
      " crosses ",
      true,
      true,
      false,
      true,
      true,
      true,
      false,
      -1,
      false,
      {},
      {},
      {},
      {},
      {},
      {}};

  sj::SweeperCfg noDiagBox{
      NUM_THREADS,
      NUM_THREADS,
      1000,
      1000,
      " intersects ",
      " contains ",
      " covers ",
      " touches ",
      " equals ",
      " overlaps ",
      " crosses ",
      true,
      true,
      true,
      false,
      true,
      true,
      false,
      -1,
      false,
      {},
      {},
      {},
      {},
      {},
      {}};

  sj::SweeperCfg noFastSweep{
      NUM_THREADS,
      NUM_THREADS,
      1000,
      1000,
      " intersects ",
      " contains ",
      " covers ",
      " touches ",
      " equals ",
      " overlaps ",
      " crosses ",
      true,
      true,
      true,
      true,
      false,
      true,
      false,
      -1,
      false,
      {},
      {},
      {},
      {},
      {},
      {}};

  sj::SweeperCfg noInnerOuter{
      NUM_THREADS,  NUM_THREADS, 1000,        1000,       " intersects ",
      " contains ", " covers ",  " touches ", " equals ", " overlaps ",
      " crosses ",  true,        true,        true,       true,
      true,         false,       false,       -1,         false,
      {}, {},           {},          {},          {},         {}};

  std::vector<sj::SweeperCfg> cfgs{baseline,    all,         noSurfaceArea,
                                   noBoxIds,    noObb,       noDiagBox,
                                   noFastSweep, noInnerOuter};

  for (auto cfg : cfgs) {
    {
      auto res = fullRun("../src/spatialjoin/tests/datasets/freiburg", cfg);

      TEST(res.find("$freiburg1 covers freiburg2$") != std::string::npos);
      TEST(res.find("$freiburg2 covers freiburg1$") != std::string::npos);
      TEST(res.find("$freiburg1 equals freiburg2$") != std::string::npos);
      TEST(res.find("$freiburg2 equals freiburg1$") != std::string::npos);
      TEST(res.find("$freiburg1 contains freiburg2$") != std::string::npos);
      TEST(res.find("$freiburg2 contains freiburg1$") != std::string::npos);
      TEST(res.find("$freiburg1 intersects freiburg2$") != std::string::npos);
      TEST(res.find("$freiburg2 intersects freiburg1$") != std::string::npos);
      TEST(res.find("$freiburg2 overlaps freiburg1$") == std::string::npos);
      TEST(res.find("$freiburg2 touches freiburg1$") == std::string::npos);

      TEST(res.find("$freiburg1 covers grenzpart$") != std::string::npos);
      TEST(res.find("$freiburg1 contains grenzpart$") == std::string::npos);
      TEST(res.find("$grenzpart covers freiburg1$") == std::string::npos);

      TEST(res.find("$freiburg1 intersects grenz$") != std::string::npos);
      TEST(res.find("$freiburg2 intersects grenz$") != std::string::npos);
      TEST(res.find("$freiburg1 covers grenz$") == std::string::npos);
      TEST(res.find("$freiburg2 covers grenz$") == std::string::npos);
      TEST(res.find("$freiburg1 contains grenz$") == std::string::npos);
      TEST(res.find("$freiburg2 contains grenz$") == std::string::npos);

      TEST(res.find("$freiburg1 intersects a$") != std::string::npos);
      TEST(res.find("$freiburg2 intersects a$") != std::string::npos);
      TEST(res.find("$freiburg1 covers a$") == std::string::npos);
      TEST(res.find("$freiburg2 covers a$") == std::string::npos);
      TEST(res.find("$freiburg1 contains a$") == std::string::npos);
      TEST(res.find("$freiburg2 contains a$") == std::string::npos);

      TEST(res.find("$freiburg1 intersects b$") != std::string::npos);
      TEST(res.find("$freiburg2 intersects b$") != std::string::npos);
      TEST(res.find("$freiburg1 covers b$") == std::string::npos);
      TEST(res.find("$freiburg2 covers b$") == std::string::npos);
      TEST(res.find("$freiburg1 contains b$") == std::string::npos);
      TEST(res.find("$freiburg2 contains b$") == std::string::npos);

      TEST(res.find("$freiburg1 intersects c$") != std::string::npos);
      TEST(res.find("$freiburg2 intersects c$") != std::string::npos);
      TEST(res.find("$freiburg1 covers c$") == std::string::npos);
      TEST(res.find("$freiburg2 covers c$") == std::string::npos);
      TEST(res.find("$freiburg1 contains c$") == std::string::npos);
      TEST(res.find("$freiburg2 contains c$") == std::string::npos);

      TEST(res.find("$freiburg1 intersects d$") != std::string::npos);
      TEST(res.find("$freiburg2 intersects d$") != std::string::npos);
      TEST(res.find("$freiburg1 covers d$") == std::string::npos);
      TEST(res.find("$freiburg2 covers d$") == std::string::npos);
      TEST(res.find("$freiburg1 contains d$") == std::string::npos);
      TEST(res.find("$freiburg2 contains d$") == std::string::npos);

      TEST(res.find("$freiburg1 intersects grenzpunkt$") != std::string::npos);
      TEST(res.find("$freiburg2 intersects grenzpunkt$") != std::string::npos);
      TEST(res.find("$freiburg1 covers grenzpunkt$") != std::string::npos);
      TEST(res.find("$freiburg2 covers grenzpunkt$") != std::string::npos);
      TEST(res.find("$freiburg1 contains grenzpunkt$") == std::string::npos);
      TEST(res.find("$freiburg2 contains grenzpunkt$") == std::string::npos);

      TEST(res.find("$freiburg1 intersects grenzpunkt2$") != std::string::npos);
      TEST(res.find("$freiburg2 intersects grenzpunkt2$") != std::string::npos);
      TEST(res.find("$freiburg1 covers grenzpunkt2$") != std::string::npos);
      TEST(res.find("$freiburg2 covers grenzpunkt2$") != std::string::npos);
      TEST(res.find("$freiburg1 contains grenzpunkt2$") == std::string::npos);
      TEST(res.find("$freiburg2 contains grenzpunkt2$") == std::string::npos);

      TEST(res.find("$freiburg1 covers Umkirch$") == std::string::npos);

      TEST(res.find("$freiburg1 covers Sankt Georgen$") != std::string::npos);
      TEST(res.find("$freiburg1 intersects Sankt Georgen$") !=
           std::string::npos);
      TEST(res.find("$freiburg1 contains Sankt Georgen$") != std::string::npos);

      TEST(res.find("$freiburg1 covers Haslach$") != std::string::npos);

      TEST(res.find("$freiburg1 covers Günterstal$") != std::string::npos);
      TEST(res.find("$freiburg1 intersects Günterstal$") != std::string::npos);
      TEST(res.find("$freiburg1 contains Günterstal$") != std::string::npos);

      TEST(res.find("$freiburg1 covers Kappel$") != std::string::npos);
      TEST(res.find("$freiburg1 intersects Kappel$") != std::string::npos);
      TEST(res.find("$freiburg1 contains Kappel$") != std::string::npos);

      TEST(res.find("$freiburg1 covers Littenweiler$") != std::string::npos);
      TEST(res.find("$freiburg1 intersects Littenweiler$") !=
           std::string::npos);
      TEST(res.find("$freiburg1 contains Littenweiler$") != std::string::npos);

      TEST(res.find("$freiburg1 covers Waldsee$") != std::string::npos);
      TEST(res.find("$freiburg1 intersects Waldsee$") != std::string::npos);
      TEST(res.find("$freiburg1 contains Waldsee$") != std::string::npos);

      TEST(res.find("$freiburg1 covers Wiehre$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Waltershofen$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Hochdorf$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Opfingen$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Betzenhausen$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Brühl$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Landwasser$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Lehen$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Mooswald$") != std::string::npos);

      TEST(res.find("$freiburg1 covers Munzingen$") != std::string::npos);
      TEST(res.find("$freiburg1 intersects Munzingen$") != std::string::npos);
      TEST(res.find("$freiburg1 contains Munzingen$") != std::string::npos);

      TEST(res.find("$freiburg1 covers Tiengen$") != std::string::npos);
      TEST(res.find("$freiburg1 intersects Tiengen$") != std::string::npos);
      TEST(res.find("$freiburg1 contains Tiengen$") != std::string::npos);

      TEST(res.find("$freiburg1 covers Mundenhof$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Rieselfeld$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Weingarten$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Altstadt$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Stühlinger$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Neuburg$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Herdern$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Zähringen$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Ebnet$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Oberau$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Sankt Georgen Süd$") !=
           std::string::npos);
      TEST(res.find("$freiburg1 covers Sankt Georgen Nord$") !=
           std::string::npos);
      TEST(res.find("$Sankt Georgen covers Sankt Georgen Süd$") !=
           std::string::npos);
      TEST(res.find("$Sankt Georgen covers Sankt Georgen Nord$") !=
           std::string::npos);
      TEST(res.find("$Sankt Georgen Süd touches Sankt Georgen Nord$") !=
           std::string::npos);
      TEST(res.find("$Sankt Georgen Nord touches Sankt Georgen Süd$") !=
           std::string::npos);
      TEST(res.find("$freiburg1 covers Haslach-Haid$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Haslach-Gartenstadt$") !=
           std::string::npos);
      TEST(res.find("$Haslach covers Haslach-Haid$") != std::string::npos);
      TEST(res.find("$Haslach covers Haslach-Gartenstadt$") !=
           std::string::npos);
      TEST(res.find("$Haslach covers Haslach-Schildacker$") !=
           std::string::npos);
      TEST(res.find("$Haslach covers Haslach-Egerten$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Haslach-Schildacker$") !=
           std::string::npos);
      TEST(res.find("$freiburg1 covers Haslach-Egerten$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Unterwiehre-Nord$") !=
           std::string::npos);
      TEST(res.find("$freiburg1 covers Unterwiehre-Süd$") != std::string::npos);
      TEST(res.find("$Wiehre covers Unterwiehre-Nord$") != std::string::npos);
      TEST(res.find("$Wiehre covers Unterwiehre-Süd$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Altstadt-Ring$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Alt-Stühlinger$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Betzenhausen-Bischofslinde$") !=
           std::string::npos);
      TEST(res.find("$freiburg1 covers Stühlinger-Eschholz$") !=
           std::string::npos);
      TEST(res.find("$freiburg1 covers Mooswald-Ost$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Brühl-Beurbarung$") !=
           std::string::npos);
      TEST(res.find("$freiburg1 covers Brühl-Industriegebiet$") !=
           std::string::npos);
      TEST(res.find("$freiburg1 covers Altstadt-Mitte$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Mittelwiehre$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Oberwiehre$") != std::string::npos);
      TEST(res.find("$Wiehre covers Mittelwiehre$") != std::string::npos);
      TEST(res.find("$Wiehre contains Mittelwiehre$") != std::string::npos);
      TEST(res.find("$Wiehre covers Oberwiehre$") != std::string::npos);
      TEST(res.find("$Wiehre contains Oberwiehre$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Alt-Betzenhausen$") !=
           std::string::npos);
      TEST(res.find("$freiburg1 covers Mooswald-West$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Brühl-Güterbahnhof$") !=
           std::string::npos);
      TEST(res.find("$freiburg1 covers Herdern-Süd$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Herdern-Nord$") != std::string::npos);
      TEST(res.find("$Herdern covers Herdern-Süd$") != std::string::npos);
      TEST(res.find("$Herdern covers Herdern-Nord$") != std::string::npos);
      TEST(res.find("$freiburg1 covers Vauban$") != std::string::npos);

      TEST(res.find("$someway intersects someotherway$") != std::string::npos);
      TEST(res.find("$someway covers someotherway$") == std::string::npos);
      TEST(res.find("$someway contains someotherway$") == std::string::npos);

      TEST(res.find("$someway covers someway2$") != std::string::npos);
      TEST(res.find("$someway intersects someway2$") != std::string::npos);

      TEST(res.find("$someway2 covers someway$") != std::string::npos);
      TEST(res.find("$someway2 intersects someway$") != std::string::npos);

      TEST(res.find("$someotherway intersects someway$") != std::string::npos);
      TEST(res.find("$someotherway covers someway$") == std::string::npos);
      TEST(res.find("$someotherway contains someway$") == std::string::npos);
    }

    {
      auto res = fullRun("../src/spatialjoin/tests/datasets/brandenburg", cfg);
      TEST(res.find("$Brandenburg covers Brandenburg2$") != std::string::npos);
      TEST(res.find("$Brandenburg intersects Brandenburg-Way$") !=
           std::string::npos);
      TEST(res.find("$Brandenburg equals Brandenburg2$") != std::string::npos);
      TEST(res.find("$Brandenburg2 equals Brandenburg$") != std::string::npos);
      TEST(res.find("$Brandenburg intersects Grenzpart$") != std::string::npos);
      TEST(res.find("$Brandenburg covers Grenzpart$") != std::string::npos);
      TEST(res.find("$Brandenburg contains Grenzpart$") == std::string::npos);

      TEST(res.find("$Brandenburg intersects Berlin$") != std::string::npos);
      TEST(res.find("$Brandenburg touches Berlin$") != std::string::npos);
      TEST(res.find("$Brandenburg overlaps Berlin$") == std::string::npos);
      TEST(res.find("$Berlin touches Brandenburg$") != std::string::npos);
      TEST(res.find("$Berlin overlaps Brandenburg$") == std::string::npos);
      TEST(res.find("$Brandenburg contains Berlin$") == std::string::npos);
      TEST(res.find("$Brandenburg covers Berlin$") == std::string::npos);

      TEST(res.find("$Berlin intersects Grenzpart$") != std::string::npos);
      TEST(res.find("$Berlin covers Grenzpart$") != std::string::npos);
      TEST(res.find("$Berlin contains Grenzpart$") == std::string::npos);

      TEST(res.find("$Haus overlaps Brandenburg$") != std::string::npos);
      TEST(res.find("$Haus-Way intersects Brandenburg$") != std::string::npos);
      TEST(res.find("$Haus intersects Brandenburg-Way$") != std::string::npos);
      TEST(res.find("$Haus-Way intersects Brandenburg-Way$") !=
           std::string::npos);

      TEST(res.find("$Brandenburg-Point intersects Brandenburg-Way$") !=
           std::string::npos);
      TEST(res.find("$Brandenburg-Point intersects Brandenburg$") !=
           std::string::npos);
    }

    {
      auto res =
          fullRun("../src/spatialjoin/tests/datasets/brandenburg_nonself", cfg);
      TEST(res.find("$Brandenburg covers Brandenburg2$") == std::string::npos);
      TEST(res.find("$Brandenburg intersects Brandenburg-Way$") ==
           std::string::npos);
      TEST(res.find("$Brandenburg equals Brandenburg2$") == std::string::npos);
      TEST(res.find("$Brandenburg2 equals Brandenburg$") == std::string::npos);
      TEST(res.find("$Brandenburg intersects Grenzpart$") == std::string::npos);
      TEST(res.find("$Grenzpart intersects Brandenburg$") != std::string::npos);
      TEST(res.find("$Brandenburg covers Grenzpart$") == std::string::npos);
      TEST(res.find("$Brandenburg contains Grenzpart$") == std::string::npos);

      TEST(res.find("$Brandenburg intersects Berlin$") == std::string::npos);
      TEST(res.find("$Berlin intersects Brandenburg$") != std::string::npos);
      TEST(res.find("$Brandenburg touches Berlin$") == std::string::npos);
      TEST(res.find("$Berlin touches Brandenburg$") != std::string::npos);
      TEST(res.find("$Brandenburg overlaps Berlin$") == std::string::npos);
      TEST(res.find("$Berlin touches Brandenburg$") != std::string::npos);
      TEST(res.find("$Berlin overlaps Brandenburg$") == std::string::npos);
      TEST(res.find("$Brandenburg contains Berlin$") == std::string::npos);
      TEST(res.find("$Brandenburg covers Berlin$") == std::string::npos);

      TEST(res.find("$Berlin intersects Grenzpart$") == std::string::npos);
      TEST(res.find("$Berlin covers Grenzpart$") == std::string::npos);
      TEST(res.find("$Berlin contains Grenzpart$") == std::string::npos);

      TEST(res.find("$Haus overlaps Brandenburg$") != std::string::npos);
      TEST(res.find("$Haus-Way intersects Brandenburg$") != std::string::npos);
      TEST(res.find("$Haus intersects Brandenburg-Way$") != std::string::npos);
      TEST(res.find("$Haus-Way intersects Brandenburg-Way$") !=
           std::string::npos);

      TEST(res.find("$Brandenburg-Point intersects Brandenburg-Way$") !=
           std::string::npos);
      TEST(res.find("$Brandenburg-Point intersects Brandenburg$") !=
           std::string::npos);
    }

    {
      auto res =
          fullRun("../src/spatialjoin/tests/datasets/collectiontests", cfg);

      TEST(res.find("$28 covers 27$") != std::string::npos);

      TEST(res.find("$27 overlaps 28$") == std::string::npos);
      TEST(res.find("$28 overlaps 27$") == std::string::npos);

      TEST(res.find("$17 touches 18$") != std::string::npos);
      TEST(res.find("$19 touches 18$") != std::string::npos);
      TEST(res.find("$17 touches 16$") != std::string::npos);

      TEST(res.find("$1 equals 2$") != std::string::npos);
      TEST(res.find("$1 overlaps 2$") == std::string::npos);
      TEST(res.find("$2 equals 1$") != std::string::npos);
      TEST(res.find("$3 equals 1$") == std::string::npos);
      TEST(res.find("$3 equals 2$") == std::string::npos);
      TEST(res.find("$3 equals 4$") != std::string::npos);
      TEST(res.find("$3 equals 5$") != std::string::npos);
      TEST(res.find("$4 equals 3$") != std::string::npos);
      TEST(res.find("$5 equals 3$") != std::string::npos);
      TEST(res.find("$5 equals 4$") != std::string::npos);
      TEST(res.find("$4 equals 5$") != std::string::npos);
      TEST(res.find("$4 overlaps 5$") == std::string::npos);
      TEST(res.find("$1 touches 6$") != std::string::npos);
      TEST(res.find("$6 touches 1$") != std::string::npos);
      TEST(res.find("$2 touches 6$") != std::string::npos);
      TEST(res.find("$6 touches 2$") != std::string::npos);
      TEST(res.find("$3 touches 6$") != std::string::npos);
      TEST(res.find("$6 touches 3$") != std::string::npos);
      TEST(res.find("$4 touches 6$") != std::string::npos);
      TEST(res.find("$6 touches 4$") != std::string::npos);
      TEST(res.find("$6 touches 7$") == std::string::npos);
      TEST(res.find("$7 touches 6$") == std::string::npos);

      TEST(res.find("$8 touches 9$") != std::string::npos);
      TEST(res.find("$9 touches 8$") != std::string::npos);

      TEST(res.find("$8 touches 11$") == std::string::npos);

      TEST(res.find("$9 touches 10$") == std::string::npos);
      TEST(res.find("$8 touches 10$") == std::string::npos);
      TEST(res.find("$12 touches 10$") != std::string::npos);
      TEST(res.find("$13 equals 10$") != std::string::npos);
      TEST(res.find("$13 equals 53$") != std::string::npos);
      TEST(res.find("$10 equals 53$") != std::string::npos);

      TEST(res.find("$28 equals 54$") != std::string::npos);

      TEST(res.find("$6 equals 14$") != std::string::npos);
      TEST(res.find("$15 touches 16$") != std::string::npos);
      TEST(res.find("$17 touches 6$") == std::string::npos);
      TEST(res.find("$17 intersects 16$") != std::string::npos);
      TEST(res.find("$17 intersects 18$") != std::string::npos);

      TEST(res.find("$8 overlaps 20$") != std::string::npos);
      TEST(res.find("$20 overlaps 8$") != std::string::npos);

      TEST(res.find("$17 crosses 21$") != std::string::npos);
      TEST(res.find("$21 crosses 17$") != std::string::npos);

      TEST(res.find("$17 crosses 22$") == std::string::npos);
      TEST(res.find("$22 crosses 17$") == std::string::npos);

      TEST(res.find("$17 overlaps 22$") == std::string::npos);
      TEST(res.find("$22 overlaps 17$") == std::string::npos);

      TEST(res.find("$24 covers 25$") != std::string::npos);

      TEST(res.find("$25 overlaps 24$") == std::string::npos);
      TEST(res.find("$24 overlaps 25$") == std::string::npos);

      TEST(res.find("$23 overlaps 24$") == std::string::npos);
      TEST(res.find("$24 overlaps 23$") == std::string::npos);

      TEST(res.find("$26 overlaps 24$") != std::string::npos);
      TEST(res.find("$24 overlaps 26$") != std::string::npos);

      TEST(res.find("$26 overlaps 27$") != std::string::npos);
      TEST(res.find("$27 overlaps 26$") != std::string::npos);

      TEST(res.find("$26 overlaps 28$") != std::string::npos);
      TEST(res.find("$28 overlaps 26$") != std::string::npos);

      TEST(res.find("8 overlaps 29$") == std::string::npos);
      TEST(res.find("8 overlaps 29$") == std::string::npos);
      TEST(res.find("30 covers 8$") != std::string::npos);
      TEST(res.find("8 overlaps 30$") == std::string::npos);
      TEST(res.find("29 overlaps 30$") == std::string::npos);
      TEST(res.find("30 overlaps 29") == std::string::npos);

      TEST(res.find("31 overlaps 30$") != std::string::npos);
      TEST(res.find("30 overlaps 31") != std::string::npos);
    }

    {
      auto res = fullRun("../src/spatialjoin/tests/datasets/multitests", cfg);

      TEST(res.find("$28 covers 27$") != std::string::npos);

      TEST(res.find("$27 overlaps 28$") == std::string::npos);
      TEST(res.find("$28 overlaps 27$") == std::string::npos);

      TEST(res.find("$17 touches 18$") != std::string::npos);
      TEST(res.find("$19 touches 18$") != std::string::npos);
      TEST(res.find("$17 touches 16$") != std::string::npos);

      TEST(res.find("$1 equals 2$") != std::string::npos);
      TEST(res.find("$1 overlaps 2$") == std::string::npos);
      TEST(res.find("$2 equals 1$") != std::string::npos);
      TEST(res.find("$3 equals 1$") == std::string::npos);
      TEST(res.find("$3 equals 2$") == std::string::npos);
      TEST(res.find("$3 equals 4$") != std::string::npos);
      TEST(res.find("$3 equals 5$") != std::string::npos);
      TEST(res.find("$4 equals 3$") != std::string::npos);
      TEST(res.find("$5 equals 3$") != std::string::npos);
      TEST(res.find("$5 equals 4$") != std::string::npos);
      TEST(res.find("$4 equals 5$") != std::string::npos);
      TEST(res.find("$4 overlaps 5$") == std::string::npos);
      TEST(res.find("$1 touches 6$") != std::string::npos);
      TEST(res.find("$6 touches 1$") != std::string::npos);
      TEST(res.find("$2 touches 6$") != std::string::npos);
      TEST(res.find("$6 touches 2$") != std::string::npos);
      TEST(res.find("$3 touches 6$") != std::string::npos);
      TEST(res.find("$6 touches 3$") != std::string::npos);
      TEST(res.find("$4 touches 6$") != std::string::npos);
      TEST(res.find("$6 touches 4$") != std::string::npos);
      TEST(res.find("$6 touches 7$") == std::string::npos);
      TEST(res.find("$7 touches 6$") == std::string::npos);

      TEST(res.find("$8 touches 9$") != std::string::npos);
      TEST(res.find("$9 touches 8$") != std::string::npos);

      TEST(res.find("$8 touches 11$") == std::string::npos);

      TEST(res.find("$9 touches 10$") == std::string::npos);
      TEST(res.find("$8 touches 10$") == std::string::npos);
      TEST(res.find("$12 touches 10$") != std::string::npos);
      TEST(res.find("$13 equals 10$") != std::string::npos);

      TEST(res.find("$6 equals 14$") != std::string::npos);
      TEST(res.find("$15 touches 16$") != std::string::npos);
      TEST(res.find("$17 touches 6$") == std::string::npos);
      TEST(res.find("$17 intersects 16$") != std::string::npos);
      TEST(res.find("$17 intersects 18$") != std::string::npos);

      TEST(res.find("$8 overlaps 20$") != std::string::npos);
      TEST(res.find("$20 overlaps 8$") != std::string::npos);

      TEST(res.find("$17 crosses 21$") != std::string::npos);
      TEST(res.find("$21 crosses 17$") != std::string::npos);

      TEST(res.find("$17 crosses 22$") == std::string::npos);
      TEST(res.find("$22 crosses 17$") == std::string::npos);

      TEST(res.find("$17 overlaps 22$") == std::string::npos);
      TEST(res.find("$22 overlaps 17$") == std::string::npos);

      TEST(res.find("$24 covers 25$") != std::string::npos);

      TEST(res.find("$25 overlaps 24$") == std::string::npos);
      TEST(res.find("$24 overlaps 25$") == std::string::npos);

      TEST(res.find("$23 overlaps 24$") == std::string::npos);
      TEST(res.find("$24 overlaps 23$") == std::string::npos);

      TEST(res.find("$26 overlaps 24$") != std::string::npos);
      TEST(res.find("$24 overlaps 26$") != std::string::npos);

      TEST(res.find("$26 overlaps 27$") != std::string::npos);
      TEST(res.find("$27 overlaps 26$") != std::string::npos);

      TEST(res.find("$26 overlaps 28$") != std::string::npos);
      TEST(res.find("$28 overlaps 26$") != std::string::npos);

      TEST(res.find("8 overlaps 29$") == std::string::npos);
      TEST(res.find("8 overlaps 29$") == std::string::npos);
      TEST(res.find("30 covers 8$") != std::string::npos);
      TEST(res.find("8 overlaps 30$") == std::string::npos);
      TEST(res.find("29 overlaps 30$") == std::string::npos);
      TEST(res.find("30 overlaps 29") == std::string::npos);

      TEST(res.find("31 overlaps 30$") != std::string::npos);
      TEST(res.find("30 overlaps 31") != std::string::npos);
    }

    {
      auto res = fullRun("../src/spatialjoin/tests/datasets/coverfail", cfg);
      TEST(res.find("$1 intersects 2$") != std::string::npos);
      TEST(res.find("$2 intersects 1$") != std::string::npos);
      TEST(res.find("$1 contains 2$") == std::string::npos);
      TEST(res.find("$2 contains 1$") == std::string::npos);
      TEST(res.find("$1 covers 2$") == std::string::npos);
      TEST(res.find("$2 covers 1$") == std::string::npos);
      TEST(res.find("$1 touches 2$") != std::string::npos);
      TEST(res.find("$2 touches 1$") != std::string::npos);
    }

    {
      auto res = fullRun("../src/spatialjoin/tests/datasets/touchfail", cfg);
      TEST(res.find("$1 intersects 2$") != std::string::npos);
      TEST(res.find("$2 intersects 1$") != std::string::npos);
      TEST(res.find("$1 overlaps 2$") != std::string::npos);
      TEST(res.find("$2 overlaps 1$") != std::string::npos);
      TEST(res.find("$1 touches 2$") == std::string::npos);
      TEST(res.find("$2 touches 1$") == std::string::npos);
    }

    {
      auto res = fullRun("../src/spatialjoin/tests/datasets/touchwayfail", cfg);
      TEST(res.find("$1 touches 2$") != std::string::npos);
      TEST(res.find("$2 touches 1$") != std::string::npos);
    }

    {
      auto res =
          fullRun("../src/spatialjoin/tests/datasets/simpleareafail", cfg);
      TEST(res.find("$1 intersects 2$") != std::string::npos);
      TEST(res.find("$2 intersects 1$") != std::string::npos);
      TEST(res.find("$2 crosses 1$") != std::string::npos);
    }

    {
      auto res = fullRun("../src/spatialjoin/tests/datasets/touchfail2", cfg);
      TEST(res.find("$1 covers 2$") != std::string::npos);
      TEST(res.find("$1 intersects 2$") != std::string::npos);
      TEST(res.find("$2 intersects 1$") != std::string::npos);
    }

    {
      auto res = fullRun("../src/spatialjoin/tests/datasets/boxidfail", cfg);
      TEST(res.find("$osmway:312944635 intersects osmway:312944634$") !=
           std::string::npos);
      TEST(res.find("$osmway:312944634 intersects osmway:312944635$") !=
           std::string::npos);
    }
    {
      auto res = fullRun("../src/spatialjoin/tests/datasets/boxidfail2", cfg);
      TEST(res.find("$osmway:205756242 intersects osmway:50218266$") !=
           std::string::npos);
      TEST(res.find("$osmway:50218266 intersects osmway:205756242$") !=
           std::string::npos);
    }
    {
      auto res = fullRun("../src/spatialjoin/tests/datasets/boxidfail3", cfg);
      TEST(res.find("$osmway:901094335 intersects osmnode:8370757906$") !=
           std::string::npos);
      TEST(res.find("$osmnode:8370757906 intersects osmway:901094335$") !=
           std::string::npos);
    }

    {
      auto res = fullRun("../src/spatialjoin/tests/datasets/references", cfg);
      TEST(res.find("$RefA crosses TestC$") != std::string::npos);
      TEST(res.find("$TestC crosses RefA$") != std::string::npos);
      TEST(res.find("$TestA crosses RefA$") == std::string::npos);
      TEST(res.find("$TestA crosses TestB$") != std::string::npos);
      TEST(res.find("$TestB crosses TestA$") != std::string::npos);
      TEST(res.find("$RefA crosses TestA$") == std::string::npos);
      TEST(res.find("$RefA crosses TestB$") == std::string::npos);
      TEST(res.find("$RefA intersects TestB$") != std::string::npos);
      TEST(res.find("$TestB intersects RefA$") != std::string::npos);
      TEST(res.find("$RefA intersects TestA$") != std::string::npos);
      TEST(res.find("$TestA intersects RefA$") != std::string::npos);
      TEST(res.find("$RefA covers TestA$") != std::string::npos);
      TEST(res.find("$RefA covers TestB$") != std::string::npos);
      TEST(res.find("$RefA covers TestC$") == std::string::npos);
      TEST(res.find("$RefB equals TestA$") != std::string::npos);
      TEST(res.find("$TestA equals RefB$") != std::string::npos);
      TEST(res.find("$<> equals RefG$") != std::string::npos);
      TEST(res.find("$RefG equals <>$") != std::string::npos);
      TEST(res.find("$RefG equals RefJ$") != std::string::npos);
      TEST(res.find("$<> equals RefJ$") != std::string::npos);
      TEST(res.find("$RefJ equals RefG$") != std::string::npos);
      TEST(res.find("$RefJ equals <>$") != std::string::npos);
    }

    {
      auto res = fullRun("../src/spatialjoin/tests/datasets/bawue", cfg);
      TEST(res.find("$germany covers bawue$") != std::string::npos);
      TEST(res.find("$germany contains bawue$") != std::string::npos);
    }
  }

  // DE9IM
  for (auto cfg : cfgs) {
    cfg.computeDE9IM = true;
    {
      auto res = fullRun("../src/spatialjoin/tests/datasets/freiburg", cfg);
      TEST(res.find("$freiburg1\t2FFF1FFF2\tfreiburg2$") != std::string::npos);
      TEST(res.find("$freiburg2\t2FFF1FFF2\tfreiburg1$") != std::string::npos);
      TEST(res.find("$freiburg1\t2FFF1FFF2\tfreiburg2$") != std::string::npos);
      TEST(res.find("$freiburg2\t2FFF1FFF2\tfreiburg1$") != std::string::npos);

      TEST(res.find("$freiburg1\tFF210FFF2\tgrenzpart$") != std::string::npos);
      TEST(res.find("$grenzpart\tF1FF0F2F2\tfreiburg1$") != std::string::npos);

      TEST(res.find("$Umkirch\tFF2F11212\tfreiburg1$") != std::string::npos);
      TEST(res.find("$freiburg1\t212FF1FF2\tAltstadt$") != std::string::npos);
    }
  }
}
