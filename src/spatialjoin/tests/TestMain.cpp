// Copyright 2024
// Author: Patrick Brosi

#include <string>
#include <iostream>

#include "spatialjoin/BoxIds.h"
#include "spatialjoin/Sweeper.h"
#include "spatialjoin/WKTParse.h"
#include "util/Misc.h"
#include "util/geo/Geo.h"
#include "util/log/Log.h"

using sj::Sweeper;

// _____________________________________________________________________________
std::string fullRun(const std::string& file) {
  Sweeper sweeper(
      {1, "$", " intersects ", " contains ", " covers ", "$\n", true, true},
      false, ".", ".resTmp");

  size_t gid = 0;

  char* buf = new char[1024 * 1024 * 100];

  size_t len = 0;
  std::string dangling;

  int f = open(file.c_str(), O_RDONLY);
  TEST(f >=0);

  while ((len = read(f, buf, 1024 * 1024 * 100)) > 0) {
    parse(buf, len, dangling, &gid, sweeper);
  }

  sweeper.flush();
  sweeper.sweep();

  std::string ret;

  std::stringstream ss;
  std::ifstream ifs(".resTmp");
  ss << ifs.rdbuf();

  unlink("tmp");

  return ss.str();
}

// _____________________________________________________________________________
int main(int, char**) {
  {
  auto res = fullRun("../src/spatialjoin/tests/datasets/freiburg");

  TEST(res.find("$freiburg1 covers freiburg2$") != std::string::npos);
  TEST(res.find("$freiburg2 covers freiburg1$") != std::string::npos);
  TEST(res.find("$freiburg1 contains freiburg2$") == std::string::npos);
  TEST(res.find("$freiburg2 contains freiburg1$") == std::string::npos);
  TEST(res.find("$freiburg1 intersects freiburg2$") != std::string::npos);
  TEST(res.find("$freiburg2 intersects freiburg1$") != std::string::npos);

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
  TEST(res.find("$freiburg1 intersects Sankt Georgen$") != std::string::npos);
  TEST(res.find("$freiburg1 contains Sankt Georgen$") == std::string::npos);

  TEST(res.find("$freiburg1 covers Haslach$") != std::string::npos);

  TEST(res.find("$freiburg1 covers Günterstal$") != std::string::npos);
  TEST(res.find("$freiburg1 intersects Günterstal$") != std::string::npos);
  TEST(res.find("$freiburg1 contains Günterstal$") == std::string::npos);

  TEST(res.find("$freiburg1 covers Kappel$") != std::string::npos);
  TEST(res.find("$freiburg1 intersects Kappel$") != std::string::npos);
  TEST(res.find("$freiburg1 contains Kappel$") == std::string::npos);

  TEST(res.find("$freiburg1 covers Littenweiler$") != std::string::npos);
  TEST(res.find("$freiburg1 intersects Littenweiler$") != std::string::npos);
  TEST(res.find("$freiburg1 contains Littenweiler$") == std::string::npos);

  TEST(res.find("$freiburg1 covers Waldsee$") != std::string::npos);
  TEST(res.find("$freiburg1 intersects Waldsee$") != std::string::npos);
  TEST(res.find("$freiburg1 contains Waldsee$") == std::string::npos);

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
  TEST(res.find("$freiburg1 contains Munzingen$") == std::string::npos);

  TEST(res.find("$freiburg1 covers Tiengen$") != std::string::npos);
  TEST(res.find("$freiburg1 intersects Tiengen$") != std::string::npos);
  TEST(res.find("$freiburg1 contains Tiengen$") == std::string::npos);

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
  TEST(res.find("$freiburg1 covers Sankt Georgen Süd$") != std::string::npos);
  TEST(res.find("$freiburg1 covers Sankt Georgen Nord$") != std::string::npos);
  TEST(res.find("$Sankt Georgen covers Sankt Georgen Süd$") != std::string::npos);
  TEST(res.find("$Sankt Georgen covers Sankt Georgen Nord$") != std::string::npos);
  TEST(res.find("$freiburg1 covers Haslach-Haid$") != std::string::npos);
  TEST(res.find("$freiburg1 covers Haslach-Gartenstadt$") != std::string::npos);
  TEST(res.find("$Haslach covers Haslach-Haid$") != std::string::npos);
  TEST(res.find("$Haslach covers Haslach-Gartenstadt$") != std::string::npos);
  TEST(res.find("$Haslach covers Haslach-Schildacker$") != std::string::npos);
  TEST(res.find("$Haslach covers Haslach-Egerten$") != std::string::npos);
  TEST(res.find("$freiburg1 covers Haslach-Schildacker$") != std::string::npos);
  TEST(res.find("$freiburg1 covers Haslach-Egerten$") != std::string::npos);
  TEST(res.find("$freiburg1 covers Unterwiehre-Nord$") != std::string::npos);
  TEST(res.find("$freiburg1 covers Unterwiehre-Süd$") != std::string::npos);
  TEST(res.find("$Wiehre covers Unterwiehre-Nord$") != std::string::npos);
  TEST(res.find("$Wiehre covers Unterwiehre-Süd$") != std::string::npos);
  TEST(res.find("$freiburg1 covers Altstadt-Ring$") != std::string::npos);
  TEST(res.find("$freiburg1 covers Alt-Stühlinger$") != std::string::npos);
  TEST(res.find("$freiburg1 covers Betzenhausen-Bischofslinde$") != std::string::npos);
  TEST(res.find("$freiburg1 covers Stühlinger-Eschholz$") != std::string::npos);
  TEST(res.find("$freiburg1 covers Mooswald-Ost$") != std::string::npos);
  TEST(res.find("$freiburg1 covers Brühl-Beurbarung$") != std::string::npos);
  TEST(res.find("$freiburg1 covers Brühl-Industriegebiet$") != std::string::npos);
  TEST(res.find("$freiburg1 covers Altstadt-Mitte$") != std::string::npos);
  TEST(res.find("$freiburg1 covers Mittelwiehre$") != std::string::npos);
  TEST(res.find("$freiburg1 covers Oberwiehre$") != std::string::npos);
  TEST(res.find("$Wiehre covers Mittelwiehre$") != std::string::npos);
  TEST(res.find("$Wiehre contains Mittelwiehre$") == std::string::npos);
  TEST(res.find("$Wiehre covers Oberwiehre$") != std::string::npos);
  TEST(res.find("$Wiehre contains Oberwiehre$") == std::string::npos);
  TEST(res.find("$freiburg1 covers Alt-Betzenhausen$") != std::string::npos);
  TEST(res.find("$freiburg1 covers Mooswald-West$") != std::string::npos);
  TEST(res.find("$freiburg1 covers Brühl-Güterbahnhof$") != std::string::npos);
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
  auto res = fullRun("../src/spatialjoin/tests/datasets/brandenburg");
  TEST(res.find("$Brandenburg covers Brandenburg2$") != std::string::npos);
  TEST(res.find("$Brandenburg intersects Grenzpart$") != std::string::npos);
  TEST(res.find("$Brandenburg covers Grenzpart$") != std::string::npos);
  TEST(res.find("$Brandenburg contains Grenzpart$") == std::string::npos);

  TEST(res.find("$Brandenburg intersects Berlin$") != std::string::npos);
  TEST(res.find("$Brandenburg contains Berlin$") == std::string::npos);
  TEST(res.find("$Brandenburg covers Berlin$") == std::string::npos);

  TEST(res.find("$Berlin intersects Grenzpart$") != std::string::npos);
  TEST(res.find("$Berlin covers Grenzpart$") != std::string::npos);
  TEST(res.find("$Berlin contains Grenzpart$") == std::string::npos);
}
}
