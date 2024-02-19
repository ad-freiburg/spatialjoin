// Copyright 2016
// Author: Patrick Brosi
//

#include <string>
#include "util/Misc.h"
#include "util/Nullable.h"
#include "util/String.h"
#include "util/tests/QuadTreeTest.h"
#include "util/geo/Geo.h"
#include "util/geo/Grid.h"
#include "util/graph/Algorithm.h"
#include "util/graph/Dijkstra.h"
#include "util/graph/BiDijkstra.h"
#include "util/graph/DirGraph.h"
#include "util/graph/EDijkstra.h"
#include "util/graph/UndirGraph.h"
#include "util/json/Writer.h"

using namespace util;
using namespace util::geo;
using namespace util::graph;

using util::approx;

// _____________________________________________________________________________
int main(int argc, char** argv) {
	UNUSED(argc);
	UNUSED(argv);

  std::setlocale(LC_ALL, "en_US.utf8");

  QuadTreeTest quadTreeTest;
  quadTreeTest.run();

  // x sorted polygons
  // ___________________________________________________________________________
  {
  Polygon<int> poly1({Point<int>(1, 1), Point<int>(3, 2),
                        Point<int>(4, 3), Point<int>(6, 3),
                        Point<int>(5, 1), Point<int>(1, 1)});

  Polygon<int> poly2({Point<int>(6, 2), Point<int>(5, 4),
                        Point<int>(7, 4), Point<int>(6, 2)});

  Polygon<int> poly3({Point<int>(1, 3), Point<int>(1, 4),
                        Point<int>(2, 4), Point<int>(2, 3), Point<int>(1, 3)});

  Polygon<int> poly4({Point<int>(0, 2), Point<int>(0, 5),
                        Point<int>(3, 5), Point<int>(3, 2), Point<int>(0, 2)});

  Polygon<int> poly5({Point<int>(-1, 5), Point<int>(4, 0),
                        Point<int>(-1, 5)});

  LineSegment<int64_t> a{{9426978,71128476},{9432711,71047835}};
  LineSegment<int64_t> b{{9377553,71093079},{9537853,71093079}};
  LineSegment<int64_t> c{{9377553,71253379},{9537853,71253379}};

  std::set<LineSegment<int64_t>> testSet{b, c};

  TEST(b < a);
  TEST(!(b > a));
  TEST(a > b);
  TEST(!(a < b));

  TEST(a < c);
  TEST(!(c < a));
  TEST(!(a > c));
  TEST(c > a);
  TEST(!(c < a));

  TEST(*testSet.lower_bound(a) == c);


  XSortedPolygon<int> spoly1(poly1);
  XSortedPolygon<int> spoly2(poly2);
  XSortedPolygon<int> spoly3(poly3);
  XSortedPolygon<int> spoly4(poly4);
  XSortedPolygon<int> spoly5(poly5);

  ///

    TEST(geo::intersects(spoly1.getOuter(), spoly2.getOuter()));
    TEST(geo::intersects(spoly2.getOuter(), spoly1.getOuter()));

    TEST(!geo::intersects(spoly1.getOuter(), spoly3.getOuter()));
    TEST(!geo::intersects(spoly3.getOuter(), spoly1.getOuter()));

    // 3 is contained in 4
    TEST(!geo::intersects(spoly4.getOuter(), spoly3.getOuter()));
    TEST(!geo::intersects(spoly3.getOuter(), spoly4.getOuter()));

    // 1 does not intersect in 4
    TEST(!geo::intersects(spoly4.getOuter(), spoly1.getOuter()));
    TEST(!geo::intersects(spoly1.getOuter(), spoly4.getOuter()));

    // 2 does not intersect in 3
    TEST(!geo::intersects(spoly3.getOuter(), spoly2.getOuter()));
    TEST(!geo::intersects(spoly2.getOuter(), spoly3.getOuter()));

    // 2 does not intersect in 4
    TEST(!geo::intersects(spoly4.getOuter(), spoly2.getOuter()));
    TEST(!geo::intersects(spoly2.getOuter(), spoly4.getOuter()));

    // 5 intersects 4
    TEST(geo::intersects(spoly4.getOuter(), spoly5.getOuter()));
    TEST(geo::intersects(spoly5.getOuter(), spoly4.getOuter()));

    // 5 intersects 3
    TEST(geo::intersects(spoly3.getOuter(), spoly5.getOuter()));
    TEST(geo::intersects(spoly5.getOuter(), spoly3.getOuter()));

    // 5 intersects 1
    TEST(geo::intersects(spoly1.getOuter(), spoly5.getOuter()));
    TEST(geo::intersects(spoly5.getOuter(), spoly1.getOuter()));

    // 5 does intersects 2
    TEST(!geo::intersects(spoly2.getOuter(), spoly5.getOuter()));
    TEST(!geo::intersects(spoly5.getOuter(), spoly2.getOuter()));

    ///

    TEST(geo::intersects(spoly1.getOuter(), spoly2.getOuter(), spoly1.getOuter().getMaxSegLen(), spoly2.getOuter().getMaxSegLen()));
    TEST(geo::intersects(spoly2.getOuter(), spoly1.getOuter(), spoly2.getOuter().getMaxSegLen(), spoly1.getOuter().getMaxSegLen()));

    TEST(geo::intersects(spoly1.getOuter(), spoly2.getOuter(), spoly1.getOuter().getMaxSegLen(), spoly2.getOuter().getMaxSegLen(), util::geo::getBoundingBox(poly1), util::geo::getBoundingBox(poly2)));
    TEST(geo::intersects(spoly2.getOuter(), spoly1.getOuter(), spoly2.getOuter().getMaxSegLen(), spoly1.getOuter().getMaxSegLen(), util::geo::getBoundingBox(poly2), util::geo::getBoundingBox(poly1)));

    TEST(!geo::intersects(spoly1.getOuter(), spoly3.getOuter(), spoly1.getOuter().getMaxSegLen(), spoly3.getOuter().getMaxSegLen()));
    TEST(!geo::intersects(spoly3.getOuter(), spoly1.getOuter(), spoly3.getOuter().getMaxSegLen(), spoly1.getOuter().getMaxSegLen()));

    TEST(!geo::intersects(spoly1.getOuter(), spoly3.getOuter(), spoly1.getOuter().getMaxSegLen(), spoly3.getOuter().getMaxSegLen(), util::geo::getBoundingBox(poly1), util::geo::getBoundingBox(poly3)));
    TEST(!geo::intersects(spoly3.getOuter(), spoly1.getOuter(), spoly3.getOuter().getMaxSegLen(), spoly1.getOuter().getMaxSegLen(), util::geo::getBoundingBox(poly3), util::geo::getBoundingBox(poly1)));

    // 3 is contained in 4
    TEST(!geo::intersects(spoly4.getOuter(), spoly3.getOuter(), spoly4.getOuter().getMaxSegLen(), spoly3.getOuter().getMaxSegLen()));
    TEST(!geo::intersects(spoly3.getOuter(), spoly4.getOuter(), spoly3.getOuter().getMaxSegLen(), spoly4.getOuter().getMaxSegLen()));

    TEST(!geo::intersects(spoly4.getOuter(), spoly3.getOuter(), spoly4.getOuter().getMaxSegLen(), spoly3.getOuter().getMaxSegLen(), util::geo::getBoundingBox(poly4), util::geo::getBoundingBox(poly3)));
    TEST(!geo::intersects(spoly3.getOuter(), spoly4.getOuter(), spoly3.getOuter().getMaxSegLen(), spoly4.getOuter().getMaxSegLen(), util::geo::getBoundingBox(poly3), util::geo::getBoundingBox(poly4)));

    // 1 does not intersect in 4
    TEST(!geo::intersects(spoly4.getOuter(), spoly1.getOuter(), spoly4.getOuter().getMaxSegLen(), spoly1.getOuter().getMaxSegLen()));
    TEST(!geo::intersects(spoly1.getOuter(), spoly4.getOuter(), spoly1.getOuter().getMaxSegLen(), spoly4.getOuter().getMaxSegLen()));

    TEST(!geo::intersects(spoly4.getOuter(), spoly1.getOuter(), spoly4.getOuter().getMaxSegLen(), spoly1.getOuter().getMaxSegLen(), util::geo::getBoundingBox(poly4), util::geo::getBoundingBox(poly1)));
    TEST(!geo::intersects(spoly1.getOuter(), spoly4.getOuter(), spoly1.getOuter().getMaxSegLen(), spoly4.getOuter().getMaxSegLen(), util::geo::getBoundingBox(poly1), util::geo::getBoundingBox(poly4)));

    // 2 does not intersect in 3
    TEST(!geo::intersects(spoly3.getOuter(), spoly2.getOuter(), spoly3.getOuter().getMaxSegLen(), spoly2.getOuter().getMaxSegLen()));
    TEST(!geo::intersects(spoly2.getOuter(), spoly3.getOuter(), spoly2.getOuter().getMaxSegLen(), spoly3.getOuter().getMaxSegLen()));

    // 2 does not intersect in 4
    TEST(!geo::intersects(spoly4.getOuter(), spoly2.getOuter(), spoly4.getOuter().getMaxSegLen(), spoly2.getOuter().getMaxSegLen()));
    TEST(!geo::intersects(spoly2.getOuter(), spoly4.getOuter(), spoly2.getOuter().getMaxSegLen(), spoly4.getOuter().getMaxSegLen()));

    // 5 intersects 4
    TEST(geo::intersects(spoly4.getOuter(), spoly5.getOuter(), spoly4.getOuter().getMaxSegLen(), spoly5.getOuter().getMaxSegLen()));
    TEST(geo::intersects(spoly5.getOuter(), spoly4.getOuter(), spoly5.getOuter().getMaxSegLen(), spoly4.getOuter().getMaxSegLen()));

    // 5 intersects 3
    TEST(geo::intersects(spoly3.getOuter(), spoly5.getOuter(), spoly3.getOuter().getMaxSegLen(), spoly5.getOuter().getMaxSegLen()));
    TEST(geo::intersects(spoly5.getOuter(), spoly3.getOuter(), spoly5.getOuter().getMaxSegLen(), spoly3.getOuter().getMaxSegLen()));

    // 5 intersects 1
    TEST(geo::intersects(spoly1.getOuter(), spoly5.getOuter()));
    TEST(geo::intersects(spoly5.getOuter(), spoly1.getOuter()));

    // 5 does intersects 2
    TEST(!geo::intersects(spoly2.getOuter(), spoly5.getOuter()));
    TEST(!geo::intersects(spoly5.getOuter(), spoly2.getOuter()));

    // point contains tests
    TEST(geo::contains(Point<int>(4, 2), spoly1));
    TEST(!geo::contains(Point<int>(3, 3), spoly1));

    TEST(geo::contains(Point<int>(6, 3), spoly2));
    TEST(!geo::contains(Point<int>(5, 2), spoly2));

    auto ap = multiPolygonFromWKT<double>("MULTIPOLYGON(((6.2120237 51.5133909,6.2127637 51.4913082,6.2190321 51.4849833,6.2236759 51.4687148,6.2205935 51.4466463,6.2139367 51.4462637,6.2147615 51.4338807,6.2052570 51.3995201,6.2266681 51.4002680,6.2144248 51.3896343,6.2263867 51.3603360,6.1898779 51.3394613,6.1942347 51.3348784,6.1685573 51.3329887,6.1693785 51.3293737,6.1595899 51.3196913,6.1538213 51.3074476,6.1290121 51.2857005,6.1244995 51.2747284,6.0856172 51.2476261,6.0726534 51.2425555,6.0860218 51.2226673,6.0679863 51.2205478,6.0731453 51.1827944,6.0821902 51.1716335,6.1000891 51.1698870,6.1224059 51.1813028,6.1651704 51.1944141,6.1807243 51.1863585,6.1388017 51.1733392,6.1754219 51.1584640,6.1628485 51.1526712,6.1633395 51.1486975,6.1258299 51.1451168,6.1163882 51.1391547,6.0919960 51.1350476,6.0871032 51.1293025,6.0887723 51.1278975,6.0844241 51.1260187,6.0869921 51.1245949,6.0809012 51.1259794,6.0805510 51.1221417,6.0758834 51.1191919,6.0602933 51.1159264,6.0552379 51.1107423,6.0568320 51.1095496,6.0366919 51.0965942,6.0206756 51.0928383,6.0175749 51.0945726,5.9977935 51.0842457,5.9884518 51.0746139,5.9802940 51.0723171,5.9817297 51.0694779,5.9699784 51.0607039,5.9691978 51.0467722,5.9578301 51.0409811,5.9577783 51.0347261,5.9498013 51.0369672,5.9381295 51.0351172,5.9263166 51.0482309,5.9187317 51.0639528,5.9132078 51.0668830,5.8920004 51.0531278,5.8667370 51.0515704,5.8672852 51.0462799,5.8780282 51.0375835,5.8770005 51.0320418,5.8567106 51.0283384,5.8526142 51.0293181,5.8496511 51.0352277,5.8527429 51.0381243,5.8485262 51.0463281,5.8272517 51.0475237,5.8239605 51.0725105,5.8196232 51.0726099,5.8069344 51.0575720,5.7993389 51.0600718,5.8010586 51.0640196,5.7966202 51.0718244,5.8049752 51.0787873,5.7958236 51.0878501,5.7960227 51.0914785,5.8082580 51.0963390,5.8241431 51.0923501,5.8336723 51.1000264,5.8334053 51.1037509,5.8288865 51.1070398,5.8099984 51.1096468,5.8078497 51.1135931,5.8098503 51.1184222,5.8248993 51.1293451,5.8413514 51.1314787,5.8458406 51.1407588,5.8557834 51.1446260,5.8363544 51.1536850,5.8386107 51.1570866,5.8328812 51.1590374,5.8326946 51.1624299,5.8253200 51.1674270,5.8159712 51.1631708,5.8150566 51.1588482,5.8049355 51.1628762,5.7987806 51.1576670,5.7776462 51.1513039,5.7747592 51.1546822,5.7794826 51.1593831,5.7794024 51.1633163,5.7701886 51.1642239,5.7695650 51.1690833,5.7797360 51.1717992,5.7729522 51.1733698,5.7767286 51.1784855,5.7671605 51.1836911,5.7559633 51.1844607,5.7456413 51.1894990,5.7397405 51.1847477,5.7196764 51.1846996,5.7093393 51.1804207,5.6894115 51.1854196,5.6765411 51.1827256,5.6710369 51.1857682,5.6580417 51.1847425,5.6497400 51.1936177,5.6540832 51.1942417,5.6527338 51.1976610,5.5660454 51.2209094,5.6187816 51.2294253,5.6259723 51.2736016,5.6720479 51.3150820,5.8745432 51.3533127,5.9312876 51.3847527,5.8716953 51.4501120,5.8607237 51.4919665,5.8525522 51.5041766,5.8382385 51.5664146,5.8914662 51.5602047,5.9066627 51.5520309,5.9354450 51.5536002,6.0042529 51.5702435,6.0316689 51.5523388,6.0343207 51.5574973,6.0481208 51.5584625,6.0376590 51.5699586,6.0385724 51.5842855,6.0249435 51.5980637,6.0239180 51.6157805,6.0206301 51.6212914,5.9963523 51.6367191,5.9739312 51.6446108,5.9651055 51.6522503,5.9643928 51.6773720,5.9552911 51.7093049,5.9412963 51.7147757,5.8995668 51.7201899,5.8876474 51.7253981,5.8795605 51.7499136,5.8644689 51.7576817,5.8692448 51.7628605,5.8678933 51.7755210,5.8934093 51.7778529,5.9111146 51.7624057,5.9152848 51.7522869,5.9333232 51.7480986,5.9299483 51.7444285,5.9327669 51.7419384,5.9461343 51.7423614,5.9524661 51.7445538,5.9532879 51.7480242,5.9522944 51.7426841,5.9551552 51.7381176,5.9941968 51.7383094,6.0295222 51.7254848,6.0354244 51.7177743,6.0378809 51.7199298,6.0449392 51.7169134,6.0420092 51.7133446,6.0315200 51.7129896,6.0260539 51.7086881,6.0317806 51.6925333,6.0282478 51.6896244,6.0322344 51.6847963,6.0298231 51.6780994,6.0346725 51.6751459,6.0315405 51.6745827,6.0757244 51.6648257,6.0795451 51.6615933,6.0853449 51.6629141,6.0878884 51.6598498,6.0996545 51.6581159,6.1028380 51.6605047,6.1180876 51.6559729,6.1172476 51.6507311,6.1094122 51.6468665,6.1116826 51.6447300,6.1000122 51.6240785,6.0972292 51.6208835,6.0939341 51.6221540,6.0914239 51.6058486,6.1214855 51.5927445,6.1305600 51.5810876,6.1570325 51.5665755,6.1769020 51.5385557,6.1999290 51.5273814,6.2120237 51.5133909)))");

    auto bp = multiPolygonFromWKT<double>("MULTIPOLYGON(((5.8161104 51.1097054,5.8161127 51.1097311,5.8161193 51.1097563,5.8161651 51.1097780,5.8163299 51.1098021,5.8173204 51.1099397,5.8184653 51.1100911,5.8184926 51.1100910,5.8185151 51.1100881,5.8185332 51.1100765,5.8186024 51.1099088,5.8186329 51.1098348,5.8186576 51.1098019,5.8186686 51.1097819,5.8186956 51.1097633,5.8187296 51.1097502,5.8187565 51.1097473,5.8187974 51.1097429,5.8188698 51.1097352,5.8189396 51.1097220,5.8189779 51.1097233,5.8190199 51.1097237,5.8190595 51.1097244,5.8192236 51.1097277,5.8199365 51.1097572,5.8200295 51.1097684,5.8200863 51.1097881,5.8201750 51.1098335,5.8202380 51.1098656,5.8203005 51.1099025,5.8203312 51.1099198,5.8203496 51.1099256,5.8203790 51.1099283,5.8213954 51.1099088,5.8228084 51.1098864,5.8228694 51.1098760,5.8230269 51.1098536,5.8231398 51.1098742,5.8231083 51.1093755,5.8230140 51.1090536,5.8228888 51.1090689,5.8227712 51.1090494,5.8227237 51.1090052,5.8226520 51.1088723,5.8226199 51.1088084,5.8225177 51.1087594,5.8213256 51.1087121,5.8195553 51.1086471,5.8194630 51.1086528,5.8178802 51.1086039,5.8164549 51.1085495,5.8163851 51.1085470,5.8163390 51.1085662,5.8163165 51.1085756,5.8163054 51.1085949,5.8163005 51.1086109,5.8162167 51.1091479,5.8161170 51.1096889,5.8161104 51.1097054)))");

    Polygon<int> api;
    for (const auto& p : ap[0].getOuter()) {
      auto pp = latLngToWebMerc(p);
      api.getOuter().push_back({pp.getX() * 10, pp.getY() * 10});
    }

    Polygon<int> bpi;
    for (const auto& p : bp[0].getOuter()) {
      auto pp = latLngToWebMerc(p);
      bpi.getOuter().push_back({pp.getX() * 10, pp.getY() * 10});
    }

  XSortedPolygon<double> app(ap[0]);
  XSortedPolygon<double> bpp(bp[0]);

    TEST(geo::intersects(app.getOuter(), bpp.getOuter()));
    TEST(geo::intersects(app.getOuter(), bpp.getOuter(), app.getOuter().getMaxSegLen(), bpp.getOuter().getMaxSegLen()));
    TEST(geo::intersects(app.getOuter(), bpp.getOuter(), app.getOuter().getMaxSegLen(), bpp.getOuter().getMaxSegLen(), util::geo::getBoundingBox(ap[0]), util::geo::getBoundingBox(bp[0])));

  XSortedPolygon<int> appi(api);
  XSortedPolygon<int> bppi(bpi);

    TEST(geo::intersects(appi.getOuter(), bppi.getOuter()));
    TEST(geo::intersects(appi.getOuter(), bppi.getOuter(), appi.getOuter().getMaxSegLen(), bppi.getOuter().getMaxSegLen()));
    TEST(geo::intersects(appi.getOuter(), bppi.getOuter(), appi.getOuter().getMaxSegLen(), bppi.getOuter().getMaxSegLen(), util::geo::getBoundingBox(api), util::geo::getBoundingBox(bpi)));

    auto failtest = polygonFromWKT<int>("POLYGON((1 6, 3 1, 7 1, 8 8, 1 8, 1 6))");
    auto bboxfail = polygonFromWKT<int>("POLYGON((2 2, 6 2, 6 6, 2 6, 2 2))");

    XSortedPolygon<int> failtestx(failtest);
    XSortedPolygon<int> bboxfailx(bboxfail);

    TEST(geo::intersects(failtest.getOuter(), bboxfail.getOuter()));
    TEST(geo::intersects(failtestx.getOuter(), bboxfailx.getOuter()));
    TEST(geo::intersects(bboxfailx.getOuter(), failtestx.getOuter()));

    auto bremer = polygonFromWKT<int>("POLYGON((1015809 7145218, 1012312 7144769, 1002806 7139896, 999939 7140594, 990566 7138626, 983285 7139256, 975967 7141358, 967081 7150512, 958806 7148906, 950876 7137498, 947207 7124657, 942697 7112847, 943271 7104783, 947659 7096463, 946968 7095405, 950408 7089285, 946959 7086008, 942172 7086314, 933754 7090164, 929359 7097636, 924945 7098974, 920492 7097197, 917320 7092782, 915594 7079870, 908884 7079883, 909849 7083046, 907594 7095477, 900084 7104114, 895517 7106541, 893739 7128746, 878146 7200635, 882580 7204925, 966909 7220260, 972527 7224139, 977476 7222917, 981986 7224770, 983763 7221751, 983075 7211566, 980973 7209253, 979769 7203807, 980438 7201476, 982941 7195781, 986018 7191864, 989572 7194272, 992248 7194826, 993757 7192896, 995095 7186647, 996968 7181660, 994522 7178870, 988177 7179768, 980935 7176424, 984126 7167805, 988216 7160200, 994178 7152327, 997770 7149613, 1000264 7149050, 1003321 7149289, 1005576 7150397, 1012656 7148429, 1015828 7148352, 1015809 7145218, 1015809 7145218))");
    auto bbox = polygonFromWKT<int>("POLYGON((889665 7133352, 905695 7133352, 905695 7149382, 889665 7149382, 889665 7133352))");

    XSortedPolygon<int> bremerx(bremer);
    XSortedPolygon<int> bboxx(bbox);

    TEST(geo::intersects(bbox.getOuter(), bremer.getOuter()));
    TEST(geo::intersectsNaive(bremerx.getOuter(), bboxx.getOuter()));
    TEST(geo::intersects(bremerx.getOuter().rawRing(), bboxx.getOuter().rawRing()));
    TEST(geo::intersects(bremerx.getOuter(), bboxx.getOuter()));
    TEST(geo::intersects(bboxx.getOuter(), bremerx.getOuter()));

    auto wattbbox = polygonFromWKT<int>("POLYGON ((7614253 71173229, 7774553 71173229, 7774553 71333529, 7614253 71333529, 7614253 71173229))");


    auto watt = polygonFromWKT<int>("POLYGON ((7325750 71235260, 7326028 71277936, 7791418 71378183, 7875482 71367056, 8146422 71330500, 8146422 71221663, 7792364 71144836, 7792364 71346733, 7325750 71235260, 7325750 71235260))");

    XSortedPolygon<int> wattx(watt);
    XSortedPolygon<int> wattbboxx(wattbbox);

    TEST(geo::intersects(watt.getOuter(), wattbbox.getOuter()));
    TEST(geo::intersects(wattx.getOuter(), wattbboxx.getOuter()));
    TEST(geo::intersects(wattbboxx.getOuter(), wattx.getOuter()));

    TEST(geo::intersects(wattx.getOuter(), wattx.getOuter()));
    TEST(geo::intersects(wattbboxx.getOuter(), wattbboxx.getOuter()));


    auto wattbbox2 = polygonFromWKT<int>("POLYGON ((913710 7093277, 937755 7093277, 937755 7109307, 913710 7109307, 913710 7093277))");
    auto watt2 = polygonFromWKT<int>("POLYGON ((1015809 7145218, 1012312 7144769, 1002806 7139896, 999939 7140594, 990566 7138626, 983285 7139256, 975967 7141358, 967081 7150512, 958806 7148906, 950876 7137498, 947207 7124657, 942697 7112847, 943271 7104783, 947659 7096463, 946968 7095405, 950408 7089285, 946959 7086008, 942172 7086314, 933754 7090164, 929359 7097636, 924945 7098974, 920492 7097197, 917320 7092782, 915594 7079870, 908884 7079883, 909849 7083046, 907594 7095477, 900084 7104114, 895517 7106541, 893739 7128746, 878146 7200635, 882580 7204925, 966909 7220260, 972527 7224139, 977476 7222917, 981986 7224770, 983763 7221751, 983075 7211566, 980973 7209253, 979769 7203807, 980438 7201476, 982941 7195781, 986018 7191864, 989572 7194272, 992248 7194826, 993757 7192896, 995095 7186647, 996968 7181660, 994522 7178870, 988177 7179768, 980935 7176424, 984126 7167805, 988216 7160200, 994178 7152327, 997770 7149613, 1000264 7149050, 1003321 7149289, 1005576 7150397, 1012656 7148429, 1015828 7148352, 1015809 7145218, 1015809 7145218)) ");

    XSortedPolygon<int> wattx2(watt2);
    XSortedPolygon<int> wattbboxx2(wattbbox2);

    TEST(geo::intersects(watt2.getOuter(), wattbbox2.getOuter()));
    TEST(geo::intersects(wattx2.getOuter(), wattbboxx2.getOuter()));
    TEST(geo::intersects(wattbboxx2.getOuter(), wattx2.getOuter()));


    auto anothertest = polygonFromWKT<int64_t>("POLYGON ((15482959 73076173, 15596431 73176015, 15706928 73045744, 15749740 73081874, 15870403 73277506, 16051803 73257507, 16026155 73024804, 16292312 72722538, 16419624 72654248, 16320974 72522587, 16249995 72429299, 15974662 72019053, 15849195 71940131, 15849066 71940131, 15806390 72177589, 15768752 72203056, 15758857 72275088, 15702653 72305851, 15709997 72540286, 15671882 72541478, 15609994 72860919, 15598659 72894904, 15582016 72940327, 15565146 72972142, 15551031 72994672, 15532330 73020790, 15498495 73060139, 15482959 73076173, 15482959 73076173))");
    auto anotherbox = polygonFromWKT<int64_t>("POLYGON ((15949856 72696080, 16270456 72696080, 16270456 73176980, 15949856 73176980, 15949856 72696080))");

    XSortedPolygon<int64_t> anothertestx(anothertest);
    XSortedPolygon<int64_t> anotherboxx(anotherbox);

    TEST(geo::intersects(anothertest.getOuter(), anotherbox.getOuter()));
    TEST(geo::intersects(anothertestx.getOuter(), anotherboxx.getOuter()));
    TEST(geo::intersects(anotherboxx.getOuter(), anothertestx.getOuter()));


    auto anothertest2 = polygonFromWKT<int64_t>("POLYGON ((10158098 71452187, 10123128 71447696, 10028060 71398968, 9999396 71405943, 9905665 71386260, 9832859 71392566, 9759670 71413587, 9670812 71505120, 9588069 71489068, 9508765 71374986, 9472076 71246572, 9426978 71128476, 9432711 71047835, 9476590 70964638, 9469687 70954056, 9504084 70892859, 9469592 70860087, 9421723 70863144, 9337547 70901649, 9293595 70976367, 9249453 70989743, 9204928 70971972, 9173207 70927829, 9155949 70798708, 9088840 70798834, 9098490 70830467, 9075941 70954773, 9000842 71041147, 8955170 71065416, 8937399 71287466, 8781467 72006356, 8825801 72049256, 9669092 72202608, 9725274 72241399, 9774767 72229170, 9819864 72247706, 9837636 72217513, 9830757 72115661, 9809736 72092538, 9797698 72038077, 9804386 72014764, 9829419 71957818, 9860185 71918644, 9895728 71942722, 9922481 71948263, 9937577 71928963, 9950954 71866476, 9969681 71816601, 9945221 71788701, 9881778 71797683, 9809354 71764241, 9841266 71678059, 9882161 71602004, 9941781 71523274, 9977707 71496139, 10002645 71490501, 10033219 71492890, 10055768 71503973, 10126568 71484291, 10158289 71483526, 10158098 71452187, 10158098 71452187))");
    auto anotherbox2 = polygonFromWKT<int64_t>("POLYGON ((8976803 70932779, 9137103 70932779, 9137103 71093079, 8976803 71093079, 8976803 70932779))");

    Box<int64_t> anotherbox2m{{8976803,70932779}, {9137103,71093079}};

    XSortedPolygon<int64_t> anothertestx2(anothertest2);
    XSortedPolygon<int64_t> anotherboxx2(anotherbox2);
    XSortedPolygon<int64_t> anotherboxx2m(anotherbox2m);
    XSortedPolygon<int64_t> anotherboxx2mp(Polygon<int64_t>{anotherbox2m});

    TEST(geo::intersects(anothertest2.getOuter(), anotherbox2.getOuter()));
    TEST(geo::intersects(anothertestx2.getOuter(), anotherboxx2.getOuter()));
    TEST(geo::intersects(anotherboxx2.getOuter(), anothertestx2.getOuter()));

    TEST(geo::intersects(anothertestx2.getOuter(), anotherboxx2m.getOuter()));
    TEST(geo::intersects(anotherboxx2m.getOuter(), anothertestx2.getOuter()));

    TEST(geo::intersects(anothertestx2.getOuter(), anotherboxx2mp.getOuter()));
    TEST(geo::intersects(anotherboxx2mp.getOuter(), anothertestx2.getOuter()));

    {

    auto anothertest2 = polygonFromWKT<int64_t>("POLYGON ((10158098 71452187, 10123128 71447696, 10028060 71398968, 9999396 71405943, 9905665 71386260, 9832859 71392566, 9759670 71413587, 9670812 71505120, 9588069 71489068, 9508765 71374986, 9472076 71246572,9426978 71128476, 9432711 71047835, 9476590 70964638, 9469687 70954056, 9504084 70892859, 9469592 70860087, 9421723 70863144, 9337547 70901649, 9293595 70976367, 9249453 70989743, 9204928 70971972, 9173207 70927829, 9155949 70798708, 9088840 70798834, 9098490 70830467, 9075941 70954773, 9000842 71041147, 8955170 71065416, 8937399 71287466, 8781467 72006356, 8825801 72049256, 9669092 72202608, 9725274 72241399, 9774767 72229170, 9819864 72247706, 9837636 72217513, 9830757 72115661, 9809736 72092538, 9797698 72038077, 9804386 72014764, 9829419 71957818, 9860185 71918644, 9895728 71942722, 9922481 71948263, 9937577 71928963, 9950954 71866476, 9969681 71816601, 9945221 71788701, 9881778 71797683, 9809354 71764241, 9841266 71678059, 9882161 71602004, 9941781 71523274, 9977707 71496139, 10002645 71490501, 10033219 71492890, 10055768 71503973, 10126568 71484291, 10158289 71483526, 10158098 71452187, 10158098 71452187))");
    auto anotherbox2 = polygonFromWKT<int64_t>("POLYGON ((9377553 71093079, 9537853 71093079, 9537853 71253379, 9377553 71253379, 9377553 71093079))");


    Box<int64_t> anotherbox2m{{8976803,70932779}, {9137103,71093079}};

    XSortedPolygon<int64_t> anothertestx2(anothertest2);
    XSortedPolygon<int64_t> anotherboxx2(anotherbox2);
    XSortedPolygon<int64_t> anotherboxx2m(anotherbox2m);
    XSortedPolygon<int64_t> anotherboxx2mp(Polygon<int64_t>{anotherbox2m});

    TEST(geo::intersects(anothertest2.getOuter(), anotherbox2.getOuter()));
    TEST(geo::intersects(anothertestx2.getOuter(), anotherboxx2.getOuter()));
    TEST(geo::intersects(anotherboxx2.getOuter(), anothertestx2.getOuter()));

    TEST(geo::intersects(anothertestx2.getOuter(), anotherboxx2m.getOuter()));
    TEST(geo::intersects(anotherboxx2m.getOuter(), anothertestx2.getOuter()));

    TEST(geo::intersects(anothertestx2.getOuter(), anotherboxx2mp.getOuter()));
    TEST(geo::intersects(anotherboxx2mp.getOuter(), anothertestx2.getOuter()));
    }

    {

    auto anothertest2 = polygonFromWKT<int64_t>("POLYGON ((8846056 67477080, 8846056 67496954, 8882746 67520649, 8917142 67545873, 8921729 67570333, 8903384 67582563, 8879688 67591736, 8870516 67617724, 8882746 67636069, 8898033 67662822, 8897804 67690470, 8875102 67756075, 8900326 67781299, 8946953 67789707, 8982878 67787414, 9037149 67769069, 9093712 67729322, 9138810 67696454, 9191551 67696454, 9202595 67673506, 9210957 67647543, 9213769 67600456, 9215502 67564186, 9194609 67514534, 9037149 67471730, 8983643 67462557, 8931666 67458735, 8863975 67457113, 8851706 67460758, 8846056 67477080, 8846056 67477080))");
    auto anotherbox2 = polygonFromWKT<int64_t>("POLYGON ((8896653 67486328, 9056953 67486328, 9056953 67646628, 8896653 67646628, 8896653 67486328))");

    Box<int64_t> anotherbox2m{{8896653,67486328}, {9056953,67646628}};

    XSortedPolygon<int64_t> anothertestx2(anothertest2);
    XSortedPolygon<int64_t> anotherboxx2(anotherbox2);
    XSortedPolygon<int64_t> anotherboxx2m(anotherbox2m);
    XSortedPolygon<int64_t> anotherboxx2mp(Polygon<int64_t>{anotherbox2m});

    TEST(geo::intersects(LineSegment<int64_t>{{8896653,67486328}, {8896653,67646628}},LineSegment<int64_t>{{8903384,67582563}, {8879688,67591736}}));

    TEST(geo::intersects(anothertestx2.getOuter(), anotherboxx2.getOuter()));
    TEST(geo::intersects(anotherboxx2.getOuter(), anothertestx2.getOuter()));

    TEST(geo::intersects(anothertestx2.getOuter(), anotherboxx2m.getOuter()));
    TEST(geo::intersects(anotherboxx2m.getOuter(), anothertestx2.getOuter()));

    TEST(geo::intersects(anothertestx2.getOuter(), anotherboxx2mp.getOuter()));
    TEST(geo::intersects(anotherboxx2mp.getOuter(), anothertestx2.getOuter()));
    }


    {
    auto a = polygonFromWKT<int>("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0))");
    auto b = polygonFromWKT<int>("POLYGON((4 4, 5 4, 5 5, 4 5, 4 4))");

    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(!geo::intersects(ax.getOuter(), bx.getOuter()));
    TEST(geo::contains(b, a));
    TEST(geo::intersects(a, b));
    TEST(geo::intersectsContains(ax, bx).first);
    TEST(!geo::intersectsContains(ax, bx).second);
    TEST(geo::intersectsContains(bx, ax).first);
    TEST(geo::intersectsContains(bx, ax).second);
    }

    {
    auto a = polygonFromWKT<int>("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0), (1 1, 9 1, 9 9, 1 9, 1 1))");
    auto b = polygonFromWKT<int>("POLYGON((4 4, 5 4, 5 5, 4 5, 4 4))");

    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(!geo::intersects(ax.getOuter(), bx.getOuter()));
    TEST(!geo::contains(b, a));
    TEST(!geo::contains(a, b));
    TEST(!geo::intersects(a, b));
    TEST(!geo::intersects(b, a));
    auto res = intersectsContains(bx.getOuter(), ax.getInners().front());
    TEST(std::get<1>(res));
    TEST(!geo::intersectsContains(bx, ax).first);
    TEST(!geo::intersectsContains(bx, ax).second);
    TEST(!geo::intersectsContains(ax, bx).first);
    TEST(!geo::intersectsContains(ax, bx).second);
    }

    {
    auto a = polygonFromWKT<int>("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0), (4 4, 5 4, 5 5, 4 5, 4 4))");
    auto b = polygonFromWKT<int>("POLYGON((1 1, 9 1, 9 9, 1 9, 1 1), (3 3, 6 3, 6 6, 3 6, 3 3))");

    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(!geo::intersects(ax.getOuter(), bx.getOuter()));
    TEST(geo::contains(b, a));
    TEST(!geo::contains(a, b));
    TEST(geo::intersects(a, b));
    TEST(geo::intersects(b, a));
    TEST(geo::intersectsContains(bx, ax).first);
    TEST(geo::intersectsContains(bx, ax).second);
    TEST(geo::intersectsContains(ax, bx).first);
    TEST(!geo::intersectsContains(ax, bx).second);
    }

    {
    auto a = polygonFromWKT<int>("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0), (4 4, 5 4, 4 5, 4 4))");
    auto b = polygonFromWKT<int>("POLYGON((4.1 4.1, 5 4, 5 5, 4 5, 4 4))");

    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(!geo::intersects(ax.getOuter(), bx.getOuter()));
    TEST(!geo::contains(b, a));
    TEST(!geo::contains(a, b));
    TEST(geo::intersects(a, b));
    TEST(geo::intersects(b, a));
    TEST(!std::get<1>(geo::intersectsContains(bx.getOuter(), ax.getInners().front())));
    TEST(std::get<2>(geo::intersectsContains(bx.getOuter(), ax.getInners().front())));
    TEST(geo::ringContains(bx.getOuter().rawRing().front().seg().first, ax.getOuter(), 0));
    TEST(geo::intersectsContains(bx, ax).first);
    TEST(!geo::intersectsContains(bx, ax).second);
    TEST(geo::intersectsContains(ax, bx).first);
    TEST(!geo::intersectsContains(ax, bx).second);
    }

    {
    auto a = polygonFromWKT<int>("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0), (4 4, 5 4, 4 5, 4 4))");
    auto b = pointFromWKT<int>("POINT(4.1 4.1)");

    XSortedPolygon<int> ax(a);

    TEST(ax.getInners().size() == 1);
    TEST(ax.getInnerBoxes().size() == 1);
    TEST(ax.getInnerAreas().size() == 1);
    TEST(!geo::contains(b, a));
    TEST(!geo::contains(b, ax));
    }

    {
    auto a = polygonFromWKT<int>("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0), (4 4, 5 4, 4 5, 4 4))");
    auto b = pointFromWKT<int>("POINT(3.9 3.9)");

    XSortedPolygon<int> ax(a);

    TEST(ax.getInners().size() == 1);
    TEST(ax.getInnerBoxes().size() == 1);
    TEST(ax.getInnerAreas().size() == 1);
    TEST(geo::contains(b, a));
    TEST(geo::contains(b, ax));
    }

    {
    auto a = polygonFromWKT<int32_t>("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0), (4 4, 5 4, 4 5, 4 4))");
    auto b = polygonFromWKT<int32_t>("POLYGON((4.1 4.1, 5 4, 5 5, 4 5, 4 4))");

    XSortedPolygon<int32_t> ax(a);
    XSortedPolygon<int32_t> bx(b);

    TEST(!geo::intersects(ax.getOuter(), bx.getOuter()));
    TEST(!geo::contains(b, a));
    TEST(!geo::contains(a, b));
    TEST(geo::intersects(a, b));
    TEST(geo::intersects(b, a));
    TEST(!std::get<1>(geo::intersectsContains(bx.getOuter(), ax.getInners().front())));
    TEST(std::get<2>(geo::intersectsContains(bx.getOuter(), ax.getInners().front())));
    TEST(geo::ringContains(bx.getOuter().rawRing().front().seg().first, ax.getOuter(), 0));
    TEST(geo::intersectsContains(bx, ax).first);
    TEST(!geo::intersectsContains(bx, ax).second);
    TEST(geo::intersectsContains(ax, bx).first);
    TEST(!geo::intersectsContains(ax, bx).second);
    }

    {
    auto a = polygonFromWKT<int>("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0), (4 4, 5 4, 5 5, 4 5, 4 4))");
    auto b = polygonFromWKT<int>("POLYGON((1 1, 9 1, 9 9, 1 9, 1 1), (3 3, 6 3, 6 6, 3 6, 3 3))");

    TEST(geo::getWKT(a), ==, "POLYGON ((0 0, 10 0, 10 10, 0 10, 0 0), (4 4, 5 4, 5 5, 4 5, 4 4))");
    TEST(geo::getWKT(b.getOuter().front()), ==, "POINT (1 1)");
    TEST(geo::getWKT(a.getInners().front()), ==, "LINESTRING (4 4, 5 4, 5 5, 4 5)");

    TEST(geo::contains(b, a));
    TEST(!geo::contains(a, b));
    TEST(geo::intersects(a, b));
    TEST(geo::intersects(b, a));

    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(!geo::intersects(ax.getOuter(), bx.getOuter()));
    TEST(geo::intersectsContains(bx, ax).first);
    TEST(geo::intersectsContains(bx, ax).second);
    TEST(geo::intersectsContains(ax, bx).first);
    TEST(!geo::intersectsContains(ax, bx).second);
    }

    {
    auto a = polygonFromWKT<int>("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0), (4 4, 5 4, 5 5, 4 5, 4 4))");
    auto b = polygonFromWKT<int>("POLYGON((1 1, 9 1, 9 9, 1 9, 1 1), (3 3, 6 3, 6 6, 3 6, 3 3))");

    TEST(geo::getWKT(a), ==, "POLYGON ((0 0, 10 0, 10 10, 0 10, 0 0), (4 4, 5 4, 5 5, 4 5, 4 4))");
    TEST(geo::getWKT(b.getOuter().front()), ==, "POINT (1 1)");
    TEST(geo::getWKT(a.getInners().front()), ==, "LINESTRING (4 4, 5 4, 5 5, 4 5)");

    TEST(geo::contains(b, a));
    TEST(!geo::contains(a, b));
    TEST(geo::intersects(a, b));
    TEST(geo::intersects(b, a));

    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(!geo::intersects(ax.getOuter(), bx.getOuter()));
    TEST(geo::intersectsContains(bx, ax).first);
    TEST(geo::intersectsContains(bx, util::geo::getBoundingBox(b), util::geo::area(b),  ax, util::geo::getBoundingBox(a), util::geo::area(a)).first);
    TEST(geo::intersectsContains(bx, ax).second);
    TEST(geo::intersectsContains(bx, util::geo::getBoundingBox(b), util::geo::area(b),  ax, util::geo::getBoundingBox(a), util::geo::area(a)).second);
    TEST(geo::intersectsContains(ax, bx).first);
    TEST(geo::intersectsContains(ax, util::geo::getBoundingBox(a), util::geo::area(a),  bx, util::geo::getBoundingBox(b), util::geo::area(b)).first);
    TEST(!geo::intersectsContains(ax, bx).second);
    TEST(!geo::intersectsContains(ax, util::geo::getBoundingBox(a), util::geo::area(a),  bx, util::geo::getBoundingBox(b), util::geo::area(b)).second);
    }

    {
    auto a = polygonFromWKT<int>("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0), (1 1, 9 1, 9 9, 1 9, 1 1))");
    auto b = polygonFromWKT<int>("POLYGON((4 4, 5 4, 5 5, 4 5, 4 4))");

    TEST(!geo::contains(b, a));
    TEST(!geo::contains(a, b));
    TEST(!geo::intersects(a, b));
    TEST(!geo::intersects(b, a));

    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(!geo::intersectsContains(bx, ax).first);
    TEST(!geo::intersectsContains(bx, ax).second);
    TEST(!geo::intersectsContains(ax, bx).first);
    TEST(!geo::intersectsContains(ax, bx).second);
    }

    {
    auto a = polygonFromWKT<int>("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0), (1 1, 5 5, 9 9, 1 9, 1 1))");
    auto b = polygonFromWKT<int>("POLYGON((4 4, 5 4, 5 5, 4 5, 4 4))");

    TEST(!geo::contains(b, a));
    TEST(!geo::contains(a, b));
    TEST(geo::intersects(a, b));
    TEST(geo::intersects(b, a));

    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(geo::intersectsContains(bx, ax).first);
    TEST(!geo::intersectsContains(bx, ax).second);
    TEST(geo::intersectsContains(ax, bx).first);
    TEST(!geo::intersectsContains(ax, bx).second);
    }

    {
    auto a = lineFromWKT<int>("LINESTRING(4 4, 5 4, 5 5, 4 5, 4 4)");
    auto b = polygonFromWKT<int>("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0))");

    // TEST(!geo::contains(b, a));
    // TEST(!geo::contains(a, b));
    // TEST(geo::intersects(a, b));
    // TEST(geo::intersects(b, a));

    XSortedLine<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(geo::intersectsContains(ax, bx).first);
    TEST(geo::intersectsContains(ax, bx).second);
    }
    {
    auto a = lineFromWKT<int>("LINESTRING(4 4, 5 4, 5 5, 4 5, 4 4)");
    auto b = polygonFromWKT<int>("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0), (1 1, 9 1, 9 9, 1 9, 1 1))");

    // TEST(!geo::contains(b, a));
    // TEST(!geo::contains(a, b));
    // TEST(geo::intersects(a, b));
    // TEST(geo::intersects(b, a));

    XSortedLine<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(!geo::intersectsContains(ax, bx).first);
    TEST(!geo::intersectsContains(ax, bx).second);
    }
  }

  // ___________________________________________________________________________
  {
    TEST(geo::frechetDist(Line<double>{{0, 0}, {10, 10}}, Line<double>{{0, 0},
                                  {10, 10}}, 1) == approx(0));

    TEST(geo::frechetDist(Line<double>{{0, 0}, {10, 10}}, Line<double>{{0, 0},
                                  {10, 10}}, 0.1) == approx(0));

    TEST(geo::frechetDist(Line<double>{{0, 10}, {10, 10}}, Line<double>{{0, 0},
                                  {10, 0}}, 0.1) == approx(10));

    TEST(geo::frechetDist(Line<double>{{0, 10}, {10, 10}}, Line<double>{{0, 0},
                                  {0, 0}}, 0.1)
        == approx(util::geo::dist(Point<double>{0, 0}, Point<double>{10, 10})));

    TEST(geo::frechetDist(Line<double>{{0, 10}, {0, 10}}, Line<double>{{0, 0},
                                  {0, 5}}, 0.1) == approx(10));

    TEST(geo::frechetDist(Line<double>{{0, 10}, {0, 10}}, Line<double>{{0, 5},
                                  {0, 5}}, 0.1) == approx(5));
  }

  // ___________________________________________________________________________
  {
    TEST(util::btsSimi("", ""), ==, approx(1));
    TEST(util::btsSimi("Hallo", "Test"), ==, approx(0));
    TEST(util::btsSimi("Test", "Hallo"), ==, approx(0));
    TEST(util::btsSimi("Test", "Test"), ==, approx(1));
    TEST(util::btsSimi("Milner Road / Wandlee Road", "Wandlee Road"), ==, approx(1));
    TEST(util::btsSimi("bla blubb blob", "blubb blib"), ==, approx(0.9));
    TEST(util::btsSimi("St Pancras International", "London St Pancras"), ==, approx(0.588235));
    TEST(util::btsSimi("Reiterstraße", "Reiterstraße Freiburg im Breisgau"), ==, approx(1));
    TEST(util::btsSimi("Reiterstraße", "Reiter Freiburg im Breisgau"), ==, approx(.466666666));
    TEST(util::btsSimi("AA", "Reiterstraße, Freiburg im Breisgau"), ==, approx(0));
    TEST(util::btsSimi("blibb blabbel bla blubb blob", "blubb blib blabb"), ==, approx(0.875));
    TEST(util::btsSimi("blibb blabbel bla blubb blobo", "blubb blib blabb blabo"), ==, approx(0.84));
    TEST(util::btsSimi("blubb blib blabb", "blibb blabbel bla blubb blob"), ==, approx(0.875));
    TEST(util::btsSimi("blubbb blib blabb blobo", "blibb blabbel bla blubb blobo"), ==, approx(0.84));
    TEST(util::btsSimi("Reiter Freiburg im Breisgau", "Reiter Frei burg im Brei sgau"), ==, approx(0.931034));
    // fallback to jaccard
    TEST(util::btsSimi("Freiburg im Breisgau, Germany, Main Railway Station", "Main Railway Station Freiburg im Breisgau, Germany"), ==, approx(1));

  }

  // ___________________________________________________________________________
  {
    std::string test = u8"Zürich, Hauptbahnhof (Nord)";
    auto tokens = util::tokenize(test);

    TEST(tokens.size(), ==, 3);

    TEST(util::jaccardSimi("Zürich Hauptbahnhof Nord", "Zürich, Hauptbahnhof (Nord)"), ==, approx(1));
    TEST(util::jaccardSimi("Zürich Hauptbahnhof", "Zürich, Hauptbahnhof ()"), ==, approx(1));
    TEST(util::jaccardSimi("Zürich Hauptbahnhof", "Zürich, Hauptbahnhof (Nord)"), ==, approx(2./3.));
  }

  // ___________________________________________________________________________
  {
    TEST(util::atof("45.534215"), ==, approx(45.534215));
    TEST(util::atof("5.534"), ==, approx(5.534));
    TEST(util::atof("534"), ==, approx(534));
    TEST(util::atof("-534"), ==, approx(-534));
    TEST(util::atof("-45.534215"), ==, approx(-45.534215));
    TEST(util::atof("-45.534215", 2), ==, approx(-45.53));

    // TODO: more test cases
  }

  // ___________________________________________________________________________
  {
    std::stringstream ss;
    util::json::Writer wr(&ss, 2, false);

    util::json::Val a("bla");
    util::json::Val b(1);
    util::json::Val c(1.0);
    util::json::Val d("a");
    util::json::Val e({"a", "b", "c"});

    util::json::Val f({1, json::Array{2, 3, 4}, 3});

    ss.str("");
    wr = util::json::Writer(&ss, 2, false);
    util::json::Val i({1, json::Array{2, json::Null(), 4}, true});
    wr.val(i);
    wr.closeAll();
    TEST(ss.str(), ==, "[1,[2,null,4],true]");

    ss.str("");
    wr = util::json::Writer(&ss, 2, false);
    i = util::json::Val({1, json::Array{2, json::Null(), 4}, false});
    wr.val(i);
    wr.closeAll();
    TEST(ss.str(), ==, "[1,[2,null,4],false]");

    ss.str("");
    wr = util::json::Writer(&ss, 2, false);
    i = util::json::Val({1, json::Array{2, json::Null(), 4}, false});
    wr.val(i);
    wr.closeAll();
    TEST(ss.str(), ==, "[1,[2,null,4],false]");

    ss.str("");
    wr = util::json::Writer(&ss, 2, false);
    i = util::json::Val({1, json::Array{2.13, "", 4}, 0});
    wr.val(i);
    wr.closeAll();
    TEST(ss.str(), ==, "[1,[2.13,\"\",4],0]");

    ss.str("");
    wr = util::json::Writer(&ss, 2, false);
    i = util::json::Val(
        {1, json::Array{2.13, json::Dict{{"a", 1}, {"B", 2.123}}, 4}, 0});
    wr.val(i);
    wr.closeAll();
    TEST((ss.str() == "[1,[2.13,{\"a\":1,\"B\":2.12},4],0]" ||
            ss.str() == "[1,[2.13,{\"B\":2.12,\"a\":1},4],0]"));
  }

  // ___________________________________________________________________________
  {
    DirGraph<int, int> g;

    DirNode<int, int>* a = new DirNode<int, int>(0);
    DirNode<int, int>* b = new DirNode<int, int>(0);
    g.addNd(a);
    TEST(g.getNds().size(), ==, (size_t)1);
    g.addNd(b);
    TEST(g.getNds().size(), ==, (size_t)2);

    g.addEdg(a, b);
    TEST(a->getDeg(), ==, (size_t)1);
    TEST(b->getDeg(), ==, (size_t)0);

    auto c = g.addNd();

    g.addEdg(a, c);
    g.addEdg(c, b);
    TEST(a->getDeg(), ==, (size_t)2);
    TEST(b->getDeg(), ==, (size_t)0);
    TEST(c->getDeg(), ==, (size_t)1);

    g.delEdg(a, c);

    TEST(a->getDeg(), ==, (size_t)1);
    TEST(b->getDeg(), ==, (size_t)0);
    TEST(c->getDeg(), ==, (size_t)1);

    g.addEdg(a, a);
    TEST(a->getDeg(), ==, (size_t)2);

    g.delEdg(a, a);
    TEST(a->getDeg(), ==, (size_t)1);

    g.delEdg(a, a);
    TEST(a->getDeg(), ==, (size_t)1);

    // TODO: more test cases
  }

  // ___________________________________________________________________________
  {
    UndirGraph<int, int> g;

    UndirNode<int, int>* a = new UndirNode<int, int>(0);
    UndirNode<int, int>* b = new UndirNode<int, int>(0);
    g.addNd(a);
    TEST(g.getNds().size(), ==, (size_t)1);
    g.addNd(b);
    TEST(g.getNds().size(), ==, (size_t)2);

    g.addEdg(a, b);
    TEST(a->getDeg(), ==, (size_t)1);
    TEST(b->getDeg(), ==, (size_t)1);

    auto c = g.addNd();

    g.addEdg(a, c);
    g.addEdg(c, b);
    TEST(a->getDeg(), ==, (size_t)2);
    TEST(b->getDeg(), ==, (size_t)2);
    TEST(c->getDeg(), ==, (size_t)2);

    g.delEdg(a, c);

    TEST(a->getDeg(), ==, (size_t)1);
    TEST(b->getDeg(), ==, (size_t)2);
    TEST(c->getDeg(), ==, (size_t)1);

    g.delNd(b);

    TEST(a->getDeg(), ==, (size_t)0);
    TEST(c->getDeg(), ==, (size_t)0);

    g.addEdg(a, a);
    TEST(a->getDeg(), ==, (size_t)1);

    g.delEdg(a, a);
    TEST(a->getDeg(), ==, (size_t)0);

    // TODO: more test cases
  }

  // ___________________________________________________________________________
  {
    Grid<int, Line, double> g(
        .5, .5, Box<double>(Point<double>(0, 0), Point<double>(3, 3)));

    Line<double> l;
    l.push_back(Point<double>(0, 0));
    l.push_back(Point<double>(1.5, 2));

    Line<double> l2;
    l2.push_back(Point<double>(2.5, 1));
    l2.push_back(Point<double>(2.5, 2));

    g.add(l, 1);
    g.add(l2, 2);

    std::set<int> ret;

    Box<double> req(Point<double>(.5, 1), Point<double>(1, 1.5));
    g.get(req, &ret);
    TEST(ret.size(), ==, (size_t)1);

    ret.clear();
    g.getNeighbors(1, 0, &ret);
    TEST(ret.size(), ==, (size_t)1);

    ret.clear();
    g.getNeighbors(1, 0.55, &ret);
    TEST(ret.size(), ==, (size_t)2);

    // TODO: more test cases
  }

  // ___________________________________________________________________________
  {
    Line<double> a;
    a.push_back(Point<double>(1, 1));
    a.push_back(Point<double>(10, 1));

    auto dense = util::geo::densify(a, 1);

    TEST(dense.size(), ==, (size_t)10);

    for (int i = 0; i < 10; i++) {
      TEST(dense[i].getX(), ==, approx(i + 1.0));
    }

    dense = util::geo::simplify(dense, 0.1);
    TEST(dense.size(), ==, (size_t)2);

    Line<double> b;
    b.push_back(Point<double>(1, 1));
    b.push_back(Point<double>(5, 7));
    b.push_back(Point<double>(10, 3));

    dense = util::geo::densify(b, 1);

    dense = util::geo::simplify(dense, 0.1);
    TEST(dense.size(), ==, (size_t)3);
  }

  // ___________________________________________________________________________
  {
    Line<double> a;
    a.push_back(Point<double>(1, 1));
    a.push_back(Point<double>(2, 1));
    a.push_back(Point<double>(3, 1));
    a.push_back(Point<double>(3, 2));
    a.push_back(Point<double>(4, 2));
    a.push_back(Point<double>(4, 1));
    a.push_back(Point<double>(5, 1));
    a.push_back(Point<double>(6, 1));

    Line<double> b;
    b.push_back(Point<double>(1, 1));
    b.push_back(Point<double>(2, 1));
    b.push_back(Point<double>(3, 1));
    b.push_back(Point<double>(4, 1));
    b.push_back(Point<double>(5, 1));
    b.push_back(Point<double>(6, 1));

    double fd = util::geo::accFrechetDistC(a, b, 0.1);
    TEST(fd, ==, approx(2));
  }

  // ___________________________________________________________________________
  {
    Line<double> e;
    e.push_back(Point<double>(1, 1));
    e.push_back(Point<double>(1, 2));

    Line<double> f;
    f.push_back(Point<double>(1, 1));
    f.push_back(Point<double>(1, 2));

    double fd = util::geo::frechetDist(e, f, 0.1);

    TEST(fd, ==, approx(0));

    Line<double> a;
    a.push_back(Point<double>(1, 1));
    a.push_back(Point<double>(2, 1));
    a.push_back(Point<double>(3, 2));
    a.push_back(Point<double>(4, 2));
    a.push_back(Point<double>(5, 1));
    a.push_back(Point<double>(6, 1));

    Line<double> b;
    b.push_back(Point<double>(1, 1));
    b.push_back(Point<double>(2, 1));
    b.push_back(Point<double>(3, 1));
    b.push_back(Point<double>(4, 1));
    b.push_back(Point<double>(5, 1));
    b.push_back(Point<double>(6, 1));

    auto adense = util::geo::densify(a, 0.1);
    auto bdense = util::geo::densify(b, 0.1);

    fd = util::geo::frechetDist(a, b, 0.1);

    TEST(fd, ==, approx(1));

    Line<double> c;
    c.push_back(Point<double>(1, 1));
    c.push_back(Point<double>(2, 1));

    Line<double> d;
    d.push_back(Point<double>(3, 1));
    d.push_back(Point<double>(4, 1));

    fd = util::geo::frechetDist(c, d, 0.1);

    TEST(fd, ==, approx(2));

    Line<double> g;
    g.push_back(Point<double>(1, 1));
    g.push_back(Point<double>(10, 1));

    Line<double> h;
    h.push_back(Point<double>(1, 1));
    h.push_back(Point<double>(3, 2));
    h.push_back(Point<double>(3, 1));
    h.push_back(Point<double>(10, 1));

    fd = util::geo::frechetDist(g, h, 0.1);

    TEST(fd, ==, approx(1));
  }

  // ___________________________________________________________________________
  {
    Line<double> a;
    a.push_back(Point<double>(1, 1));
    a.push_back(Point<double>(1, 2));

    Line<double> b;
    b.push_back(Point<double>(1, 2));
    b.push_back(Point<double>(2, 2));

    Line<double> c;
    c.push_back(Point<double>(2, 2));
    c.push_back(Point<double>(2, 1));

    Line<double> d;
    d.push_back(Point<double>(2, 1));
    d.push_back(Point<double>(1, 1));

    Box<double> box(Point<double>(2, 3), Point<double>(5, 4));
    MultiLine<double> ml;
    ml.push_back(a);
    ml.push_back(b);
    ml.push_back(c);
    ml.push_back(d);

    TEST(parallelity(box, ml), ==, approx(1));
    ml = rotate(ml, 45);
    TEST(parallelity(box, ml), ==, approx(0));
    ml = rotate(ml, 45);
    TEST(parallelity(box, ml), ==, approx(1));
    ml = rotate(ml, 45);
    TEST(parallelity(box, ml), ==, approx(0));
    ml = rotate(ml, 45);
    TEST(parallelity(box, ml), ==, approx(1));
  }

  // ___________________________________________________________________________
  {
    TEST("zürich", ==, util::urlDecode("z%C3%BCrich"));
    TEST("!@$%^*()", ==, util::urlDecode("!%40%24%25%5E*()"));
    TEST("Løkken", ==, util::urlDecode("L%C3%B8kken"));
    TEST("á é", ==, util::urlDecode("%C3%A1%20%C3%A9"));
    TEST("á é", ==, util::urlDecode("%C3%A1+%C3%A9"));
  }

  // ___________________________________________________________________________
  {
    TEST("Hello\\\\Goodbye!" == util::jsonStringEscape("Hello\\Goodbye!"));
    TEST("\\\"Hello\\\"" == util::jsonStringEscape("\"Hello\""));
  }

  // ___________________________________________________________________________
  {
    TEST(util::split("hello,again", ',').size(), ==, (size_t)2);
    TEST(util::split("hello,,again", ',').size(), ==, (size_t)3);
    TEST(util::split("hello", ',').size(), ==, (size_t)1);
    TEST(util::split("", ',').size(), ==, (size_t)0);
  }

  // ___________________________________________________________________________
  {
    TEST(util::editDist("hello", "mello"), ==, (size_t)1);
    TEST(util::editDist("mello", "hello"), ==, (size_t)1);
    TEST(util::editDist("abcde", "abfde"), ==, (size_t)1);
    TEST(util::editDist("abcd", "abcde"), ==, (size_t)1);
    TEST(util::editDist("xabcd", "abcde"), ==, (size_t)2);
    TEST(util::editDist("abcd", "abcdes"), ==, (size_t)2);
    TEST(util::editDist("hello", "hello"), ==, (size_t)0);
  }

  // ___________________________________________________________________________
  {
    TEST(util::prefixEditDist("hello", "hello", 0), ==, (size_t)0);
    TEST(util::prefixEditDist("hello", "hello", 100), ==, (size_t)0);
    TEST(util::prefixEditDist("hello", "hello"), ==, (size_t)0);
    TEST(util::prefixEditDist("hel", "hello"), ==, (size_t)0);
    TEST(util::prefixEditDist("hel", "hello", 0), ==, (size_t)0);
    TEST(util::prefixEditDist("hel", "hello", 1), ==, (size_t)0);
    TEST(util::prefixEditDist("hel", "hello", 2), ==, (size_t)0);
    TEST(util::prefixEditDist("hal", "hello", 2), ==, (size_t)1);
    TEST(util::prefixEditDist("hal", "hello", 1), ==, (size_t)1);
    TEST(util::prefixEditDist("hal", "hello", 0), >, (size_t)0);
    TEST(util::prefixEditDist("fel", "hello", 0), >, (size_t)0);
    TEST(util::prefixEditDist("fel", "hello", 1), ==, (size_t)1);
    TEST(util::prefixEditDist("fel", "hello", 2), ==, (size_t)1);
    TEST(util::prefixEditDist("fal", "hello", 2), ==, (size_t)2);
    TEST(util::prefixEditDist("fal", "hello", 1), >, (size_t)1);
    TEST(util::prefixEditDist("fal", "hello", 0), >, (size_t)0);
    TEST(util::prefixEditDist("far", "hello", 0), >, (size_t)0);
    TEST(util::prefixEditDist("far", "hello", 1), >, (size_t)1);
    TEST(util::prefixEditDist("far", "hello", 2), >, (size_t)2);
    TEST(util::prefixEditDist("far", "hello", 3), ==, (size_t)3);
    TEST(util::prefixEditDist("far", "hello", 4), ==, (size_t)3);
    TEST(util::prefixEditDist("far", "hello"), ==, (size_t)3);
    TEST(util::prefixEditDist("hefar", "hello"), ==, (size_t)3);
    TEST(util::prefixEditDist("hefaree", "hello"), ==, (size_t)5);
    TEST(util::prefixEditDist("helloo", "hello"), ==, (size_t)1);
    TEST(util::prefixEditDist("helloo", "hello", 0), >, (size_t)0);
    TEST(util::prefixEditDist("helloo", "hello", 1), ==, (size_t)1);
    TEST(util::prefixEditDist("helloo", "hello", 2), ==, (size_t)1);
    TEST(util::prefixEditDist("", "hello", 2), ==, (size_t)0);
    TEST(util::prefixEditDist("e", "hello", 2), ==, (size_t)1);
    TEST(util::prefixEditDist("el", "hello", 2), ==, (size_t)1);
    TEST(util::prefixEditDist("ello", "hello", 2), ==, (size_t)1);
    TEST(util::prefixEditDist("hell", "hello", 2), ==, (size_t)0);
    TEST(util::prefixEditDist("hell", "", 2), >, (size_t)2);
    TEST(util::prefixEditDist("hell", ""), ==, (size_t)4);
  }

  // ___________________________________________________________________________
  {
    TEST(util::toString(34) == "34");
    TEST(util::toString("34") == "34");
  }

  // ___________________________________________________________________________
  {
    std::string a("lorem ipsum ipsum lorem");

    TEST(util::replace(a, "ips", "aa"));
    TEST(a, ==, "lorem aaum ipsum lorem");

    TEST(!util::replace(a, "blablabla", ""));
    TEST(a, ==, "lorem aaum ipsum lorem");

    TEST(util::replace(a, "m", ""));
    TEST(a, ==, "lore aaum ipsum lorem");

    TEST(!util::replace(a, "", ""));
    TEST(a, ==, "lore aaum ipsum lorem");

    std::string b("lorem ipsum ipsum lorem");
    TEST(util::replaceAll(b, "ips", "aa"));
    TEST(b, ==, "lorem aaum aaum lorem");

    TEST(util::replaceAll(b, "m", ""));
    TEST(b, ==, "lore aau aau lore");

    TEST(util::replaceAll(b, "a", "aa"));
    TEST(b, ==, "lore aaaau aaaau lore");

    TEST(util::replaceAll(b, "e", "e"));
    TEST(b, ==, "lore aaaau aaaau lore");

    TEST(util::replaceAll(b, "e", "ee"));
    TEST(b, ==, "loree aaaau aaaau loree");

    TEST(!util::replaceAll(b, "", "ee"));
    TEST(b, ==, "loree aaaau aaaau loree");
  }

  // ___________________________________________________________________________
  {
    UndirGraph<std::string, int> g;

    auto a = g.addNd("A");
    auto b = g.addNd("B");
    auto c = g.addNd("C");
    auto d = g.addNd("D");
    auto e = g.addNd("E");

    g.addEdg(a, c, 1);
    g.addEdg(a, b, 5);
    g.addEdg(d, c, 1);
    g.addEdg(d, b, 3);
    g.addEdg(e, d, 1);
    g.addEdg(e, b, 1);

    auto comps = util::graph::Algorithm::connectedComponents(g);

    TEST(comps.size(), ==, static_cast<size_t>(1));
    TEST(comps[0].count(a));
    TEST(comps[0].count(b));
    TEST(comps[0].count(c));
    TEST(comps[0].count(d));
    TEST(comps[0].count(e));

    auto f = g.addNd("F");
    comps = util::graph::Algorithm::connectedComponents(g);
    TEST(comps.size(), ==, static_cast<size_t>(2));

    auto gn = g.addNd("G");
    comps = util::graph::Algorithm::connectedComponents(g);
    TEST(comps.size(), ==, static_cast<size_t>(3));

    g.addEdg(f, gn, 1);
    comps = util::graph::Algorithm::connectedComponents(g);
    TEST(comps.size(), ==, static_cast<size_t>(2));

    g.addEdg(f, a, 1);
    comps = util::graph::Algorithm::connectedComponents(g);
    TEST(comps.size(), ==, static_cast<size_t>(1));
  }

  // ___________________________________________________________________________
  {
    DirGraph<std::string, int> g;

    auto a = g.addNd("A");
    auto b = g.addNd("B");
    auto c = g.addNd("C");
    auto d = g.addNd("D");
    auto e = g.addNd("E");

    auto eAC = g.addEdg(a, c, 1);
    auto eAB = g.addEdg(a, b, 5);
    auto eDC = g.addEdg(d, c, 1);
    auto eDB = g.addEdg(d, b, 3);
    auto eED = g.addEdg(e, d, 1);
    auto eEB = g.addEdg(e, b, 1);

    UNUSED(eAC);
    UNUSED(eDC);
    UNUSED(eDB);
    UNUSED(eED);
    UNUSED(eEB);

    struct CostFunc : public EDijkstra::CostFunc<std::string, int, int> {
      int operator()(const Edge<std::string, int>* from,
                     const Node<std::string, int>* n,
                     const Edge<std::string, int>* to) const {
        UNUSED(from);

        // dont count cost of start edge
        if (n) return to->pl();
        return 0;
      };
      int inf() const { return 999; };
    };

    CostFunc cFunc;

    auto cost = EDijkstra::shortestPath(eAB, cFunc);

    for (auto u : cost) {
      int single = EDijkstra::shortestPath(eAB, u.first, cFunc);
      TEST(single, ==, u.second);
    }

    // all to 1
    auto eBC = g.addEdg(b, c, 10);

    auto costb = EDijkstra::shortestPathRev(eBC, cFunc);
    for (auto u : costb) {
      int single = EDijkstra::shortestPath(u.first, eBC, cFunc);
      TEST(single, ==, u.second);
    }
  }

  // ___________________________________________________________________________
  {
    UndirGraph<std::string, int> g;

    auto a = g.addNd("A");
    auto b = g.addNd("B");
    auto c = g.addNd("C");
    auto d = g.addNd("D");
    auto e = g.addNd("E");

    auto eAC = g.addEdg(a, c, 1);
    auto eAB = g.addEdg(a, b, 5);
    auto eDC = g.addEdg(d, c, 1);
    auto eDB = g.addEdg(d, b, 3);
    auto eED = g.addEdg(e, d, 1);
    auto eEB = g.addEdg(e, b, 1);

    UNUSED(eAC);
    UNUSED(eDC);
    UNUSED(eDB);
    UNUSED(eED);
    UNUSED(eEB);

    struct CostFunc : public EDijkstra::CostFunc<std::string, int, int> {
      int operator()(const Edge<std::string, int>* from,
                     const Node<std::string, int>* n,
                     const Edge<std::string, int>* to) const {
        UNUSED(from);

        // dont count cost of start edge
        if (n) return to->pl();
        return 0;
      };
      int inf() const { return 999; };
    };

    CostFunc cFunc;

    EDijkstra::NList<std::string, int> res;
    EDijkstra::EList<std::string, int> resE;
    int cost = EDijkstra::shortestPath(eAB, d, cFunc, &resE, &res);

    TEST(cost, ==, 2);

    TEST(resE.size(), ==, (size_t)3);
    TEST(res.size(), ==, (size_t)3);
    TEST((*(res.rbegin()))->pl(), ==, "A");
    TEST((*(++res.rbegin()))->pl(), ==, "C");
    TEST((*(++++res.rbegin()))->pl(), ==, "D");

    TEST((*(resE.rbegin())), ==, eAB);
    TEST((*(++resE.rbegin())), ==, eAC);
    TEST((*(++++resE.rbegin())), ==, eDC);

    cost = EDijkstra::shortestPath(eAB, b, cFunc, &resE, &res);
    TEST(cost, ==, 0);
  }

  // ___________________________________________________________________________
  {
    UndirGraph<std::string, int> g;

    auto a = g.addNd("A");
    auto b = g.addNd("B");
    auto c = g.addNd("C");
    auto d = g.addNd("D");
    auto e = g.addNd("E");

    auto eAC = g.addEdg(a, c, 1);
    auto eAB = g.addEdg(a, b, 5);
    auto eDC = g.addEdg(d, c, 1);
    auto eDB = g.addEdg(d, b, 3);
    auto eED = g.addEdg(e, d, 1);
    auto eEB = g.addEdg(e, b, 1);

    UNUSED(eAC);
    UNUSED(eDC);
    UNUSED(eDB);
    UNUSED(eED);
    UNUSED(eEB);

    struct CostFunc : public EDijkstra::CostFunc<std::string, int, int> {
      int operator()(const Edge<std::string, int>* from,
                     const Node<std::string, int>* n,
                     const Edge<std::string, int>* to) const {
        UNUSED(from);

        // dont count cost of start edge
        if (n) return to->pl();
        return 0;
      };
      int inf() const { return 999; };
    };

    CostFunc cFunc;

    std::set<Node<std::string, int>*> tos;
    tos.insert(d);
    tos.insert(b);
    tos.insert(b);

    EDijkstra::NList<std::string, int> res;
    EDijkstra::EList<std::string, int> resE;
    int cost = EDijkstra::shortestPath(eAB, tos, cFunc, &resE, &res);
    TEST(cost, ==, 0);
  }

  // ___________________________________________________________________________
  {
    UndirGraph<std::string, int> g;

    auto a = g.addNd("A");
    auto b = g.addNd("B");
    auto c = g.addNd("C");
    auto d = g.addNd("D");
    auto e = g.addNd("E");

    g.addEdg(a, c, 1);
    auto eAB = g.addEdg(a, b, 5);
    auto eDC = g.addEdg(d, c, 1);
    g.addEdg(d, b, 3);
    auto eED = g.addEdg(e, d, 1);
    g.addEdg(e, b, 1);

    struct CostFunc : public EDijkstra::CostFunc<std::string, int, int> {
      int operator()(const Edge<std::string, int>* from,
                     const Node<std::string, int>* n,
                     const Edge<std::string, int>* to) const {
        UNUSED(from);

        // dont count cost of start edge
        if (n) return to->pl();
        return 0;
      };
      int inf() const { return 999; };
    };

    CostFunc cFunc;

    std::set<Edge<std::string, int>*> tos;
    tos.insert(eDC);
    tos.insert(eED);

    std::unordered_map<Edge<std::string, int>*,
                       EDijkstra::EList<std::string, int>*>
        resE;
    resE[eDC] = new EDijkstra::EList<std::string, int>();
    resE[eED] = new EDijkstra::EList<std::string, int>();
    std::unordered_map<Edge<std::string, int>*,
                       EDijkstra::NList<std::string, int>*>
        res;
    res[eDC] = new EDijkstra::NList<std::string, int>();
    res[eED] = new EDijkstra::NList<std::string, int>();
    auto hFunc = ZeroHeurFunc<std::string, int, int>();
    std::unordered_map<Edge<std::string, int>*, int> cost =
        EDijkstra::shortestPath(eAB, tos, cFunc, hFunc, resE, res);

    TEST(cost[eDC], ==, 2);
    TEST(cost[eED], ==, 2);

    TEST(resE[eDC]->size(), ==, (size_t)3);
    TEST(res[eED]->size(), ==, (size_t)3);

    TEST(resE[eDC]->size(), ==, (size_t)3);
    TEST(res[eED]->size(), ==, (size_t)3);
  }

  // ___________________________________________________________________________
  {
    UndirGraph<std::string, int> g;

    auto a = g.addNd("A");
    auto b = g.addNd("B");
    auto c = g.addNd("C");
    auto d = g.addNd("D");
    auto e = g.addNd("E");

    g.addEdg(a, c, 1);
    g.addEdg(a, b, 5);
    g.addEdg(d, c, 1);
    g.addEdg(d, b, 3);
    g.addEdg(e, d, 1);
    g.addEdg(e, b, 1);

    struct CostFunc : public EDijkstra::CostFunc<std::string, int, int> {
      int operator()(const Edge<std::string, int>* fr,
                     const Node<std::string, int>* n,
                     const Edge<std::string, int>* to) const {
        UNUSED(fr);
        UNUSED(n);
        return to->pl();
      };
      int inf() const { return 999; };
    };

    CostFunc cFunc;

    EDijkstra::NList<std::string, int> res;
    EDijkstra::EList<std::string, int> resE;
    int cost = EDijkstra::shortestPath(a, b, cFunc, &resE, &res);

    TEST(res.size(), ==, (size_t)5);
    TEST((*(res.rbegin()))->pl(), ==, "A");
    TEST((*(++res.rbegin()))->pl(), ==, "C");
    TEST((*(++++res.rbegin()))->pl(), ==, "D");
    TEST((*(++++++res.rbegin()))->pl(), ==, "E");
    TEST((*(++++++++res.rbegin()))->pl(), ==, "B");
    TEST(cost, ==, 4);
    TEST((*(resE.rbegin()))->getFrom()->pl(), ==, "A");
    TEST((*(++resE.rbegin()))->getFrom()->pl(), ==, "D");
    TEST((*(++++resE.rbegin()))->getFrom()->pl(), ==, "E");
    TEST((*(++++++resE.rbegin()))->getTo()->pl(), ==, "B");

    TEST(resE.size(), ==, (size_t)4);

    cost = EDijkstra::shortestPath(d, b, cFunc, &res);
    TEST(cost, ==, 2);

    cost = EDijkstra::shortestPath(b, d, cFunc, &res);
    TEST(cost, ==, 2);

    cost = EDijkstra::shortestPath(e, b, cFunc, &res);
    TEST(cost, ==, 1);

    cost = EDijkstra::shortestPath(b, e, cFunc, &res);
    TEST(cost, ==, 1);

    cost = EDijkstra::shortestPath(b, a, cFunc, &res);
    TEST(cost, ==, 4);

    cost = EDijkstra::shortestPath(c, a, cFunc, &res);
    TEST(cost, ==, 1);

    cost = EDijkstra::shortestPath(a, c, cFunc, &res);
    TEST(cost, ==, 1);

    cost = EDijkstra::shortestPath(a, d, cFunc, &res);
    TEST(cost, ==, 2);
  }

  // ___________________________________________________________________________
  {
    DirGraph<int, int> g;

    DirNode<int, int>* a = new DirNode<int, int>(1);
    DirNode<int, int>* b = new DirNode<int, int>(4);
    g.addNd(a);
    g.addNd(b);

    auto c = g.addNd(2);
    auto d = g.addNd(3);
    auto x = g.addNd();

    g.addEdg(a, d, 4);
    g.addEdg(a, c, 1);
    g.addEdg(c, b, 1);
    g.addEdg(b, d, 1);

    struct CostFunc : public EDijkstra::CostFunc<int, int, int> {
      int operator()(const Edge<int, int>* fr, const Node<int, int>* n,
                     const Edge<int, int>* to) const {
        UNUSED(fr);
        UNUSED(n);
        return to->pl();
      };
      int inf() const { return 999; };
    };

    CostFunc cFunc;

    EDijkstra::NList<int, int> res;
    int cost = EDijkstra::shortestPath(a, d, cFunc, &res);

    TEST(cost, ==, 3);

    g.addEdg(c, d, 3);
    cost = EDijkstra::shortestPath(a, d, cFunc, &res);

    TEST(cost, ==, 3);

    g.addEdg(a, b, 1);
    g.addEdg(x, a, 1);
    cost = EDijkstra::shortestPath(a, d, cFunc, &res);

    TEST(cost, ==, 2);
  }

  // ___________________________________________________________________________
  {
    DirGraph<int, int> g;

    auto source = g.addNd();
    auto target = g.addNd();
    auto a = g.addNd();
    auto b = g.addNd();

    g.addEdg(source, a, 4);
    g.addEdg(source, b, 5);
    g.addEdg(a, target, 3);
    g.addEdg(b, target, 1);

    struct CostFunc : public BiDijkstra::CostFunc<int, int, int> {
      int operator()(const Node<int, int>* fr, const Edge<int, int>* e,
                     const Node<int, int>* to) const {
        UNUSED(fr);
        UNUSED(to);
        return e->pl();
      };
      int inf() const { return 999; };
    };

    CostFunc cFunc;

    BiDijkstra::NList<int, int> res;
    int cost = BiDijkstra::shortestPath(source, target, cFunc, &res);

    TEST(cost, ==, 6);
  }

  // ___________________________________________________________________________
  {
    DirGraph<int, int> g;

    DirNode<int, int>* a = new DirNode<int, int>(1);
    DirNode<int, int>* b = new DirNode<int, int>(0);
    g.addNd(a);
    g.addNd(b);

    auto c = g.addNd();
    auto d = g.addNd(4);
    auto x = g.addNd();

    g.addEdg(a, d, 4);
    g.addEdg(a, c, 1);
    g.addEdg(c, b, 1);
    g.addEdg(b, d, 1);

    struct CostFunc : public BiDijkstra::CostFunc<int, int, int> {
      int operator()(const Node<int, int>* fr, const Edge<int, int>* e,
                     const Node<int, int>* to) const {
        UNUSED(fr);
        UNUSED(to);
        return e->pl();
      };
      int inf() const { return 999; };
    };

    CostFunc cFunc;

    BiDijkstra::NList<int, int> res;
    int cost = BiDijkstra::shortestPath(a, d, cFunc, &res);

    TEST(cost, ==, 3);
    TEST(res.size(), ==, (size_t)4);

    g.addEdg(c, d, 3);
    cost = BiDijkstra::shortestPath(a, d, cFunc, &res);

    TEST(cost, ==, 3);

    g.addEdg(a, b, 1);
    g.addEdg(x, a, 1);
    cost = BiDijkstra::shortestPath(a, d, cFunc, &res);

    TEST(cost, ==, 2);

    // const std::set<Node<int, int>*> to{b, c, d, x};
    // std::unordered_map<Node<int, int>*, BiDijkstra::EList<int, int>*> resEdges;
    // std::unordered_map<Node<int, int>*, BiDijkstra::NList<int, int>*> resNodes;

    // for (auto n : to) {
      // resEdges[n] = new BiDijkstra::EList<int, int>();
      // resNodes[n] = new BiDijkstra::NList<int, int>();
    // }

    // auto costs = BiDijkstra::shortestPath(a, to, cFunc, resEdges, resNodes);

    // TEST(costs[b], ==, 1);
    // TEST(costs[c], ==, 1);
    // TEST(costs[d], ==, 2);
    // TEST(costs[x], ==, 999);
  }

  // ___________________________________________________________________________
  {
    DirGraph<int, int> g;

    DirNode<int, int>* a = new DirNode<int, int>(1);
    DirNode<int, int>* b = new DirNode<int, int>(0);
    g.addNd(a);
    g.addNd(b);

    auto c = g.addNd();
    auto d = g.addNd(4);
    auto x = g.addNd();

    g.addEdg(a, d, 4);
    g.addEdg(a, c, 1);
    g.addEdg(c, b, 1);
    g.addEdg(b, d, 1);

    struct CostFunc : public Dijkstra::CostFunc<int, int, int> {
      int operator()(const Node<int, int>* fr, const Edge<int, int>* e,
                     const Node<int, int>* to) const {
        UNUSED(fr);
        UNUSED(to);
        return e->pl();
      };
      int inf() const { return 999; };
    };

    CostFunc cFunc;

    Dijkstra::NList<int, int> res;
    int cost = Dijkstra::shortestPath(a, d, cFunc, &res);

    TEST(cost, ==, 3);
    TEST(res.size(), ==, (size_t)4);

    g.addEdg(c, d, 3);
    cost = Dijkstra::shortestPath(a, d, cFunc, &res);

    TEST(cost, ==, 3);

    g.addEdg(a, b, 1);
    g.addEdg(x, a, 1);
    cost = Dijkstra::shortestPath(a, d, cFunc, &res);

    TEST(cost, ==, 2);

    const std::set<Node<int, int>*> to{b, c, d, x};
    std::unordered_map<Node<int, int>*, Dijkstra::EList<int, int>*> resEdges;
    std::unordered_map<Node<int, int>*, Dijkstra::NList<int, int>*> resNodes;

    for (auto n : to) {
      resEdges[n] = new Dijkstra::EList<int, int>();
      resNodes[n] = new Dijkstra::NList<int, int>();
    }

    auto costs = Dijkstra::shortestPath(a, to, cFunc, resEdges, resNodes);

    TEST(costs[b], ==, 1);
    TEST(costs[c], ==, 1);
    TEST(costs[d], ==, 2);
    TEST(costs[x], ==, 999);
  }

  // ___________________________________________________________________________
  {{util::Nullable<std::string> nullable;
  TEST(nullable.isNull());
}

{
  util::Nullable<std::string> nullable(0);
  TEST(nullable.isNull());
}

{
  std::string str = "aa";
  util::Nullable<std::string> nullable(&str);
  TEST(!nullable.isNull());

  TEST(nullable == "aa");
  TEST(!(nullable == "aaa"));
  TEST(!(nullable != "aa"));
  TEST(nullable == "aa");

  TEST(nullable.get(), ==, "aa");
  TEST(std::string(nullable), ==, "aa");
}

{
  int a = 23;
  util::Nullable<int> nullable(a);
  util::Nullable<int> nullable2(24);
  TEST(!nullable.isNull());

  TEST(nullable, ==, 23);
  TEST(nullable, >=, 23);
  TEST(nullable, <=, 23);
  TEST(nullable, <, 24);
  TEST(nullable, <, 24);
  TEST(!(nullable < 22));
  TEST(nullable, !=, nullable2);
  TEST(nullable, <, nullable2);
  TEST(nullable2, >, nullable);

  util::Nullable<int> nullable3(nullable);
  TEST(nullable == nullable3);

  nullable3 = nullable2;
  TEST(nullable2 == nullable3);
  TEST(nullable3 == 24);
  TEST(nullable2 == 24);
  TEST(nullable2 == nullable2.get());
  TEST(int(nullable2) == nullable2.get());
  TEST(!nullable3.isNull());
  TEST(!nullable2.isNull());

  util::Nullable<int> voidnull;
  TEST(voidnull.isNull());
}
}

// ___________________________________________________________________________
{
  auto p = pointFromWKT<double>("POINT(10 50)");
  TEST(p.getX(), ==, approx(10));
  TEST(p.getY(), ==, approx(50));

  p = pointFromWKT<double>("POINT( 10 50)");
  TEST(p.getX(), ==, approx(10));
  TEST(p.getY(), ==, approx(50));

  p = pointFromWKT<double>("POINT (10 50 30)");
  TEST(p.getX(), ==, approx(10));
  TEST(p.getY(), ==, approx(50));

  p = pointFromWKT<double>("POINT (10     50 30)");
  TEST(p.getX(), ==, approx(10));
  TEST(p.getY(), ==, approx(50));

  p = pointFromWKT<double>("POINT(10 50 30)");
  TEST(p.getX(), ==, approx(10));
  TEST(p.getY(), ==, approx(50));

  p = pointFromWKT<double>("POINT (10    50) ");
  TEST(p.getX(), ==, approx(10));
  TEST(p.getY(), ==, approx(50));

  p = pointFromWKT<double>("MPOINT(10 50 30)");
  TEST(p.getX(), ==, approx(10));
  TEST(p.getY(), ==, approx(50));

  p = pointFromWKT<double>("MPOINT(10 50)");
  TEST(p.getX(), ==, approx(10));
  TEST(p.getY(), ==, approx(50));

  p = pointFromWKT<double>("POINT(10.05 50.05)");
  TEST(p.getX(), ==, approx(10.05));
  TEST(p.getY(), ==, approx(50.05));

  auto wktl = lineFromWKT<double>("LINESTRING(0 0, 1 1,2 3, 0 1)");
  TEST(wktl.size(), ==, (size_t)4);
  TEST(wktl[0].getX(), ==, approx(0));
  TEST(wktl[0].getY(), ==, approx(0));
  TEST(wktl[1].getX(), ==, approx(1));
  TEST(wktl[1].getY(), ==, approx(1));
  TEST(wktl[2].getX(), ==, approx(2));
  TEST(wktl[2].getY(), ==, approx(3));
  TEST(wktl[3].getX(), ==, approx(0));
  TEST(wktl[3].getY(), ==, approx(1));

  wktl = lineFromWKT<double>("MLINESTRING(0 0, 1 1,2 3, 0 1)");
  TEST(wktl.size(), ==, (size_t)4);
  TEST(wktl[0].getX(), ==, approx(0));
  TEST(wktl[0].getY(), ==, approx(0));
  TEST(wktl[1].getX(), ==, approx(1));
  TEST(wktl[1].getY(), ==, approx(1));
  TEST(wktl[2].getX(), ==, approx(2));
  TEST(wktl[2].getY(), ==, approx(3));
  TEST(wktl[3].getX(), ==, approx(0));
  TEST(wktl[3].getY(), ==, approx(1));

  wktl = lineFromWKT<double>("MLINESTRING (0 0, 1  1,2   3, 0 1 )");
  TEST(wktl.size(), ==, (size_t)4);
  TEST(wktl[0].getX(), ==, approx(0));
  TEST(wktl[0].getY(), ==, approx(0));
  TEST(wktl[1].getX(), ==, approx(1));
  TEST(wktl[1].getY(), ==, approx(1));
  TEST(wktl[2].getX(), ==, approx(2));
  TEST(wktl[2].getY(), ==, approx(3));
  TEST(wktl[3].getX(), ==, approx(0));
  TEST(wktl[3].getY(), ==, approx(1));
}
// ___________________________________________________________________________
{
  geo::Point<double> a(1, 2);
  geo::Point<double> b(2, 3);
  geo::Point<double> c(4, 5);
  TEST(a.getX(), ==, approx(1));
  TEST(a.getY(), ==, approx(2));

  a.setX(3);
  TEST(a.getX(), ==, approx(3));
  TEST(a.getY(), ==, approx(2));

  a.setY(4);
  TEST(a.getX(), ==, approx(3));
  TEST(a.getY(), ==, approx(4));

  auto d = a + b;
  TEST(d.getX(), ==, approx(5));
  TEST(d.getY(), ==, approx(7));

  a.setX(1);
  a.setY(2);

  TEST(geo::dist(a, a), ==, approx(0));
  TEST(geo::dist(a, b), ==, approx(sqrt(2)));

  d = d + d;

  geo::Box<double> box(a, c);
  TEST(geo::contains(a, box));
  TEST(geo::contains(b, box));
  TEST(geo::contains(c, box));
  TEST(!geo::contains(d, box));

  geo::Line<double> line{a, b, c};

  TEST(geo::contains(line, box));
  line.push_back(d);
  TEST(!geo::contains(line, box));

  geo::LineSegment<double> ls{a, b};
  TEST(geo::contains(a, ls));
  TEST(geo::contains(b, ls));
  TEST(!geo::contains(c, ls));
  TEST(geo::contains(a + geo::Point<double>(.5, .5), ls));
  TEST(!geo::contains(a + geo::Point<double>(1.5, 1.5), ls));

  geo::LineSegment<double> lsa{geo::Point<double>(1, 1),
                               geo::Point<double>(2, 2)};
  geo::LineSegment<double> lsb{geo::Point<double>(1, 2),
                               geo::Point<double>(2, 1)};
  geo::LineSegment<double> lsc{geo::Point<double>(2.1, 2),
                               geo::Point<double>(3, 3)};

  TEST(geo::crossProd(lsa.first, lsb), ==, approx(-1));
  TEST(geo::crossProd(lsa.second, lsb), ==, approx(1));

  TEST(geo::intersects(lsa, lsb));

  TEST(!geo::intersects(lsa, lsa));
  TEST(!geo::intersects(lsb, lsb));
  TEST(!geo::intersects(lsa, lsc));

  TEST(!geo::intersects(geo::Point<double>(871569.2, 6104550.4),
                          geo::Point<double>(871581.2, 6104536),
                          geo::Point<double>(871580.3, 6104541.3),
                          geo::Point<double>(871625.7, 6104510.1)));

  TEST(!geo::intersects(geo::Point<double>(0, 0), geo::Point<double>(1, 1),
                          geo::Point<double>(0.5, 0.5),
                          geo::Point<double>(1.5, 1.5)));

  geo::Line<double> l{geo::Point<double>(1, 1), geo::Point<double>(2, 2),
                      geo::Point<double>(2, 4)};
  TEST(!geo::contains(geo::Point<double>(1, 2), l));
  TEST(geo::contains(geo::Point<double>(2, 2), l));
  TEST(geo::contains(geo::Point<double>(2, 3), l));

  geo::Box<double> bbox(geo::Point<double>(1, 1), geo::Point<double>(3, 3));
  TEST(geo::intersects(l, bbox));
  geo::Line<double> ll{geo::Point<double>(0, 0), geo::Point<double>(4, 4)};
  TEST(geo::intersects(ll, bbox));
  geo::Line<double> lll{geo::Point<double>(0, 0), geo::Point<double>(0, 4)};
  TEST(!geo::intersects(lll, bbox));
  geo::Line<double> llll{geo::Point<double>(1.2, 0), geo::Point<double>(1, 2)};
  TEST(geo::intersects(llll, bbox));

  Line<double> l5new;
  l5new.push_back(Point<double>(-10, -5));
  l5new.push_back(Point<double>(-8, -4));
  TEST(geo::getBoundingBox(l5new).getUpperRight().getX(), ==, approx(-8));
  TEST(geo::getBoundingBox(l5new).getUpperRight().getY(), ==, approx(-4));

  Line<double> l5;
  l5.push_back(Point<double>(0, 0));
  l5.push_back(Point<double>(1.5, 2));
  Box<double> req(Point<double>(.5, 1), Point<double>(1, 1.5));

  TEST(geo::getBoundingBox(l5[0]).getLowerLeft().getX(), ==, approx(0));
  TEST(geo::getBoundingBox(l5[0]).getLowerLeft().getY(), ==, approx(0));

  TEST(geo::getBoundingBox(l5).getLowerLeft().getX(), ==, approx(0));
  TEST(geo::getBoundingBox(l5).getLowerLeft().getY(), ==, approx(0));
  TEST(geo::getBoundingBox(l5).getUpperRight().getX(), ==, approx(1.5));
  TEST(geo::getBoundingBox(l5).getUpperRight().getY(), ==, approx(2));
  TEST(geo::intersects(geo::getBoundingBox(l5),
                         geo::getBoundingBox(Line<double>{
                             Point<double>(.5, 1), Point<double>(1, 1)})));
  TEST(geo::intersects(
      l5, Line<double>{Point<double>(.5, 1), Point<double>(1, 1)}));
  TEST(geo::intersects(l5, req));

  Box<double> boxa(Point<double>(1, 1), Point<double>(2, 2));
  TEST(geo::intersects(
      boxa, Box<double>(Point<double>(1.5, 1.5), Point<double>(1.7, 1.7))));
  TEST(geo::intersects(
      boxa, Box<double>(Point<double>(0, 0), Point<double>(3, 3))));
  TEST(geo::intersects(
      boxa, Box<double>(Point<double>(1.5, 1.5), Point<double>(3, 3))));
  TEST(geo::intersects(
      boxa, Box<double>(Point<double>(0, 0), Point<double>(1.5, 1.5))));

  TEST(geo::intersects(
      Box<double>(Point<double>(1.5, 1.5), Point<double>(1.7, 1.7)), boxa));
  TEST(geo::intersects(Box<double>(Point<double>(0, 0), Point<double>(3, 3)),
                         boxa));
  TEST(geo::intersects(
      Box<double>(Point<double>(1.5, 1.5), Point<double>(3, 3)), boxa));
  TEST(geo::intersects(
      Box<double>(Point<double>(0, 0), Point<double>(1.5, 1.5)), boxa));

  Polygon<double> poly({Point<double>(1, 1), Point<double>(3, 2),
                        Point<double>(4, 3), Point<double>(6, 3),
                        Point<double>(5, 1)});
  TEST(geo::getWKT(poly), ==, "POLYGON ((1 1, 3 2, 4 3, 6 3, 5 1, 1 1))");
  TEST(geo::contains(Point<double>(4, 2), poly));
  TEST(!geo::contains(Point<double>(3, 3), poly));
  TEST(geo::contains(Point<double>(1, 1), poly));
  TEST(geo::contains(Point<double>(3, 2), poly));
  TEST(geo::contains(Point<double>(4, 3), poly));
  TEST(geo::contains(Point<double>(6, 3), poly));
  TEST(geo::contains(Point<double>(5, 1), poly));

  TEST(geo::contains(Line<double>{Point<double>(6, 3), Point<double>(5, 1)},
                       poly));
  TEST(!geo::contains(Line<double>{Point<double>(6, 3), Point<double>(50, 1)},
                        poly));
  TEST(geo::contains(Line<double>{Point<double>(4, 2), Point<double>(4.5, 2)},
                       poly));
  TEST(geo::contains(Line<double>{Point<double>(4, 2), Point<double>(5, 1)},
                       poly));

  Box<double> polybox(Point<double>(1, 1), Point<double>(6, 4));
  TEST(geo::centroid(polybox).getX(), ==, approx(3.5));
  TEST(geo::centroid(polybox).getY(), ==, approx(2.5));
  TEST(geo::contains(poly, polybox));
  TEST(!geo::contains(polybox, poly));
  Box<double> polybox2(Point<double>(4, 1), Point<double>(5, 2));
  TEST(geo::contains(polybox2, poly));
  TEST(geo::contains(poly, getBoundingBox(poly)));

  Point<double> rotP(2, 2);
  TEST(geo::dist(geo::rotate(rotP, 180, Point<double>(1, 1)),
                   Point<double>(0, 0)) == approx(0));
  TEST(geo::dist(geo::rotate(rotP, 360, Point<double>(1, 1)), rotP) ==
         approx(0));

  Line<double> rotLine({{1, 1}, {3, 3}});
  TEST(geo::rotate(rotLine, 90, Point<double>(2, 2))[0].getX(), ==, approx(1));
  TEST(geo::rotate(rotLine, 90, Point<double>(2, 2))[0].getY(), ==, approx(3));
  TEST(geo::rotate(rotLine, 90, Point<double>(2, 2))[1].getX(), ==, approx(3));
  TEST(geo::rotate(rotLine, 90, Point<double>(2, 2))[1].getY(), ==, approx(1));

  MultiLine<double> multiRotLine({{{1, 1}, {3, 3}}, {{1, 3}, {3, 1}}});
  TEST(geo::rotate(multiRotLine, 90, Point<double>(2, 2))[0][0].getX() ==
         approx(1));
  TEST(geo::rotate(multiRotLine, 90, Point<double>(2, 2))[0][0].getY() ==
         approx(3));
  TEST(geo::rotate(multiRotLine, 90, Point<double>(2, 2))[0][1].getX() ==
         approx(3));
  TEST(geo::rotate(multiRotLine, 90, Point<double>(2, 2))[0][1].getY() ==
         approx(1));
  TEST(geo::rotate(multiRotLine, 90, Point<double>(2, 2))[1][0].getX() ==
         approx(3));
  TEST(geo::rotate(multiRotLine, 90, Point<double>(2, 2))[1][0].getY() ==
         approx(3));
  TEST(geo::rotate(multiRotLine, 90, Point<double>(2, 2))[1][1].getX() ==
         approx(1));
  TEST(geo::rotate(multiRotLine, 90, Point<double>(2, 2))[1][1].getY() ==
         approx(1));

  TEST(geo::getWKT(multiRotLine) ==
         "MULTILINESTRING ((1 1, 3 3), (1 3, 3 1))");

  TEST(geo::contains(
      multiRotLine[0],
      geo::move(geo::move(multiRotLine, 1.0, 2.0), -1.0, -2.0)[0]));
  TEST(geo::contains(multiRotLine, geo::getBoundingBox(Line<double>{
                                         {1, 1}, {3, 3}, {1, 3}, {3, 1}})));

  TEST(geo::contains(
      getBoundingBox(multiRotLine),
      geo::getBoundingBox(Line<double>{{1, 1}, {3, 3}, {1, 3}, {3, 1}})));
  TEST(geo::contains(
      geo::getBoundingBox(Line<double>{{1, 1}, {3, 3}, {1, 3}, {3, 1}}),
      getBoundingBox(multiRotLine)));

  TEST(geo::dist(geo::centroid(rotP), rotP), ==, approx(0));
  TEST(geo::dist(geo::centroid(rotLine), rotP), ==, approx(0));
  TEST(geo::dist(geo::centroid(polybox), Point<double>(3.5, 2.5)) ==
         approx(0));
  TEST(geo::dist(geo::centroid(Polygon<double>({{0, 0}, {3, 4}, {4, 3}})),
                   Point<double>(7.0 / 3.0, 7.0 / 3.0)) == approx(0));

  auto polyy = Polygon<double>({{0, 0}, {3, 4}, {4, 3}});
  MultiPolygon<double> mpoly{polyy, polyy};

  TEST(geo::getWKT(polyy), ==, "POLYGON ((0 0, 3 4, 4 3, 0 0))");
  TEST(geo::getWKT(mpoly) ==
         "MULTIPOLYGON (((0 0, 3 4, 4 3, 0 0)), ((0 0, 3 4, 4 3, 0 0)))");

  auto hull = geo::convexHull(Line<double>{
      {0.1, 3}, {1, 1}, {2, 2}, {4, 4}, {0, 0}, {1, 2}, {3, 1}, {3, 3}});
  TEST(hull.getOuter().size(), ==, size_t(4));
  TEST(hull.getOuter()[0].getX(), ==, approx(0));
  TEST(hull.getOuter()[0].getY(), ==, approx(0));
  TEST(hull.getOuter()[1].getX(), ==, approx(0.1));
  TEST(hull.getOuter()[1].getY(), ==, approx(3));
  TEST(hull.getOuter()[2].getX(), ==, approx(4));
  TEST(hull.getOuter()[2].getY(), ==, approx(4));
  TEST(hull.getOuter()[3].getX(), ==, approx(3));
  TEST(hull.getOuter()[3].getY(), ==, approx(1));
  TEST(geo::contains(geo::convexHull(geo::getBoundingBox(poly)),
                       geo::getBoundingBox(poly)));
  TEST(geo::contains(geo::getBoundingBox(poly),
                       geo::convexHull(geo::getBoundingBox(poly))));

  auto hull2 = geo::convexHull(Line<double>{{0.1, 3},
                                            {1, 1},
                                            {2, 2},
                                            {4, 4},
                                            {0, 0},
                                            {1, 2},
                                            {3, 1},
                                            {3, 3},
                                            {-0.1, 1}});
  TEST(hull2.getOuter().size(), ==, size_t(5));
  TEST(hull2.getOuter()[0].getX(), ==, approx(-.1));
  TEST(hull2.getOuter()[0].getY(), ==, approx(1));
  TEST(hull2.getOuter()[1].getX(), ==, approx(0.1));
  TEST(hull2.getOuter()[1].getY(), ==, approx(3));
  TEST(hull2.getOuter()[2].getX(), ==, approx(4));
  TEST(hull2.getOuter()[2].getY(), ==, approx(4));
  TEST(hull2.getOuter()[3].getX(), ==, approx(3));
  TEST(hull2.getOuter()[3].getY(), ==, approx(1));
  TEST(hull2.getOuter()[4].getX(), ==, approx(0));
  TEST(hull2.getOuter()[4].getY(), ==, approx(0));

  auto hull3 =
      geo::convexHull(Line<double>{{0.1, 3}, {4, 4}, {0, 0}, {1, 2}, {3, 1}});
  TEST(hull3.getOuter().size(), ==, size_t(4));
  TEST(hull3.getOuter()[0].getX(), ==, approx(0));
  TEST(hull3.getOuter()[0].getY(), ==, approx(0));
  TEST(hull3.getOuter()[3].getX(), ==, approx(3));
  TEST(hull3.getOuter()[3].getY(), ==, approx(1));
  TEST(hull3.getOuter()[2].getX(), ==, approx(4));
  TEST(hull3.getOuter()[2].getY(), ==, approx(4));
  TEST(hull3.getOuter()[1].getX(), ==, approx(0.1));
  TEST(hull3.getOuter()[1].getY(), ==, approx(3));

  hull3 = geo::convexHull(
      Line<double>{{0.1, 3}, {4, 4}, {2, 1}, {3, 2}, {0, 0}, {1, 2}, {3, 1}});
  TEST(hull3.getOuter().size(), ==, size_t(4));
  TEST(hull3.getOuter()[0].getX(), ==, approx(0));
  TEST(hull3.getOuter()[0].getY(), ==, approx(0));
  TEST(hull3.getOuter()[3].getX(), ==, approx(3));
  TEST(hull3.getOuter()[3].getY(), ==, approx(1));
  TEST(hull3.getOuter()[2].getX(), ==, approx(4));
  TEST(hull3.getOuter()[2].getY(), ==, approx(4));
  TEST(hull3.getOuter()[1].getX(), ==, approx(0.1));
  TEST(hull3.getOuter()[1].getY(), ==, approx(3));

  hull3 = geo::convexHull(Line<double>{
      {4, 4}, {1, 2}, {2, 1}, {3, 2}, {0.1, 3}, {0, 0}, {1, 2}, {3, 1}});
  TEST(hull3.getOuter().size(), ==, size_t(4));
  TEST(hull3.getOuter()[0].getX(), ==, approx(0));
  TEST(hull3.getOuter()[0].getY(), ==, approx(0));
  TEST(hull3.getOuter()[3].getX(), ==, approx(3));
  TEST(hull3.getOuter()[3].getY(), ==, approx(1));
  TEST(hull3.getOuter()[2].getX(), ==, approx(4));
  TEST(hull3.getOuter()[2].getY(), ==, approx(4));
  TEST(hull3.getOuter()[1].getX(), ==, approx(0.1));
  TEST(hull3.getOuter()[1].getY(), ==, approx(3));

  hull3 = geo::convexHull(Line<double>{{4, 4}, {1, 2}, {3, 1}});
  TEST(hull3.getOuter().size(), ==, size_t(3));
  TEST(hull3.getOuter()[0].getX(), ==, approx(1));
  TEST(hull3.getOuter()[0].getY(), ==, approx(2));
  TEST(hull3.getOuter()[2].getX(), ==, approx(3));
  TEST(hull3.getOuter()[2].getY(), ==, approx(1));
  TEST(hull3.getOuter()[1].getX(), ==, approx(4));
  TEST(hull3.getOuter()[1].getY(), ==, approx(4));

  hull3 = geo::convexHull(Line<double>{{4, 4}, {1, 2}, {3, 10}});
  TEST(hull3.getOuter().size(), ==, size_t(3));
  TEST(hull3.getOuter()[0].getX(), ==, approx(1));
  TEST(hull3.getOuter()[0].getY(), ==, approx(2));
  TEST(hull3.getOuter()[2].getX(), ==, approx(4));
  TEST(hull3.getOuter()[2].getY(), ==, approx(4));
  TEST(hull3.getOuter()[1].getX(), ==, approx(3));
  TEST(hull3.getOuter()[1].getY(), ==, approx(10));

  Line<double> test{{0.3215348546593775, 0.03629583077160248},
                    {0.02402358131857918, -0.2356728797179394},
                    {0.04590851212470659, -0.4156409924995536},
                    {0.3218384001607433, 0.1379850698988746},
                    {0.11506479756447, -0.1059521474930943},
                    {0.2622539999543261, -0.29702873322836},
                    {-0.161920957418085, -0.4055339716426413},
                    {0.1905378631228002, 0.3698601009043493},
                    {0.2387090918968516, -0.01629827079949742},
                    {0.07495888748668034, -0.1659825110491202},
                    {0.3319341836794598, -0.1821814101954749},
                    {0.07703635755650362, -0.2499430638271785},
                    {0.2069242999022122, -0.2232970760420869},
                    {0.04604079532068295, -0.1923573186549892},
                    {0.05054295812784038, 0.4754929463150845},
                    {-0.3900589168910486, 0.2797829520700341},
                    {0.3120693385713448, -0.0506329867529059},
                    {0.01138812723698857, 0.4002504701728471},
                    {0.009645149586391732, 0.1060251100976254},
                    {-0.03597933197019559, 0.2953639456959105},
                    {0.1818290866742182, 0.001454397571696298},
                    {0.444056063372694, 0.2502497166863175},
                    {-0.05301752458607545, -0.06553921621808712},
                    {0.4823896228171788, -0.4776170002088109},
                    {-0.3089226845734964, -0.06356112199235814},
                    {-0.271780741188471, 0.1810810595574612},
                    {0.4293626522918815, 0.2980897964891882},
                    {-0.004796652127799228, 0.382663812844701},
                    {0.430695573269106, -0.2995073500084759},
                    {0.1799668387323309, -0.2973467472915973},
                    {0.4932166845474547, 0.4928094162538735},
                    {-0.3521487911717489, 0.4352656197131292},
                    {-0.4907368011686362, 0.1865826865533206},
                    {-0.1047924716070224, -0.247073392148198},
                    {0.4374961861758457, -0.001606279519951237},
                    {0.003256207800708899, -0.2729194320486108},
                    {0.04310378203457577, 0.4452604050238248},
                    {0.4916198379282093, -0.345391701297268},
                    {0.001675087028811806, 0.1531837672490476},
                    {-0.4404289572876217, -0.2894855991839297}

  };
  hull3 = geo::convexHull(test);
  TEST(geo::contains(test, hull3));
  TEST(hull3.getOuter().size(), ==, size_t(8));
  TEST(geo::contains(
      Polygon<double>({{-0.161920957418085, -0.4055339716426413},
                       {0.05054295812784038, 0.4754929463150845},
                       {0.4823896228171788, -0.4776170002088109},
                       {0.4932166845474547, 0.4928094162538735},
                       {-0.3521487911717489, 0.4352656197131292},
                       {-0.4907368011686362, 0.1865826865533206},
                       {0.4916198379282093, -0.345391701297268},
                       {-0.4404289572876217, -0.2894855991839297}}),
      hull3));
  TEST(geo::contains(
      hull3, Polygon<double>({{-0.161920957418085, -0.4055339716426413},
                              {0.05054295812784038, 0.4754929463150845},
                              {0.4823896228171788, -0.4776170002088109},
                              {0.4932166845474547, 0.4928094162538735},
                              {-0.3521487911717489, 0.4352656197131292},
                              {-0.4907368011686362, 0.1865826865533206},
                              {0.4916198379282093, -0.345391701297268},
                              {-0.4404289572876217, -0.2894855991839297}})));

  hull3 = geo::convexHull(Line<double>{{3, 6},
                                       {8, 10},
                                       {3, 5},
                                       {20, -10},
                                       {-4, 5},
                                       {10, 2},
                                       {5, 1},
                                       {45, 1},
                                       {30, -9},
                                       {3, 14},
                                       {25, -5.5}});
  TEST(hull3.getOuter().size(), ==, size_t(5));
  TEST(hull3.getOuter()[0].getX(), ==, approx(-4));
  TEST(hull3.getOuter()[0].getY(), ==, approx(5));
  TEST(hull3.getOuter()[4].getX(), ==, approx(20));
  TEST(hull3.getOuter()[4].getY(), ==, approx(-10));
  TEST(hull3.getOuter()[3].getX(), ==, approx(30));
  TEST(hull3.getOuter()[3].getY(), ==, approx(-9));
  TEST(hull3.getOuter()[2].getX(), ==, approx(45));
  TEST(hull3.getOuter()[2].getY(), ==, approx(1));
  TEST(hull3.getOuter()[1].getX(), ==, approx(3));
  TEST(hull3.getOuter()[1].getY(), ==, approx(14));

  hull3 = geo::convexHull(Line<double>{
      {7, 7}, {7, -7}, {-7, -7}, {-7, 7}, {9, 0}, {-9, 0}, {0, 9}, {0, -9}});
  TEST(hull3.getOuter().size(), ==, size_t(8));
  TEST(geo::contains(geo::Polygon<double>({{-9, 0},
                                             {-7, -7},
                                             {0, -9},
                                             {7, -7},
                                             {9, 0},
                                             {7, 7},
                                             {0, 9},
                                             {-7, 7}}),
                       hull3));
  TEST(geo::contains(hull3, geo::Polygon<double>({{-9, 0},
                                                    {-7, -7},
                                                    {0, -9},
                                                    {7, -7},
                                                    {9, 0},
                                                    {7, 7},
                                                    {0, 9},
                                                    {-7, 7}})));

  hull3 = geo::convexHull(Line<double>{{7, 7},
                                       {7, -7},
                                       {-7, -7},
                                       {-7, 7},
                                       {9, 0},
                                       {-9, 0},
                                       {0, 9},
                                       {0, -9},
                                       {0, 0},
                                       {1, 2},
                                       {-2, 1},
                                       {-1, -1},
                                       {3, 4},
                                       {4, 3},
                                       {-5, 4},
                                       {6, 5}});
  TEST(hull3.getOuter().size(), ==, size_t(8));
  TEST(geo::contains(geo::Polygon<double>({{-9, 0},
                                             {-7, -7},
                                             {0, -9},
                                             {7, -7},
                                             {9, 0},
                                             {7, 7},
                                             {0, 9},
                                             {-7, 7}}),
                       hull3));
  TEST(geo::contains(hull3, geo::Polygon<double>({{-9, 0},
                                                    {-7, -7},
                                                    {0, -9},
                                                    {7, -7},
                                                    {9, 0},
                                                    {7, 7},
                                                    {0, 9},
                                                    {-7, 7}})));

  hull3 = geo::convexHull(Line<double>{
      {0, 0},   {1, 2},  {-2, 1}, {-1, -1}, {3, 4},   {4, 3},   {-5, 4},
      {6, 5},   {7, 7},  {7, -7}, {-7, -7}, {-7, 7},  {9, 0},   {-9, 0},
      {0, 9},   {0, -9}, {-8, 0}, {8, 0},   {-7, 0},  {7, 0},   {-6, 0},
      {6, 0},   {-5, 0}, {5, 0},  {-4, 0},  {4, 0},   {-3, 0},  {3, 0},
      {-2, 0},  {2, 0},  {-1, 0}, {1, 0},   {0, -8},  {0, 8},   {0, -7},
      {0, 7},   {0, -6}, {0, 6},  {0, -5},  {0, 5},   {0, -4},  {0, 4},
      {0, -3},  {0, 3},  {0, -2}, {0, 2},   {0, -1},  {0, 1},   {1, 1},
      {2, 2},   {3, 3},  {4, 4},  {5, 5},   {6, 6},   {1, -1},  {2, -2},
      {3, -3},  {4, -4}, {5, -5}, {6, -6},  {-1, 1},  {-2, 2},  {-3, 3},
      {-4, 4},  {-5, 5}, {-6, 6}, {-1, -1}, {-2, -2}, {-3, -3}, {-4, -4},
      {-5, -5}, {-6, -6}});
  TEST(hull3.getOuter().size(), ==, size_t(8));
  TEST(geo::contains(geo::Polygon<double>({{-9, 0},
                                             {-7, -7},
                                             {0, -9},
                                             {7, -7},
                                             {9, 0},
                                             {7, 7},
                                             {0, 9},
                                             {-7, 7}}),
                       hull3));
  TEST(geo::contains(hull3, geo::Polygon<double>({{-9, 0},
                                                    {-7, -7},
                                                    {0, -9},
                                                    {7, -7},
                                                    {9, 0},
                                                    {7, 7},
                                                    {0, 9},
                                                    {-7, 7}})));

  TEST(geo::area(geo::Point<double>(1, 2)), ==, approx(0));
  TEST(geo::area(geo::Line<double>{{1, 2}, {2, 5}}), ==, approx(0));
  TEST(geo::area(geo::Box<double>({0, 0}, {1, 1})), ==, approx(1));
  TEST(geo::area(geo::Box<double>({1, 1}, {1, 1})), ==, approx(0));
  TEST(geo::area(geo::Box<double>({0, 0}, {2, 2})), ==, approx(4));
  TEST(geo::area(geo::Polygon<double>({{0, 0}, {1, 0}, {1, 1}, {0, 1}})) ==
         approx(1));
  TEST(geo::area(geo::Polygon<double>({{0, 0}, {1, 0}, {1, 1}})) ==
         approx(0.5));

  auto obox =
      geo::getOrientedEnvelope(geo::Line<double>{{0, 0}, {1, 1}, {1.5, 0.5}});
  TEST(geo::contains(
      geo::convexHull(obox),
      geo::Polygon<double>({{0.0, 0.0}, {1.0, 1.0}, {1.5, 0.5}, {0.5, -0.5}})));
  TEST(geo::contains(
      geo::Polygon<double>({{0.0, 0.0}, {1.0, 1.0}, {1.5, 0.5}, {0.5, -0.5}}),
      geo::convexHull(obox)));

  TEST(geo::dist(geo::LineSegment<double>{{1, 1}, {3, 1}},
                   geo::LineSegment<double>{{2, 2}, {2, 0}}) == approx(0));
  TEST(geo::dist(geo::LineSegment<double>{{1, 1}, {3, 1}},
                   geo::LineSegment<double>{{2, 4}, {2, 2}}) == approx(1));
  TEST(geo::dist(geo::LineSegment<double>{{1, 1}, {3, 1}},
                   geo::LineSegment<double>{{1, 1}, {3, 1}}) == approx(0));
  TEST(geo::dist(geo::LineSegment<double>{{1, 1}, {3, 1}},
                   geo::LineSegment<double>{{1, 2}, {3, 2}}) == approx(1));
  TEST(geo::dist(geo::LineSegment<double>{{1, 1}, {3, 1}},
                   geo::LineSegment<double>{{1, 2}, {3, 5}}) == approx(1));

  TEST(geo::dist(geo::Line<double>{{1, 1}, {3, 1}},
                   geo::Point<double>{2, 1}) == approx(0));
  TEST(geo::dist(geo::Line<double>{{1, 1}, {3, 1}},
                   geo::Point<double>{2, 2}) == approx(1));
  TEST(geo::dist(geo::Line<double>{{1, 1}, {3, 1}},
                   geo::Point<double>{3, 1}) == approx(0));
  TEST(geo::dist(geo::Line<double>{{1, 1}, {3, 1}},
                   geo::Point<double>{1, 1}) == approx(0));

  TEST(geo::dist(Line<double>{{7, 7},
                                {7, -7},
                                {-7, -7},
                                {-7, 7},
                                {9, 0},
                                {-9, 0},
                                {0, 9},
                                {0, -9}},
                   Line<double>{{7, 7},
                                {7, -7},
                                {-7, -7},
                                {-7, 7},
                                {9, 0},
                                {-9, 0},
                                {0, 9},
                                {0, -9}}) == approx(0));
  TEST(geo::dist(Line<double>{{7, 7},
                                {7, -7},
                                {-7, -7},
                                {-7, 7},
                                {9, 0},
                                {-9, 0},
                                {0, 9},
                                {0, -9}},
                   LineSegment<double>{{6, 7}, {8, -7}}) == approx(0));
  TEST(geo::dist(Line<double>{{7, 7},
                                {7, -7},
                                {-7, -7},
                                {-7, 7},
                                {9, 0},
                                {-9, 0},
                                {0, 9},
                                {0, -9}},
                   Point<double>{7, 4}) == approx(0));
  TEST(geo::dist(Line<double>{{0, 0}, {1, 1}, {2, 0}},
                   Line<double>{{1.5, 0.5}, {1.5, 100}}) == approx(0));
  TEST(geo::dist(Line<double>{{0, 0}, {1, 1}, {2, 0}},
                   Line<double>{{2, 0.5}, {2, 100}}) == approx(0.353553));

  TEST(geo::contains(util::geo::Point<double>{1.5, 0.5},
                       util::geo::LineSegment<double>{{1, 1}, {1.5, 0.5}}));
  TEST(geo::contains(util::geo::Point<double>{1.5, 0.5},
                       util::geo::LineSegment<double>{{1, 1}, {1.5, 0.5}}));

  auto polyTest =
      geo::Polygon<double>({{1, 1}, {3, 1}, {2, 2}, {3, 3}, {1, 3}});
  TEST(!geo::contains(util::geo::LineSegment<double>({2.5, 1.3}, {2.5, 2.6}),
                        polyTest));

  TEST(!geo::contains(util::geo::LineSegment<double>{{2.5, 1.3}, {2.5, 2.6}},
                        polyTest));
  TEST(geo::contains(util::geo::LineSegment<double>{{2.5, 2.6}, {1.5, 2}},
                       polyTest));
  TEST(!geo::contains(
      util::geo::Line<double>{{2.5, 1.3}, {2.5, 2.6}, {1.5, 2}}, polyTest));
  TEST(geo::contains(
      util::geo::Line<double>{{2.5, 1.3}, {1.5, 2}, {2.5, 2.6}}, polyTest));

  TEST(!geo::contains(util::geo::Box<double>{{1, 1}, {2.5, 2.6}}, polyTest));

  TEST(
      geo::intersects(Box<double>(Point<double>(0, 0), Point<double>(10, 10)),
                      Box<double>(Point<double>(2, 2), Point<double>(8, 8))));
  TEST(
      geo::intersects(Box<double>(Point<double>(0, 0), Point<double>(10, 10)),
                      Box<double>(Point<double>(-2, -2), Point<double>(8, 8))));
  TEST(geo::intersects(
      Box<double>(Point<double>(0, 0), Point<double>(10, 10)),
      Box<double>(Point<double>(-2, -2), Point<double>(12, 12))));
  TEST(
      geo::intersects(Box<double>(Point<double>(0, 0), Point<double>(10, 10)),
                      Box<double>(Point<double>(5, 5), Point<double>(12, 12))));

  TEST(!geo::intersects(
      Box<double>(Point<double>(0, 0), Point<double>(10, 10)),
      Box<double>(Point<double>(15, 15), Point<double>(12, 12))));

  double rad = 10.0;
  int n = 20;
  util::geo::MultiPoint<double> mp;

  for (int i = 0; i < n; i++) {
    double x = rad * cos((2.0 * M_PI / static_cast<double>(n)) *
                         static_cast<double>(i));
    double y = rad * sin((2.0 * M_PI / static_cast<double>(n)) *
                         static_cast<double>(i));

    mp.push_back(util::geo::DPoint(x, y));
  }

  auto h = util::geo::convexHull(mp);

  TEST(geo::contains(mp, h));

  TEST(geo::dist(geo::interpolate(DPoint(1.0, 1.0), DPoint(1.0, 3.0), 0.5), DPoint(1.0, 2.0)), ==, approx(0));
  TEST(geo::dist(geo::interpolate(DPoint(1.0, 1.0), DPoint(2.0, 2.0), 0), DPoint(1.0, 1.0)), ==, approx(0));
  TEST(geo::dist(geo::interpolate(DPoint(1.0, 1.0), DPoint(2.0, 2.0), 0.5), DPoint(1.5, 1.5)), ==, approx(0));
  TEST(geo::dist(geo::interpolate(DPoint(1.0, 1.0), DPoint(2.0, 2.0), 1), DPoint(2, 2)), ==, approx(0));
  TEST(geo::dist(geo::pointAtDist(DLine{{0, 0}, {0, 1}, {0, 2}}, 1), DPoint{0, 1}), ==, approx(0));
  TEST(geo::dist(geo::pointAtDist(DLine{{0, 0}, {0, 1}, {0, 2}}, 2), DPoint{0, 2}), ==, approx(0));
  TEST(geo::dist(geo::pointAtDist(DLine{{0, 0}, {0, 1}, {0, 2}}, 0), DPoint{0, 0}), ==, approx(0));

  TEST(geo::getWKT(geo::orthoLineAtDist(DLine{{0, 0}, {0, 1}, {0, 2}}, 1, 1)), ==, "LINESTRING (-0.5 1, 0.5 1)");

  TEST(geo::getWKT(geo::segment(DLine{{0, 0}, {0, 1}, {0, 2}}, 0, 0.5)), ==, "LINESTRING (0 0, 0 1)");
  TEST(geo::getWKT(geo::segment(DLine{{0, 0}, {0, 1}, {0, 2}}, 0.5, 1)), ==, "LINESTRING (0 1, 0 2)");
}

  // inversion count
  std::vector<int> test = {2, 1};
  TEST(inversions(test), ==, 1);

  test = {};
  TEST(inversions(test), ==, 0);

  test = {2};
  TEST(inversions(test), ==, 0);

  test = {2, 1};
  TEST(inversions(test), ==, 1);

  test = {1, 2};
  TEST(inversions(test), ==, 0);

  test = {2, 1, 3};
  TEST(inversions(test), ==, 1);

  test = {2, 3, 1};
  TEST(inversions(test), ==, 2);

  test = {3, 2, 1};
  TEST(inversions(test), ==, 3);

  test = {1, 2, 3};
  TEST(inversions(test), ==, 0);

  test = {1, 3, 2, 6, 5, 4, 8, 7, 9};
  TEST(inversions(test), ==, 5);

  test = {1, 2, 3, 4, 5, 6};
  TEST(inversions(test), ==, 0);

  test = {9, 8, 7, 6, 5, 4, 3, 2, 1};
  TEST(inversions(test), ==, 8 + 7 + 6 + 5 + 4 + 3 + 2 + 1);

  // nice float formatting
	TEST(formatFloat(15.564, 3), ==, "15.564");
	TEST(formatFloat(15.564, 0), ==, "16");
	TEST(formatFloat(15.0000, 10), ==, "15");
	TEST(formatFloat(15.0100, 10), ==, "15.01");
	TEST(formatFloat(0.0000, 10), ==, "0");
	TEST(formatFloat(-1.0000, 10), ==, "-1");
	TEST(formatFloat(-15.000001, 10), ==, "-15.000001");
}
