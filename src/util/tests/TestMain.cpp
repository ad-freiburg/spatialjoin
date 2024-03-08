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

  {
    auto a = lineFromWKT<double>("LINESTRING(5.816117 51.1096889,5.8162167 51.1091479)");
    auto b = lineFromWKT<double>("LINESTRING(5.8099984 51.1096468,5.8288865 51.1070398)");


    TEST(intersects(getBoundingBox(a), getBoundingBox(b)));

    auto ls1 = LineSegment<double>{a[0], a[1]};
    auto ls2 = LineSegment<double>{b[0], b[1]};

    TEST(!contains(ls1.first, ls2));
    TEST(!contains(ls1.second, ls2));
    TEST(!contains(ls2.first, ls1));
    TEST(!contains(ls2.second, ls1));

    TEST(!(((crossProd(ls1.first, ls2) < 0) ^
            (crossProd(ls1.second, ls2) < 0)) &&
           ((crossProd(ls2.first, ls1) < 0) ^
            (crossProd(ls2.second, ls1) < 0))));

    TEST(!geo::intersects(ls1, ls2));

    TEST(!geo::intersects(a, b));
    TEST(!geo::intersects(b, a));
  }

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

    TEST(geo::intersects(bp[0].getOuter(), ap[0].getOuter()));

    TEST(geo::intersects(bpp.getOuter(), app.getOuter()));

    TEST(geo::intersects(ap[0].getOuter(), bp[0].getOuter()));
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

    {
    auto a = lineFromWKT<double>("LINESTRING(7.9173212 47.9753418,7.9175186 47.9752904)");
    auto b = polygonFromWKT<double>("POLYGON((7.6620055 47.9656508,7.6630102 47.9665559,7.6629048 47.9665955,7.6628829 47.9666037,7.6628610 47.9666095,7.6627878 47.9666289,7.6630373 47.9669817,7.6632727 47.9674659,7.6637081 47.9673955,7.6638901 47.9677598,7.6639145 47.9678088,7.6639264 47.9678326,7.6637319 47.9678627,7.6640616 47.9686563,7.6643310 47.9686358,7.6647225 47.9700551,7.6649306 47.9700420,7.6653140 47.9699707,7.6653266 47.9699850,7.6653464 47.9700072,7.6656322 47.9703280,7.6654436 47.9704101,7.6658590 47.9708978,7.6660904 47.9712826,7.6665375 47.9712221,7.6665801 47.9713934,7.6666246 47.9715414,7.6667855 47.9719724,7.6667640 47.9720999,7.6666876 47.9722993,7.6668744 47.9723343,7.6669483 47.9725569,7.6671125 47.9728599,7.6671304 47.9728928,7.6675553 47.9736099,7.6675694 47.9736337,7.6690714 47.9732557,7.6692243 47.9732169,7.6693559 47.9731834,7.6692777 47.9733036,7.6690887 47.9735539,7.6700912 47.9739607,7.6702769 47.9737657,7.6714034 47.9738591,7.6712492 47.9740710,7.6712922 47.9740727,7.6724646 47.9741189,7.6734807 47.9742380,7.6736484 47.9743262,7.6739315 47.9743326,7.6746221 47.9743774,7.6746957 47.9743732,7.6747611 47.9743695,7.6748064 47.9743668,7.6750553 47.9743527,7.6753087 47.9742382,7.6755015 47.9742747,7.6755705 47.9742928,7.6758402 47.9743704,7.6758822 47.9743866,7.6760004 47.9744246,7.6764452 47.9745697,7.6764528 47.9745729,7.6765791 47.9746155,7.6765991 47.9746536,7.6769135 47.9748808,7.6770655 47.9748226,7.6771711 47.9747378,7.6773039 47.9747595,7.6777060 47.9748253,7.6778016 47.9748409,7.6778779 47.9748651,7.6780110 47.9749132,7.6780181 47.9749154,7.6780758 47.9749330,7.6781510 47.9749560,7.6782817 47.9749962,7.6784195 47.9750384,7.6784773 47.9750609,7.6784204 47.9750888,7.6781283 47.9752318,7.6778232 47.9754603,7.6776310 47.9756654,7.6780450 47.9757758,7.6781200 47.9757995,7.6785053 47.9759052,7.6785266 47.9759110,7.6783968 47.9760852,7.6783487 47.9761494,7.6782718 47.9763192,7.6782590 47.9764263,7.6782503 47.9765643,7.6782777 47.9767058,7.6783034 47.9767750,7.6783670 47.9768056,7.6784196 47.9768145,7.6785596 47.9768169,7.6786555 47.9768202,7.6788129 47.9768215,7.6787850 47.9768862,7.6787743 47.9769401,7.6787903 47.9770766,7.6788359 47.9772019,7.6788858 47.9773195,7.6790904 47.9778075,7.6790818 47.9778114,7.6789509 47.9780272,7.6789422 47.9780460,7.6784867 47.9782188,7.6781172 47.9783484,7.6781620 47.9786257,7.6782646 47.9790206,7.6784180 47.9794016,7.6785863 47.9796996,7.6789129 47.9801800,7.6791950 47.9804815,7.6792677 47.9806067,7.6794301 47.9808130,7.6795617 47.9807014,7.6795871 47.9807647,7.6798627 47.9814508,7.6799156 47.9815698,7.6801707 47.9821069,7.6801893 47.9822907,7.6801604 47.9824873,7.6801668 47.9826264,7.6801408 47.9827565,7.6801304 47.9829256,7.6801444 47.9829857,7.6802243 47.9832106,7.6802746 47.9832634,7.6805082 47.9834153,7.6806581 47.9834690,7.6807087 47.9834871,7.6808917 47.9835574,7.6811467 47.9834799,7.6813812 47.9834002,7.6817203 47.9832854,7.6818061 47.9832586,7.6818171 47.9832768,7.6818660 47.9833235,7.6819079 47.9833659,7.6819856 47.9833227,7.6821723 47.9832496,7.6822721 47.9833026,7.6823973 47.9833703,7.6824842 47.9834080,7.6825093 47.9834190,7.6826547 47.9835871,7.6828267 47.9837541,7.6828645 47.9838202,7.6829219 47.9839311,7.6831242 47.9843221,7.6833384 47.9846564,7.6828536 47.9847716,7.6828239 47.9847787,7.6829418 47.9850186,7.6831222 47.9854066,7.6832481 47.9857807,7.6833015 47.9859393,7.6834623 47.9862951,7.6834104 47.9863508,7.6834891 47.9865700,7.6835705 47.9867619,7.6836607 47.9869247,7.6836788 47.9870473,7.6838659 47.9870376,7.6843586 47.9875206,7.6845428 47.9877468,7.6847661 47.9879554,7.6848783 47.9880701,7.6844387 47.9882462,7.6842659 47.9882265,7.6840771 47.9883267,7.6837613 47.9884942,7.6837577 47.9884962,7.6837238 47.9885142,7.6840096 47.9888696,7.6839706 47.9888871,7.6833910 47.9891208,7.6833805 47.9891221,7.6835955 47.9893437,7.6831308 47.9895268,7.6830850 47.9895427,7.6830829 47.9895457,7.6830182 47.9895660,7.6827414 47.9896427,7.6824269 47.9894054,7.6821313 47.9891811,7.6809114 47.9898966,7.6814198 47.9904016,7.6812048 47.9905860,7.6811550 47.9906287,7.6806179 47.9909278,7.6804910 47.9910150,7.6803562 47.9910665,7.6797971 47.9912626,7.6796576 47.9913129,7.6796720 47.9913489,7.6797212 47.9914718,7.6794642 47.9915241,7.6789664 47.9916256,7.6787877 47.9916620,7.6791184 47.9921410,7.6792528 47.9925596,7.6793863 47.9928807,7.6796880 47.9930134,7.6791448 47.9934132,7.6785249 47.9940422,7.6783100 47.9947168,7.6782919 47.9947372,7.6783746 47.9948839,7.6784033 47.9949720,7.6784822 47.9952379,7.6787263 47.9955463,7.6789070 47.9958097,7.6789670 47.9957913,7.6791544 47.9960579,7.6796610 47.9965422,7.6796204 47.9965901,7.6799106 47.9968491,7.6803795 47.9972675,7.6804114 47.9972584,7.6810410 47.9977892,7.6809943 47.9978148,7.6815387 47.9982133,7.6817703 47.9983546,7.6817868 47.9983427,7.6818122 47.9983241,7.6820966 47.9984763,7.6822653 47.9985657,7.6824625 47.9986289,7.6827290 47.9987241,7.6827165 47.9987550,7.6835746 47.9990990,7.6846215 47.9995065,7.6851320 47.9997054,7.6851658 47.9996695,7.6854927 47.9998195,7.6858339 47.9999789,7.6861174 48.0001247,7.6863834 48.0002761,7.6866720 48.0004621,7.6871269 48.0008325,7.6873419 48.0010153,7.6876130 48.0011905,7.6878887 48.0013710,7.6881457 48.0015608,7.6883488 48.0017102,7.6887608 48.0019483,7.6889529 48.0020380,7.6889915 48.0020888,7.6893103 48.0022353,7.6897755 48.0024632,7.6911889 48.0030142,7.6917026 48.0032202,7.6922105 48.0034016,7.6927952 48.0036391,7.6932258 48.0037837,7.6934438 48.0038228,7.6937186 48.0038617,7.6937365 48.0038944,7.6938421 48.0040978,7.6938632 48.0041385,7.6939528 48.0043112,7.6939742 48.0043524,7.6933706 48.0044285,7.6932123 48.0044484,7.6932737 48.0046888,7.6938902 48.0046277,7.6940259 48.0049330,7.6940421 48.0049991,7.6940821 48.0050822,7.6942607 48.0053690,7.6943056 48.0054770,7.6945136 48.0054845,7.6948548 48.0055963,7.6949055 48.0056460,7.6950160 48.0057543,7.6952745 48.0060078,7.6953747 48.0061340,7.6958056 48.0065362,7.6963644 48.0069037,7.6970732 48.0072845,7.6991927 48.0084681,7.6996951 48.0080452,7.7008165 48.0085336,7.7020546 48.0090211,7.7021205 48.0089975,7.7051376 48.0099248,7.7052538 48.0099597,7.7077053 48.0107995,7.7084084 48.0110329,7.7084205 48.0111307,7.7083789 48.0111827,7.7082936 48.0112016,7.7082126 48.0112195,7.7081634 48.0112359,7.7081482 48.0112410,7.7078854 48.0112016,7.7074839 48.0112206,7.7068401 48.0113929,7.7060140 48.0116118,7.7048698 48.0118025,7.7047014 48.0118280,7.7048830 48.0122296,7.7052930 48.0130840,7.7057510 48.0140430,7.7055638 48.0141238,7.7053316 48.0143148,7.7054252 48.0145389,7.7055151 48.0148168,7.7055291 48.0150637,7.7056551 48.0151903,7.7053653 48.0152908,7.7051174 48.0154379,7.7060177 48.0162639,7.7058553 48.0163142,7.7042214 48.0166863,7.7042529 48.0167222,7.7042771 48.0167498,7.7049491 48.0175162,7.7049877 48.0175602,7.7039973 48.0178401,7.7040055 48.0178503,7.7040102 48.0178578,7.7045444 48.0185215,7.7045616 48.0185429,7.7036041 48.0186539,7.7036051 48.0186909,7.7036157 48.0190943,7.7038052 48.0190912,7.7038244 48.0191197,7.7042197 48.0197082,7.7045343 48.0202841,7.7045792 48.0203849,7.7037536 48.0205046,7.7039637 48.0213354,7.7039869 48.0214006,7.7039988 48.0214340,7.7039081 48.0214326,7.7034282 48.0214251,7.7030934 48.0214371,7.7029606 48.0214529,7.7029417 48.0214607,7.7030525 48.0216039,7.7032455 48.0219154,7.7032498 48.0219222,7.7040058 48.0231221,7.7032941 48.0233293,7.7033012 48.0233374,7.7033294 48.0233693,7.7034984 48.0235610,7.7036147 48.0237417,7.7036893 48.0238964,7.7037019 48.0239540,7.7037089 48.0239857,7.7037182 48.0240280,7.7037619 48.0249421,7.7037640 48.0249878,7.7037707 48.0251273,7.7035687 48.0253015,7.7034409 48.0259589,7.7033528 48.0260076,7.7030183 48.0260761,7.7024422 48.0260230,7.7024270 48.0261148,7.7019578 48.0261597,7.7017567 48.0262903,7.7016318 48.0265478,7.7016826 48.0267608,7.7018284 48.0268978,7.7023766 48.0268912,7.7029523 48.0270295,7.7030037 48.0271036,7.7027349 48.0275258,7.7022576 48.0275313,7.7021749 48.0277680,7.7023448 48.0278848,7.7025751 48.0280977,7.7029164 48.0282851,7.7035392 48.0288063,7.7039962 48.0288928,7.7041821 48.0289050,7.7042423 48.0291244,7.7041990 48.0292535,7.7047051 48.0297148,7.7047367 48.0298355,7.7047817 48.0299436,7.7047923 48.0299690,7.7049439 48.0302833,7.7051641 48.0305972,7.7052444 48.0306666,7.7048739 48.0307796,7.7047842 48.0308070,7.7047900 48.0308138,7.7055517 48.0316983,7.7063025 48.0323632,7.7066560 48.0327742,7.7067719 48.0329344,7.7068351 48.0330218,7.7066615 48.0331499,7.7067186 48.0332026,7.7067172 48.0332167,7.7067353 48.0332382,7.7075261 48.0341794,7.7072636 48.0343085,7.7073192 48.0343532,7.7073343 48.0343644,7.7077221 48.0347348,7.7079007 48.0348861,7.7079162 48.0348987,7.7084079 48.0352964,7.7091214 48.0350308,7.7091505 48.0350211,7.7096601 48.0348402,7.7101872 48.0346146,7.7102200 48.0346006,7.7102832 48.0345735,7.7113195 48.0342278,7.7113330 48.0342233,7.7113441 48.0342198,7.7117704 48.0340856,7.7122725 48.0339275,7.7123567 48.0339008,7.7125854 48.0338282,7.7126450 48.0338093,7.7127479 48.0339600,7.7127935 48.0339508,7.7128156 48.0339480,7.7137737 48.0338263,7.7138077 48.0338220,7.7138332 48.0338188,7.7138766 48.0338134,7.7137175 48.0331477,7.7136893 48.0330295,7.7138170 48.0330475,7.7143056 48.0331166,7.7150294 48.0332165,7.7150450 48.0332187,7.7150641 48.0332213,7.7155313 48.0332859,7.7155616 48.0332901,7.7159015 48.0333370,7.7161390 48.0333660,7.7162107 48.0333797,7.7162738 48.0333883,7.7163149 48.0333939,7.7166401 48.0334383,7.7167150 48.0336413,7.7166978 48.0338703,7.7166716 48.0340755,7.7166612 48.0341569,7.7166985 48.0342806,7.7167217 48.0343894,7.7167391 48.0344713,7.7168686 48.0344621,7.7176094 48.0343683,7.7177405 48.0343517,7.7178476 48.0343446,7.7179998 48.0343345,7.7180579 48.0343314,7.7185434 48.0343052,7.7185472 48.0343283,7.7186177 48.0347630,7.7186615 48.0347550,7.7186946 48.0347490,7.7187250 48.0347453,7.7196197 48.0346362,7.7196363 48.0346342,7.7196279 48.0346037,7.7195571 48.0343485,7.7196044 48.0343422,7.7196512 48.0343359,7.7196727 48.0343330,7.7201661 48.0342673,7.7204912 48.0342304,7.7205641 48.0343129,7.7205797 48.0343056,7.7205993 48.0342964,7.7206126 48.0342902,7.7215557 48.0338491,7.7215823 48.0338367,7.7215981 48.0338240,7.7216302 48.0337981,7.7217495 48.0338543,7.7217779 48.0338677,7.7220292 48.0339860,7.7223971 48.0341010,7.7226437 48.0342077,7.7227742 48.0342429,7.7228088 48.0342522,7.7229390 48.0342697,7.7230407 48.0342668,7.7231328 48.0342425,7.7233366 48.0341887,7.7234657 48.0342843,7.7234894 48.0343223,7.7235876 48.0344795,7.7240434 48.0351488,7.7252856 48.0350271,7.7258430 48.0356889,7.7259024 48.0357401,7.7263305 48.0357259,7.7263491 48.0357253,7.7263832 48.0357242,7.7264520 48.0357219,7.7264893 48.0358231,7.7270918 48.0357763,7.7271972 48.0357681,7.7272573 48.0357634,7.7275142 48.0363563,7.7275919 48.0364708,7.7277508 48.0366799,7.7278938 48.0368212,7.7280592 48.0369311,7.7288173 48.0374245,7.7291314 48.0373748,7.7291791 48.0373672,7.7303507 48.0382522,7.7305350 48.0382897,7.7307488 48.0382219,7.7314957 48.0378646,7.7318175 48.0384205,7.7321217 48.0383845,7.7323832 48.0383497,7.7325993 48.0383444,7.7327203 48.0383569,7.7329550 48.0384566,7.7333149 48.0386169,7.7334262 48.0386888,7.7335904 48.0387670,7.7336362 48.0383858,7.7333690 48.0377548,7.7332679 48.0374438,7.7334616 48.0372701,7.7341989 48.0366545,7.7344953 48.0364395,7.7343675 48.0360110,7.7340745 48.0344812,7.7339122 48.0339837,7.7338252 48.0337365,7.7337342 48.0335905,7.7338026 48.0329000,7.7338091 48.0328246,7.7351260 48.0320933,7.7356376 48.0317960,7.7360353 48.0313583,7.7363337 48.0311938,7.7369975 48.0308122,7.7378089 48.0308696,7.7383809 48.0304552,7.7388717 48.0301108,7.7394283 48.0297754,7.7400935 48.0299064,7.7403590 48.0289700,7.7410296 48.0283476,7.7417375 48.0280710,7.7418420 48.0276685,7.7425189 48.0275998,7.7430514 48.0273438,7.7432483 48.0270386,7.7438950 48.0269336,7.7439321 48.0266557,7.7462944 48.0263567,7.7461211 48.0258137,7.7462309 48.0257172,7.7466616 48.0256288,7.7467991 48.0256413,7.7471080 48.0255742,7.7472901 48.0254618,7.7473421 48.0254269,7.7475286 48.0253017,7.7480704 48.0249393,7.7486497 48.0245788,7.7494222 48.0241195,7.7502108 48.0237069,7.7509752 48.0232656,7.7515304 48.0229427,7.7515914 48.0228408,7.7514046 48.0214199,7.7540316 48.0202956,7.7575935 48.0190609,7.7581937 48.0188419,7.7588185 48.0199393,7.7589293 48.0202576,7.7589167 48.0210072,7.7588838 48.0213029,7.7587742 48.0216153,7.7587752 48.0217491,7.7587937 48.0218487,7.7588369 48.0219503,7.7589761 48.0222229,7.7597990 48.0218368,7.7602188 48.0216346,7.7605388 48.0214805,7.7616455 48.0212067,7.7619384 48.0211212,7.7626349 48.0209542,7.7624665 48.0207346,7.7624398 48.0206577,7.7624724 48.0205525,7.7625644 48.0204600,7.7627099 48.0203504,7.7634307 48.0199461,7.7638038 48.0198053,7.7642011 48.0196776,7.7648346 48.0195036,7.7658902 48.0193239,7.7663309 48.0192033,7.7665906 48.0191109,7.7674763 48.0185046,7.7676228 48.0183303,7.7678276 48.0179132,7.7695684 48.0170805,7.7696604 48.0159551,7.7697668 48.0151107,7.7697236 48.0149885,7.7693628 48.0143393,7.7711375 48.0157979,7.7718584 48.0163705,7.7732937 48.0173589,7.7744203 48.0182013,7.7758801 48.0191706,7.7764420 48.0195255,7.7775305 48.0201980,7.7784120 48.0207352,7.7792433 48.0212500,7.7797884 48.0216200,7.7800414 48.0217915,7.7803347 48.0220210,7.7806210 48.0222365,7.7809420 48.0225288,7.7811310 48.0227836,7.7811846 48.0230040,7.7812331 48.0233038,7.7812476 48.0236602,7.7812390 48.0241998,7.7811884 48.0244495,7.7811807 48.0244812,7.7811355 48.0246532,7.7805080 48.0251685,7.7797096 48.0255919,7.7800931 48.0259591,7.7817972 48.0254036,7.7822343 48.0253301,7.7831297 48.0253148,7.7847994 48.0252581,7.7854761 48.0252475,7.7858723 48.0252680,7.7861659 48.0253237,7.7864405 48.0254082,7.7868376 48.0255358,7.7874697 48.0257839,7.7884377 48.0262503,7.7891173 48.0266557,7.7893248 48.0267478,7.7901889 48.0272674,7.7902377 48.0272988,7.7903396 48.0273622,7.7901514 48.0276481,7.7898349 48.0281532,7.7884408 48.0299812,7.7869308 48.0320696,7.7871545 48.0323730,7.7872772 48.0326254,7.7873793 48.0327818,7.7881214 48.0327614,7.7879059 48.0332098,7.7899733 48.0360466,7.7904257 48.0366349,7.7905599 48.0373755,7.7907564 48.0383948,7.7908074 48.0385750,7.7908884 48.0388954,7.7910578 48.0395658,7.7917022 48.0420437,7.7917307 48.0422127,7.7917434 48.0422625,7.7919653 48.0424313,7.7912628 48.0429087,7.7928815 48.0436178,7.7931831 48.0437346,7.7932241 48.0437505,7.7933142 48.0437798,7.7933902 48.0437700,7.7934088 48.0438197,7.7940541 48.0447474,7.7931272 48.0450283,7.7931651 48.0450930,7.7935369 48.0456280,7.7938349 48.0461004,7.7940460 48.0465181,7.7942010 48.0469961,7.7942803 48.0475464,7.7943296 48.0479883,7.7944046 48.0483920,7.7944815 48.0486312,7.7945593 48.0488912,7.7947794 48.0496271,7.7950644 48.0513223,7.7950654 48.0514134,7.7951125 48.0515749,7.7952336 48.0522070,7.7959731 48.0524790,7.7960019 48.0525420,7.7960031 48.0525823,7.7960078 48.0525962,7.7960309 48.0526645,7.7961537 48.0530285,7.7962761 48.0534721,7.7960622 48.0535259,7.7954156 48.0535536,7.7951240 48.0535562,7.7949207 48.0535473,7.7943230 48.0535587,7.7939039 48.0535668,7.7936374 48.0535854,7.7930570 48.0536197,7.7929263 48.0536274,7.7928797 48.0536302,7.7929060 48.0536475,7.7929305 48.0536636,7.7929964 48.0537069,7.7943797 48.0546162,7.7954505 48.0553089,7.7952456 48.0554298,7.7952645 48.0554699,7.7952867 48.0555172,7.7959306 48.0568871,7.7959484 48.0569234,7.7965105 48.0580086,7.7965302 48.0580461,7.7969745 48.0589781,7.7963385 48.0589928,7.7956070 48.0590267,7.7955287 48.0590304,7.7954937 48.0590320,7.7955809 48.0591897,7.7959447 48.0599702,7.7959551 48.0599925,7.7959626 48.0600113,7.7963145 48.0608934,7.7963391 48.0609551,7.7964825 48.0609653,7.7968964 48.0609947,7.7973432 48.0609970,7.7985883 48.0609760,7.7991738 48.0609844,7.8001209 48.0610086,7.8008343 48.0610393,7.8011325 48.0610521,7.8018431 48.0610669,7.8020698 48.0610593,7.8027251 48.0610385,7.8027478 48.0610385,7.8027685 48.0610400,7.8032622 48.0610253,7.8034955 48.0610267,7.8036710 48.0610713,7.8038232 48.0611156,7.8038472 48.0611219,7.8040344 48.0611830,7.8041929 48.0612447,7.8046151 48.0613923,7.8053714 48.0616865,7.8057232 48.0618396,7.8058280 48.0619018,7.8058919 48.0620226,7.8059592 48.0621616,7.8060206 48.0623122,7.8060589 48.0624603,7.8059906 48.0626009,7.8059228 48.0627286,7.8058801 48.0628626,7.8058296 48.0629904,7.8056146 48.0632465,7.8054821 48.0634454,7.8053795 48.0636162,7.8052336 48.0637888,7.8066935 48.0648938,7.8082426 48.0660739,7.8109004 48.0680823,7.8149535 48.0710579,7.8174223 48.0696185,7.8175389 48.0695652,7.8176571 48.0695512,7.8177987 48.0695734,7.8189389 48.0699153,7.8197144 48.0701846,7.8200876 48.0703592,7.8205206 48.0702735,7.8209469 48.0701887,7.8216300 48.0700491,7.8222637 48.0699291,7.8227646 48.0697209,7.8232037 48.0695402,7.8236746 48.0693426,7.8244556 48.0690145,7.8247616 48.0688952,7.8248570 48.0686613,7.8249108 48.0685285,7.8250088 48.0683548,7.8250925 48.0682342,7.8252617 48.0680497,7.8255330 48.0676833,7.8256687 48.0675009,7.8261346 48.0669895,7.8265529 48.0665063,7.8267455 48.0662852,7.8268759 48.0661333,7.8274168 48.0655994,7.8278888 48.0652479,7.8291171 48.0642586,7.8299551 48.0636501,7.8305923 48.0631912,7.8309700 48.0629289,7.8318353 48.0623159,7.8321262 48.0620898,7.8324811 48.0618648,7.8327391 48.0615749,7.8335927 48.0604756,7.8340827 48.0595744,7.8347762 48.0586342,7.8358719 48.0579224,7.8384450 48.0562511,7.8392453 48.0557238,7.8403764 48.0553563,7.8418675 48.0548833,7.8431646 48.0544687,7.8444390 48.0535649,7.8452477 48.0529834,7.8458111 48.0525795,7.8460223 48.0524295,7.8461585 48.0523821,7.8463979 48.0523324,7.8470001 48.0522709,7.8477945 48.0521341,7.8486425 48.0518860,7.8503659 48.0513614,7.8509846 48.0507221,7.8511133 48.0505592,7.8511269 48.0505420,7.8511658 48.0505104,7.8511945 48.0504346,7.8512795 48.0502872,7.8522314 48.0483644,7.8531193 48.0465163,7.8533768 48.0456010,7.8535029 48.0451324,7.8536067 48.0449109,7.8536105 48.0448932,7.8536409 48.0448511,7.8538252 48.0446378,7.8541172 48.0439209,7.8541272 48.0433709,7.8542182 48.0424520,7.8542902 48.0421701,7.8543552 48.0420661,7.8543771 48.0419021,7.8544770 48.0401963,7.8543097 48.0398657,7.8543042 48.0376716,7.8543422 48.0371686,7.8544157 48.0366459,7.8554464 48.0368843,7.8559213 48.0367582,7.8562848 48.0366895,7.8571448 48.0364598,7.8571887 48.0362292,7.8585439 48.0355870,7.8587146 48.0354606,7.8588093 48.0354197,7.8594833 48.0352829,7.8599963 48.0351245,7.8601681 48.0350641,7.8603777 48.0349919,7.8605146 48.0350043,7.8607107 48.0350220,7.8607878 48.0350289,7.8608485 48.0350332,7.8609444 48.0350430,7.8610125 48.0350544,7.8610932 48.0350771,7.8611490 48.0350899,7.8612990 48.0351243,7.8616627 48.0352117,7.8624568 48.0352211,7.8633473 48.0351396,7.8635089 48.0351258,7.8636464 48.0351095,7.8637746 48.0350886,7.8638784 48.0350678,7.8639021 48.0350664,7.8639722 48.0350510,7.8643686 48.0349611,7.8647131 48.0348773,7.8651536 48.0347362,7.8656567 48.0345486,7.8662248 48.0343754,7.8663782 48.0343237,7.8666812 48.0341983,7.8668931 48.0341091,7.8669865 48.0340708,7.8672132 48.0339173,7.8673625 48.0337213,7.8675052 48.0336134,7.8676982 48.0334572,7.8679367 48.0332640,7.8680783 48.0332006,7.8681658 48.0331614,7.8684240 48.0330476,7.8690277 48.0327823,7.8694227 48.0326088,7.8697138 48.0324786,7.8700846 48.0322653,7.8703041 48.0321310,7.8705670 48.0317831,7.8708688 48.0311803,7.8708982 48.0311216,7.8707125 48.0310967,7.8706252 48.0310636,7.8708083 48.0308062,7.8709212 48.0306358,7.8710327 48.0298805,7.8718007 48.0297255,7.8720823 48.0297443,7.8722435 48.0298096,7.8725696 48.0298141,7.8728829 48.0296136,7.8730607 48.0295859,7.8732828 48.0300315,7.8734419 48.0301957,7.8737845 48.0304303,7.8740199 48.0305923,7.8747307 48.0309932,7.8754690 48.0313410,7.8757590 48.0311802,7.8761341 48.0306100,7.8767043 48.0297776,7.8777879 48.0284447,7.8780699 48.0279597,7.8784449 48.0270028,7.8785209 48.0268648,7.8796037 48.0260449,7.8801916 48.0254800,7.8807446 48.0249500,7.8811765 48.0240152,7.8812568 48.0239113,7.8813255 48.0237412,7.8814051 48.0236203,7.8815334 48.0234542,7.8816248 48.0233550,7.8816847 48.0232408,7.8818131 48.0230554,7.8820604 48.0227417,7.8822507 48.0225003,7.8823550 48.0224685,7.8828751 48.0224245,7.8831279 48.0224192,7.8836678 48.0221917,7.8839095 48.0222447,7.8842893 48.0223948,7.8844839 48.0226220,7.8846139 48.0226988,7.8848134 48.0228166,7.8849856 48.0229183,7.8859902 48.0226601,7.8863575 48.0221936,7.8866605 48.0216511,7.8867752 48.0215598,7.8874631 48.0210951,7.8880158 48.0208098,7.8887906 48.0203146,7.8896365 48.0201286,7.8902507 48.0197499,7.8909172 48.0192850,7.8923590 48.0192847,7.8930262 48.0192980,7.8934076 48.0192790,7.8939376 48.0190653,7.8942547 48.0188571,7.8949466 48.0184918,7.8955572 48.0181448,7.8963597 48.0175458,7.8970521 48.0168950,7.8971693 48.0167100,7.8978254 48.0157464,7.8979965 48.0155423,7.8978323 48.0154771,7.8975741 48.0153737,7.8970579 48.0154430,7.8964478 48.0154904,7.8957055 48.0157981,7.8950516 48.0162710,7.8946787 48.0164470,7.8941197 48.0165840,7.8930419 48.0166570,7.8921110 48.0168820,7.8895464 48.0165270,7.8894026 48.0164963,7.8890244 48.0165736,7.8885059 48.0168507,7.8884610 48.0172257,7.8882603 48.0173431,7.8877746 48.0176339,7.8872687 48.0180338,7.8865608 48.0184188,7.8863828 48.0185678,7.8862048 48.0188767,7.8863508 48.0193077,7.8863218 48.0195937,7.8859478 48.0200176,7.8852299 48.0205825,7.8848380 48.0207695,7.8844748 48.0212708,7.8843701 48.0213905,7.8830902 48.0207305,7.8813265 48.0199946,7.8804916 48.0195447,7.8798738 48.0189385,7.8791538 48.0193427,7.8790608 48.0195297,7.8786129 48.0197656,7.8784629 48.0199756,7.8782219 48.0200136,7.8780349 48.0201126,7.8777930 48.0201626,7.8778110 48.0202996,7.8774010 48.0205745,7.8770461 48.0206085,7.8767111 48.0208075,7.8760592 48.0210555,7.8756503 48.0210935,7.8744196 48.0203838,7.8738216 48.0203007,7.8731184 48.0198714,7.8728516 48.0196162,7.8725312 48.0190924,7.8723612 48.0175079,7.8723570 48.0171823,7.8724520 48.0166434,7.8724974 48.0165185,7.8730340 48.0159154,7.8740435 48.0152096,7.8746767 48.0151091,7.8759770 48.0142142,7.8761228 48.0141218,7.8786262 48.0132849,7.8793476 48.0129506,7.8809474 48.0125670,7.8821146 48.0123632,7.8829661 48.0120056,7.8841671 48.0117665,7.8843529 48.0117051,7.8848220 48.0112856,7.8861117 48.0107640,7.8865578 48.0106216,7.8869977 48.0104928,7.8874013 48.0103476,7.8876870 48.0102390,7.8896068 48.0101470,7.8911836 48.0102715,7.8955534 48.0101547,7.8971395 48.0101215,7.8989272 48.0100955,7.9005097 48.0100381,7.9016968 48.0100052,7.9021029 48.0104517,7.9031557 48.0111856,7.9035071 48.0115301,7.9036310 48.0119181,7.9040195 48.0123736,7.9049506 48.0128079,7.9058128 48.0128301,7.9062381 48.0129269,7.9065139 48.0129593,7.9072153 48.0132462,7.9077651 48.0131860,7.9082110 48.0131405,7.9094675 48.0128951,7.9102049 48.0129012,7.9103812 48.0129671,7.9107275 48.0130194,7.9114833 48.0127315,7.9118687 48.0125034,7.9123689 48.0122419,7.9129417 48.0118684,7.9141979 48.0121314,7.9142559 48.0120716,7.9146795 48.0123959,7.9149648 48.0126984,7.9156298 48.0129605,7.9165997 48.0136884,7.9166953 48.0137294,7.9177289 48.0140487,7.9191179 48.0145534,7.9200029 48.0149592,7.9218818 48.0156302,7.9224028 48.0152983,7.9229804 48.0150843,7.9234433 48.0147568,7.9238793 48.0143433,7.9245452 48.0140620,7.9254626 48.0138989,7.9262681 48.0138124,7.9266402 48.0131285,7.9269862 48.0128385,7.9273341 48.0124225,7.9277892 48.0111624,7.9282009 48.0108503,7.9283880 48.0088179,7.9286270 48.0085470,7.9286290 48.0082340,7.9278471 48.0059753,7.9280271 48.0048504,7.9281511 48.0026036,7.9277671 48.0019967,7.9272102 48.0013448,7.9271692 48.0011008,7.9272802 48.0006659,7.9275212 48.0001929,7.9279131 47.9997160,7.9280231 47.9994140,7.9279981 47.9986481,7.9293269 47.9975412,7.9291169 47.9971902,7.9290590 47.9967133,7.9296679 47.9962254,7.9301968 47.9953895,7.9302008 47.9950695,7.9299498 47.9944476,7.9299608 47.9936356,7.9305943 47.9924659,7.9306688 47.9922658,7.9305216 47.9918892,7.9305468 47.9916859,7.9308444 47.9911327,7.9308031 47.9909891,7.9303904 47.9905671,7.9303046 47.9901009,7.9301891 47.9898486,7.9292846 47.9886151,7.9276558 47.9892333,7.9275290 47.9886749,7.9271566 47.9881842,7.9268695 47.9877378,7.9268514 47.9876895,7.9268748 47.9876669,7.9268856 47.9876094,7.9268188 47.9875079,7.9266880 47.9873363,7.9266852 47.9872641,7.9266788 47.9872474,7.9268284 47.9870992,7.9274964 47.9866525,7.9278276 47.9863837,7.9280604 47.9861354,7.9283062 47.9858339,7.9277143 47.9857000,7.9276399 47.9857340,7.9251883 47.9855278,7.9252308 47.9852655,7.9252361 47.9851543,7.9253174 47.9830222,7.9252640 47.9825114,7.9252121 47.9819305,7.9252174 47.9816550,7.9250688 47.9811734,7.9248921 47.9808717,7.9244099 47.9808371,7.9159867 47.9795350,7.9149476 47.9794137,7.9139845 47.9794096,7.9143523 47.9790793,7.9147049 47.9787257,7.9148705 47.9784687,7.9149182 47.9783946,7.9151825 47.9779895,7.9153752 47.9778125,7.9158008 47.9775643,7.9160614 47.9773951,7.9161999 47.9771671,7.9163274 47.9768586,7.9164189 47.9767817,7.9166653 47.9765749,7.9169295 47.9763944,7.9173251 47.9762361,7.9175453 47.9761787,7.9177504 47.9760154,7.9178046 47.9758729,7.9178175 47.9757311,7.9176580 47.9755321,7.9175085 47.9753953,7.9173987 47.9752603,7.9173781 47.9751475,7.9173886 47.9750350,7.9174558 47.9748769,7.9176117 47.9747341,7.9178923 47.9746191,7.9181470 47.9745876,7.9184598 47.9746496,7.9186946 47.9746770,7.9194118 47.9745049,7.9195653 47.9744833,7.9198041 47.9744802,7.9202646 47.9745062,7.9204892 47.9744973,7.9206626 47.9744573,7.9209013 47.9740858,7.9212697 47.9738148,7.9213768 47.9737008,7.9214323 47.9736655,7.9215947 47.9735430,7.9216748 47.9734823,7.9216477 47.9734402,7.9212308 47.9727922,7.9212014 47.9727449,7.9211323 47.9725905,7.9210974 47.9725651,7.9215891 47.9723622,7.9226848 47.9719235,7.9233228 47.9716323,7.9239281 47.9712566,7.9245159 47.9707574,7.9243968 47.9706077,7.9243809 47.9705894,7.9239173 47.9701193,7.9239003 47.9701073,7.9236947 47.9698436,7.9225770 47.9696673,7.9219781 47.9683095,7.9217911 47.9674356,7.9196994 47.9673366,7.9179776 47.9676225,7.9178336 47.9672566,7.9169868 47.9665846,7.9165099 47.9662638,7.9163822 47.9661780,7.9159209 47.9658677,7.9154910 47.9654678,7.9152370 47.9650208,7.9151957 47.9644441,7.9151220 47.9634150,7.9147311 47.9629421,7.9153360 47.9623201,7.9156820 47.9616942,7.9158939 47.9606953,7.9160779 47.9602264,7.9168008 47.9593985,7.9171448 47.9591385,7.9172228 47.9589865,7.9170128 47.9585895,7.9171588 47.9580516,7.9190962 47.9572751,7.9187416 47.9567048,7.9189485 47.9559958,7.9189375 47.9557249,7.9193461 47.9549629,7.9198004 47.9542140,7.9207343 47.9534431,7.9213012 47.9530542,7.9220201 47.9526882,7.9226090 47.9522073,7.9233080 47.9518183,7.9237309 47.9513644,7.9247428 47.9510404,7.9242948 47.9505135,7.9241378 47.9501855,7.9243628 47.9501135,7.9241758 47.9495256,7.9241658 47.9492396,7.9243538 47.9485757,7.9243528 47.9479727,7.9242498 47.9476948,7.9242638 47.9472328,7.9245838 47.9464659,7.9247548 47.9458180,7.9249168 47.9450241,7.9249198 47.9447537,7.9249218 47.9445741,7.9246508 47.9437922,7.9241849 47.9414765,7.9241979 47.9412445,7.9243439 47.9408395,7.9248698 47.9400916,7.9241689 47.9393097,7.9238361 47.9387906,7.9232305 47.9381958,7.9225744 47.9374236,7.9224871 47.9373271,7.9224213 47.9372447,7.9221218 47.9369554,7.9219941 47.9368352,7.9200155 47.9356741,7.9197145 47.9355871,7.9188106 47.9355221,7.9186177 47.9354301,7.9185127 47.9352852,7.9181897 47.9351822,7.9172649 47.9350792,7.9169239 47.9347862,7.9157981 47.9335493,7.9149402 47.9316695,7.9146442 47.9312766,7.9141103 47.9308376,7.9125955 47.9297618,7.9117996 47.9296438,7.9111567 47.9293498,7.9088571 47.9290488,7.9082221 47.9282209,7.9079873 47.9276439,7.9068764 47.9249133,7.9067394 47.9245543,7.9067004 47.9242534,7.9077792 47.9223806,7.9087521 47.9203628,7.9100939 47.9182760,7.9099790 47.9171202,7.9097630 47.9171462,7.9095044 47.9154484,7.9084115 47.9143185,7.9048773 47.9134895,7.9046091 47.9130757,7.9035342 47.9126238,7.9028028 47.9123252,7.9017541 47.9121457,7.9007443 47.9120117,7.9000910 47.9119195,7.8984616 47.9119437,7.8982200 47.9116269,7.8979939 47.9113389,7.8969333 47.9107769,7.8963818 47.9106576,7.8948086 47.9103248,7.8936337 47.9101734,7.8928046 47.9099329,7.8924961 47.9082794,7.8924038 47.9077995,7.8921347 47.9073682,7.8915956 47.9068373,7.8908057 47.9062599,7.8899433 47.9058614,7.8891982 47.9055808,7.8890178 47.9054813,7.8887435 47.9053743,7.8883297 47.9051954,7.8881145 47.9051167,7.8874882 47.9048686,7.8869577 47.9046922,7.8860965 47.9044744,7.8848724 47.9040957,7.8844610 47.9039563,7.8839838 47.9040855,7.8832336 47.9042174,7.8826877 47.9042739,7.8824324 47.9042805,7.8821640 47.9042851,7.8812903 47.9041983,7.8803837 47.9040941,7.8797802 47.9040185,7.8792478 47.9039852,7.8785688 47.9039762,7.8776385 47.9038594,7.8773139 47.9038073,7.8750256 47.9036068,7.8749760 47.9035777,7.8733161 47.9039476,7.8715013 47.9045125,7.8706523 47.9057086,7.8704685 47.9059393,7.8697356 47.9075942,7.8695616 47.9078231,7.8692476 47.9080941,7.8683543 47.9087863,7.8679629 47.9092743,7.8679228 47.9094291,7.8678043 47.9097706,7.8676295 47.9100340,7.8675029 47.9102612,7.8674150 47.9104226,7.8673009 47.9105266,7.8671828 47.9106650,7.8670333 47.9108198,7.8670106 47.9109475,7.8669364 47.9110998,7.8668651 47.9112841,7.8668045 47.9114366,7.8664664 47.9115424,7.8663327 47.9115834,7.8662792 47.9117176,7.8661260 47.9119369,7.8659690 47.9121547,7.8657139 47.9123512,7.8650822 47.9126860,7.8645236 47.9129347,7.8642964 47.9130880,7.8640681 47.9131204,7.8636489 47.9131784,7.8630391 47.9132532,7.8629567 47.9134808,7.8628895 47.9138067,7.8628788 47.9140189,7.8628375 47.9142264,7.8627297 47.9143601,7.8625702 47.9145301,7.8624963 47.9146618,7.8624741 47.9149471,7.8624505 47.9152961,7.8623626 47.9156470,7.8630142 47.9158831,7.8636182 47.9161234,7.8642149 47.9163956,7.8648183 47.9166655,7.8644643 47.9168794,7.8640715 47.9171592,7.8636300 47.9175071,7.8634134 47.9178226,7.8633757 47.9180255,7.8634300 47.9182109,7.8634551 47.9185056,7.8634891 47.9186908,7.8634620 47.9188664,7.8633700 47.9190665,7.8632908 47.9192965,7.8631438 47.9195304,7.8628883 47.9197474,7.8627216 47.9199354,7.8626706 47.9201177,7.8627011 47.9203029,7.8626776 47.9204716,7.8625758 47.9206442,7.8624812 47.9208010,7.8625126 47.9209337,7.8625879 47.9210873,7.8626457 47.9212727,7.8626927 47.9214900,7.8627309 47.9218169,7.8628403 47.9221488,7.8629870 47.9224902,7.8629800 47.9226819,7.8628822 47.9228204,7.8627467 47.9229836,7.8625872 47.9231489,7.8625329 47.9233288,7.8625194 47.9235068,7.8624641 47.9237323,7.8624962 47.9238377,7.8625973 47.9238910,7.8627635 47.9239062,7.8629877 47.9239172,7.8632230 47.9238827,7.8633936 47.9238476,7.8636121 47.9237969,7.8638575 47.9237693,7.8641338 47.9237123,7.8644185 47.9237649,7.8645829 47.9238827,7.8647015 47.9240892,7.8647190 47.9243973,7.8647313 47.9245598,7.8647705 47.9245417,7.8654129 47.9242984,7.8655886 47.9241834,7.8658811 47.9241722,7.8665284 47.9239287,7.8668149 47.9237026,7.8673236 47.9235881,7.8675194 47.9234869,7.8677717 47.9234479,7.8680764 47.9238816,7.8679452 47.9243129,7.8672537 47.9249079,7.8663037 47.9255167,7.8660580 47.9260858,7.8662392 47.9269005,7.8657744 47.9277305,7.8657804 47.9282925,7.8658889 47.9284784,7.8660624 47.9286444,7.8664777 47.9287963,7.8674282 47.9285142,7.8677117 47.9293230,7.8684606 47.9296217,7.8681692 47.9302681,7.8687050 47.9308642,7.8679125 47.9310540,7.8676980 47.9314200,7.8677205 47.9321878,7.8679709 47.9329483,7.8686218 47.9337396,7.8688843 47.9338858,7.8701526 47.9335263,7.8699878 47.9330931,7.8699891 47.9330689,7.8702035 47.9328926,7.8704615 47.9327396,7.8708196 47.9325119,7.8711946 47.9322914,7.8714862 47.9321638,7.8717634 47.9320680,7.8719677 47.9320583,7.8721300 47.9321054,7.8723220 47.9321985,7.8725100 47.9322851,7.8738036 47.9311769,7.8748619 47.9312429,7.8748843 47.9312893,7.8749684 47.9313806,7.8750040 47.9314164,7.8750428 47.9314591,7.8755632 47.9320125,7.8757287 47.9326498,7.8757499 47.9327175,7.8757688 47.9327918,7.8759487 47.9334817,7.8761124 47.9337209,7.8763341 47.9340842,7.8765512 47.9343583,7.8766906 47.9346225,7.8768945 47.9348850,7.8771044 47.9351796,7.8771931 47.9352971,7.8773704 47.9359097,7.8774012 47.9360356,7.8774325 47.9361410,7.8774595 47.9362805,7.8774701 47.9364447,7.8774693 47.9364885,7.8774656 47.9365406,7.8775069 47.9368152,7.8775205 47.9369455,7.8775672 47.9371173,7.8775866 47.9372841,7.8775648 47.9374711,7.8775393 47.9376671,7.8775097 47.9377604,7.8774490 47.9378761,7.8774167 47.9379396,7.8772240 47.9382222,7.8772087 47.9382697,7.8772231 47.9383147,7.8772749 47.9384435,7.8772983 47.9384918,7.8773249 47.9385390,7.8774010 47.9386881,7.8775836 47.9392555,7.8779804 47.9400586,7.8769569 47.9400675,7.8763978 47.9402111,7.8764804 47.9402835,7.8764813 47.9403395,7.8765864 47.9404823,7.8767342 47.9406826,7.8768825 47.9408646,7.8770011 47.9410120,7.8771564 47.9412844,7.8772099 47.9414490,7.8772366 47.9416022,7.8772995 47.9417970,7.8773957 47.9420172,7.8775036 47.9421851,7.8776360 47.9423190,7.8778247 47.9425152,7.8779543 47.9426286,7.8781151 47.9427195,7.8782971 47.9427855,7.8784982 47.9428928,7.8786911 47.9430593,7.8788366 47.9432163,7.8789206 47.9433792,7.8788876 47.9435311,7.8787887 47.9436806,7.8787242 47.9438077,7.8786625 47.9439554,7.8786249 47.9440987,7.8786006 47.9442491,7.8785931 47.9444111,7.8785487 47.9445498,7.8784810 47.9447616,7.8784534 47.9448069,7.8784248 47.9448537,7.8783361 47.9450177,7.8782844 47.9451746,7.8782380 47.9453932,7.8781840 47.9456437,7.8783755 47.9458505,7.8784864 47.9459835,7.8786306 47.9461668,7.8787786 47.9463320,7.8788518 47.9464739,7.8789415 47.9466866,7.8790346 47.9468880,7.8791945 47.9472062,7.8792842 47.9474053,7.8793414 47.9475983,7.8792226 47.9477613,7.8791122 47.9479229,7.8789727 47.9482489,7.8788515 47.9484607,7.8787792 47.9485495,7.8786256 47.9486082,7.8781989 47.9487501,7.8777690 47.9489012,7.8775713 47.9489505,7.8773569 47.9489655,7.8768364 47.9489837,7.8761390 47.9490034,7.8758295 47.9490248,7.8756044 47.9490741,7.8754197 47.9491828,7.8750809 47.9493777,7.8746776 47.9495791,7.8743492 47.9497649,7.8740882 47.9499042,7.8734267 47.9502539,7.8729883 47.9504703,7.8711603 47.9512900,7.8686582 47.9510248,7.8687284 47.9508745,7.8688739 47.9506972,7.8691420 47.9505117,7.8694719 47.9502762,7.8693530 47.9498186,7.8667081 47.9497899,7.8665355 47.9501885,7.8661893 47.9501520,7.8657622 47.9501546,7.8656586 47.9501518,7.8656079 47.9501527,7.8655826 47.9501517,7.8654398 47.9501459,7.8646733 47.9501147,7.8640612 47.9501198,7.8635140 47.9500270,7.8634023 47.9500767,7.8630020 47.9500078,7.8624462 47.9498922,7.8620024 47.9497820,7.8613962 47.9496250,7.8609220 47.9495008,7.8603843 47.9493077,7.8599810 47.9492296,7.8596988 47.9492209,7.8593970 47.9491664,7.8587676 47.9489750,7.8583195 47.9489240,7.8580021 47.9489995,7.8575029 47.9491881,7.8567494 47.9495441,7.8562631 47.9497784,7.8559195 47.9500091,7.8556810 47.9500933,7.8556776 47.9502565,7.8555596 47.9504202,7.8554163 47.9506705,7.8552589 47.9509596,7.8550953 47.9512074,7.8548979 47.9514414,7.8545361 47.9519006,7.8543187 47.9521632,7.8539934 47.9525479,7.8539845 47.9526825,7.8539950 47.9528882,7.8540542 47.9532380,7.8540259 47.9535280,7.8539479 47.9539614,7.8539651 47.9541717,7.8541546 47.9546778,7.8541361 47.9547713,7.8540536 47.9548347,7.8539059 47.9549228,7.8536655 47.9550834,7.8533397 47.9552617,7.8529564 47.9554236,7.8521316 47.9557631,7.8509979 47.9562923,7.8506073 47.9564793,7.8504128 47.9565259,7.8501328 47.9565995,7.8494948 47.9567322,7.8492961 47.9568268,7.8492296 47.9569452,7.8490900 47.9571818,7.8488190 47.9577762,7.8486427 47.9581771,7.8485396 47.9584687,7.8485987 47.9588346,7.8487368 47.9595711,7.8489411 47.9604358,7.8488979 47.9608125,7.8487446 47.9612773,7.8486063 47.9616373,7.8485308 47.9616848,7.8484281 47.9617230,7.8482464 47.9618131,7.8477122 47.9620585,7.8473458 47.9622137,7.8471231 47.9623172,7.8469480 47.9624257,7.8440738 47.9637483,7.8433652 47.9639242,7.8429697 47.9641021,7.8412396 47.9640638,7.8394037 47.9638697,7.8374339 47.9656699,7.8358706 47.9662518,7.8346282 47.9666029,7.8356643 47.9681244,7.8366566 47.9694196,7.8374267 47.9711616,7.8373161 47.9713118,7.8373045 47.9713280,7.8372782 47.9713700,7.8365278 47.9725686,7.8353271 47.9724190,7.8335122 47.9724650,7.8329252 47.9725788,7.8328070 47.9725979,7.8327839 47.9725996,7.8318418 47.9726052,7.8316988 47.9726084,7.8306187 47.9727189,7.8300959 47.9727624,7.8300145 47.9727803,7.8299752 47.9727856,7.8286519 47.9729133,7.8284741 47.9729338,7.8284577 47.9729356,7.8283686 47.9729608,7.8282577 47.9729886,7.8279156 47.9731000,7.8275250 47.9732172,7.8273224 47.9729975,7.8270519 47.9726753,7.8269310 47.9725313,7.8267142 47.9722826,7.8265342 47.9721603,7.8263582 47.9724352,7.8254674 47.9727822,7.8253195 47.9722158,7.8250411 47.9723001,7.8249579 47.9723422,7.8249175 47.9724145,7.8248028 47.9724611,7.8243419 47.9725123,7.8236072 47.9727220,7.8233026 47.9727033,7.8231273 47.9727468,7.8224918 47.9723653,7.8225528 47.9722439,7.8225020 47.9722237,7.8223418 47.9721599,7.8218779 47.9718791,7.8212869 47.9715103,7.8210185 47.9712934,7.8205125 47.9708464,7.8202125 47.9705953,7.8200736 47.9704840,7.8186950 47.9701837,7.8179364 47.9686022,7.8165842 47.9676604,7.8133954 47.9677254,7.8130712 47.9677293,7.8120513 47.9671344,7.8115034 47.9669054,7.8105972 47.9664905,7.8092057 47.9659785,7.8090438 47.9659107,7.8076340 47.9657005,7.8073831 47.9655631,7.8070286 47.9654027,7.8067577 47.9652797,7.8065905 47.9650953,7.8062526 47.9647654,7.8061335 47.9646253,7.8052680 47.9644568,7.8044804 47.9643607,7.8021357 47.9640765,7.8011659 47.9643497,7.8004371 47.9645513,7.7992973 47.9652724,7.7975770 47.9656425,7.7956299 47.9662675,7.7921661 47.9683632,7.7906040 47.9688368,7.7887501 47.9688177,7.7880604 47.9690069,7.7877411 47.9690862,7.7874126 47.9691066,7.7872847 47.9691639,7.7870520 47.9692643,7.7869726 47.9692707,7.7868028 47.9692817,7.7867658 47.9692657,7.7866847 47.9692391,7.7865040 47.9691762,7.7864630 47.9691634,7.7859688 47.9695141,7.7856967 47.9693130,7.7856583 47.9693374,7.7851711 47.9696456,7.7849833 47.9694416,7.7849270 47.9694784,7.7844377 47.9698095,7.7844403 47.9698580,7.7844077 47.9698912,7.7842856 47.9699493,7.7839198 47.9697414,7.7835857 47.9699300,7.7834811 47.9698583,7.7834206 47.9698885,7.7829049 47.9701416,7.7827848 47.9700540,7.7821013 47.9704311,7.7820000 47.9704227,7.7819833 47.9704219,7.7816451 47.9703939,7.7813068 47.9703688,7.7812617 47.9703667,7.7811814 47.9703559,7.7811336 47.9703508,7.7806256 47.9705629,7.7806106 47.9705726,7.7803875 47.9706633,7.7803276 47.9706855,7.7802949 47.9707120,7.7802604 47.9707280,7.7799635 47.9709360,7.7800547 47.9709855,7.7783070 47.9720288,7.7782493 47.9721202,7.7782786 47.9722306,7.7783068 47.9723369,7.7769135 47.9727199,7.7764101 47.9728453,7.7760990 47.9729228,7.7759130 47.9729828,7.7758840 47.9729880,7.7759793 47.9732369,7.7761470 47.9736937,7.7760786 47.9737168,7.7750532 47.9740641,7.7743057 47.9743172,7.7722987 47.9749969,7.7726552 47.9754488,7.7725879 47.9754685,7.7725058 47.9754868,7.7723968 47.9755147,7.7723714 47.9755251,7.7722926 47.9755485,7.7722620 47.9755620,7.7722478 47.9755683,7.7721618 47.9756064,7.7720782 47.9756607,7.7718351 47.9758365,7.7710539 47.9765796,7.7709999 47.9766301,7.7699126 47.9776210,7.7680771 47.9784134,7.7680036 47.9784432,7.7679460 47.9784665,7.7679391 47.9784619,7.7673258 47.9780763,7.7657209 47.9770208,7.7633721 47.9768122,7.7624212 47.9770412,7.7600885 47.9776511,7.7576469 47.9783760,7.7563940 47.9787160,7.7559641 47.9788150,7.7551190 47.9788009,7.7567772 47.9805245,7.7559120 47.9804332,7.7542355 47.9801573,7.7524518 47.9798073,7.7500465 47.9793570,7.7494325 47.9792140,7.7482194 47.9788969,7.7470062 47.9787431,7.7456581 47.9786920,7.7444508 47.9788897,7.7434958 47.9791607,7.7432370 47.9792402,7.7423119 47.9795648,7.7421457 47.9796284,7.7416225 47.9796717,7.7408258 47.9797448,7.7408382 47.9798773,7.7403929 47.9799120,7.7399539 47.9799024,7.7394344 47.9799439,7.7392659 47.9800561,7.7385748 47.9800929,7.7379971 47.9801005,7.7369790 47.9801206,7.7367796 47.9801153,7.7362980 47.9800773,7.7360845 47.9799409,7.7357512 47.9796052,7.7353230 47.9793258,7.7349800 47.9792241,7.7343700 47.9790659,7.7337784 47.9790363,7.7334672 47.9790104,7.7332827 47.9789679,7.7330600 47.9789164,7.7328428 47.9788113,7.7325373 47.9786349,7.7316675 47.9780144,7.7313340 47.9777735,7.7312308 47.9778487,7.7312043 47.9778680,7.7309468 47.9780556,7.7307792 47.9781388,7.7291861 47.9770584,7.7288694 47.9771955,7.7284952 47.9774459,7.7276290 47.9781221,7.7275218 47.9779833,7.7274989 47.9779585,7.7272160 47.9776029,7.7271482 47.9776331,7.7266987 47.9771617,7.7265843 47.9772099,7.7255530 47.9763814,7.7255286 47.9763617,7.7251833 47.9760843,7.7244146 47.9755761,7.7244015 47.9755817,7.7243243 47.9756149,7.7243054 47.9756213,7.7239496 47.9757689,7.7237542 47.9757220,7.7235597 47.9757069,7.7240177 47.9753873,7.7240609 47.9753571,7.7242306 47.9752387,7.7251325 47.9746173,7.7250938 47.9745940,7.7250290 47.9745551,7.7240411 47.9739661,7.7239786 47.9739240,7.7238149 47.9738257,7.7236677 47.9737372,7.7235930 47.9736923,7.7234903 47.9736306,7.7234764 47.9736223,7.7230080 47.9733425,7.7229807 47.9733262,7.7218112 47.9726275,7.7218638 47.9725635,7.7218833 47.9725397,7.7219733 47.9724266,7.7220055 47.9723885,7.7220452 47.9723417,7.7220710 47.9723112,7.7221678 47.9721927,7.7222708 47.9720857,7.7223480 47.9720050,7.7223734 47.9719784,7.7220637 47.9717463,7.7217095 47.9714902,7.7214191 47.9712967,7.7211722 47.9711448,7.7208901 47.9709629,7.7206556 47.9708480,7.7202097 47.9706044,7.7198130 47.9702940,7.7197089 47.9700806,7.7195982 47.9699297,7.7191484 47.9693400,7.7186934 47.9687751,7.7172745 47.9690494,7.7171341 47.9689319,7.7168015 47.9686822,7.7154511 47.9679551,7.7134807 47.9668201,7.7122211 47.9662210,7.7104648 47.9653697,7.7101772 47.9651975,7.7099343 47.9650521,7.7089992 47.9645169,7.7075369 47.9636951,7.7056708 47.9628125,7.7051275 47.9625871,7.7048310 47.9625130,7.7047962 47.9625043,7.7047279 47.9624730,7.7037277 47.9620022,7.6986534 47.9596870,7.6981390 47.9599970,7.6980231 47.9600727,7.6979922 47.9600941,7.6978727 47.9601638,7.6975911 47.9603264,7.6971338 47.9607269,7.6966617 47.9610941,7.6965316 47.9612011,7.6962876 47.9613968,7.6956814 47.9618512,7.6954601 47.9619706,7.6952250 47.9620599,7.6948901 47.9621664,7.6946204 47.9622469,7.6938635 47.9624527,7.6936483 47.9624996,7.6924466 47.9628229,7.6922693 47.9628777,7.6909706 47.9631985,7.6907167 47.9632581,7.6906240 47.9632787,7.6901528 47.9633982,7.6894186 47.9635416,7.6893739 47.9634717,7.6887997 47.9625739,7.6877967 47.9610878,7.6866893 47.9614219,7.6866406 47.9614366,7.6847087 47.9620195,7.6838112 47.9622861,7.6836960 47.9622840,7.6836785 47.9622837,7.6836583 47.9622834,7.6834197 47.9622723,7.6814085 47.9615190,7.6813628 47.9615025,7.6813455 47.9614948,7.6795107 47.9608818,7.6791831 47.9605728,7.6784086 47.9599489,7.6776233 47.9592875,7.6775611 47.9592406,7.6761416 47.9603412,7.6756373 47.9607244,7.6755355 47.9607969,7.6754934 47.9608212,7.6755967 47.9608845,7.6755606 47.9609088,7.6738809 47.9620399,7.6735798 47.9622420,7.6733386 47.9624684,7.6728711 47.9630022,7.6726293 47.9632594,7.6713767 47.9642489,7.6701325 47.9642971,7.6700798 47.9642991,7.6700758 47.9642993,7.6680750 47.9624566,7.6656356 47.9643822,7.6651264 47.9648003,7.6650179 47.9649034,7.6647473 47.9650810,7.6620055 47.9656508))");

    TEST(geo::intersects(a, b));
    TEST(geo::intersects(b, a));
    TEST(!geo::contains(a, b));

    XSortedLine<double> ax(a);
    XSortedPolygon<double> bx(b);

    TEST(geo::intersectsContains(ax, bx).first);
    TEST(!geo::intersectsContains(ax, bx).second);

    Line<int> api;
    for (const auto& p : a) {
      auto pp = latLngToWebMerc(p);
      api.push_back({pp.getX() * 10, pp.getY() * 10});
    }

    Polygon<int> bpi;
    for (const auto& p : b.getOuter()) {
      auto pp = latLngToWebMerc(p);
      bpi.getOuter().push_back({pp.getX() * 10, pp.getY() * 10});
    }
    XSortedLine<int> axi(api);
    XSortedPolygon<int> bxi(bpi);

    TEST(geo::intersectsContains(axi, bxi).first);
    TEST(!geo::intersectsContains(axi, bxi).second);

    XSortedLine<int> axi2(LineSegment<int>{api[0], api[1]});

    TEST(geo::intersectsContains(axi2, bxi).first);
    TEST(!geo::intersectsContains(axi2, bxi).second);
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
