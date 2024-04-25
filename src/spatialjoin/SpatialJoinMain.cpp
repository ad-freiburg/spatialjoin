// Copyright 2023, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#include <iostream>
#include <string_view>
#include <optional>

#include <absl/strings/charconv.h>

#include "BoxIds.h"
#include "Sweeper.h"
#include "util/Misc.h"
#include "util/geo/Geo.h"
#include "util/http/Server.h"
#include "util/log/Log.h"
#include "ctre.hpp"

using sj::Sweeper;
using util::geo::DLine;
using util::geo::DPoint;
using util::geo::I32Line;
using util::geo::I32MultiPolygon;
using util::geo::I32Point;
using util::geo::I32Polygon;

static const char *YEAR = &__DATE__[7];
static const char *COPY =
        "University of Freiburg - Chair of Algorithms and Data Structures";
static const char *AUTHORS = "Patrick Brosi <brosi@informatik.uni-freiburg.de>";

constexpr auto fs(const auto& s) {
    return ctll::fixed_string{s};
}
/// Helper function for ctre: concatenation of fixed_strings
template <size_t A, size_t B>
constexpr ctll::fixed_string<A + B> operator+(const ctll::fixed_string<A>& a,
                                              const ctll::fixed_string<B>& b) {
    char32_t comb[A + B + 1] = {};
    for (size_t i = 0; i < A; ++i) {  // omit the trailing 0 of the first string
        comb[i] = a.content[i];
    }
    for (size_t i = 0; i < B; ++i) {
        comb[i + A] = b.content[i];
    }
    // the input array must be zero-terminated
    comb[A + B] = '\0';
    return ctll::fixed_string(comb);
}

/// Helper function for ctre: concatenation of fixed_strings
template <size_t A, size_t B>
constexpr ctll::fixed_string<A + B - 1> operator+(
        const ctll::fixed_string<A>& a, const char (&b)[B]) {
    auto bStr = ctll::fixed_string(b);
    return a + bStr;
}

/// Helper function for ctre: concatenation of fixed_strings
template <size_t A, size_t B>
constexpr ctll::fixed_string<A + B - 1> operator+(
        const char (&a)[A], const ctll::fixed_string<B>& b) {
    auto aStr = ctll::fixed_string(a);
    return aStr + b;
}

/// Create a regex group by putting the argument in parentheses
template <size_t N>
static constexpr auto grp(const ctll::fixed_string<N>& s) {
    return ctll::fixed_string("(") + s + ctll::fixed_string(")");
}
template <size_t N, size_t M>
static constexpr auto grp(const ctll::fixed_string<N>& s, const ctll::fixed_string<M>& groupName) {
    return grp(ctll::fixed_string("?<") + groupName + ctll::fixed_string(">") + s);
}

/// Create a regex character class by putting the argument in square brackets
template <size_t N>
static constexpr auto cls(const ctll::fixed_string<N>& s) {
    return ctll::fixed_string("[") + s + ctll::fixed_string("]");
}

template <size_t N>
static constexpr auto paren(const ctll::fixed_string<N>& s) {
    return ctll::fixed_string("\\(") + s + ctll::fixed_string("\\)");
}

template <size_t N>
static constexpr auto optWsAround(const ctll::fixed_string<N>& s) {
    return ctll::fixed_string("\\s*") + s + ctll::fixed_string("\\s*");
}

template <size_t N>
static constexpr auto star(const ctll::fixed_string<N>& s) {
    return s + fs("*");
}
// _____________________________________________________________________________
void printHelp(int argc, char **argv) {
    UNUSED(argc);
    std::cout << "\n"
              << "(C) 2023-" << YEAR << " " << COPY << "\n"
              << "Authors: " << AUTHORS << "\n\n"
              << "Usage: " << argv[0] << " [--help] [-h] <input>\n\n"
              << "Allowed options:\n\n"
              << std::setfill(' ') << std::left << "General:\n"
              << std::setw(41) << "  -h [ --help ]"
              << "show this help message\n"
              << std::setw(41) << "  -o [ --output ] (default: '')"
              << "output file, empty (default) prints to stdout\n"
              << std::setw(41) << "  -c [ --cache ] (default: '.')"
              << "cache directory for intermediate files\n"
              << std::setw(41) << "  -C"
              << "don't parse input, re-use intermediate cache files\n"
              << std::setw(41) << "  --prefix (default: '')"
              << "prefix added at the beginning of every relation\n"
              << std::setw(41) << "  --intersects (default: ' intersects ')"
              << "separator between intersecting geometry IDs\n"
              << std::setw(41) << "  --contains (default: ' contains ')"
              << "separator between containing geometry IDs\n"
              << std::setw(41) << "  --suffix (default: '\\n')"
              << "suffix added at the beginning of every relation\n\n"
              << std::setfill(' ') << std::left << "Geometric computation:\n"
              << std::setw(41) << "  --no-box-ids"
              << "disable box id criteria for contains/intersect computation\n"
              << std::setw(41) << "  --no-surface-area"
              << "disable surface area criteria for polygon contains\n"
              << std::endl;
}

// _____________________________________________________________________________
util::geo::I32Line parseLineString(const std::string &a, size_t p) {
    util::geo::I32Line line;
    line.reserve(2);
    auto end = memchr(a.c_str() + p, ')', a.size() - p);
    assert(end);

    while (true) {
        while (*(a.c_str() + p) && isspace(*(a.c_str() + p))) p++;
        double x = util::atof(a.c_str() + p, 10);
        double y = util::atof(
                static_cast<const char *>(memchr(a.c_str() + p, ' ', a.size() - p)) + 1,
                10);
        auto projPoint = latLngToWebMerc(DPoint(x, y));

        line.push_back(I32Point{projPoint.getX() * PREC, projPoint.getY() * PREC});

        auto n = memchr(a.c_str() + p, ',', a.size() - p);
        if (!n || n > end) break;
        p = static_cast<const char *>(n) - a.c_str() + 1;
    }

    return util::geo::simplify(line, 0);
}

// _____________________________________________________________________________
util::geo::I32Point parsePoint(const std::string &a, size_t p) {
    auto point = latLngToWebMerc(DPoint(
            util::atof(a.c_str() + p, 10),
            util::atof(
                    static_cast<const char *>(memchr(a.c_str() + p, ' ', a.size() - p)) +
                    1,
                    10)));

    return {point.getX() * PREC, point.getY() * PREC};
}

static constexpr ctll::fixed_string floatRegex = "[0-9]+(\\.[0-9]+)?";
static constexpr ctll::fixed_string coordinateRegex = grp(floatRegex, fs("n1")) + fs("\\s+") + grp(floatRegex, fs("n2"));
static constexpr ctll::fixed_string innerParens = paren(star(cls(fs("^)"))));

std::optional<I32Point> matchPoint(std::string_view input) {
    static constexpr ctll::fixed_string pointRegex = fs("\\s*POINT\\s*\\(") + coordinateRegex + fs("\\)");
    static_assert(ctre::match<pointRegex>("POINT (3.102 30)"));
    auto match = ctre::match<pointRegex>(input);
    if (!match) {
        return std::nullopt;
    }
    float f1;
    float f2;
    const auto &m1 = match.get<"n1">().to_view();
    const auto &m2 = match.get<"n2">().to_view();
    absl::from_chars(m1.data(), m1.data() + m1.size(), f1);
    absl::from_chars(m2.data(), m2.data() + m2.size(), f2);
    auto point = latLngToWebMerc(DPoint(
            f1, f2));
    return I32Point{point.getX() * PREC, point.getY() * PREC};
}

I32Line parseCoordinateList(std::string_view input) {
    I32Line line;
    for (const auto &m: ctre::range<coordinateRegex>(input)) {
        float f1;
        float f2;
        const auto &m1 = m.get<"n1">().to_view();
        const auto &m2 = m.get<"n2">().to_view();
        absl::from_chars(m1.data(), m1.data() + m1.size(), f1);
        absl::from_chars(m2.data(), m2.data() + m2.size(), f2);
        auto projPoint = latLngToWebMerc(DPoint(f1, f2));
        line.push_back(I32Point{projPoint.getX() * PREC, projPoint.getY() * PREC});
    }
    return line;
}

std::optional<I32Line> matchLinestring(std::string_view input) {
    static_assert(ctre::starts_with<"\\s*LINESTRING">("LINESTRING(.asdfj)"));
    if (!ctre::starts_with<"\\s*LINESTRING">(input)) {
        return std::nullopt;
    }

    // TODO<joka921> We could also remove the beginning first
    return parseCoordinateList(input);
}

I32Polygon matchPolygonInner(std::string_view input) {
    std::vector<I32Line> lines;
    for (const auto &m: ctre::range<"\\([^(]*\\)">(input)) {
        lines.push_back(parseCoordinateList(m.to_view()));
    }
    I32Polygon result;
    result.getOuter() = std::move(lines.at(0));
    result.getInners().reserve(lines.size() - 1);
    std::move(lines.begin() + 1, lines.end(), std::back_inserter(result.getInners()));
    return result;
}

std::optional<I32Polygon> matchPolygon(std::string_view input) {
    if (!ctre::starts_with<"\\s*POLYGON">(input)) {
        return std::nullopt;
    }

    return matchPolygonInner(input);
}

std::optional<I32MultiPolygon> matchMultipolygon(std::string_view input) {

    if (const auto& m = ctre::starts_with<"\\s*MULTIPOLYGON\\s*\\(">(input); !m) {
        return std::nullopt;
    } else {
        input.remove_prefix(m.to_view().size());
    }

    static constexpr auto polyStructure = paren(innerParens + grp(optWsAround(fs(",")) + innerParens) + fs("*"));

    static_assert(ctre::match<innerParens>("( asdif )"));
    static_assert(ctre::match<"[^(]">("y"));

    std::vector<I32Polygon> polygons;
    for (const auto &m: ctre::range<polyStructure>(input)) {
        polygons.push_back(matchPolygonInner(m.to_view()));
    }
    return I32MultiPolygon{std::move(polygons)};
}

// _____________________________________________________________________________
void parse(const char *c, size_t size, std::string &dangling, size_t *gid,
           Sweeper &idx) {
    const char *start = c;
    while (c < start + size) {
        if (*c == '\n') {
            (*gid)++;

            auto idp = dangling.find("\t");

            std::string id = std::to_string(*gid);

            size_t start = 2;

            if (idp != std::string::npos) {
                id = dangling.substr(0, idp);
                start = idp + 2;
            }

            if (auto p = matchPoint(dangling)) {

            }
            auto p = dangling.rfind("POINT(", start);

            if (p != std::string::npos) {
                p += 6;
                auto point = parsePoint(dangling, p);
                idx.add(point, id);
            } else if ((p = dangling.rfind("LINESTRING(", start)) !=

                       std::string::npos) {
                p += 11;
                const auto &line = parseLineString(dangling, p);
                if (line.size() != 0) {
                    idx.add(line, id);
                }
            } else if ((p = dangling.rfind("MULTILINESTRING(", start)) !=
                       std::string::npos) {
                p += 16;
                size_t i = 0;
                while ((p = dangling.find("(", p + 1)) != std::string::npos) {
                    const auto &line = parseLineString(dangling, p + 1);
                    if (line.size() != 0) {
                        // TODO, i is the line number
                    }
                    i++;
                }
            } else if ((p = dangling.rfind("POLYGON(", start)) != std::string::npos) {
                p += 7;
                size_t i = 0;
                I32Polygon poly;
                while ((p = dangling.find("(", p + 1)) != std::string::npos) {
                    const auto &line = parseLineString(dangling, p + 1);
                    if (i == 0) {
                        // outer
                        poly.getOuter() = line;
                    } else {
                        poly.getInners().push_back(line);
                    }
                    i++;
                }
                idx.add(poly, id);
            } else if ((p = dangling.rfind("MULTIPOLYGON(", start)) !=
                       std::string::npos) {
                p += 12;
                I32MultiPolygon mp;
                while (p != std::string::npos &&
                       (p = dangling.find("(", p + 1)) != std::string::npos) {
                    I32Polygon poly;
                    size_t i = 0;
                    while ((p = dangling.find("(", p + 1)) != std::string::npos) {
                        const auto &line = parseLineString(dangling, p + 1);
                        if (i == 0) {
                            // outer
                            poly.getOuter() = line;
                        } else {
                            poly.getInners().push_back(line);
                        }

                        // check if multipolygon is closed
                        auto q = dangling.find(
                                ")", p + 1);  // this is the closing of the linestring
                        auto q2 = dangling.find(")", q + 1);
                        auto q3 = dangling.find(",", q + 1);
                        if (q2 != std::string::npos && q3 != std::string::npos && q2 < q3) {
                            p = q3;
                            break;
                        }

                        i++;
                    }
                    mp.push_back(poly);
                }
                idx.add(mp, id);
            }

            dangling.clear();
            c++;
            continue;
        }

        dangling += *c;

        c++;
    }
}

// _____________________________________________________________________________
void parseElement(std::string_view line, std::string gid,
                  Sweeper &idx) {
    if (auto p = matchPoint(line)) {
        idx.add(p.value(), std::move(gid));
        return;
    } else if (auto l = matchLinestring(line)) {
        idx.add(l.value(), std::move(gid));
    } else if (auto poly = matchPolygon(line)) {
        idx.add(poly.value(), std::move(gid));
    } else if (auto mpoly = matchMultipolygon(line)) {
        idx.add(mpoly.value(), std::move(gid));
    } else {
        throw std::runtime_error("Couldn't parse element \"" + std::string{line});
    }
}

// _____________________________________________________________________________
void parseNew(std::string_view &input, size_t *gid,
              Sweeper &idx) {
    while (true) {
        auto idp = input.find('\t');
        auto newline = input.find('\n');

        if (newline == std::string_view::npos) {
            return;
            // TODO<joka921> Fix the rest.
        }

        auto line = input.substr(0, newline);
        input.remove_prefix(newline + 1);


        std::string id;
        if (idp != std::string::npos) {
            id = line.substr(0, idp);
            line.remove_prefix(idp + 1);
        } else {
            id = std::to_string(*gid);
        }

        parseElement(line, std::move(id), idx);
        ++(*gid);
    }
}

void parseStdin(Sweeper& idx) {
    std::vector<char> buffer;
    buffer.resize(100'000'000);
    size_t offset = 0;
    // The lines by default were 1-based...
    size_t gid = 1;
    while (true) {
        size_t toRead = buffer.size() - offset;
        auto numRead = read(0, buffer.data() + offset, toRead);
        if (numRead < toRead) {
            if (offset + numRead == 0) {
                return;
            }
            if (buffer.at(offset + numRead - 1) != '\n') {
                buffer.push_back('\n');
                ++numRead;
            }
            std::string_view sv {buffer.data(), offset + numRead};
            parseNew(sv, &gid, idx);
            return;
        }
        assert(offset + numRead == buffer.size());
        std::string_view sv {buffer.data(), offset + numRead};
        parseNew(sv, &gid, idx);
        auto beg = sv.data() - buffer.data();
        auto end = beg + sv.size();
        offset = sv.size();
        std::shift_left(buffer.data() + beg, buffer.data() + end, beg);
    }
}


// _____________________________________________________________________________
int main(int argc, char **argv) {
    // disable output buffering for standard output
    setbuf(stdout, NULL);

    // initialize randomness
    srand(time(NULL) + rand());  // NOLINT

    bool useCache = false;

    int state = 0;

    std::string prefix = "";
    std::string output = "";
    std::string cache = ".";
    std::string contains = " contains ";
    std::string intersects = " intersects ";
    std::string suffix = "\n";

    bool useBoxIds = true;
    bool useArea = true;

    for (int i = 1; i < argc; i++) {
        std::string cur = argv[i];
        switch (state) {
            case 0:
                if (cur == "-h" || cur == "--help") {
                    printHelp(argc, argv);
                    exit(0);
                }
                if (cur == "-C") {
                    useCache = true;
                }
                if (cur == "--prefix") {
                    state = 1;
                }
                if (cur == "--contains") {
                    state = 2;
                }
                if (cur == "--intersects") {
                    state = 3;
                }
                if (cur == "--suffix") {
                    state = 4;
                }
                if (cur == "--output" || cur == "-o") {
                    state = 5;
                }
                if (cur == "--cache" || cur == "-c") {
                    state = 6;
                }
                if (cur == "--no-box-ids") {
                    useBoxIds = false;
                }
                if (cur == "--no-surface-area") {
                    useArea = false;
                }
                break;
            case 1:
                prefix = cur;
                state = 0;
                break;
            case 2:
                contains = cur;
                state = 0;
                break;
            case 3:
                intersects = cur;
                state = 0;
                break;
            case 4:
                suffix = cur;
                state = 0;
                break;
            case 5:
                output = cur;
                state = 0;
                break;
            case 6:
                cache = cur;
                state = 0;
                break;
        }
    }

    char *buf = new char[1024 * 1024 * 100];

    size_t len;

    std::string dangling;

    size_t NUM_THREADS = std::thread::hardware_concurrency();

    Sweeper sweeper({NUM_THREADS, prefix, intersects, contains, suffix, useBoxIds, useArea}, useCache,
                    cache, output);

    size_t gid = 0;

    if (!useCache) {
        LOGTO(INFO, std::cerr) << "Parsing input geometries...";

        parseStdin(sweeper);

        /*
        while ((len = read(0, buf, 1024 * 1024 * 100)) > 0) {
            parse(buf, len, dangling, &gid, sweeper);
        }
         */

        sweeper.flush();
    }

    LOGTO(INFO, std::cerr) << "done.";

    LOGTO(INFO, std::cerr) << "Sweeping...";
    sweeper.sweep();
    LOGTO(INFO, std::cerr) << "done.";
}
