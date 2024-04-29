// Copyright 2023, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Johannes Kalmbach <kalmbach@informatik.uni-freiburg.de>

#include "WktParser.h"

#include <iostream>
#include <string_view>
#include <optional>
#include <variant>

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

constexpr auto fs(const auto &s) {
    return ctll::fixed_string{s};
}

/// Helper function for ctre: concatenation of fixed_strings
template<size_t A, size_t B>
constexpr ctll::fixed_string<A + B> operator+(const ctll::fixed_string<A> &a,
                                              const ctll::fixed_string<B> &b) {
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
template<size_t A, size_t B>
constexpr ctll::fixed_string<A + B - 1> operator+(
        const ctll::fixed_string<A> &a, const char (&b)[B]) {
    auto bStr = ctll::fixed_string(b);
    return a + bStr;
}

/// Helper function for ctre: concatenation of fixed_strings
template<size_t A, size_t B>
constexpr ctll::fixed_string<A + B - 1> operator+(
        const char (&a)[A], const ctll::fixed_string<B> &b) {
    auto aStr = ctll::fixed_string(a);
    return aStr + b;
}

/// Create a regex group by putting the argument in parentheses
template<size_t N>
static constexpr auto grp(const ctll::fixed_string<N> &s) {
    return ctll::fixed_string("(") + s + ctll::fixed_string(")");
}

template<size_t N, size_t M>
static constexpr auto grp(const ctll::fixed_string<N> &s, const ctll::fixed_string<M> &groupName) {
    return grp(ctll::fixed_string("?<") + groupName + ctll::fixed_string(">") + s);
}

/// Create a regex character class by putting the argument in square brackets
template<size_t N>
static constexpr auto cls(const ctll::fixed_string<N> &s) {
    return ctll::fixed_string("[") + s + ctll::fixed_string("]");
}

template<size_t N>
static constexpr auto paren(const ctll::fixed_string<N> &s) {
    return ctll::fixed_string("\\(") + s + ctll::fixed_string("\\)");
}

template<size_t N>
static constexpr auto optWsAround(const ctll::fixed_string<N> &s) {
    return ctll::fixed_string("\\s*") + s + ctll::fixed_string("\\s*");
}

template<size_t N>
static constexpr auto star(const ctll::fixed_string<N> &s) {
    return s + fs("*");
}


static constexpr ctll::fixed_string floatRegex = "[0-9]+(\\.[0-9]+)?";
static constexpr ctll::fixed_string coordinateRegex =
        grp(floatRegex, fs("n1")) + fs("\\s+") + grp(floatRegex, fs("n2"));
static constexpr ctll::fixed_string innerParens = paren(star(cls(fs("^)"))));

I32Point matchPointCoordinates(std::string_view input) {
    auto match = ctre::search<coordinateRegex>(input);
    if (!match) {
        throw std::runtime_error("invalid point: " + std::string{input});
    }
    float f1;
    float f2;
    const auto &m1 = match.get<"n1">().to_view();
    const auto &m2 = match.get<"n2">().to_view();
    absl::from_chars(m1.data(), m1.data() + m1.size(), f1);
    absl::from_chars(m2.data(), m2.data() + m2.size(), f2);
    auto point = latLngToWebMerc(DPoint(
            f1, f2));
    return {point.getX() * PREC, point.getY() * PREC};
}

std::optional<I32Point> matchPoint(std::string_view input) {
    static constexpr ctll::fixed_string pointPrefix = fs(R"(\s*POINT\s*\()");
    auto match = ctre::match<pointPrefix>(input);
    if (!match) {
        return std::nullopt;
    }
    return matchPointCoordinates(input);
}

I32Line parseCoordinateList(std::string_view input) {
    I32Line line;
    for (const auto &m: ctre::range<coordinateRegex>(input)) {
        // TODO<joka921> We currently match the regex twice.
        line.push_back(matchPointCoordinates(m.to_view()));
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
    size_t position = 0;
    while (true) {
        position = input.find('(', position + 1);
        if (position == std::string_view::npos) {
            break;
        }
        while (true) {
            auto close = input.find_first_of("()", position + 1);
            if (close == std::string_view::npos) {
                throw std::runtime_error("unmatched parentheses");
            }
            if (input[close] == '(') {
                position = close;
                continue;
            }
            lines.push_back(parseCoordinateList(std::string_view{input.data() + position, input.data() + close}));
            break;
        }
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
    if (const auto &m = ctre::starts_with<R"(\s*MULTIPOLYGON\s*\()">(input); !m) {
        return std::nullopt;
    } else {
        input.remove_prefix(m.to_view().size());
    }

    static constexpr auto polyStructure = paren(innerParens + grp(optWsAround(fs(",")) + innerParens) + fs("*"));

    std::vector<I32Polygon> polygons;
    for (const auto &m: ctre::range<polyStructure>(input)) {
        polygons.push_back(matchPolygonInner(m.to_view()));
    }
    return I32MultiPolygon{std::move(polygons)};
}


using ElementVariant = std::variant<I32Point, I32Polygon, I32MultiPolygon, I32Line>;


// _____________________________________________________________________________
ElementVariant parseElement(std::string_view line
) {
    if (auto p = matchPoint(line)) {
        return std::move(p.value());
    } else if (auto l = matchLinestring(line)) {
        return std::move(l.value());
    } else if (auto poly = matchPolygon(line)) {
        return std::move(poly.value());
    } else if (auto mpoly = matchMultipolygon(line)) {
        return std::move(mpoly.value());
    } else {
        std::cerr << "Couldn't parse line of size " << line.size() << std::endl;
        std::cerr << "Couldn't parse element \"" << line.substr(0, 40) << std::endl;
        throw std::runtime_error("Illegal element found, aborting");
    }
}

// _____________________________________________________________________________
std::vector<std::pair<ElementVariant, std::string>> parseNew(std::string_view &input, size_t *gid
                                                             ) {

    std::vector<std::pair<ElementVariant, std::string>> result;
    while (true) {
        auto idp = input.find('\t');
        auto newline = input.find('\n');

        if (newline == std::string_view::npos) {
            return result;
        }

        auto line = input.substr(0, newline);
        input.remove_prefix(newline + 1);


        std::string id;
        if (idp != std::string_view::npos) {
            id = line.substr(0, idp);
            line.remove_prefix(idp + 1);
        } else {
            id = std::to_string(*gid);
        }

        result.emplace_back(parseElement(line), std::move(id));
        ++(*gid);
    }
}

void parseStdin(Sweeper &idx) {
    std::vector<char> buffer;
    buffer.resize(100'000'000);
    size_t offset = 0;
    // The lines by default were 1-based...
    size_t gid = 1;
    while (true) {
        size_t toRead = buffer.size() - offset;
        size_t actualOffset = offset;
        while (true) {
            auto numRead = read(0, buffer.data() + actualOffset, toRead);
            toRead -= numRead;
            actualOffset += numRead;
            if (toRead == 0) {
                // read a complete block, break;
                break;
            }
            //std::cerr << "to Read" << toRead << "num Read " << numRead << std::endl;
            if (numRead == 0) {
                if (offset + numRead == 0) {
                    return;
                }
                if (buffer.at(offset + numRead - 1) != '\n') {
                    buffer.push_back('\n');
                    ++numRead;
                }
                std::string_view sv{buffer.data(), offset + numRead};
                //std::cerr << "parsing " << sv.size() << "bytes " << std::endl;
                auto results = parseNew(sv, &gid );
                for (auto& [element, id] : results) {
                    std::visit([&idx, &id](auto &x) {
                        idx.add(x, std::move(id));
                    }, element);
                }
                return;
            }
        }
        std::string_view sv{buffer.data(), buffer.size()};
        //std::cerr << "parsing " << sv.size() << "bytes(intermediate)" << std::endl;
        auto results = parseNew(sv, &gid );
        for (auto& [element, id] : results) {
            std::visit([&idx, &id](auto &x) {
                idx.add(x, std::move(id));
            }, element);
        }
        auto beg = sv.data() - buffer.data();
        auto end = beg + sv.size();
        offset = sv.size();
        std::shift_left(buffer.data(), buffer.data() + end, beg);
    }
}