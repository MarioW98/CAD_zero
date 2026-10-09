// tests/unit/viewport/test_selection.cpp
//
// Tests for the SelectionModel class.
//
#include <doctest/doctest.h>

#include "CAD_0/viewport/selection.hpp"
#include "CAD_0/geometry/shape_id.hpp"

using namespace CAD_0::viewport;
using namespace CAD_0::geometry;

namespace {

// Helper: build a ShapeId from a feature id.
ShapeId make_id(std::uint64_t feature_value) {
    ShapeId id;
    id.feature = FeatureId{feature_value};
    id.generation = Generation{1};
    return id;
}

} // namespace

TEST_CASE("SelectionModel: default is empty") {
    SelectionModel sel;
    CHECK(sel.empty());
    CHECK(sel.size() == 0);
    CHECK_FALSE(sel.primary().has_value());
}

TEST_CASE("SelectionModel: Replace mode replaces selection") {
    SelectionModel sel;
    sel.select(make_id(1));
    CHECK(sel.size() == 1);
    CHECK(sel.contains(make_id(1)));

    sel.select(make_id(2), SelectionMode::Replace);
    CHECK(sel.size() == 1);
    CHECK(sel.contains(make_id(2)));
    CHECK_FALSE(sel.contains(make_id(1)));
}

TEST_CASE("SelectionModel: Add mode accumulates") {
    SelectionModel sel;
    sel.select(make_id(1));
    sel.select(make_id(2), SelectionMode::Add);
    sel.select(make_id(3), SelectionMode::Add);
    CHECK(sel.size() == 3);
    CHECK(sel.contains(make_id(1)));
    CHECK(sel.contains(make_id(2)));
    CHECK(sel.contains(make_id(3)));
}

TEST_CASE("SelectionModel: Add mode doesn't duplicate") {
    SelectionModel sel;
    sel.select(make_id(1));
    sel.select(make_id(1), SelectionMode::Add);
    CHECK(sel.size() == 1);
}

TEST_CASE("SelectionModel: Subtract mode removes") {
    SelectionModel sel;
    sel.select(make_id(1));
    sel.select(make_id(2), SelectionMode::Add);
    CHECK(sel.size() == 2);

    sel.select(make_id(1), SelectionMode::Subtract);
    CHECK(sel.size() == 1);
    CHECK_FALSE(sel.contains(make_id(1)));
    CHECK(sel.contains(make_id(2)));
}

TEST_CASE("SelectionModel: Subtract mode on missing entry is a no-op") {
    SelectionModel sel;
    sel.select(make_id(1));
    sel.select(make_id(99), SelectionMode::Subtract);
    CHECK(sel.size() == 1);
    CHECK(sel.contains(make_id(1)));
}

TEST_CASE("SelectionModel: Toggle mode adds when absent") {
    SelectionModel sel;
    sel.select(make_id(1), SelectionMode::Toggle);
    CHECK(sel.size() == 1);
    CHECK(sel.contains(make_id(1)));
}

TEST_CASE("SelectionModel: Toggle mode removes when present") {
    SelectionModel sel;
    sel.select(make_id(1));
    sel.select(make_id(1), SelectionMode::Toggle);
    CHECK(sel.empty());
    CHECK_FALSE(sel.contains(make_id(1)));
}

TEST_CASE("SelectionModel: clear empties the selection") {
    SelectionModel sel;
    sel.select(make_id(1));
    sel.select(make_id(2), SelectionMode::Add);
    CHECK(sel.size() == 2);

    sel.clear();
    CHECK(sel.empty());
    CHECK(sel.size() == 0);
}

TEST_CASE("SelectionModel: primary returns the single entry") {
    SelectionModel sel;
    sel.select(make_id(42));
    const auto p = sel.primary();
    REQUIRE(p.has_value());
    CHECK(p->shape.feature.value == 42);
}

TEST_CASE("SelectionModel: primary returns nullopt for multi-selection") {
    SelectionModel sel;
    sel.select(make_id(1));
    sel.select(make_id(2), SelectionMode::Add);
    CHECK_FALSE(sel.primary().has_value());
}

TEST_CASE("SelectionModel: primary returns nullopt when empty") {
    SelectionModel sel;
    CHECK_FALSE(sel.primary().has_value());
}

TEST_CASE("SelectionModel: select_many replaces") {
    SelectionModel sel;
    sel.select(make_id(1));
    sel.select_many({make_id(10), make_id(20), make_id(30)});
    CHECK(sel.size() == 3);
    CHECK(sel.contains(make_id(10)));
    CHECK(sel.contains(make_id(20)));
    CHECK(sel.contains(make_id(30)));
    CHECK_FALSE(sel.contains(make_id(1)));
}

TEST_CASE("SelectionModel: select_many deduplicates") {
    SelectionModel sel;
    std::vector<ShapeId> ids = {make_id(1), make_id(1), make_id(2), make_id(2)};
    sel.select_many(ids);
    CHECK(sel.size() == 2);
}

TEST_CASE("SelectionModel: hover set and clear") {
    SelectionModel sel;
    CHECK_FALSE(sel.hover().has_value());

    sel.set_hover(make_id(5));
    const auto h = sel.hover();
    REQUIRE(h.has_value());
    CHECK(h->shape.feature.value == 5);

    sel.clear_hover();
    CHECK_FALSE(sel.hover().has_value());
}

TEST_CASE("SelectionModel: hover independent of selection") {
    SelectionModel sel;
    sel.select(make_id(1));
    sel.set_hover(make_id(2));

    // Hover doesn't affect selection.
    CHECK(sel.size() == 1);
    CHECK(sel.contains(make_id(1)));
    CHECK(sel.hover().has_value());
    CHECK(sel.hover()->shape.feature.value == 2);

    // Selection doesn't affect hover.
    sel.clear();
    CHECK(sel.hover().has_value());
    CHECK(sel.hover()->shape.feature.value == 2);
}
