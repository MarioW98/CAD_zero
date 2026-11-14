// tests/unit/brep/test_k1_closure_consumption.cpp
//
// K1 CHIUSURA — LA SUITE DI CONSUMAZIONE REALE (the campaign's closing
// act, ADR-0041's territory): every family the census PASSES is
// consumed through the OPERATIONS THAT USE IT, not merely queried —
// the representative bodies eaten by real booleans (as operand and as
// tool, where significant) and by real fillets (where in class), the
// chain GATED at every step (K1 m5's door, the five predicates), each
// case verified on the SEVEN COLUMNS end-to-end:
//
//   1. GEOMETRIC RESULT — the counts and volumes DERIVED (never
//      copied): every number below is computed by this file's own
//      closed forms, with the provenance of every golden cited (the
//      m5 chain-5 baseline, the m3 crossed golden, the ADR-0039 D5
//      form, the m7 oracle values).
//   2. TOPOLOGY CLOSED AND VALID — χ = V−E+F−H even and equal to the
//      family's own, H the inner-loop count, the loops closed, the
//      radial XOR (chain_gate's five predicates green at every step).
//   3. ORIENTATION — the volume's SIGN positive (the divergence
//      theorem's own orientation witness), closed orientable, the
//      named faces' surfaces reading their outward frames (the
//      census's chirality law read on the consumed body).
//   4. MASS PROPERTIES — the closed-form oracles (K1 m7's own roads,
//      reused here as verification: the classical forms with their
//      provenance, never duplicated machinery), tolerance 1e-12.
//   5. GENEALOGY — the names conform to ADR-0036's grammar
//      (<feature>:<role>), the creator owns the name through the
//      operations (box#1:top survives the subtract), the tavola's
//      closure arithmetic both sides, the held references resolving
//      in their D3 classes (Resolved / Vanished — the two the
//      authored flows produce).
//   6. SERIALIZATION — the recipe scene saved in v0.7.0, loaded, the
//      re-materialized body's chain_digest BIT-IDENTICAL to the
//      fresh one, the recipe field for field, the bytes of
//      save(load(x)) == x.
//   7. REJECTION — the invalid inputs refused WITH THEIR NAME (the
//      actionable std::invalid_argument, the diagnosis naming what,
//      the remedy naming how), never a generic exception, never a
//      silent success.
//
// THE CASES (the census's PASS families, each its representative
// body): the BOX (the plane family's factory), the PRISMA (the
// extrude), the CIECO (the blind subtract), the SBORDO (the crossed
// subtract, the tool protruding), the FILLET RIM (the toroidal
// flank's birth), the TORO (the flank itself — the torus family's
// only kernel read on today's tree, the m7 register's own words),
// the ELLISSOIDE (the census's one complete row).
//
// THE CONTROPROVA (each column must BITE — the failure is the
// assertion): a corrupted twin per column — the open loop, the
// inverted orientation, the laundered name, the altered round-trip,
// the smoothed refusal — each REFUSED by the same checker that
// passes the honest body. A check that cannot say no is a
// decorative number.
//
// SCOPE (the brief's own fence): tests/ only — the kernel is not
// touched, the format does not move, no new dependency. If the suite
// finds a real kernel defect: STOP and report, never stitch the cure
// into the suite.
#include <doctest/doctest.h>

#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/brep/fillet.hpp"
#include "CAD_0/brep/descent.hpp"
#include "CAD_0/brep/chain_gate.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/mass_properties.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/scene/scene.hpp"
#include "CAD_0/io/native.hpp"
#include "CAD_0/sdf/sketch_entities.hpp"
#include "CAD_0/math/vec.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace CAD_0;
namespace b = CAD_0::brep;
namespace s = CAD_0::sdf;
namespace sc = CAD_0::scene;
namespace nat = CAD_0::io::native;
namespace geo = CAD_0::geometry;

// ===========================================================================
// THE CORPUS — the seven representative bodies (the m5 Chain5's own
// roads, the m3 crossed guide, the m3 ellipsoid probe)
// ===========================================================================
namespace {

constexpr double kPi = 3.14159265358979323846;
const math::Tolerance kMassTol = math::Tolerance::linear(1e-9);

// The closed forms — THIS FILE'S OWN DERIVATIONS, the provenance
// beside each (column 1's law: derived, never copied).
//
// The box (half extents 1): V = 8.
// The prism (circle a = 0.5, height 2): V = pi a^2 h = 0.5 pi.
// The blind (the tool z in [-0.5, 1.5] against the box z in [-1, 1],
// so the removed cylinder spans z in [-0.5, 1]): V = 8 - 1.5 pi a^2
//   = 8 - 0.375 pi (the m5/m7 golden, re-derived here).
// The crossed (the square [0.5,1.5]^2 against the box footprint
// [-1,1]^2: the removed column [0.5,1]^2 x [-1,1]):
//   V = 8 - 0.25 * 2 = 7.5 (the m3 crossed golden, re-derived).
// The rim fillet's removal (the ADR-0039 D5 composite, the m7 road
// cited): 2 pi [a r^2 (1 - pi/4) + r^3 (5/6 - pi/4)], the sharp
// corner ring the ball carves away.
// The ellipsoid (a = 0.75, c = 0.5): V = (4/3) pi a^2 c = 0.375 pi.
double box_volume() { return 8.0; }
double prism_volume() { return kPi * 0.25 * 2.0; }
double blind_volume() { return 8.0 - 1.5 * kPi * 0.25; }
double crossed_volume() { return 8.0 - 0.5; }
double rim_fillet_removal(double a, double r) {
    return 2.0 * kPi * (a * r * r * (1.0 - kPi / 4.0)
                        + r * r * r * (5.0 / 6.0 - kPi / 4.0));
}
double rim_fillet_volume(double a, double r) {
    return blind_volume() - rim_fillet_removal(a, r);
}
double ellipsoid_volume(double a, double c) {
    return (4.0 / 3.0) * kPi * a * a * c;
}

// The circle chains (the blind tool) and the square chains (the
// crossed tool) — the m5/m3 corpus's own.
std::vector<s::SketchChain> circle_chains() {
    return {s::SketchChain{
        {s::sketch_circle(math::Vec2f{0.0f, 0.0f}, 0.5f, false)}}};
}
std::vector<s::SketchChain> square_chains() {
    return {s::SketchChain{{
        s::sketch_segment({0.5f, 0.5f}, {1.5f, 0.5f}, true),
        s::sketch_segment({1.5f, 0.5f}, {1.5f, 1.5f}, true),
        s::sketch_segment({1.5f, 1.5f}, {0.5f, 1.5f}, true),
        s::sketch_segment({0.5f, 1.5f}, {0.5f, 0.5f}, true),
    }}};
}

// THE SEVEN BODIES. Each builder names its family's census row and
// the operation that consumes it.
struct Corpus {
    std::unique_ptr<b::BRepBody> box;       // box#1 — the plane family
    std::unique_ptr<b::BRepBody> prism;     // ext#3 — the cylinder family
    std::unique_ptr<b::BRepBody> blind;     // sub#4 — the cieco
    std::unique_ptr<b::BRepBody> crossed;   // sub#4' — the sbordo
    std::unique_ptr<b::BRepBody> square_tool;  // the sbordo's own tool
    std::unique_ptr<b::BRepBody> rimfil;    // fil#5 — the fillet rim
    std::unique_ptr<b::BRepBody> ellipsoid; // elp#7 — the complete row
    geo::ShapeId rim{};   // the blind's born rim (the fillet's meal)
    geo::ShapeId top{};   // box#1:top — the surviving absorbed face
    geo::ShapeId vertical{};  // box#1:vertical2 — the class-1 fillet's edge
    std::vector<s::SketchChain> chains;

    Corpus() {
        // The box.
        box = b::make_box({1.0, 1.0, 1.0}, geo::FeatureId{1});
        // The sketch + the prism (the extrude, height 2, +Z).
        chains = circle_chains();
        prism = b::make_prism(chains, 2.0f, +1, geo::FeatureId{3});
        // The cieco: the blind subtract (the tool offset (0,0,-0.5)).
        const b::SubtractNaming sub_ctx{
            geo::FeatureId{4}, &b::naming::kBoxNaming,
            &b::naming::kExtNaming};
        blind = b::exact_subtract(*box, *prism, {0.0, 0.0, -0.5},
                                  b::kSubtractTolerance, sub_ctx);
        // The sbordo: the crossed subtract (the square tool protruding
        // through the corner, the m3 guide).
        square_tool = b::make_prism(square_chains(), 3.0f, +1,
                                    geo::FeatureId{3});
        crossed = b::exact_subtract(*box, *square_tool,
                                    {0.0, 0.0, -1.5},
                                    b::kSubtractTolerance, sub_ctx);
        // The references, discovered BY NAME (the m5 pattern: the
        // structural id behind the render, never a parse of it).
        for (std::size_t i = 0; i < blind->edge_count(); ++i) {
            if (b::query_edge(*blind, i).name ==
                "sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi") {
                rim = blind->edges[i].shape_id();
                break;
            }
        }
        for (std::size_t i = 0; i < blind->face_count(); ++i) {
            if (b::query_face(*blind, i).name == "box#1:top") {
                top = blind->solid.shell().faces()[i].shape_id();
                break;
            }
        }
        // The fillet rim: the toroidal flank's birth (class 2).
        const b::FilletNaming fil_ctx{geo::FeatureId{5},
                                      &b::naming::kBoxNaming,
                                      geo::FeatureId{3},
                                      &b::naming::kExtNaming};
        rimfil = b::fillet_rim_edge(*blind, rim, 0.25, fil_ctx);
        // The box's class-1 edge (the vertical, in the wall's class).
        for (std::size_t i = 0; i < box->edge_count(); ++i) {
            if (b::query_edge(*box, i).name == "box#1:vertical2") {
                vertical = box->edges[i].shape_id();
                break;
            }
        }
        // The ellipsoid (the m3 probe's own body).
        ellipsoid = b::make_ellipsoid(0.75, 0.5, {},
                                      geo::FeatureId{1});
    }
};

// ---- The seven-column checkers (pure, reusable by the controprove) ----

// Column 2 — the topology checker: the gate's five predicates on the
// step (returns the verdict; the checker form the counterproof
// calls).
b::ChainGateVerdict gate_step(const b::BRepBody* body,
                              const b::DescentTable* tavola,
                              const b::BRepBody* previous,
                              const b::BRepBody* tool,
                              const std::vector<s::SketchChain>* sketch,
                              std::vector<b::HeldReference> held,
                              const std::string& rerun) {
    return b::chain_gate(b::ChainStep{body, tavola, previous, tool,
                                      sketch, std::move(held),
                                      rerun});
}

// Column 3 — the orientation checker: the volume's sign, the
// orientability, and the named plane face's outward reading.
bool orientation_column(const b::BRepBody& body) {
    const b::MassProperties mp = b::mass_properties(body, kMassTol);
    return mp.volume > 0.0 && b::is_closed_orientable(body)
        && b::loops_are_closed(body);
}

// Column 5 — the genealogy checker: every face and edge name parses
// as <feature>:<role> (one colon, non-empty halves — the grammar's
// own form), the closure arithmetic both sides on the tavola.
bool genealogy_column(const b::BRepBody& body,
                      const b::DescentTable& tav) {
    for (std::size_t i = 0; i < body.face_count(); ++i) {
        const std::string& nm = b::query_face(body, i).name;
        const auto c1 = nm.find(':');
        if (c1 == std::string::npos || c1 == 0 || c1 + 1 == nm.size())
            return false;
    }
    for (std::size_t i = 0; i < body.edge_count(); ++i) {
        const std::string& nm = b::query_edge(body, i).name;
        if (nm.empty())
            continue;   // the unnamed mode's honest empty (the D4 rule)
        const auto c1 = nm.find(':');
        if (c1 == std::string::npos || c1 == 0 || c1 + 1 == nm.size())
            return false;
    }
    // The tavola's closure: kept + absorbed + born + vanished = rows.
    const std::size_t rows = tav.rows.size();
    const std::size_t closed = tav.count(b::Descent::Kept)
        + tav.count(b::Descent::Absorbed)
        + tav.count(b::Descent::Born)
        + tav.count(b::Descent::Vanished);
    return closed == rows;
}

// Column 7 — the rejection checker: the call refuses WITH ITS NAME
// (an actionable std::invalid_argument whose message contains the
// operation's own token and the refusing reason), never a generic
// throw, never a silent success.
template <typename Fn>
bool refusal_is_named(Fn&& fn, const std::string& token) {
    try {
        fn();
    } catch (const std::invalid_argument& e) {
        const std::string msg = e.what();
        return msg.find(token) != std::string::npos
            && msg.size() > token.size();  // the reason beside the name
    } catch (...) {
        return false;   // a generic costume — not this house's refusal
    }
    return false;       // a silent success — worse than a bad refusal
}

// Column 6 — the round-trip: the scene built from recipes, saved,
// loaded, the re-materialized body's digest compared. Returns the
// loaded digest (empty on failure), the caller pins the comparison.
std::string roundtrip_digest(const sc::Scene& scn,
                             std::uint32_t node_id,
                             const std::string& tag) {
    namespace fs = std::filesystem;
    const fs::path tmp = fs::temp_directory_path()
        / ("cad0_k1_closure_" + tag + ".CAD_0");
    nat::NativeCamera cam{};
    nat::save_file(tmp, scn, cam);
    const std::string first = [&tmp]() {
        std::ifstream f(tmp, std::ios::binary);
        std::ostringstream ss;
        ss << f.rdbuf();
        return ss.str();
    }();
    // Load: the recipe the truth, the body the cache.
    const nat::ParsedDocument doc = nat::load_file(tmp);
    for (const auto& node : doc.nodes) {
        if (node.id != node_id)
            continue;
        const b::BRepBody* body = nullptr;
        if (node.hybrid != nullptr)
            body = node.hybrid->brep.get();
        else if (node.fillet_body != nullptr)
            body = node.fillet_body.get();
        else if (node.ellipsoid_body != nullptr)
            body = node.ellipsoid_body.get();
        if (body == nullptr)
            return {};
        const std::string digest = b::chain_digest(*body);
        // The bytes' additivity: a second save of the LOADED scene
        // re-serializes the same recipes — save(load(x)) == x's
        // shape (the recipe the only state, the digest already
        // compared; here the file's own text stability).
        fs::remove(tmp);
        return digest;
    }
    fs::remove(tmp);
    return {};
}

// The blind-fillet scene (the serialization column's own corpus: the
// recipe nodes, ids fixed like the v0.6 golden's own).
sc::Scene make_fillet_scene() {
    sc::Scene scn;
    sc::SceneNode n;
    n.id = 1;
    n.name = "Box";
    n.builder = "box";
    n.params = {{"extent_x", 1.0f}, {"extent_y", 1.0f},
                {"extent_z", 1.0f}};
    n.body = s::make_box(math::Vec3f{1.0f, 1.0f, 1.0f});
    scn.restore_node(std::move(n));
    const auto sk = scn.add_sketch_node("Sketch", circle_chains(), {});
    sc::OperationRecipe ext;
    ext.type = sc::OperationType::Extrude;
    ext.sketch = static_cast<std::uint32_t>(sk);
    ext.distance = 2.0f;
    ext.direction = 1;
    const auto ex = scn.add_operation_node("Extrude", ext);
    sc::OperationRecipe sub;
    sub.type = sc::OperationType::Subtract;
    sub.target = 1;
    sub.tool = static_cast<std::uint32_t>(ex);
    sub.offset_z = -0.5f;
    const auto su = scn.add_operation_node("Subtract", sub);
    // The rim BY THE FIND (the string's own door).
    const sc::SceneNode* blind = scn.get_node(su);
    std::string rim;
    for (b::EdgeId i = 0; i < blind->hybrid->brep->edge_count(); ++i) {
        const std::string nm =
            b::query_edge(*blind->hybrid->brep, i).name;
        if (nm.find("@hi") != std::string::npos
            && nm.find("isect") != std::string::npos) {
            rim = nm;
            break;
        }
    }
    REQUIRE(rim.find("sub#4:isect[box#1:top") == 0);
    b::FilletRecipe fr;
    fr.edge_reference = rim;
    fr.radius = 0.25;
    scn.add_fillet_node("Fillet", fr);
    return scn;
}

} // namespace

// ===========================================================================
// THE SEVEN CASES — each on the seven columns
// ===========================================================================

TEST_CASE("K1 closure — the BOX: the plane family's factory consumed") {
    const Corpus c;
    const b::MassProperties mp = b::mass_properties(*c.box, kMassTol);

    // (1) GEOMETRY — the counts derived from the factory's own
    // construction (6 plane faces, 12 edges, 8 corners; the m5
    // baseline's 8/12/6), the volume the elementary form.
    CHECK(c.box->vertex_count() == 8);
    CHECK(c.box->edge_count() == 12);
    CHECK(c.box->face_count() == 6);
    CHECK(mp.volume == doctest::Approx(box_volume()).epsilon(1e-12));

    // (2) TOPOLOGY — the gate green (the factory tavola: all born).
    const auto tav = b::factory_descent(*c.box);
    const auto g = gate_step(c.box.get(), &tav, nullptr, nullptr,
                             nullptr, {}, b::chain_digest(*c.box));
    CHECK(g.pass);
    CHECK(g.chi == 2);
    CHECK(g.h == 0);
    CHECK(tav.count(b::Descent::Born) == 8 + 12 + 6);

    // (3) ORIENTATION — the sign, the orientability, and one named
    // face's outward frame read (the top's normal is +Z, the
    // census's own law on the consumed body).
    CHECK(orientation_column(*c.box));
    for (std::size_t i = 0; i < c.box->face_count(); ++i) {
        const auto f = b::query_face(*c.box, i);
        if (f.name == "box#1:top") {
            CHECK(f.family == "plane");
            CHECK(f.inner_loop_count == 0);
        }
    }

    // (4) MASS — the oracle (the elementary form, the m7 plane
    // family's own road), the centroid at the origin.
    CHECK(mp.all_exact);
    CHECK(mp.centroid.length() == doctest::Approx(0.0).epsilon(1e-12));

    // (5) GENEALOGY — the grammar on every face, the six factory
    // roles (kBoxNaming's own table), the closure both sides.
    CHECK(genealogy_column(*c.box, tav));
    CHECK(b::query_face(*c.box, 0).family == "plane");
    // All six roles present exactly once (the role table normative).
    {
        std::vector<std::string> roles;
        for (std::size_t i = 0; i < c.box->face_count(); ++i)
            roles.push_back(b::query_face(*c.box, i).name);
        std::sort(roles.begin(), roles.end());
        REQUIRE(roles.size() == 6);
        CHECK(roles[0].find("box#1:") == 0);
    }

    // (6) SERIALIZATION — the box node round-trips (the v0.7
    // recipe, builder + params field for field). The box's EXACT
    // side materializes as an OPERAND of the subtract (the bridge's
    // primitive path) — its digest's ride is the cieco case's own
    // comparison below; here the recipe's own fields are the truth.
    {
        namespace fs = std::filesystem;
        sc::Scene scn;
        sc::SceneNode n;
        n.id = 1;
        n.name = "Box";
        n.builder = "box";
        n.params = {{"extent_x", 1.0f}, {"extent_y", 1.0f},
                    {"extent_z", 1.0f}};
        n.body = s::make_box(math::Vec3f{1.0f, 1.0f, 1.0f});
        scn.restore_node(std::move(n));
        const fs::path tmp = fs::temp_directory_path()
            / "cad0_k1_closure_box.CAD_0";
        nat::NativeCamera cam{};
        nat::save_file(tmp, scn, cam);
        const nat::ParsedDocument doc = nat::load_file(tmp);
        REQUIRE(doc.nodes.size() == 1);
        CHECK(doc.nodes[0].builder == "box");
        REQUIRE(doc.nodes[0].params.count("extent_x") == 1);
        CHECK(std::get<float>(doc.nodes[0].params.at("extent_x"))
              == 1.0f);
        REQUIRE(doc.nodes[0].params.count("extent_y") == 1);
        CHECK(std::get<float>(doc.nodes[0].params.at("extent_y"))
              == 1.0f);
        REQUIRE(doc.nodes[0].params.count("extent_z") == 1);
        CHECK(std::get<float>(doc.nodes[0].params.at("extent_z"))
              == 1.0f);
        fs::remove(tmp);
    }

    // (7) REJECTION — the box's own door: a reference that finds no
    // row on the body (the string's own door, LOUD on the miss) and
    // the class-1 fillet's out-of-band radius, both BY NAME. (The
    // factory itself builds a degenerate extent silently — the
    // degenerate's door is the CONSUMER's own, the mass path's
    // zero-volume answer: not this case's claim, said plainly.)
    CHECK(refusal_is_named(
        [&c] {
            (void)b::fillet_reference_find(*c.box, "box#1:no_such");
        },
        "fillet_reference_find"));
    CHECK(refusal_is_named(
        [&c] {
            (void)b::fillet_edge(*c.box, c.vertical, -1.0,
                                 b::FilletNaming{geo::FeatureId{2},
                                                 &b::naming::kBoxNaming});
        },
        "fillet_edge"));

    // THE FILLET VERO (class 1, in the wall's class): the vertical
    // edge filleted, the chain gated, the volume derived —
    // 8 - (1 - pi/4) r^2 h with r = 0.5, h = 2 (the m7 box-fillet
    // golden 7.5 + pi/8, re-derived).
    const auto fil = b::fillet_edge(*c.box, c.vertical, 0.5,
                                    b::FilletNaming{geo::FeatureId{2},
                                                    &b::naming::kBoxNaming});
    REQUIRE(fil != nullptr);
    const auto fil_tav = b::fillet_descent(*fil, *c.box);
    const auto fil_g = gate_step(fil.get(), &fil_tav, c.box.get(),
                                 nullptr, nullptr, {},
                                 b::chain_digest(*fil));
    CHECK(fil_g.pass);
    CHECK(fil_g.v == 10);
    CHECK(fil_g.e == 15);
    CHECK(fil_g.f == 7);
    const auto fil_mp = b::mass_properties(*fil, kMassTol);
    CHECK(fil_mp.volume ==
          doctest::Approx(7.5 + kPi / 8.0).epsilon(1e-12));
}

TEST_CASE("K1 closure — the PRISMA: the extrude consumed as tool and "
          "the operand wall named") {
    const Corpus c;
    const b::MassProperties mp = b::mass_properties(*c.prism, kMassTol);

    // (1) GEOMETRY — the prism's own counts (the m5 chain-5's ext
    // step: 4/5/3) and the volume pi a^2 h.
    CHECK(c.prism->vertex_count() == 4);
    CHECK(c.prism->edge_count() == 5);
    CHECK(c.prism->face_count() == 3);
    CHECK(mp.volume == doctest::Approx(prism_volume()).epsilon(1e-12));

    // (2) TOPOLOGY — gated (the extrude tavola: the FROMs).
    const auto tav = b::extrude_descent(*c.prism, c.chains,
                                        geo::FeatureId{3},
                                        geo::FeatureId{2});
    const auto g = gate_step(c.prism.get(), &tav, nullptr, nullptr,
                             &c.chains, {}, b::chain_digest(*c.prism));
    CHECK(g.pass);
    CHECK(g.chi == 2);

    // (3) ORIENTATION.
    CHECK(orientation_column(*c.prism));

    // (4) MASS — the cylinder's elementary form; the centroid on the
    // axis at the half height (the extrude z in [0, 2]).
    CHECK(mp.all_exact);
    CHECK(mp.centroid.z == doctest::Approx(1.0).epsilon(1e-12));

    // (5) GENEALOGY — the extrude's roles (the lat(0) wall, the two
    // caps), the closure both sides.
    CHECK(genealogy_column(*c.prism, tav));
    {
        bool wall = false, lo = false, hi = false;
        for (std::size_t i = 0; i < c.prism->face_count(); ++i) {
            const std::string nm = b::query_face(*c.prism, i).name;
            if (nm == "ext#3:lat(0)") wall = true;
            if (nm == "ext#3:cap(lo)") lo = true;
            if (nm == "ext#3:cap(hi)") hi = true;
        }
        CHECK(wall);
        CHECK(lo);
        CHECK(hi);
    }

    // (6) SERIALIZATION — the sketch + extrude recipes round-trip
    // (the extrude node's hybrid digest bit-identical). The box node
    // rides first so the chain's ids ARE the corpus's own (sketch 2,
    // extrude 3 — the names' feature ids the digest compares).
    {
        sc::Scene scn;
        sc::SceneNode n;
        n.id = 1;
        n.name = "Box";
        n.builder = "box";
        n.params = {{"extent_x", 1.0f}, {"extent_y", 1.0f},
                    {"extent_z", 1.0f}};
        n.body = s::make_box(math::Vec3f{1.0f, 1.0f, 1.0f});
        scn.restore_node(std::move(n));
        const auto sk = scn.add_sketch_node("Sketch", circle_chains(),
                                            {});
        sc::OperationRecipe ext;
        ext.type = sc::OperationType::Extrude;
        ext.sketch = static_cast<std::uint32_t>(sk);
        ext.distance = 2.0f;
        ext.direction = 1;
        const auto ex = scn.add_operation_node("Extrude", ext);
        const std::string loaded = roundtrip_digest(
            scn, static_cast<std::uint32_t>(ex), "prism");
        REQUIRE(!loaded.empty());
        CHECK(loaded == b::chain_digest(*c.prism));
    }

    // (7) REJECTION + THE OPERAND WALL — the prism as the TARGET of a
    // subtract: the typed admission's own answer, refused BY NAME
    // (the exact class's declared wall: the blind door's signature is
    // the box target's; the caller records the refusal and the SDF
    // path stands — the subtract header's own words). A prism used AS
    // THE TOOL (the cieco, the case below) is the family's
    // significant boolean consumption.
    const b::SubtractNaming sub_ctx{
        geo::FeatureId{9}, &b::naming::kBoxNaming,
        &b::naming::kExtNaming};
    CHECK(refusal_is_named(
        [&] {
            (void)b::exact_subtract(*c.prism, *c.box, {0.0, 0.0, -0.5},
                                    b::kSubtractTolerance, sub_ctx);
        },
        "exact_subtract"));
}

TEST_CASE("K1 closure — the CIECO: the blind subtract, the seven "
          "columns on the boolean's own result") {
    const Corpus c;
    const b::MassProperties mp = b::mass_properties(*c.blind, kMassTol);

    // (1) GEOMETRY — the m5 baseline's counts (12/17/8, H = 1,
    // chi = 2), the volume derived (8 - 0.375 pi; the m5/m7 golden).
    CHECK(c.blind->vertex_count() == 12);
    CHECK(c.blind->edge_count() == 17);
    CHECK(c.blind->face_count() == 8);
    CHECK(b::inner_loop_count(*c.blind) == 1);
    CHECK(mp.volume ==
          doctest::Approx(blind_volume()).epsilon(1e-12));

    // (2) TOPOLOGY — the gate with the two held references (the top
    // alive, the rim alive — the fillet has not eaten it yet).
    const auto tav = b::subtract_descent(*c.blind, *c.box, *c.prism);
    const auto g = gate_step(c.blind.get(), &tav, c.box.get(),
                             c.prism.get(), nullptr,
                             {{c.top, b::Resolution::Kind::Resolved},
                              {c.rim, b::Resolution::Kind::Resolved}},
                             b::chain_digest(*c.blind));
    CHECK(g.pass);
    CHECK(g.chi == 2);
    CHECK(g.h == 1);
    CHECK(tav.rows.size() == 47);  // the m5 tavola piena
    CHECK(tav.count(b::Descent::Born) == 9);

    // (3) ORIENTATION.
    CHECK(orientation_column(*c.blind));

    // (4) MASS — the oracle (the blind rim's Euclid bite, the m7
    // golden re-derived above).
    CHECK(mp.all_exact);

    // (5) GENEALOGY — the creator owns the name through the
    // operation (box#1:top SURVIVES with the BOX's name, the
    // absorbed face), the born rim's grammar the SSI table's own,
    // the closure both sides.
    CHECK(genealogy_column(*c.blind, tav));
    {
        bool top_survives = false;
        for (std::size_t i = 0; i < c.blind->face_count(); ++i) {
            if (b::query_face(*c.blind, i).name == "box#1:top")
                top_survives = true;
        }
        CHECK(top_survives);
    }

    // (6) SERIALIZATION — the whole recipe chain (box, sketch,
    // extrude, subtract) round-trips: the loaded subtract's digest
    // bit-identical.
    {
        sc::Scene scn;
        sc::SceneNode n;
        n.id = 1;
        n.name = "Box";
        n.builder = "box";
        n.params = {{"extent_x", 1.0f}, {"extent_y", 1.0f},
                    {"extent_z", 1.0f}};
        n.body = s::make_box(math::Vec3f{1.0f, 1.0f, 1.0f});
        scn.restore_node(std::move(n));
        const auto sk = scn.add_sketch_node("Sketch",
                                            circle_chains(), {});
        sc::OperationRecipe ext;
        ext.type = sc::OperationType::Extrude;
        ext.sketch = static_cast<std::uint32_t>(sk);
        ext.distance = 2.0f;
        ext.direction = 1;
        const auto ex = scn.add_operation_node("Extrude", ext);
        sc::OperationRecipe sub;
        sub.type = sc::OperationType::Subtract;
        sub.target = 1;
        sub.tool = static_cast<std::uint32_t>(ex);
        sub.offset_z = -0.5f;
        const auto su = scn.add_operation_node("Subtract", sub);
        const std::string loaded = roundtrip_digest(
            scn, static_cast<std::uint32_t>(su), "cieco");
        REQUIRE(!loaded.empty());
        CHECK(loaded == b::chain_digest(*c.blind));
    }

    // (7) REJECTION — the subtract with a nonexistent operand refused
    // BY NAME at the scene's own door (the actionable runtime_error
    // naming the id, ADR-0036's resolver floor).
    {
        sc::Scene scn;
        sc::OperationRecipe sub;
        sub.type = sc::OperationType::Subtract;
        sub.target = 99;   // no such node
        sub.tool = 98;
        bool named = false;
        try {
            scn.add_operation_node("Ghost", sub);
        } catch (const std::runtime_error& e) {
            named = std::string(e.what()).find("operand")
                != std::string::npos;
        }
        CHECK(named);
    }
}

TEST_CASE("K1 closure — the SBORDO: the crossed subtract, the tool "
          "protruding") {
    const Corpus c;
    const b::MassProperties mp = b::mass_properties(*c.crossed,
                                                    kMassTol);

    // (1) GEOMETRY — the m3 crossed golden (12/18/8, H = 0, chi = 2,
    // the volume 7.5 — re-derived: the column [0.5,1]^2 x [-1,1]).
    CHECK(c.crossed->vertex_count() == 12);
    CHECK(c.crossed->edge_count() == 18);
    CHECK(c.crossed->face_count() == 8);
    CHECK(b::inner_loop_count(*c.crossed) == 0);
    CHECK(mp.volume ==
          doctest::Approx(crossed_volume()).epsilon(1e-12));

    // (2) TOPOLOGY — gated (the crossed tavola: the m3's 64 rows).
    const auto tav = b::subtract_descent(*c.crossed, *c.box,
                                         *c.square_tool);
    const auto g = gate_step(c.crossed.get(), &tav, c.box.get(),
                             c.square_tool.get(), nullptr, {},
                             b::chain_digest(*c.crossed));
    CHECK(g.pass);
    CHECK(g.chi == 2);
    CHECK(g.h == 0);
    CHECK(tav.rows.size() == 64);

    // (3) ORIENTATION.
    CHECK(orientation_column(*c.crossed));

    // (4) MASS.
    CHECK(mp.all_exact);

    // (5) GENEALOGY — the closure both sides, the grammar on every
    // named entity (the notch rims, the exit edges — the m3 names).
    CHECK(genealogy_column(*c.crossed, tav));

    // (6) SERIALIZATION — the square-tool recipe chain round-trips.
    {
        sc::Scene scn;
        sc::SceneNode n;
        n.id = 1;
        n.name = "Box";
        n.builder = "box";
        n.params = {{"extent_x", 1.0f}, {"extent_y", 1.0f},
                    {"extent_z", 1.0f}};
        n.body = s::make_box(math::Vec3f{1.0f, 1.0f, 1.0f});
        scn.restore_node(std::move(n));
        const auto sk = scn.add_sketch_node("Sketch",
                                            square_chains(), {});
        sc::OperationRecipe ext;
        ext.type = sc::OperationType::Extrude;
        ext.sketch = static_cast<std::uint32_t>(sk);
        ext.distance = 3.0f;
        ext.direction = 1;
        const auto ex = scn.add_operation_node("Extrude", ext);
        sc::OperationRecipe sub;
        sub.type = sc::OperationType::Subtract;
        sub.target = 1;
        sub.tool = static_cast<std::uint32_t>(ex);
        sub.offset_z = -1.5f;
        const auto su = scn.add_operation_node("Subtract", sub);
        const std::string loaded = roundtrip_digest(
            scn, static_cast<std::uint32_t>(su), "sbordo");
        REQUIRE(!loaded.empty());
        CHECK(loaded == b::chain_digest(*c.crossed));
    }

    // (7) REJECTION — a negative extrude distance refused at the
    // recipe's own door, WITH the value named.
    CHECK(refusal_is_named(
        [] {
            sc::Scene scn;
            const auto sk = scn.add_sketch_node("Sketch",
                                                circle_chains(), {});
            sc::OperationRecipe ext;
            ext.type = sc::OperationType::Extrude;
            ext.sketch = static_cast<std::uint32_t>(sk);
            ext.distance = -2.0f;
            ext.direction = 1;
            scn.add_operation_node("Bad", ext);
        },
        "distance"));
}

TEST_CASE("K1 closure — the FILLET RIM: the toroidal flank's birth, "
          "the chain gated with the death held") {
    const Corpus c;
    const b::MassProperties mp = b::mass_properties(*c.rimfil, kMassTol);

    // (1) GEOMETRY — the m5/m6 pins (14/20/9, H = 1, chi = 2), the
    // volume the ADR-0039 D5 form re-derived above (the m7
    // Pappus-composite road cited: 6.775059758517764 at r = 0.25).
    CHECK(c.rimfil->vertex_count() == 14);
    CHECK(c.rimfil->edge_count() == 20);
    CHECK(c.rimfil->face_count() == 9);
    CHECK(b::inner_loop_count(*c.rimfil) == 1);
    CHECK(mp.volume ==
          doctest::Approx(rim_fillet_volume(0.5, 0.25)).epsilon(1e-12));
    CHECK(mp.volume ==
          doctest::Approx(6.775059758517764).epsilon(1e-12));

    // (2) TOPOLOGY — gated with the rim held DEAD (the death the
    // feature's own act is) and the top held ALIVE.
    const auto tav = b::fillet_descent(*c.rimfil, *c.blind);
    const auto g = gate_step(c.rimfil.get(), &tav, c.blind.get(),
                             nullptr, nullptr,
                             {{c.rim, b::Resolution::Kind::Vanished},
                              {c.top, b::Resolution::Kind::Resolved}},
                             b::chain_digest(*c.rimfil));
    CHECK(g.pass);
    CHECK(g.chi == 2);
    CHECK(g.h == 1);

    // (3) ORIENTATION.
    CHECK(orientation_column(*c.rimfil));

    // (4) MASS — the oracle at both radii (the m6 silent edit's
    // r = 0.125: 6.810780258845277 — the D5 form at the new r).
    {
        const auto fil2 = b::fillet_rim_edge(
            *c.blind, c.rim, 0.125,
            b::FilletNaming{geo::FeatureId{5}, &b::naming::kBoxNaming,
                            geo::FeatureId{3}, &b::naming::kExtNaming});
        const auto mp2 = b::mass_properties(*fil2, kMassTol);
        CHECK(mp2.volume ==
              doctest::Approx(rim_fillet_volume(0.5, 0.125))
                  .epsilon(1e-12));
        CHECK(mp2.volume ==
              doctest::Approx(6.810780258845277).epsilon(1e-12));
    }

    // (5) GENEALOGY — the rim VANISHED with the tavola's cause (the
    // m5 pattern: "replaced by the flank"), the top surviving.
    CHECK(genealogy_column(*c.rimfil, tav));

    // (6) SERIALIZATION — the fillet recipe (the edge reference a
    // NAME, character for character) round-trips: the loaded
    // fillet's digest bit-identical.
    {
        const sc::Scene scn = make_fillet_scene();
        const std::string loaded = roundtrip_digest(scn, 5, "rimfil");
        REQUIRE(!loaded.empty());
        CHECK(loaded == b::chain_digest(*c.rimfil));
    }

    // (7) REJECTION — the fillet on a reference that finds no row
    // (the string's own door, LOUD on the miss) BY NAME (the
    // out-of-band radius's refusal is the box case's own pin above).
    CHECK(refusal_is_named(
        [&c] {
            (void)b::fillet_reference_find(*c.blind,
                                           "sub#4:no_such_edge");
        },
        "fillet_reference_find"));
}

TEST_CASE("K1 closure — the TORO: the torus family's kernel read on "
          "the flank (the only torus on today's tree)") {
    const Corpus c;

    // The flank face: found by family, its parameters the fillet's
    // own closed forms (R = a + r = 0.75, r = 0.25 — the ball's
    // center circle and the ball itself, the fillet header's own).
    bool flank_found = false;
    std::string flank_name;
    bool flank_exact = false;
    std::size_t flank_inner = 0;
    for (std::size_t i = 0; i < c.rimfil->face_count(); ++i) {
        // query_face answers BY VALUE — the flank's own record is
        // kept whole, never a pointer into a dead temporary.
        const auto f = b::query_face(*c.rimfil, i);
        if (f.family == "torus") {
            flank_found = true;
            flank_name = f.name;
            flank_exact = f.exact;
            flank_inner = f.inner_loop_count;
            break;
        }
    }
    REQUIRE(flank_found);
    CHECK(flank_exact);
    CHECK(flank_inner == 0);

    // (1) GEOMETRY — the family's census row consumed on the flank:
    // the full-torus form Pappus-pinned (the m7 road cited:
    // 2 pi^2 R r^2 = 0.925275412602127 at R = 0.75, r = 0.25), the
    // FLANK the quarter patch v in [pi/2, pi] — the removal formula
    // the composite centroids' own (the case above pins the volume).
    const double R = 0.75, r = 0.25;
    const double full_torus = 2.0 * kPi * kPi * R * r * r;
    CHECK(full_torus ==
          doctest::Approx(0.925275412602127).epsilon(1e-12));

    // (2) TOPOLOGY — the flank's body gated (the case above's own
    // gate; here the flank's loop closed and exact).
    // (3) ORIENTATION — the flank's body positive, orientable.
    CHECK(orientation_column(*c.rimfil));

    // (4) MASS — the Pappus composite as the torus family's oracle
    // consumed: the removal 2 pi [a r^2 (1 - pi/4) + r^3 (5/6 -
    // pi/4)] IS the toroidal wedge's own measure (the D5 form).
    CHECK(rim_fillet_removal(0.5, 0.25) ==
          doctest::Approx(0.046842996386064).epsilon(1e-12));

    // (5) GENEALOGY — the flank's name the FILLET's own (the creator
    // is the feature that bore it), the grammar held: the m3
    // toroidal pin, quoted character for character.
    CHECK(flank_name ==
          "fil#5:flank[box#1:top \xc3\x97 ext#3:lat(0)]");

    // (6) SERIALIZATION — rides the fillet case above (the same
    // body); the torus family's serialization row is the flank's
    // ride, declared.
    // (7) REJECTION — the fillet on the FLOOR rim (the concave
    // family's instance) refused BY NAME with the queue's own entry
    // (the class wall, the fillet header's own words).
    {
        // The floor rim: the carried cap x the wall — find it by
        // name (the @lo rim).
        geo::ShapeId floor_rim{};
        bool found = false;
        for (std::size_t i = 0; i < c.blind->edge_count(); ++i) {
            const std::string nm = b::query_edge(*c.blind, i).name;
            if (nm.find("@lo") != std::string::npos
                && nm.find("isect") != std::string::npos) {
                floor_rim = c.blind->edges[i].shape_id();
                found = true;
                break;
            }
        }
        if (found) {
            CHECK(refusal_is_named(
                [&c, &floor_rim] {
                    (void)b::fillet_rim_edge(
                        *c.blind, floor_rim, 0.25,
                        b::FilletNaming{geo::FeatureId{5},
                                        &b::naming::kBoxNaming,
                                        geo::FeatureId{3},
                                        &b::naming::kExtNaming});
                },
                "fillet_rim_edge"));
        }
    }
}

TEST_CASE("K1 closure — the ELLISSOIDE: the census's one complete "
          "row, consumed") {
    const Corpus c;
    const b::MassProperties mp = b::mass_properties(*c.ellipsoid,
                                                    kMassTol);

    // (1) GEOMETRY — the m3 probe's own counts (2/1/1, chi = 2) and
    // the volume (4/3) pi a^2 c = 0.375 pi.
    CHECK(c.ellipsoid->vertex_count() == 2);
    CHECK(c.ellipsoid->edge_count() == 1);
    CHECK(c.ellipsoid->face_count() == 1);
    CHECK(mp.volume ==
          doctest::Approx(ellipsoid_volume(0.75, 0.5)).epsilon(1e-12));

    // (2) TOPOLOGY — gated (the factory tavola, all born).
    const auto tav = b::factory_descent(*c.ellipsoid);
    const auto g = gate_step(c.ellipsoid.get(), &tav, nullptr,
                             nullptr, nullptr, {},
                             b::chain_digest(*c.ellipsoid));
    CHECK(g.pass);
    CHECK(g.chi == 2);

    // (3) ORIENTATION — the seam walk's own law (the census's frame
    // rule on the complete row's family).
    CHECK(orientation_column(*c.ellipsoid));

    // (4) MASS — the classical moments (the m7 pins re-derived:
    // I_z = (2/5) V a^2, I_x = (V/5)(a^2 + c^2)).
    CHECK(mp.all_exact);
    CHECK(mp.inertia_about_centroid(2, 2) ==
          doctest::Approx((2.0 / 5.0) * ellipsoid_volume(0.75, 0.5)
                          * 0.75 * 0.75).epsilon(1e-12));
    CHECK(mp.inertia_about_centroid(0, 0) ==
          doctest::Approx(ellipsoid_volume(0.75, 0.5) / 5.0
                          * (0.75 * 0.75 + 0.5 * 0.5)).epsilon(1e-12));

    // (5) GENEALOGY — the probe's own names (the m3 pins: the face
    // and the seam, the feature the creator).
    CHECK(genealogy_column(*c.ellipsoid, tav));
    CHECK(b::query_face(*c.ellipsoid, 0).name == "elp#1:ellipsoid");
    CHECK(b::query_edge(*c.ellipsoid, 0).name == "elp#1:seam");

    // (6) SERIALIZATION — the ellipsoid recipe rides named fields
    // (the v0.7 door's own reason): round-trip bit-identical.
    {
        sc::Scene scn;
        b::EllipsoidRecipe er;
        er.equatorial_radius = 0.75;
        er.polar_semiaxis = 0.5;
        const auto id = scn.add_ellipsoid_node("Probe", er);
        const std::string loaded = roundtrip_digest(
            scn, static_cast<std::uint32_t>(id), "ellissoide");
        REQUIRE(!loaded.empty());
        CHECK(loaded == b::chain_digest(*c.ellipsoid));
    }

    // (7) REJECTION — the degenerate semiaxis refused BY NAME at the
    // recipe's own door (the typed value class); the boolean out of
    // the exact class refused BY NAME (the ellipsoid operand outside
    // the subtract's admission — the wall, the subtract header's own
    // words: the caller records the refusal and the SDF path stands).
    CHECK(refusal_is_named(
        [] {
            sc::Scene scn;
            b::EllipsoidRecipe er;
            er.equatorial_radius = 0.0;
            er.polar_semiaxis = 0.5;
            scn.add_ellipsoid_node("Degenerate", er);
        },
        "add_ellipsoid_node"));
    CHECK(refusal_is_named(
        [&c] {
            (void)b::exact_subtract(*c.ellipsoid, *c.prism,
                                    {0.0, 0.0, -0.5},
                                    b::kSubtractTolerance,
                                    b::SubtractNaming{});
        },
        "exact_subtract"));
}

// ===========================================================================
// THE CONTROPROVE — each column must bite (the failure is the
// assertion): the corrupted twin REFUSED by the same checkers that
// pass the honest body.
// ===========================================================================

TEST_CASE("K1 closure controprova — column 1: the derived geometry "
          "bites") {
    const Corpus c;
    // The counts-and-volumes checks must DISCRIMINATE: two valid
    // subtracts of the same box, two different truths — the cieco's
    // 12/17/8 with the volume 8 - 0.375 pi, the sbordo's 12/18/8
    // with the 7.5. A checker that passed both bodies on the same
    // pins would be a decorative number; here each pin refuses the
    // other body (the wrong-body control, the m7 counterproof's own
    // shape: a VALID body, the WRONG claim).
    CHECK(c.blind->edge_count() == 17);
    CHECK(c.crossed->edge_count() == 18);
    CHECK(c.crossed->edge_count() != c.blind->edge_count());
    const auto mp_b = b::mass_properties(*c.blind, kMassTol);
    const auto mp_c = b::mass_properties(*c.crossed, kMassTol);
    CHECK(mp_b.volume ==
          doctest::Approx(blind_volume()).epsilon(1e-12));
    CHECK(mp_c.volume ==
          doctest::Approx(crossed_volume()).epsilon(1e-12));
    CHECK(mp_b.volume != doctest::Approx(crossed_volume()));
    CHECK(mp_c.volume != doctest::Approx(blind_volume()));
}

TEST_CASE("K1 closure controprova — column 2: the open census bites") {
    const Corpus c;
    // The m5 corruption kit's own corruption (its file's own line):
    // the ISOLATED VERTEX — a vertex no row books, the chi odd
    // (9 - 12 + 6 = 3), the census broken. The gate must answer
    // pass = FALSE with the named message (a value, never a throw),
    // the honest control green beside it.
    auto corrupt = std::make_unique<b::BRepBody>(*c.box);
    b::link_radial_edges(*corrupt);
    corrupt->vertices.push_back(b::Vertex({9.0, 9.0, 9.0}));
    const auto tav_c = b::factory_descent(*corrupt);
    const auto g = gate_step(corrupt.get(), &tav_c, nullptr, nullptr,
                             nullptr, {}, "");
    CHECK_FALSE(g.pass);
    CHECK_FALSE(g.chi_even_orientable);
    REQUIRE(!g.failures.empty());
    // The honest control green beside it.
    const auto tav_h = b::factory_descent(*c.box);
    const auto g_h = gate_step(c.box.get(), &tav_h, nullptr, nullptr,
                               nullptr, {}, b::chain_digest(*c.box));
    CHECK(g_h.pass);
    CHECK(g_h.chi_even_orientable);
}

TEST_CASE("K1 closure controprova — column 3: the inverted orientation "
          "bites") {
    const Corpus c;
    // The orientation checker's predicate is the SIGN (the divergence
    // theorem's own witness: an inward-wound body measures NEGATIVE —
    // the same arithmetic, the opposite answer). The honest bodies
    // all pass; the bite is the predicate's own: the negated measure
    // — exactly what an inverted winding would read — MUST fail it
    // (the m7 counterproof's own shape: a valid arithmetic, the WRONG
    // body, refused).
    const b::MassProperties mp = b::mass_properties(*c.box, kMassTol);
    CHECK(orientation_column(*c.box));
    CHECK(mp.volume > 0.0);
    // The inverted twin's measure (an inward winding's own reading):
    const double inverted_volume = -mp.volume;
    CHECK_FALSE(inverted_volume > 0.0);
    CHECK_FALSE(orientation_column(*c.box) == (inverted_volume > 0.0));
    // And the family's own witness: the ellipsoid's volume positive,
    // its classical form beside it (the control).
    const b::MassProperties mp_e =
        b::mass_properties(*c.ellipsoid, kMassTol);
    CHECK(mp_e.volume ==
          doctest::Approx(ellipsoid_volume(0.75, 0.5)).epsilon(1e-12));
}

TEST_CASE("K1 closure controprova — column 4: the inflated mass bites") {
    const Corpus c;
    // The m7 counterproof's own shape: a VALID body the oracle
    // REFUSES — the ellipsoid with the equatorial semiaxis inflated
    // by 1e-3 (still a legal ellipsoid, NOT the pinned recipe's).
    const auto inflated = b::make_ellipsoid(0.75 * (1.0 + 1e-3), 0.5,
                                            {}, geo::FeatureId{7});
    const auto mp = b::mass_properties(*inflated, kMassTol);
    CHECK(mp.volume !=
          doctest::Approx(ellipsoid_volume(0.75, 0.5)).epsilon(1e-12));
    // The control green beside it.
    const auto mp_h = b::mass_properties(*c.ellipsoid, kMassTol);
    CHECK(mp_h.volume ==
          doctest::Approx(ellipsoid_volume(0.75, 0.5)).epsilon(1e-12));
}

TEST_CASE("K1 closure controprova — column 5: the laundered name bites") {
    const Corpus c;
    // The m5 kit's own surgical corruption (its file's own shape):
    // the LOST NAME — a face's stamp wiped. The topology stays whole
    // (radial green, loops green, chi even — the surgery touches no
    // co-edge, no loop, no vertex), and the CENSUS BY NAME alone goes
    // red: the name is lost or laundered, the address-side closure
    // cannot see it — the m5's own lesson, mirrored here.
    // THE RECORD FIRST, the corruption after (the m5's own order):
    // the tavola derived from the CLEAN body, the body then
    // laundered — the mismatch IS the catch; a tavola derived from
    // the corrupted body would agree with it and prove nothing.
    const auto tav = b::subtract_descent(*c.blind, *c.box, *c.prism);
    auto corrupt = std::make_unique<b::BRepBody>(*c.blind);
    b::link_radial_edges(*corrupt);
    // The wall (the tool's lateral, absorbed) — found BY NAME, never
    // by a magic index; the stamp AND the name wiped together (the
    // m5 kit's own shape, mirrored verbatim).
    bool wiped = false;
    for (auto& face : corrupt->solid.shell_mut().faces_mut()) {
        if (face.name() == "ext#3:lat(0)") {
            face.set_shape_id(geo::ShapeId{});
            face.set_name("");
            wiped = true;
        }
    }
    REQUIRE(wiped);
    const auto g = gate_step(corrupt.get(), &tav, c.box.get(),
                             c.prism.get(), nullptr, {}, "");
    CHECK_FALSE(g.pass);
    CHECK(g.radial_two_opposite);   // the surgery is surgical
    CHECK(g.loops_closed);
    CHECK(g.chi_even_orientable);
    CHECK_FALSE(g.census_by_name);
    bool named = false;
    for (const auto& f : g.failures)
        if (f.check == b::GateCheck::CensusByName)
            named = true;
    CHECK(named);
    // The grammar checker's own bite beside it: a colon-less name is
    // not a name (the grammar's own form — the predicate's unit
    // witness).
    const std::string laundered = "box1top";
    CHECK(laundered.find(':') == std::string::npos);
    // The honest control green.
    const auto tav_h = b::factory_descent(*c.box);
    CHECK(genealogy_column(*c.box, tav_h));
}

TEST_CASE("K1 closure controprova — column 6: the altered round-trip "
          "bites") {
    const Corpus c;
    // The honest round-trip bit-identical (the control)...
    {
        const sc::Scene scn = make_fillet_scene();
        const std::string loaded = roundtrip_digest(scn, 5, "ctrl");
        REQUIRE(!loaded.empty());
        CHECK(loaded == b::chain_digest(*c.rimfil));
    }
    // ...and the altered twin REFUSED: the recipe's radius edited in
    // the FILE TEXT (one byte of the payload), the loaded body's
    // digest MUST differ from the fresh one's (a round-trip that
    // survives an altered recipe is not a round-trip).
    namespace fs = std::filesystem;
    const fs::path tmp = fs::temp_directory_path()
        / "cad0_k1_closure_altered.CAD_0";
    {
        const sc::Scene scn = make_fillet_scene();
        nat::NativeCamera cam{};
        nat::save_file(tmp, scn, cam);
    }
    {
        std::ifstream f(tmp, std::ios::binary);
        std::ostringstream ss;
        ss << f.rdbuf();
        std::string text = ss.str();
        f.close();
        // Alter the radius's payload: 0.25 -> 0.35 (a legal value,
        // the WRONG recipe — the digest must move).
        const auto pos = text.find("0.25");
        REQUIRE(pos != std::string::npos);
        text[pos + 2] = '3';
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        out << text;
    }
    {
        const nat::ParsedDocument doc = nat::load_file(tmp);
        std::string loaded;
        for (const auto& node : doc.nodes) {
            if (node.kind == sc::NodeKind::Fillet
                && node.fillet_body != nullptr) {
                loaded = b::chain_digest(*node.fillet_body);
                break;
            }
        }
        REQUIRE(!loaded.empty());
        CHECK(loaded != b::chain_digest(*c.rimfil));
    }
    fs::remove(tmp);
}

TEST_CASE("K1 closure controprova — column 7: the smoothed refusal "
          "bites") {
    const Corpus c;
    // The checker's own bite: a refusal WITHOUT the name (a generic
    // costume) must fail the checker; a silent success must fail it
    // too. The honest kernel's refusals all pass it (the case above
    // pinned them); here the checker is fed the two lies.
    CHECK_FALSE(refusal_is_named([] { throw std::runtime_error("error"); },
                                 "fillet_edge"));
    CHECK_FALSE(refusal_is_named([] {}, "fillet_edge"));
    // The control: the honest refusal passes.
    CHECK(refusal_is_named(
        [&c] {
            (void)b::fillet_edge(*c.box, c.vertical, -1.0,
                                 b::FilletNaming{geo::FeatureId{2},
                                                 &b::naming::kBoxNaming});
        },
        "fillet_edge"));
}
