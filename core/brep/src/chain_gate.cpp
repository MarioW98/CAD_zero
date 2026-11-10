// core/brep/src/chain_gate.cpp
//
// THE CHAIN GATE'S IMPLEMENTATION (ADR-0041 D1.3, K1 m5 — the header
// carries the contract; this file walks it). The order of the walk:
// the five predicates in their D1.3 order, every failure collected
// with its name — a gate that stopped at the first corruption would
// hide the second behind the first, and half a gate is no gate.
//
// The arithmetic is the authorities' own, verbatim: euler_
// characteristic / inner_loop_count / is_closed_orientable (ADR-0032
// D3's post-pass helpers), descent_closed / old_generation_closed /
// extrude_froms_resolve (ADR-0036 D2's closure checks),
// chain_is_closed (ADR-0030 D2's sketch predicate), query_all_faces
// / query_all_edges (the H2 unit 6 query surface). What is NEW here
// is the per-entity NAMING of the failures and the digest — the
// instrument D1.3 demands and the tree did not have: "each with its
// named failure message" is the metric's own sentence.
#include "CAD_0/brep/chain_gate.hpp"

#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

namespace CAD_0::brep {
namespace {

// The digest's quantizer — ADR-0014's double rule at the digest's
// own boundary: 12 significant digits, one snprintf, deterministic
// for the same double on the same flags (the chain's doubles are
// the kernel's own; the native file's float32 shortest-round-trip
// is the OTHER world's rule, never imported).
std::string q(double d) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.12g", d);
    return std::string(buf);
}

std::string q2(const math::Vec2d& p) {
    return "(" + q(p.x) + "," + q(p.y) + ")";
}

std::string q2f(const math::Vec2f& p) {
    return "(" + q(static_cast<double>(p.x)) + "," +
           q(static_cast<double>(p.y)) + ")";
}

std::string q3(const math::Vec3d& p) {
    return "(" + q(p.x) + "," + q(p.y) + "," + q(p.z) + ")";
}

// The by-name walk — the structural find the census reads (ShapeId
// equality, the operators' own semantics; the render never parsed).
// The v1 named classes are Edge and Face; the vertices carry no
// names (ADR-0036 D4's queue) and are SKIPPED here by design — the
// header says so where the rule bites.
bool edge_stamp_answers(const BRepBody& body,
                        const geometry::ShapeId& id) {
    for (const auto& e : body.edges)
        if (e.shape_id() == id) return true;
    return false;
}

bool face_stamp_answers(const BRepBody& body,
                        const geometry::ShapeId& id) {
    for (const auto& f : body.solid.shell().faces())
        if (f.shape_id() == id) return true;
    return false;
}

// The tavola's row for a held reference — the old generation's own
// bookkeeping read BY IDENTITY (the resolver's floor, D3's guard;
// vertex rows skipped: no vertex names in v1, and a default ShapeId
// must never match a vertex row's default by accident).
const DescentRow* row_of(const DescentTable& tav,
                         const geometry::ShapeId& id) {
    for (const auto& row : tav.rows) {
        if (row.entity == EntityKind::Vertex) continue;
        if (row.shape_id == id) return &row;
    }
    return nullptr;
}

// One p-curve as the digest reads it — the typed payload verbatim,
// every number quantized (the H2 unit 6 answer's own fields).
void pcurve_line(std::string& out, const PCurveInfo& pc) {
    out += "pc=";
    switch (pc.kind) {
    case PCurveInfo::Kind::None:
        out += "none";
        break;
    case PCurveInfo::Kind::Line:
        out += "line a=" + q2(pc.a) + " b=" + q2(pc.b) +
               " il=" + (pc.interior_left ? "1" : "0");
        break;
    case PCurveInfo::Kind::Circle:
        out += "circle c=" + q2(pc.center) + " r=" + q(pc.radius) +
               " hole=" + (pc.hole ? "1" : "0");
        break;
    case PCurveInfo::Kind::Arc:
        out += "arc c=" + q2(pc.center) + " r=" + q(pc.radius) +
               " hole=" + (pc.hole ? "1" : "0") +
               " sa=" + q(pc.start_angle) + " sw=" + q(pc.sweep) +
               " il=" + (pc.interior_left ? "1" : "0");
        break;
    }
}

} // namespace

const char* gate_check_token(GateCheck c) noexcept {
    switch (c) {
    case GateCheck::RadialTwoOpposite: return "radial";
    case GateCheck::LoopsClosed:       return "loops";
    case GateCheck::ChiEvenOrientable: return "chi";
    case GateCheck::CensusByName:      return "census";
    case GateCheck::DriftZero:         return "drift";
    }
    return "?";
}

// ---------------------------------------------------------------------------
// The digests
// ---------------------------------------------------------------------------

std::string chain_digest(const BRepBody& body) {
    std::string out;
    out.reserve(256);

    // The summary line — the authorities' own arithmetic (never
    // query_body here: it refuses the empty body, and the digest's
    // contract is to DESCRIBE, including the corrupt empties the
    // gate exists to catch).
    out += "body V=" + std::to_string(body.vertex_count()) +
           " E=" + std::to_string(body.edge_count()) +
           " F=" + std::to_string(body.face_count()) +
           " H=" + std::to_string(inner_loop_count(body)) +
           " chi=" + std::to_string(euler_characteristic(body)) +
           "\n";

    // The vertices — the geometry the drift must observe.
    for (std::size_t i = 0; i < body.vertices.size(); ++i)
        out += "v" + std::to_string(i) + " p=" +
               q3(body.vertices[i].position()) + "\n";

    // The edges — the query surface's own answer (endpoints, the
    // name riding the record).
    const auto edges = query_all_edges(body);
    for (std::size_t i = 0; i < edges.size(); ++i)
        out += "e" + std::to_string(i) + " v0=" +
               std::to_string(edges[i].v0) + " v1=" +
               std::to_string(edges[i].v1) + " name='" +
               edges[i].name + "' exact=" +
               (edges[i].exact ? "1" : "0") + "\n";

    // The faces — the full query answer: the family, the loops with
    // every co-edge's typed p-curve, the name.
    const auto faces = query_all_faces(body);
    for (std::size_t i = 0; i < faces.size(); ++i) {
        const auto& f = faces[i];
        out += "f" + std::to_string(i) + " family=" + f.family +
               " loops=" + std::to_string(f.loop_count) + " inner=" +
               std::to_string(f.inner_loop_count) + " name='" +
               f.name + "' exact=" + (f.exact ? "1" : "0") + "\n";
        for (std::size_t l = 0; l < f.loops.size(); ++l) {
            out += " l" + std::to_string(l) + " outer=" +
                   (f.loops[l].is_outer ? "1" : "0") + "\n";
            for (std::size_t c = 0; c < f.loops[l].coedges.size();
                 ++c) {
                const auto& pc = f.loops[l].coedges[c];
                out += "  c" + std::to_string(c) + " e=" +
                       std::to_string(pc.edge) + " rev=" +
                       (pc.reversed ? "1" : "0") + " ";
                pcurve_line(out, pc);
                out += "\n";
            }
        }
    }
    return out;
}

std::string sketch_digest(
    const std::vector<sdf::SketchChain>& chains) {
    std::string out;
    out += "profile chains=" + std::to_string(chains.size()) + "\n";
    for (std::size_t i = 0; i < chains.size(); ++i) {
        const auto& chain = chains[i];
        out += "c" + std::to_string(i) + " entities=" +
               std::to_string(chain.entities.size()) +
               " closed=" +
               (sdf::chain_is_closed(chain) ? "1" : "0") + "\n";
        for (std::size_t j = 0; j < chain.entities.size(); ++j) {
            const auto& e = chain.entities[j];
            out += " e" + std::to_string(j) + " kind=";
            switch (e.kind) {
            case sdf::SketchEntity::Kind::Segment:
                out += "segment a=" + q2f(e.segment.a) +
                       " b=" + q2f(e.segment.b) + " il=" +
                       (e.segment.interior_left ? "1" : "0");
                break;
            case sdf::SketchEntity::Kind::Circle:
                out += "circle c=" + q2f(e.circle.center) +
                       " r=" + q(e.circle.radius) + " hole=" +
                       (e.circle.hole ? "1" : "0");
                break;
            case sdf::SketchEntity::Kind::Arc:
                out += "arc c=" + q2f(e.arc.center) + " r=" +
                       q(e.arc.radius) + " hole=" +
                       (e.arc.hole ? "1" : "0") + " sa=" +
                       q(e.arc.start_angle) + " sw=" +
                       q(e.arc.sweep);
                break;
            }
            out += "\n";
        }
    }
    return out;
}

// ---------------------------------------------------------------------------
// The sketch step's door
// ---------------------------------------------------------------------------

SketchGateVerdict chain_gate_sketch(
    const std::vector<sdf::SketchChain>& chains,
    const std::string& rerun_digest) {
    SketchGateVerdict v;
    v.not_applicable = {GateCheck::RadialTwoOpposite,
                        GateCheck::ChiEvenOrientable,
                        GateCheck::CensusByName};

    // (2) LOOPS — the sketch world's own predicate, the one opinion
    // the kernel and the canvas already share (ADR-0030 D2).
    for (std::size_t i = 0; i < chains.size(); ++i) {
        if (!sdf::chain_is_closed(chains[i])) {
            v.loops_closed = false;
            v.failures.push_back({GateCheck::LoopsClosed,
                "the open profile: chain " + std::to_string(i) +
                " does not close (the sketch world's own predicate "
                "chain_is_closed — the extrude's closure "
                "precondition; the walk must return)"});
        }
    }

    // (5) DRIFT — the profile's digest against the re-run's.
    v.digest = sketch_digest(chains);
    if (rerun_digest.empty()) {
        v.not_applicable.push_back(GateCheck::DriftZero);
    } else if (v.digest != rerun_digest) {
        v.drift_zero = false;
        v.failures.push_back({GateCheck::DriftZero,
            "the drift: the profile's digest and the re-run's "
            "differ (the chain must reproduce bit-identically — "
            "ADR-0014)"});
    }

    v.pass = v.failures.empty();
    return v;
}

// ---------------------------------------------------------------------------
// The door
// ---------------------------------------------------------------------------

ChainGateVerdict chain_gate(const ChainStep& step) {
    if (step.body == nullptr) {
        throw std::invalid_argument(
            "chain_gate: the step carries no body — a body step's "
            "evidence is the result body (the radial, the loops, the "
            "chi, the census's stamps and the digest all read it); "
            "remedy: pass the step's result, or use "
            "chain_gate_sketch for the profile steps.");
    }

    ChainGateVerdict v;
    const BRepBody& body = *step.body;
    const auto fail = [&v](GateCheck check, std::string message) {
        v.failures.push_back({check, std::move(message)});
    };

    // The step's numbers — the authorities' own answers, first (the
    // messages below read them).
    v.v = body.vertex_count();
    v.e = body.edge_count();
    v.f = body.face_count();
    v.h = inner_loop_count(body);
    v.chi = euler_characteristic(body);

    // ------------------------------------------------------------------
    // (1) RADIAL — every edge exactly two OPPOSITE co-edges.
    // ------------------------------------------------------------------
    for (EdgeId i = 0; i < body.edges.size(); ++i) {
        const auto& ces = body.edges[i].coedges();
        if (ces.size() != 2) {
            v.radial_two_opposite = false;
            fail(GateCheck::RadialTwoOpposite,
                 "the radial violation: edge " + std::to_string(i) +
                 " ('" + body.edges[i].name() + "') carries " +
                 std::to_string(ces.size()) +
                 " co-edges (exactly two required — the radial "
                 "structure, the census's own arithmetic)");
            continue;
        }
        if (ces[0]->reversed() == ces[1]->reversed()) {
            v.radial_two_opposite = false;
            fail(GateCheck::RadialTwoOpposite,
                 "the radial violation: edge " + std::to_string(i) +
                 " ('" + body.edges[i].name() +
                 "') carries two CO-DIRECTED co-edges (opposite "
                 "orientations required — the duplicated interface "
                 "ring)");
        }
    }

    // ------------------------------------------------------------------
    // (2) LOOPS — every loop closed head-tail (the walk returns).
    // ------------------------------------------------------------------
    {
        const auto& faces = body.solid.shell().faces();
        for (std::size_t fi = 0; fi < faces.size(); ++fi) {
            const auto& loops = faces[fi].loops();
            for (std::size_t li = 0; li < loops.size(); ++li) {
                const auto& ces = loops[li].coedges();
                const std::size_t n = ces.size();
                for (std::size_t ci = 0; ci < n; ++ci) {
                    const VertexId end =
                        ces[ci].end_vertex(body.edges);
                    const VertexId start =
                        ces[(ci + 1) % n].start_vertex(body.edges);
                    if (end != start) {
                        v.loops_closed = false;
                        fail(GateCheck::LoopsClosed,
                             "the open loop: face " +
                             std::to_string(fi) + " ('" +
                             faces[fi].name() + "') loop " +
                             std::to_string(li) + " breaks at "
                             "co-edge " + std::to_string(ci) +
                             ": end vertex " +
                             std::to_string(end) +
                             " != start vertex " +
                             std::to_string(start) +
                             " of the next — the walk must return");
                    }
                }
            }
        }
    }

    // ------------------------------------------------------------------
    // (3) CHI — even, and the closed orientable composition.
    // ------------------------------------------------------------------
    if ((v.chi % 2 != 0)) {
        v.chi_even_orientable = false;
        fail(GateCheck::ChiEvenOrientable,
             "the odd chi: chi = V-E+F-H = " + std::to_string(v.chi) +
             " is ODD (a closed orientable composition's chi is "
             "even — the assembly defect the structural walk cannot "
             "see; an odd chi under a passing shell contract is "
             "reported, never smoothed into a half-integer genus)");
    }
    if (!is_closed_orientable(body)) {
        v.chi_even_orientable = false;
        fail(GateCheck::ChiEvenOrientable,
             "the body is not a closed orientable composition (the "
             "shell contract 0-3: V>0, E>0, F>0, the shell closed, "
             "every edge two co-edges, opposite, the loops closed)");
    }

    // ------------------------------------------------------------------
    // (4) CENSUS — the tavola's closure both sides, read BY NAME.
    // ------------------------------------------------------------------
    if (step.tavola == nullptr) {
        v.not_applicable.push_back(GateCheck::CensusByName);
    } else {
        const DescentTable& tav = *step.tavola;

        // The counts — the tavola's own arithmetic (the row's
        // born/kept/absorbed/vanished numbers).
        v.born = tav.count(Descent::Born);
        v.kept = tav.count(Descent::Kept);
        v.absorbed = tav.count(Descent::Absorbed);
        v.split = tav.count(Descent::Split);
        v.merged = tav.count(Descent::Merged);
        v.vanished = tav.count(Descent::Vanished);

        std::string why;

        // The RESULT side — the existing authority, its diagnosis
        // carried verbatim.
        if (!descent_closed(body, tav, &why)) {
            v.census_by_name = false;
            fail(GateCheck::CensusByName,
                 "the census: the tavola's result side does not "
                 "close — " + why);
        }

        // The OLD side — per the step's own operand shape (the
        // factory root books no old generation: vacuously closed,
        // the root has nothing behind it).
        if (step.previous != nullptr && step.tool != nullptr) {
            if (!old_generation_closed(*step.previous, *step.tool,
                                       tav, &why)) {
                v.census_by_name = false;
                fail(GateCheck::CensusByName,
                     "the census: the tavola's old side does not "
                     "close — " + why);
            }
        } else if (step.previous != nullptr) {
            const BRepBody no_second{};  // the fillet's own pattern
            if (!old_generation_closed(*step.previous, no_second,
                                       tav, &why)) {
                v.census_by_name = false;
                fail(GateCheck::CensusByName,
                     "the census: the tavola's old side does not "
                     "close — " + why);
            }
        } else if (step.sketch != nullptr) {
            if (!extrude_froms_resolve(tav, *step.sketch, &why)) {
                v.census_by_name = false;
                fail(GateCheck::CensusByName,
                     "the census: the tavola's FROM side does not "
                     "resolve — " + why);
            }
        }

        // BY NAME — the part the closure-by-address cannot see:
        // every result row's stamp ANSWERS on the body it describes,
        // every vanished row's stamp answers NOWHERE (the death is
        // real). The vertex rows are skipped: v1 has no vertex names
        // (ADR-0036 D4's queue), said in the header where it bites.
        for (const auto& row : tav.rows) {
            if (row.entity == EntityKind::Vertex) continue;
            if (row.relation == Descent::Vanished) {
                const bool answers =
                    row.entity == EntityKind::Edge
                        ? edge_stamp_answers(body, row.shape_id)
                        : face_stamp_answers(body, row.shape_id);
                if (answers) {
                    v.census_by_name = false;
                    fail(GateCheck::CensusByName,
                         "the census by name: the vanished " +
                         std::string(row.entity ==
                                     EntityKind::Edge
                                         ? "edge"
                                         : "face") +
                         " '" + row.name +
                         "' still answers on the result — the death "
                         "is not real (the tavola books it dead with "
                         "the cause '" + row.cause + "')");
                }
            } else {
                const bool answers =
                    row.entity == EntityKind::Edge
                        ? edge_stamp_answers(body, row.shape_id)
                        : face_stamp_answers(body, row.shape_id);
                const std::size_t addr =
                    static_cast<std::size_t>(row.address);
                const bool in_range =
                    row.entity == EntityKind::Edge
                        ? addr < body.edge_count()
                        : addr < body.face_count();
                const geometry::ShapeId& live =
                    row.entity == EntityKind::Edge
                        ? body.edges[addr].shape_id()
                        : body.solid.shell()
                              .faces()[addr]
                              .shape_id();
                if (!answers || !in_range || !(live == row.shape_id)) {
                    v.census_by_name = false;
                    fail(GateCheck::CensusByName,
                         "the census by name: the " +
                         std::string(row.entity ==
                                     EntityKind::Edge
                                         ? "edge"
                                         : "face") +
                         " row at address " +
                         std::to_string(row.address) + " stamps '" +
                         row.name +
                         "' but the body's entity there does not "
                         "carry it — the name is lost or laundered "
                         "(the row and the body must say the same "
                         "thing about the entity)");
                }
            }
        }

        // The HELD references — the census in movement (ADR-0036
        // D3's classes at the gate: the surviving found, the dead
        // gone with the tavola's own cause).
        for (const auto& held : step.held) {
            if (held.expected != Resolution::Kind::Resolved &&
                held.expected != Resolution::Kind::Vanished) {
                throw std::invalid_argument(
                    "chain_gate: the held reference '" +
                    held.reference.to_string() +
                    "' asks for a class the authored flows do not "
                    "produce — MATCH (reference_resolved) and "
                    "VANISHED (reference_vanished) are the two the "
                    "chain holds; RENAMED and AMBIGUOUS are the "
                    "registered v1 boundary (ADR-0036 D3.2/D3.3: "
                    "machinery, no instance — a boundary, not a "
                    "promise); remedy: hold the reference through a "
                    "step that answers one of the two.");
            }
            const DescentRow* row = row_of(tav, held.reference);
            if (row == nullptr) {
                v.census_by_name = false;
                fail(GateCheck::CensusByName,
                     "the census by name: the held reference '" +
                     held.reference.to_string() +
                     "' finds no row at all in the step's tavola — "
                     "a closure violation (D2's own invariant; the "
                     "tavola is the resolver's floor)");
                continue;
            }
            const bool answers =
                row->entity == EntityKind::Edge
                    ? edge_stamp_answers(body, held.reference)
                    : face_stamp_answers(body, held.reference);
            if (held.expected == Resolution::Kind::Resolved) {
                if (row->relation == Descent::Vanished) {
                    v.census_by_name = false;
                    fail(GateCheck::CensusByName,
                         "the census by name: the held reference '" +
                         row->name +
                         "' is expected to survive "
                         "(reference_resolved) but the tavola books "
                         "it VANISHED — the cause: '" + row->cause +
                         "'");
                } else if (!answers) {
                    v.census_by_name = false;
                    fail(GateCheck::CensusByName,
                         "the census by name: the held reference '" +
                         row->name +
                         "' is expected to survive "
                         "(reference_resolved) but its identity "
                         "does not answer on the step's body — the "
                         "name is lost");
                }
            } else {  // Vanished
                if (row->relation != Descent::Vanished) {
                    v.census_by_name = false;
                    fail(GateCheck::CensusByName,
                         "the census by name: the held reference '" +
                         row->name +
                         "' is expected dead "
                         "(reference_vanished) but the tavola books "
                         "it alive (" + descent_token(row->relation) +
                         ") — the death is not real");
                } else if (answers) {
                    v.census_by_name = false;
                    fail(GateCheck::CensusByName,
                         "the census by name: the held reference '" +
                         row->name +
                         "' is expected dead "
                         "(reference_vanished) but its identity "
                         "still answers on the step's body — the "
                         "death is not real");
                }
            }
        }
    }

    // ------------------------------------------------------------------
    // (5) DRIFT — the digest bit-identical to the re-run's.
    // ------------------------------------------------------------------
    v.digest = chain_digest(body);
    if (step.rerun_digest.empty()) {
        v.not_applicable.push_back(GateCheck::DriftZero);
    } else if (v.digest != step.rerun_digest) {
        v.drift_zero = false;
        // The first divergence, named — the digest is a multi-line
        // answer; the first differing line IS the drift's address.
        std::string mine, theirs;
        std::size_t line = 1;
        std::size_t a = 0, b = 0;
        bool named = false;
        while (a < v.digest.size() || b < step.rerun_digest.size()) {
            const std::size_t ae = v.digest.find('\n', a);
            const std::size_t be =
                step.rerun_digest.find('\n', b);
            mine = v.digest.substr(
                a, (ae == std::string::npos ? v.digest.size() : ae) -
                          a);
            theirs = step.rerun_digest.substr(
                b, (be == std::string::npos
                        ? step.rerun_digest.size()
                        : be) - b);
            if (mine != theirs) {
                named = true;
                break;
            }
            if (ae == std::string::npos || be == std::string::npos)
                break;
            a = ae + 1;
            b = be + 1;
            ++line;
        }
        if (named) {
            fail(GateCheck::DriftZero,
                 "the drift: the step's digest and the re-run's "
                 "differ at line " + std::to_string(line) +
                 " — the step: '" + mine + "', the re-run: '" +
                 theirs + "' (the chain must reproduce "
                 "bit-identically — ADR-0014: the flags, the "
                 "accumulation order, the quantizer)");
        } else {
            fail(GateCheck::DriftZero,
                 "the drift: the step's digest and the re-run's "
                 "differ (one ends where the other continues — the "
                 "chain must reproduce bit-identically, ADR-0014)");
        }
    }

    v.pass = v.failures.empty();
    return v;
}

} // namespace CAD_0::brep
