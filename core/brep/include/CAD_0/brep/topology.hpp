// core/brep/include/CAD_0/brep/topology.hpp
//
// B-Rep topology — radial-edge structure (Weiler 1986, see ADR-0012).
//
// The full entity set, ownership rules, ID scheme, radial-edge population
// contract, and Euler invariant are documented in ADR-0021. This header
// is the canonical declaration; the implementation of the post-pass
// helpers lives in core/brep/src/topology.cpp.
//
#pragma once

#include "CAD_0/math/vec.hpp"
#include "CAD_0/math/bbox.hpp"
#include "CAD_0/math/tolerance.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace CAD_0::brep {

class Vertex;
class Edge;
class CoEdge;
class Loop;
class Face;
class Shell;
class Solid;
class BRepBody;

using VertexId = std::uint64_t;
using EdgeId = std::uint64_t;
using FaceId = std::uint64_t;
using ShellId = std::uint64_t;

// Vertex — a 3D point with an ID
class Vertex {
public:
    Vertex() = default;
    explicit Vertex(math::Vec3f pos) : position_(pos) {}
    const math::Vec3f& position() const noexcept { return position_; }
    void set_position(math::Vec3f p) noexcept { position_ = p; }
    VertexId id() const noexcept { return id_; }
    void set_id(VertexId id) noexcept { id_ = id; }
private:
    math::Vec3f position_{};
    VertexId id_{0};
};

// Edge — connects two vertices, has radial co-edges.
//
// `coedges_` is the radial-edge list mandated by ADR-0012 §Decision.
// The pointers are **non-owning** (ADR-0021 §3): each CoEdge is owned
// by the Loop that uses it. `link_radial_edges(BRepBody&)` is the only
// sanctioned way to populate this list (ADR-0021 §4).
//
// The list stores `const CoEdge*` because the Edge never mutates the
// CoEdge instances it references — it only reads them (for orientation,
// adjacency queries, future radial walking). The owning Loop retains
// exclusive mutation rights.
class Edge {
public:
    Edge() = default;
    Edge(VertexId v0, VertexId v1) : v_{v0, v1} {}
    VertexId vertex(int i) const noexcept { return v_[i]; }
    void set_vertex(int i, VertexId vid) noexcept { v_[i] = vid; }
    EdgeId id() const noexcept { return id_; }
    void set_id(EdgeId id) noexcept { id_ = id; }
    const std::vector<const CoEdge*>& coedges() const noexcept { return coedges_; }
    void add_coedge(const CoEdge* ce) { coedges_.push_back(ce); }
    void clear_coedges() noexcept { coedges_.clear(); }
    bool is_manifold() const noexcept { return coedges_.size() == 2; }
private:
    VertexId v_[2]{0, 0};
    EdgeId id_{0};
    std::vector<const CoEdge*> coedges_;  // ADR-0012 radial-edge list; ADR-0021 ownership
};

// CoEdge — oriented use of an Edge within a Loop
class CoEdge {
public:
    CoEdge() = default;
    CoEdge(EdgeId edge, bool reversed) : edge_(edge), reversed_(reversed) {}
    EdgeId edge() const noexcept { return edge_; }
    void set_edge(EdgeId e) noexcept { edge_ = e; }
    bool reversed() const noexcept { return reversed_; }
    void set_reversed(bool r) noexcept { reversed_ = r; }
    VertexId start_vertex(const std::vector<Edge>& edges) const;
    VertexId end_vertex(const std::vector<Edge>& edges) const;
private:
    EdgeId edge_{0};
    bool reversed_{false};
};

// Loop — closed sequence of co-edges bounding a face
class Loop {
public:
    Loop() = default;
    const std::vector<CoEdge>& coedges() const noexcept { return coedges_; }
    void add_coedge(CoEdge ce) { coedges_.push_back(std::move(ce)); }
    bool is_outer() const noexcept { return is_outer_; }
    void set_outer(bool outer) noexcept { is_outer_ = outer; }
private:
    std::vector<CoEdge> coedges_;
    bool is_outer_{true};
};

// SurfaceGeometry — abstract base for analytic surfaces
class SurfaceGeometry {
public:
    virtual ~SurfaceGeometry() = default;
    virtual math::Vec3f evaluate(float u, float v) const = 0;
    virtual math::Vec3f normal(float u, float v) const = 0;
    virtual const char* type_name() const = 0;
    virtual math::Bboxf bounds() const = 0;
};

// Face — bounded region of a surface
class Face {
public:
    Face() = default;
    explicit Face(std::shared_ptr<SurfaceGeometry> surf) : surface_(std::move(surf)) {}
    const SurfaceGeometry* surface() const noexcept { return surface_.get(); }
    void set_surface(std::shared_ptr<SurfaceGeometry> surf) { surface_ = std::move(surf); }
    const std::vector<Loop>& loops() const noexcept { return loops_; }
    void add_loop(Loop loop) { loops_.push_back(std::move(loop)); }
    FaceId id() const noexcept { return id_; }
    void set_id(FaceId id) noexcept { id_ = id; }
    bool forward() const noexcept { return forward_; }
    void set_forward(bool f) noexcept { forward_ = f; }
private:
    std::shared_ptr<SurfaceGeometry> surface_;
    std::vector<Loop> loops_;
    FaceId id_{0};
    bool forward_{true};
};

// Shell — set of connected faces
class Shell {
public:
    Shell() = default;
    const std::vector<Face>& faces() const noexcept { return faces_; }
    std::vector<Face>& faces_mut() { return faces_; }
    void add_face(Face face) { faces_.push_back(std::move(face)); }
    ShellId id() const noexcept { return id_; }
    void set_id(ShellId id) noexcept { id_ = id; }
    bool is_closed() const noexcept { return is_closed_; }
    void set_closed(bool c) noexcept { is_closed_ = c; }
private:
    std::vector<Face> faces_;
    ShellId id_{0};
    bool is_closed_{false};
};

// Solid — closed shell forming a volume
class Solid {
public:
    Solid() = default;
    explicit Solid(Shell shell) : shell_(std::move(shell)) {}
    const Shell& shell() const noexcept { return shell_; }
    Shell& shell_mut() { return shell_; }
    void set_shell(Shell shell) { shell_ = std::move(shell); }
private:
    Shell shell_;
};

// BRepBody — public-facing B-Rep body
class BRepBody {
public:
    BRepBody() = default;
    Solid solid;
    std::vector<Vertex> vertices;
    std::vector<Edge> edges;

    std::size_t vertex_count() const noexcept { return vertices.size(); }
    std::size_t edge_count() const noexcept { return edges.size(); }
    std::size_t face_count() const noexcept {
        return solid.shell().faces().size();
    }
    math::Bboxf bounds() const noexcept {
        math::Bboxf b;
        for (const auto& v : vertices) b.expand(v.position());
        return b;
    }
    ~BRepBody() = default;
};

// Inline implementations
inline VertexId CoEdge::start_vertex(const std::vector<Edge>& edges) const {
    if (edge_ >= edges.size()) return 0;
    return reversed_ ? edges[edge_].vertex(1) : edges[edge_].vertex(0);
}
inline VertexId CoEdge::end_vertex(const std::vector<Edge>& edges) const {
    if (edge_ >= edges.size()) return 0;
    return reversed_ ? edges[edge_].vertex(0) : edges[edge_].vertex(1);
}

// ---------------------------------------------------------------------------
// Post-pass helpers — ADR-0021 §4 and §5
// ---------------------------------------------------------------------------

// link_radial_edges — populate Edge::coedges_ from the (Face, Loop, CoEdge)
// hierarchy. Idempotent: clears every edge's radial list before rebuilding.
// Must be called after every body mutation that adds or removes a face
// (factories call it once at the end of construction).
//
// The registered CoEdge* pointers are valid as long as the owning Loop is
// not reallocated (ADR-0021 §3). After this call returns, factories must
// not add further co-edges to existing loops.
void link_radial_edges(BRepBody& body);

// euler_characteristic — χ = V − E + F for the body's single shell.
// For a closed manifold solid (e.g. make_box output) the result is 2.
// Returns a signed 64-bit integer to avoid overflow on large meshes.
//
// Note: a torus represented as one periodic face has χ = 0 + 0 + 1 = 1,
// which is the documented limitation of the MVP (the surface is not yet
// bounded by a real seam wire — Phase D step (b)).
std::int64_t euler_characteristic(const BRepBody& body) noexcept;

// loops_are_closed — for every loop of every face, check that the end
// vertex of co-edge i equals the start vertex of co-edge (i+1) mod n.
// Returns false on the first violation. Used by tests as a regression
// guard (ADR-0021 §6).
bool loops_are_closed(const BRepBody& body) noexcept;

} // namespace CAD_0::brep
