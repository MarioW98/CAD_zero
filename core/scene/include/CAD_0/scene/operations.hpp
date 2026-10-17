// core/scene/include/CAD_0/scene/operations.hpp
//
// The Phase F document layer over the Scene (ADR-0027 D2): the DAG
// rebuild pass and the dangling-reference reporting.
//
// Ownership of the decisions, stated once:
//
//   * The recipe is the SOURCE OF TRUTH of an operation node; the
//     materialized body is derived state (materialize_operation in
//     scene.hpp — THE pure function of (recipe, operand bodies)).
//   * rebuild_operations() re-materializes every operation node from
//     the CURRENT operand bodies in topological order. The document
//     layer calls it after any mutation that can change an operand
//     body (param edit, node delete, node restore) — the same pass
//     serves edits, dangling detection and recovery.
//   * A dangling reference is VISIBLE, never silent (ADR-0005's
//     generation rule at feature granularity): the node keeps its
//     last materialized body (display continues), flags
//     operand_missing, and operation_error() carries the actionable
//     message. NOTHING ever re-picks a reference — recovery happens
//     when the operand returns WITH ITS ORIGINAL ID (the LIFO command
//     stack guarantees the order in-app).
//   * Naming operands by geometric signature is FORBIDDEN here
//     (ADR-0005's documented wrong turn; ADR-0027 D2 violation
//     clause): references are FeatureIds, nothing else.
//
#pragma once

#include "CAD_0/scene/scene.hpp"

#include <cstddef>
#include <functional>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace CAD_0::scene {

// What one rebuild_operations() pass did.
struct OperationRebuildResult {
    // Operation nodes whose body was re-materialized this pass
    // (operand edits propagate; an unchanged input re-materializes
    // deterministically to the same body — the pass is idempotent).
    std::size_t rebuilt{0};

    // Operation nodes currently dangling (an operand id has no live
    // node). Their last materialized body is retained for display.
    std::vector<std::uint32_t> dangling;
};

namespace detail {

// The rebuild core, shared by the in-memory pass (rebuild_operations)
// and the loader's pass 2 (io/native.cpp: rebuild from the recipe,
// catching stale caches — ADR-0027 D4). `nodes` addresses EVERY node
// of the document (lookup set); `notify` fires per re-materialized or
// state-changed node (the Scene wrapper routes it to mark_dirty; the
// loader passes a no-op — no scene exists yet).
inline OperationRebuildResult rebuild_operations_core(
    const std::vector<SceneNode*>& nodes,
    const std::function<void(std::uint32_t)>& notify) {
    OperationRebuildResult result;

    auto find = [&nodes](std::uint32_t id) -> SceneNode* {
        for (SceneNode* n : nodes)
            if (n->id == id) return n;
        return nullptr;
    };

    // Operation nodes in document order.
    std::vector<std::uint32_t> ops;
    for (const SceneNode* n : nodes)
        if (n->operation) ops.push_back(n->id);
    if (ops.empty()) return result;

    // Topological order over the operation subgraph (Kahn). Edges run
    // operand-operation -> dependent-operation; a self-reference
    // (target/tool == own id) contributes a self-loop and is caught by
    // the same cycle check.
    std::map<std::uint32_t, std::size_t> index;  // op id -> ops index
    for (std::size_t i = 0; i < ops.size(); ++i) index.emplace(ops[i], i);

    std::vector<std::vector<std::size_t>> dependents(ops.size());
    std::vector<std::size_t> indegree(ops.size(), 0);
    for (std::size_t i = 0; i < ops.size(); ++i) {
        const SceneNode* n = find(ops[i]);
        const OperationRecipe& r = *n->operation;
        for (const std::uint32_t operand : {r.target, r.tool}) {
            const auto it = index.find(operand);
            if (it == index.end()) continue;  // not an operation: ready
            dependents[it->second].push_back(i);
            ++indegree[i];  // includes the self-loop case
        }
    }

    std::vector<std::size_t> ready;
    for (std::size_t i = 0; i < ops.size(); ++i)
        if (indegree[i] == 0) ready.push_back(i);
    std::vector<std::size_t> order;
    order.reserve(ops.size());
    while (!ready.empty()) {
        const std::size_t i = ready.back();
        ready.pop_back();
        order.push_back(i);
        for (const std::size_t d : dependents[i])
            if (--indegree[d] == 0) ready.push_back(d);
    }
    if (order.size() != ops.size()) {
        // Cycle: name the ids still stuck (they reference each other).
        std::string stuck;
        for (std::size_t i = 0; i < ops.size(); ++i)
            if (indegree[i] > 0)
                stuck += (stuck.empty() ? "" : ", ") + std::to_string(ops[i]);
        throw std::runtime_error(
            "rebuild_operations: cyclic operation references (node ids " +
            stuck + ") - an operation must reference its operands, never "
            "form a cycle (ADR-0027 D2; the file is structurally corrupt)");
    }

    // Materialize in topological order. Operands are looked up by id
    // AFTER their own re-materialization, so chains propagate in one
    // pass.
    for (const std::size_t i : order) {
        const std::uint32_t id = ops[i];
        SceneNode* n = find(id);
        const OperationRecipe& r = *n->operation;
        const SceneNode* t = find(r.target);
        const SceneNode* w = find(r.tool);

        if (t == nullptr || w == nullptr) {
            // Dangling — visible, never silent: keep the last good body
            // (for the loader: the file body — exactly the cached
            // last-good state ADR-0027 D4 defines), flag it, report it.
            // No rebind, no guess, no geometric-signature fallback.
            if (!n->operand_missing) {
                n->operand_missing = true;
                if (notify) notify(id);
            }
            result.dangling.push_back(id);
            continue;
        }

        n->body = materialize_operation(r, t->body, w->body);
        n->mesh_dirty = true;
        n->operand_missing = false;
        if (notify) notify(id);
        ++result.rebuilt;
    }
    return result;
}

} // namespace detail

// The DAG rebuild pass (ADR-0027 D2). For every node carrying an
// OperationRecipe, re-materialize its body from the CURRENT operand
// bodies in topological order (an operand that is itself an operation
// is materialized first). Throws std::runtime_error — with the ids
// involved — on CYCLIC references: impossible to build in-app (an
// operation is created from live operands, so it is strictly newer
// than both), reachable only through a corrupt or hand-edited file,
// and exactly as corrupt as it sounds. A MISSING operand is not an
// error: it is the visible dangling state (see above).
inline OperationRebuildResult rebuild_operations(Scene& s) {
    std::vector<SceneNode*> nodes;
    nodes.reserve(s.node_count());
    for (std::size_t i = 0; i < s.node_count(); ++i)
        nodes.push_back(s.node_at(i));
    return detail::rebuild_operations_core(
        nodes, [&s](std::uint32_t id) { s.mark_dirty(id); });
}

// The actionable dangling message for the property panel / tree
// (ADR-0027 D2): empty when the node is not a dangling operation.
// Names the missing operand by id and role, and the two remedies the
// state actually admits (undo the deletion — the operand returns with
// its original id — or delete the operation). Never a suggestion to
// "pick another operand": references are never re-picked.
inline std::string operation_error(const Scene& s, std::uint32_t id) {
    const SceneNode* n = s.get_node(id);
    if (n == nullptr || !n->operation || !n->operand_missing) return {};

    const OperationRecipe& r = *n->operation;
    const bool target_missing = s.get_node(r.target) == nullptr;
    const bool tool_missing = s.get_node(r.tool) == nullptr;
    std::string out = "operand ";
    if (target_missing)
        out += "#" + std::to_string(r.target) + " (target)";
    if (target_missing && tool_missing) out += " and ";
    if (tool_missing) out += "#" + std::to_string(r.tool) + " (tool)";
    out += " was deleted - undo its deletion, or delete this operation "
           "(references are never re-picked automatically)";
    return out;
}

// The loader's pass-2 entry (ADR-0027 D4): the SAME DAG protocol over
// a bare node list — the parsed document, pre-install. Operands that
// exist re-materialize from the recipe (deterministic; catches stale
// cached bodies); a missing operand keeps the FILE body and enters
// the error state; cycles/self-references throw the explicit error.
inline OperationRebuildResult rebuild_document_operations(
    std::vector<SceneNode>& nodes) {
    std::vector<SceneNode*> ptrs;
    ptrs.reserve(nodes.size());
    for (SceneNode& n : nodes) ptrs.push_back(&n);
    return detail::rebuild_operations_core(ptrs, nullptr);
}

} // namespace CAD_0::scene
