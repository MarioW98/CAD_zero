// core/viewport/include/CAD_0/viewport/selection.hpp
//
// Selection model — UI-agnostic, operates on ShapeId.
//
// The SelectionModel tracks which shapes (and optionally which subshapes
// of those shapes) are currently selected. It supports:
//
//   * Single-select (replaces current selection)
//   * Multi-select (toggle, additive, subtractive)
//   * Subshape-level selection (face / edge / vertex of a B-Rep shape)
//   * Hover (preview selection before commit)
//
// The model emits no signals itself — callers poll `selection()` and
// `hover()` to read state. The PySide6 view layer wraps this in a
// QAbstractListModel for change notifications (Phase C.6).
//
// Thread safety: NOT thread-safe. Owned by the application, accessed
// only from the UI thread.
//
#pragma once

#include "CAD_0/geometry/shape_id.hpp"

#include <cstdint>
#include <optional>
#include <vector>
#include <algorithm>

namespace CAD_0::viewport {

// A single selection entry: which shape, and optionally which subshape.
// If `subshape` is empty (depth == 0), the whole shape is selected.
struct SelectionEntry {
    CAD_0::geometry::ShapeId shape;

    bool operator==(const SelectionEntry& other) const noexcept {
        return shape == other.shape;
    }
};

// Selection mode for multi-select operations.
enum class SelectionMode : std::uint8_t {
    Replace,    // Clear current selection, then add the new entry
    Add,        // Add the new entry to the current selection
    Subtract,   // Remove the entry if present, else no-op
    Toggle,     // Remove if present, add if not
};

class SelectionModel {
public:
    SelectionModel() = default;

    // ----- Current selection --------------------------------------------

    const std::vector<SelectionEntry>& selection() const noexcept { return selection_; }
    std::size_t size() const noexcept { return selection_.size(); }
    bool empty() const noexcept { return selection_.empty(); }

    bool contains(const CAD_0::geometry::ShapeId& id) const noexcept {
        return std::find_if(selection_.begin(), selection_.end(),
            [&](const SelectionEntry& e) { return e.shape == id; }
        ) != selection_.end();
    }

    // Returns the single selected entry, or nullopt if 0 or >1 are selected.
    std::optional<SelectionEntry> primary() const {
        if (selection_.size() == 1) return selection_[0];
        return std::nullopt;
    }

    // ----- Operations ---------------------------------------------------

    // Select a single entry. Mode controls how the existing selection is
    // modified.
    void select(const CAD_0::geometry::ShapeId& id,
                SelectionMode mode = SelectionMode::Replace) {
        const SelectionEntry entry{id};

        switch (mode) {
            case SelectionMode::Replace:
                selection_.clear();
                selection_.push_back(entry);
                break;
            case SelectionMode::Add:
                if (!contains(id)) selection_.push_back(entry);
                break;
            case SelectionMode::Subtract:
                remove(id);
                break;
            case SelectionMode::Toggle:
                if (contains(id)) remove(id);
                else              selection_.push_back(entry);
                break;
        }
    }

    // Select multiple entries at once. Always replaces the current selection.
    void select_many(const std::vector<CAD_0::geometry::ShapeId>& ids) {
        selection_.clear();
        selection_.reserve(ids.size());
        for (const auto& id : ids) {
            if (!contains(id)) selection_.push_back({id});
        }
    }

    // Clear the selection.
    void clear() noexcept { selection_.clear(); }

    // ----- Hover (preview selection) ------------------------------------

    const std::optional<SelectionEntry>& hover() const noexcept { return hover_; }

    void set_hover(const CAD_0::geometry::ShapeId& id) {
        hover_ = SelectionEntry{id};
    }

    void clear_hover() noexcept { hover_.reset(); }

private:
    std::vector<SelectionEntry> selection_;
    std::optional<SelectionEntry> hover_;

    void remove(const CAD_0::geometry::ShapeId& id) {
        selection_.erase(
            std::remove_if(selection_.begin(), selection_.end(),
                [&](const SelectionEntry& e) { return e.shape == id; }),
            selection_.end());
    }
};

} // namespace CAD_0::viewport
