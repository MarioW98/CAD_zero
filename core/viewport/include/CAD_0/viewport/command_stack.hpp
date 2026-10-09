// core/viewport/include/CAD_0/viewport/command_stack.hpp
//
// Command stack — undo/redo support for the CAD_0 viewport.
//
// Classic Command pattern: each user action is wrapped in a Command
// object with `execute()`, `undo()`, and `redo()`. The CommandStack
// maintains two stacks (undo + redo) and exposes:
//
//   * push(cmd)        — execute and push to undo stack, clear redo
//   * undo()           — pop from undo, call undo(), push to redo
//   * redo()           — pop from redo, call redo(), push to undo
//   * can_undo()       — true if undo stack is non-empty
//   * can_redo()       — true if redo stack is non-empty
//   * clear()          — empty both stacks
//   * max_stack_size() — bound to prevent unbounded memory growth
//
// Commands are move-only (they typically own substantial state — e.g.
// a snapshot of the model before/after the operation).
//
// Thread safety: NOT thread-safe. The viewport runs on the UI thread;
// the command stack is owned by the application and accessed only from
// the UI thread.
//
#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace CAD_0::viewport {

// Abstract base class for all undoable commands.
class Command {
public:
    virtual ~Command() = default;

    // Execute the command (called on push and on redo).
    virtual void execute() = 0;

    // Undo the command (called on undo).
    virtual void undo() = 0;

    // Optional: human-readable description for the undo/redo menu.
    // Returns a const reference to a stable string, so callers don't
    // need to worry about lifetime.
    virtual const std::string& description() const {
        static const std::string empty{"command"};
        return empty;
    }
};

// A command backed by two lambdas (execute_fn, undo_fn).
// Useful for simple commands that don't need a full class hierarchy.
class LambdaCommand final : public Command {
public:
    LambdaCommand(std::function<void()> execute_fn,
                  std::function<void()> undo_fn,
                  std::string desc = "command")
        : execute_fn_(std::move(execute_fn)),
          undo_fn_(std::move(undo_fn)),
          desc_(std::move(desc)) {}

    void execute() override { execute_fn_(); }
    void undo() override { undo_fn_(); }
    const std::string& description() const override { return desc_; }

private:
    std::function<void()> execute_fn_;
    std::function<void()> undo_fn_;
    std::string desc_;
};

// Command stack — owns the undo and redo stacks.
class CommandStack {
public:
    CommandStack() = default;
    explicit CommandStack(std::size_t max_size) : max_size_(max_size) {}

    // Non-copyable (contains unique_ptr<Command> vectors)
    CommandStack(const CommandStack&) = delete;
    CommandStack& operator=(const CommandStack&) = delete;
    // Movable
    CommandStack(CommandStack&&) = default;
    CommandStack& operator=(CommandStack&&) = default;

    // Push a command: execute it, add to undo stack, clear redo stack.
    void push(std::unique_ptr<Command> cmd) {
        if (!cmd) return;
        cmd->execute();
        undo_stack_.push_back(std::move(cmd));
        redo_stack_.clear();
        trim_undo_stack();
    }

    // Convenience overload for LambdaCommand-style usage.
    void push(std::function<void()> execute_fn,
              std::function<void()> undo_fn,
              std::string desc = "command") {
        push(std::make_unique<LambdaCommand>(std::move(execute_fn),
                                              std::move(undo_fn),
                                              std::move(desc)));
    }

    // Undo the last command. Returns true if anything was undone.
    bool undo() {
        if (undo_stack_.empty()) return false;
        auto cmd = std::move(undo_stack_.back());
        undo_stack_.pop_back();
        cmd->undo();
        redo_stack_.push_back(std::move(cmd));
        return true;
    }

    // Redo the last undone command. Returns true if anything was redone.
    bool redo() {
        if (redo_stack_.empty()) return false;
        auto cmd = std::move(redo_stack_.back());
        redo_stack_.pop_back();
        cmd->execute();
        undo_stack_.push_back(std::move(cmd));
        trim_undo_stack();
        return true;
    }

    bool can_undo() const noexcept { return !undo_stack_.empty(); }
    bool can_redo() const noexcept { return !redo_stack_.empty(); }

    std::size_t undo_count() const noexcept { return undo_stack_.size(); }
    std::size_t redo_count() const noexcept { return redo_stack_.size(); }

    // Description of the next command to be undone (or empty string).
    const std::string& next_undo_description() const {
        static const std::string empty{};
        if (undo_stack_.empty()) return empty;
        return undo_stack_.back()->description();
    }

    // Description of the next command to be redone (or empty string).
    const std::string& next_redo_description() const {
        static const std::string empty{};
        if (redo_stack_.empty()) return empty;
        return redo_stack_.back()->description();
    }

    // Clear both stacks. Use this when the model is reset to a fresh state.
    void clear() noexcept {
        undo_stack_.clear();
        redo_stack_.clear();
    }

    // Maximum number of commands retained in the undo stack.
    // When the limit is exceeded, the oldest commands are dropped.
    std::size_t max_size() const noexcept { return max_size_; }
    void set_max_size(std::size_t n) noexcept {
        max_size_ = n;
        trim_undo_stack();
    }

private:
    std::vector<std::unique_ptr<Command>> undo_stack_;
    std::vector<std::unique_ptr<Command>> redo_stack_;
    std::size_t max_size_{1000};

    void trim_undo_stack() noexcept {
        while (undo_stack_.size() > max_size_) {
            undo_stack_.erase(undo_stack_.begin());
        }
    }
};

} // namespace CAD_0::viewport
