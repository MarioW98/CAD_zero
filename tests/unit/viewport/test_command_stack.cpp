// tests/unit/viewport/test_command_stack.cpp
//
// Tests for the CommandStack (undo/redo) class.
//
#include <doctest/doctest.h>

#include "CAD_0/viewport/command_stack.hpp"

#include <memory>
#include <string>

using namespace CAD_0::viewport;

namespace {

// A test command that increments/decrements a counter.
class CounterCommand : public Command {
public:
    CounterCommand(int& target, int delta, const char* desc = "counter")
        : target_(target), delta_(delta), desc_(desc) {}

    void execute() override { target_ += delta_; }
    void undo() override { target_ -= delta_; }
    const char* description() const override { return desc_; }

private:
    int& target_;
    int delta_;
    const char* desc_;
};

} // namespace

TEST_CASE("CommandStack: default construction is empty") {
    CommandStack stack;
    CHECK_FALSE(stack.can_undo());
    CHECK_FALSE(stack.can_redo());
    CHECK(stack.undo_count() == 0);
    CHECK(stack.redo_count() == 0);
}

TEST_CASE("CommandStack: push executes the command") {
    int counter = 0;
    CommandStack stack;
    stack.push(std::make_unique<CounterCommand>(counter, 5));
    CHECK(counter == 5);
    CHECK(stack.can_undo());
    CHECK_FALSE(stack.can_redo());
}

TEST_CASE("CommandStack: undo reverses the command") {
    int counter = 0;
    CommandStack stack;
    stack.push(std::make_unique<CounterCommand>(counter, 7));
    CHECK(counter == 7);
    CHECK(stack.undo());
    CHECK(counter == 0);
    CHECK_FALSE(stack.can_undo());
    CHECK(stack.can_redo());
}

TEST_CASE("CommandStack: redo re-applies the command") {
    int counter = 0;
    CommandStack stack;
    stack.push(std::make_unique<CounterCommand>(counter, 7));
    stack.undo();
    CHECK(counter == 0);
    CHECK(stack.redo());
    CHECK(counter == 7);
    CHECK(stack.can_undo());
    CHECK_FALSE(stack.can_redo());
}

TEST_CASE("CommandStack: push clears the redo stack") {
    int counter = 0;
    CommandStack stack;
    stack.push(std::make_unique<CounterCommand>(counter, 1));
    stack.push(std::make_unique<CounterCommand>(counter, 2));
    stack.undo();  // counter back to 1, redo has the +2
    CHECK(stack.can_redo());
    CHECK(counter == 1);

    // Pushing a new command should clear the redo stack.
    stack.push(std::make_unique<CounterCommand>(counter, 10));
    CHECK_FALSE(stack.can_redo());
    CHECK(counter == 11);
}

TEST_CASE("CommandStack: multiple undos in order") {
    int counter = 0;
    CommandStack stack;
    stack.push(std::make_unique<CounterCommand>(counter, 1));
    stack.push(std::make_unique<CounterCommand>(counter, 2));
    stack.push(std::make_unique<CounterCommand>(counter, 3));
    CHECK(counter == 6);

    stack.undo();  // undo the +3
    CHECK(counter == 3);
    stack.undo();  // undo the +2
    CHECK(counter == 1);
    stack.undo();  // undo the +1
    CHECK(counter == 0);
    CHECK_FALSE(stack.can_undo());
}

TEST_CASE("CommandStack: multiple redos in order") {
    int counter = 0;
    CommandStack stack;
    stack.push(std::make_unique<CounterCommand>(counter, 1));
    stack.push(std::make_unique<CounterCommand>(counter, 2));
    stack.push(std::make_unique<CounterCommand>(counter, 3));
    stack.undo();
    stack.undo();
    stack.undo();
    CHECK(counter == 0);

    stack.redo();  // redo +1
    CHECK(counter == 1);
    stack.redo();  // redo +2
    CHECK(counter == 3);
    stack.redo();  // redo +3
    CHECK(counter == 6);
    CHECK_FALSE(stack.can_redo());
}

TEST_CASE("CommandStack: undo returns false when empty") {
    CommandStack stack;
    CHECK_FALSE(stack.undo());
    CHECK_FALSE(stack.can_undo());
}

TEST_CASE("CommandStack: redo returns false when empty") {
    CommandStack stack;
    CHECK_FALSE(stack.redo());
    CHECK_FALSE(stack.can_redo());
}

TEST_CASE("CommandStack: description exposed for menu") {
    int counter = 0;
    CommandStack stack;
    stack.push(std::make_unique<CounterCommand>(counter, 1, "add 1"));
    stack.push(std::make_unique<CounterCommand>(counter, 2, "add 2"));

    const char* desc = stack.next_undo_description();
    REQUIRE(desc != nullptr);
    CHECK(std::string(desc) == "add 2");

    stack.undo();
    desc = stack.next_redo_description();
    REQUIRE(desc != nullptr);
    CHECK(std::string(desc) == "add 2");

    desc = stack.next_undo_description();
    REQUIRE(desc != nullptr);
    CHECK(std::string(desc) == "add 1");
}

TEST_CASE("CommandStack: next_undo/redo_description is null when empty") {
    CommandStack stack;
    CHECK(stack.next_undo_description() == nullptr);
    CHECK(stack.next_redo_description() == nullptr);
}

TEST_CASE("CommandStack: clear empties both stacks") {
    int counter = 0;
    CommandStack stack;
    stack.push(std::make_unique<CounterCommand>(counter, 1));
    stack.push(std::make_unique<CounterCommand>(counter, 2));
    stack.undo();  // redo stack now has 1 entry

    stack.clear();
    CHECK_FALSE(stack.can_undo());
    CHECK_FALSE(stack.can_redo());
    CHECK(stack.undo_count() == 0);
    CHECK(stack.redo_count() == 0);
}

TEST_CASE("CommandStack: max_size limits undo history") {
    int counter = 0;
    CommandStack stack(3);  // keep only 3 commands
    for (int i = 0; i < 5; ++i) {
        stack.push(std::make_unique<CounterCommand>(counter, 1));
    }
    CHECK(counter == 5);
    CHECK(stack.undo_count() == 3);  // only the last 3 are kept

    // Undo 3 times — should bring counter to 5 - 3 = 2 (oldest 2 lost).
    stack.undo();
    stack.undo();
    stack.undo();
    CHECK(counter == 2);
    CHECK_FALSE(stack.can_undo());
}

TEST_CASE("CommandStack: lambda command overload") {
    int value = 0;
    CommandStack stack;
    stack.push(
        [&value]() { value = 42; },
        [&value]() { value = 0; },
        "set value to 42"
    );
    CHECK(value == 42);
    CHECK(stack.can_undo());
    CHECK(std::string(stack.next_undo_description()) == "set value to 42");

    stack.undo();
    CHECK(value == 0);
    stack.redo();
    CHECK(value == 42);
}

TEST_CASE("CommandStack: set_max_size trims existing stack") {
    int counter = 0;
    CommandStack stack;
    for (int i = 0; i < 5; ++i) {
        stack.push(std::make_unique<CounterCommand>(counter, 1));
    }
    CHECK(stack.undo_count() == 5);
    stack.set_max_size(2);
    CHECK(stack.undo_count() == 2);
}
