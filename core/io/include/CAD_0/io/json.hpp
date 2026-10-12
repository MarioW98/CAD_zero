// core/io/include/CAD_0/io/json.hpp
//
// Minimal JSON value model, parser and writer — in-house (ADR-0003:
// no new third-party dependencies without a companion ADR).
//
// Design goals for the native .CAD_0 format (ADR-0007):
//   * Deterministic output — objects preserve insertion order so that
//     golden-file tests are byte-stable.
//   * Float round-trip — numbers are stored as double; all format numbers
//     originate from float32, and the writer emits them with %.9g, which
//     guarantees bit-exact float32 -> text -> float32 round-trips.
//   * Explicit errors — the parser reports line/column on failure; the
//     native layer surfaces unknown/unsupported fields instead of
//     silently dropping them (ADR-0007: "surfaces unsupported features").
//
// Supported JSON: the strict subset (RFC 8259) — objects, arrays, strings,
// numbers, true/false/null. UTF-8 in, UTF-8 out; \uXXXX escapes handled
// on parse, minimal escaping on write.
//
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace CAD_0::io::json {

class JsonValue;
using JsonArray = std::vector<JsonValue>;

// Insertion-ordered object: key/value pairs in write order.
using JsonObject = std::vector<std::pair<std::string, JsonValue>>;

class JsonValue {
public:
    using Null = std::monostate;

    JsonValue() = default;
    JsonValue(std::nullptr_t) {}
    JsonValue(bool b) : v_(b) {}
    JsonValue(double d) : v_(d) {}
    JsonValue(int i) : v_(static_cast<double>(i)) {}
    JsonValue(std::string s) : v_(std::move(s)) {}
    JsonValue(std::string_view s) : v_(std::string(s)) {}
    JsonValue(const char* s) : v_(std::string(s ? s : "")) {}
    JsonValue(JsonArray a) : v_(std::move(a)) {}
    JsonValue(JsonObject o) : v_(std::move(o)) {}

    // ---- type queries ----
    bool is_null()   const noexcept { return std::holds_alternative<Null>(v_); }
    bool is_bool()   const noexcept { return std::holds_alternative<bool>(v_); }
    bool is_number() const noexcept { return std::holds_alternative<double>(v_); }
    bool is_string() const noexcept { return std::holds_alternative<std::string>(v_); }
    bool is_array()  const noexcept { return std::holds_alternative<JsonArray>(v_); }
    bool is_object() const noexcept { return std::holds_alternative<JsonObject>(v_); }

    // ---- accessors (unchecked; use the is_* queries first) ----
    bool        as_bool()   const { return std::get<bool>(v_); }
    double      as_number() const { return std::get<double>(v_); }
    std::string as_string() const { return std::get<std::string>(v_); }
    int         as_int()    const { return static_cast<int>(std::get<double>(v_)); }

    const JsonArray&  as_array()  const { return std::get<JsonArray>(v_); }
    JsonArray&        as_array()        { return std::get<JsonArray>(v_); }
    const JsonObject& as_object() const { return std::get<JsonObject>(v_); }
    JsonObject&       as_object()       { return std::get<JsonObject>(v_); }

    // ---- object helpers ----
    const JsonValue* find(std::string_view key) const noexcept {
        if (!is_object()) return nullptr;
        for (const auto& [k, val] : as_object())
            if (k == key) return &val;
        return nullptr;
    }
    // Checked lookup: throws std::runtime_error with an explicit message
    // naming the missing field (surfaced to the user by the native layer).
    const JsonValue& at(std::string_view key) const;

    // Array element access.
    const JsonValue& at(std::size_t index) const;

    // Object builder: returns *this for chaining.
    JsonValue& set(std::string key, JsonValue value) {
        if (!is_object()) v_ = JsonObject{};
        as_object().emplace_back(std::move(key), std::move(value));
        return *this;
    }
    JsonValue& push_back(JsonValue value) {
        if (!is_array()) v_ = JsonArray{};
        as_array().push_back(std::move(value));
        return *this;
    }

    // ---- number formatting for the writer ----
    // How many significant digits to use when writing a number.
    // 9 digits guarantee exact float32 round-trips (the native format
    // only ever stores float32-originated numbers). Exposed for tests.
    static std::string format_number(double d);

    // --- parsing / writing (implemented in json.cpp) ---

    // Parse strict JSON. Throws std::runtime_error with "line L, column C"
    // information on malformed input.
    static JsonValue parse(std::string_view text);

    // Serialize. `pretty` adds newlines + 2-space indentation (used for
    // the .CAD_0 native file so golden files are diffable); compact form
    // is single-line.
    std::string dump(bool pretty = true) const;

    // Version of dump() that starts at a given indent level (internal).
    void dump_to(std::string& out, bool pretty, int depth) const;

private:
    std::variant<Null, bool, double, std::string, JsonArray, JsonObject> v_;
};

// ---- Parser (recursive descent, strict JSON) -------------------------------
// Exposed for tests and for the native layer's migration entry point.

// Maximum container nesting (objects/arrays) accepted by the parser.
// Input nested deeper than this is rejected with an explicit "too deeply
// nested" error instead of recursing on: the parser is recursive descent,
// so an unbounded ~100k-deep file would stack-overflow (SIGSEGV) before
// any error handling could run. 256 is far above any legitimate .CAD_0
// document (typical depth: < 15) while leaving orders of magnitude of
// stack headroom. Format-neutral: this bounds the PARSER, not the format.
inline constexpr int kMaxNestingDepth = 256;

class Parser {
public:
    explicit Parser(std::string_view text) : text_(text) {}
    JsonValue parse_document(); // throws std::runtime_error on failure

private:
    std::string_view text_;
    std::size_t pos_{0};
    std::size_t line_{1};
    std::size_t col_{1};

    [[noreturn]] void fail(const std::string& message) const;
    char peek() const;
    char advance();
    bool at_end() const noexcept { return pos_ >= text_.size(); }
    void skip_whitespace();
    void expect(char c);

    // `depth` is the nesting level of the value being parsed (root = 0);
    // parse_value rejects depth > kMaxNestingDepth with an explicit error.
    JsonValue parse_value(int depth);
    JsonValue parse_object(int depth);
    JsonValue parse_array(int depth);
    std::string parse_string();
    double parse_number();
    void parse_literal(std::string_view word); // consume exact word, throw on mismatch
    static void append_utf8(std::string& out, std::uint32_t cp);
    std::uint32_t parse_hex4();
};

// Escape/unescape helpers (used by parser & writer; exposed for tests).
std::string escape_string(const std::string& s);

} // namespace CAD_0::io::json
