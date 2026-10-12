// core/io/src/json.cpp
//
// Implementation of the minimal JSON value model (see json.hpp).
//
#include "CAD_0/io/json.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace CAD_0::io::json {

// ---- JsonValue helpers -----------------------------------------------------

const JsonValue& JsonValue::at(std::string_view key) const {
    const JsonValue* v = find(key);
    if (!v) {
        throw std::runtime_error("missing required field \"" + std::string(key) + "\"");
    }
    return *v;
}

const JsonValue& JsonValue::at(std::size_t index) const {
    if (!is_array())
        throw std::runtime_error("indexed access on non-array value");
    const auto& arr = as_array();
    if (index >= arr.size())
        throw std::runtime_error("array index " + std::to_string(index) +
                                 " out of bounds (size " + std::to_string(arr.size()) + ")");
    return arr[index];
}

std::string JsonValue::format_number(double d) {
    // Shortest-round-trip formatting for float32-originated numbers: try
    // increasing precision until the text parses back to the exact same
    // float32 value. Gives clean output ("0.1", "2.5", "1") while
    // guaranteeing bit-exact float round-trips. 9 significant digits
    // always suffice for float32 (the fallback).
    const float f = static_cast<float>(d);
    char buf[64];
    for (int prec = 1; prec <= 9; ++prec) {
        std::snprintf(buf, sizeof(buf), "%.*g", prec, d);
        if (static_cast<float>(std::strtod(buf, nullptr)) == f)
            return std::string(buf);
    }
    // Fallback (never reached for float32 inputs).
    std::snprintf(buf, sizeof(buf), "%.17g", d);
    return std::string(buf);
}

std::string JsonValue::dump(bool pretty) const {
    std::string out;
    out.reserve(256);
    dump_to(out, pretty, 0);
    return out;
}

void JsonValue::dump_to(std::string& out, bool pretty, int depth) const {
    const std::string indent(pretty ? depth * 2 : 0, ' ');
    const std::string indent1(pretty ? (depth + 1) * 2 : 0, ' ');

    switch (v_.index()) {
    case 0: out += "null"; break; // Null
    case 1: out += (std::get<bool>(v_) ? "true" : "false"); break;
    case 2: out += format_number(std::get<double>(v_)); break;
    case 3: out += escape_string(std::get<std::string>(v_)); break;
    case 4: { // JsonArray
        const auto& arr = std::get<JsonArray>(v_);
        if (arr.empty()) { out += "[]"; break; }
        out += '[';
        for (std::size_t i = 0; i < arr.size(); ++i) {
            if (i) out += ',';
            if (pretty) { out += '\n'; out += indent1; }
            arr[i].dump_to(out, pretty, depth + 1);
        }
        if (pretty) { out += '\n'; out += indent; }
        out += ']';
        break;
    }
    case 5: { // JsonObject
        const auto& obj = std::get<JsonObject>(v_);
        if (obj.empty()) { out += "{}"; break; }
        out += '{';
        for (std::size_t i = 0; i < obj.size(); ++i) {
            if (i) out += ',';
            if (pretty) { out += '\n'; out += indent1; }
            out += escape_string(obj[i].first);
            out += pretty ? ": " : ":";
            obj[i].second.dump_to(out, pretty, depth + 1);
        }
        if (pretty) { out += '\n'; out += indent; }
        out += '}';
        break;
    }
    default: out += "null"; break;
    }
}

// ---- string escaping --------------------------------------------------------

std::string escape_string(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 2);
    out += '"';
    for (unsigned char c : s) {
        switch (c) {
        case '"':  out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        case '\b': out += "\\b"; break;
        case '\f': out += "\\f"; break;
        default:
            if (c < 0x20) {
                char buf[8];
                std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                out += buf;
            } else {
                out += static_cast<char>(c); // UTF-8 passthrough
            }
        }
    }
    out += '"';
    return out;
}

// ---- Parser -----------------------------------------------------------------

char Parser::peek() const {
    if (at_end()) fail("unexpected end of input");
    return text_[pos_];
}

char Parser::advance() {
    if (at_end()) fail("unexpected end of input");
    const char c = text_[pos_++];
    if (c == '\n') { ++line_; col_ = 1; } else { ++col_; }
    return c;
}

void Parser::fail(const std::string& message) const {
    throw std::runtime_error("JSON parse error at line " + std::to_string(line_) +
                             ", column " + std::to_string(col_) + ": " + message);
}

void Parser::skip_whitespace() {
    while (!at_end()) {
        const char c = text_[pos_];
        if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
            advance();
        } else {
            break;
        }
    }
}

void Parser::expect(char c) {
    if (at_end() || text_[pos_] != c)
        fail(std::string("expected '") + c + "'");
    advance();
}

std::uint32_t Parser::parse_hex4() {
    std::uint32_t value = 0;
    for (int i = 0; i < 4; ++i) {
        const char c = advance();
        value <<= 4;
        if (c >= '0' && c <= '9') value |= static_cast<std::uint32_t>(c - '0');
        else if (c >= 'a' && c <= 'f') value |= static_cast<std::uint32_t>(c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') value |= static_cast<std::uint32_t>(c - 'A' + 10);
        else fail("invalid \\u escape");
    }
    return value;
}

void Parser::append_utf8(std::string& out, std::uint32_t cp) {
    if (cp <= 0x7F) {
        out += static_cast<char>(cp);
    } else if (cp <= 0x7FF) {
        out += static_cast<char>(0xC0 | (cp >> 6));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else if (cp <= 0xFFFF) {
        out += static_cast<char>(0xE0 | (cp >> 12));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else {
        out += static_cast<char>(0xF0 | (cp >> 18));
        out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    }
}

std::string Parser::parse_string() {
    expect('"');
    std::string out;
    while (true) {
        if (at_end()) fail("unterminated string");
        const char c = advance();
        if (c == '"') break;
        if (c == '\\') {
            const char e = advance();
            switch (e) {
            case '"':  out += '"';  break;
            case '\\': out += '\\'; break;
            case '/':  out += '/';  break;
            case 'n':  out += '\n'; break;
            case 'r':  out += '\r'; break;
            case 't':  out += '\t'; break;
            case 'b':  out += '\b'; break;
            case 'f':  out += '\f'; break;
            case 'u': {
                std::uint32_t cp = parse_hex4();
                // Surrogate pair?
                if (cp >= 0xD800 && cp <= 0xDBFF) {
                    if (at_end() || text_[pos_] != '\\') fail("missing low surrogate");
                    advance(); // '\'
                    if (at_end() || text_[pos_] != 'u') fail("missing low surrogate");
                    advance(); // 'u'
                    const std::uint32_t lo = parse_hex4();
                    if (lo < 0xDC00 || lo > 0xDFFF) fail("invalid low surrogate");
                    cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                }
                append_utf8(out, cp);
                break;
            }
            default: fail("invalid escape character");
            }
        } else if (static_cast<unsigned char>(c) < 0x20) {
            fail("control character in string");
        } else {
            out += c;
        }
    }
    return out;
}

double Parser::parse_number() {
    const std::size_t start = pos_;
    if (!at_end() && text_[pos_] == '-') advance();
    if (at_end()) fail("unexpected end of number");
    // Integer part: 0 | [1-9][0-9]*
    if (text_[pos_] == '0') {
        advance();
    } else if (text_[pos_] >= '1' && text_[pos_] <= '9') {
        while (!at_end() && text_[pos_] >= '0' && text_[pos_] <= '9') advance();
    } else {
        fail("invalid number");
    }
    // Fraction
    if (!at_end() && text_[pos_] == '.') {
        advance();
        if (at_end() || text_[pos_] < '0' || text_[pos_] > '9') fail("invalid number fraction");
        while (!at_end() && text_[pos_] >= '0' && text_[pos_] <= '9') advance();
    }
    // Exponent
    if (!at_end() && (text_[pos_] == 'e' || text_[pos_] == 'E')) {
        advance();
        if (!at_end() && (text_[pos_] == '+' || text_[pos_] == '-')) advance();
        if (at_end() || text_[pos_] < '0' || text_[pos_] > '9') fail("invalid number exponent");
        while (!at_end() && text_[pos_] >= '0' && text_[pos_] <= '9') advance();
    }
    const std::string num_text(text_.substr(start, pos_ - start));
    try {
        return std::stod(num_text);
    } catch (...) {
        fail("number out of range");
        return 0.0;
    }
}

// Consume an exact literal word ("true" / "false"); throws on mismatch.
void Parser::parse_literal(std::string_view word) {
    for (const char w : word) {
        if (at_end() || text_[pos_] != w)
            fail(std::string("invalid literal, expected '") + std::string(word) + "'");
        advance();
    }
}

JsonValue Parser::parse_value() {
    skip_whitespace();
    if (at_end()) fail("unexpected end of input");
    const char c = peek();
    switch (c) {
    case '{': return parse_object();
    case '[': return parse_array();
    case '"': return JsonValue(parse_string());
    case 't': {
        parse_literal("true");
        return JsonValue(true);
    }
    case 'f': {
        parse_literal("false");
        return JsonValue(false);
    }
    case 'n': {
        parse_literal("null");
        return JsonValue();
    }
    default:
        if (c == '-' || (c >= '0' && c <= '9')) return JsonValue(parse_number());
        fail("unexpected character");
    }
}

JsonValue Parser::parse_object() {
    expect('{');
    JsonObject obj;
    skip_whitespace();
    if (!at_end() && peek() == '}') { advance(); return JsonValue(std::move(obj)); }
    while (true) {
        skip_whitespace();
        std::string key = parse_string();
        skip_whitespace();
        expect(':');
        JsonValue value = parse_value();
        obj.emplace_back(std::move(key), std::move(value));
        skip_whitespace();
        if (at_end()) fail("unterminated object");
        const char c = advance();
        if (c == '}') break;
        if (c != ',') fail("expected ',' or '}' in object");
    }
    return JsonValue(std::move(obj));
}

JsonValue Parser::parse_array() {
    expect('[');
    JsonArray arr;
    skip_whitespace();
    if (!at_end() && peek() == ']') { advance(); return JsonValue(std::move(arr)); }
    while (true) {
        arr.push_back(parse_value());
        skip_whitespace();
        if (at_end()) fail("unterminated array");
        const char c = advance();
        if (c == ']') break;
        if (c != ',') fail("expected ',' or ']' in array");
    }
    return JsonValue(std::move(arr));
}

JsonValue Parser::parse_document() {
    JsonValue root = parse_value();
    skip_whitespace();
    if (!at_end())
        fail("trailing content after document");
    return root;
}

JsonValue JsonValue::parse(std::string_view text) {
    Parser p(text);
    return p.parse_document();
}

} // namespace CAD_0::io::json
