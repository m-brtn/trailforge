#include "core/Json.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace tf::json {
namespace {

class Parser {
public:
    explicit Parser(const std::string& t) : m_t(t) {}

    bool document(Value& out) {
        skipWs();
        if (!value(out, 0)) return false;
        skipWs();
        if (m_i != m_t.size()) return fail("trailing characters");
        return true;
    }
    std::string error;

private:
    bool fail(const char* what) {
        error = std::string(what) + " at offset " + std::to_string(m_i);
        return false;
    }
    void skipWs() {
        while (m_i < m_t.size() && (m_t[m_i] == ' ' || m_t[m_i] == '\n' || m_t[m_i] == '\r' || m_t[m_i] == '\t')) ++m_i;
    }
    bool literal(const char* word) {
        std::size_t n = 0;
        while (word[n]) ++n;
        if (m_t.compare(m_i, n, word) != 0) return fail("invalid literal");
        m_i += n;
        return true;
    }
    bool string(std::string& out) {
        if (m_t[m_i] != '"') return fail("expected string");
        ++m_i;
        while (m_i < m_t.size() && m_t[m_i] != '"') {
            char c = m_t[m_i++];
            if (c == '\\') {
                if (m_i >= m_t.size()) return fail("bad escape");
                const char e = m_t[m_i++];
                switch (e) {
                case 'n': c = '\n'; break;
                case 't': c = '\t'; break;
                case 'r': c = '\r'; break;
                case 'b': c = '\b'; break;
                case 'f': c = '\f'; break;
                case 'u': {
                    if (m_i + 4 > m_t.size()) return fail("bad \\u escape");
                    const unsigned cp = unsigned(std::strtoul(m_t.substr(m_i, 4).c_str(), nullptr, 16));
                    m_i += 4;
                    // Basic Multilingual Plane only, encoded as UTF-8.
                    if (cp < 0x80) {
                        out += char(cp);
                    } else if (cp < 0x800) {
                        out += char(0xC0 | (cp >> 6));
                        out += char(0x80 | (cp & 0x3F));
                    } else {
                        out += char(0xE0 | (cp >> 12));
                        out += char(0x80 | ((cp >> 6) & 0x3F));
                        out += char(0x80 | (cp & 0x3F));
                    }
                    continue;
                }
                default: c = e; break; // \" \\ \/
                }
            }
            out += c;
        }
        if (m_i >= m_t.size()) return fail("unterminated string");
        ++m_i;
        return true;
    }
    bool value(Value& v, int depth) {
        if (depth > 64) return fail("nesting too deep");
        if (m_i >= m_t.size()) return fail("unexpected end");
        const char c = m_t[m_i];
        if (c == '{') {
            v.type = Value::Type::Object;
            ++m_i;
            skipWs();
            if (m_i < m_t.size() && m_t[m_i] == '}') { ++m_i; return true; }
            for (;;) {
                skipWs();
                std::string key;
                if (m_i >= m_t.size() || !string(key)) return fail("expected key");
                skipWs();
                if (m_i >= m_t.size() || m_t[m_i] != ':') return fail("expected ':'");
                ++m_i;
                skipWs();
                if (!value(v.object[key], depth + 1)) return false;
                skipWs();
                if (m_i < m_t.size() && m_t[m_i] == ',') { ++m_i; continue; }
                if (m_i < m_t.size() && m_t[m_i] == '}') { ++m_i; return true; }
                return fail("expected ',' or '}'");
            }
        }
        if (c == '[') {
            v.type = Value::Type::Array;
            ++m_i;
            skipWs();
            if (m_i < m_t.size() && m_t[m_i] == ']') { ++m_i; return true; }
            for (;;) {
                skipWs();
                v.array.emplace_back();
                if (!value(v.array.back(), depth + 1)) return false;
                skipWs();
                if (m_i < m_t.size() && m_t[m_i] == ',') { ++m_i; continue; }
                if (m_i < m_t.size() && m_t[m_i] == ']') { ++m_i; return true; }
                return fail("expected ',' or ']'");
            }
        }
        if (c == '"') {
            v.type = Value::Type::String;
            return string(v.string);
        }
        if (c == 't') { v.type = Value::Type::Bool; v.boolean = true; return literal("true"); }
        if (c == 'f') { v.type = Value::Type::Bool; v.boolean = false; return literal("false"); }
        if (c == 'n') { v.type = Value::Type::Null; return literal("null"); }
        const char* begin = m_t.c_str() + m_i;
        char* end = nullptr;
        v.number = std::strtod(begin, &end);
        if (end == begin) return fail("unexpected character");
        v.type = Value::Type::Number;
        m_i += std::size_t(end - begin);
        return true;
    }

    const std::string& m_t;
    std::size_t m_i = 0;
};

} // namespace

const Value& Value::operator[](const std::string& key) const {
    static const Value kNull;
    if (type != Type::Object) return kNull;
    const auto it = object.find(key);
    return it != object.end() ? it->second : kNull;
}

bool parse(const std::string& text, Value& out, std::string* error) {
    Parser p(text);
    out = Value{};
    const bool ok = p.document(out);
    if (!ok && error) *error = p.error;
    return ok;
}

Value Value::makeNumber(double v) {
    Value r;
    r.type = Type::Number;
    r.number = v;
    return r;
}
Value Value::makeBool(bool v) {
    Value r;
    r.type = Type::Bool;
    r.boolean = v;
    return r;
}
Value Value::makeString(std::string v) {
    Value r;
    r.type = Type::String;
    r.string = std::move(v);
    return r;
}
Value Value::makeArray() {
    Value r;
    r.type = Type::Array;
    return r;
}
Value Value::makeObject() {
    Value r;
    r.type = Type::Object;
    return r;
}

namespace {

void writeString(std::string& out, const std::string& s) {
    out += '"';
    for (const char c : s) {
        switch (c) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if (static_cast<unsigned char>(c) < 0x20) {
                char buf[8];
                std::snprintf(buf, sizeof(buf), "\\u%04x", unsigned(static_cast<unsigned char>(c)));
                out += buf;
            } else {
                out += c;
            }
        }
    }
    out += '"';
}

void writeNumber(std::string& out, double v) {
    if (!std::isfinite(v)) {
        out += "null";
        return;
    }
    char buf[40];
    for (int precision = 6; precision <= 17; ++precision) {
        std::snprintf(buf, sizeof(buf), "%.*g", precision, v);
        if (std::strtod(buf, nullptr) == v) break;
    }
    out += buf;
}

void writeValue(std::string& out, const Value& v, int indent) {
    const std::string pad(std::size_t(indent) * 2, ' ');
    const std::string padIn(std::size_t(indent + 1) * 2, ' ');
    switch (v.type) {
    case Value::Type::Null: out += "null"; break;
    case Value::Type::Bool: out += v.boolean ? "true" : "false"; break;
    case Value::Type::Number: writeNumber(out, v.number); break;
    case Value::Type::String: writeString(out, v.string); break;
    case Value::Type::Array: {
        bool flat = true;
        for (const Value& e : v.array) flat = flat && (e.type == Value::Type::Number || e.type == Value::Type::Bool);
        if (v.array.empty()) {
            out += "[]";
        } else if (flat) {
            out += '[';
            for (std::size_t i = 0; i < v.array.size(); ++i) {
                if (i) out += ", ";
                writeValue(out, v.array[i], indent + 1);
            }
            out += ']';
        } else {
            out += "[\n";
            for (std::size_t i = 0; i < v.array.size(); ++i) {
                out += padIn;
                writeValue(out, v.array[i], indent + 1);
                out += i + 1 < v.array.size() ? ",\n" : "\n";
            }
            out += pad + "]";
        }
        break;
    }
    case Value::Type::Object: {
        if (v.object.empty()) {
            out += "{}";
            break;
        }
        out += "{\n";
        std::size_t i = 0;
        for (const auto& [key, member] : v.object) {
            out += padIn;
            writeString(out, key);
            out += ": ";
            writeValue(out, member, indent + 1);
            out += ++i < v.object.size() ? ",\n" : "\n";
        }
        out += pad + "}";
        break;
    }
    }
}

} // namespace

std::string write(const Value& v) {
    std::string out;
    writeValue(out, v, 0);
    out += '\n';
    return out;
}

void merge(Value& base, const Value& overlay) {
    if (base.type == Value::Type::Object && overlay.type == Value::Type::Object) {
        for (const auto& [key, member] : overlay.object) merge(base.object[key], member);
        return;
    }
    base = overlay;
}

} // namespace tf::json
