#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace tf::json {

// Minimal JSON document model, parser and writer (audio manifest, vehicle definitions, garage setups).
// UTF-8 passthrough, no \u surrogate handling. Object keys are kept sorted (std::map).
struct Value {
    enum class Type { Null, Bool, Number, String, Array, Object };
    Type type = Type::Null;
    bool boolean = false;
    double number = 0.0;
    std::string string;
    std::vector<Value> array;
    std::map<std::string, Value> object;

    Value() = default;
    static Value makeNumber(double v);
    static Value makeBool(bool v);
    static Value makeString(std::string v);
    static Value makeArray();
    static Value makeObject();

    bool isNull() const { return type == Type::Null; }
    bool isObject() const { return type == Type::Object; }
    bool isArray() const { return type == Type::Array; }
    bool isNumber() const { return type == Type::Number; }
    // Member lookup; returns a static null value when this is not an object or the key is absent.
    const Value& operator[](const std::string& key) const;
    bool has(const std::string& key) const { return type == Type::Object && object.count(key) != 0; }
    double numberOr(double fallback) const { return type == Type::Number ? number : fallback; }
    std::string stringOr(const std::string& fallback) const { return type == Type::String ? string : fallback; }
    bool boolOr(bool fallback) const { return type == Type::Bool ? boolean : fallback; }
};

// Parses text; on error returns false and writes a message with the byte offset.
bool parse(const std::string& text, Value& out, std::string* error = nullptr);

// Serializes with two-space indentation; numbers use the shortest form that reads back to the same double.
std::string write(const Value& v);

// Recursive merge: object members of `overlay` replace or extend those of `base`; any other overlay value replaces
// the base value.
void merge(Value& base, const Value& overlay);

} // namespace tf::json
