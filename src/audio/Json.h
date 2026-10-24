#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace tf::json {

// Minimal JSON document model and parser for the audio manifest (UTF-8 passthrough, no \u surrogate handling).
struct Value {
    enum class Type { Null, Bool, Number, String, Array, Object };
    Type type = Type::Null;
    bool boolean = false;
    double number = 0.0;
    std::string string;
    std::vector<Value> array;
    std::map<std::string, Value> object;

    bool isNull() const { return type == Type::Null; }
    // Member lookup; returns a static null value when this is not an object or the key is absent.
    const Value& operator[](const std::string& key) const;
    double numberOr(double fallback) const { return type == Type::Number ? number : fallback; }
    std::string stringOr(const std::string& fallback) const { return type == Type::String ? string : fallback; }
    bool boolOr(bool fallback) const { return type == Type::Bool ? boolean : fallback; }
};

// Parses text; on error returns false and writes a message with the byte offset.
bool parse(const std::string& text, Value& out, std::string* error = nullptr);

} // namespace tf::json
