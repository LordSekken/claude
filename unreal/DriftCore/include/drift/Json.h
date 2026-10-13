// Minimal JSON reader (no exceptions, no RTTI) - enough for car specs.
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace drift {

class JsonValue {
public:
	enum class Type { Null, Bool, Number, String, Array, Object };

	Type type = Type::Null;
	bool b = false;
	double num = 0.0;
	std::string str;
	std::vector<JsonValue> arr;
	std::map<std::string, JsonValue> obj;

	bool IsNull() const { return type == Type::Null; }
	bool IsObject() const { return type == Type::Object; }
	bool IsArray() const { return type == Type::Array; }
	bool IsNumber() const { return type == Type::Number; }

	// Object member or a shared null value.
	const JsonValue& operator[](const std::string& key) const;
	bool Has(const std::string& key) const { return type == Type::Object && obj.count(key) > 0; }
	double Num(const std::string& key, double fallback) const;
	bool Bool(const std::string& key, bool fallback) const;
	std::string Str(const std::string& key, const std::string& fallback) const;

	// Parses text; on failure returns false and fills err (with position).
	static bool Parse(const std::string& text, JsonValue& out, std::string* err = nullptr);
};

}  // namespace drift
