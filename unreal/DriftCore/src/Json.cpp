#include "drift/Json.h"

#include <cstdlib>

namespace drift {

namespace {

struct Parser {
	const std::string& s;
	size_t i = 0;
	std::string err;

	explicit Parser(const std::string& text) : s(text) {}

	void Ws() {
		while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) ++i;
	}
	bool Fail(const char* what) {
		if (err.empty()) err = std::string(what) + " at offset " + std::to_string(i);
		return false;
	}
	bool Lit(const char* word) {
		size_t n = 0;
		while (word[n]) ++n;
		if (s.compare(i, n, word) != 0) return Fail("bad literal");
		i += n;
		return true;
	}
	bool String(std::string& out) {
		if (i >= s.size() || s[i] != '"') return Fail("expected string");
		++i;
		out.clear();
		while (i < s.size() && s[i] != '"') {
			char c = s[i++];
			if (c == '\\') {
				if (i >= s.size()) return Fail("bad escape");
				char e = s[i++];
				switch (e) {
					case 'n': out += '\n'; break;
					case 't': out += '\t'; break;
					case 'r': out += '\r'; break;
					case 'b': out += '\b'; break;
					case 'f': out += '\f'; break;
					case 'u': {
						if (i + 4 > s.size()) return Fail("bad unicode escape");
						unsigned code = static_cast<unsigned>(std::strtoul(s.substr(i, 4).c_str(), nullptr, 16));
						i += 4;
						// Encode as UTF-8 (BMP only; enough for names).
						if (code < 0x80) {
							out += static_cast<char>(code);
						} else if (code < 0x800) {
							out += static_cast<char>(0xC0 | (code >> 6));
							out += static_cast<char>(0x80 | (code & 0x3F));
						} else {
							out += static_cast<char>(0xE0 | (code >> 12));
							out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
							out += static_cast<char>(0x80 | (code & 0x3F));
						}
						break;
					}
					default: out += e; break;
				}
			} else {
				out += c;
			}
		}
		if (i >= s.size()) return Fail("unterminated string");
		++i;
		return true;
	}
	bool Value(JsonValue& v, int depth) {
		if (depth > 64) return Fail("nesting too deep");
		Ws();
		if (i >= s.size()) return Fail("unexpected end");
		char c = s[i];
		if (c == '{') {
			++i;
			v.type = JsonValue::Type::Object;
			Ws();
			if (i < s.size() && s[i] == '}') { ++i; return true; }
			for (;;) {
				Ws();
				std::string key;
				if (!String(key)) return false;
				Ws();
				if (i >= s.size() || s[i] != ':') return Fail("expected ':'");
				++i;
				JsonValue child;
				if (!Value(child, depth + 1)) return false;
				v.obj[key] = std::move(child);
				Ws();
				if (i < s.size() && s[i] == ',') { ++i; continue; }
				if (i < s.size() && s[i] == '}') { ++i; return true; }
				return Fail("expected ',' or '}'");
			}
		}
		if (c == '[') {
			++i;
			v.type = JsonValue::Type::Array;
			Ws();
			if (i < s.size() && s[i] == ']') { ++i; return true; }
			for (;;) {
				JsonValue child;
				if (!Value(child, depth + 1)) return false;
				v.arr.push_back(std::move(child));
				Ws();
				if (i < s.size() && s[i] == ',') { ++i; continue; }
				if (i < s.size() && s[i] == ']') { ++i; return true; }
				return Fail("expected ',' or ']'");
			}
		}
		if (c == '"') {
			v.type = JsonValue::Type::String;
			return String(v.str);
		}
		if (c == 't') { v.type = JsonValue::Type::Bool; v.b = true; return Lit("true"); }
		if (c == 'f') { v.type = JsonValue::Type::Bool; v.b = false; return Lit("false"); }
		if (c == 'n') { v.type = JsonValue::Type::Null; return Lit("null"); }
		// Number.
		const char* start = s.c_str() + i;
		char* end = nullptr;
		double d = std::strtod(start, &end);
		if (end == start) return Fail("bad value");
		i += static_cast<size_t>(end - start);
		v.type = JsonValue::Type::Number;
		v.num = d;
		return true;
	}
};

const JsonValue& NullValue() {
	static const JsonValue kNull;
	return kNull;
}

}  // namespace

const JsonValue& JsonValue::operator[](const std::string& key) const {
	if (type != Type::Object) return NullValue();
	auto it = obj.find(key);
	return it == obj.end() ? NullValue() : it->second;
}

double JsonValue::Num(const std::string& key, double fallback) const {
	const JsonValue& v = (*this)[key];
	if (v.type == Type::Number) return v.num;
	if (v.type == Type::Bool) return v.b ? 1.0 : 0.0;
	return fallback;
}

bool JsonValue::Bool(const std::string& key, bool fallback) const {
	const JsonValue& v = (*this)[key];
	if (v.type == Type::Bool) return v.b;
	if (v.type == Type::Number) return v.num != 0.0;
	return fallback;
}

std::string JsonValue::Str(const std::string& key, const std::string& fallback) const {
	const JsonValue& v = (*this)[key];
	return v.type == Type::String ? v.str : fallback;
}

bool JsonValue::Parse(const std::string& text, JsonValue& out, std::string* err) {
	Parser p(text);
	out = JsonValue();
	bool ok = p.Value(out, 0);
	if (ok) {
		p.Ws();
		if (p.i != text.size()) ok = p.Fail("trailing characters");
	}
	if (!ok && err) *err = p.err;
	return ok;
}

}  // namespace drift
