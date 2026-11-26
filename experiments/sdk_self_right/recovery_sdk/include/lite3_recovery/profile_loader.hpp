#pragma once

#include "lite3_recovery/controller.hpp"
#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace lite3_recovery {

namespace detail {

struct JsonValue {
    enum class Kind { null_v, bool_v, num_v, str_v, arr_v, obj_v };
    Kind kind{Kind::null_v};
    bool b{};
    double num{};
    std::string str;
    std::vector<JsonValue> arr;
    std::map<std::string, JsonValue> obj;

    const JsonValue& at(const std::string& key) const {
        if (kind != Kind::obj_v) throw std::runtime_error("Expected JSON object for key: " + key);
        auto it = obj.find(key);
        if (it == obj.end()) throw std::runtime_error("Missing JSON key: " + key);
        return it->second;
    }
};

class JsonParser {
public:
    explicit JsonParser(const std::string& src) : s_(src) {}

    JsonValue parse() {
        skip_ws();
        auto val = parse_value();
        skip_ws();
        if (pos_ != s_.size()) throw std::runtime_error("Trailing data in JSON");
        return val;
    }

private:
    void skip_ws() {
        while (pos_ < s_.size() && std::isspace(static_cast<unsigned char>(s_[pos_]))) ++pos_;
    }

    char peek() const {
        if (pos_ >= s_.size()) throw std::runtime_error("Unexpected end of JSON");
        return s_[pos_];
    }

    JsonValue parse_value() {
        skip_ws();
        char c = peek();
        if (c == '{') return parse_object();
        if (c == '[') return parse_array();
        if (c == '"') return parse_string();
        if (c == 't' || c == 'f') return parse_bool();
        if (c == 'n') return parse_null();
        if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) return parse_number();
        throw std::runtime_error("Unexpected token in JSON");
    }

    JsonValue parse_object() {
        JsonValue v;
        v.kind = JsonValue::Kind::obj_v;
        ++pos_; // '{'
        skip_ws();
        if (peek() == '}') { ++pos_; return v; }
        while (true) {
            skip_ws();
            auto key = parse_string();
            skip_ws();
            if (peek() != ':') throw std::runtime_error("Expected ':' in JSON object");
            ++pos_;
            v.obj[key.str] = parse_value();
            skip_ws();
            if (peek() == '}') { ++pos_; break; }
            if (peek() != ',') throw std::runtime_error("Expected ',' in JSON object");
            ++pos_;
        }
        return v;
    }

    JsonValue parse_array() {
        JsonValue v;
        v.kind = JsonValue::Kind::arr_v;
        ++pos_; // '['
        skip_ws();
        if (peek() == ']') { ++pos_; return v; }
        while (true) {
            v.arr.push_back(parse_value());
            skip_ws();
            if (peek() == ']') { ++pos_; break; }
            if (peek() != ',') throw std::runtime_error("Expected ',' in JSON array");
            ++pos_;
        }
        return v;
    }

    JsonValue parse_string() {
        if (peek() != '"') throw std::runtime_error("Expected '\"'");
        ++pos_;
        JsonValue v;
        v.kind = JsonValue::Kind::str_v;
        while (pos_ < s_.size() && s_[pos_] != '"') {
            if (s_[pos_] == '\\') {
                ++pos_;
                if (pos_ >= s_.size()) throw std::runtime_error("Invalid escape");
                char e = s_[pos_++];
                if (e == 'n') v.str.push_back('\n');
                else if (e == 't') v.str.push_back('\t');
                else v.str.push_back(e);
            } else {
                v.str.push_back(s_[pos_++]);
            }
        }
        if (pos_ >= s_.size()) throw std::runtime_error("Unterminated string");
        ++pos_;
        return v;
    }

    JsonValue parse_bool() {
        JsonValue v;
        v.kind = JsonValue::Kind::bool_v;
        if (s_.compare(pos_, 4, "true") == 0) { v.b = true; pos_ += 4; return v; }
        if (s_.compare(pos_, 5, "false") == 0) { v.b = false; pos_ += 5; return v; }
        throw std::runtime_error("Invalid boolean");
    }

    JsonValue parse_null() {
        if (s_.compare(pos_, 4, "null") == 0) { pos_ += 4; return JsonValue{}; }
        throw std::runtime_error("Invalid null");
    }

    JsonValue parse_number() {
        std::size_t start = pos_;
        if (s_[pos_] == '-') ++pos_;
        while (pos_ < s_.size() && (std::isdigit(static_cast<unsigned char>(s_[pos_]))
               || s_[pos_] == '.' || s_[pos_] == 'e' || s_[pos_] == 'E'
               || s_[pos_] == '+' || s_[pos_] == '-')) {
            if ((s_[pos_] == '+' || s_[pos_] == '-') && pos_ > start
                && s_[pos_ - 1] != 'e' && s_[pos_ - 1] != 'E') break;
            ++pos_;
        }
        JsonValue v;
        v.kind = JsonValue::Kind::num_v;
        v.num = std::stod(s_.substr(start, pos_ - start));
        if (!std::isfinite(v.num)) throw std::runtime_error("Non-finite number");
        return v;
    }

    const std::string& s_;
    std::size_t pos_{};
};

inline Axis parse_axis(const JsonValue& v) {
    if (v.kind != JsonValue::Kind::arr_v || v.arr.size() != joint_count)
        throw std::runtime_error("Expected 12-element array");
    Axis a{};
    for (std::size_t i = 0; i < joint_count; ++i) {
        if (v.arr[i].kind != JsonValue::Kind::num_v)
            throw std::runtime_error("Expected numeric array element");
        a[i] = v.arr[i].num;
    }
    return a;
}

inline Joints parse_joints(const JsonValue& obj, double kp_scale) {
    const auto pos = parse_axis(obj.at("position"));
    const auto kp = parse_axis(obj.at("kp"));
    const auto kd = parse_axis(obj.at("kd"));
    Joints j{};
    for (std::size_t i = 0; i < joint_count; ++i) {
        j[i] = {pos[i], 0.0, kp[i] * kp_scale, kd[i], 0.0};
    }
    return j;
}

} // namespace detail

inline Profile load_profile_json(const std::string& path, double kp_scale = 1.0,
                                 bool smooth_entry_from_actual = true) {
    if (!within(kp_scale, 0.2, 1.0))
        throw std::runtime_error("kp_scale must be within [0.2, 1.0]");
    std::ifstream ifs(path);
    if (!ifs) throw std::runtime_error("Cannot open profile file: " + path);
    std::ostringstream ss;
    ss << ifs.rdbuf();
    auto root = detail::JsonParser(ss.str()).parse();

    Profile p{};
    const auto& b = root.at("bounds");
    p.bounds.min_position = detail::parse_axis(b.at("min_position"));
    p.bounds.max_position = detail::parse_axis(b.at("max_position"));
    p.bounds.max_speed = detail::parse_axis(b.at("max_speed"));
    p.bounds.max_kp = detail::parse_axis(b.at("max_kp"));
    p.bounds.max_kd = detail::parse_axis(b.at("max_kd"));
    p.bounds.max_effort = detail::parse_axis(b.at("max_effort"));
    p.bounds.max_acceleration = detail::parse_axis(b.at("max_acceleration"));
    p.bounds.max_gain_rate = detail::parse_axis(b.at("max_gain_rate"));

    p.damping = detail::parse_axis(root.at("damping"));
    p.initial_position_tolerance = root.at("initial_position_tolerance").num;
    p.smooth_entry_from_actual = smooth_entry_from_actual;

    const auto& st = root.at("stall");
    p.stall.hold_ms = static_cast<Millis>(st.at("hold_ms").num);
    p.stall.motion_rad = st.at("motion_rad").num;
    p.stall.speed_rad_s = st.at("speed_rad_s").num;
    p.stall.attitude_deg = st.at("attitude_deg").num;

    p.start = detail::parse_joints(root.at("start"), kp_scale);
    const auto& segs = root.at("segments");
    if (segs.kind != detail::JsonValue::Kind::arr_v || segs.arr.empty())
        throw std::runtime_error("Profile segments must be a non-empty array");
    for (const auto& s : segs.arr) {
        Segment seg{};
        seg.duration_ms = static_cast<Millis>(s.at("duration_ms").num);
        seg.target = detail::parse_joints(s, kp_scale);
        seg.expects_progress = s.at("expects_progress").b;
        p.segments.push_back(seg);
    }

    if (!valid_bounds(p.bounds) || !valid_trajectory(p.start, p.segments, p.bounds)
        || !p.stall.valid()) {
        throw std::runtime_error("Profile failed safety bounds/trajectory/stall validation");
    }
    return p;
}

} // namespace lite3_recovery
