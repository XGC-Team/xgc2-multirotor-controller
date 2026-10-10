// The JSON configuration object of an xgc2-module instance, shared by this repository's modules.
//
// create() and configure() hand a module one UTF-8 JSON object. A key path "a/b" addresses
// {"a": {"b": ...}}, the same spelling as the ROS parameter namespace the ROS nodes read.
// Every key a module reads is remembered, so a key the module never reads (a typo, or a key
// of another module) is reported instead of silently leaving a default in place.
#pragma once

#include <jsoncpp/json/json.h>

#include <cstring>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "xgc2/module.h"

namespace module_support {

class JsonConfig {
   public:
    // Throws std::invalid_argument when the text is not a JSON object (strict JSON: no comments,
    // no duplicate keys, nothing after the object).
    explicit JsonConfig(const xgc2_config* config) {
        if (config == nullptr || config->json == nullptr) {
            throw std::invalid_argument("missing configuration object");
        }
        Json::CharReaderBuilder builder;
        Json::CharReaderBuilder::strictMode(&builder.settings_);
        const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
        std::string errors;
        const char* text = config->json;
        const size_t length = config->length != 0 ? config->length : std::strlen(text);
        if (!reader->parse(text, text + length, &root_, &errors) || !root_.isObject()) {
            throw std::invalid_argument("configuration is not a JSON object: " +
                                        (errors.empty() ? std::string("not an object") : errors));
        }
    }

    // True when `path` is present. Throws std::invalid_argument when its type is not the requested
    // one. Reading a path marks it as used.
    bool read(const std::string& path, bool& out) {
        const Json::Value* value = find(path);
        if (value == nullptr)
            return false;
        if (!value->isBool())
            throw typeError(path, "a boolean");
        out = value->asBool();
        return true;
    }
    bool read(const std::string& path, int& out) {
        const Json::Value* value = find(path);
        if (value == nullptr)
            return false;
        if (!value->isInt())
            throw typeError(path, "an integer");
        out = value->asInt();
        return true;
    }
    bool read(const std::string& path, double& out) {
        const Json::Value* value = find(path);
        if (value == nullptr)
            return false;
        if (!value->isNumeric())
            throw typeError(path, "a number");
        out = value->asDouble();
        return true;
    }
    bool read(const std::string& path, std::string& out) {
        const Json::Value* value = find(path);
        if (value == nullptr)
            return false;
        if (!value->isString())
            throw typeError(path, "a string");
        out = value->asString();
        return true;
    }
    bool read(const std::string& path, std::vector<double>& out) {
        const Json::Value* value = find(path);
        if (value == nullptr)
            return false;
        if (!value->isArray())
            throw typeError(path, "an array of numbers");
        out.clear();
        for (const Json::Value& item : *value) {
            if (!item.isNumeric())
                throw typeError(path, "an array of numbers");
            out.push_back(item.asDouble());
        }
        return true;
    }

    // The first leaf (array, string, number, boolean or null; sorted by path) that was never read,
    // or an empty string when every leaf was.
    std::string firstUnusedKey() const {
        std::set<std::string> leaves;
        collectLeaves(root_, "", leaves);
        for (const std::string& leaf : leaves) {
            if (used_.count(leaf) == 0)
                return leaf;
        }
        return {};
    }

   private:
    const Json::Value* find(const std::string& path) {
        const Json::Value* node = &root_;
        size_t offset = 0;
        for (;;) {
            const size_t slash = path.find('/', offset);
            const std::string part = path.substr(offset, slash - offset);
            if (!node->isObject() || !node->isMember(part))
                return nullptr;
            node = &(*node)[part];
            if (slash == std::string::npos)
                break;
            offset = slash + 1;
        }
        used_.insert(path);
        return node;
    }

    static void collectLeaves(const Json::Value& node, const std::string& prefix,
                              std::set<std::string>& leaves) {
        for (const std::string& name : node.getMemberNames()) {
            const std::string path = prefix.empty() ? name : prefix + "/" + name;
            const Json::Value& child = node[name];
            if (child.isObject()) {
                collectLeaves(child, path, leaves);
            } else {
                leaves.insert(path);
            }
        }
    }

    static std::invalid_argument typeError(const std::string& path, const char* expected) {
        return std::invalid_argument("configuration key '" + path + "' must be " + expected);
    }

    Json::Value root_;
    std::set<std::string> used_;
};

}  // namespace module_support
