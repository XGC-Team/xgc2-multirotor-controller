#include "px4_multirotor_controller/common/world_boundary.h"

#include <json/json.h>

#include <cmath>
#include <memory>
#include <set>
#include <stdexcept>

namespace px4_multirotor_controller {
namespace {

void requireKeys(const Json::Value& value, const std::set<std::string>& expected) {
    if (!value.isObject()) {
        throw std::invalid_argument(
            "world_boundary_json requires the complete worldBoundary object or null");
    }
    const auto names = value.getMemberNames();
    if (std::set<std::string>(names.begin(), names.end()) != expected) {
        throw std::invalid_argument("world_boundary_json has missing or unknown fields");
    }
}

double finiteNumber(const Json::Value& value) {
    if (!value.isNumeric() || !std::isfinite(value.asDouble())) {
        throw std::invalid_argument(
            "world_boundary_json endpoints and groundZ must be finite numbers");
    }
    return value.asDouble();
}

Json::Value boundaryJSON(const std::optional<WorldBoundary>& boundary) {
    if (!boundary) {
        return Json::Value();
    }
    Json::Value value(Json::objectValue);
    value["schemaVersion"] = 1;
    value["frameId"] = "world";
    value["unit"] = "m";
    value["groundZ"] = boundary->ground_z ? Json::Value(*boundary->ground_z) : Json::Value();
    value["controlBounds"] = Json::Value();
    if (boundary->control_bounds) {
        const auto& bounds = *boundary->control_bounds;
        auto& output = value["controlBounds"];
        output["xMin"] = bounds.x_min;
        output["xMax"] = bounds.x_max;
        output["yMin"] = bounds.y_min;
        output["yMax"] = bounds.y_max;
        output["zMin"] = bounds.z_min;
        output["zMax"] = bounds.z_max;
    }
    return value;
}

}  // namespace

std::optional<WorldBoundary> parseWorldBoundary(const std::string& raw) {
    Json::CharReaderBuilder builder;
    builder["allowComments"] = false;
    builder["collectComments"] = false;
    builder["allowDroppedNullPlaceholders"] = false;
    builder["allowNumericKeys"] = false;
    builder["allowSingleQuotes"] = false;
    builder["allowSpecialFloats"] = false;
    builder["rejectDupKeys"] = true;
    builder["failIfExtra"] = true;
    builder["strictRoot"] = false;  // Explicit null is the current unset value.
    builder["stackLimit"] = 16;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    Json::Value value;
    std::string errors;
    if (!reader->parse(raw.data(), raw.data() + raw.size(), &value, &errors)) {
        throw std::invalid_argument("world_boundary_json is not valid unique-key JSON");
    }
    if (value.isNull()) {
        return std::nullopt;
    }
    requireKeys(value, {"schemaVersion", "frameId", "unit", "controlBounds", "groundZ"});
    const auto& version = value["schemaVersion"];
    if ((version.type() != Json::intValue && version.type() != Json::uintValue) ||
        version.asLargestInt() != 1 || !value["frameId"].isString() ||
        value["frameId"].asString() != "world" || !value["unit"].isString() ||
        value["unit"].asString() != "m") {
        throw std::invalid_argument(
            "world_boundary_json requires schemaVersion=1, frameId=world, unit=m");
    }
    WorldBoundary result;
    if (!value["groundZ"].isNull()) {
        result.ground_z = finiteNumber(value["groundZ"]);
    }
    const auto& bounds = value["controlBounds"];
    if (!bounds.isNull()) {
        requireKeys(bounds, {"xMin", "xMax", "yMin", "yMax", "zMin", "zMax"});
        result.control_bounds =
            WorldControlBounds{finiteNumber(bounds["xMin"]), finiteNumber(bounds["xMax"]),
                               finiteNumber(bounds["yMin"]), finiteNumber(bounds["yMax"]),
                               finiteNumber(bounds["zMin"]), finiteNumber(bounds["zMax"])};
        const auto& checked = *result.control_bounds;
        if (checked.x_min >= checked.x_max || checked.y_min >= checked.y_max ||
            checked.z_min >= checked.z_max) {
            throw std::invalid_argument("world_boundary_json requires min < max on all three axes");
        }
    }
    return result;
}

std::string controllerLoadFactJSON(const std::optional<WorldBoundary>& boundary,
                                   const ControllerLoadFact& fact) {
    Json::Value output(Json::objectValue);
    output["schemaVersion"] = 1;
    output["source"]["kind"] = "multirotor-controller";
    output["source"]["id"] = fact.node_name;
    output["source"]["instanceId"] = fact.instance_id;
    output["sequence"] = 1;
    output["event"] = "applied";
    output["evidence"] = "producer-applied";
    output["effectiveAt"]["rosTimeNs"] = std::to_string(fact.ros_time_ns);
    output["effectiveAt"]["unixTimeNs"] = std::to_string(fact.unix_time_ns);
    output["effectiveAt"]["monotonicTimeNs"] = std::to_string(fact.monotonic_time_ns);
    auto& values = output["values"];
    values["worldBoundary"] = boundaryJSON(boundary);
    values["fenceEnabled"] = boundary && boundary->control_bounds.has_value();
    values["positionDistanceLimitMetres"] = fact.position_distance_limit_metres;
    values["canonicalPoseTopic"] = fact.canonical_pose_topic;
    values["localPoseTopic"] = fact.local_pose_topic;
    values["worldPoseTopic"] = fact.world_pose_topic;
    values["trackingBackend"] = fact.tracking_backend;
    auto& provenance = output["provenance"];
    provenance["boundary"] = "controller-setConfig-returned";
    provenance["topics"] = "sensor-input-producer-set-args";
    provenance["appliesTo"] = "controller-geofence-and-position-distance-configuration";
    provenance["parameter"] = "~world_boundary_json";
    provenance["coordinates"] = "experiment-world-metres-no-controller-offset";
    provenance["groundZUsage"] = "retained-metadata-not-control-threshold";
    provenance["coverage"] = "listed-loaded-fields-only-not-all-controller-parameters";
    Json::StreamWriterBuilder writer;
    writer["indentation"] = "";
    return Json::writeString(writer, output);
}

}  // namespace px4_multirotor_controller
