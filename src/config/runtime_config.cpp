#include "config/runtime_config.h"

#include <cmath>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <unordered_map>

namespace apmesh {
namespace {

using ConfigValues = std::unordered_map<std::string, std::string>;

const std::set<std::string> kSupportedKeys = {
    "ADAPTATION_MAX_DELTA",
    "ADAPTATION_RELAXATION",
    "ADAPTER_RESOLVE_NEGATIVE_QUADTREE_DEPTH",
    "ADAPTIVE_INTENSITY",
    "ADAPTIVE_MAX_ITERATIONS",
    "ADAPTIVE_MAX_STEPS",
    "ADAPTIVE_MODE",
    "ADAPTIVE_QUALITY_PRIORITY",
    "ADAPTIVE_RETRY_COUNT",
    "ADAPTIVE_RETRY_SHRINK",
    "ADAPTIVE_TARGET_GROWTH",
    "AFT_LOCAL_POSTPROCESS_BLEND",
    "AFT_LOCAL_POSTPROCESS_PASSES",
    "AFT_LOCAL_POSTPROCESS_QUALITY_THRESHOLD",
    "BUILD_DIR",
    "CURVE_ADAPTATION_BLEND",
    "CURVE_ADAPTATION_POLICY",
    "CURVE_FACTOR_SENSITIVITY",
    "CURVE_POINT_BUDGET_BLEND",
    "CURVE_POINT_BUDGET_MODE",
    "CURVE_POINT_GROWTH_STEP1",
    "CURVE_POINT_GROWTH_STEPN",
    "CURVE_POINT_MAX",
    "CURVE_POINT_MIN",
    "ENABLE_HYBRID_RECONSTRUCTION",
    "ENABLE_SHARED_CURVE_SYNC",
    "EPSYLON",
    "EXECUTABLE",
    "INPUT_MODEL",
    "MIN_IMPROVEMENT",
    "NUM_PROCESSES",
    "NUM_THREADS",
    "OUTPUT_PREFIX",
    "PATCH_COARSENING_STRENGTH",
    "PATCH_FACTOR_MAX",
    "PATCH_FACTOR_MIN",
    "PATCH_QUADTREE_DEPTH_STEP1",
    "PATCH_QUADTREE_DEPTH_STEPN",
    "PATCH_QUADTREE_MIN_H_SCALE",
    "PATCH_REFINEMENT_STRENGTH",
    "PATIENCE",
    "QUADTREE_FACE_QUALITY_THRESHOLD",
    "QUADTREE_LOW_QUALITY_H_FACTOR",
    "SMOOTHING_LAPLACIAN_FACTOR",
    "SMOOTHING_LAPLACIAN_NUMBER",
    "STEP2_ELIGIBLE_REFINEMENT_DAMP",
    "TOL_LOCAL",
    "USE_TEMPLATE",
    "WRITE_MODE",
    "WRITE_RUNTIME_SUMMARY",
};

std::string Trim(const std::string& value) {
  const std::string whitespace = " \t\r\n";
  const std::size_t start = value.find_first_not_of(whitespace);
  if (start == std::string::npos) {
    return "";
  }
  const std::size_t end = value.find_last_not_of(whitespace);
  return value.substr(start, end - start + 1);
}

bool HasModelData(std::istream& model) {
  std::string raw_line;
  while (std::getline(model, raw_line)) {
    const std::string line = Trim(raw_line);
    if (!line.empty() && line[0] != '#' && line.rfind("//", 0) != 0) {
      return true;
    }
  }
  return false;
}

ConfigValues ReadConfigFile(const std::string& path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw ConfigError("cannot open configuration file: " + path);
  }

  ConfigValues values;
  std::string raw_line;
  std::size_t line_number = 0;
  while (std::getline(file, raw_line)) {
    ++line_number;
    const std::string line = Trim(raw_line);
    if (line.empty() || line[0] == '#' || line.rfind("//", 0) == 0) {
      continue;
    }
    const std::size_t equal_pos = line.find('=');
    if (equal_pos == std::string::npos) {
      throw ConfigError("line " + std::to_string(line_number) +
                        " must use KEY=VALUE syntax");
    }
    const std::string key = Trim(line.substr(0, equal_pos));
    const std::string value = Trim(line.substr(equal_pos + 1));
    if (key.empty()) {
      throw ConfigError("line " + std::to_string(line_number) +
                        " contains an empty key");
    }
    if (kSupportedKeys.count(key) == 0U) {
      throw ConfigError("line " + std::to_string(line_number) +
                        " contains unsupported key: " + key);
    }
    if (!values.emplace(key, value).second) {
      throw ConfigError("line " + std::to_string(line_number) +
                        " duplicates key: " + key);
    }
  }
  return values;
}

std::string GetString(const ConfigValues& values, const std::string& key,
                      const std::string& fallback = "") {
  const auto found = values.find(key);
  return found == values.end() ? fallback : found->second;
}

int ParseInt(const ConfigValues& values, const std::string& key, int fallback) {
  const auto found = values.find(key);
  if (found == values.end()) {
    return fallback;
  }
  if (found->second.empty()) {
    throw ConfigError(key + " must not be empty");
  }
  std::size_t parsed = 0;
  long long value = 0;
  try {
    value = std::stoll(found->second, &parsed, 10);
  } catch (const std::exception&) {
    throw ConfigError(key + " must be an integer: " + found->second);
  }
  if (parsed != found->second.size() ||
      value < std::numeric_limits<int>::min() ||
      value > std::numeric_limits<int>::max()) {
    throw ConfigError(key + " must be an integer: " + found->second);
  }
  return static_cast<int>(value);
}

double ParseDouble(const ConfigValues& values, const std::string& key,
                   double fallback) {
  const auto found = values.find(key);
  if (found == values.end()) {
    return fallback;
  }
  if (found->second.empty()) {
    throw ConfigError(key + " must not be empty");
  }
  std::size_t parsed = 0;
  double value = 0.0;
  try {
    value = std::stod(found->second, &parsed);
  } catch (const std::exception&) {
    throw ConfigError(key + " must be a finite number: " + found->second);
  }
  if (parsed != found->second.size() || !std::isfinite(value)) {
    throw ConfigError(key + " must be a finite number: " + found->second);
  }
  return value;
}

void Require(bool condition, const std::string& message) {
  if (!condition) {
    throw ConfigError(message);
  }
}

void RequireRange(double value, double minimum, double maximum,
                  const std::string& key) {
  Require(value >= minimum && value <= maximum,
          key + " must be in [" + std::to_string(minimum) + ", " +
              std::to_string(maximum) + "]");
}

int ParseBoolean(const ConfigValues& values, const std::string& key,
                 int fallback) {
  const int value = ParseInt(values, key, fallback);
  Require(value == 0 || value == 1, key + " must be 0 or 1");
  return value;
}

}  // namespace

RuntimeConfig LoadRuntimeConfig(const std::string& path) {
  const ConfigValues values = ReadConfigFile(path);
  RuntimeConfig config;

  config.num_processes = ParseInt(values, "NUM_PROCESSES", config.num_processes);
  config.num_threads = ParseInt(values, "NUM_THREADS", config.num_threads);
  config.input_model = GetString(values, "INPUT_MODEL");
  config.write_mode = GetString(values, "WRITE_MODE", config.write_mode);
  config.output_prefix = GetString(values, "OUTPUT_PREFIX");
  config.use_template = GetString(values, "USE_TEMPLATE", "y");
  config.adaptive_mode =
      GetString(values, "ADAPTIVE_MODE", config.adaptive_mode);
  config.adaptive_intensity =
      ParseDouble(values, "ADAPTIVE_INTENSITY", config.adaptive_intensity);
  config.adaptive_quality_priority = ParseDouble(
      values, "ADAPTIVE_QUALITY_PRIORITY", config.adaptive_quality_priority);
  const auto max_iterations = values.find("ADAPTIVE_MAX_ITERATIONS");
  const auto max_steps = values.find("ADAPTIVE_MAX_STEPS");
  Require(!(max_iterations != values.end() && max_steps != values.end()),
          "use only one of ADAPTIVE_MAX_ITERATIONS or ADAPTIVE_MAX_STEPS");
  config.adaptive_max_iterations =
      max_iterations != values.end()
          ? ParseInt(values, "ADAPTIVE_MAX_ITERATIONS",
                     config.adaptive_max_iterations)
          : ParseInt(values, "ADAPTIVE_MAX_STEPS",
                     config.adaptive_max_iterations);
  config.adaptive_target_growth = ParseDouble(
      values, "ADAPTIVE_TARGET_GROWTH", config.adaptive_target_growth);
  config.epsylon = ParseDouble(values, "EPSYLON", config.epsylon);
  config.min_improvement =
      ParseDouble(values, "MIN_IMPROVEMENT", config.min_improvement);
  config.patience = ParseInt(values, "PATIENCE", config.patience);
  config.tol_local = ParseDouble(values, "TOL_LOCAL", config.tol_local);
  config.smoothing_laplacian_number = ParseDouble(
      values, "SMOOTHING_LAPLACIAN_NUMBER",
      config.smoothing_laplacian_number);
  config.smoothing_laplacian_factor = ParseDouble(
      values, "SMOOTHING_LAPLACIAN_FACTOR",
      config.smoothing_laplacian_factor);
  config.adaptation_relaxation = ParseDouble(
      values, "ADAPTATION_RELAXATION", config.adaptation_relaxation);
  config.adaptation_max_delta = ParseDouble(
      values, "ADAPTATION_MAX_DELTA", config.adaptation_max_delta);
  config.adaptive_retry_count =
      ParseInt(values, "ADAPTIVE_RETRY_COUNT", config.adaptive_retry_count);
  config.adaptive_retry_shrink = ParseDouble(
      values, "ADAPTIVE_RETRY_SHRINK", config.adaptive_retry_shrink);
  config.patch_factor_min =
      ParseDouble(values, "PATCH_FACTOR_MIN", config.patch_factor_min);
  config.patch_factor_max =
      ParseDouble(values, "PATCH_FACTOR_MAX", config.patch_factor_max);
  config.patch_refinement_strength =
      ParseDouble(values, "PATCH_REFINEMENT_STRENGTH",
                  config.patch_refinement_strength);
  config.patch_coarsening_strength =
      ParseDouble(values, "PATCH_COARSENING_STRENGTH",
                  config.patch_coarsening_strength);
  config.curve_adaptation_policy =
      ParseInt(values, "CURVE_ADAPTATION_POLICY",
               config.curve_adaptation_policy);
  config.curve_adaptation_blend =
      ParseDouble(values, "CURVE_ADAPTATION_BLEND",
                  config.curve_adaptation_blend);
  config.curve_factor_sensitivity =
      ParseDouble(values, "CURVE_FACTOR_SENSITIVITY",
                  config.curve_factor_sensitivity);
  config.curve_point_budget_mode =
      ParseInt(values, "CURVE_POINT_BUDGET_MODE",
               config.curve_point_budget_mode);
  config.curve_point_growth_step1 =
      ParseDouble(values, "CURVE_POINT_GROWTH_STEP1",
                  config.curve_point_growth_step1);
  config.curve_point_growth_stepn =
      ParseDouble(values, "CURVE_POINT_GROWTH_STEPN",
                  config.curve_point_growth_stepn);
  config.curve_point_budget_blend =
      ParseDouble(values, "CURVE_POINT_BUDGET_BLEND",
                  config.curve_point_budget_blend);
  config.curve_point_min =
      ParseInt(values, "CURVE_POINT_MIN", config.curve_point_min);
  config.curve_point_max =
      ParseInt(values, "CURVE_POINT_MAX", config.curve_point_max);
  config.quadtree_face_quality_threshold =
      ParseDouble(values, "QUADTREE_FACE_QUALITY_THRESHOLD",
                  config.quadtree_face_quality_threshold);
  config.quadtree_low_quality_h_factor =
      ParseDouble(values, "QUADTREE_LOW_QUALITY_H_FACTOR",
                  config.quadtree_low_quality_h_factor);
  config.aft_local_postprocess_passes =
      ParseInt(values, "AFT_LOCAL_POSTPROCESS_PASSES",
               config.aft_local_postprocess_passes);
  config.aft_local_postprocess_quality_threshold =
      ParseDouble(values, "AFT_LOCAL_POSTPROCESS_QUALITY_THRESHOLD",
                  config.aft_local_postprocess_quality_threshold);
  config.aft_local_postprocess_blend =
      ParseDouble(values, "AFT_LOCAL_POSTPROCESS_BLEND",
                  config.aft_local_postprocess_blend);
  config.step2_eligible_refinement_damp =
      ParseDouble(values, "STEP2_ELIGIBLE_REFINEMENT_DAMP",
                  config.step2_eligible_refinement_damp);
  config.patch_quadtree_depth_step1 =
      ParseInt(values, "PATCH_QUADTREE_DEPTH_STEP1",
               config.patch_quadtree_depth_step1);
  config.patch_quadtree_depth_stepn =
      ParseInt(values, "PATCH_QUADTREE_DEPTH_STEPN",
               config.patch_quadtree_depth_stepn);
  config.patch_quadtree_min_h_scale =
      ParseDouble(values, "PATCH_QUADTREE_MIN_H_SCALE",
                  config.patch_quadtree_min_h_scale);
  config.adapter_resolve_negative_quadtree_depth =
      ParseBoolean(values, "ADAPTER_RESOLVE_NEGATIVE_QUADTREE_DEPTH",
                   config.adapter_resolve_negative_quadtree_depth);
  config.enable_shared_curve_sync =
      ParseBoolean(values, "ENABLE_SHARED_CURVE_SYNC",
                   config.enable_shared_curve_sync);
  config.enable_hybrid_reconstruction =
      ParseBoolean(values, "ENABLE_HYBRID_RECONSTRUCTION",
                   config.enable_hybrid_reconstruction);
  config.write_runtime_summary =
      ParseBoolean(values, "WRITE_RUNTIME_SUMMARY",
                   config.write_runtime_summary);

  Require(!config.input_model.empty(), "INPUT_MODEL is required");
  Require(!config.output_prefix.empty(), "OUTPUT_PREFIX is required");
  std::ifstream model(config.input_model);
  Require(model.good(), "INPUT_MODEL does not exist or is unreadable: " +
                            config.input_model);
  Require(HasModelData(model),
          "INPUT_MODEL contains no model data: " + config.input_model);
  Require(config.num_processes >= 1, "NUM_PROCESSES must be at least 1");
  Require(config.num_threads >= 1, "NUM_THREADS must be at least 1");
  Require(config.write_mode == "m" || config.write_mode == "q" ||
              config.write_mode == "h",
          "WRITE_MODE must be one of m, q or h");
  Require(config.use_template == "y" || config.use_template == "n",
          "USE_TEMPLATE must be y or n");
  Require(config.adaptive_mode == "adaptive_stable" ||
              config.adaptive_mode == "research_debug" ||
              config.adaptive_mode == "legacy",
          "ADAPTIVE_MODE must be adaptive_stable, research_debug or legacy");
  RequireRange(config.adaptive_intensity, 0.0, 1.0, "ADAPTIVE_INTENSITY");
  RequireRange(config.adaptive_quality_priority, 0.0, 1.0,
               "ADAPTIVE_QUALITY_PRIORITY");
  Require(config.adaptive_max_iterations >= 2,
          "ADAPTIVE_MAX_STEPS must be at least 2");
  Require(config.adaptive_target_growth >= 1.0,
          "ADAPTIVE_TARGET_GROWTH must be at least 1");
  Require(config.epsylon > 0.0, "EPSYLON must be positive");
  Require(config.min_improvement >= 0.0,
          "MIN_IMPROVEMENT must not be negative");
  Require(config.patience >= 1, "PATIENCE must be at least 1");
  Require(config.tol_local > 0.0, "TOL_LOCAL must be positive");
  RequireRange(config.smoothing_laplacian_number, 5.0, 7.0,
               "SMOOTHING_LAPLACIAN_NUMBER");
  RequireRange(config.smoothing_laplacian_factor, 0.0, 1.0,
               "SMOOTHING_LAPLACIAN_FACTOR");
  RequireRange(config.adaptation_relaxation, 0.0, 1.0,
               "ADAPTATION_RELAXATION");
  Require(config.adaptation_max_delta >= 0.0,
          "ADAPTATION_MAX_DELTA must not be negative");
  Require(config.adaptive_retry_count >= 0,
          "ADAPTIVE_RETRY_COUNT must not be negative");
  RequireRange(config.adaptive_retry_shrink, 0.05, 1.0,
               "ADAPTIVE_RETRY_SHRINK");
  Require(config.patch_factor_min >= 0.1,
          "PATCH_FACTOR_MIN must be at least 0.1");
  Require(config.patch_factor_max >= config.patch_factor_min,
          "PATCH_FACTOR_MAX must be at least PATCH_FACTOR_MIN");
  RequireRange(config.patch_refinement_strength, 0.0, 1.0,
               "PATCH_REFINEMENT_STRENGTH");
  RequireRange(config.patch_coarsening_strength, 0.0, 1.0,
               "PATCH_COARSENING_STRENGTH");
  Require(config.curve_adaptation_policy >= 0 &&
              config.curve_adaptation_policy <= 3,
          "CURVE_ADAPTATION_POLICY must be in [0, 3]");
  RequireRange(config.curve_adaptation_blend, 0.0, 1.0,
               "CURVE_ADAPTATION_BLEND");
  Require(config.curve_factor_sensitivity >= 0.0,
          "CURVE_FACTOR_SENSITIVITY must not be negative");
  Require(config.curve_point_budget_mode == 0 ||
              config.curve_point_budget_mode == 1,
          "CURVE_POINT_BUDGET_MODE must be 0 or 1");
  Require(config.curve_point_growth_step1 >= 1.0,
          "CURVE_POINT_GROWTH_STEP1 must be at least 1");
  Require(config.curve_point_growth_stepn >= 1.0,
          "CURVE_POINT_GROWTH_STEPN must be at least 1");
  RequireRange(config.curve_point_budget_blend, 0.0, 1.0,
               "CURVE_POINT_BUDGET_BLEND");
  Require(config.curve_point_min >= 2,
          "CURVE_POINT_MIN must be at least 2");
  Require(config.curve_point_max >= config.curve_point_min,
          "CURVE_POINT_MAX must be at least CURVE_POINT_MIN");
  RequireRange(config.quadtree_face_quality_threshold, 0.0, 1.0,
               "QUADTREE_FACE_QUALITY_THRESHOLD");
  RequireRange(config.quadtree_low_quality_h_factor, 0.1, 1.5,
               "QUADTREE_LOW_QUALITY_H_FACTOR");
  Require(config.aft_local_postprocess_passes >= 0,
          "AFT_LOCAL_POSTPROCESS_PASSES must not be negative");
  RequireRange(config.aft_local_postprocess_quality_threshold, 0.0, 1.0,
               "AFT_LOCAL_POSTPROCESS_QUALITY_THRESHOLD");
  RequireRange(config.aft_local_postprocess_blend, 0.0, 1.0,
               "AFT_LOCAL_POSTPROCESS_BLEND");
  RequireRange(config.step2_eligible_refinement_damp, 0.0, 1.0,
               "STEP2_ELIGIBLE_REFINEMENT_DAMP");
  Require(config.patch_quadtree_depth_step1 >= 0 &&
              config.patch_quadtree_depth_step1 <= 32,
          "PATCH_QUADTREE_DEPTH_STEP1 must be in [0, 32]");
  Require(config.patch_quadtree_depth_stepn >= 0 &&
              config.patch_quadtree_depth_stepn <= 32,
          "PATCH_QUADTREE_DEPTH_STEPN must be in [0, 32]");
  RequireRange(config.patch_quadtree_min_h_scale, 0.5, 1.5,
               "PATCH_QUADTREE_MIN_H_SCALE");

  return config;
}

}  // namespace apmesh
