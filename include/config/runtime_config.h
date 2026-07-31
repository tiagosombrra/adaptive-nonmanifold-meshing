#ifndef AP_MESH_CONFIG_RUNTIME_CONFIG_H
#define AP_MESH_CONFIG_RUNTIME_CONFIG_H

#include <stdexcept>
#include <string>

namespace apmesh {

class ConfigError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

struct RuntimeConfig {
  int num_processes = 1;
  int num_threads = 1;
  std::string input_model;
  std::string write_mode = "h";
  std::string output_prefix;
  std::string use_template = "n";
  std::string adaptive_mode = "adaptive_stable";
  double adaptive_intensity = 0.45;
  double adaptive_quality_priority = 0.75;
  int adaptive_max_iterations = 6;
  double adaptive_target_growth = 1.6;
  double epsylon = 0.0000001;
  double min_improvement = 0.01;
  int patience = 2;
  double tol_local = 0.0000001;
  double smoothing_laplacian_number = 7.0;
  double smoothing_laplacian_factor = 0.5;
  double adaptation_relaxation = 1.0;
  double adaptation_max_delta = 0.10;
  int adaptive_retry_count = 0;
  double adaptive_retry_shrink = 1.0;
  double patch_factor_min = 0.80;
  double patch_factor_max = 1.12;
  double patch_refinement_strength = 0.22;
  double patch_coarsening_strength = 0.10;
  int curve_adaptation_policy = 0;
  double curve_adaptation_blend = 0.75;
  double curve_factor_sensitivity = 1.0;
  int curve_point_budget_mode = 0;
  double curve_point_growth_step1 = 2.5;
  double curve_point_growth_stepn = 1.4;
  double curve_point_budget_blend = 0.7;
  int curve_point_min = 2;
  int curve_point_max = 128;
  double quadtree_face_quality_threshold = 0.0;
  double quadtree_low_quality_h_factor = 1.0;
  int aft_local_postprocess_passes = 0;
  double aft_local_postprocess_quality_threshold = 0.35;
  double aft_local_postprocess_blend = 0.35;
  double step2_eligible_refinement_damp = 0.65;
  int patch_quadtree_depth_step1 = 5;
  int patch_quadtree_depth_stepn = 4;
  double patch_quadtree_min_h_scale = 1.0;
  int adapter_resolve_negative_quadtree_depth = 1;
  int enable_shared_curve_sync = 1;
  int enable_hybrid_reconstruction = 1;
  int write_runtime_summary = 1;
};

RuntimeConfig LoadRuntimeConfig(const std::string& path);

}  // namespace apmesh

#endif
