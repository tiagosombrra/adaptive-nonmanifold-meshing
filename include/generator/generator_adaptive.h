#ifndef GENERATOR_GENERATOR_ADAPTIVE_H
#define GENERATOR_GENERATOR_ADAPTIVE_H

#include <omp.h>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "../adapter/adapter.h"
#include "../crab_mesh/performer/ranged_id_manager.h"
#include "../curvature/curvature_analytical.h"
#include "../curvature/curvature_discrete.h"
#include "../data/definitions.h"
#include "../data/node_adaptive.h"
#include "../data/patch/patch_bezier.h"
#include "../data/patch/patch_coons.h"
#include "../data/triangle_adaptive.h"
#include "../data/mesh/mesh_adaptive.h"
#include "../estimate/charge_estimate_process.h"
#include "../input_output/models_3d.h"
#include "../input_output/patch_reader.h"
#include "../parallel/ApMeshCommunicator.h"
#include "../timer/timer.h"

using namespace Data;

extern double TOLERANCE;
extern double TOLERANCE_CURVATURE;
extern int NUM_THREADS;
extern double DISCRETIZATION_CURVE_FACTOR;
extern int STEPS;
extern std::string WRITE_MESH;
extern int RANK_MPI;
extern int SIZE_MPI;
extern double EPSYLON;
extern double MIN_IMPROVEMENT;
extern int PATIENCE;
extern double TOL_LOCAL;
extern double ADAPTATION_RELAXATION;
extern double ADAPTATION_MAX_DELTA;
extern int ADAPTIVE_RETRY_COUNT;
extern double ADAPTIVE_RETRY_SHRINK;
extern double MAX_RETRY_ELEMENT_GROWTH_FACTOR;
extern double MAX_RETRY_ELEMENT_GROWTH_ABS;
extern double ERROR_FLOOR;
extern double MIN_ERROR_DROP_ABS;
extern double MIN_ERROR_DROP_PER_ELEMENT;
extern int PATCH_ADAPTATION_MODE;
extern double PATCH_FACTOR_MIN;
extern double PATCH_FACTOR_MAX;
extern double PATCH_REFINEMENT_STRENGTH;
extern double PATCH_COARSENING_STRENGTH;
extern double PATCH_ERROR_EXPONENT;
extern double PATCH_QUALITY_WEIGHT;
extern double PATCH_QUALITY_TARGET;
extern double PATCH_QUALITY_MIN_TARGET;
extern double PATCH_TOP_ERROR_FRACTION;
extern int CURVE_ADAPTATION_POLICY;
extern double CURVE_ADAPTATION_BLEND;
extern double CURVE_FACTOR_SENSITIVITY;
extern int ACCEPTANCE_MODE;
extern double ACCEPTANCE_WEIGHT_ERROR;
extern double ACCEPTANCE_WEIGHT_QUALITY_MIN;
extern double ACCEPTANCE_WEIGHT_POOR_RATIO;
extern double ACCEPTANCE_WEIGHT_GROWTH;
extern double ACCEPTANCE_STEP1_GROWTH_WEIGHT;
extern double ACCEPTANCE_STEPN_GROWTH_WEIGHT;
extern double ACCEPTANCE_STEP1_MAX_GROWTH_SOFT;
extern double ACCEPTANCE_STEPN_MAX_GROWTH_SOFT;
extern int ACCEPTANCE_GROWTH_PENALTY_MODE;
extern double ACCEPTANCE_STEP1_MIN_ERROR_DROP_ABS;
extern double ACCEPTANCE_STEP1_MIN_ERROR_DROP_PCT;
extern double ACCEPTANCE_MIN_SCORE;
extern double ACCEPTANCE_MAX_ERROR_INCREASE_PCT;
extern double ACCEPTANCE_POOR_RATIO_TARGET;
extern int WRITE_ADAPTATION_DEBUG;
extern double PATCH_EFFICIENCY_WEIGHT;
extern double PATCH_EFFICIENCY_TARGET;
extern double PATCH_HISTORY_BLEND;
extern double PATCH_STEP2_REFINEMENT_SCALE;
extern double PATCH_INEFFICIENT_CAP;
extern int PATCH_STEPWISE_MODE;
extern double STEP2_REFINEMENT_ATTENUATION;
extern double STEP2_TOP_ERROR_FRACTION_SCALE;
extern double STEP2_FACTOR_RANGE_SHRINK;
extern double STEP2_CURVE_SENSITIVITY_SCALE;
extern double CURVE_EFFICIENCY_WEIGHT;
extern int STEP2_HARD_SPATIAL_FILTER_MODE;
extern int STEP2_TOP_PATCH_COUNT_MIN;
extern int STEP2_TOP_PATCH_COUNT_MAX;
extern double STEP2_NONELIGIBLE_FACTOR_MIN;
extern int STEP2_NONELIGIBLE_FORCE_COARSEN;
extern double STEP2_SMOOTHING_NEIGHBOR_WEIGHT;
extern double STEP2_ELIGIBLE_REFINEMENT_DAMP;
extern double STEP2_ELIGIBLE_REFINEMENT_DAMP_ON_RETRY;
extern double STEP2_ELIGIBLE_ELEMENTS_THRESHOLD;
extern double STEP2_ELIGIBLE_H_SCALE;
extern int QUADTREE_SATURATION_MODE;
extern double QUADTREE_SATURATION_ELEMENTS_THRESHOLD;
extern double QUADTREE_SATURATION_EFFICIENCY_THRESHOLD;
extern double QUADTREE_SATURATION_H_SCALE;
extern int QUADTREE_HARD_SATURATION_MODE;
extern int QUADTREE_HARD_SATURATION_STEPS;
extern double QUADTREE_HARD_SATURATION_FACTOR_MIN;
extern int QUADTREE_SKIP_LOW_QUALITY_SUBDIVIDE_ON_SATURATED;
extern int QUADTREE_POSTPROCESS_DAMP_ON_SATURATED;
extern int STEP_ELEMENT_BUDGET_MODE;
extern double STEP_ELEMENT_BUDGET_ABS;
extern double STEP_ELEMENT_BUDGET_GROWTH_FACTOR;
extern double STEP_ELEMENT_BUDGET_SOFT_RATIO;
extern double STEP_ELEMENT_BUDGET_RETRY_SHRINK_TOPK;
extern double STEP_ELEMENT_BUDGET_RETRY_H_SCALE;
extern double STEP_ELEMENT_BUDGET_RETRY_FACTOR_RANGE_SCALE;
extern int STEP_ELEMENT_BUDGET_PRESSURE_MODE;
extern double STEP_ELEMENT_BUDGET_PRESSURE_START;
extern double STEP_ELEMENT_BUDGET_PRESSURE_H_SCALE;
extern double STEP_ELEMENT_BUDGET_PRESSURE_FACTOR_SHRINK;
extern int STEP2_MICRO_REFINEMENT_MODE;
extern double STEP2_MICRO_RELAXATION_SCALE;
extern double STEP2_MICRO_MAX_DELTA_SCALE;
extern double STEP2_MICRO_PATCH_REFINEMENT_SCALE;
extern double STEP2_MICRO_FACTOR_RANGE_SCALE;
extern int STEP2_ACCEPT_SMALL_GAINS_MODE;
extern double STEP2_MIN_ERROR_DROP_ABS;
extern double STEP2_MIN_ERROR_DROP_PER_ELEMENT;
extern double STEP2_ACCEPTANCE_GROWTH_WEIGHT;
extern int PATCH_LOCAL_ELEMENT_BUDGET_MODE;
extern double PATCH_LOCAL_ELEMENT_BUDGET_BASE;
extern double PATCH_LOCAL_ELEMENT_BUDGET_SCALE_BY_RANK;
extern double PATCH_LOCAL_ELEMENT_BUDGET_SCALE_BY_EFFICIENCY;
extern double PATCH_LOCAL_BUDGET_H_SCALE;
extern double PATCH_LOCAL_BUDGET_HARD_CAP;
extern int PATCH_GENERATION_CONTROL_MODE;
extern double PATCH_GENERATION_BUDGET_PRESSURE_START;
extern double PATCH_GENERATION_MICRO_H_SCALE;
extern double PATCH_GENERATION_CONSTRAINED_H_SCALE;
extern int PATCH_GENERATION_TOP_RANK_ONLY_MODE;
extern int STEP_TARGET_ELEMENT_GROWTH_MODE;
extern double STEP2_TARGET_ELEMENT_GROWTH;
extern double STEP3PLUS_TARGET_ELEMENT_GROWTH;
extern double STEP_TARGET_GROWTH_H_SCALE;
extern int STEP_ACCEPT_EFFICIENCY_MODE;
extern double STEP_ACCEPT_MIN_EFFICIENCY;
extern double STEP_ACCEPT_MAX_ELEMENT_GROWTH_SOFT;
extern double STEP_ACCEPT_QUALITY_DROP_TOLERANCE;
extern int BUDGET_DRIVEN_ADAPTATION;
extern int STEP_ELEMENT_TARGET_MODE;
extern double STEP1_TARGET_ELEMENT_GROWTH;
extern double STEP_ELEMENT_TARGET_MIN;
extern double STEP_ELEMENT_TARGET_MAX;
extern int PATCH_BUDGET_ALLOCATION_MODE;
extern double PATCH_BUDGET_ERROR_WEIGHT;
extern double PATCH_BUDGET_EFFICIENCY_WEIGHT;
extern double PATCH_BUDGET_QUALITY_WEIGHT;
extern double PATCH_BUDGET_MIN_SHARE;
extern int PATCH_BUDGET_TOPK_ONLY_MODE;
extern int CURVE_POINT_BUDGET_MODE;
extern double CURVE_POINT_GROWTH_STEP1;
extern double CURVE_POINT_GROWTH_STEPN;
extern double CURVE_POINT_BUDGET_BLEND;
extern int CURVE_POINT_MIN;
extern int CURVE_POINT_MAX;
extern int PATCH_QUADTREE_CAP_MODE;
extern int PATCH_QUADTREE_DEPTH_STEP1;
extern int PATCH_QUADTREE_DEPTH_STEPN;
extern double PATCH_QUADTREE_MIN_H_SCALE;
extern double PATCH_QUADTREE_NEIGHBOR_RELAX;
extern int PATCH_QUADTREE_TEMPLATE_LIMIT_MODE;
extern double STEP_ACCEPT_TARGET_GROWTH_TOLERANCE;
extern double STEP_ACCEPT_ERROR_DROP_MIN;
extern double STEP_ACCEPT_BUDGET_MISS_PENALTY;
extern int PATCH_CONSISTENCY_MODE;
extern double PATCH_CONSISTENCY_AREA_TOL;
extern double PATCH_CONSISTENCY_ERROR_TOL;
extern double PATCH_CONSISTENCY_CURVATURE_TOL;
extern double PATCH_CONSISTENCY_QUALITY_TOL;
extern int PATCH_CONSISTENCY_NEIGHBOR_ONLY;
extern double PATCH_CONSISTENCY_MAX_BUDGET_RATIO_STEP2;
extern double PATCH_CONSISTENCY_MAX_BUDGET_RATIO_STEPN;
extern double PATCH_CONSISTENCY_MIN_GROUP_SHARE;
extern double PATCH_CONSISTENCY_H_CLAMP_RATIO;
extern int PATCH_CONSISTENCY_DEPTH_CLAMP;
extern int CURVE_CONSISTENCY_MODE;
extern double CURVE_CONSISTENCY_MAX_POINT_RATIO_STEP2;
extern int CURVE_CONSISTENCY_PRIORITY_BONUS;
extern double CURVE_CONSISTENCY_GROUP_BLEND;
extern double STEP2_CONSISTENCY_WEIGHT;
extern int STEP2_GROUP_MIN_PATCHES;
extern int STEP2_GROUP_NEIGHBOR_PROTECTION;
extern int STEP2_DISABLE_GROUP_COARSEN_IF_UNBALANCED;
extern int STEP_ACCEPT_CONSISTENCY_MODE;
extern double STEP_ACCEPT_CONSISTENCY_PENALTY_WEIGHT;
extern double STEP_ACCEPT_MAX_GROUP_DISPERSION_STEP2;
extern std::string ADAPTIVE_MODE;
extern double ADAPTIVE_INTENSITY;
extern double ADAPTIVE_QUALITY_PRIORITY;
extern int ADAPTIVE_MAX_STEPS;
extern double ADAPTIVE_TARGET_GROWTH;
extern int ENABLE_SHARED_CURVE_SYNC;
extern int ENABLE_HYBRID_RECONSTRUCTION;
extern int WRITE_RUNTIME_SUMMARY;
extern std::vector<CurveAdaptiveParametric*> ptr_aux;

class GeneratorAdaptive {
 public:
  struct PatchAdaptationSnapshot {
    unsigned int patch_id = 0U;
    double local_error = 0.0;
    double normalized_error = 0.0;
    double quality_min = 0.0;
    double quality_mean = 0.0;
    double poor_ratio = 0.0;
    double quality_min_3d = 0.0;
    double quality_mean_3d = 0.0;
    double poor_ratio_3d = 0.0;
    double good_ratio_ge_0_60_3d = 0.0;
    double quality_min_uv = 0.0;
    double quality_mean_uv = 0.0;
    double poor_ratio_uv = 0.0;
    double good_ratio_ge_0_60_uv = 0.0;
    double raw_factor = 1.0;
    double smoothed_factor = 1.0;
    double limited_factor = 1.0;
    unsigned long long elements_prev = 0ULL;
    unsigned long long elements_current = 0ULL;
    unsigned long long elements_delta = 0ULL;
    double local_error_drop_abs = 0.0;
    double local_error_drop_pct = 0.0;
    double local_error_drop_per_element = 0.0;
    double efficiency_score = 0.0;
    int eligible_for_refinement = 0;
    int saturated_soft = 0;
    int saturated_hard = 0;
    unsigned int consecutive_inefficient_steps = 0U;
    int budget_blocked = 0;
    int refinement_rank = -1;
    std::string final_refinement_class = "freeze";
    double eligible_h_scale_applied = 1.0;
    int micro_refinement_active = 0;
    double budget_pressure_local = 0.0;
    int intra_patch_damped = 0;
    double local_patch_budget = 0.0;
    int patch_budget_exceeded = 0;
    std::string patch_generation_mode = "normal";
    double target_patch_elements = 0.0;
    double actual_patch_elements = 0.0;
    double patch_budget_share = 0.0;
    double error_budget_share = 0.0;
    double quality_repair_budget_share = 0.0;
    int similar_patch_group = -1;
    double similarity_score = 0.0;
    double similarity_score_to_neighbors = 0.0;
    int is_consistency_anchor = 0;
    int consistency_clamped = 0;
    double budget_before_consistency = 0.0;
    double budget_after_consistency = 0.0;
    double target_h_before_consistency = 0.0;
    double target_h_after_consistency = 0.0;
    int group_anchor_patch = -1;
    double group_budget_ratio = 1.0;
    double target_patch_growth = 0.0;
    double target_patch_h = 0.0;
    double applied_patch_h = 0.0;
    int quadtree_cap_applied = 0;
    int max_quadtree_depth_applied = -1;
    double transition_floor = 0.0;
    int transition_underresolved = 0;
    double interior_transition_imbalance = 0.0;
    double curve_domain_balance = 1.0;
    double curve_interior_coupling = 1.0;
    double good_ratio_ge_0_60 = 0.0;
    unsigned long long boundary_element_count = 0ULL;
    double boundary_quality_min = 0.0;
    double boundary_quality_mean = 0.0;
    double boundary_good_ratio_ge_0_60 = 0.0;
    unsigned long long transition_band_elements = 0ULL;
    double transition_band_quality_min = 0.0;
    double transition_band_quality_mean = 0.0;
    double transition_band_q_ge_0_60 = 0.0;
    unsigned long long interior_core_elements = 0ULL;
    double interior_core_quality_min = 0.0;
    double interior_core_quality_mean = 0.0;
    double interior_core_q_ge_0_60 = 0.0;
    unsigned long long generated_faces_structural = 0ULL;
    std::string generation_status_structural = "unknown";
  };

  struct CurveAdaptationSnapshot {
    unsigned long curve_id = 0UL;
    double factor = 1.0;
    unsigned int num_points = 0U;
    int policy = 0;
    std::string adjacent_patch_ids;
    std::string adjacent_patch_weights;
    int target_points = 0;
    int actual_points = 0;
    double point_growth_factor = 1.0;
    std::string curve_budget_source_patches;
    int geometric_target_points = 0;
    double min_adjacent_target_h = 0.0;
    double max_adjacent_target_elements = 0.0;
    int sensitive_curve = 0;
    double curve_patch_balance_ratio = 1.0;
    int curve_consistency_clamped = 0;
    int target_points_before_consistency = 0;
    int target_points_after_consistency = 0;
  };

  struct PatchHistoryState {
    double prev_patch_error = 0.0;
    unsigned long long prev_patch_elements = 0ULL;
    double prev_patch_quality_min = 0.0;
    double prev_patch_quality_mean = 0.0;
    double prev_patch_poor_ratio = 0.0;
    double prev_patch_factor = 1.0;
    double prev_patch_error_drop_abs = 0.0;
    double prev_patch_error_drop_per_element = 0.0;
    double prev_patch_efficiency = 0.0;
    double prev_patch_template_score_mean = 0.0;
    unsigned long long prev_patch_generated_faces = 0ULL;
    double prev_patch_budget_share = 0.0;
    double prev_patch_target_elements = 0.0;
    int prev_patch_improved = 0;
    std::string prev_patch_decision_class = "freeze";
    unsigned int consecutive_inefficient_steps = 0U;
    bool saturated_last_step = false;
  };

  struct MeshQualityStats {
    double quality_min = 0.0;
    double quality_mean = 0.0;
    double poor_ratio = 0.0;
    double good_ratio_ge_0_60 = 0.0;
    double quality_min_3d = 0.0;
    double quality_mean_3d = 0.0;
    double poor_ratio_3d = 0.0;
    double good_ratio_ge_0_60_3d = 0.0;
    double quality_min_uv = 0.0;
    double quality_mean_uv = 0.0;
    double poor_ratio_uv = 0.0;
    double good_ratio_ge_0_60_uv = 0.0;
    double angle_quality_mean_3d = 0.0;
    double edge_ratio_mean_3d = 0.0;
    double min_angle_mean_3d = 0.0;
    double min_angle_p05_3d = 0.0;
    double max_angle_mean_3d = 0.0;
    double max_angle_p95_3d = 0.0;
    unsigned long long total_elements = 0ULL;
  };

  struct AdaptivePolicyRuntime {
    std::string mode = "legacy";
    bool stable_mode = false;
    bool debug_mode = false;
    int max_steps = 3;
    double intensity = 0.45;
    double quality_priority = 0.75;
    double step1_target_growth = 4.5;
    double stepn_target_growth = 1.6;
    double step2_accept_quality_min = 0.20;
    double step2_accept_max_error_increase_abs = 0.50;
    double equivalent_patch_min_ratio = 0.50;
  };

  typedef std::vector<std::pair<int, std::unique_ptr<MeshAdaptive>>> MeshVector;
  typedef std::vector<std::pair<int, MeshAdaptive*>> ErrorMeshVector;

  GeneratorAdaptive();
  ~GeneratorAdaptive() = default;

#if USE_MPI
  int Execute(char* argv[], Timer* timer, MPI_Status status);
  std::list<PatchBezier*> EstimateChargeofPatches(Geometry* geometry, Timer* timer);
  std::vector<CurveAdaptive*> CreateVectorOfCurves(std::list<PatchBezier*> patches);
  std::list<PatchBezier*> OrderPatchesDistribProcess(std::list<PatchBezier*> patches);
  bool VerifyCurve(PointAdaptive p0, PointAdaptive p1, PointAdaptive p2, PointAdaptive p3, std::vector<CurveAdaptive*> curves);
  void CalculateEstimateProcessElements(int size_process, std::list<PatchBezier*> patches);
  std::list<PatchBezier*>::iterator GetIteratorListPatches(int size_patches, std::list<PatchBezier*> patches);
  void Generator(double patches[], int size_patches, Timer* timer, int id_range = 1024, int size_rank = 1, int size_thread = 1);
  Geometry* UnpakGeometry(double patches[], int size_patches);
#else
  int Execute(char* argv[], Timer* timer);
  void Generator(Model& model, Timer* timer, int id_range = 1024, int size_rank = 1, int size_thread = 1);
#endif

  SubMesh* InitialMesh(PatchCoons* patch, Performer::IdManager* id_manager);
  double ErrorGlobal(MeshAdaptive* mesh, Timer* timer, int rank = 0, int size_thread = 0);
  
  std::unique_ptr<Performer::IdManager> MakeIdManager(const Parallel::TMCommunicator* comm, Int id) const;
  std::unique_ptr<Performer::IdManager> MakeIdManagerOmp(const Parallel::TMCommunicator* comm, Int id) const;
  std::unique_ptr<Performer::IdManager> MakeIdManagerElementOmp(const Parallel::TMCommunicator* comm, Int id) const;

  void WriteMesh(MeshAdaptive* mesh, int step);
  void WriteMesh(MeshAdaptive* mesh, int step, std::vector<double> error_step, int rank = -1);
  void WriteQualityMesh(MeshAdaptive* mesh, int step, std::vector<double> error_step, int rank = -1);
  void SaveErrorMesh(MeshAdaptive* mesh);
  void SaveMesh(std::unique_ptr<MeshAdaptive> mesh, int step);
  void SaveErrorMesh(MeshAdaptive* mesh, int step);
  void AdaptCurve(Geometry* geometry);
  void AdaptDomain(Geometry* geometry, MeshAdaptive* mesh);
  std::vector<double> ComputePatchAdaptationFactors(
      const Geometry* geometry);
  std::vector<double> SmoothPatchAdaptationFactors(
      const Geometry* geometry, const std::vector<double>& raw_factors);
  std::vector<double> ApplyAdaptationRateLimit(
      const std::vector<double>& target_factors);
  double ComputeCurveAdaptationFactor(
      CurveAdaptive* curve, const std::vector<double>& patch_factors) const;
  AdaptivePolicyRuntime BuildAdaptivePolicyRuntime() const;
  MeshQualityStats ComputeMeshQualityStats(const MeshAdaptive* mesh) const;
  void WritePatchAdaptationLog(int step, bool accepted) const;
  void WriteCurveAdaptationLog(int step, bool accepted) const;
  void WriteAcceptanceLog(int step, double score, const MeshQualityStats& stats,
                          double error_improvement, double element_growth_factor,
                          bool accepted) const;
  void WriteCompatibilitySummary(const Geometry* geometry) const;
  void WriteRuntimeSummary(const Timer* timer) const;
  void UpdatePatchHistory(const Geometry* geometry, const MeshAdaptive* mesh,
                          const std::vector<double>& patch_factors,
                          const std::vector<double>& current_errors,
                          bool accepted_step);
  void RegularizeEquivalentPatchBudgets(const Geometry* geometry);
  void RegularizeCurveTargetsFromPatchConsistency(const Geometry* geometry);
  void GeneratorInitialMesh(Geometry* geometry, MeshAdaptive* mesh, Timer* timer, int size_thread, int size_patch);
  void PrintElments(MeshAdaptive* mesh, int step, std::vector<double> error_step, int rank = -1);

#if USE_OPENMP
  virtual SubMesh* GeneratorInitialMeshOmp(PatchCoons* patch, Performer::IdManager* id_manager);
  virtual double CalculateErrorGlobalOmp(MeshAdaptive* mesh, Timer* timer, int rank = 0, int size_thread = 0);
  int GeneratorOmp(Model& model, Timer* timer, int id_range = 0, int sizeRank = 1, int sizeThread = 1);
  void AdaptCurveOmp(Geometry* geometry);
  void AdaptDomainOmp(Geometry* geometry, MeshAdaptive* mesh, Timer* timer, int size_thread, int size_patch);
#endif  // #USE_OPENMP

#if USE_MPI
  Model model_;
  Geometry* geometry_;
  PatchCoons* patch_;
#endif  // USE_MPI

  std::unique_ptr<MeshAdaptive> mesh_;

 protected:
#if (USE_MPI || USE_OPENMP)
  ApMeshCommunicator* communicator_;
#else
  std::unique_ptr<Parallel::TMCommunicator> communicator_;
#endif  // USE_MPI

  std::unique_ptr<Performer::IdManager> id_manager_;
  Performer::IdManagerVector id_managers_;
  mutable ULInt id_off_set_;
  ULInt id_range_;

 private:
  MeshVector save_mesh_;
  ErrorMeshVector save_error_mesh_;
  std::vector<double> error_step_;
  std::vector<double> last_submesh_error_;
  std::vector<double> previous_patch_factors_;
  // Para adaptive_stable: em passos rejeitados, guardamos o applied_patch_h
  // do último candidato rejeitado por patch. No passo seguinte, isso serve
  // para impedir "desrefino" (coarsen) visível em patches ligados a curvas.
  std::vector<double> last_candidate_patch_h_;
  // Em adaptive_stable, guardamos também o número de elementos do último
  // candidato rejeitado por patch para que os passos seguintes não assumam
  // erroneamente uma "base mais grossa" (o que leva a malhas complementares
  // inadequadas).
  std::vector<unsigned long long> last_candidate_elements_current_;
  std::vector<unsigned long long> elements_step_;
  std::vector<unsigned long long> gauss_nodes_step_;
  std::vector<unsigned long long> mean_nodes_step_;
  std::vector<unsigned int> retry_attempt_step_;
  std::vector<double> applied_relaxation_step_;
  std::vector<double> applied_max_delta_step_;
  std::vector<double> error_drop_abs_step_;
  std::vector<double> error_drop_pct_step_;
  std::vector<double> error_drop_per_element_step_;
  std::vector<double> element_growth_factor_step_;
  std::vector<double> error_normalized_step_;
  std::vector<double> error_total_reduction_pct_step_;
  std::vector<double> quality_min_step_;
  std::vector<double> quality_mean_step_;
  std::vector<double> poor_ratio_step_;
  std::vector<double> good_ratio_ge_0_60_step_;
  std::vector<double> angle_quality_mean_3d_step_;
  std::vector<double> edge_ratio_mean_3d_step_;
  std::vector<double> min_angle_p05_3d_step_;
  std::vector<double> max_angle_p95_3d_step_;
  std::vector<double> acceptance_score_step_;
  std::vector<double> element_budget_abs_step_;
  std::vector<double> element_budget_growth_factor_limit_step_;
  std::vector<double> element_budget_soft_limit_step_;
  std::vector<unsigned long long> candidate_elements_before_budget_retry_step_;
  std::vector<unsigned int> budget_retry_count_step_;
  std::vector<int> budget_limited_step_;
  std::vector<std::string> budget_final_status_step_;
  std::vector<std::string> step_mode_step_;
  std::vector<double> budget_pressure_step_;
  std::vector<unsigned int> eligible_patch_count_step_;
  std::vector<unsigned long long> eligible_patch_elements_sum_step_;
  std::vector<double> eligible_patch_mean_factor_step_;
  std::vector<double> eligible_patch_mean_h_scale_step_;
  std::vector<int> retry_enabled_step_;
  std::vector<int> local_budget_mode_step_;
  std::vector<double> target_element_growth_step_;
  std::vector<double> actual_element_growth_step_;
  std::vector<double> step_efficiency_step_;
  std::vector<std::string> generation_control_mode_step_;
  std::vector<double> target_elements_step_;
  std::vector<unsigned long long> actual_elements_step_;
  std::vector<double> target_growth_step_;
  std::vector<double> actual_growth_step_;
  std::vector<std::string> budget_driver_mode_step_;
  std::vector<double> patch_budget_dispersion_step_;
  std::vector<double> patch_h_dispersion_step_;
  std::vector<double> actual_patch_element_dispersion_step_;
  std::vector<double> curve_balance_dispersion_step_;
  std::vector<double> underresolved_transition_fraction_step_;
  std::vector<double> transition_imbalance_max_step_;
  std::vector<double> patch_consistency_penalty_step_;
  std::vector<int> consistency_regularization_active_step_;
  std::vector<double> local_progress_coverage_step_;
  std::vector<unsigned int> improved_top_patches_step_;
  std::vector<unsigned int> top_patch_candidate_count_step_;
  std::vector<std::string> step_purpose_step_;
  std::vector<PatchAdaptationSnapshot> current_patch_adaptation_;
  std::vector<CurveAdaptationSnapshot> current_curve_adaptation_;
  std::vector<PatchHistoryState> patch_history_;
  std::vector<double> last_domain_patch_factors_;
  std::vector<int> current_patch_eligible_mask_;
  std::vector<int> current_patch_saturated_soft_mask_;
  std::vector<int> current_patch_saturated_hard_mask_;
  unsigned int current_top_patch_count_ = 0U;
  bool current_budget_exceeded_ = false;
  unsigned int current_budget_retry_count_ = 0U;
  bool current_budget_limited_ = false;
  std::string current_budget_final_status_ = "not_applied";
  unsigned long long current_candidate_elements_before_budget_retry_ = 0ULL;
  double current_element_budget_abs_ = 0.0;
  double current_element_budget_growth_factor_limit_ = 0.0;
  double current_element_budget_soft_limit_ = 0.0;
  std::string current_step_mode_ = "normal";
  double current_budget_pressure_ = 0.0;
  unsigned int current_eligible_patch_count_ = 0U;
  unsigned long long current_eligible_patch_elements_sum_ = 0ULL;
  double current_eligible_patch_mean_factor_ = 0.0;
  double current_eligible_patch_mean_h_scale_ = 0.0;
  double current_target_element_growth_ = 0.0;
  double current_step_efficiency_ = 0.0;
  std::string current_generation_control_mode_ = "normal";
  double current_target_elements_step_ = 0.0;
  std::string current_budget_driver_mode_ = "factor_driven";
  double current_patch_budget_dispersion_ = 0.0;
  double current_patch_h_dispersion_ = 0.0;
  double current_actual_patch_element_dispersion_ = 0.0;
  double current_curve_balance_dispersion_ = 0.0;
  double current_curve_domain_penalty_ = 0.0;
  double current_transition_coherence_penalty_ = 0.0;
  double current_overshoot_ratio_ = 1.0;
  unsigned int current_underresolved_transition_patch_count_ = 0U;
  unsigned int current_underresolved_transition_patch_count_real_ = 0U;
  double current_underresolved_transition_fraction_real_ = 0.0;
  double current_transition_imbalance_mean_real_ = 0.0;
  double current_transition_imbalance_max_real_ = 0.0;
  std::string current_step_purpose_ = "normal";
  double current_patch_consistency_penalty_ = 0.0;
  int current_consistency_regularization_active_ = 0;
  AdaptivePolicyRuntime current_policy_;
  void DisableSharedCurveSynchronization(Geometry* geometry);
  double current_local_progress_coverage_ = 0.0;
  unsigned int current_improved_top_patches_ = 0U;
  unsigned int current_top_patch_candidate_count_ = 0U;
  std::vector<double> current_patch_target_elements_;
  std::vector<double> current_patch_target_h_;
  std::vector<int> current_patch_quadtree_depth_cap_;
  std::vector<double> current_curve_target_points_;
  Adapter adapter_;
  std::string stop_reason_;
  unsigned long long current_gauss_nodes_;
  unsigned long long current_mean_nodes_;
  double current_adaptation_relaxation_;
  double current_adaptation_max_delta_;

  int step_;
  /// Em adaptive_stable: rejeicoes seguidas sem mudar malha aceite; usado para nao
  /// repetir o mesmo candidato (mesmas entradas de erro/fatores) nos passos 3+.
  unsigned int stable_consecutive_reject_count_ = 0;
  double error_local_process_;
};

#endif  // GENERATOR_GENERATOR_ADAPTIVE_H
