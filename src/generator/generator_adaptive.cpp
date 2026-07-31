#include "../../include/generator/generator_adaptive.h"

#include "data/curve/curve_adaptive.h"
#include "data/curve/curve_adaptive_parametric_bezier.h"
#include "data/curve/curve_adaptive_parametric_hermite.h"
#include "data/geometry.h"

#include <ostream>
#include <cstdio>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <map>
#include <filesystem>
#include <stdexcept>
#include <system_error>

namespace {

unsigned long CheckedLegacyId(const ULInt id) {
  if (id > static_cast<ULInt>(std::numeric_limits<unsigned long>::max())) {
    throw std::overflow_error("generated identifier exceeds platform limit");
  }
  return static_cast<unsigned long>(id);
}

void EnsureOutputParentDirectories(const std::string& file_path) {
  const std::filesystem::path p(file_path);
  const std::filesystem::path parent = p.parent_path();
  if (parent.empty()) {
    return;
  }
  std::error_code ec;
  std::filesystem::create_directories(parent, ec);
}

// GeneratorInitialMeshOmp skips curves that already have discretization points
// (GetNumBerPoints() != 0): the first patch processed along a shared edge
// "owns" InsertPoint on that curve. Neighbors then only copy corner geometry
// into their SubMesh. Order must be deterministic; prefer patches with more
// true boundary edges (curve shared by a single patch) before interior caps.
int CountTrueBoundaryEdges(Geometry* geometry, int patch_index) {
  PatchCoons* patch =
      static_cast<PatchCoons*>(geometry->GetPatch(patch_index));
  if (patch == nullptr) return 0;
  int count = 0;
  for (unsigned int e = 0; e < 4; ++e) {
    CurveAdaptive* c = patch->GetCurve(e);
    if (c != nullptr && c->GetNumBerPatches() == 1) {
      ++count;
    }
  }
  return count;
}

std::vector<int> InitialMeshPatchProcessOrder(Geometry* geom, int size_patch) {
  std::vector<int> order(static_cast<size_t>(size_patch));
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [geom](int a, int b) {
    const int ba = CountTrueBoundaryEdges(geom, a);
    const int bb = CountTrueBoundaryEdges(geom, b);
    if (ba != bb) return ba > bb;
    return a < b;
  });
  return order;
}

CurveAdaptive* CloneCurveForPatch(const CurveAdaptive* curve, Patch* patch) {
  if (curve == nullptr) {
    return nullptr;
  }
  CurveAdaptive* clone = nullptr;
  if (const auto* bezier =
          dynamic_cast<const CurveAdaptiveParametricBezier*>(curve)) {
    clone = new CurveAdaptiveParametricBezier(
        const_cast<CurveAdaptiveParametricBezier*>(bezier));
  } else if (const auto* hermite =
                 dynamic_cast<const CurveAdaptiveParametricHermite*>(curve)) {
    clone = new CurveAdaptiveParametricHermite(
        const_cast<CurveAdaptiveParametricHermite*>(hermite));
  } else if (const auto* parametric =
                 dynamic_cast<const CurveAdaptiveParametric*>(curve)) {
    clone = new CurveAdaptiveParametric(
        const_cast<CurveAdaptiveParametric*>(parametric));
  } else {
    clone = new CurveAdaptive(const_cast<CurveAdaptive*>(curve));
  }
  clone->ClearPatches();
  if (patch != nullptr) {
    clone->InsertPatch(patch);
  }
  return clone;
}

double ComputeCompatibilityErrorForCurveGroup(
    const std::vector<const CurveAdaptive*>& group) {
  if (group.size() <= 1U) {
    return 0.0;
  }
  double accumulated = 0.0;
  unsigned long long comparisons = 0ULL;
  for (size_t i = 0; i < group.size(); ++i) {
    for (size_t j = i + 1; j < group.size(); ++j) {
      const CurveAdaptive* a = group[i];
      const CurveAdaptive* b = group[j];
      const unsigned int na = a ? a->GetNumBerPoints() : 0U;
      const unsigned int nb = b ? b->GetNumBerPoints() : 0U;
      if (na == 0U || nb == 0U) {
        continue;
      }
      const unsigned int samples = std::max(na, nb);
      for (unsigned int k = 0; k < samples; ++k) {
        const unsigned int ia =
            (samples <= 1U || na <= 1U)
                ? 0U
                : static_cast<unsigned int>(std::llround(
                      static_cast<double>(k) * static_cast<double>(na - 1U) /
                      static_cast<double>(samples - 1U)));
        const unsigned int ib =
            (samples <= 1U || nb <= 1U)
                ? 0U
                : static_cast<unsigned int>(std::llround(
                      static_cast<double>(k) * static_cast<double>(nb - 1U) /
                      static_cast<double>(samples - 1U)));
        const PointAdaptive* pa =
            const_cast<CurveAdaptive*>(a)->GetPoint(std::min(ia, na - 1U));
        const PointAdaptive* pb =
            const_cast<CurveAdaptive*>(b)->GetPoint(std::min(ib, nb - 1U));
        if (pa == nullptr || pb == nullptr) {
          continue;
        }
        const double dx = pa->GetX() - pb->GetX();
        const double dy = pa->GetY() - pb->GetY();
        const double dz = pa->GetZ() - pb->GetZ();
        accumulated += std::sqrt(dx * dx + dy * dy + dz * dz);
        ++comparisons;
      }
    }
  }
  return (comparisons > 0ULL)
             ? (accumulated / static_cast<double>(comparisons))
             : 0.0;
}

const char* TimerTypeName(int type) {
  switch (type) {
    case 0:
      return "initialization";
    case 1:
      return "load_estimate";
    case 2:
      return "initial_mesh";
    case 3:
      return "curve_update";
    case 4:
      return "patch_reconstruction";
    case 5:
      return "input_read";
    case 6:
      return "curvature_aux";
    case 7:
      return "error_evaluation";
    case 8:
      return "overhead";
    case 9:
      return "mpi_transfer";
    case 10:
      return "full_without_io";
    case 11:
      return "save_output";
    default:
      return "unknown";
  }
}

}  // namespace

extern int MAX_THREADS;
extern double SMOOTHING_LAPLACIAN_NUMBER;
extern double SMOOTHING_LAPLACIAN_FACTOR;
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
extern double QUADTREE_FACE_QUALITY_THRESHOLD;
extern double QUADTREE_LOW_QUALITY_H_FACTOR;
extern double TEMPLATE_POOR_SCORE_THRESHOLD;
extern int AFT_LOCAL_POSTPROCESS_PASSES;
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
extern int ADAPTIVE_CURRENT_STEP;
extern int ADAPTIVE_CURRENT_RETRY;

namespace {
double Clamp01(double value) {
  return std::max(0.0, std::min(1.0, value));
}

double ClampValue(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

double TriangleQualityFromNodes(const NodeAdaptive& a, const NodeAdaptive& b,
                                const NodeAdaptive& c) {
  const double abx = b.GetX() - a.GetX();
  const double aby = b.GetY() - a.GetY();
  const double abz = b.GetZ() - a.GetZ();
  const double bcx = c.GetX() - b.GetX();
  const double bcy = c.GetY() - b.GetY();
  const double bcz = c.GetZ() - b.GetZ();
  const double cax = a.GetX() - c.GetX();
  const double cay = a.GetY() - c.GetY();
  const double caz = a.GetZ() - c.GetZ();

  const double lab = std::sqrt(abx * abx + aby * aby + abz * abz);
  const double lbc = std::sqrt(bcx * bcx + bcy * bcy + bcz * bcz);
  const double lca = std::sqrt(cax * cax + cay * cay + caz * caz);
  const double perimeter = lab + lbc + lca;
  if (perimeter <= std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }

  const double cx = aby * caz - abz * cay;
  const double cy = abz * cax - abx * caz;
  const double cz = abx * cay - aby * cax;
  const double area = 0.5 * std::sqrt(cx * cx + cy * cy + cz * cz);
  if (area <= std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }

  const double r = (2.0 * area) / perimeter;
  const double R = (lab * lbc * lca) / (4.0 * area);
  if (R <= std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }

  return Clamp01((2.0 * r) / R);
}

double TriangleQualityFromParameters(const std::tuple<double, double>& a,
                                     const std::tuple<double, double>& b,
                                     const std::tuple<double, double>& c) {
  const double abx = std::get<0>(b) - std::get<0>(a);
  const double aby = std::get<1>(b) - std::get<1>(a);
  const double bcx = std::get<0>(c) - std::get<0>(b);
  const double bcy = std::get<1>(c) - std::get<1>(b);
  const double cax = std::get<0>(a) - std::get<0>(c);
  const double cay = std::get<1>(a) - std::get<1>(c);

  const double lab = std::sqrt(abx * abx + aby * aby);
  const double lbc = std::sqrt(bcx * bcx + bcy * bcy);
  const double lca = std::sqrt(cax * cax + cay * cay);
  const double perimeter = lab + lbc + lca;
  if (perimeter <= std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }

  const double twice_area =
      std::fabs(abx * (std::get<1>(c) - std::get<1>(a)) -
                aby * (std::get<0>(c) - std::get<0>(a)));
  const double area = 0.5 * twice_area;
  if (area <= std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }

  const double r = (2.0 * area) / perimeter;
  const double R = (lab * lbc * lca) / (4.0 * area);
  if (R <= std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }

  return Clamp01((2.0 * r) / R);
}

void AccumulateQualitySample(const double quality, const bool first_sample,
                             double& quality_min, double& quality_sum,
                             double& poor_count, double& good_count) {
  quality_sum += quality;
  quality_min = first_sample ? quality : std::min(quality_min, quality);
  if (quality < 0.30) {
    poor_count += 1.0;
  }
  if (quality >= 0.60) {
    good_count += 1.0;
  }
}

struct TriangleShapeMetrics {
  double min_angle_deg = 0.0;
  double max_angle_deg = 0.0;
  double angle_quality = 0.0;
  double edge_ratio = 0.0;
};

TriangleShapeMetrics ComputeTriangleShapeMetrics3D(const NodeAdaptive& a,
                                                   const NodeAdaptive& b,
                                                   const NodeAdaptive& c) {
  const double abx = b.GetX() - a.GetX();
  const double aby = b.GetY() - a.GetY();
  const double abz = b.GetZ() - a.GetZ();
  const double acx = c.GetX() - a.GetX();
  const double acy = c.GetY() - a.GetY();
  const double acz = c.GetZ() - a.GetZ();
  const double bcx = c.GetX() - b.GetX();
  const double bcy = c.GetY() - b.GetY();
  const double bcz = c.GetZ() - b.GetZ();

  const double lab = std::sqrt(abx * abx + aby * aby + abz * abz);
  const double lac = std::sqrt(acx * acx + acy * acy + acz * acz);
  const double lbc = std::sqrt(bcx * bcx + bcy * bcy + bcz * bcz);
  const double min_edge = std::min({lab, lac, lbc});
  const double max_edge = std::max({lab, lac, lbc});

  TriangleShapeMetrics metrics;
  if (min_edge <= std::numeric_limits<double>::epsilon() ||
      max_edge <= std::numeric_limits<double>::epsilon()) {
    return metrics;
  }

  auto safe_angle_deg = [](const double ux, const double uy, const double uz,
                           const double vx, const double vy, const double vz) {
    const double nu = std::sqrt(ux * ux + uy * uy + uz * uz);
    const double nv = std::sqrt(vx * vx + vy * vy + vz * vz);
    if (nu <= std::numeric_limits<double>::epsilon() ||
        nv <= std::numeric_limits<double>::epsilon()) {
      return 0.0;
    }
    double cs = (ux * vx + uy * vy + uz * vz) / (nu * nv);
    cs = ClampValue(cs, -1.0, 1.0);
    constexpr double kRadToDeg = 57.2957795130823208768;
    return std::acos(cs) * kRadToDeg;
  };

  const double angle_a = safe_angle_deg(abx, aby, abz, acx, acy, acz);
  const double angle_b = safe_angle_deg(-abx, -aby, -abz, bcx, bcy, bcz);
  const double angle_c = std::max(0.0, 180.0 - angle_a - angle_b);

  metrics.min_angle_deg = std::min({angle_a, angle_b, angle_c});
  metrics.max_angle_deg = std::max({angle_a, angle_b, angle_c});
  metrics.edge_ratio = Clamp01(min_edge / max_edge);
  metrics.angle_quality =
      Clamp01(std::min(metrics.min_angle_deg / 60.0,
                       (180.0 - metrics.max_angle_deg) / 120.0));
  return metrics;
}

double ComputeGrowthPenalty(const double element_growth_factor,
                            const int current_step) {
  const double growth_weight =
      (current_step <= 1) ? ACCEPTANCE_STEP1_GROWTH_WEIGHT
                          : ACCEPTANCE_STEPN_GROWTH_WEIGHT;
  const double soft_limit =
      (current_step <= 1) ? ACCEPTANCE_STEP1_MAX_GROWTH_SOFT
                          : ACCEPTANCE_STEPN_MAX_GROWTH_SOFT;
  const double growth_excess =
      std::max(0.0, element_growth_factor - std::max(1.0, soft_limit));
  if (ACCEPTANCE_GROWTH_PENALTY_MODE == 1) {
    return growth_weight * growth_excess * growth_excess;
  }
  return growth_weight * std::max(0.0, element_growth_factor - 1.0);
}

double SafeSqrtTargetH(const double patch_area, const double target_elements) {
  if (patch_area <= 0.0 || target_elements <= 0.0) {
    return -1.0;
  }
  return std::sqrt(std::max(1.0e-12, patch_area / target_elements));
}

double ClampTargetElements(const double target_elements) {
  double value = target_elements;
  if (STEP_ELEMENT_TARGET_MIN > 0.0) {
    value = std::max(value, STEP_ELEMENT_TARGET_MIN);
  }
  if (STEP_ELEMENT_TARGET_MAX > 0.0) {
    value = std::min(value, STEP_ELEMENT_TARGET_MAX);
  }
  return value;
}

int EstimateQuadtreeDepthCap(const double target_patch_h, const int step) {
  if (PATCH_QUADTREE_CAP_MODE != 1 || target_patch_h <= 0.0) {
    return -1;
  }
  double effective_h =
      std::max(1.0e-6, target_patch_h * PATCH_QUADTREE_MIN_H_SCALE);
  if (effective_h > 0.55) {
    effective_h = 0.42;
  }
  const double raw_depth = std::ceil(std::log2(1.0 / effective_h));
  const int structural_cap =
      (step <= 1) ? PATCH_QUADTREE_DEPTH_STEP1 : PATCH_QUADTREE_DEPTH_STEPN;
  return std::max(0, std::min(structural_cap, static_cast<int>(raw_depth)));
}

double RelativeDifference(const double a, const double b) {
  const double denom = std::max({1.0e-9, std::fabs(a), std::fabs(b)});
  return std::fabs(a - b) / denom;
}

double ComputePatchArea(const Patch* patch) {
  const SubMesh* sub_mesh = patch ? patch->GetSubMesh() : nullptr;
  if (!sub_mesh) {
    return 0.0;
  }
  double area = 0.0;
  for (unsigned int elem_idx = 0; elem_idx < sub_mesh->GetNumberElements();
       ++elem_idx) {
    const TriangleAdaptive* tri =
        static_cast<const TriangleAdaptive*>(sub_mesh->GetElement(elem_idx));
    if (tri) {
      area += tri->GetArea();
    }
  }
  return area;
}

unsigned int CountBoundaryCurves(const PatchCoons* patch) {
  if (!patch) {
    return 0U;
  }
  unsigned int boundary_curves = 0U;
  for (unsigned int c = 0; c < patch->GetNumBerCurves(); ++c) {
    CurveAdaptive* curve = const_cast<CurveAdaptive*>(patch->GetCurve(c));
    if (curve && curve->GetNumBerPatches() <= 1U) {
      ++boundary_curves;
    }
  }
  return boundary_curves;
}

double ComputeNormalizedDispersion(const std::vector<double>& values) {
  if (values.size() < 2) {
    return 0.0;
  }
  const auto minmax = std::minmax_element(values.begin(), values.end());
  const double vmax = *minmax.second;
  if (vmax <= 1.0e-9) {
    return 0.0;
  }
  return std::max(0.0, (*minmax.second - *minmax.first) / vmax);
}

template <typename T>
std::vector<T> ListToVector(const std::list<T>& values) {
  return std::vector<T>(values.begin(), values.end());
}

unsigned int ResolvePatchIndex(const Geometry* geometry, Patch* patch) {
  if (!geometry || !patch) {
    return std::numeric_limits<unsigned int>::max();
  }
  for (unsigned int i = 0; i < geometry->GetNumberPatches(); ++i) {
    if (geometry->GetPatch(i) == patch) {
      return i;
    }
  }
  const unsigned int patch_id = patch->GetId();
  if (patch_id < geometry->GetNumberPatches() &&
      geometry->GetPatch(patch_id) == patch) {
    return patch_id;
  }
  return std::numeric_limits<unsigned int>::max();
}
}  // namespace

GeneratorAdaptive::GeneratorAdaptive() {}

GeneratorAdaptive::AdaptivePolicyRuntime
GeneratorAdaptive::BuildAdaptivePolicyRuntime() const {
  AdaptivePolicyRuntime policy;
  policy.mode = ADAPTIVE_MODE.empty() ? "legacy" : ADAPTIVE_MODE;
  policy.stable_mode = (policy.mode == "adaptive_stable");
  policy.debug_mode = (policy.mode == "research_debug");
  policy.max_steps = policy.stable_mode ? ADAPTIVE_MAX_STEPS : STEPS;
  policy.intensity = ClampValue(ADAPTIVE_INTENSITY, 0.0, 1.0);
  policy.quality_priority = ClampValue(ADAPTIVE_QUALITY_PRIORITY, 0.0, 1.0);
  policy.step1_target_growth = 1.0 + 8.0 * policy.intensity;
  policy.stepn_target_growth = std::max(1.0, ADAPTIVE_TARGET_GROWTH);
  policy.step2_accept_quality_min = 0.16 + 0.10 * policy.quality_priority;
  policy.step2_accept_max_error_increase_abs =
      std::max(0.10, 1.10 - 0.80 * policy.quality_priority);
  policy.equivalent_patch_min_ratio = 0.55 + 0.20 * policy.quality_priority;
  return policy;
}

#if USE_MPI
int GeneratorAdaptive::Execute(char* argv[], Timer* timer, MPI_Status status)
#else
int GeneratorAdaptive::Execute(char* argv[], Timer* timer)
#endif  // USE_MPI
{
  for (char** arg = argv; *arg != nullptr; ++arg) {
    std::cout << "argv[]:" << *arg << std::endl;
  }

  Geometry geometry;

#if USE_MPI
  if (RANK_MPI == 0) {
    // estimativa e ordenação dos patches para carga de distribuiçao dos patches
    std::list<PatchBezier*> patches = EstimateChargeofPatches(geometry, timer);

    // distribuição dos patches entre os (n) processos de acordo com suas cargas
    // ordenadas
    patches = OrderPatchesDistribProcess(patches);

#if USE_PRINT_ESTIMATE
    // mostra a distribuição de patches para cada processo
    calculateEstimateProcessElements(SIZE_MPI, listBezierPt);
#endif  // USE_PRINT_ESTIMATE

    double patches_process0[patches.size() * 48];
    double patches_process[patches.size() * 48];

    int i = 0, izero = 0;

    for (int j = 0; j < SIZE_MPI; j++) {
      i = 0;

      for (std::list<PatchBezier*>::iterator it = patches.begin();
           it != patches.end(); it++) {
        if (j == (*it)->GetIdProcess()) {
          patches_process[i] = (*it)->GetPt00().GetX();
          i++;
          patches_process[i] = (*it)->GetPt00().GetY();
          i++;
          patches_process[i] = (*it)->GetPt00().GetZ();
          i++;
          patches_process[i] = (*it)->GetPt10().GetX();
          i++;
          patches_process[i] = (*it)->GetPt10().GetY();
          i++;
          patches_process[i] = (*it)->GetPt10().GetZ();
          i++;
          patches_process[i] = (*it)->GetPt20().GetX();
          i++;
          patches_process[i] = (*it)->GetPt20().GetY();
          i++;
          patches_process[i] = (*it)->GetPt20().GetZ();
          i++;
          patches_process[i] = (*it)->GetPt30().GetX();
          i++;
          patches_process[i] = (*it)->GetPt30().GetY();
          i++;
          patches_process[i] = (*it)->GetPt30().GetZ();
          i++;

          patches_process[i] = (*it)->GetPt01().GetX();
          i++;
          patches_process[i] = (*it)->GetPt01().GetY();
          i++;
          patches_process[i] = (*it)->GetPt01().GetZ();
          i++;
          patches_process[i] = (*it)->GetPt11().GetX();
          i++;
          patches_process[i] = (*it)->GetPt11().GetY();
          i++;
          patches_process[i] = (*it)->GetPt11().GetZ();
          i++;
          patches_process[i] = (*it)->GetPt21().GetX();
          i++;
          patches_process[i] = (*it)->GetPt21().GetY();
          i++;
          patches_process[i] = (*it)->GetPt21().GetZ();
          i++;
          patches_process[i] = (*it)->GetPt31().GetX();
          i++;
          patches_process[i] = (*it)->GetPt31().GetY();
          i++;
          patches_process[i] = (*it)->GetPt31().GetZ();
          i++;

          patches_process[i] = (*it)->GetPt02().GetX();
          i++;
          patches_process[i] = (*it)->GetPt02().GetY();
          i++;
          patches_process[i] = (*it)->GetPt02().GetZ();
          i++;
          patches_process[i] = (*it)->GetPt12().GetX();
          i++;
          patches_process[i] = (*it)->GetPt12().GetY();
          i++;
          patches_process[i] = (*it)->GetPt12().GetZ();
          i++;
          patches_process[i] = (*it)->GetPt22().GetX();
          i++;
          patches_process[i] = (*it)->GetPt22().GetY();
          i++;
          patches_process[i] = (*it)->GetPt22().GetZ();
          i++;
          patches_process[i] = (*it)->GetPt32().GetX();
          i++;
          patches_process[i] = (*it)->GetPt32().GetY();
          i++;
          patches_process[i] = (*it)->GetPt32().GetZ();
          i++;

          patches_process[i] = (*it)->GetPt03().GetX();
          i++;
          patches_process[i] = (*it)->GetPt03().GetY();
          i++;
          patches_process[i] = (*it)->GetPt03().GetZ();
          i++;
          patches_process[i] = (*it)->GetPt13().GetX();
          i++;
          patches_process[i] = (*it)->GetPt13().GetY();
          i++;
          patches_process[i] = (*it)->GetPt13().GetZ();
          i++;
          patches_process[i] = (*it)->GetPt23().GetX();
          i++;
          patches_process[i] = (*it)->GetPt23().GetY();
          i++;
          patches_process[i] = (*it)->GetPt23().GetZ();
          i++;
          patches_process[i] = (*it)->GetPt33().GetX();
          i++;
          patches_process[i] = (*it)->GetPt33().GetY();
          i++;
          patches_process[i] = (*it)->GetPt33().GetZ();
          i++;
        }
      }

      if (j == 0) {
        izero = i;

        for (int m = 0; m < i; ++m) {
          patches_process0[m] = patches_process[m];
        }

      } else {
        timer->InitTimerParallel(RANK_MPI, 0, 0);  // Send
        MPI_Send(&TIME_READ_FILE, 1, MPI_DOUBLE, j, TAG_TIME, MPI_COMM_WORLD);
        MPI_Send(&i, 1, MPI_INT, j, TAG_SIZE_OF_DOUBLE, MPI_COMM_WORLD);
        MPI_Send(&patches_process, i, MPI_DOUBLE, j, TAG_DOUBLE,
                 MPI_COMM_WORLD);
        timer->EndTimerParallel(RANK_MPI, 0, 0);  // Send
      }
    }

    // gerador de malha para o processo root
#if USE_OPENMP
    Generator(patches_process0, izero, timer, 1024, atoi(argv[1]),
              atoi(argv[2]));
#else
    Generator(patches_process0, izero, timer, 1024, atoi(argv[1]));
#endif

  } else {
    timer->InitTimerParallel(RANK_MPI, 0, 9);  // Recv

    MPI_Recv(&TIME_READ_FILE, 1, MPI_DOUBLE, 0, TAG_TIME, MPI_COMM_WORLD,
             &status);

    int size_patches = 0;
    MPI_Recv(&size_patches, 1, MPI_INT, 0, TAG_SIZE_OF_DOUBLE, MPI_COMM_WORLD,
             &status);

    double patches[size_patches];
    MPI_Recv(&patches, size_patches, MPI_DOUBLE, 0, TAG_DOUBLE, MPI_COMM_WORLD,
             &status);

    timer->EndTimerParallel(RANK_MPI, 0, 9);  // Recv

    // gerador de malha para o processo (n)
#if USE_OPENMP
    Generator(patches, size_patches, timer, 1024, atoi(argv[1]), atoi(argv[2]));
#else
    Generator(patches, size_patches, timer, 1024, atoi(argv[1]));
#endif
  }
#else

  Model model;
  PatchReader patch_reader;

  // Inclusão dos patches no Modelo
  // Models3d models3d;
  // model.SetGeometry(models3d.ModelPneu(geometry));

  if (argv[3]) {
    timer->InitTimerParallel(0, 0, 5);  // Leitura arquivo
    model.SetGeometry(patch_reader.ReaderFilePatches(&geometry, argv[3]));
    timer->EndTimerParallel(0, 0, 5);  // Leitura arquivo
  } else {
    Models3d models3d;
    model.SetGeometry(models3d.ModelBaseQuadrada(&geometry));
  }

#if USE_OPENMP
  Generator(model, timer, 1024, atoi(argv[1]), atoi(argv[2]));
#else
  Generator(model, timer, 1024);
#endif  // USE_OPENMP

#endif  // USE_MPI
  return 0;
}

#if USE_MPI
std::list<PatchBezier*> GeneratorAdaptive::EstimateChargeofPatches(
    Geometry* geometry, Timer* timer) {
  ChargeEstimateProcess* charge_estimate_process = new ChargeEstimateProcess();
  std::list<PatchBezier*> patches =
      charge_estimate_process->ChargeEstimate(geometry, timer);
  delete charge_estimate_process;

  return patches;
}

std::vector<CurveAdaptive*> GeneratorAdaptive::CreateVectorOfCurves(
    std::list<PatchBezier*> patches) {
  CurveAdaptive* curve;
  std::vector<CurveAdaptive*> curves;

  for (std::list<PatchBezier*>::iterator it = patches.begin();
       it != patches.end(); it++) {
    if (curves.size() > 0) {
      if (VerifyCurve((*it)->GetPt00(), (*it)->GetPt10(), (*it)->GetPt20(),
                      (*it)->GetPt30(), curves)) {
        curve = new CurveAdaptiveParametricBezier(
            (*it)->GetPt00(), (*it)->GetPt10(), (*it)->GetPt20(),
            (*it)->GetPt30());
        curves.push_back(curve);
      }
      if (VerifyCurve((*it)->GetPt30(), (*it)->GetPt31(), (*it)->GetPt32(),
                      (*it)->GetPt33(), curves)) {
        curve = new CurveAdaptiveParametricBezier(
            (*it)->GetPt30(), (*it)->GetPt31(), (*it)->GetPt32(),
            (*it)->GetPt33());
        curves.push_back(curve);
      }
      if (VerifyCurve((*it)->GetPt03(), (*it)->GetPt13(), (*it)->GetPt23(),
                      (*it)->GetPt33(), curves)) {
        curve = new CurveAdaptiveParametricBezier(
            (*it)->GetPt03(), (*it)->GetPt13(), (*it)->GetPt23(),
            (*it)->GetPt33());
        curves.push_back(curve);
      }
      if (VerifyCurve((*it)->GetPt00(), (*it)->GetPt01(), (*it)->GetPt02(),
                      (*it)->GetPt03(), curves)) {
        curve = new CurveAdaptiveParametricBezier(
            (*it)->GetPt00(), (*it)->GetPt01(), (*it)->GetPt02(),
            (*it)->GetPt03());
        curves.push_back(curve);
      }
    } else if (curves.size() == 0) {
      curve =
          new CurveAdaptiveParametricBezier((*it)->GetPt00(), (*it)->GetPt10(),
                                            (*it)->GetPt20(), (*it)->GetPt30());
      curves.push_back(curve);
      curve =
          new CurveAdaptiveParametricBezier((*it)->GetPt30(), (*it)->GetPt31(),
                                            (*it)->GetPt32(), (*it)->GetPt33());
      curves.push_back(curve);
      curve =
          new CurveAdaptiveParametricBezier((*it)->GetPt03(), (*it)->GetPt13(),
                                            (*it)->GetPt23(), (*it)->GetPt33());
      curves.push_back(curve);
      curve =
          new CurveAdaptiveParametricBezier((*it)->GetPt00(), (*it)->GetPt01(),
                                            (*it)->GetPt02(), (*it)->GetPt03());
      curves.push_back(curve);
    }
  }

  return curves;
}

std::list<PatchBezier*> GeneratorAdaptive::OrderPatchesDistribProcess(
    std::list<PatchBezier*> patches) {
  std::vector<std::pair<double, int> > process_pairs;
  std::list<PatchBezier*> patches_order;

  if (SIZE_MPI > 1) {
    for (int i = 0; i < SIZE_MPI; i++) {
      process_pairs.push_back(std::make_pair(0.0, i));
    }

    for (std::list<PatchBezier*>::iterator it = patches.begin();
         it != patches.end(); it++) {
      std::sort(process_pairs.begin(), process_pairs.end());

      (*it)->SetIdProcess(process_pairs.front().second);

      process_pairs.front().first += (*it)->GetNumberTriangle();
    }

    for (int i = 0; i < SIZE_MPI; i++) {
      for (std::list<PatchBezier*>::iterator it = patches.begin();
           it != patches.end(); it++) {
        if ((*it)->GetIdProcess() == i) {
          patches_order.push_back((*it));
        }
      }
    }

    return patches_order;

  } else {
    for (std::list<PatchBezier*>::iterator it = patches.begin();
         it != patches.end(); it++) {
      (*it)->SetIdProcess(0);
    }

    return patches;
  }
}

bool GeneratorAdaptive::VerifyCurve(PointAdaptive p0, PointAdaptive p1,
                                    PointAdaptive p2, PointAdaptive p3,
                                    std::vector<CurveAdaptive*> curves) {
  for (vector<CurveAdaptive*>::iterator it = curves.begin(); it != curves.end();
       it++) {
    if (static_cast<CurveAdaptiveParametricBezier*>(*it)->GetPoint0().
        operator==(p0) &&
        static_cast<CurveAdaptiveParametricBezier*>(*it)->GetPoint1().
        operator==(p1) &&
        static_cast<CurveAdaptiveParametricBezier*>(*it)->GetPoint2().
        operator==(p2) &&
        static_cast<CurveAdaptiveParametricBezier*>(*it)->GetPoint3().
        operator==(p3)) {
      return false;
    }
  }

  return true;
}

void GeneratorAdaptive::CalculateEstimateProcessElements(
    int size_process, std::list<PatchBezier*> patches) {
  double estimate[size_process];

  for (std::list<PatchBezier*>::iterator it = patches.begin();
       it != patches.end(); it++) {
    estimate[(*it)->GetIdProcess()] += (*it)->GetNumberTriangle();
  }

  for (int i = 0; i < SIZE_MPI; ++i) {
    cout << "Estimativa para o processo " << i << " = " << estimate[i] << endl;
  }
}

std::list<PatchBezier*>::iterator GeneratorAdaptive::GetIteratorListPatches(
    int size_patches, std::list<PatchBezier*> patches) {
  std::list<PatchBezier*>::iterator it = patches.begin();
  advance(it, size_patches);
  return it;
}

Geometry* GeneratorAdaptive::UnpakGeometry(double patches[], int size_patches) {
  Geometry* geometry = new Geometry;

  PointAdaptive* p00;
  PointAdaptive* p01;
  PointAdaptive* p02;
  PointAdaptive* p03;
  PointAdaptive* p10;
  PointAdaptive* p11;
  PointAdaptive* p12;
  PointAdaptive* p13;
  PointAdaptive* p20;
  PointAdaptive* p21;
  PointAdaptive* p22;
  PointAdaptive* p23;
  PointAdaptive* p30;
  PointAdaptive* p31;
  PointAdaptive* p32;
  PointAdaptive* p33;

  CurveAdaptive* patch_c1;
  CurveAdaptive* patch_c2;
  CurveAdaptive* patch_c3;
  CurveAdaptive* patch_c4;

  PatchBezier* patch_bezier;

  for (int i = 0; i < size_patches; i = i + 48) {
    p00 = new VertexAdaptive(patches[i], patches[i + 1], patches[i + 2]);
    p10 = new VertexAdaptive(patches[i + 3], patches[i + 4], patches[i + 5]);
    p20 = new VertexAdaptive(patches[i + 6], patches[i + 7], patches[i + 8]);
    p30 = new VertexAdaptive(patches[i + 9], patches[i + 10], patches[i + 11]);

    p01 = new VertexAdaptive(patches[i + 12], patches[i + 13], patches[i + 14]);
    p11 = new VertexAdaptive(patches[i + 15], patches[i + 16], patches[i + 17]);
    p21 = new VertexAdaptive(patches[i + 18], patches[i + 19], patches[i + 20]);
    p31 = new VertexAdaptive(patches[i + 21], patches[i + 22], patches[i + 23]);

    p02 = new VertexAdaptive(patches[i + 24], patches[i + 25], patches[i + 26]);
    p12 = new VertexAdaptive(patches[i + 27], patches[i + 28], patches[i + 29]);
    p22 = new VertexAdaptive(patches[i + 30], patches[i + 31], patches[i + 32]);
    p32 = new VertexAdaptive(patches[i + 33], patches[i + 34], patches[i + 35]);

    p03 = new VertexAdaptive(patches[i + 36], patches[i + 37], patches[i + 38]);
    p13 = new VertexAdaptive(patches[i + 39], patches[i + 40], patches[i + 41]);
    p23 = new VertexAdaptive(patches[i + 42], patches[i + 43], patches[i + 44]);
    p33 = new VertexAdaptive(patches[i + 45], patches[i + 46], patches[i + 47]);

    patch_c1 = new CurveAdaptiveParametricBezier(*p00, *p10, *p20, *p30);
    patch_c2 = new CurveAdaptiveParametricBezier(*p30, *p31, *p32, *p33);
    patch_c3 = new CurveAdaptiveParametricBezier(*p03, *p13, *p23, *p33);
    patch_c4 = new CurveAdaptiveParametricBezier(*p00, *p01, *p02, *p03);

    if (geometry->VerifyCurveGeometry(p00, p10, p20, p30) == nullptr) {
      patch_c1 = new CurveAdaptiveParametricBezier(*p00, *p10, *p20, *p30);
      geometry->InsertCurve(patch_c1);
    } else {
      patch_c1 = geometry->VerifyCurveGeometry(p00, p10, p20, p30);
    }

    if (geometry->VerifyCurveGeometry(p30, p31, p32, p33) == nullptr) {
      patch_c2 = new CurveAdaptiveParametricBezier(*p30, *p31, *p32, *p33);
      geometry->InsertCurve(patch_c2);
    } else {
      patch_c2 = geometry->VerifyCurveGeometry(p30, p31, p32, p33);
    }

    if (geometry->VerifyCurveGeometry(p03, p13, p23, p33) == nullptr) {
      patch_c3 = new CurveAdaptiveParametricBezier(*p03, *p13, *p23, *p33);
      geometry->InsertCurve(patch_c3);
    } else {
      patch_c3 = geometry->VerifyCurveGeometry(p03, p13, p23, p33);
    }

    if (geometry->VerifyCurveGeometry(p00, p01, p02, p03) == nullptr) {
      patch_c4 = new CurveAdaptiveParametricBezier(*p00, *p01, *p02, *p03);
      geometry->InsertCurve(patch_c4);
    } else {
      patch_c4 = geometry->VerifyCurveGeometry(p00, p01, p02, p03);
    }

    patch_bezier = new PatchBezier(patch_c1, patch_c2, patch_c3, patch_c4, *p11,
                                   *p21, *p12, *p22);
    patch_bezier->SetId(i / 48);

    geometry->InsertPatch(patch_bezier);
  }

  return geometry;
}

#endif

#if USE_MPI
void GeneratorAdaptive::Generator(double patches[], int size_patches,
                                  Timer* timer, int id_range,
                                  [[maybe_unused]] int size_rank,
                                  int size_thread)
#else
void GeneratorAdaptive::Generator(Model& model, Timer* timer, int id_range,
                                  [[maybe_unused]] int size_rank,
                                  int size_thread)
#endif
{
#if USE_MPI
  this->communicator_ = new ApMeshCommunicator(true);
  Geometry* geometry = UnpakGeometry(patches, size_patches);
#elif USE_OPENMP
  this->communicator_ = new ApMeshCommunicator(true);
  Geometry* geometry = model.GetGeometry();
#else
  this->communicator_ = std::make_unique<Parallel::TMCommunicator>(false);
  Geometry* geometry = model.GetGeometry();
#endif

  if (ENABLE_SHARED_CURVE_SYNC == 0) {
    DisableSharedCurveSynchronization(geometry);
  }

  int size_patch = geometry->GetNumberPatches();

  mesh_ = std::make_unique<MeshAdaptive>();
  mesh_->ResizeSubMeshAdaptiveByPosition(size_patch);

  this->id_manager_ = nullptr;
  this->id_off_set_ = 0;
  this->id_range_ = id_range;

  this->step_ = 0;
  this->stable_consecutive_reject_count_ = 0U;
  current_policy_ = BuildAdaptivePolicyRuntime();
 if (current_policy_.stable_mode) {
    BUDGET_DRIVEN_ADAPTATION = 1;
    STEP_ELEMENT_TARGET_MODE = 1;
    STEP_TARGET_ELEMENT_GROWTH_MODE = 1;
    PATCH_BUDGET_ALLOCATION_MODE = 1;
    PATCH_BUDGET_TOPK_ONLY_MODE = 0;
    CURVE_POINT_BUDGET_MODE = 1;
    PATCH_QUADTREE_CAP_MODE = 1;
    PATCH_GENERATION_CONTROL_MODE = 1;
    PATCH_GENERATION_TOP_RANK_ONLY_MODE = 0;
    PATCH_CONSISTENCY_MODE = 1;
    CURVE_CONSISTENCY_MODE = 1;
    STEP_ACCEPT_CONSISTENCY_MODE = 1;
    PATCH_STEPWISE_MODE = 1;
    STEP2_MICRO_REFINEMENT_MODE = 1;
    STEP2_HARD_SPATIAL_FILTER_MODE = 0;
    ADAPTIVE_RETRY_COUNT = std::max(1, ADAPTIVE_RETRY_COUNT);

    ADAPTATION_RELAXATION = 0.30;
    ADAPTATION_MAX_DELTA = 0.05;

    PATCH_FACTOR_MIN = 0.90;
    PATCH_FACTOR_MAX = 1.03;

    PATCH_REFINEMENT_STRENGTH = 0.14;
    PATCH_COARSENING_STRENGTH = 0.02;

    PATCH_TOP_ERROR_FRACTION = 1.0;
    PATCH_QUALITY_WEIGHT = 0.40;

    STEP2_TOP_PATCH_COUNT_MIN = 4;
    STEP2_TOP_PATCH_COUNT_MAX = 8;
    STEP2_NONELIGIBLE_FACTOR_MIN = 1.0;
    STEP2_NONELIGIBLE_FORCE_COARSEN = 0;

    QUADTREE_FACE_QUALITY_THRESHOLD = 0.40;
    QUADTREE_LOW_QUALITY_H_FACTOR = 0.85;
    TEMPLATE_POOR_SCORE_THRESHOLD = 0.45;
    AFT_LOCAL_POSTPROCESS_PASSES = 2;
}

  MAX_THREADS = size_thread =
      size_thread > static_cast<Parallel::TMCommunicator*>(this->communicator_.get())
                        ->getMaxThreads()
          ? static_cast<Parallel::TMCommunicator*>(this->communicator_.get())
                ->getMaxThreads()
          : size_thread;

  ptr_aux.resize(size_thread, nullptr);

if (this->id_managers_.size() < static_cast<size_t>(size_thread)) {
  this->id_managers_.resize(size_thread);
}

  stop_reason_ = "max_steps_reached";
  error_step_.clear();
  elements_step_.clear();
  gauss_nodes_step_.clear();
  mean_nodes_step_.clear();
  retry_attempt_step_.clear();
  applied_relaxation_step_.clear();
  applied_max_delta_step_.clear();
  error_drop_abs_step_.clear();
  error_drop_pct_step_.clear();
  error_drop_per_element_step_.clear();
  element_growth_factor_step_.clear();
  error_normalized_step_.clear();
  error_total_reduction_pct_step_.clear();
  quality_min_step_.clear();
  quality_mean_step_.clear();
  poor_ratio_step_.clear();
  good_ratio_ge_0_60_step_.clear();
  angle_quality_mean_3d_step_.clear();
  edge_ratio_mean_3d_step_.clear();
  min_angle_p05_3d_step_.clear();
  max_angle_p95_3d_step_.clear();
  acceptance_score_step_.clear();
  element_budget_abs_step_.clear();
  element_budget_growth_factor_limit_step_.clear();
  element_budget_soft_limit_step_.clear();
  candidate_elements_before_budget_retry_step_.clear();
  budget_retry_count_step_.clear();
  budget_limited_step_.clear();
  budget_final_status_step_.clear();
  step_mode_step_.clear();
  budget_pressure_step_.clear();
  eligible_patch_count_step_.clear();
  eligible_patch_elements_sum_step_.clear();
  eligible_patch_mean_factor_step_.clear();
  eligible_patch_mean_h_scale_step_.clear();
  retry_enabled_step_.clear();
  local_budget_mode_step_.clear();
  target_element_growth_step_.clear();
  actual_element_growth_step_.clear();
  step_efficiency_step_.clear();
  generation_control_mode_step_.clear();
  target_elements_step_.clear();
  actual_elements_step_.clear();
  target_growth_step_.clear();
  actual_growth_step_.clear();
  budget_driver_mode_step_.clear();
  patch_budget_dispersion_step_.clear();
  patch_h_dispersion_step_.clear();
  actual_patch_element_dispersion_step_.clear();
  curve_balance_dispersion_step_.clear();
  underresolved_transition_fraction_step_.clear();
  transition_imbalance_max_step_.clear();
  patch_consistency_penalty_step_.clear();
  consistency_regularization_active_step_.clear();
  local_progress_coverage_step_.clear();
  improved_top_patches_step_.clear();
  top_patch_candidate_count_step_.clear();
  step_purpose_step_.clear();
  current_patch_adaptation_.clear();
  current_curve_adaptation_.clear();
  patch_history_.assign(static_cast<size_t>(size_patch), PatchHistoryState{});
  last_domain_patch_factors_.assign(static_cast<size_t>(size_patch), 1.0);
  current_patch_eligible_mask_.assign(static_cast<size_t>(size_patch), 0);
  current_patch_saturated_soft_mask_.assign(static_cast<size_t>(size_patch), 0);
  current_patch_saturated_hard_mask_.assign(static_cast<size_t>(size_patch), 0);
  current_top_patch_count_ = 0U;
  current_budget_exceeded_ = false;
  current_budget_retry_count_ = 0U;
  current_budget_limited_ = false;
  current_budget_final_status_ = "not_applied";
  current_candidate_elements_before_budget_retry_ = 0ULL;
  current_element_budget_abs_ = 0.0;
  current_element_budget_growth_factor_limit_ = 0.0;
  current_element_budget_soft_limit_ = 0.0;
  current_step_mode_ = "normal";
  current_budget_pressure_ = 0.0;
  current_eligible_patch_count_ = 0U;
  current_eligible_patch_elements_sum_ = 0ULL;
  current_eligible_patch_mean_factor_ = 0.0;
  current_eligible_patch_mean_h_scale_ = 0.0;
  previous_patch_factors_.clear();
  current_gauss_nodes_ = 0ULL;
  current_mean_nodes_ = 0ULL;
  current_adaptation_relaxation_ = ADAPTATION_RELAXATION;
  current_adaptation_max_delta_ = ADAPTATION_MAX_DELTA;
  current_target_element_growth_ = 0.0;
  current_step_efficiency_ = 0.0;
  current_generation_control_mode_ = "normal";
  current_target_elements_step_ = 0.0;
  current_budget_driver_mode_ = "factor_driven";
  current_curve_domain_penalty_ = 0.0;
  current_transition_coherence_penalty_ = 0.0;
  current_overshoot_ratio_ = 1.0;
  current_underresolved_transition_patch_count_ = 0U;
  current_underresolved_transition_patch_count_real_ = 0U;
  current_underresolved_transition_fraction_real_ = 0.0;
  current_transition_imbalance_mean_real_ = 0.0;
  current_transition_imbalance_max_real_ = 0.0;
  current_step_purpose_ = "normal";
  current_local_progress_coverage_ = 0.0;
  current_improved_top_patches_ = 0U;
  current_top_patch_candidate_count_ = 0U;
  current_patch_target_elements_.clear();
  current_patch_target_h_.clear();
  current_patch_quadtree_depth_cap_.clear();
  current_curve_target_points_.clear();

  auto count_elements = [](const MeshAdaptive* mesh) -> unsigned long long {
    if (!mesh) {
      return 0ULL;
    }

    unsigned long long total = 0ULL;
    for (unsigned int idx = 0; idx < mesh->GetNumberSubMeshesAdaptive(); ++idx) {
      const SubMesh* sub = mesh->GetSubMeshAdaptiveByPosition(idx);
      if (sub) {
        total += static_cast<unsigned long long>(sub->GetNumberElements());
      }
    }
    return total;
  };

  // Gerar a malha inicial
#if USE_OPENMP
  GeneratorInitialMesh(geometry, mesh_.get(), timer, size_thread, size_patch);
#else
  this->id_managers_[0] = this->MakeIdManager(communicator_.get(), 0);
#if USE_MPI
  timer->InitTimerParallel(RANK_MPI, 0, 2);  // Malha inicial
#else
  timer->InitTimerParallel(0, 0, 2);    // Malha inicial
#endif  // USE_MPI

  GeneratorInitialMesh(geometry, mesh_.get(), timer, size_thread, size_patch);

#if USE_MPI
  timer->EndTimerParallel(RANK_MPI, 0, 2);   // Malha inicial
#else
  timer->EndTimerParallel(0, 0, 2);     // Malha inicial
#endif  // USE_MPI
#endif  // USE_OPENMP

  // Calcula o erro global para a malha inicial
#if USE_OPENMP
  if (size_patch > 1) {
    this->error_local_process_ =
        this->CalculateErrorGlobalOmp(mesh_.get(), timer, 0, size_thread);

  } else {
    this->error_local_process_ =
        this->ErrorGlobal(mesh_.get(), timer, 0, size_thread);
  }
#else
#if USE_MPI
  timer->InitTimerParallel(RANK_MPI, 0, 7);  // Calculo do erro Global
#else
  timer->InitTimerParallel(0, 0, 7);    // Calculo do erro Global
#endif  // USE_MPI
  this->error_local_process_ = this->ErrorGlobal(mesh_.get(), timer);
#if USE_MPI
  timer->EndTimerParallel(RANK_MPI, 0, 7);  // Calculo do erro Global
#else
  timer->EndTimerParallel(0, 0, 7);     // Calculo do erro Global
#endif  // USE_MPI
#endif  // USE_OPENMP

  this->error_step_.push_back(this->error_local_process_);
  this->elements_step_.push_back(count_elements(mesh_.get()));
  this->gauss_nodes_step_.push_back(current_gauss_nodes_);
  this->mean_nodes_step_.push_back(current_mean_nodes_);
  this->retry_attempt_step_.push_back(0U);
  this->applied_relaxation_step_.push_back(current_adaptation_relaxation_);
  this->applied_max_delta_step_.push_back(current_adaptation_max_delta_);
  this->error_drop_abs_step_.push_back(0.0);
  this->error_drop_pct_step_.push_back(0.0);
  this->error_drop_per_element_step_.push_back(0.0);
  this->element_growth_factor_step_.push_back(1.0);
  this->error_normalized_step_.push_back(1.0);
  this->error_total_reduction_pct_step_.push_back(0.0);
  const MeshQualityStats initial_quality = ComputeMeshQualityStats(mesh_.get());
  quality_min_step_.push_back(initial_quality.quality_min);
  quality_mean_step_.push_back(initial_quality.quality_mean);
  poor_ratio_step_.push_back(initial_quality.poor_ratio);
  good_ratio_ge_0_60_step_.push_back(initial_quality.good_ratio_ge_0_60);
  angle_quality_mean_3d_step_.push_back(initial_quality.angle_quality_mean_3d);
  edge_ratio_mean_3d_step_.push_back(initial_quality.edge_ratio_mean_3d);
  min_angle_p05_3d_step_.push_back(initial_quality.min_angle_p05_3d);
  max_angle_p95_3d_step_.push_back(initial_quality.max_angle_p95_3d);
  acceptance_score_step_.push_back(0.0);
  element_budget_abs_step_.push_back(0.0);
  element_budget_growth_factor_limit_step_.push_back(0.0);
  element_budget_soft_limit_step_.push_back(0.0);
  candidate_elements_before_budget_retry_step_.push_back(0ULL);
  budget_retry_count_step_.push_back(0U);
  budget_limited_step_.push_back(0);
  budget_final_status_step_.push_back("not_applied");
  step_mode_step_.push_back("initial");
  budget_pressure_step_.push_back(0.0);
  eligible_patch_count_step_.push_back(0U);
  eligible_patch_elements_sum_step_.push_back(0ULL);
  eligible_patch_mean_factor_step_.push_back(0.0);
  eligible_patch_mean_h_scale_step_.push_back(0.0);
  retry_enabled_step_.push_back(ADAPTIVE_RETRY_COUNT > 0 ? 1 : 0);
  local_budget_mode_step_.push_back(PATCH_LOCAL_ELEMENT_BUDGET_MODE);
  target_element_growth_step_.push_back(0.0);
  actual_element_growth_step_.push_back(1.0);
  step_efficiency_step_.push_back(0.0);
  generation_control_mode_step_.push_back("initial");
  target_elements_step_.push_back(static_cast<double>(elements_step_.back()));
  actual_elements_step_.push_back(elements_step_.back());
  target_growth_step_.push_back(1.0);
  actual_growth_step_.push_back(1.0);
  budget_driver_mode_step_.push_back("initial");
  patch_budget_dispersion_step_.push_back(0.0);
  patch_h_dispersion_step_.push_back(0.0);
  actual_patch_element_dispersion_step_.push_back(0.0);
  curve_balance_dispersion_step_.push_back(0.0);
  underresolved_transition_fraction_step_.push_back(0.0);
  transition_imbalance_max_step_.push_back(0.0);
  patch_consistency_penalty_step_.push_back(0.0);
  consistency_regularization_active_step_.push_back(0);
  local_progress_coverage_step_.push_back(0.0);
  improved_top_patches_step_.push_back(0U);
  top_patch_candidate_count_step_.push_back(0U);
  step_purpose_step_.push_back("initial");

#if USE_SAVE_ERRO_MESH
  SaveErrorMesh(malha, step_);
#endif  // USE_SAVE_ERRO_MESH

  double max_sub_error = 0.0;
  if (!last_submesh_error_.empty()) {
    max_sub_error = *std::max_element(last_submesh_error_.begin(),
                                      last_submesh_error_.end());
  }

#if USE_MPI
#if USE_PRINT_RESULTS
  cout << "*************** ERRO " << this->step_ << " rank " << RANK_MPI
       << " = " << this->error_local_process_ << endl;
#endif  // #if USE_PRINT_RESULTS
#else
#if USE_PRINT_RESULTS
  cout << "*************** ERRO " << this->step_ << " = "
       << this->error_local_process_ << endl;
  cout << "Resumo step " << this->step_ << ": elementos="
       << this->elements_step_.back() << ", erro_global="
       << this->error_local_process_ << ", erro_local_max=" << max_sub_error
       << ", quality_min=" << initial_quality.quality_min
       << ", quality_mean=" << initial_quality.quality_mean
       << ", poor_ratio=" << initial_quality.poor_ratio
       << ", nos_gauss=" << this->gauss_nodes_step_.back()
       << ", nos_mean=" << this->mean_nodes_step_.back()
       << endl;
#endif  // #if USE_PRINT_RESULTS
#endif  // USE_MPI

  UpdatePatchHistory(geometry, mesh_.get(),
                     std::vector<double>(static_cast<size_t>(size_patch), 1.0),
                     last_submesh_error_, true);
  timer->InitTimerParallel(0, 0, 11);
  if (WRITE_MESH == std::string("m") || WRITE_MESH == std::string("q") ||
      WRITE_MESH == std::string("h")) {
    SaveMesh(std::move(mesh_), step_);
  }
  WriteAcceptanceLog(step_, 0.0, initial_quality, 0.0, 1.0, true);
  timer->EndTimerParallel(0, 0, 11);

  // this->error_local_process_ = 1000;
  double prev_error = this->error_local_process_;
  int stagnant = 0;

  auto restore_geometry_submeshes = [this](Geometry* target_geometry,
                                           MeshAdaptive* source_mesh) {
    if (!target_geometry || !source_mesh) {
      return;
    }
    const unsigned int patch_count = target_geometry->GetNumberPatches();
    for (unsigned int patch_index = 0; patch_index < patch_count; ++patch_index) {
      target_geometry->GetPatch(patch_index)->SetSubMesh(
          source_mesh->GetSubMeshAdaptiveByPosition(patch_index));
    }
  };

  struct CurvePointSnapshot {
    unsigned long id;
    double x;
    double y;
    double z;
  };

  auto capture_curve_snapshots =
      [](Geometry* target_geometry)
          -> std::vector<std::vector<CurvePointSnapshot>> {
    std::vector<std::vector<CurvePointSnapshot>> snapshots;
    if (!target_geometry) {
      return snapshots;
    }
    snapshots.resize(target_geometry->GetNumberCurves());
    for (unsigned int curve_index = 0;
         curve_index < target_geometry->GetNumberCurves(); ++curve_index) {
      for (PointAdaptive* point :
           target_geometry->GetCurve(curve_index)->GetPoints()) {
        snapshots[curve_index].push_back(
            {point->GetId(), point->GetX(), point->GetY(), point->GetZ()});
      }
    }
    return snapshots;
  };

  auto restore_curve_snapshots =
      [](Geometry* target_geometry,
         const std::vector<std::vector<CurvePointSnapshot>>& snapshots) {
    if (!target_geometry ||
        snapshots.size() != target_geometry->GetNumberCurves()) {
      return;
    }

    std::unordered_set<PointAdaptive*> unique_points;
    for (unsigned int curve_index = 0;
         curve_index < target_geometry->GetNumberCurves(); ++curve_index) {
      for (PointAdaptive* point :
           target_geometry->GetCurve(curve_index)->GetPoints()) {
        unique_points.insert(point);
      }
    }
    for (PointAdaptive* point : unique_points) {
      delete point;
    }

    std::unordered_map<unsigned long, PointAdaptive*> point_by_id;
    for (unsigned int curve_index = 0;
         curve_index < target_geometry->GetNumberCurves(); ++curve_index) {
      std::list<PointAdaptive*> restored_points;
      for (const CurvePointSnapshot& point : snapshots[curve_index]) {
        PointAdaptive* restored = nullptr;
        auto it = point_by_id.find(point.id);
        if (it != point_by_id.end()) {
          restored = it->second;
        } else {
          restored = new NodeAdaptive(point.x, point.y, point.z, point.id);
          point_by_id[point.id] = restored;
        }
        restored_points.push_back(restored);
      }
      target_geometry->GetCurve(curve_index)->SetPoints(
          std::move(restored_points));
    }
  };

  const bool force_min_steps_for_logging = current_policy_.stable_mode;
  // Em modo estável, queremos pelo menos um conjunto mais longo de steps intermediários
  // para poder analisar casos em que step2/step3 são rejeitados mas steps seguintes
  // ainda geram malhas úteis (ex.: "maha_4", "maha_5" e também "maha_6").
  const int min_steps_for_logging = std::min(current_policy_.max_steps, 6);
  bool has_provisional_progress_state = false;
  unsigned long long provisional_previous_elements = 0ULL;
  double provisional_prev_error = 0.0;

  // Gerar malha enquanto o erro global for acima do erro desejado
  while (this->step_ < current_policy_.max_steps) {
    if (this->error_local_process_ <= EPSYLON) {
      stop_reason_ = "target_error_reached";
#if USE_PRINT_RESULTS
      cout << "Parando: erro global <= EPSYLON (" << EPSYLON << ") no step "
           << this->step_ << " valor " << this->error_local_process_ << endl;
#endif
      break;
    }
    if (max_sub_error <= TOL_LOCAL) {
      stop_reason_ = "local_error_threshold_reached";
#if USE_PRINT_RESULTS
      cout << "Parando: erro local máximo (" << max_sub_error
           << ") <= TOL_LOCAL (" << TOL_LOCAL << ") no step " << this->step_
           << endl;
#endif
      break;
    }
    // #if USE_MPI
    //         timer->InitTimerParallel(RANK_MPI,0,9); // SendRecv
    //         MPI_Allreduce(&this->error_local_process_,
    //         &this->error_local_process_, 1, MPI_DOUBLE, MPI_SUM,
    //         MPI_COMM_WORLD); timer->EndTimerParallel(RANK_MPI,0,9); //
    //         SendRecv this->error_local_process_ = this->error_local_process_
    //         / sizeRank;
    // #endif //USE_MPI

    this->step_++;

    MeshAdaptive* accepted_mesh =
        mesh_ ? mesh_.get()
              : (!save_mesh_.empty() ? save_mesh_.back().second.get() : nullptr);

    const std::vector<double> accepted_submesh_error = last_submesh_error_;
    const std::vector<double> accepted_patch_factors = previous_patch_factors_;
    const std::vector<std::vector<CurvePointSnapshot>> accepted_curve_snapshots =
        capture_curve_snapshots(geometry);
    const double base_relaxation = ADAPTATION_RELAXATION;
    const double base_max_delta = ADAPTATION_MAX_DELTA;
    bool accepted_step = false;
    unsigned int accepted_retry_attempt = 0U;
    double improvement = 0.0;
    double error_drop_abs = 0.0;
    unsigned long long previous_elements =
        elements_step_.empty() ? 0ULL : elements_step_.back();
    if (has_provisional_progress_state) {
      previous_elements = provisional_previous_elements;
      prev_error = provisional_prev_error;
    }
    unsigned long long current_elements = 0ULL;
    unsigned long long delta_elements = 0ULL;
    double error_drop_per_element = 0.0;
    double element_growth_factor = 1.0;
    double acceptance_score = -std::numeric_limits<double>::infinity();
    MeshQualityStats candidate_quality;
    const double previous_quality_min =
        quality_min_step_.empty() ? 0.0 : quality_min_step_.back();
      const double previous_poor_ratio =
          poor_ratio_step_.empty() ? 0.0 : poor_ratio_step_.back();
      const double previous_good_ratio =
          good_ratio_ge_0_60_step_.empty() ? 0.0 : good_ratio_ge_0_60_step_.back();
    std::vector<double> accepted_candidate_patch_factors = accepted_patch_factors;
    current_budget_exceeded_ = false;
    current_budget_retry_count_ = 0U;
    current_budget_limited_ = false;
    current_budget_final_status_ = "not_applied";
    current_candidate_elements_before_budget_retry_ = 0ULL;
    current_element_budget_abs_ = 0.0;
    current_element_budget_growth_factor_limit_ = 0.0;
    current_element_budget_soft_limit_ = 0.0;
    current_step_mode_ =
        current_policy_.stable_mode ? "iterating"
                                    : ((this->step_ <= 1) ? "normal"
                                                          : "micro_refinement");
    current_budget_pressure_ = 0.0;
    current_eligible_patch_count_ = 0U;
    current_eligible_patch_elements_sum_ = 0ULL;
    current_eligible_patch_mean_factor_ = 0.0;
    current_eligible_patch_mean_h_scale_ = 0.0;
    double stable_step1_budget_growth = current_policy_.step1_target_growth;
      if (current_policy_.stable_mode && this->step_ == 1 &&
          previous_elements > 0ULL && previous_elements <= 32ULL) {
        if (prev_error > 0.0 && prev_error < 0.35) {
          stable_step1_budget_growth =
              1.008 + 0.18 * current_policy_.intensity;
        } else if (prev_error > 0.0 && prev_error < 0.55) {
          stable_step1_budget_growth =
              1.03 + 0.45 * current_policy_.intensity;
        } else {
          stable_step1_budget_growth =
              1.10 + 0.90 * current_policy_.intensity;
        }
      }
    current_target_element_growth_ =
        (current_policy_.stable_mode && previous_elements > 0ULL)
            ? ((this->step_ == 1) ? stable_step1_budget_growth
                                  : current_policy_.stepn_target_growth)
            : ((STEP_TARGET_ELEMENT_GROWTH_MODE == 1 && previous_elements > 0ULL)
                   ? ((this->step_ <= 2) ? STEP2_TARGET_ELEMENT_GROWTH
                                         : STEP3PLUS_TARGET_ELEMENT_GROWTH)
                   : 0.0);
    if (BUDGET_DRIVEN_ADAPTATION == 1 && STEP_ELEMENT_TARGET_MODE == 1 &&
        previous_elements > 0ULL) {
      current_target_element_growth_ =
          current_policy_.stable_mode
              ? ((this->step_ == 1) ? stable_step1_budget_growth
                                    : current_policy_.stepn_target_growth)
              : ((this->step_ == 1)
                     ? STEP1_TARGET_ELEMENT_GROWTH
                     : ((this->step_ == 2) ? STEP2_TARGET_ELEMENT_GROWTH
                                           : STEP3PLUS_TARGET_ELEMENT_GROWTH));
      current_target_elements_step_ = ClampTargetElements(
          static_cast<double>(previous_elements) * current_target_element_growth_);
      current_budget_driver_mode_ = "budget_driven";
    } else {
      current_target_elements_step_ =
          static_cast<double>(previous_elements > 0ULL ? previous_elements : 0ULL);
      current_budget_driver_mode_ = "factor_driven";
    }
    // Congela crescimento só em passos iniciais; queremos permitir que step_ >= 4
    // volte a refinar para gerar malhas adicionais (maha_4/maha_5).
    if (current_policy_.stable_mode && this->step_ >= 2 && this->step_ <= 3 &&
        prev_error > 0.0 && prev_error < 0.12 && previous_elements > 0ULL &&
        geometry != nullptr && geometry->GetNumberPatches() <= 4U) {
      current_target_element_growth_ = 1.0;
      if (BUDGET_DRIVEN_ADAPTATION == 1 && STEP_ELEMENT_TARGET_MODE == 1) {
        current_target_elements_step_ = ClampTargetElements(
            static_cast<double>(previous_elements) * current_target_element_growth_);
      }
    }
    if (current_policy_.stable_mode && this->step_ >= 5 && previous_elements > 0ULL) {
      constexpr double kStableLateTargetGrowthCap = 1.10;
      current_target_element_growth_ =
          std::min(current_target_element_growth_, kStableLateTargetGrowthCap);
      if (BUDGET_DRIVEN_ADAPTATION == 1 && STEP_ELEMENT_TARGET_MODE == 1) {
        current_target_elements_step_ = ClampTargetElements(
            static_cast<double>(previous_elements) * current_target_element_growth_);
      }
    }
    if (current_policy_.stable_mode && this->step_ >= 3 && previous_elements > 0ULL &&
        stable_consecutive_reject_count_ > 0U) {
      const unsigned int r =
          std::min(10U, stable_consecutive_reject_count_);
      const double growth_damp =
          std::pow(0.87, static_cast<double>(r));
      const double step_tail =
          this->step_ > 3
              ? std::pow(0.93, static_cast<double>(this->step_ - 3))
              : 1.0;
      current_target_element_growth_ = std::max(
          1.0, current_target_element_growth_ * growth_damp * step_tail);
      if (BUDGET_DRIVEN_ADAPTATION == 1 && STEP_ELEMENT_TARGET_MODE == 1) {
        current_target_elements_step_ = ClampTargetElements(
            static_cast<double>(previous_elements) * current_target_element_growth_);
      }
    }
    if (current_policy_.stable_mode && previous_elements > 0ULL &&
        !good_ratio_ge_0_60_step_.empty()) {
      const double prev_good_ratio = good_ratio_ge_0_60_step_.back();
      const double prev_underresolved =
          underresolved_transition_fraction_step_.empty()
              ? 0.0
              : underresolved_transition_fraction_step_.back();
      const double prev_transition_imbalance =
          transition_imbalance_max_step_.empty()
              ? 0.0
              : transition_imbalance_max_step_.back();
      const double quality_gap =
          Clamp01((0.70 - prev_good_ratio) / 0.70);
      const double transition_pressure =
          std::max(prev_underresolved,
                   Clamp01(prev_transition_imbalance / 0.18));
      const double quality_growth_scale =
          ClampValue(1.0 - 0.28 * quality_gap - 0.22 * transition_pressure,
                     0.72, 1.0);
      current_target_element_growth_ =
          std::max(1.02, current_target_element_growth_ * quality_growth_scale);
      if (BUDGET_DRIVEN_ADAPTATION == 1 && STEP_ELEMENT_TARGET_MODE == 1) {
        current_target_elements_step_ = ClampTargetElements(
            static_cast<double>(previous_elements) * current_target_element_growth_);
      }
      if (prev_underresolved > 0.20 || prev_transition_imbalance > 0.14) {
        current_step_purpose_ = "transition_catch_up";
      } else if (prev_good_ratio < 0.70) {
        current_step_purpose_ = "interior_quality_recovery";
      } else {
        current_step_purpose_ = "curve_catch_up";
      }
    } else {
      current_step_purpose_ = "curve_catch_up";
    }
    current_patch_budget_dispersion_ = 0.0;
    current_patch_h_dispersion_ = 0.0;
    current_actual_patch_element_dispersion_ = 0.0;
    current_curve_balance_dispersion_ = 0.0;
    current_patch_consistency_penalty_ = 0.0;
    current_consistency_regularization_active_ = 0;
    current_step_efficiency_ = 0.0;
    current_local_progress_coverage_ = 0.0;
    current_improved_top_patches_ = 0U;
    current_top_patch_candidate_count_ = 0U;
    current_generation_control_mode_ =
        (this->step_ <= 1)
            ? "normal"
            : ((PATCH_GENERATION_CONTROL_MODE == 1) ? "micro_refinement"
                                                    : "normal");

    auto compute_current_error = [&]() {
#if USE_OPENMP
      if (size_patch > 1) {
        this->error_local_process_ =
            this->CalculateErrorGlobalOmp(mesh_.get(), timer, 0, size_thread);
      } else {
        this->error_local_process_ =
            this->ErrorGlobal(mesh_.get(), timer, 0, size_thread);
      }
#else
#if USE_MPI
      timer->InitTimerParallel(RANK_MPI, 0, 7);
#else
      timer->InitTimerParallel(0, 0, 7);
#endif
      this->error_local_process_ = this->ErrorGlobal(mesh_.get(), timer);
#if USE_MPI
      timer->EndTimerParallel(RANK_MPI, 0, 7);
#else
      timer->EndTimerParallel(0, 0, 7);
#endif
#endif
    };

    const bool stable_small_mesh_step1_retry =
        (current_policy_.stable_mode && this->step_ == 1 && previous_elements <= 32ULL);
    const int stable_small_mesh_extra_retries =
        (stable_small_mesh_step1_retry && prev_error > 0.0 && prev_error < 0.35)
            ? 0
            : (stable_small_mesh_step1_retry ? 1 : 0);
    const int effective_retry_limit =
        std::max(ADAPTIVE_RETRY_COUNT, stable_small_mesh_extra_retries);
    const double effective_retry_shrink =
        stable_small_mesh_step1_retry
            ? ClampValue(std::min(ADAPTIVE_RETRY_SHRINK, 0.70), 0.40, 0.95)
            : ADAPTIVE_RETRY_SHRINK;
    for (int retry = 0; retry <= effective_retry_limit; ++retry) {
      ADAPTIVE_CURRENT_STEP = this->step_;
      ADAPTIVE_CURRENT_RETRY = retry;
      current_budget_retry_count_ = static_cast<unsigned int>(retry);
      current_budget_limited_ = (retry > 0);
      const bool micro_refinement_active =
          (this->step_ > 1 && STEP2_MICRO_REFINEMENT_MODE == 1);
      if (current_policy_.stable_mode) {
        current_step_mode_ = "iterating";
        current_generation_control_mode_ = "adaptive_stable";
      } else {
        current_step_mode_ = micro_refinement_active ? "micro_refinement" : "normal";
        current_generation_control_mode_ =
            (this->step_ <= 1)
                ? "normal"
                : ((PATCH_GENERATION_CONTROL_MODE == 1)
                       ? (micro_refinement_active ? "micro_refinement"
                                                  : "budget_constrained")
                       : "normal");
      }
      if (current_step_purpose_ == "transition_catch_up") {
        current_generation_control_mode_ = "transition_catch_up";
      } else if (current_step_purpose_ == "interior_quality_recovery") {
        current_generation_control_mode_ = "interior_quality_recovery";
      }
      current_adaptation_relaxation_ =
          std::max(1.0e-3, base_relaxation * pow(effective_retry_shrink, retry));
      current_adaptation_max_delta_ =
          std::max(1.0e-3, base_max_delta * pow(effective_retry_shrink, retry));
      if (micro_refinement_active) {
        current_adaptation_relaxation_ *= STEP2_MICRO_RELAXATION_SCALE;
        current_adaptation_max_delta_ *= STEP2_MICRO_MAX_DELTA_SCALE;
      }
      if (current_policy_.stable_mode && this->step_ >= 5) {
        // ApplyAdaptationRateLimit só corre em AdaptDomain; AdaptCurve usa fatores
        // " crus" e ignorava estes escalares. Mantemos o escalonamento no domínio
        // e espelhamos o efeito nas curvas (ver AdaptCurve).
        constexpr double kStableLateAdaptScale = 0.52;
        current_adaptation_relaxation_ *= kStableLateAdaptScale;
        current_adaptation_max_delta_ *= kStableLateAdaptScale;
      }
      last_submesh_error_ = accepted_submesh_error;
      previous_patch_factors_ = accepted_patch_factors;
      restore_geometry_submeshes(geometry, accepted_mesh);
      restore_curve_snapshots(geometry, accepted_curve_snapshots);
      current_eligible_patch_count_ = 0U;
      current_eligible_patch_elements_sum_ = 0ULL;
      current_eligible_patch_mean_factor_ = 0.0;
      current_eligible_patch_mean_h_scale_ = 0.0;

#if USE_PRINT_RESULTS
      if (retry > 0) {
        cout << "Retry adaptativo step " << this->step_ << " tentativa "
             << retry << ": relaxation=" << current_adaptation_relaxation_
             << ", max_delta=" << current_adaptation_max_delta_ << endl;
      }
#endif

      mesh_ = std::make_unique<MeshAdaptive>();
      mesh_->ResizeSubMeshAdaptiveByPosition(geometry->GetNumberPatches());
#if USE_MPI
      timer->InitTimerParallel(RANK_MPI, 0, 3);
#else
      timer->InitTimerParallel(0, 0, 3);
#endif
      AdaptCurve(geometry);
#if USE_MPI
      timer->EndTimerParallel(RANK_MPI, 0, 3);
#else
      timer->EndTimerParallel(0, 0, 3);
#endif

#if USE_OPENMP
      AdaptDomainOmp(geometry, mesh_.get(), timer, size_thread, size_patch);
#else
#if USE_MPI
      timer->InitTimerParallel(RANK_MPI, 0, 4);
#else
      timer->InitTimerParallel(0, 0, 4);
#endif
      AdaptDomain(geometry, mesh_.get());
#if USE_MPI
      timer->EndTimerParallel(RANK_MPI, 0, 4);
#else
      timer->EndTimerParallel(0, 0, 4);
#endif
#endif

      current_elements = count_elements(mesh_.get());
      delta_elements =
          current_elements > previous_elements ? current_elements - previous_elements
                                               : 0ULL;
      element_growth_factor =
          previous_elements > 0ULL
              ? static_cast<double>(current_elements) /
                    static_cast<double>(previous_elements)
              : 1.0;
      const bool budget_active =
          (STEP_ELEMENT_BUDGET_MODE == 1 && this->step_ > 1 &&
           previous_elements > 0ULL &&
           (STEP_ELEMENT_BUDGET_ABS > 0.0 || STEP_ELEMENT_BUDGET_GROWTH_FACTOR > 0.0));
      current_element_budget_abs_ = budget_active ? STEP_ELEMENT_BUDGET_ABS : 0.0;
      current_element_budget_growth_factor_limit_ =
          budget_active ? STEP_ELEMENT_BUDGET_GROWTH_FACTOR : 0.0;
      double hard_budget_limit = 0.0;
      if (budget_active && STEP_ELEMENT_BUDGET_ABS > 0.0) {
        hard_budget_limit = STEP_ELEMENT_BUDGET_ABS;
      }
      if (budget_active && STEP_ELEMENT_BUDGET_GROWTH_FACTOR > 0.0) {
        const double growth_budget =
            static_cast<double>(previous_elements) * STEP_ELEMENT_BUDGET_GROWTH_FACTOR;
        hard_budget_limit =
            (hard_budget_limit > 0.0) ? std::min(hard_budget_limit, growth_budget)
                                      : growth_budget;
      }
      current_element_budget_soft_limit_ =
          (hard_budget_limit > 0.0)
              ? (hard_budget_limit * std::max(1.0, STEP_ELEMENT_BUDGET_SOFT_RATIO))
              : 0.0;
      current_budget_pressure_ =
          (current_element_budget_soft_limit_ > 0.0)
              ? (static_cast<double>(current_elements) /
                 current_element_budget_soft_limit_)
              : 0.0;
      if (retry == 0) {
        current_candidate_elements_before_budget_retry_ = current_elements;
      }
      for (unsigned int patch_index = 0;
           patch_index < geometry->GetNumberPatches() &&
           patch_index < current_patch_adaptation_.size();
           ++patch_index) {
        const SubMesh* current_submesh =
            mesh_->GetSubMeshAdaptiveByPosition(patch_index);
        const unsigned long long patch_elements =
            current_submesh ? current_submesh->GetNumberElements() : 0ULL;
        current_patch_adaptation_[patch_index].elements_current = patch_elements;
        current_patch_adaptation_[patch_index].elements_delta =
            patch_elements > current_patch_adaptation_[patch_index].elements_prev
                ? patch_elements -
                      current_patch_adaptation_[patch_index].elements_prev
                : 0ULL;
      }
      {
        std::vector<double> actual_elements_by_group;
        std::map<int, std::vector<double>> grouped_actuals;
        for (const auto& snapshot : current_patch_adaptation_) {
          if (snapshot.similar_patch_group >= 0) {
            grouped_actuals[snapshot.similar_patch_group].push_back(
                static_cast<double>(snapshot.elements_current));
          }
        }
        for (const auto& entry : grouped_actuals) {
          if (entry.second.size() >= 2) {
            actual_elements_by_group.push_back(
                ComputeNormalizedDispersion(entry.second));
          }
        }
        current_actual_patch_element_dispersion_ =
            ComputeNormalizedDispersion(actual_elements_by_group);
        current_patch_consistency_penalty_ +=
            0.15 * current_actual_patch_element_dispersion_;
      }
      if (current_eligible_patch_count_ > 0U) {
        current_eligible_patch_mean_factor_ /=
            static_cast<double>(current_eligible_patch_count_);
        current_eligible_patch_mean_h_scale_ /=
            static_cast<double>(current_eligible_patch_count_);
      }

      bool budget_exceeded = false;
      if (budget_active && current_element_budget_soft_limit_ > 0.0 &&
          static_cast<double>(current_elements) > current_element_budget_soft_limit_) {
        budget_exceeded = true;
        current_budget_exceeded_ = true;
        current_budget_limited_ = true;
        current_budget_final_status_ =
            (retry < effective_retry_limit) ? "retry_shrink" : "rejected";
#if USE_PRINT_RESULTS
        cout << "Tentativa rejeitada no step " << this->step_
             << " por budget de elementos: elementos=" << current_elements
             << " > limite_suave=" << current_element_budget_soft_limit_
             << " (limite_duro=" << hard_budget_limit << ")" << endl;
#endif
      }
      if (budget_exceeded) {
        compute_current_error();
        max_sub_error = 0.0;
        if (!last_submesh_error_.empty()) {
          max_sub_error = *std::max_element(last_submesh_error_.begin(),
                                            last_submesh_error_.end());
        }
        candidate_quality = ComputeMeshQualityStats(mesh_.get());
        improvement =
            (prev_error - this->error_local_process_) / std::max(prev_error, 1e-12);
        error_drop_abs = prev_error - this->error_local_process_;
        error_drop_per_element =
            delta_elements > 0ULL
                ? error_drop_abs / static_cast<double>(delta_elements)
                : error_drop_abs;
        const double quality_min_delta =
            candidate_quality.quality_min - previous_quality_min;
        const double poor_ratio_delta =
            previous_poor_ratio - candidate_quality.poor_ratio;
        acceptance_score =
            ACCEPTANCE_WEIGHT_ERROR * improvement +
            ACCEPTANCE_WEIGHT_QUALITY_MIN * quality_min_delta +
            ACCEPTANCE_WEIGHT_POOR_RATIO * poor_ratio_delta -
            ComputeGrowthPenalty(element_growth_factor, this->step_);
        continue;
      }

      bool rejected_by_growth = false;
      const bool allow_growth_reject =
          (effective_retry_limit > 0 && this->step_ > 1);
      if (allow_growth_reject && previous_elements > 0ULL &&
          MAX_RETRY_ELEMENT_GROWTH_FACTOR > 0.0) {
        if (element_growth_factor > MAX_RETRY_ELEMENT_GROWTH_FACTOR) {
          rejected_by_growth = true;
#if USE_PRINT_RESULTS
          cout << "Tentativa rejeitada no step " << this->step_
               << " por crescimento de elementos: fator="
               << element_growth_factor
               << " > limite=" << MAX_RETRY_ELEMENT_GROWTH_FACTOR << endl;
#endif
        }
      }
      if (allow_growth_reject && !rejected_by_growth &&
          MAX_RETRY_ELEMENT_GROWTH_ABS > 0.0 &&
          delta_elements >
              static_cast<unsigned long long>(MAX_RETRY_ELEMENT_GROWTH_ABS)) {
        rejected_by_growth = true;
#if USE_PRINT_RESULTS
        cout << "Tentativa rejeitada no step " << this->step_
             << " por delta de elementos: delta=" << delta_elements
             << " > limite=" << MAX_RETRY_ELEMENT_GROWTH_ABS << endl;
#endif
      }

      if (rejected_by_growth) {
        candidate_quality = ComputeMeshQualityStats(mesh_.get());
        this->error_local_process_ = prev_error + 1.0;
        improvement = -1.0;
        error_drop_abs = prev_error - this->error_local_process_;
        error_drop_per_element =
            delta_elements > 0ULL
                ? error_drop_abs / static_cast<double>(delta_elements)
                : error_drop_abs;
        const double quality_min_delta =
            candidate_quality.quality_min - previous_quality_min;
        const double poor_ratio_delta =
            previous_poor_ratio - candidate_quality.poor_ratio;
        acceptance_score =
            ACCEPTANCE_WEIGHT_ERROR * improvement +
            ACCEPTANCE_WEIGHT_QUALITY_MIN * quality_min_delta +
            ACCEPTANCE_WEIGHT_POOR_RATIO * poor_ratio_delta -
            ComputeGrowthPenalty(element_growth_factor, this->step_);
        continue;
      }

      compute_current_error();

      current_improved_top_patches_ = 0U;
      current_top_patch_candidate_count_ = 0U;
      current_local_progress_coverage_ = 0.0;
      if (this->step_ > 1) {
        for (size_t patch_index = 0;
             patch_index < current_patch_adaptation_.size() &&
             patch_index < patch_history_.size() &&
             patch_index < last_submesh_error_.size();
             ++patch_index) {
          const PatchAdaptationSnapshot& snapshot =
              current_patch_adaptation_[patch_index];
          if (snapshot.refinement_rank <= 0 ||
              snapshot.refinement_rank >
                  static_cast<int>(std::max(1U, current_top_patch_count_))) {
            continue;
          }
          ++current_top_patch_candidate_count_;
          if (patch_history_[patch_index].prev_patch_error >
              last_submesh_error_[patch_index]) {
            ++current_improved_top_patches_;
          }
        }
        if (current_top_patch_candidate_count_ > 0U) {
          current_local_progress_coverage_ =
              static_cast<double>(current_improved_top_patches_) /
              static_cast<double>(current_top_patch_candidate_count_);
        }
      }

      max_sub_error = 0.0;
      if (!last_submesh_error_.empty()) {
        max_sub_error = *std::max_element(last_submesh_error_.begin(),
                                          last_submesh_error_.end());
      }

      improvement =
          (prev_error - this->error_local_process_) / std::max(prev_error, 1e-12);
      error_drop_abs = prev_error - this->error_local_process_;
      error_drop_per_element =
          delta_elements > 0ULL
              ? error_drop_abs / static_cast<double>(delta_elements)
              : error_drop_abs;
      current_step_efficiency_ = error_drop_per_element;
      candidate_quality = ComputeMeshQualityStats(mesh_.get());
      const double quality_min_delta =
          candidate_quality.quality_min - previous_quality_min;
      const double poor_ratio_delta =
          previous_poor_ratio - candidate_quality.poor_ratio;
      const double good_ratio_delta =
          candidate_quality.good_ratio_ge_0_60 - previous_good_ratio;
      const double previous_angle_quality =
          angle_quality_mean_3d_step_.empty() ? 0.0 : angle_quality_mean_3d_step_.back();
      const double previous_edge_ratio =
          edge_ratio_mean_3d_step_.empty() ? 0.0 : edge_ratio_mean_3d_step_.back();
      const double previous_min_angle_p05 =
          min_angle_p05_3d_step_.empty() ? 0.0 : min_angle_p05_3d_step_.back();
      const double previous_max_angle_p95 =
          max_angle_p95_3d_step_.empty() ? 180.0 : max_angle_p95_3d_step_.back();
      const double acceptance_growth_weight =
          (this->step_ > 1 && STEP2_ACCEPT_SMALL_GAINS_MODE == 1)
              ? STEP2_ACCEPTANCE_GROWTH_WEIGHT
              : ACCEPTANCE_WEIGHT_GROWTH;
      const double growth_penalty =
          ComputeGrowthPenalty(element_growth_factor, this->step_);
      const double target_growth_miss =
          (BUDGET_DRIVEN_ADAPTATION == 1 && current_target_element_growth_ > 0.0)
              ? std::fabs(element_growth_factor - current_target_element_growth_)
              : 0.0;
      current_overshoot_ratio_ =
          (current_target_elements_step_ > 0.0)
              ? (static_cast<double>(current_candidate_elements_before_budget_retry_) /
                 current_target_elements_step_)
              : 1.0;
      current_underresolved_transition_patch_count_ = 0U;
      current_curve_domain_penalty_ = 0.0;
      for (const auto& curve_snapshot : current_curve_adaptation_) {
        if (curve_snapshot.geometric_target_points <= 0) {
          continue;
        }
        if (curve_snapshot.actual_points < curve_snapshot.geometric_target_points) {
          ++current_underresolved_transition_patch_count_;
          const double deficit =
              static_cast<double>(curve_snapshot.geometric_target_points -
                                  curve_snapshot.actual_points) /
              static_cast<double>(
                  std::max(1, curve_snapshot.geometric_target_points));
          current_curve_domain_penalty_ +=
              deficit * (curve_snapshot.sensitive_curve == 1 ? 1.5 : 1.0);
        }
      }
      if (!current_curve_adaptation_.empty()) {
        current_curve_domain_penalty_ /=
            static_cast<double>(current_curve_adaptation_.size());
      }
      const double overshoot_excess = std::max(0.0, current_overshoot_ratio_ - 1.0);
      const double underresolved_real =
          Clamp01(current_underresolved_transition_fraction_real_);
      const double imbalance_mean_real =
          Clamp01(current_transition_imbalance_mean_real_ / 0.35);
      const double curve_aux_penalty = Clamp01(current_curve_domain_penalty_);
      current_transition_coherence_penalty_ =
          0.55 * underresolved_real + 0.30 * imbalance_mean_real +
          0.25 * overshoot_excess + 0.10 * curve_aux_penalty;
      acceptance_score = ACCEPTANCE_WEIGHT_ERROR * improvement +
                         ACCEPTANCE_WEIGHT_QUALITY_MIN * quality_min_delta +
                         ACCEPTANCE_WEIGHT_POOR_RATIO * poor_ratio_delta +
                         0.40 * good_ratio_delta -
                         (growth_penalty *
                          (ACCEPTANCE_WEIGHT_GROWTH > 0.0
                               ? (acceptance_growth_weight /
                                  ACCEPTANCE_WEIGHT_GROWTH)
                               : acceptance_growth_weight)) -
                         ((BUDGET_DRIVEN_ADAPTATION == 1)
                              ? (STEP_ACCEPT_BUDGET_MISS_PENALTY *
                                 std::max(0.0, target_growth_miss -
                                                   STEP_ACCEPT_TARGET_GROWTH_TOLERANCE))
                              : 0.0);
      if (overshoot_excess > 0.0 && good_ratio_delta <= 0.0) {
        acceptance_score -= 0.35 * overshoot_excess;
      }
      if (current_policy_.stable_mode && this->step_ >= 2) {
        const double coherence_w =
            (this->step_ >= 4) ? 0.14 : 0.30;
        acceptance_score -= coherence_w * current_transition_coherence_penalty_;
      }
      if (STEP_ACCEPT_CONSISTENCY_MODE == 1 && this->step_ > 1) {
        acceptance_score -= STEP_ACCEPT_CONSISTENCY_PENALTY_WEIGHT *
                            current_patch_consistency_penalty_;
      }
      const bool error_within_tolerance =
          improvement >= (-ACCEPTANCE_MAX_ERROR_INCREASE_PCT / 100.0);
      const bool small_step_drop_ok =
          (this->step_ > 1 && STEP2_ACCEPT_SMALL_GAINS_MODE == 1)
              ? (error_drop_abs >= STEP2_MIN_ERROR_DROP_ABS &&
                 error_drop_per_element >= STEP2_MIN_ERROR_DROP_PER_ELEMENT)
              : true;
      const bool step1_override =
          (this->step_ == 1 && error_drop_abs >= ACCEPTANCE_STEP1_MIN_ERROR_DROP_ABS &&
           improvement * 100.0 >= ACCEPTANCE_STEP1_MIN_ERROR_DROP_PCT &&
           candidate_quality.poor_ratio <=
               std::max(ACCEPTANCE_POOR_RATIO_TARGET, previous_poor_ratio + 0.01));
      const bool budget_driven_step1_ok =
          (BUDGET_DRIVEN_ADAPTATION == 1 && this->step_ == 1)
              ? (error_drop_abs > 0.0 &&
                 candidate_quality.quality_mean + STEP_ACCEPT_QUALITY_DROP_TOLERANCE >=
                     quality_mean_step_.back() &&
                 (current_target_element_growth_ <= 0.0 ||
                  element_growth_factor <=
                      current_target_element_growth_ + STEP_ACCEPT_TARGET_GROWTH_TOLERANCE))
              : false;
      const bool efficiency_ok =
          (this->step_ <= 1 || STEP_ACCEPT_EFFICIENCY_MODE == 0)
              ? true
              : (error_drop_per_element >= STEP_ACCEPT_MIN_EFFICIENCY &&
                 element_growth_factor <= STEP_ACCEPT_MAX_ELEMENT_GROWTH_SOFT &&
                 candidate_quality.quality_mean + STEP_ACCEPT_QUALITY_DROP_TOLERANCE >=
                     quality_mean_step_.back());
      const bool target_growth_ok =
          (BUDGET_DRIVEN_ADAPTATION == 0 || current_target_element_growth_ <= 0.0 ||
           this->step_ <= 1)
              ? true
              : (target_growth_miss <= STEP_ACCEPT_TARGET_GROWTH_TOLERANCE);
      const bool step2_quality_stable =
          (this->step_ != 2 || BUDGET_DRIVEN_ADAPTATION == 0)
              ? true
              : (candidate_quality.quality_min >=
                     (current_policy_.stable_mode
                          ? current_policy_.step2_accept_quality_min
                          : 0.20) &&
                 candidate_quality.poor_ratio <=
                     std::max(0.30, previous_poor_ratio + 0.08) &&
                 current_eligible_patch_count_ >=
                     std::min(2U, std::max(1U, current_top_patch_count_)));
      const bool stable_step2_error_ok =
          (!current_policy_.stable_mode || this->step_ != 2)
              ? true
              : ((this->error_local_process_ - prev_error) <=
                 current_policy_.step2_accept_max_error_increase_abs);
      const bool step2_overshoot_ok =
          (!current_policy_.stable_mode || this->step_ != 2 ||
           current_target_elements_step_ <= 0.0)
              ? true
              : (current_overshoot_ratio_ <= 1.22 ||
                 (improvement > 0.15 &&
                  current_underresolved_transition_fraction_real_ <= 0.05 &&
                  current_transition_imbalance_max_real_ <= 0.12));
      const bool stable_progressive_overshoot_ok =
          (!current_policy_.stable_mode || this->step_ <= 2 ||
           current_target_elements_step_ <= 0.0)
              ? true
              : (current_overshoot_ratio_ <= 1.28 ||
                 (improvement > 0.20 &&
                  current_underresolved_transition_fraction_real_ <= 0.03 &&
                  current_transition_imbalance_max_real_ <= 0.10));
      const double transition_penalty_cap =
          (!current_policy_.stable_mode || this->step_ <= 1)
              ? 1.0
              : ((this->step_ >= 4 && current_policy_.stable_mode) ? 0.42
                                                                     : 0.25);
      const bool transition_coherence_ok =
          (!current_policy_.stable_mode || this->step_ <= 1)
              ? true
              : ((this->step_ != 2)
                     ? (current_transition_coherence_penalty_ <= transition_penalty_cap)
                     : (current_underresolved_transition_patch_count_real_ == 0U &&
                        current_transition_imbalance_max_real_ <= 0.12 &&
                        current_transition_coherence_penalty_ <= 0.20));
      const bool consistency_ok =
          (STEP_ACCEPT_CONSISTENCY_MODE == 0 || this->step_ <= 1)
              ? true
              : ((this->step_ != 2) ||
                 (current_patch_budget_dispersion_ <=
                  STEP_ACCEPT_MAX_GROUP_DISPERSION_STEP2));
      if (this->step_ >= 3 && BUDGET_DRIVEN_ADAPTATION == 1) {
        acceptance_score += 0.30 * (current_local_progress_coverage_ - 0.5);
      }
      const bool local_progress_ok =
          (this->step_ <= 2 || BUDGET_DRIVEN_ADAPTATION == 0)
              ? true
              : (current_policy_.stable_mode && this->step_ >= 4)
                    ? (current_local_progress_coverage_ >= 0.20 ||
                       current_improved_top_patches_ >= 1U ||
                       improvement > -0.05)
                    : (current_local_progress_coverage_ >= 0.35 ||
                       current_improved_top_patches_ >= 2U);
      // Modo 0 exige improvement>=0; em passos intermediarios (ex.: ribbed com muitas
      // patches) o candidato pode subir ~20% o indicador ao refinar sem "melhoria" >= 0,
      // deixando a malha aceite estagnada no passo 0 ate explore no passo 4.
      const bool stable_mid_explore_accept =
          (ACCEPTANCE_MODE == 0) && current_policy_.stable_mode &&
          this->step_ >= 2 && this->step_ <= 3 && prev_error > 1.0e-30 &&
          this->error_local_process_ <= prev_error * 1.24 &&
          candidate_quality.quality_min + 1.0e-9 >= previous_quality_min * 0.68 &&
          candidate_quality.poor_ratio <=
              std::max(0.70, previous_poor_ratio + 0.28) &&
          element_growth_factor <= 3.55 && element_growth_factor >= 1.02;
      // Modo 0 exige improvement>=0; em passos tardios do adaptive_stable o refinamento
      // pode subir ligeiramente o indicador de erro enquanto a malha ganha elementos.
      const bool stable_late_explore_accept =
          (ACCEPTANCE_MODE == 0) && current_policy_.stable_mode &&
          this->step_ >= 4 && prev_error > 1.0e-30 &&
          this->error_local_process_ <= prev_error * 1.28 &&
          candidate_quality.quality_min + 1.0e-9 >= previous_quality_min * 0.75 &&
          candidate_quality.poor_ratio <=
              std::max(0.65, previous_poor_ratio + 0.22);
      const bool stable_transition_ready =
          (!current_policy_.stable_mode || this->step_ <= 1)
              ? true
              : (current_underresolved_transition_fraction_real_ <=
                     ((this->step_ >= 4) ? 0.25 : 0.10) &&
                 current_transition_imbalance_max_real_ <=
                     ((this->step_ >= 4) ? 0.18 : 0.12));
      const bool quality_target_hit =
          candidate_quality.good_ratio_ge_0_60 >= 0.70;
      const bool quality_trending_ok =
          candidate_quality.good_ratio_ge_0_60 >= previous_good_ratio + 0.04 &&
          current_underresolved_transition_fraction_real_ <=
              std::max(0.20, underresolved_transition_fraction_step_.empty()
                                 ? 0.20
                                 : underresolved_transition_fraction_step_.back());
      const bool shape_target_hit =
          candidate_quality.angle_quality_mean_3d >= 0.46 &&
          candidate_quality.edge_ratio_mean_3d >= 0.46 &&
          candidate_quality.min_angle_p05_3d >= 15.0 &&
          candidate_quality.max_angle_p95_3d <= 128.0;
      const bool shape_trending_ok =
          candidate_quality.angle_quality_mean_3d >= previous_angle_quality - 0.05 &&
          candidate_quality.edge_ratio_mean_3d >= previous_edge_ratio - 0.05 &&
          candidate_quality.min_angle_p05_3d >= previous_min_angle_p05 - 3.0 &&
          candidate_quality.max_angle_p95_3d <= previous_max_angle_p95 + 7.0;
      const bool hybrid_shape_gate_ok =
          (this->step_ <= 1)
              ? true
              : ((shape_target_hit || shape_trending_ok) &&
                 candidate_quality.min_angle_p05_3d >= 13.0 &&
                 candidate_quality.max_angle_p95_3d <= 132.0);
      const bool quality_gate_ok =
          (!current_policy_.stable_mode || this->step_ <= 1)
              ? true
              : ((quality_target_hit || quality_trending_ok || shape_target_hit) &&
                 hybrid_shape_gate_ok);
      const bool score_ok =
          (ACCEPTANCE_MODE == 0)
              ? ((improvement >= 0.0 || stable_mid_explore_accept ||
                  stable_late_explore_accept) &&
                 transition_coherence_ok && local_progress_ok && consistency_ok &&
                 stable_transition_ready && quality_gate_ok)
              : ((error_within_tolerance && small_step_drop_ok &&
                  error_drop_abs >= STEP_ACCEPT_ERROR_DROP_MIN &&
                  acceptance_score >= ACCEPTANCE_MIN_SCORE && efficiency_ok &&
                  target_growth_ok && local_progress_ok && step2_quality_stable &&
                  consistency_ok && stable_step2_error_ok &&
                  step2_overshoot_ok && stable_progressive_overshoot_ok &&
                  transition_coherence_ok && quality_gate_ok) ||
                 step1_override || budget_driven_step1_ok);
      const bool small_mesh_progress_override =
          (!current_policy_.stable_mode) && previous_elements > 0ULL &&
          previous_elements <= 32ULL && this->step_ <= 3 &&
          element_growth_factor <= 3.1 &&
          this->error_local_process_ <= prev_error * 2.0 &&
          candidate_quality.quality_min >= 0.04 &&
          candidate_quality.poor_ratio <= 0.35 &&
          current_underresolved_transition_fraction_real_ <= 0.35;

      if (score_ok || small_mesh_progress_override) {
        accepted_step = true;
        accepted_retry_attempt = static_cast<unsigned int>(retry);
        current_budget_final_status_ =
            (this->step_ > 1 && STEP_ELEMENT_BUDGET_MODE == 1)
                ? ((retry > 0) ? "accepted_after_retry" : "within_budget")
                : "not_applied";
        accepted_candidate_patch_factors.resize(current_patch_adaptation_.size(), 1.0);
        for (size_t patch_index = 0; patch_index < current_patch_adaptation_.size();
             ++patch_index) {
          accepted_candidate_patch_factors[patch_index] =
              current_patch_adaptation_[patch_index].limited_factor;
        }
        break;
      }
    }

    if (!accepted_step) {
      if (current_policy_.stable_mode) {
        ++stable_consecutive_reject_count_;
      }
      if (current_budget_exceeded_) {
        stop_reason_ = "step_element_budget_exceeded";
      } else if (this->error_local_process_ < prev_error) {
        // Erro global desceu, mas score/orcamento/qualidade nao passaram no hibrido.
        stop_reason_ = "hybrid_criterion_rejected";
      } else {
        stop_reason_ = "global_error_increased";
      }
      if (!current_budget_exceeded_) {
        current_budget_final_status_ = "rejected";
      }
#if USE_PRINT_RESULTS
      if (this->error_local_process_ < prev_error) {
        cout << "Parando: candidato do step " << this->step_
             << " reduziu o erro global de " << prev_error << " para "
             << this->error_local_process_
             << ", mas foi rejeitado pelo criterio hibrido"
             << " (score=" << acceptance_score
             << ", crescimento_elementos=" << element_growth_factor << ")"
             << endl;
      } else {
        cout << "Parando: erro global aumentou de " << prev_error << " para "
             << this->error_local_process_ << " no step " << this->step_
             << " (score=" << acceptance_score
             << ", crescimento_elementos=" << element_growth_factor << ")"
             << endl;
      }
      if (mesh_ && (WRITE_MESH == std::string("m") || WRITE_MESH == std::string("q") ||
                    WRITE_MESH == std::string("h"))) {
        cout << "Salvando malha candidata rejeitada para analise no step "
             << this->step_ << endl;
      }
#endif
      if (mesh_ && (WRITE_MESH == std::string("m") || WRITE_MESH == std::string("q") ||
                    WRITE_MESH == std::string("h"))) {
        timer->InitTimerParallel(0, 0, 11);
        SaveMesh(std::move(mesh_), step_);
        timer->EndTimerParallel(0, 0, 11);
      }
      timer->InitTimerParallel(0, 0, 11);
      WritePatchAdaptationLog(step_, false);
      WriteCurveAdaptationLog(step_, false);
      WriteAcceptanceLog(step_, acceptance_score, candidate_quality, improvement,
                         element_growth_factor, false);
      timer->EndTimerParallel(0, 0, 11);
      last_submesh_error_ = accepted_submesh_error;
      previous_patch_factors_ = accepted_patch_factors;
      current_adaptation_relaxation_ = base_relaxation;
      current_adaptation_max_delta_ = base_max_delta;
      // adaptive_stable: guarda por patch a discretização do último candidato
      // rejeitado para evitar "desrefino" visível em patches curvos no passo
      // seguinte (stable_consecutive_reject_count_>0).
      if (current_policy_.stable_mode && this->step_ >= 3) {
        if (last_candidate_patch_h_.size() != current_patch_adaptation_.size()) {
          last_candidate_patch_h_.assign(current_patch_adaptation_.size(), -1.0);
        }
        if (last_candidate_elements_current_.size() != current_patch_adaptation_.size()) {
          last_candidate_elements_current_.assign(current_patch_adaptation_.size(), 0ULL);
        }
        for (size_t patch_index = 0; patch_index < current_patch_adaptation_.size(); ++patch_index) {
          last_candidate_patch_h_[patch_index] = current_patch_adaptation_[patch_index].applied_patch_h;
          last_candidate_elements_current_[patch_index] =
              current_patch_adaptation_[patch_index].elements_current;
        }
      }
      // Sempre restaura para o último estado aceito após rejeição.
      // Isso evita que candidatos rejeitados "contaminem" os passos seguintes.
      restore_geometry_submeshes(geometry, accepted_mesh);
      restore_curve_snapshots(geometry, accepted_curve_snapshots);
      if (force_min_steps_for_logging && this->step_ < min_steps_for_logging) {
        if (current_policy_.stable_mode && previous_elements > 0ULL &&
            previous_elements <= 64ULL && current_elements > 0ULL) {
          has_provisional_progress_state = true;
          provisional_previous_elements = previous_elements;
          provisional_prev_error = prev_error;
        }
#if USE_PRINT_RESULTS
        cout << "Continuando apos rejeicao para registrar passos intermediarios "
             << "(step " << this->step_ << " de " << min_steps_for_logging << ")"
             << endl;
#endif
        continue;
      }
      if (this->step_ < current_policy_.max_steps) {
#if USE_PRINT_RESULTS
        cout << "Continuando apos rejeicao: step " << this->step_
             << " < max_steps " << current_policy_.max_steps
             << " (tentar refinamento seguinte)" << endl;
#endif
        continue;
      }
      break;
    }

    stable_consecutive_reject_count_ = 0U;

    if (improvement < MIN_IMPROVEMENT) {
      ++stagnant;
    } else {
      stagnant = 0;
    }

    this->error_step_.push_back(this->error_local_process_);
    this->elements_step_.push_back(current_elements);
    this->gauss_nodes_step_.push_back(current_gauss_nodes_);
    this->mean_nodes_step_.push_back(current_mean_nodes_);
    this->retry_attempt_step_.push_back(accepted_retry_attempt);
    this->applied_relaxation_step_.push_back(current_adaptation_relaxation_);
    this->applied_max_delta_step_.push_back(current_adaptation_max_delta_);
    this->error_drop_abs_step_.push_back(error_drop_abs);
    this->error_drop_pct_step_.push_back(improvement * 100.0);
    this->error_drop_per_element_step_.push_back(error_drop_per_element);
    this->element_growth_factor_step_.push_back(element_growth_factor);
    const double initial_error =
        !this->error_step_.empty() ? this->error_step_.front() : this->error_local_process_;
    const double normalized_error =
        initial_error > 0.0 ? this->error_local_process_ / initial_error : 0.0;
    this->error_normalized_step_.push_back(normalized_error);
    this->error_total_reduction_pct_step_.push_back(
        initial_error > 0.0
            ? ((initial_error - this->error_local_process_) / initial_error) *
                  100.0
            : 0.0);
    quality_min_step_.push_back(candidate_quality.quality_min);
    quality_mean_step_.push_back(candidate_quality.quality_mean);
    poor_ratio_step_.push_back(candidate_quality.poor_ratio);
    good_ratio_ge_0_60_step_.push_back(candidate_quality.good_ratio_ge_0_60);
    angle_quality_mean_3d_step_.push_back(candidate_quality.angle_quality_mean_3d);
    edge_ratio_mean_3d_step_.push_back(candidate_quality.edge_ratio_mean_3d);
    min_angle_p05_3d_step_.push_back(candidate_quality.min_angle_p05_3d);
    max_angle_p95_3d_step_.push_back(candidate_quality.max_angle_p95_3d);
    acceptance_score_step_.push_back(acceptance_score);
    step_mode_step_.push_back(current_step_mode_);
    budget_pressure_step_.push_back(current_budget_pressure_);
    eligible_patch_count_step_.push_back(current_eligible_patch_count_);
    eligible_patch_elements_sum_step_.push_back(current_eligible_patch_elements_sum_);
    eligible_patch_mean_factor_step_.push_back(current_eligible_patch_mean_factor_);
    eligible_patch_mean_h_scale_step_.push_back(current_eligible_patch_mean_h_scale_);
    element_budget_abs_step_.push_back(current_element_budget_abs_);
    element_budget_growth_factor_limit_step_.push_back(
        current_element_budget_growth_factor_limit_);
    element_budget_soft_limit_step_.push_back(current_element_budget_soft_limit_);
    candidate_elements_before_budget_retry_step_.push_back(
        current_candidate_elements_before_budget_retry_);
    budget_retry_count_step_.push_back(current_budget_retry_count_);
    budget_limited_step_.push_back(current_budget_limited_ ? 1 : 0);
    budget_final_status_step_.push_back(current_budget_final_status_);
    retry_enabled_step_.push_back(ADAPTIVE_RETRY_COUNT > 0 ? 1 : 0);
    local_budget_mode_step_.push_back(PATCH_LOCAL_ELEMENT_BUDGET_MODE);
    target_element_growth_step_.push_back(current_target_element_growth_);
    actual_element_growth_step_.push_back(element_growth_factor);
    step_efficiency_step_.push_back(current_step_efficiency_);
    generation_control_mode_step_.push_back(current_generation_control_mode_);
    target_elements_step_.push_back(current_target_elements_step_);
    actual_elements_step_.push_back(current_elements);
    target_growth_step_.push_back(current_target_element_growth_);
    actual_growth_step_.push_back(element_growth_factor);
    budget_driver_mode_step_.push_back(current_budget_driver_mode_);
    patch_budget_dispersion_step_.push_back(current_patch_budget_dispersion_);
    patch_h_dispersion_step_.push_back(current_patch_h_dispersion_);
    actual_patch_element_dispersion_step_.push_back(
        current_actual_patch_element_dispersion_);
    curve_balance_dispersion_step_.push_back(current_curve_balance_dispersion_);
    underresolved_transition_fraction_step_.push_back(
        current_underresolved_transition_fraction_real_);
    transition_imbalance_max_step_.push_back(current_transition_imbalance_max_real_);
    patch_consistency_penalty_step_.push_back(current_patch_consistency_penalty_);
    consistency_regularization_active_step_.push_back(
        current_consistency_regularization_active_);
    local_progress_coverage_step_.push_back(current_local_progress_coverage_);
    improved_top_patches_step_.push_back(current_improved_top_patches_);
    top_patch_candidate_count_step_.push_back(current_top_patch_candidate_count_);
    step_purpose_step_.push_back(current_step_purpose_);
    UpdatePatchHistory(geometry, mesh_.get(),
                       last_domain_patch_factors_.empty()
                           ? accepted_candidate_patch_factors
                           : last_domain_patch_factors_,
                       last_submesh_error_, true);
    prev_error = this->error_local_process_;
    has_provisional_progress_state = false;

#if USE_SAVE_ERRO_MESH
    SaveErrorMesh(malha, step_);
#endif  // USE_SAVE_ERRO_MESH

#if USE_MPI
#if USE_PRINT_RESULTS
    cout << "*************** ERRO " << this->step_ << " rank " << RANK_MPI
         << " = " << this->error_local_process_ << endl;
#endif  // #if USE_PRINT_RESULTS
#else
#if USE_PRINT_RESULTS
    cout << "*************** ERRO " << this->step_ << " = "
         << this->error_local_process_ << endl;
    cout << "Resumo step " << this->step_ << ": elementos="
         << current_elements << ", delta_elementos=" << delta_elements
         << ", erro_global=" << this->error_local_process_
         << ", queda_abs=" << error_drop_abs
         << ", queda_pct=" << (improvement * 100.0)
         << ", queda_por_elemento=" << error_drop_per_element
         << ", fator_crescimento_elementos=" << element_growth_factor
         << ", erro_normalizado=" << normalized_error
         << ", quality_min=" << candidate_quality.quality_min
         << ", quality_mean=" << candidate_quality.quality_mean
         << ", poor_ratio=" << candidate_quality.poor_ratio
         << ", acceptance_score=" << acceptance_score
         << ", retry=" << accepted_retry_attempt
         << ", relaxation=" << current_adaptation_relaxation_
         << ", max_delta=" << current_adaptation_max_delta_
         << ", erro_local_max=" << max_sub_error
         << ", nos_gauss=" << current_gauss_nodes_
         << ", nos_mean=" << current_mean_nodes_ << endl;
#endif  // #if USE_PRINT_RESULTS
#endif  // USE_MPI

    timer->InitTimerParallel(0, 0, 11);
    if (WRITE_MESH == std::string("m") || WRITE_MESH == std::string("q") ||
        WRITE_MESH == std::string("h")) {
      SaveMesh(std::move(mesh_), step_);
    }
    WritePatchAdaptationLog(step_, true);
    WriteCurveAdaptationLog(step_, true);
    WriteAcceptanceLog(step_, acceptance_score, candidate_quality, improvement,
                       element_growth_factor, true);
    timer->EndTimerParallel(0, 0, 11);

    if (this->error_local_process_ <= EPSYLON) {
      stop_reason_ = "target_error_reached";
#if USE_PRINT_RESULTS
      cout << "Parando: erro global <= EPSYLON (" << EPSYLON
           << ") no step " << this->step_ << " valor "
           << this->error_local_process_ << endl;
#endif
      break;
    }

    if (ERROR_FLOOR > 0.0 && this->error_local_process_ <= ERROR_FLOOR) {
      stop_reason_ = "error_floor_reached";
#if USE_PRINT_RESULTS
      cout << "Parando: erro global <= ERROR_FLOOR (" << ERROR_FLOOR
           << ") no step " << this->step_ << " valor "
           << this->error_local_process_ << endl;
#endif
      break;
    }

    if (MIN_ERROR_DROP_ABS > 0.0 && error_drop_abs <= MIN_ERROR_DROP_ABS) {
      stop_reason_ = "saturation_reached";
#if USE_PRINT_RESULTS
      cout << "Parando: queda absoluta do erro (" << error_drop_abs
           << ") <= MIN_ERROR_DROP_ABS (" << MIN_ERROR_DROP_ABS
           << ") no step " << this->step_ << endl;
#endif
      break;
    }

    if (MIN_ERROR_DROP_PER_ELEMENT > 0.0 && delta_elements > 0ULL &&
        error_drop_per_element <= MIN_ERROR_DROP_PER_ELEMENT) {
      stop_reason_ = "saturation_reached";
#if USE_PRINT_RESULTS
      cout << "Parando: eficiencia de queda do erro por elemento ("
           << error_drop_per_element << ") <= MIN_ERROR_DROP_PER_ELEMENT ("
           << MIN_ERROR_DROP_PER_ELEMENT << ") no step " << this->step_ << endl;
#endif
      break;
    }

    if (stagnant >= PATIENCE) {
      stop_reason_ = "saturation_reached";
#if USE_PRINT_RESULTS
      cout << "Parando: melhoria relativa < " << MIN_IMPROVEMENT
           << " por " << PATIENCE << " iterações consecutivas no step "
           << this->step_ << " (última melhoria " << improvement << ")"
           << endl;
#endif
      break;
    }

    // Early-stop só para cenários curtos (max_steps baixo). Para requests que querem
    // malhas adicionais (passos 4 e 5), não interromper quando max_steps > 3.
    if (current_policy_.stable_mode && current_policy_.max_steps <= 3 &&
        this->step_ == 1 && geometry != nullptr &&
        geometry->GetNumberPatches() <= 4U && !error_step_.empty() &&
        error_step_.back() < 0.12) {
      stop_reason_ = "stable_few_patch_low_error";
#if USE_PRINT_RESULTS
      cout << "Parando: malha com poucos patches e erro global ja baixo; "
           << "evitando passo extra cuja remalha tende a aumentar o erro."
           << endl;
#endif
      break;
    }
  }

#if USE_MPI
  timer->EndTimerParallel(RANK_MPI, 0, 10);  // Full
  timer->PrintTime(RANK_MPI);
#endif  // USE_MPI

  // Escreve o(s) arquivo(s) com suas respectivas malhas em cada step_
#if USE_MPI
  for (unsigned int i = 0; i < error_step_.size(); ++i) {
    cout << "Erro do processo " << RANK_MPI << " no step_ " << i << " = "
         << error_step_[i] << endl;
  }
#else

  timer->EndTimerParallel(0, 0, 10);  // Full
  timer->PrintTime();

  for (unsigned int i = 0; i < error_step_.size(); ++i) {
    cout << "Erro no step_ " << i << " = " << error_step_[i]
         << ", elementos = " << elements_step_[i]
         << ", nos_gauss = " << gauss_nodes_step_[i]
         << ", nos_mean = " << mean_nodes_step_[i]
         << ", quality_min = " << quality_min_step_[i]
         << ", quality_mean = " << quality_mean_step_[i]
         << ", poor_ratio = " << poor_ratio_step_[i]
         << ", acceptance_score = " << acceptance_score_step_[i]
         << ", retry = " << retry_attempt_step_[i]
         << ", relaxation = " << applied_relaxation_step_[i]
         << ", max_delta = " << applied_max_delta_step_[i]
         << ", queda_abs = " << error_drop_abs_step_[i]
         << ", queda_pct = " << error_drop_pct_step_[i]
         << ", queda_por_elemento = " << error_drop_per_element_step_[i]
         << ", fator_crescimento_elementos = " << element_growth_factor_step_[i]
         << ", erro_normalizado = " << error_normalized_step_[i]
         << endl;
  }

  cout << "Motivo da parada: " << stop_reason_ << endl;
  WriteCompatibilitySummary(geometry);
  WriteRuntimeSummary(timer);

  // Após terminar, libere os pontos de discretização das curvas.
  // Usa set para não deletar duas vezes pontos compartilhados entre curvas.
  std::unordered_set<PointAdaptive*> unique_points;
  for (unsigned int i = 0; i < geometry->GetNumberCurves(); ++i) {
    auto* curve = geometry->GetCurve(i);
    for (auto* p : curve->GetPoints()) {
      unique_points.insert(p);
    }
    curve->GetPoints().clear();
  }
  for (auto* p : unique_points) {
    delete p;
  }
#endif  // USE_MPI
}

void GeneratorAdaptive::DisableSharedCurveSynchronization(Geometry* geometry) {
  if (geometry == nullptr) {
    return;
  }

  for (unsigned int curve_index = 0; curve_index < geometry->GetNumberCurves();
       ++curve_index) {
    CurveAdaptive* curve = geometry->GetCurve(curve_index);
    if (curve == nullptr || curve->GetNumBerPatches() <= 1U) {
      continue;
    }

    std::vector<Patch*> incident_patches;
    incident_patches.reserve(curve->GetNumBerPatches());
    for (unsigned int patch_index = 0; patch_index < curve->GetNumBerPatches();
         ++patch_index) {
      Patch* patch = curve->GetPatch(patch_index);
      if (patch != nullptr) {
        incident_patches.push_back(patch);
      }
    }
    if (incident_patches.size() <= 1U) {
      continue;
    }

    curve->ClearPatches();
    curve->InsertPatch(incident_patches.front());

    for (size_t patch_pos = 0; patch_pos < incident_patches.size(); ++patch_pos) {
      PatchCoons* patch = dynamic_cast<PatchCoons*>(incident_patches[patch_pos]);
      if (patch == nullptr) {
        continue;
      }
      CurveAdaptive* replacement = curve;
      if (patch_pos > 0U) {
        replacement = CloneCurveForPatch(curve, patch);
        geometry->InsertCurve(replacement);
      }

      for (unsigned int edge_index = 0; edge_index < patch->GetNumBerCurves();
           ++edge_index) {
        if (patch->GetCurve(edge_index) == curve) {
          patch->ReplaceCurve(edge_index, replacement);
        }
      }
    }
  }
}

void GeneratorAdaptive::AdaptCurve(Geometry* geometry) {
  // Use std::vector to avoid non-standard VLA.
  std::vector<std::list<PointAdaptive*>> new_points(geometry->GetNumberCurves());
  std::map<PointAdaptive*, PointAdaptive*> map_points;
  std::unordered_set<PointAdaptive*> old_points;
  const std::vector<double> patch_factors = ComputePatchAdaptationFactors(geometry);
  current_curve_adaptation_.clear();
  current_curve_target_points_.assign(geometry->GetNumberCurves(), 0.0);
  current_curve_adaptation_.assign(geometry->GetNumberCurves(),
                                   CurveAdaptationSnapshot{});

  auto clamp_curve_target = [](int value) {
    value = std::max(value, std::max(2, CURVE_POINT_MIN));
    if (CURVE_POINT_MAX > 0) {
      value = std::min(value, CURVE_POINT_MAX);
    }
    return value;
  };

  auto rebuild_curve_with_target =
      [this](CurveAdaptive* curve, std::list<PointAdaptive*>& points, int target_count) {
        CurveAdaptiveParametric* param_curve =
            static_cast<CurveAdaptiveParametric*>(curve);
        std::vector<PointAdaptive*> point_vec(points.begin(), points.end());
        std::vector<double> param_vec(param_curve->parameters_.begin(),
                                      param_curve->parameters_.end());
        if (point_vec.size() < 2 || param_vec.size() != point_vec.size()) {
          return;
        }
        const int capped_target = std::max(2, target_count);
        if (capped_target == static_cast<int>(point_vec.size())) {
          return;
        }
        if (capped_target > static_cast<int>(point_vec.size())) {
          std::list<PointAdaptive*> rebuilt_points;
          std::list<double> rebuilt_parameters;
          std::unordered_set<PointAdaptive*> keep_existing;
          for (int idx = 0; idx < capped_target; ++idx) {
            const double alpha =
                (capped_target <= 1)
                    ? 0.0
                    : static_cast<double>(idx) /
                          static_cast<double>(capped_target - 1);
            const double t =
                (idx == 0) ? 0.0 : ((idx == capped_target - 1) ? 1.0 : alpha);
            if (idx == 0) {
              rebuilt_points.push_back(point_vec.front());
              keep_existing.insert(point_vec.front());
            } else if (idx == capped_target - 1) {
              rebuilt_points.push_back(point_vec.back());
              keep_existing.insert(point_vec.back());
            } else {
              const PointAdaptive point = param_curve->FindPointByParameter(t);
              unsigned long new_id = 0UL;
              if (!this->id_managers_.empty() && this->id_managers_[0]) {
                new_id = CheckedLegacyId(this->id_managers_[0]->next());
              }
              if (new_id > 0UL) {
                rebuilt_points.push_back(new NodeAdaptive(point.GetX(), point.GetY(),
                                                          point.GetZ(), new_id));
              } else {
                rebuilt_points.push_back(new NodeAdaptive(point.GetX(), point.GetY(),
                                                          point.GetZ()));
              }
            }
            rebuilt_parameters.push_back(t);
          }
          for (PointAdaptive* existing_point : point_vec) {
            if (keep_existing.find(existing_point) == keep_existing.end()) {
              delete existing_point;
            }
          }
          param_curve->UpdateParameters(rebuilt_parameters);
          points = std::move(rebuilt_points);
          return;
        }
        std::vector<int> selected_indices;
        selected_indices.reserve(static_cast<size_t>(capped_target));
        selected_indices.push_back(0);
        for (int idx = 1; idx < capped_target - 1; ++idx) {
          const double alpha =
              static_cast<double>(idx) / static_cast<double>(capped_target - 1);
          const int selected = static_cast<int>(std::round(
              alpha * static_cast<double>(point_vec.size() - 1)));
          if (selected > selected_indices.back() &&
              selected < static_cast<int>(point_vec.size() - 1)) {
            selected_indices.push_back(selected);
          }
        }
        selected_indices.push_back(static_cast<int>(point_vec.size() - 1));
        std::sort(selected_indices.begin(), selected_indices.end());
        selected_indices.erase(std::unique(selected_indices.begin(), selected_indices.end()),
                               selected_indices.end());
        std::unordered_set<int> keep_indices(selected_indices.begin(),
                                             selected_indices.end());
        std::list<PointAdaptive*> trimmed_points;
        std::list<double> trimmed_parameters;
        for (int idx = 0; idx < static_cast<int>(point_vec.size()); ++idx) {
          if (keep_indices.find(idx) != keep_indices.end()) {
            trimmed_points.push_back(point_vec[static_cast<size_t>(idx)]);
            trimmed_parameters.push_back(param_vec[static_cast<size_t>(idx)]);
          } else {
            delete point_vec[static_cast<size_t>(idx)];
          }
        }
        param_curve->UpdateParameters(trimmed_parameters);
        points = std::move(trimmed_points);
      };

  // Capture all current curve points to delete after we swap to new ones.
  for (unsigned int i = 0; i < geometry->GetNumberCurves(); ++i) {
    for (auto* p : geometry->GetCurve(i)->GetPoints()) {
      old_points.insert(p);
    }
  }

  for (unsigned int i = 0; i < geometry->GetNumberCurves(); ++i) {
    CurveAdaptive* curve = geometry->GetCurve(i);
    const int previous_points = std::max(2, static_cast<int>(curve->GetNumBerPoints()));
    int target_curve_points = previous_points;
    std::vector<unsigned int> source_patch_ids;
    double min_adjacent_target_h = std::numeric_limits<double>::max();
    double max_adjacent_target_elements = 0.0;
    bool sensitive_curve = false;
    if (BUDGET_DRIVEN_ADAPTATION == 1 && CURVE_POINT_BUDGET_MODE == 1) {
      double max_patch_growth = 1.0;
      double max_patch_budget = 0.0;
      bool touches_priority_patch = false;
      for (unsigned int patch_index = 0; patch_index < curve->GetNumBerPatches();
           ++patch_index) {
        Patch* patch = curve->GetPatch(patch_index);
        if (!patch) {
          continue;
        }
        const unsigned int patch_id = ResolvePatchIndex(geometry, patch);
        if (patch_id == std::numeric_limits<unsigned int>::max()) {
          continue;
        }
        source_patch_ids.push_back(patch_id);
        if (patch_id < current_patch_target_elements_.size() &&
            patch_id < patch_history_.size()) {
          max_patch_budget =
              std::max(max_patch_budget, current_patch_target_elements_[patch_id]);
          max_adjacent_target_elements =
              std::max(max_adjacent_target_elements,
                       current_patch_target_elements_[patch_id]);
          if (patch_id < current_patch_target_h_.size() &&
              current_patch_target_h_[patch_id] > 0.0) {
            min_adjacent_target_h =
                std::min(min_adjacent_target_h, current_patch_target_h_[patch_id]);
            if (step_ >= 2 && current_patch_target_h_[patch_id] <= 0.14) {
              sensitive_curve = true;
            }
          }
          const double prev_patch_elements = std::max(
              1.0, static_cast<double>(patch_history_[patch_id].prev_patch_elements));
          max_patch_growth =
              std::max(max_patch_growth, current_patch_target_elements_[patch_id] /
                                             prev_patch_elements);
          if (patch_id < current_patch_adaptation_.size() &&
              current_patch_adaptation_[patch_id].refinement_rank > 0 &&
              current_patch_adaptation_[patch_id].refinement_rank <=
                  static_cast<int>(std::max(1U, current_top_patch_count_))) {
            touches_priority_patch = true;
          }
        }
      }
      const double curve_growth_cap =
          (step_ <= 1) ? CURVE_POINT_GROWTH_STEP1 : CURVE_POINT_GROWTH_STEPN;
      const double blended_growth =
          CURVE_POINT_BUDGET_BLEND * max_patch_growth +
          (1.0 - CURVE_POINT_BUDGET_BLEND) * 1.0;
      target_curve_points = clamp_curve_target(static_cast<int>(std::round(
          static_cast<double>(previous_points) *
          std::min(curve_growth_cap, std::max(1.0, blended_growth)))));
      if (step_ >= 2 && touches_priority_patch) {
        target_curve_points =
            std::max(target_curve_points, clamp_curve_target(previous_points + 4));
      }
      if (step_ >= 3 && touches_priority_patch) {
        target_curve_points =
            std::max(target_curve_points, clamp_curve_target(previous_points + 6));
      }
      if (sensitive_curve) {
        target_curve_points =
            std::max(target_curve_points,
                     clamp_curve_target(previous_points + ((step_ <= 1) ? 5 : 7)));
      }
      if (current_policy_.stable_mode && step_ >= 2 &&
          min_adjacent_target_h < std::numeric_limits<double>::max()) {
        const double curve_length = std::max(1.0e-9, curve->GetLength());
        double spacing_scale = 1.35;
        if (sensitive_curve) {
          spacing_scale = 0.82;
        }
        if (current_step_purpose_ == "curve_catch_up") {
          spacing_scale = std::min(spacing_scale, 0.62);
        } else if (current_step_purpose_ == "transition_catch_up") {
          spacing_scale = std::min(spacing_scale, 0.72);
        }
        const double desired_spacing =
            std::max(1.0e-6, min_adjacent_target_h * spacing_scale);
        int geometric_target =
            static_cast<int>(std::ceil(curve_length / desired_spacing)) + 1;
        if (max_adjacent_target_elements > 250.0) geometric_target += 1;
        if (max_adjacent_target_elements > 450.0) geometric_target += 1;
        if (sensitive_curve) geometric_target += 4;
        if (touches_priority_patch) geometric_target += 2;
        target_curve_points =
            std::max(target_curve_points, clamp_curve_target(geometric_target));
      }
    }
    CurveAdaptationSnapshot snapshot;
    snapshot.curve_id = curve->GetId() != 0UL ? curve->GetId()
                                              : static_cast<unsigned long>(i);
    snapshot.factor = ComputeCurveAdaptationFactor(curve, patch_factors);
    snapshot.num_points = curve->GetNumBerPoints();
    snapshot.policy = CURVE_ADAPTATION_POLICY;
    snapshot.target_points = target_curve_points;
    snapshot.actual_points = previous_points;
    snapshot.geometric_target_points = target_curve_points;
    snapshot.min_adjacent_target_h =
        (min_adjacent_target_h < std::numeric_limits<double>::max())
            ? min_adjacent_target_h
            : 0.0;
    snapshot.max_adjacent_target_elements = max_adjacent_target_elements;
    snapshot.sensitive_curve = sensitive_curve ? 1 : 0;
    snapshot.target_points_before_consistency = target_curve_points;
    snapshot.target_points_after_consistency = target_curve_points;
    snapshot.point_growth_factor = 1.0;
    std::ostringstream adjacent_ids;
    std::ostringstream adjacent_weights;
    bool first_adjacent = true;
    for (unsigned int patch_index = 0;
         patch_index < curve->GetNumBerPatches(); ++patch_index) {
      Patch* patch = curve->GetPatch(patch_index);
      if (!patch) {
        continue;
      }
      const unsigned int patch_id = ResolvePatchIndex(geometry, patch);
      if (patch_id == std::numeric_limits<unsigned int>::max()) {
        continue;
      }
      if (!first_adjacent) {
        adjacent_ids << "|";
        adjacent_weights << "|";
      }
      adjacent_ids << patch_id;
      double weight = 1.0;
      if (CURVE_ADAPTATION_POLICY == 3 && patch_id < patch_history_.size()) {
        weight = std::max(1.0e-9, patch_history_[patch_id].prev_patch_efficiency);
      } else if (patch_id < last_submesh_error_.size()) {
        weight = std::max(1.0e-9, last_submesh_error_[patch_id]);
      }
      adjacent_weights << weight;
      first_adjacent = false;
    }
    snapshot.adjacent_patch_ids = adjacent_ids.str();
    snapshot.adjacent_patch_weights = adjacent_weights.str();
    {
      std::ostringstream source_patches;
      for (size_t idx = 0; idx < source_patch_ids.size(); ++idx) {
        if (idx > 0) source_patches << "|";
        source_patches << source_patch_ids[idx];
      }
      snapshot.curve_budget_source_patches = source_patches.str();
    }
    current_curve_target_points_[i] = static_cast<double>(target_curve_points);
    current_curve_adaptation_[i] = snapshot;
  }

  if (PATCH_CONSISTENCY_MODE == 1 && CURVE_CONSISTENCY_MODE == 1 &&
      BUDGET_DRIVEN_ADAPTATION == 1) {
    RegularizeCurveTargetsFromPatchConsistency(geometry);
  }

  if (current_policy_.stable_mode && BUDGET_DRIVEN_ADAPTATION == 1 && step_ >= 2) {
    for (unsigned int i = 0; i < geometry->GetNumberCurves(); ++i) {
      CurveAdaptive* curve = geometry->GetCurve(i);
      CurveAdaptationSnapshot& snapshot = current_curve_adaptation_[i];
      const int previous_points =
          std::max(2, static_cast<int>(curve->GetNumBerPoints()));
      const double growth_cap_factor = (step_ == 2) ? 1.55 : 1.75;
      const int growth_cap_points = clamp_curve_target(static_cast<int>(std::ceil(
          static_cast<double>(previous_points) * growth_cap_factor)));
      int elements_cap_points = growth_cap_points;
      if (snapshot.max_adjacent_target_elements > 0.0) {
        elements_cap_points = clamp_curve_target(
            static_cast<int>(std::ceil(
                1.8 * std::sqrt(snapshot.max_adjacent_target_elements))) +
            6);
      }
      const int structural_cap =
          std::max(previous_points + 1,
                   std::min(growth_cap_points, elements_cap_points));
      const int before_cap = clamp_curve_target(
          static_cast<int>(std::round(current_curve_target_points_[i])));
      if (before_cap > structural_cap) {
        current_curve_target_points_[i] = static_cast<double>(structural_cap);
        snapshot.curve_consistency_clamped = 1;
      }
      snapshot.target_points_after_consistency = clamp_curve_target(
          static_cast<int>(std::round(current_curve_target_points_[i])));
      snapshot.target_points = snapshot.target_points_after_consistency;
      snapshot.geometric_target_points = snapshot.target_points_after_consistency;
    }
  }

  // In sequential, ensure id_managers_[0] exists.
  for (unsigned int i = 0; i < geometry->GetNumberCurves(); ++i) {
    CurveAdaptive* curve = geometry->GetCurve(i);
    const double factor_disc_global =
        ComputeCurveAdaptationFactor(curve, patch_factors);
    const int previous_points = std::max(2, static_cast<int>(curve->GetNumBerPoints()));
    const int target_curve_points = clamp_curve_target(
        static_cast<int>(std::round(current_curve_target_points_[i])));

    auto curve_refined = adapter_.AdaptCurveByCurve(
        curve, map_points, this->id_managers_[0].get(), factor_disc_global);
    curve->SetPoints(curve_refined);

    auto surface_refined = adapter_.AdaptCurveBySurface(
        curve, map_points, this->id_managers_[0].get(), factor_disc_global);
    curve->SetPoints(surface_refined);
    if (BUDGET_DRIVEN_ADAPTATION == 1 && CURVE_POINT_BUDGET_MODE == 1) {
      rebuild_curve_with_target(curve, surface_refined, target_curve_points);
      curve->SetPoints(surface_refined);
    }

    std::unordered_set<PointAdaptive*> keep(surface_refined.begin(),
                                            surface_refined.end());
    for (auto* p : curve_refined) {
      if (keep.find(p) == keep.end()) {
        delete p;
      }
    }

    new_points[i] = std::move(surface_refined);
    CurveAdaptationSnapshot& snapshot = current_curve_adaptation_[i];
    snapshot.factor = factor_disc_global;
    snapshot.num_points = curve->GetNumBerPoints();
    snapshot.target_points = target_curve_points;
    snapshot.actual_points = static_cast<int>(curve->GetNumBerPoints());
    snapshot.point_growth_factor =
        previous_points > 0
            ? static_cast<double>(snapshot.actual_points) /
                  static_cast<double>(previous_points)
            : 1.0;
  }

  // Delete old discretization points (shared endpoints handled by set).
  for (auto* p : old_points) {
    delete p;
  }
}

void GeneratorAdaptive::AdaptDomain(Geometry* geometry, MeshAdaptive* mesh) {
  const std::vector<double> patch_factors = ComputePatchAdaptationFactors(geometry);
  const bool few_patch_low_error =
      current_policy_.stable_mode && geometry != nullptr &&
      geometry->GetNumberPatches() <= 4U && !error_step_.empty() &&
      error_step_.back() < 0.12 && step_ <= 3;
  last_domain_patch_factors_.assign(patch_factors.begin(), patch_factors.end());
  current_underresolved_transition_patch_count_real_ = 0U;
  current_underresolved_transition_fraction_real_ = 0.0;
  current_transition_imbalance_mean_real_ = 0.0;
  current_transition_imbalance_max_real_ = 0.0;
  double structural_imbalance_sum = 0.0;
  unsigned int structural_count = 0U;

  for (unsigned int i = 0; i < geometry->GetNumberPatches(); ++i) {
    PatchCoons* p = static_cast<PatchCoons*>(geometry->GetPatch(i));
    double domain_factor = patch_factors[i];
    std::string patch_generation_mode = "normal";
    bool soft_saturated = false;
    bool hard_saturated = false;
    if (QUADTREE_SATURATION_MODE == 1 && step_ > 1 && i < patch_history_.size()) {
      const PatchHistoryState& history = patch_history_[i];
      if (history.prev_patch_generated_faces >
              static_cast<unsigned long long>(
                  std::max(1.0, QUADTREE_SATURATION_ELEMENTS_THRESHOLD)) &&
          history.prev_patch_efficiency <=
              QUADTREE_SATURATION_EFFICIENCY_THRESHOLD) {
        soft_saturated = true;
        domain_factor = ClampValue(domain_factor * QUADTREE_SATURATION_H_SCALE,
                                   PATCH_FACTOR_MIN, PATCH_FACTOR_MAX);
      }
      if (QUADTREE_HARD_SATURATION_MODE == 1 &&
          history.consecutive_inefficient_steps >=
              static_cast<unsigned int>(QUADTREE_HARD_SATURATION_STEPS)) {
        hard_saturated = true;
        domain_factor =
            std::max(domain_factor, QUADTREE_HARD_SATURATION_FACTOR_MIN);
      }
    }
    if (STEP_ELEMENT_BUDGET_MODE == 1 && step_ > 1 &&
        current_budget_retry_count_ > 0U) {
      domain_factor = ClampValue(
          domain_factor *
              std::pow(STEP_ELEMENT_BUDGET_RETRY_H_SCALE,
                       static_cast<int>(current_budget_retry_count_)),
          PATCH_FACTOR_MIN, PATCH_FACTOR_MAX);
    }
    const int refinement_rank =
        (i < current_patch_adaptation_.size())
            ? current_patch_adaptation_[i].refinement_rank
            : -1;
    const bool step2_quality_critical =
        (step_ == 2 && i < current_patch_adaptation_.size() &&
         (current_patch_adaptation_[i].quality_min < 0.25 ||
          current_patch_adaptation_[i].poor_ratio > 0.18));
    if (step_ > 1 && PATCH_GENERATION_CONTROL_MODE == 1) {
      patch_generation_mode =
          (STEP2_MICRO_REFINEMENT_MODE == 1) ? "micro_refinement" : "normal";
      if (PATCH_GENERATION_TOP_RANK_ONLY_MODE == 1 &&
          (i >= current_patch_eligible_mask_.size() ||
           current_patch_eligible_mask_[i] == 0)) {
        patch_generation_mode = "budget_constrained";
        domain_factor = std::max(domain_factor, STEP2_NONELIGIBLE_FACTOR_MIN);
      }
    }
    if (step_ > 1 && !few_patch_low_error &&
        static_cast<size_t>(i) < current_patch_eligible_mask_.size() &&
        current_patch_eligible_mask_[i] == 1) {
      double eligible_h_scale = 1.0;
      if (STEP2_MICRO_REFINEMENT_MODE == 1) {
        eligible_h_scale *= STEP2_ELIGIBLE_H_SCALE;
      }
      if (static_cast<size_t>(i) < patch_history_.size() &&
          patch_history_[i].prev_patch_elements >=
              static_cast<unsigned long long>(
                  std::max(1.0, STEP2_ELIGIBLE_ELEMENTS_THRESHOLD))) {
        eligible_h_scale *= 1.05;
      }
      if (STEP_ELEMENT_BUDGET_PRESSURE_MODE == 1 &&
          current_budget_pressure_ > STEP_ELEMENT_BUDGET_PRESSURE_START) {
        eligible_h_scale *= STEP_ELEMENT_BUDGET_PRESSURE_H_SCALE;
      }
      if (PATCH_GENERATION_CONTROL_MODE == 1) {
        eligible_h_scale *= PATCH_GENERATION_MICRO_H_SCALE;
      }
      domain_factor = ClampValue(domain_factor * eligible_h_scale, PATCH_FACTOR_MIN,
                                 PATCH_FACTOR_MAX);
      if (i < current_patch_adaptation_.size()) {
        current_patch_adaptation_[i].eligible_h_scale_applied = eligible_h_scale;
      }
      current_eligible_patch_count_ += 1U;
      current_eligible_patch_mean_factor_ += domain_factor;
      current_eligible_patch_mean_h_scale_ += eligible_h_scale;
    }
    if (step_ == 2 && refinement_rank > 0 &&
        refinement_rank <= static_cast<int>(current_top_patch_count_ + 1U)) {
      if (!few_patch_low_error) {
        domain_factor =
            std::min(domain_factor, step2_quality_critical ? 0.96 : 1.0);
        if (step2_quality_critical) {
          patch_generation_mode = "quality_stabilized";
        }
      }
    }
    if (i < current_patch_saturated_soft_mask_.size()) {
      current_patch_saturated_soft_mask_[i] = soft_saturated ? 1 : 0;
    }
    if (i < current_patch_saturated_hard_mask_.size()) {
      current_patch_saturated_hard_mask_[i] = hard_saturated ? 1 : 0;
    }
    if (step_ > 1 && i < patch_history_.size() && i < current_patch_adaptation_.size()) {
      const PatchHistoryState& history = patch_history_[i];
      const double efficiency_norm =
          Clamp01(history.prev_patch_efficiency /
                  std::max(PATCH_EFFICIENCY_TARGET, 1.0e-9));
      const double rank_weight =
          (refinement_rank > 0)
              ? std::max(0.35,
                         1.0 + PATCH_LOCAL_ELEMENT_BUDGET_SCALE_BY_RANK *
                                   (1.0 - (static_cast<double>(refinement_rank - 1) /
                                           std::max(1.0, static_cast<double>(
                                                             current_top_patch_count_ > 1
                                                                 ? current_top_patch_count_ - 1
                                                                 : 1U)))))
              : 0.5;
      const double efficiency_weight =
          1.0 + PATCH_LOCAL_ELEMENT_BUDGET_SCALE_BY_EFFICIENCY * efficiency_norm;
      const double local_patch_budget =
          std::min(PATCH_LOCAL_BUDGET_HARD_CAP,
                   PATCH_LOCAL_ELEMENT_BUDGET_BASE * rank_weight *
                       efficiency_weight);
      current_patch_adaptation_[i].local_patch_budget = local_patch_budget;
      if (!few_patch_low_error) {
        current_patch_adaptation_[i].target_patch_elements = local_patch_budget;
      } else if (i < current_patch_target_elements_.size()) {
        current_patch_adaptation_[i].target_patch_elements =
            current_patch_target_elements_[i];
      }
      current_patch_adaptation_[i].actual_patch_elements =
          static_cast<double>(history.prev_patch_generated_faces);
      const bool patch_budget_exceeded =
          PATCH_LOCAL_ELEMENT_BUDGET_MODE == 1 &&
          history.prev_patch_generated_faces >
              static_cast<unsigned long long>(std::max(1.0, local_patch_budget));
      current_patch_adaptation_[i].patch_budget_exceeded =
          patch_budget_exceeded ? 1 : 0;
      if (patch_budget_exceeded) {
        patch_generation_mode = "budget_constrained";
        domain_factor = ClampValue(domain_factor * PATCH_LOCAL_BUDGET_H_SCALE,
                                   PATCH_FACTOR_MIN, PATCH_FACTOR_MAX);
      }
      if (!few_patch_low_error && PATCH_GENERATION_CONTROL_MODE == 1 &&
          current_target_element_growth_ > 0.0 &&
          history.prev_patch_elements > 0ULL &&
          current_target_element_growth_ <= 1.0) {
        domain_factor =
            ClampValue(domain_factor * STEP_TARGET_GROWTH_H_SCALE,
                       PATCH_FACTOR_MIN, PATCH_FACTOR_MAX);
        // Em adaptive_stable, evitar mudar para 'budget_constrained' após
        // rejeições consecutivas (isso costuma gerar "complementos" errados
        // em patches curvos). Mantemos o clamp no fator, mas não mudamos o
        // modo de geração.
        if (!(current_policy_.stable_mode && stable_consecutive_reject_count_ > 0U)) {
          patch_generation_mode = "budget_constrained";
        }
      }
      if (!few_patch_low_error && STEP_ELEMENT_BUDGET_PRESSURE_MODE == 1 &&
          current_budget_pressure_ >= PATCH_GENERATION_BUDGET_PRESSURE_START) {
        domain_factor =
            ClampValue(domain_factor * PATCH_GENERATION_CONSTRAINED_H_SCALE,
                       PATCH_FACTOR_MIN, PATCH_FACTOR_MAX);
        patch_generation_mode = "budget_constrained";
      }
        current_patch_adaptation_[i].patch_generation_mode = patch_generation_mode;
      }
    last_domain_patch_factors_[i] = domain_factor;
    double target_patch_h =
        (i < current_patch_target_h_.size()) ? current_patch_target_h_[i] : -1.0;
    const int quadtree_depth_cap =
        (i < current_patch_quadtree_depth_cap_.size())
            ? current_patch_quadtree_depth_cap_[i]
            : -1;
    // adaptive_stable guard: if we are generating a new (rejected) step after
    // having rejected the previous one, avoid increasing target_patch_h
    // (which would coarsen / "desrefinar" the domain compared to the last
    // rejected attempt).
    if (current_policy_.stable_mode && stable_consecutive_reject_count_ > 0U &&
        this->step_ >= 4 && i < last_candidate_patch_h_.size()) {
      const double prev_h = last_candidate_patch_h_[i];
      const bool top_priority_curve_patch =
          (i < current_patch_adaptation_.size() &&
           current_patch_adaptation_[i].refinement_rank > 0 &&
           current_patch_adaptation_[i].refinement_rank <= 2);
      if (prev_h > 0.0 && top_priority_curve_patch) {
        // Em steps tardios após rejeição, forçamos discretização progressiva
        // (h menor) nos patches curvos prioritários.
        target_patch_h = std::min(target_patch_h, prev_h * 0.96);
      } else if (prev_h > 0.0 && target_patch_h > prev_h) {
        target_patch_h = prev_h;
      }
    }
    // Ratio atual vs. alvo geométrico nas curvas do patch; 1.0 = sem déficit conhecido.
    double patch_curve_support_ratio = 1.0;
    int patch_curve_density_points = 0;
    if (current_policy_.stable_mode && step_ >= 2 && target_patch_h <= 0.0) {
      SubMesh* previous_sub_mesh = p->GetSubMesh();
      if (previous_sub_mesh != nullptr && previous_sub_mesh->GetNumberElements() > 0U) {
        double patch_area_est = 0.0;
        for (unsigned int e = 0; e < previous_sub_mesh->GetNumberElements(); ++e) {
          ElementAdaptive* element = previous_sub_mesh->GetElement(e);
          if (element != nullptr) {
            patch_area_est += std::fabs(element->GetArea());
          }
        }
        if (patch_area_est > 0.0) {
          const double h_ref =
              std::sqrt(patch_area_est /
                        static_cast<double>(previous_sub_mesh->GetNumberElements()));
          const double stable_scale = 0.78;
          target_patch_h = std::max(1.0e-4, h_ref * stable_scale);
        }
      }
    }

    if (current_policy_.stable_mode && step_ == 2 && !few_patch_low_error) {
      domain_factor = ClampValue(domain_factor * 1.10, PATCH_FACTOR_MIN, PATCH_FACTOR_MAX);
      if (target_patch_h > 0.0) {
        target_patch_h *= 1.18;
      }
      patch_generation_mode = "step2_global_stabilization";
    }
    if (current_policy_.stable_mode && step_ >= 2 && target_patch_h > 0.0 &&
        !few_patch_low_error) {
      patch_curve_support_ratio = 1.0;
      double patch_curve_spacing_min = std::numeric_limits<double>::max();
      for (unsigned int c = 0; c < p->GetNumBerCurves(); ++c) {
        CurveAdaptive* curve = p->GetCurve(c);
        if (curve == nullptr) {
          continue;
        }
        unsigned int curve_idx = std::numeric_limits<unsigned int>::max();
        for (unsigned int g = 0; g < geometry->GetNumberCurves(); ++g) {
          if (geometry->GetCurve(g) == curve) {
            curve_idx = g;
            break;
          }
        }
        if (curve_idx >= current_curve_adaptation_.size()) {
          continue;
        }
        const CurveAdaptationSnapshot& snapshot = current_curve_adaptation_[curve_idx];
        if (snapshot.geometric_target_points <= 0) {
          continue;
        }
        patch_curve_density_points =
            std::max(patch_curve_density_points, snapshot.actual_points);
        if (snapshot.actual_points >= 2) {
          const double curve_spacing =
              std::max(1.0e-6, curve->GetLength()) /
              static_cast<double>(snapshot.actual_points - 1);
          patch_curve_spacing_min =
              std::min(patch_curve_spacing_min, curve_spacing);
        }
        const double ratio =
            static_cast<double>(snapshot.actual_points) /
            static_cast<double>(std::max(1, snapshot.geometric_target_points));
        patch_curve_support_ratio =
            std::min(patch_curve_support_ratio, ratio);
      }
      if (patch_curve_support_ratio < 0.85) {
        const double h_boost =
            ClampValue(1.0 + (0.85 - patch_curve_support_ratio) * 0.80, 1.0,
                       1.35);
        domain_factor = ClampValue(domain_factor * h_boost, PATCH_FACTOR_MIN,
                                   PATCH_FACTOR_MAX);
        target_patch_h *= h_boost;
        patch_generation_mode = "curve_transition_balanced";
      }
      if (patch_curve_density_points >= 5) {
        const double density_norm = Clamp01(
            static_cast<double>(patch_curve_density_points - 4) / 12.0);
        const double h_follow =
            ClampValue(1.0 - 0.18 * density_norm, 0.82, 1.0);
        domain_factor = ClampValue(domain_factor * h_follow, PATCH_FACTOR_MIN,
                                   PATCH_FACTOR_MAX);
        target_patch_h *= h_follow;
        patch_generation_mode = "curve_density_follow";
      }
      if (patch_curve_spacing_min < std::numeric_limits<double>::max()) {
        const double spacing_limited_h = std::max(1.0e-4, patch_curve_spacing_min * 0.95);
        if (spacing_limited_h < target_patch_h) {
          target_patch_h = spacing_limited_h;
          patch_generation_mode = "curve_spacing_follow";
        }
      }
      if (current_step_purpose_ == "transition_catch_up") {
        target_patch_h *= 0.82;
        patch_generation_mode = "transition_band_follow";
      } else if (current_step_purpose_ == "interior_quality_recovery" &&
                 patch_curve_density_points >= 5) {
        target_patch_h *= 0.88;
        patch_generation_mode = "interior_quality_recovery";
      }
    }
    int gen_depth_cap = quadtree_depth_cap;
    // Com curvas ainda longe do alvo geométrico, não baixar profundidade nem
    // amortecer geração nos passos tardios: isso encolhia a malha (ex. passos 4–5).
    constexpr double kStableRejectCoarsenCurveRatioFloor = 0.90;
    // Além dos pontos da curva, também respeitamos o "estado no domínio"
    // (curve_domain_balance/transition_underresolved). Isto evita que um patch
    // ainda sub-amostrado no domínio acabe por coarsen/reduzir elementos em passos
    // tardios apenas por causa do comportamento das curvas adjacentes.
    double curve_domain_balance = 1.0;
    int transition_underresolved = 0;
    if (i < current_patch_adaptation_.size()) {
      curve_domain_balance = current_patch_adaptation_[i].curve_domain_balance;
      transition_underresolved = current_patch_adaptation_[i].transition_underresolved;
    }
    const bool stable_curve_allows_reject_coarsen =
        patch_curve_support_ratio >= kStableRejectCoarsenCurveRatioFloor &&
        curve_domain_balance >= 0.90 && transition_underresolved == 0;
    if (current_policy_.stable_mode && stable_consecutive_reject_count_ > 0U &&
        this->step_ >= 4 && quadtree_depth_cap >= 0 &&
        stable_curve_allows_reject_coarsen) {
      const int reduce =
          std::min(quadtree_depth_cap, static_cast<int>(this->step_ - 3));
      gen_depth_cap = std::max(0, quadtree_depth_cap - reduce);
    }
    if (current_policy_.stable_mode && stable_consecutive_reject_count_ > 0U &&
        this->step_ >= 4 && i < current_patch_adaptation_.size() &&
        current_patch_adaptation_[i].refinement_rank > 0 &&
        current_patch_adaptation_[i].refinement_rank <= 2) {
      gen_depth_cap = (gen_depth_cap >= 0) ? std::max(gen_depth_cap, 4) : 4;
    }
    if (current_policy_.stable_mode && patch_curve_density_points >= 8) {
      const int density_depth_cap = (patch_curve_density_points >= 12) ? 6 : 5;
      gen_depth_cap =
          (gen_depth_cap >= 0) ? std::max(gen_depth_cap, density_depth_cap)
                               : density_depth_cap;
    }
    if (current_step_purpose_ == "transition_catch_up") {
      gen_depth_cap = (gen_depth_cap >= 0) ? std::max(gen_depth_cap, 6) : 6;
    } else if (current_step_purpose_ == "interior_quality_recovery") {
      gen_depth_cap = (gen_depth_cap >= 0) ? std::max(gen_depth_cap, 5) : 5;
    }
    if (i < current_patch_adaptation_.size()) {
      current_patch_adaptation_[i].applied_patch_h = target_patch_h;
    }
    auto generate_patch_mesh = [&](double local_factor, double local_h,
                                   int local_depth, int local_retry) {
      return adapter_.AdaptDomain(p, this->id_managers_[0].get(), local_factor, local_h,
                                  local_depth, ADAPTIVE_CURRENT_STEP,
                                  static_cast<int>(i), local_retry);
    };
    double gen_factor = domain_factor;
    double gen_h = target_patch_h;
    const bool stable_mesh_damp =
        current_policy_.stable_mode && stable_curve_allows_reject_coarsen &&
        (this->step_ >= 5 ||
         (this->step_ >= 3 && stable_consecutive_reject_count_ > 0U));
    if (stable_mesh_damp) {
      constexpr double kStableLateFactorMul = 0.92;
      constexpr double kStableLateFactorFloor = 0.72;
      double mul = kStableLateFactorMul;
      double floor_v = kStableLateFactorFloor;
      if (stable_consecutive_reject_count_ > 0U && this->step_ >= 3) {
        const double ex = std::pow(
            0.972, static_cast<double>(
                       std::min(9U, stable_consecutive_reject_count_)));
        mul *= ex;
        floor_v = std::max(0.66, floor_v * ex);
      }
      if (this->step_ > 3 && stable_consecutive_reject_count_ > 0U) {
        const double st = std::pow(0.988, static_cast<double>(this->step_ - 3));
        mul *= st;
        floor_v = std::max(0.64, floor_v * st);
      }
      gen_factor = std::max(floor_v, domain_factor * mul);
      gen_factor = ClampValue(gen_factor, floor_v, PATCH_FACTOR_MAX);
      if (gen_h > 0.0) {
        gen_h *= (stable_consecutive_reject_count_ > 0U && this->step_ >= 3)
                     ? (0.88 * std::pow(0.985, static_cast<double>(std::min(
                                                    9U, stable_consecutive_reject_count_))))
                     : 0.88;
      }
    }
    SubMesh* sub_mesh = generate_patch_mesh(gen_factor, gen_h, gen_depth_cap,
                                            ADAPTIVE_CURRENT_RETRY);
    // Guard extra para patch curvo prioritário em stable tardio:
    // evita queda brusca de elementos entre candidatos rejeitados
    // (ex.: 40 -> 22), forçando uma nova tentativa mais refinada.
    const bool top_priority_curve_patch =
        (i < current_patch_adaptation_.size() &&
         current_patch_adaptation_[i].refinement_rank > 0 &&
         current_patch_adaptation_[i].refinement_rank <= 2);
    if (current_policy_.stable_mode && stable_consecutive_reject_count_ > 0U &&
        this->step_ >= 4 && top_priority_curve_patch && sub_mesh != nullptr &&
        i < last_candidate_elements_current_.size() &&
        last_candidate_elements_current_[i] > 0ULL) {
      const unsigned long long prev_candidate_faces =
          last_candidate_elements_current_[i];
      const unsigned long long curr_faces =
          static_cast<unsigned long long>(sub_mesh->GetNumberElements());
      if (curr_faces > 0ULL && curr_faces < prev_candidate_faces) {
        const double face_ratio =
            static_cast<double>(prev_candidate_faces) /
            static_cast<double>(std::max(1ULL, curr_faces));
        const double densify_scale =
            ClampValue(std::sqrt(std::max(1.0, face_ratio)) * 0.95, 1.05, 2.40);
        const double densify_h =
            (gen_h > 0.0) ? std::max(1.0e-6, gen_h / densify_scale) : gen_h;
        SubMesh* denser_mesh =
            generate_patch_mesh(gen_factor, densify_h, gen_depth_cap, 2);
        if (denser_mesh != nullptr &&
            denser_mesh->GetNumberElements() >= sub_mesh->GetNumberElements()) {
          delete sub_mesh;
          sub_mesh = denser_mesh;
          gen_h = densify_h;
        } else if (denser_mesh != nullptr) {
          delete denser_mesh;
        }
      }
    }
    const Adapter::DomainStructuralStats* structural_first =
        adapter_.GetPatchStructuralStats(static_cast<int>(i));
    if (current_policy_.stable_mode && stable_consecutive_reject_count_ > 0U &&
        this->step_ >= 4 && top_priority_curve_patch && structural_first != nullptr &&
        structural_first->generation_status == "degenerate_needs_retry") {
      const double aggressive_factor =
          ClampValue(gen_factor * 1.35, PATCH_FACTOR_MIN, PATCH_FACTOR_MAX);
      const double aggressive_h =
          (gen_h > 0.0) ? std::max(1.0e-6, gen_h * 0.58) : gen_h;
      const int aggressive_depth =
          (gen_depth_cap >= 0) ? std::max(0, gen_depth_cap - 1) : gen_depth_cap;
      SubMesh* aggressive_mesh =
          generate_patch_mesh(aggressive_factor, aggressive_h, aggressive_depth, 3);
      const Adapter::DomainStructuralStats* structural_aggressive =
          adapter_.GetPatchStructuralStats(static_cast<int>(i));
      bool keep_aggressive =
          (aggressive_mesh != nullptr &&
           aggressive_mesh->GetNumberElements() >= sub_mesh->GetNumberElements());
      if (structural_aggressive != nullptr &&
          structural_aggressive->generation_status != "degenerate_needs_retry") {
        keep_aggressive = true;
      }
      if (keep_aggressive && aggressive_mesh != nullptr) {
        delete sub_mesh;
        sub_mesh = aggressive_mesh;
        gen_factor = aggressive_factor;
        gen_h = aggressive_h;
        patch_generation_mode = "curve_transition_recovered";
        structural_first = structural_aggressive;
      } else if (aggressive_mesh != nullptr) {
        delete aggressive_mesh;
      }
    }
    const bool stable_small_mesh_step1 =
        (current_policy_.stable_mode && step_ == 1 && !elements_step_.empty() &&
         elements_step_.back() <= 32ULL);
    if (current_policy_.stable_mode && (step_ >= 2 || stable_small_mesh_step1) &&
        structural_first != nullptr && !few_patch_low_error) {
      double expected_faces = 1.0;
      if (i < current_patch_target_elements_.size() &&
          current_patch_target_elements_[i] > 1.0e-9) {
        expected_faces = std::max(1.0, current_patch_target_elements_[i]);
      } else {
        // BUDGET_DRIVEN=0 deixa targets por patch a zero; max(1,target) forçava
        // expected_faces=1 e local_face_cap~1.35, disparando reparo agressivo
        // (refino ~1.18–1.25x) em quase todo patch a partir do step 2.
        expected_faces = std::max(
            1.0, static_cast<double>(structural_first->generated_faces));
      }
      const double local_face_cap_ratio = (step_ == 2) ? 1.25 : 1.35;
      const double local_face_cap = expected_faces * local_face_cap_ratio;
      const bool local_faces_overshoot =
          (static_cast<double>(structural_first->generated_faces) >
           local_face_cap);
      const double structural_imbalance_th =
          (current_policy_.stable_mode && step_ >= 5) ? 0.17 : 0.11;
      const double structural_balance_th =
          (current_policy_.stable_mode && step_ >= 5) ? 0.76 : 0.85;
      const double transition_quality_floor =
          (current_policy_.stable_mode && step_ >= 4) ? 0.62 : 0.58;
      const bool structural_bad =
          stable_small_mesh_step1
              ? (local_faces_overshoot ||
                 (structural_first->transition_underresolved != 0))
              : ((structural_first->transition_underresolved != 0) ||
                 (structural_first->interior_transition_imbalance >
                  structural_imbalance_th) ||
                 (structural_first->curve_domain_balance < structural_balance_th) ||
                 (structural_first->transition_region.good_ratio_ge_0_60 <
                  transition_quality_floor) ||
                 (structural_first->curve_interior_coupling < 0.68) ||
                 local_faces_overshoot);
      if (structural_bad) {
        const bool transition_bad = (structural_first->transition_underresolved != 0);
        const bool dense_curved_patch =
            patch_curve_density_points >= 8 ||
            patch_curve_support_ratio < 0.95 ||
            structural_first->curve_domain_balance < structural_balance_th;
        const double rebalance_factor = ClampValue(
            domain_factor *
                (dense_curved_patch ? (transition_bad ? 1.28 : 1.22)
                                    : (transition_bad ? 1.22 : 1.18)),
            PATCH_FACTOR_MIN, PATCH_FACTOR_MAX);
        const double rebalance_h =
            (target_patch_h > 0.0)
                ? target_patch_h *
                      (dense_curved_patch
                           ? (transition_bad ? 0.72 : 0.82)
                           : (transition_bad ? 1.25 : 1.20))
                : target_patch_h;
        const int rebalance_depth =
            dense_curved_patch
                ? ((gen_depth_cap >= 0) ? std::min(32, gen_depth_cap + (transition_bad ? 2 : 1))
                                        : (transition_bad ? 6 : 5))
                : ((gen_depth_cap >= 0)
                       ? std::max(0, gen_depth_cap - (transition_bad ? 1 : 0))
                       : (transition_bad ? 3 : 4));
        SubMesh* repaired_mesh =
            generate_patch_mesh(rebalance_factor, rebalance_h, rebalance_depth, 1);
        const Adapter::DomainStructuralStats* structural_repaired =
            adapter_.GetPatchStructuralStats(static_cast<int>(i));
        bool keep_repaired = false;
        if (structural_repaired != nullptr) {
          const double first_score =
              (structural_first->transition_underresolved ? 1.0 : 0.0) +
              structural_first->interior_transition_imbalance +
              std::max(0.0, 0.90 - structural_first->curve_domain_balance) +
              Clamp01((static_cast<double>(structural_first->generated_faces) -
                       local_face_cap) /
                      std::max(1.0, local_face_cap));
          const double repaired_score =
              (structural_repaired->transition_underresolved ? 1.0 : 0.0) +
              structural_repaired->interior_transition_imbalance +
              std::max(0.0, 0.90 - structural_repaired->curve_domain_balance) +
              Clamp01((static_cast<double>(structural_repaired->generated_faces) -
                       local_face_cap) /
                      std::max(1.0, local_face_cap));
          keep_repaired = repaired_score + 1.0e-9 < first_score;
          if (!keep_repaired && dense_curved_patch && repaired_mesh != nullptr &&
              sub_mesh != nullptr) {
            const double first_balance_gap =
                std::max(0.0, structural_balance_th -
                                  structural_first->curve_domain_balance);
            const double repaired_balance_gap =
                std::max(0.0, structural_balance_th -
                                  structural_repaired->curve_domain_balance);
            const bool materially_better_curve_follow =
                repaired_balance_gap + 1.0e-6 < first_balance_gap &&
                structural_repaired->interior_transition_imbalance <=
                    structural_first->interior_transition_imbalance + 0.03;
            const bool materially_denser =
                repaired_mesh->GetNumberElements() >
                sub_mesh->GetNumberElements();
            keep_repaired =
                materially_better_curve_follow &&
                (materially_denser || structural_repaired->transition_underresolved == 0);
          }
        }
        if (keep_repaired) {
          if (sub_mesh != nullptr) {
            delete sub_mesh;
          }
          sub_mesh = repaired_mesh;
          domain_factor = rebalance_factor;
          target_patch_h = rebalance_h;
          patch_generation_mode = "curve_transition_repaired";
        } else if (repaired_mesh != nullptr) {
          delete repaired_mesh;
        }
      }
    }
    if (i < current_patch_adaptation_.size()) {
      current_patch_adaptation_[i].actual_patch_elements =
          sub_mesh ? static_cast<double>(sub_mesh->GetNumberElements()) : 0.0;
      current_patch_adaptation_[i].patch_generation_mode = patch_generation_mode;
      const Adapter::DomainStructuralStats* structural =
          adapter_.GetPatchStructuralStats(static_cast<int>(i));
      if (structural != nullptr) {
        current_patch_adaptation_[i].transition_floor = structural->transition_floor;
        current_patch_adaptation_[i].transition_underresolved =
            structural->transition_underresolved;
        current_patch_adaptation_[i].interior_transition_imbalance =
            structural->interior_transition_imbalance;
        current_patch_adaptation_[i].curve_domain_balance =
            structural->curve_domain_balance;
        current_patch_adaptation_[i].curve_interior_coupling =
            structural->curve_interior_coupling;
        current_patch_adaptation_[i].good_ratio_ge_0_60 =
            structural->good_ratio_ge_0_60;
        current_patch_adaptation_[i].boundary_element_count =
            structural->boundary_region.element_count;
        current_patch_adaptation_[i].boundary_quality_min =
            structural->boundary_region.quality_min;
        current_patch_adaptation_[i].boundary_quality_mean =
            structural->boundary_region.quality_mean;
        current_patch_adaptation_[i].boundary_good_ratio_ge_0_60 =
            structural->boundary_region.good_ratio_ge_0_60;
        current_patch_adaptation_[i].transition_band_elements =
            structural->transition_region.element_count;
        current_patch_adaptation_[i].transition_band_quality_min =
            structural->transition_region.quality_min;
        current_patch_adaptation_[i].transition_band_quality_mean =
            structural->transition_region.quality_mean;
        current_patch_adaptation_[i].transition_band_q_ge_0_60 =
            structural->transition_region.good_ratio_ge_0_60;
        current_patch_adaptation_[i].interior_core_elements =
            structural->interior_region.element_count;
        current_patch_adaptation_[i].interior_core_quality_min =
            structural->interior_region.quality_min;
        current_patch_adaptation_[i].interior_core_quality_mean =
            structural->interior_region.quality_mean;
        current_patch_adaptation_[i].interior_core_q_ge_0_60 =
            structural->interior_region.good_ratio_ge_0_60;
        current_patch_adaptation_[i].generated_faces_structural =
            structural->generated_faces;
        current_patch_adaptation_[i].generation_status_structural =
            structural->generation_status;
        if (structural->transition_underresolved != 0) {
          ++current_underresolved_transition_patch_count_real_;
        }
        current_transition_imbalance_max_real_ =
            std::max(current_transition_imbalance_max_real_,
                     structural->interior_transition_imbalance);
        structural_imbalance_sum += structural->interior_transition_imbalance;
        ++structural_count;
      }
    }
    current_eligible_patch_elements_sum_ +=
        (i < current_patch_eligible_mask_.size() && current_patch_eligible_mask_[i] == 1 &&
         sub_mesh)
            ? static_cast<unsigned long long>(sub_mesh->GetNumberElements())
            : 0ULL;
    sub_mesh->SetPatch(p);
    mesh->InsertSubMeshAdaptiveByPosition(sub_mesh, i);
    geometry->GetPatch(i)->SetSubMesh(mesh->GetSubMeshAdaptiveByPosition(i));
  }
  if (structural_count > 0U) {
    current_transition_imbalance_mean_real_ =
        structural_imbalance_sum / static_cast<double>(structural_count);
  }
  if (geometry->GetNumberPatches() > 0U) {
    current_underresolved_transition_fraction_real_ =
        static_cast<double>(current_underresolved_transition_patch_count_real_) /
        static_cast<double>(geometry->GetNumberPatches());
  }
}

std::vector<double> GeneratorAdaptive::ComputePatchAdaptationFactors(
    const Geometry* geometry) {
  const unsigned int patch_count = geometry->GetNumberPatches();
  std::vector<double> patch_factors(patch_count, 1.0);
  current_patch_adaptation_.assign(patch_count, PatchAdaptationSnapshot{});
  current_patch_eligible_mask_.assign(patch_count, 0);
  current_patch_saturated_soft_mask_.assign(patch_count, 0);
  current_patch_saturated_hard_mask_.assign(patch_count, 0);
  current_top_patch_count_ = 0U;

  if (last_submesh_error_.empty() || last_submesh_error_.size() != patch_count) {
    return patch_factors;
  }

  const double max_error =
      *std::max_element(last_submesh_error_.begin(), last_submesh_error_.end());

  if (max_error <= 0.0) {
    return patch_factors;
  }

  std::vector<unsigned int> sorted_patch_ids(patch_count, 0U);
  for (unsigned int i = 0; i < patch_count; ++i) {
    sorted_patch_ids[i] = i;
  }
  std::sort(sorted_patch_ids.begin(), sorted_patch_ids.end(),
            [this](unsigned int lhs, unsigned int rhs) {
              return last_submesh_error_[lhs] > last_submesh_error_[rhs];
            });
  std::vector<double> sorted_errors;
  sorted_errors.reserve(patch_count);
  std::vector<int> refinement_rank(patch_count, -1);
  const bool step2_phase = (BUDGET_DRIVEN_ADAPTATION == 1 && step_ == 2);
  const bool step3plus_phase = (BUDGET_DRIVEN_ADAPTATION == 1 && step_ >= 3);
  const bool few_patch_low_error_budget =
      current_policy_.stable_mode && geometry != nullptr &&
      patch_count <= 4U && !error_step_.empty() &&
      error_step_.back() < 0.12 && step_ <= 3;
  for (unsigned int rank = 0; rank < patch_count; ++rank) {
    sorted_errors.push_back(last_submesh_error_[sorted_patch_ids[rank]]);
    refinement_rank[sorted_patch_ids[rank]] = static_cast<int>(rank + 1U);
  }
  const double top_fraction =
      current_policy_.stable_mode
          ? 1.0
          : ((step_ <= 1)
          ? PATCH_TOP_ERROR_FRACTION
          : ClampValue(PATCH_TOP_ERROR_FRACTION * STEP2_TOP_ERROR_FRACTION_SCALE,
                       1.0 / std::max(1U, patch_count), 1.0));
  const unsigned int nominal_top_count = std::max(
      1U, static_cast<unsigned int>(std::ceil(top_fraction *
                                              static_cast<double>(patch_count))));
  unsigned int top_count = nominal_top_count;
  if (!current_policy_.stable_mode && step_ > 1 &&
      STEP2_HARD_SPATIAL_FILTER_MODE == 1) {
    const double shrink_scale =
        std::pow(STEP_ELEMENT_BUDGET_RETRY_SHRINK_TOPK,
                 static_cast<int>(current_budget_retry_count_));
    top_count = std::max(
        static_cast<unsigned int>(STEP2_TOP_PATCH_COUNT_MIN),
        static_cast<unsigned int>(std::ceil(static_cast<double>(nominal_top_count) *
                                            shrink_scale)));
    top_count = std::min(
        top_count,
        static_cast<unsigned int>(std::max(STEP2_TOP_PATCH_COUNT_MIN,
                                           STEP2_TOP_PATCH_COUNT_MAX)));
    top_count = std::min(top_count, patch_count);
    if (BUDGET_DRIVEN_ADAPTATION == 1 && current_target_element_growth_ > 1.0) {
      const unsigned int expanded_top_count =
          std::min(patch_count,
                   std::max(top_count,
                            static_cast<unsigned int>(STEP2_TOP_PATCH_COUNT_MAX)));
      top_count = std::max(top_count, expanded_top_count);
    }
    if (step3plus_phase) {
      top_count = std::max(
          top_count,
          std::min(patch_count,
                   std::max(static_cast<unsigned int>(4U),
                            static_cast<unsigned int>(STEP2_TOP_PATCH_COUNT_MAX))));
    }
    if (step2_phase) {
      top_count = std::max(
          top_count,
          std::min(patch_count, static_cast<unsigned int>(4U)));
    }
  }
  current_top_patch_count_ = top_count;
  const bool stable_small_mesh_step1 =
      (current_policy_.stable_mode && step_ == 1 && !elements_step_.empty() &&
       elements_step_.back() <= 32ULL);
  const double refine_cutoff =
      sorted_errors[std::min(static_cast<size_t>(top_count - 1U),
                             sorted_errors.size() - 1U)];
  const double refinement_scale =
      (step_ <= 1) ? 1.0 : STEP2_REFINEMENT_ATTENUATION * PATCH_STEP2_REFINEMENT_SCALE;
  const double refinement_scale_effective =
      refinement_scale * (stable_small_mesh_step1 ? 0.55 : 1.0);
  const double factor_mid = 1.0;
  const bool micro_refinement_active =
      (step_ > 1 && STEP2_MICRO_REFINEMENT_MODE == 1);
  const double micro_patch_scale =
      micro_refinement_active ? STEP2_MICRO_PATCH_REFINEMENT_SCALE : 1.0;
  const double budget_range_scale =
      (step_ > 1 && current_budget_retry_count_ > 0U)
          ? std::pow(STEP_ELEMENT_BUDGET_RETRY_FACTOR_RANGE_SCALE,
                     static_cast<int>(current_budget_retry_count_))
          : 1.0;
  const double pressure_range_scale =
      (step_ > 1 && STEP_ELEMENT_BUDGET_PRESSURE_MODE == 1 &&
       current_budget_pressure_ > STEP_ELEMENT_BUDGET_PRESSURE_START)
          ? STEP_ELEMENT_BUDGET_PRESSURE_FACTOR_SHRINK
          : 1.0;
  const double effective_range_shrink =
      ClampValue(((step_ <= 1) ? 1.0
                               : (STEP2_FACTOR_RANGE_SHRINK *
                                  (micro_refinement_active
                                       ? STEP2_MICRO_FACTOR_RANGE_SCALE
                                       : 1.0))) *
                     budget_range_scale * pressure_range_scale,
                 0.05, 1.0);
  const double shrunk_min =
      factor_mid - (factor_mid - PATCH_FACTOR_MIN) *
                       effective_range_shrink;
  const double shrunk_max =
      factor_mid + (PATCH_FACTOR_MAX - factor_mid) *
                       effective_range_shrink;

  for (unsigned int i = 0; i < patch_count; ++i) {
    const double local_error = last_submesh_error_[i];
    const double normalized_error = local_error / max_error;
    PatchAdaptationSnapshot snapshot;
    snapshot.patch_id = i;
    snapshot.local_error = local_error;
    snapshot.normalized_error = normalized_error;

    const Patch* patch = geometry->GetPatch(i);
    const SubMesh* sub_mesh = patch ? patch->GetSubMesh() : nullptr;
    if (sub_mesh) {
      MeshQualityStats patch_quality;
      for (unsigned int elem_idx = 0; elem_idx < sub_mesh->GetNumberElements();
           ++elem_idx) {
        const TriangleAdaptive* tri =
            static_cast<const TriangleAdaptive*>(sub_mesh->GetElement(elem_idx));
        if (!tri) {
          continue;
        }
        const bool first_sample = (patch_quality.total_elements == 0ULL);
        const double quality_3d = TriangleQualityFromNodes(
            tri->GetNoh(1), tri->GetNoh(2), tri->GetNoh(3));
        const double quality_uv = TriangleQualityFromParameters(
            tri->GetParametersN1(), tri->GetParametersN2(), tri->GetParametersN3());
        AccumulateQualitySample(quality_3d, first_sample,
                                patch_quality.quality_min_3d,
                                patch_quality.quality_mean_3d,
                                patch_quality.poor_ratio_3d,
                                patch_quality.good_ratio_ge_0_60_3d);
        AccumulateQualitySample(quality_uv, first_sample,
                                patch_quality.quality_min_uv,
                                patch_quality.quality_mean_uv,
                                patch_quality.poor_ratio_uv,
                                patch_quality.good_ratio_ge_0_60_uv);
        ++patch_quality.total_elements;
      }
      if (patch_quality.total_elements > 0ULL) {
        const double total = static_cast<double>(patch_quality.total_elements);
        patch_quality.quality_mean_3d /= total;
        patch_quality.poor_ratio_3d /= total;
        patch_quality.good_ratio_ge_0_60_3d /= total;
        patch_quality.quality_mean_uv /= total;
        patch_quality.poor_ratio_uv /= total;
        patch_quality.good_ratio_ge_0_60_uv /= total;
      }
      snapshot.quality_min_3d = patch_quality.quality_min_3d;
      snapshot.quality_mean_3d = patch_quality.quality_mean_3d;
      snapshot.poor_ratio_3d = patch_quality.poor_ratio_3d;
      snapshot.good_ratio_ge_0_60_3d = patch_quality.good_ratio_ge_0_60_3d;
      snapshot.quality_min_uv = patch_quality.quality_min_uv;
      snapshot.quality_mean_uv = patch_quality.quality_mean_uv;
      snapshot.poor_ratio_uv = patch_quality.poor_ratio_uv;
      snapshot.good_ratio_ge_0_60_uv = patch_quality.good_ratio_ge_0_60_uv;
      snapshot.quality_min = snapshot.quality_min_3d;
      snapshot.quality_mean = snapshot.quality_mean_3d;
      snapshot.poor_ratio = snapshot.poor_ratio_3d;
      snapshot.good_ratio_ge_0_60 = snapshot.good_ratio_ge_0_60_3d;
    }

    if (i < patch_history_.size()) {
      snapshot.elements_prev = patch_history_[i].prev_patch_elements;
      snapshot.local_error_drop_abs = patch_history_[i].prev_patch_error_drop_abs;
      if (patch_history_[i].prev_patch_error > 0.0) {
        snapshot.local_error_drop_pct =
            (patch_history_[i].prev_patch_error_drop_abs /
             patch_history_[i].prev_patch_error) *
            100.0;
      }
      snapshot.local_error_drop_per_element =
          patch_history_[i].prev_patch_error_drop_per_element;
      snapshot.efficiency_score = patch_history_[i].prev_patch_efficiency;
      snapshot.consecutive_inefficient_steps =
          patch_history_[i].consecutive_inefficient_steps;
    }

    const bool is_top_error_patch = local_error >= refine_cutoff;
    const bool efficiency_ok =
        (i < patch_history_.size())
            ? (patch_history_[i].prev_patch_efficiency >=
               std::max(PATCH_EFFICIENCY_TARGET, 1.0e-9))
            : true;
    const bool history_improved =
        (i < patch_history_.size()) ? (patch_history_[i].prev_patch_improved == 1)
                                    : false;
    const bool saturated_soft =
        (step_ > 1 && i < patch_history_.size() &&
         patch_history_[i].prev_patch_generated_faces >=
             static_cast<unsigned long long>(
                 std::max(1.0, QUADTREE_SATURATION_ELEMENTS_THRESHOLD)) &&
         patch_history_[i].prev_patch_efficiency <=
             QUADTREE_SATURATION_EFFICIENCY_THRESHOLD);
    const bool saturated_hard =
        (step_ > 1 && QUADTREE_HARD_SATURATION_MODE == 1 &&
         i < patch_history_.size() &&
         patch_history_[i].consecutive_inefficient_steps >=
             static_cast<unsigned int>(QUADTREE_HARD_SATURATION_STEPS));
    current_patch_saturated_soft_mask_[i] = saturated_soft ? 1 : 0;
    current_patch_saturated_hard_mask_[i] = saturated_hard ? 1 : 0;
    const bool eligible_for_refinement =
        current_policy_.stable_mode
            ? !saturated_hard
            : (!(step_ > 1 && STEP2_HARD_SPATIAL_FILTER_MODE == 1) ||
               (refinement_rank[i] > 0 &&
                refinement_rank[i] <= static_cast<int>(top_count) &&
                (efficiency_ok || history_improved || step2_phase ||
                 step3plus_phase) &&
                !saturated_soft && !saturated_hard));
    current_patch_eligible_mask_[i] = eligible_for_refinement ? 1 : 0;
    snapshot.eligible_for_refinement = eligible_for_refinement ? 1 : 0;
    snapshot.saturated_soft = saturated_soft ? 1 : 0;
    snapshot.saturated_hard = saturated_hard ? 1 : 0;
    snapshot.refinement_rank = refinement_rank[i];
    snapshot.budget_blocked =
        (step_ > 1 && current_budget_retry_count_ > 0U &&
         refinement_rank[i] > 0 &&
         refinement_rank[i] <= static_cast<int>(nominal_top_count) &&
         refinement_rank[i] > static_cast<int>(top_count))
            ? 1
            : 0;

    double factor = 1.0;
    if (PATCH_ADAPTATION_MODE == 0) {
      if (normalized_error >= 0.75) {
        factor = 0.80;
      } else if (normalized_error >= 0.50) {
        factor = 0.90;
      } else if (normalized_error >= 0.25) {
        factor = 1.00;
      } else if (local_error <= TOL_LOCAL) {
        factor = 1.12;
      } else {
        factor = 1.05;
      }
    } else {
      const double error_signal =
          std::pow(Clamp01(normalized_error), PATCH_ERROR_EXPONENT);
      const double mean_quality_deficit =
          PATCH_QUALITY_TARGET > 0.0
              ? Clamp01((PATCH_QUALITY_TARGET - snapshot.quality_mean) /
                        std::max(PATCH_QUALITY_TARGET, 1.0e-12))
              : 0.0;
      const double min_quality_deficit =
          PATCH_QUALITY_MIN_TARGET > 0.0
              ? Clamp01((PATCH_QUALITY_MIN_TARGET - snapshot.quality_min) /
                        std::max(PATCH_QUALITY_MIN_TARGET, 1.0e-12))
              : 0.0;
      const double good_quality_deficit =
          Clamp01((0.70 - snapshot.good_ratio_ge_0_60) / 0.70);
      const double quality_signal = Clamp01(
          0.50 * mean_quality_deficit + 0.25 * min_quality_deficit +
          0.15 * Clamp01(snapshot.poor_ratio /
                         std::max(ACCEPTANCE_POOR_RATIO_TARGET, 1.0e-6)) +
          0.10 * good_quality_deficit);
      const bool needs_quality_repair =
          (snapshot.quality_min > 0.0 && snapshot.quality_min < 0.25) ||
          snapshot.poor_ratio > 0.18 ||
          (stable_small_mesh_step1 &&
           ((snapshot.quality_min > 0.0 && snapshot.quality_min < 0.38) ||
            snapshot.poor_ratio > 0.08));
      const double refinement_gate =
          current_policy_.stable_mode
              ? (eligible_for_refinement
                     ? (0.55 + 0.45 * Clamp01(normalized_error))
                     : 0.35)
              : (eligible_for_refinement
                     ? 1.0
                     : ((step_ > 1 && STEP2_HARD_SPATIAL_FILTER_MODE == 1)
                            ? 0.0
                            : (is_top_error_patch ? 1.0 : 0.15)));
      double refine_signal =
          Clamp01(((1.0 - PATCH_QUALITY_WEIGHT) * error_signal +
                   PATCH_QUALITY_WEIGHT * quality_signal) *
                  refinement_gate);
      const double low_error_signal =
          Clamp01(1.0 - normalized_error);
      double coarsen_signal =
          (local_error <= TOL_LOCAL)
              ? Clamp01(low_error_signal *
                        (1.0 - Clamp01(snapshot.poor_ratio)) *
                        Clamp01(snapshot.quality_mean /
                                std::max(PATCH_QUALITY_TARGET, 1.0e-12)))
              : 0.0;
      if (PATCH_STEPWISE_MODE == 1 && i < patch_history_.size()) {
        const PatchHistoryState& history = patch_history_[i];
        const double efficiency_norm =
            Clamp01(history.prev_patch_efficiency /
                    std::max(PATCH_EFFICIENCY_TARGET, 1.0e-9));
        const double history_error_drop_signal =
            history.prev_patch_error > 0.0
                ? Clamp01(history.prev_patch_error_drop_abs /
                          std::max(history.prev_patch_error, 1.0e-9))
                : 0.0;
        const double history_quality_signal =
            Clamp01((history.prev_patch_quality_mean +
                     (1.0 - history.prev_patch_poor_ratio)) /
                    2.0);
        const double history_signal =
            PATCH_HISTORY_BLEND *
                (0.60 * efficiency_norm + 0.25 * history_error_drop_signal +
                 0.15 * history_quality_signal) +
            (1.0 - PATCH_HISTORY_BLEND) * refine_signal;
        refine_signal *= (1.0 - PATCH_EFFICIENCY_WEIGHT) +
                         PATCH_EFFICIENCY_WEIGHT * history_signal;
        if (step3plus_phase && history.prev_patch_improved == 1 &&
            history.prev_patch_decision_class == "refine") {
          refine_signal = std::max(refine_signal, 0.35);
        }
        if (step3plus_phase && history.prev_patch_improved == 0 &&
            history.prev_patch_decision_class == "refine") {
          refine_signal *= 0.60;
        }
        if (step3plus_phase && history.prev_patch_decision_class == "coarsen" &&
            normalized_error > 0.50) {
          coarsen_signal *= 0.40;
          refine_signal = std::max(refine_signal, 0.25);
        }
        const bool inefficient_patch =
            history.prev_patch_efficiency <
                std::max(1.0e-9, PATCH_EFFICIENCY_TARGET) &&
            history.prev_patch_elements >
                static_cast<unsigned long long>(
                    std::max(1.0, QUADTREE_SATURATION_ELEMENTS_THRESHOLD));
        if (inefficient_patch && !eligible_for_refinement) {
          refine_signal *= 0.35;
          coarsen_signal = std::max(coarsen_signal, 0.25);
        } else if (inefficient_patch) {
          refine_signal *= 0.65;
        }
      }
      if (eligible_for_refinement && step_ > 1) {
        double eligible_damp = STEP2_ELIGIBLE_REFINEMENT_DAMP;
        if (current_budget_retry_count_ > 0U) {
          eligible_damp *= STEP2_ELIGIBLE_REFINEMENT_DAMP_ON_RETRY;
        }
        if (snapshot.elements_prev >=
            static_cast<unsigned long long>(
                std::max(1.0, STEP2_ELIGIBLE_ELEMENTS_THRESHOLD))) {
          eligible_damp *= 0.75;
        }
        if (STEP_ELEMENT_BUDGET_PRESSURE_MODE == 1 &&
            current_budget_pressure_ > STEP_ELEMENT_BUDGET_PRESSURE_START) {
          const double over_pressure =
              Clamp01((current_budget_pressure_ - STEP_ELEMENT_BUDGET_PRESSURE_START) /
                      std::max(1.0e-9, 1.0 - STEP_ELEMENT_BUDGET_PRESSURE_START));
          eligible_damp *= (1.0 - 0.5 * over_pressure);
          snapshot.budget_pressure_local = current_budget_pressure_;
        }
        refine_signal *= ClampValue(eligible_damp, 0.0, 1.0);
        snapshot.intra_patch_damped = 1;
      }
      if (!current_policy_.stable_mode && step_ > 1 &&
          STEP2_HARD_SPATIAL_FILTER_MODE == 1 &&
          !eligible_for_refinement) {
        refine_signal = 0.0;
        coarsen_signal =
            (step3plus_phase || BUDGET_DRIVEN_ADAPTATION == 1)
                ? 0.0
                : (STEP2_NONELIGIBLE_FORCE_COARSEN == 1
                       ? std::max(coarsen_signal, 0.20)
                       : coarsen_signal);
      }
      if (step2_phase && needs_quality_repair &&
          refinement_rank[i] > 0 &&
          refinement_rank[i] <= static_cast<int>(top_count + 1U)) {
        refine_signal = std::max(refine_signal, 0.28);
        coarsen_signal *= 0.25;
      }
      factor = 1.0 - (PATCH_REFINEMENT_STRENGTH * micro_patch_scale) * refine_signal +
               PATCH_COARSENING_STRENGTH * coarsen_signal;
      if (!current_policy_.stable_mode &&
          (step_ <= 1 || STEP2_HARD_SPATIAL_FILTER_MODE == 0) &&
          !is_top_error_patch && local_error > TOL_LOCAL) {
        factor = std::max(1.0 + 0.5 * PATCH_COARSENING_STRENGTH,
                          factor + PATCH_COARSENING_STRENGTH);
      }
      factor = 1.0 - refinement_scale_effective * (1.0 - factor);
      if (PATCH_STEPWISE_MODE == 1 && i < patch_history_.size() &&
          patch_history_[i].prev_patch_efficiency <
              std::max(1.0e-9, PATCH_EFFICIENCY_TARGET)) {
        factor = std::max(factor, PATCH_INEFFICIENT_CAP);
      }
      if (!current_policy_.stable_mode && step_ > 1 &&
          STEP2_HARD_SPATIAL_FILTER_MODE == 1 &&
          !eligible_for_refinement) {
        factor = std::max(factor, STEP2_NONELIGIBLE_FACTOR_MIN);
      }
      if (saturated_hard) {
        factor = std::max(factor, QUADTREE_HARD_SATURATION_FACTOR_MIN);
      }
      if (!current_policy_.stable_mode &&
          BUDGET_DRIVEN_ADAPTATION == 1 && step_ > 1 &&
          current_target_element_growth_ > 1.0 && !eligible_for_refinement) {
        factor = 1.0;
      }
      if (!current_policy_.stable_mode && step2_phase && refinement_rank[i] > 0 &&
          refinement_rank[i] <= static_cast<int>(top_count + 1U)) {
        factor = std::min(factor, needs_quality_repair ? 0.97 : 1.0);
      }
      factor = ClampValue(factor, shrunk_min, shrunk_max);
    }
    if (current_policy_.stable_mode && step_ == 1 && !elements_step_.empty() &&
        elements_step_.back() <= 32ULL) {
      factor = std::min(factor, 1.0);
    }
    if (current_policy_.stable_mode && step_ >= 2 && geometry != nullptr &&
        geometry->GetNumberPatches() <= 6U && !error_step_.empty() &&
        error_step_.back() < 0.12) {
      factor = std::max(factor, 0.985);
      factor = std::min(factor, 1.0);
    }

    snapshot.raw_factor = factor;
    snapshot.micro_refinement_active = micro_refinement_active ? 1 : 0;
    if (snapshot.budget_pressure_local <= 0.0) {
      snapshot.budget_pressure_local = current_budget_pressure_;
    }
    const bool patch_needs_repair =
        (snapshot.quality_min > 0.0 && snapshot.quality_min < 0.25) ||
        snapshot.poor_ratio > 0.18;
    snapshot.final_refinement_class =
        patch_needs_repair
            ? "repair"
            : ((factor < 0.999) ? "refine"
                                : ((factor > 1.001) ? "coarsen" : "freeze"));
    // Em adaptive_stable tardio com rejeições consecutivas, evitar que os
    // patches prioritários curvos migrem para "repair" no candidato rejeitado
    // (isso tende a gerar malhas complementares fora do padrão esperado).
    if (current_policy_.stable_mode && stable_consecutive_reject_count_ > 0U &&
        this->step_ >= 4 && snapshot.refinement_rank > 0 &&
        snapshot.refinement_rank <= 2 &&
        snapshot.final_refinement_class == "repair") {
      snapshot.final_refinement_class = "refine";
    }
    current_patch_adaptation_[i] = snapshot;
    patch_factors[i] = factor;
  }

  if (current_policy_.stable_mode) {
    unsigned int refine_count = 0U;
    unsigned int repair_count = 0U;
    unsigned int coarsen_count = 0U;
    for (const auto& snapshot : current_patch_adaptation_) {
      if (snapshot.final_refinement_class == "repair") {
        ++repair_count;
      } else if (snapshot.final_refinement_class == "refine") {
        ++refine_count;
      } else if (snapshot.final_refinement_class == "coarsen") {
        ++coarsen_count;
      }
    }
    // Em adaptive_stable durante retries por rejeições consecutivas,
    // evitar que um ou mais patches com poor quality no candidato rejeitado
    // troquem o modo global para "repair" (isso costuma gerar malhas
    // complementares que não correspondem ao comportamento esperado).
    const bool ignore_repair_for_mode =
        (stable_consecutive_reject_count_ > 0U && this->step_ >= 4);
    current_step_mode_ = ignore_repair_for_mode
                              ? ((refine_count > 0U)
                                     ? "refine"
                                     : ((coarsen_count > 0U) ? "coarsen"
                                                               : "freeze"))
                              : ((repair_count > 0U)
                                     ? "repair"
                                     : ((refine_count > 0U) ? "refine"
                                                             : ((coarsen_count > 0U)
                                                                    ? "coarsen"
                                                                    : "freeze")));
    current_generation_control_mode_ = current_step_mode_;
  }

  current_patch_target_elements_.assign(patch_count, 0.0);
  current_patch_target_h_.assign(patch_count, -1.0);
  current_patch_quadtree_depth_cap_.assign(patch_count, -1);
  if (BUDGET_DRIVEN_ADAPTATION == 1 && current_target_elements_step_ > 0.0 &&
      PATCH_BUDGET_ALLOCATION_MODE == 1) {
    std::vector<double> allocation_score(patch_count, 0.0);
    double score_sum = 0.0;
    for (unsigned int i = 0; i < patch_count; ++i) {
      const PatchAdaptationSnapshot& snapshot = current_patch_adaptation_[i];
      const bool topk_allowed =
          (current_policy_.stable_mode || PATCH_BUDGET_TOPK_ONLY_MODE == 0 ||
           (snapshot.refinement_rank > 0 &&
            snapshot.refinement_rank <= static_cast<int>(current_top_patch_count_)));
      if (!topk_allowed) {
        allocation_score[i] = std::max(1.0e-9, PATCH_BUDGET_MIN_SHARE);
      } else {
        const PatchHistoryState* history =
            (i < patch_history_.size()) ? &patch_history_[i] : nullptr;
        const double error_budget_share =
            std::pow(std::max(0.0, snapshot.normalized_error), 1.35);
        const double quality_repair_budget_share =
            Clamp01((0.6 * std::max(0.0, PATCH_QUALITY_TARGET - snapshot.quality_mean) +
                     0.4 * std::max(0.0, PATCH_QUALITY_MIN_TARGET - snapshot.quality_min)) /
                    std::max(PATCH_QUALITY_TARGET, 1.0e-9)) +
            Clamp01(snapshot.poor_ratio / 0.25);
        const double error_term =
            PATCH_BUDGET_ERROR_WEIGHT * error_budget_share;
        const double efficiency_term =
            PATCH_BUDGET_EFFICIENCY_WEIGHT *
            Clamp01(snapshot.efficiency_score /
                    std::max(PATCH_EFFICIENCY_TARGET, 1.0e-9));
        const double quality_term =
            PATCH_BUDGET_QUALITY_WEIGHT * quality_repair_budget_share;
        const double rank_term =
            current_policy_.stable_mode
                ? 1.0
                : ((snapshot.refinement_rank > 0)
                ? (1.0 +
                   std::max(0.0,
                            static_cast<double>(current_top_patch_count_ -
                                                static_cast<unsigned int>(
                                                    std::min<int>(
                                                        snapshot.refinement_rank - 1,
                                                        static_cast<int>(current_top_patch_count_)))) /
                                std::max(1.0, static_cast<double>(current_top_patch_count_))))
                : 1.0);
        double score = (error_term * rank_term) + quality_term;
        score *= (0.20 + 0.80 * std::max(0.1, efficiency_term));
        if (step3plus_phase && history) {
          if (history->prev_patch_improved == 1) {
            score *= 1.20;
          }
          if (history->prev_patch_decision_class == "refine" &&
              history->prev_patch_improved == 0) {
            score *= 0.70;
          }
          if (history->prev_patch_decision_class == "coarsen" &&
              snapshot.normalized_error > 0.50) {
            score *= 1.10;
          }
        }
        if (snapshot.saturated_hard || snapshot.saturated_soft) {
          score *= 0.25;
        }
        if (!current_policy_.stable_mode &&
            BUDGET_DRIVEN_ADAPTATION == 1 && step_ > 1 &&
            snapshot.refinement_rank > 0 &&
            snapshot.refinement_rank <= 2) {
          score *= 1.35;
        }
        if (!current_policy_.stable_mode && step2_phase && snapshot.refinement_rank > 0 &&
            snapshot.refinement_rank <= static_cast<int>(top_count + 1U)) {
          score *= 1.15;
        }
        allocation_score[i] = std::max(PATCH_BUDGET_MIN_SHARE, score);
        current_patch_adaptation_[i].error_budget_share = error_budget_share;
        current_patch_adaptation_[i].quality_repair_budget_share =
            quality_repair_budget_share;
      }
      score_sum += allocation_score[i];
    }
    if (score_sum <= 0.0) {
      score_sum = static_cast<double>(patch_count);
      std::fill(allocation_score.begin(), allocation_score.end(), 1.0);
    }
    for (unsigned int i = 0; i < patch_count; ++i) {
      const Patch* patch = geometry->GetPatch(i);
      const SubMesh* sub_mesh = patch ? patch->GetSubMesh() : nullptr;
      double patch_area = 0.0;
      if (sub_mesh) {
        for (unsigned int elem_idx = 0; elem_idx < sub_mesh->GetNumberElements();
             ++elem_idx) {
          const TriangleAdaptive* tri =
              static_cast<const TriangleAdaptive*>(sub_mesh->GetElement(elem_idx));
          if (tri) {
            patch_area += tri->GetArea();
          }
        }
      }
      const double share = allocation_score[i] / score_sum;
      double target_patch_elements =
          std::max(1.0, current_target_elements_step_ * share);
      if (!few_patch_low_error_budget && (step2_phase || step3plus_phase) &&
          current_patch_adaptation_[i].refinement_rank > 0 &&
          current_patch_adaptation_[i].refinement_rank <=
              static_cast<int>(current_top_patch_count_)) {
        // Em adaptive_stable, após rejeição consecutiva, a base "prev"
        // (elements_prev) vem do último passo aceito. Para evitar que isso
        // cause desrefino/malhas complementares erradas (especialmente em
        // patch curvo), usamos como referência o último candidato rejeitado
        // quando disponível.
        unsigned long long prev_patch_elements_ll =
            current_patch_adaptation_[i].elements_prev;
        if (current_policy_.stable_mode &&
            stable_consecutive_reject_count_ > 0U && this->step_ >= 4 &&
            static_cast<size_t>(i) < last_candidate_elements_current_.size() &&
            last_candidate_elements_current_[static_cast<size_t>(i)] > 0ULL) {
          prev_patch_elements_ll =
              last_candidate_elements_current_[static_cast<size_t>(i)];
        }
        const double prev_patch_elements =
            std::max(1.0, static_cast<double>(prev_patch_elements_ll));
        double minimum_growth = step3plus_phase ? 1.10 : 1.20;
        if (current_policy_.stable_mode && step_ >= 3 &&
            stable_consecutive_reject_count_ > 0U) {
          const double rz = std::pow(
              0.92, static_cast<double>(
                        std::min(10U, stable_consecutive_reject_count_)));
          minimum_growth = 1.0 + (minimum_growth - 1.0) * rz;
        }
        target_patch_elements =
            std::max(target_patch_elements, prev_patch_elements * minimum_growth);
      }
      if (!few_patch_low_error_budget && step2_phase &&
          current_patch_adaptation_[i].refinement_rank > 0 &&
          current_patch_adaptation_[i].refinement_rank <=
              static_cast<int>(current_top_patch_count_ + 1U) &&
          (current_patch_adaptation_[i].quality_min < 0.25 ||
           current_patch_adaptation_[i].poor_ratio > 0.18)) {
        const double prev_patch_elements = std::max(
            1.0, static_cast<double>(current_patch_adaptation_[i].elements_prev));
        target_patch_elements =
            std::max(target_patch_elements, prev_patch_elements * 1.10);
      }
      if (few_patch_low_error_budget && step_ > 1) {
        target_patch_elements =
            std::max(1.0, static_cast<double>(
                               current_patch_adaptation_[i].elements_prev));
      }
      current_patch_target_elements_[i] = target_patch_elements;
      current_patch_adaptation_[i].patch_budget_share = share;
      current_patch_adaptation_[i].target_patch_elements = target_patch_elements;
      const double prev_patch_elements =
          std::max(1.0, static_cast<double>(current_patch_adaptation_[i].elements_prev));
      current_patch_adaptation_[i].target_patch_growth =
          target_patch_elements / prev_patch_elements;
      double target_h = SafeSqrtTargetH(patch_area, target_patch_elements);
      if (current_policy_.stable_mode && step_ == 1 && !elements_step_.empty() &&
          elements_step_.back() <= 32ULL && target_h > 1.02) {
        target_h = std::min(target_h, 0.40);
      }
      // Para "maha_4/maha_5": desbloquear profundidade de quadtree.
      // Se target_h permanecer alto, EstimateQuadtreeDepthCap fixa o depth em ~2,
      // e o número de elementos fica "travado" (ex.: 28 por patch).
      if (current_policy_.stable_mode && patch_count <= 4U && step_ >= 4 &&
          !error_step_.empty() && error_step_.back() < 0.12 && target_h > 0.25) {
        // Para depth >= 4, precisamos de target_h pequeno (raw_depth = ceil(log2(1/h))).
        target_h = 0.10;
      }
      current_patch_target_h_[i] = target_h;
      current_patch_adaptation_[i].target_patch_h = target_h;
      current_patch_quadtree_depth_cap_[i] =
          EstimateQuadtreeDepthCap(target_h, step_);
      current_patch_adaptation_[i].max_quadtree_depth_applied =
          current_patch_quadtree_depth_cap_[i];
      current_patch_adaptation_[i].quadtree_cap_applied =
          current_patch_quadtree_depth_cap_[i] >= 0 ? 1 : 0;
    }
  }

  if (BUDGET_DRIVEN_ADAPTATION == 1 && PATCH_CONSISTENCY_MODE == 1 &&
      (step_ > 1 || (current_policy_.stable_mode && step_ == 1)) &&
      !few_patch_low_error_budget) {
    RegularizeEquivalentPatchBudgets(geometry);
  }
  if (current_policy_.stable_mode && step_ == 1 && geometry != nullptr &&
      !elements_step_.empty() && elements_step_.back() <= 32ULL) {
    const unsigned int pc = geometry->GetNumberPatches();
    for (unsigned int i = 0; i < pc && i < current_patch_target_h_.size(); ++i) {
      if (current_patch_target_h_[i] > 1.02) {
        const double clamped = std::min(current_patch_target_h_[i], 0.40);
        current_patch_target_h_[i] = clamped;
        if (i < current_patch_adaptation_.size()) {
          current_patch_adaptation_[i].target_patch_h = clamped;
        }
        current_patch_quadtree_depth_cap_[i] =
            EstimateQuadtreeDepthCap(clamped, step_);
        if (i < current_patch_adaptation_.size()) {
          current_patch_adaptation_[i].max_quadtree_depth_applied =
              current_patch_quadtree_depth_cap_[i];
        }
      }
    }
  }

  patch_factors = SmoothPatchAdaptationFactors(geometry, patch_factors);
  const std::vector<double> limited = ApplyAdaptationRateLimit(patch_factors);
  if (current_policy_.stable_mode && PATCH_CONSISTENCY_MODE == 1 &&
      step_ >= 1) {
    std::map<int, std::vector<unsigned int>> grouped;
    for (unsigned int i = 0; i < current_patch_adaptation_.size(); ++i) {
      const int group = current_patch_adaptation_[i].similar_patch_group;
      if (group >= 0) {
        grouped[group].push_back(i);
      }
    }
    for (const auto& entry : grouped) {
      const std::vector<unsigned int>& members = entry.second;
      if (members.size() < 2) {
        continue;
      }
      std::vector<double> group_factors;
      group_factors.reserve(members.size());
      for (const unsigned int idx : members) {
        group_factors.push_back(limited[idx]);
      }
      std::sort(group_factors.begin(), group_factors.end());
      const double median_factor = group_factors[group_factors.size() / 2];
      const double min_factor = group_factors.front();
      for (const unsigned int idx : members) {
        const double before = patch_factors[idx];
        double after = 0.35 * limited[idx] + 0.65 * median_factor;
        if (median_factor < 0.999) {
          after = std::min(after, median_factor + 0.004);
          after = std::min(after, min_factor + 0.010);
        }
        patch_factors[idx] = ClampValue(after, PATCH_FACTOR_MIN, PATCH_FACTOR_MAX);
        if (std::fabs(patch_factors[idx] - before) > 1.0e-9) {
          current_patch_adaptation_[idx].consistency_clamped = 1;
          current_consistency_regularization_active_ = 1;
        }
      }
    }
  }
  const bool stable_low_error_small_mesh =
      current_policy_.stable_mode && step_ >= 2 && geometry != nullptr &&
      geometry->GetNumberPatches() <= 6U && !error_step_.empty() &&
      error_step_.back() < 0.12;
  for (unsigned int i = 0; i < current_patch_adaptation_.size(); ++i) {
    double limited_factor = patch_factors[i];
    if (step2_phase && !stable_low_error_small_mesh &&
        current_patch_adaptation_[i].refinement_rank > 0 &&
        current_patch_adaptation_[i].refinement_rank <=
            static_cast<int>(current_top_patch_count_ + 1U)) {
      limited_factor = std::min(
          limited_factor,
          (current_patch_adaptation_[i].quality_min < 0.25 ||
           current_patch_adaptation_[i].poor_ratio > 0.18)
              ? 0.97
              : 1.0);
    }
    current_patch_adaptation_[i].limited_factor = limited_factor;
    patch_factors[i] = limited_factor;
  }
  if (stable_low_error_small_mesh) {
    for (unsigned int i = 0; i < patch_factors.size(); ++i) {
      patch_factors[i] = 1.0;
      current_patch_adaptation_[i].limited_factor = 1.0;
    }
  }
  return patch_factors;
}

void GeneratorAdaptive::RegularizeEquivalentPatchBudgets(
    const Geometry* geometry) {
  const unsigned int patch_count = geometry ? geometry->GetNumberPatches() : 0U;
  if (PATCH_CONSISTENCY_MODE != 1 || patch_count == 0 ||
      current_patch_target_elements_.size() != patch_count ||
      current_patch_target_h_.size() != patch_count ||
      current_patch_adaptation_.size() != patch_count) {
    current_consistency_regularization_active_ = 0;
    return;
  }

  std::vector<int> groups(patch_count, -1);
  int next_group = 0;
  std::vector<double> patch_areas(patch_count, 0.0);
  std::vector<unsigned int> patch_boundary_curves(patch_count, 0U);
  for (unsigned int i = 0; i < patch_count; ++i) {
    const Patch* patch = geometry->GetPatch(i);
    const PatchCoons* coons = static_cast<const PatchCoons*>(patch);
    patch_areas[i] = ComputePatchArea(patch);
    patch_boundary_curves[i] = CountBoundaryCurves(coons);
  }
  auto are_similar = [&](unsigned int lhs, unsigned int rhs) {
    const PatchAdaptationSnapshot& a = current_patch_adaptation_[lhs];
    const PatchAdaptationSnapshot& b = current_patch_adaptation_[rhs];
    const bool step1_mode = current_policy_.stable_mode && step_ == 1;
    const bool boundary_ok = patch_boundary_curves[lhs] == patch_boundary_curves[rhs];
    const bool area_geom_ok =
        RelativeDifference(std::max(1.0e-9, patch_areas[lhs]),
                           std::max(1.0e-9, patch_areas[rhs])) <=
        (step1_mode ? std::max(0.40, PATCH_CONSISTENCY_AREA_TOL * 2.0)
                    : PATCH_CONSISTENCY_AREA_TOL);
    const bool area_ok =
        RelativeDifference(std::max(1.0, static_cast<double>(a.elements_prev)),
                           std::max(1.0, static_cast<double>(b.elements_prev))) <=
        (step1_mode ? std::max(0.55, PATCH_CONSISTENCY_AREA_TOL * 2.5)
                    : PATCH_CONSISTENCY_AREA_TOL);
    const bool error_ok =
        RelativeDifference(a.normalized_error, b.normalized_error) <=
        PATCH_CONSISTENCY_ERROR_TOL;
    const double curvature_proxy_a = std::fabs(a.raw_factor - 1.0);
    const double curvature_proxy_b = std::fabs(b.raw_factor - 1.0);
    const bool curvature_ok =
        RelativeDifference(curvature_proxy_a, curvature_proxy_b) <=
        PATCH_CONSISTENCY_CURVATURE_TOL;
    const bool quality_ok =
        RelativeDifference(std::max(0.05, a.quality_mean + (1.0 - a.poor_ratio)),
                           std::max(0.05, b.quality_mean + (1.0 - b.poor_ratio))) <=
        PATCH_CONSISTENCY_QUALITY_TOL;
    if (step1_mode) {
      return boundary_ok && (area_geom_ok || area_ok);
    }
    return area_geom_ok && area_ok && error_ok && curvature_ok && quality_ok;
  };

  for (unsigned int i = 0; i < patch_count; ++i) {
    if (groups[i] >= 0) {
      continue;
    }
    groups[i] = next_group++;
    std::vector<unsigned int> queue(1, i);
    while (!queue.empty()) {
      const unsigned int current = queue.back();
      queue.pop_back();
      const Patch* patch = geometry->GetPatch(current);
      if (!patch) {
        continue;
      }
      const PatchCoons* coons = static_cast<const PatchCoons*>(patch);
      for (unsigned int c = 0; c < coons->GetNumBerCurves(); ++c) {
        CurveAdaptive* curve = const_cast<CurveAdaptive*>(coons->GetCurve(c));
        for (unsigned int p = 0; p < curve->GetNumBerPatches(); ++p) {
          Patch* neighbor_patch = curve->GetPatch(p);
          const unsigned int neighbor = ResolvePatchIndex(geometry, neighbor_patch);
          if (neighbor == std::numeric_limits<unsigned int>::max() ||
              neighbor == current) {
            continue;
          }
          if (PATCH_CONSISTENCY_NEIGHBOR_ONLY == 1 && groups[neighbor] >= 0 &&
              groups[neighbor] != groups[current]) {
            continue;
          }
          if (!are_similar(current, neighbor)) {
            continue;
          }
          if (groups[neighbor] < 0) {
            groups[neighbor] = groups[current];
            queue.push_back(neighbor);
          }
        }
      }
    }
  }

  const double max_ratio =
      (step_ == 2) ? PATCH_CONSISTENCY_MAX_BUDGET_RATIO_STEP2
                   : PATCH_CONSISTENCY_MAX_BUDGET_RATIO_STEPN;
  std::map<int, std::vector<unsigned int>> grouped;
  for (unsigned int i = 0; i < patch_count; ++i) {
    grouped[groups[i]].push_back(i);
  }

  current_consistency_regularization_active_ = 0;
  std::vector<double> budget_ratios;
  std::vector<double> h_ratios;
  for (const auto& entry : grouped) {
    const std::vector<unsigned int>& members = entry.second;
    if (members.size() < static_cast<size_t>(std::max(2, STEP2_GROUP_MIN_PATCHES))) {
      for (unsigned int idx : members) {
        current_patch_adaptation_[idx].similar_patch_group = -1;
      }
      continue;
    }

    double anchor_error = -1.0;
    unsigned int anchor = members.front();
    std::vector<double> budgets;
    std::vector<double> hs;
    for (unsigned int idx : members) {
      PatchAdaptationSnapshot& snapshot = current_patch_adaptation_[idx];
      snapshot.similar_patch_group = entry.first;
      snapshot.budget_before_consistency = current_patch_target_elements_[idx];
      snapshot.target_h_before_consistency = current_patch_target_h_[idx];
      if (current_policy_.stable_mode && step_ == 1) {
        const double area_similarity =
            1.0 - Clamp01(RelativeDifference(
                      std::max(1.0e-9, patch_areas[idx]),
                      std::max(1.0e-9, patch_areas[anchor])));
        snapshot.similarity_score = area_similarity;
      } else {
        snapshot.similarity_score =
            1.0 - std::min(1.0, 0.5 * snapshot.normalized_error +
                                    0.5 * snapshot.poor_ratio);
      }
      snapshot.similarity_score_to_neighbors = snapshot.similarity_score;
      if (snapshot.local_error > anchor_error) {
        anchor_error = snapshot.local_error;
        anchor = idx;
      }
      budgets.push_back(current_patch_target_elements_[idx]);
      if (current_patch_target_h_[idx] > 0.0) {
        hs.push_back(current_patch_target_h_[idx]);
      }
    }
    std::sort(budgets.begin(), budgets.end());
    const double median_budget = budgets[budgets.size() / 2];
    const double anchor_budget = std::max(1.0, current_patch_target_elements_[anchor]);
    const double budget_floor =
        std::max(median_budget * PATCH_CONSISTENCY_MIN_GROUP_SHARE,
                 anchor_budget / std::max(1.0, max_ratio));
    const double budget_ceil =
        std::max(budget_floor, anchor_budget * std::max(1.0, max_ratio));
    std::sort(hs.begin(), hs.end());
    const double median_h = hs.empty() ? -1.0 : hs[hs.size() / 2];
    double anchor_h = current_patch_target_h_[anchor];
    if (anchor_h <= 0.0 && median_h > 0.0) {
      anchor_h = median_h;
    }
    if (anchor_h <= 0.0) {
      anchor_h = 1.0;
    }
    double stable_group_h = anchor_h;
    if (current_policy_.stable_mode && step_ > 1 && median_h > 0.0) {
      stable_group_h = std::min(median_h, anchor_h * 1.10);
    }

    const bool skip_depth_clamp_small_step1 =
        current_policy_.stable_mode && step_ == 1 && !elements_step_.empty() &&
        elements_step_.back() <= 32ULL;
    for (unsigned int idx : members) {
      PatchAdaptationSnapshot& snapshot = current_patch_adaptation_[idx];
      snapshot.group_anchor_patch = static_cast<int>(anchor);
      snapshot.is_consistency_anchor = (idx == anchor) ? 1 : 0;
      const double before_budget = current_patch_target_elements_[idx];
      const double before_h = current_patch_target_h_[idx];
      double after_budget = ClampValue(before_budget, budget_floor, budget_ceil);
      if (current_policy_.stable_mode && step_ >= 1) {
        after_budget = std::max(
            after_budget,
            current_patch_target_elements_[anchor] *
                current_policy_.equivalent_patch_min_ratio);
      }
      if (current_policy_.stable_mode && step_ == 1 && !elements_step_.empty() &&
          elements_step_.back() <= 32ULL) {
        const double prev_el =
            std::max(1.0, static_cast<double>(snapshot.elements_prev));
        after_budget = std::max(after_budget, prev_el * 1.08);
      }
      if (STEP2_GROUP_NEIGHBOR_PROTECTION == 1 &&
          current_patch_adaptation_[anchor].refinement_rank > 0 &&
          snapshot.refinement_rank > static_cast<int>(current_top_patch_count_) &&
          step_ == 2) {
        after_budget = std::max(after_budget, budget_floor);
      }
      current_patch_target_elements_[idx] = after_budget;
      snapshot.target_patch_elements = after_budget;
      snapshot.budget_after_consistency = after_budget;
      snapshot.group_budget_ratio =
          after_budget / std::max(1.0, current_patch_target_elements_[anchor]);

      double after_h = before_h;
      if (before_h > 0.0) {
        after_h = ClampValue(before_h,
                             anchor_h / std::max(1.0, PATCH_CONSISTENCY_H_CLAMP_RATIO),
                             anchor_h * std::max(1.0, PATCH_CONSISTENCY_H_CLAMP_RATIO));
        if (current_policy_.stable_mode && step_ >= 1 && stable_group_h > 0.0) {
          after_h = std::min(after_h, stable_group_h);
        }
        current_patch_target_h_[idx] = after_h;
        snapshot.target_patch_h = after_h;
      }
      snapshot.target_h_after_consistency = after_h;
      if (PATCH_CONSISTENCY_DEPTH_CLAMP == 1 && !skip_depth_clamp_small_step1 &&
          anchor < current_patch_quadtree_depth_cap_.size() &&
          idx < current_patch_quadtree_depth_cap_.size() &&
          current_patch_quadtree_depth_cap_[anchor] >= 0 &&
          current_patch_quadtree_depth_cap_[idx] >= 0) {
        current_patch_quadtree_depth_cap_[idx] = std::max(
            current_patch_quadtree_depth_cap_[anchor] - 1,
            std::min(current_patch_quadtree_depth_cap_[anchor] + 1,
                     current_patch_quadtree_depth_cap_[idx]));
        snapshot.max_quadtree_depth_applied =
            current_patch_quadtree_depth_cap_[idx];
      }
      snapshot.consistency_clamped =
          (std::fabs(after_budget - before_budget) > 1.0e-9 ||
           ((before_h > 0.0 || after_h > 0.0) &&
            std::fabs(after_h - before_h) > 1.0e-9))
              ? 1
              : 0;
      current_consistency_regularization_active_ =
          std::max(current_consistency_regularization_active_,
                   snapshot.consistency_clamped);
    }

    std::vector<double> group_budgets;
    std::vector<double> group_hs;
    for (unsigned int idx : members) {
      group_budgets.push_back(current_patch_target_elements_[idx]);
      if (current_patch_target_h_[idx] > 0.0) {
        group_hs.push_back(current_patch_target_h_[idx]);
      }
    }
    budget_ratios.push_back(ComputeNormalizedDispersion(group_budgets));
    if (!group_hs.empty()) {
      h_ratios.push_back(ComputeNormalizedDispersion(group_hs));
    }
  }

  current_patch_budget_dispersion_ = ComputeNormalizedDispersion(budget_ratios);
  current_patch_h_dispersion_ = ComputeNormalizedDispersion(h_ratios);
  current_patch_consistency_penalty_ =
      0.65 * current_patch_budget_dispersion_ + 0.35 * current_patch_h_dispersion_;
}

void GeneratorAdaptive::RegularizeCurveTargetsFromPatchConsistency(
    const Geometry* geometry) {
  if (CURVE_CONSISTENCY_MODE != 1 || !geometry ||
      current_curve_target_points_.size() != geometry->GetNumberCurves()) {
    return;
  }
  std::vector<double> balance_ratios;
  for (unsigned int i = 0; i < geometry->GetNumberCurves(); ++i) {
    CurveAdaptive* curve = const_cast<CurveAdaptive*>(geometry->GetCurve(i));
    std::vector<unsigned int> patch_ids;
    for (unsigned int p = 0; p < curve->GetNumBerPatches(); ++p) {
      const unsigned int patch_id =
          ResolvePatchIndex(geometry, curve->GetPatch(p));
      if (patch_id != std::numeric_limits<unsigned int>::max() &&
          patch_id < current_patch_adaptation_.size()) {
        patch_ids.push_back(patch_id);
      }
    }
    CurveAdaptationSnapshot& snapshot = current_curve_adaptation_[i];
    snapshot.target_points_before_consistency =
        static_cast<int>(std::round(current_curve_target_points_[i]));
    snapshot.target_points_after_consistency =
        snapshot.target_points_before_consistency;
    if (patch_ids.size() < 2) {
      snapshot.curve_patch_balance_ratio = 1.0;
      continue;
    }

    double min_budget = std::numeric_limits<double>::max();
    double max_budget = 0.0;
    bool same_group = true;
    int group = current_patch_adaptation_[patch_ids.front()].similar_patch_group;
    bool touches_priority_patch = false;
    double min_target_h = std::numeric_limits<double>::max();
    for (unsigned int patch_id : patch_ids) {
      min_budget = std::min(min_budget, current_patch_target_elements_[patch_id]);
      max_budget = std::max(max_budget, current_patch_target_elements_[patch_id]);
      if (patch_id < current_patch_target_h_.size() &&
          current_patch_target_h_[patch_id] > 0.0) {
        min_target_h = std::min(min_target_h, current_patch_target_h_[patch_id]);
      }
      same_group = same_group &&
                   (current_patch_adaptation_[patch_id].similar_patch_group == group) &&
                   group >= 0;
      if (current_patch_adaptation_[patch_id].refinement_rank > 0 &&
          current_patch_adaptation_[patch_id].refinement_rank <=
              static_cast<int>(current_top_patch_count_)) {
        touches_priority_patch = true;
      }
    }
    snapshot.curve_patch_balance_ratio =
        max_budget / std::max(1.0, min_budget);
    double max_ratio =
        (step_ == 2) ? CURVE_CONSISTENCY_MAX_POINT_RATIO_STEP2
                     : (CURVE_CONSISTENCY_MAX_POINT_RATIO_STEP2 + 0.5);
    const int before = static_cast<int>(std::round(current_curve_target_points_[i]));
    const bool low_h_sensitive =
        min_target_h < std::numeric_limits<double>::max() && min_target_h <= 0.14;
    const bool force_curve_alignment =
        current_policy_.stable_mode && step_ >= 2 &&
        (before < snapshot.geometric_target_points) &&
        (touches_priority_patch || snapshot.sensitive_curve == 1 || low_h_sensitive);
    if ((same_group && snapshot.curve_patch_balance_ratio > max_ratio) ||
        force_curve_alignment) {
      const int elevated = static_cast<int>(std::round(
          CURVE_CONSISTENCY_GROUP_BLEND * static_cast<double>(before) +
          (1.0 - CURVE_CONSISTENCY_GROUP_BLEND) *
              static_cast<double>(before) * std::min(max_ratio, snapshot.curve_patch_balance_ratio)));
      int after = std::max(before, elevated);
      if (current_policy_.stable_mode && step_ >= 2) {
        after = std::max(after, snapshot.geometric_target_points);
      }
      if (touches_priority_patch && CURVE_CONSISTENCY_PRIORITY_BONUS == 1) {
        after += 1;
      }
      current_curve_target_points_[i] = static_cast<double>(after);
      snapshot.curve_consistency_clamped = 1;
      snapshot.target_points_after_consistency = after;
      current_consistency_regularization_active_ = 1;
    }
    balance_ratios.push_back(std::max(0.0, snapshot.curve_patch_balance_ratio - 1.0));
  }
  current_curve_balance_dispersion_ = ComputeNormalizedDispersion(balance_ratios);
  current_patch_consistency_penalty_ += 0.20 * current_curve_balance_dispersion_;
}

std::vector<double> GeneratorAdaptive::SmoothPatchAdaptationFactors(
    const Geometry* geometry, const std::vector<double>& raw_factors) {
  std::vector<double> smoothed_factors = raw_factors;

  if (raw_factors.size() != geometry->GetNumberPatches()) {
    return smoothed_factors;
  }

  for (unsigned int i = 0; i < geometry->GetNumberPatches(); ++i) {
    const PatchCoons* patch =
        static_cast<const PatchCoons*>(geometry->GetPatch(i));
    double neighbor_sum = 0.0;
    int neighbor_count = 0;

    for (unsigned int c = 0; c < patch->GetNumBerCurves(); ++c) {
      CurveAdaptive* curve = const_cast<CurveAdaptive*>(patch->GetCurve(c));

      for (unsigned int p = 0; p < curve->GetNumBerPatches(); ++p) {
        const Patch* neighbor = curve->GetPatch(p);
        if (!neighbor || neighbor == patch) {
          continue;
        }

        const unsigned int neighbor_id = neighbor->GetId();
        if (neighbor_id >= raw_factors.size()) {
          continue;
        }
        if (step_ > 1 && STEP2_HARD_SPATIAL_FILTER_MODE == 1 &&
            static_cast<size_t>(i) < current_patch_eligible_mask_.size() &&
            static_cast<size_t>(neighbor_id) < current_patch_eligible_mask_.size() &&
            current_patch_eligible_mask_[neighbor_id] !=
                current_patch_eligible_mask_[i]) {
          continue;
        }

        neighbor_sum += raw_factors[neighbor_id];
        ++neighbor_count;
      }
    }

    if (neighbor_count > 0) {
      const double neighbor_mean =
          neighbor_sum / static_cast<double>(neighbor_count);
      const bool hard_spatial =
          (step_ > 1 && STEP2_HARD_SPATIAL_FILTER_MODE == 1);
      const double neighbor_weight =
          hard_spatial ? STEP2_SMOOTHING_NEIGHBOR_WEIGHT : 0.10;
      smoothed_factors[i] =
          (1.0 - neighbor_weight) * raw_factors[i] +
          neighbor_weight * neighbor_mean;
    }

    smoothed_factors[i] =
        ClampValue(smoothed_factors[i], PATCH_FACTOR_MIN, PATCH_FACTOR_MAX);
    if (step_ > 1 && STEP2_HARD_SPATIAL_FILTER_MODE == 1 &&
        static_cast<size_t>(i) < current_patch_eligible_mask_.size() &&
        current_patch_eligible_mask_[i] == 0) {
      smoothed_factors[i] =
          std::max(smoothed_factors[i], STEP2_NONELIGIBLE_FACTOR_MIN);
    }
    if (i < current_patch_adaptation_.size()) {
      current_patch_adaptation_[i].smoothed_factor = smoothed_factors[i];
    }
  }

  return smoothed_factors;
}

std::vector<double> GeneratorAdaptive::ApplyAdaptationRateLimit(
    const std::vector<double>& target_factors) {
  std::vector<double> limited_factors = target_factors;

  if (limited_factors.empty()) {
    return limited_factors;
  }

  if (previous_patch_factors_.size() != limited_factors.size()) {
    previous_patch_factors_.assign(limited_factors.size(), 1.0);
  }

  for (unsigned int i = 0; i < limited_factors.size(); ++i) {
    const double previous = previous_patch_factors_[i];
    const double relaxed =
        previous + current_adaptation_relaxation_ * (target_factors[i] - previous);
    const double lower = previous - current_adaptation_max_delta_;
    const double upper = previous + current_adaptation_max_delta_;
    limited_factors[i] = std::max(lower, std::min(upper, relaxed));
    limited_factors[i] =
        ClampValue(limited_factors[i], PATCH_FACTOR_MIN, PATCH_FACTOR_MAX);
  }

  previous_patch_factors_ = limited_factors;
  return limited_factors;
}

double GeneratorAdaptive::ComputeCurveAdaptationFactor(
    CurveAdaptive* curve, const std::vector<double>& patch_factors) const {
  double factor = 1.0;
  bool found_patch = false;
  double factor_sum = 0.0;
  double weight_sum = 0.0;
  unsigned int contributing_patches = 0U;

  for (unsigned int i = 0; i < curve->GetNumBerPatches(); ++i) {
    Patch* patch = curve->GetPatch(i);
    if (!patch) {
      continue;
    }

    const unsigned int patch_id = patch->GetId();
    if (patch_id >= patch_factors.size()) {
      continue;
    }

    factor = found_patch ? std::min(factor, patch_factors[patch_id])
                         : patch_factors[patch_id];
    factor_sum += patch_factors[patch_id];
    double weight = 1.0;
    if (CURVE_ADAPTATION_POLICY == 3 && patch_id < patch_history_.size()) {
      const double error_weight =
          (patch_id < last_submesh_error_.size())
              ? std::max(last_submesh_error_[patch_id], 1.0e-12)
              : 1.0;
      const double efficiency_weight =
          std::max(1.0e-12, patch_history_[patch_id].prev_patch_efficiency /
                                  std::max(PATCH_EFFICIENCY_TARGET, 1.0e-9));
      weight = (1.0 - CURVE_EFFICIENCY_WEIGHT) * error_weight +
               CURVE_EFFICIENCY_WEIGHT * efficiency_weight;
    } else if (patch_id < last_submesh_error_.size()) {
      weight = std::max(last_submesh_error_[patch_id], 1.0e-12);
    }
    weight_sum += weight;
    found_patch = true;
    ++contributing_patches;
  }

  if (!found_patch) {
    return 1.0;
  }

  const double mean_factor =
      contributing_patches > 0U
          ? factor_sum / static_cast<double>(contributing_patches)
          : factor;
  double weighted_factor = factor;
  if ((CURVE_ADAPTATION_POLICY == 2 || CURVE_ADAPTATION_POLICY == 3) &&
      weight_sum > 0.0) {
    weighted_factor = 0.0;
    for (unsigned int i = 0; i < curve->GetNumBerPatches(); ++i) {
      Patch* patch = curve->GetPatch(i);
      if (!patch) {
        continue;
      }
      const unsigned int patch_id = patch->GetId();
      if (patch_id >= patch_factors.size()) {
        continue;
      }
      double weight = 1.0;
      if (CURVE_ADAPTATION_POLICY == 3 && patch_id < patch_history_.size()) {
        const double error_weight =
            (patch_id < last_submesh_error_.size())
                ? std::max(last_submesh_error_[patch_id], 1.0e-12)
                : 1.0;
        const double efficiency_weight =
            std::max(1.0e-12, patch_history_[patch_id].prev_patch_efficiency /
                                    std::max(PATCH_EFFICIENCY_TARGET, 1.0e-9));
        weight = (1.0 - CURVE_EFFICIENCY_WEIGHT) * error_weight +
                 CURVE_EFFICIENCY_WEIGHT * efficiency_weight;
      } else {
        weight = (patch_id < last_submesh_error_.size())
                     ? std::max(last_submesh_error_[patch_id], 1.0e-12)
                     : 1.0;
      }
      weighted_factor += (weight / weight_sum) * patch_factors[patch_id];
    }
  }

  double base_factor = factor;
  if (CURVE_ADAPTATION_POLICY == 1) {
    base_factor = CURVE_ADAPTATION_BLEND * factor +
                  (1.0 - CURVE_ADAPTATION_BLEND) * mean_factor;
  } else if (CURVE_ADAPTATION_POLICY == 2 || CURVE_ADAPTATION_POLICY == 3) {
    base_factor = CURVE_ADAPTATION_BLEND * factor +
                  (1.0 - CURVE_ADAPTATION_BLEND) * weighted_factor;
  }

  const double sensitivity_scale =
      (step_ <= 1) ? 1.0 : STEP2_CURVE_SENSITIVITY_SCALE;
  const double attenuated_factor =
      1.0 + (CURVE_FACTOR_SENSITIVITY * sensitivity_scale) * (base_factor - 1.0);
  return ClampValue(attenuated_factor, PATCH_FACTOR_MIN, PATCH_FACTOR_MAX);
}

GeneratorAdaptive::MeshQualityStats GeneratorAdaptive::ComputeMeshQualityStats(
    const MeshAdaptive* mesh) const {
  MeshQualityStats stats;
  if (!mesh) {
    return stats;
  }

  std::vector<double> min_angles_3d;
  std::vector<double> max_angles_3d;

  for (unsigned int sub_idx = 0; sub_idx < mesh->GetNumberSubMeshesAdaptive();
       ++sub_idx) {
    const SubMesh* sub = mesh->GetSubMeshAdaptiveByPosition(sub_idx);
    if (!sub) {
      continue;
    }
    for (unsigned int elem_idx = 0; elem_idx < sub->GetNumberElements();
         ++elem_idx) {
      const TriangleAdaptive* tri =
          static_cast<const TriangleAdaptive*>(sub->GetElement(elem_idx));
      if (!tri) {
        continue;
      }
      const bool first_sample = (stats.total_elements == 0ULL);
      const double quality_3d = TriangleQualityFromNodes(
          tri->GetNoh(1), tri->GetNoh(2), tri->GetNoh(3));
      const double quality_uv = TriangleQualityFromParameters(
          tri->GetParametersN1(), tri->GetParametersN2(), tri->GetParametersN3());
      const TriangleShapeMetrics shape_3d = ComputeTriangleShapeMetrics3D(
          tri->GetNoh(1), tri->GetNoh(2), tri->GetNoh(3));
      AccumulateQualitySample(quality_3d, first_sample, stats.quality_min_3d,
                              stats.quality_mean_3d, stats.poor_ratio_3d,
                              stats.good_ratio_ge_0_60_3d);
      AccumulateQualitySample(quality_uv, first_sample, stats.quality_min_uv,
                              stats.quality_mean_uv, stats.poor_ratio_uv,
                              stats.good_ratio_ge_0_60_uv);
      stats.angle_quality_mean_3d += shape_3d.angle_quality;
      stats.edge_ratio_mean_3d += shape_3d.edge_ratio;
      stats.min_angle_mean_3d += shape_3d.min_angle_deg;
      stats.max_angle_mean_3d += shape_3d.max_angle_deg;
      min_angles_3d.push_back(shape_3d.min_angle_deg);
      max_angles_3d.push_back(shape_3d.max_angle_deg);
      ++stats.total_elements;
    }
  }

  if (stats.total_elements > 0ULL) {
    const double total = static_cast<double>(stats.total_elements);
    stats.quality_mean_3d /= total;
    stats.poor_ratio_3d /= total;
    stats.good_ratio_ge_0_60_3d /= total;
    stats.quality_mean_uv /= total;
    stats.poor_ratio_uv /= total;
    stats.good_ratio_ge_0_60_uv /= total;
    stats.angle_quality_mean_3d /= total;
    stats.edge_ratio_mean_3d /= total;
    stats.min_angle_mean_3d /= total;
    stats.max_angle_mean_3d /= total;
    std::sort(min_angles_3d.begin(), min_angles_3d.end());
    std::sort(max_angles_3d.begin(), max_angles_3d.end());
    const size_t idx_p05 =
        static_cast<size_t>(0.05 * static_cast<double>(min_angles_3d.size() - 1));
    const size_t idx_p95 =
        static_cast<size_t>(0.95 * static_cast<double>(max_angles_3d.size() - 1));
    stats.min_angle_p05_3d = min_angles_3d[idx_p05];
    stats.max_angle_p95_3d = max_angles_3d[idx_p95];
  }

  stats.quality_min = stats.quality_min_3d;
  stats.quality_mean = stats.quality_mean_3d;
  stats.poor_ratio = stats.poor_ratio_3d;
  stats.good_ratio_ge_0_60 = stats.good_ratio_ge_0_60_3d;

  return stats;
}

void GeneratorAdaptive::WritePatchAdaptationLog(int step, bool accepted) const {
  if (!WRITE_ADAPTATION_DEBUG) {
    return;
  }

  std::ostringstream name;
  name << NAME_MODEL << "_passo_" << step << "_patch_adaptation_"
       << (accepted ? "accepted" : "rejected") << ".csv";
  EnsureOutputParentDirectories(name.str());
  std::ofstream file(name.str().c_str());
  if (!file.good()) {
    return;
  }

  file << "patch_id,last_submesh_error,normalized_error,quality_min,quality_mean,"
          "good_ratio_ge_0_60,poor_ratio,"
          "quality_min_3d,quality_mean_3d,q_ge_0_60_3d,poor_ratio_3d,"
          "quality_min_uv,quality_mean_uv,q_ge_0_60_uv,poor_ratio_uv,"
          "raw_factor,smoothed_factor,limited_factor,elements_prev,"
          "elements_current,elements_delta,local_error_drop_abs,local_error_drop_pct,"
          "local_error_drop_per_element,efficiency_score,eligible_for_refinement,"
          "saturated_soft,saturated_hard,consecutive_inefficient_steps,budget_blocked,"
          "refinement_rank,final_refinement_class,eligible_h_scale_applied,"
          "micro_refinement_active,budget_pressure_local,intra_patch_damped,"
          "local_patch_budget,patch_budget_exceeded,patch_generation_mode,"
          "target_patch_elements,actual_patch_elements,patch_budget_share,error_budget_share,quality_repair_budget_share,"
          "similar_patch_group,similarity_score,similarity_score_to_neighbors,is_consistency_anchor,consistency_clamped,"
          "budget_before_consistency,budget_after_consistency,target_h_before_consistency,target_h_after_consistency,"
          "group_anchor_patch,group_budget_ratio,"
          "target_patch_growth,target_patch_h,applied_patch_h,quadtree_cap_applied,"
          "max_quadtree_depth_applied,transition_floor,transition_underresolved,"
          "interior_transition_imbalance,curve_domain_balance,curve_interior_coupling,"
          "boundary_element_count,boundary_quality_min_uv,boundary_quality_mean_uv,boundary_good_ratio_ge_0_60_uv,"
          "transition_band_elements,transition_band_quality_min_uv,transition_band_quality_mean_uv,transition_band_q_ge_0_60_uv,"
          "interior_core_elements,interior_core_quality_min_uv,interior_core_quality_mean_uv,interior_core_q_ge_0_60_uv,"
          "generated_faces_structural,"
          "generation_status_structural"
       << std::endl;
  for (const auto& snapshot : current_patch_adaptation_) {
    file << snapshot.patch_id << "," << snapshot.local_error << ","
         << snapshot.normalized_error << "," << snapshot.quality_min << ","
         << snapshot.quality_mean << "," << snapshot.good_ratio_ge_0_60 << ","
         << snapshot.poor_ratio << ","
         << snapshot.quality_min_3d << "," << snapshot.quality_mean_3d << ","
         << snapshot.good_ratio_ge_0_60_3d << "," << snapshot.poor_ratio_3d << ","
         << snapshot.quality_min_uv << "," << snapshot.quality_mean_uv << ","
         << snapshot.good_ratio_ge_0_60_uv << "," << snapshot.poor_ratio_uv << ","
         << snapshot.raw_factor << "," << snapshot.smoothed_factor << ","
         << snapshot.limited_factor << "," << snapshot.elements_prev << ","
         << snapshot.elements_current << "," << snapshot.elements_delta << ","
         << snapshot.local_error_drop_abs << "," << snapshot.local_error_drop_pct << ","
         << snapshot.local_error_drop_per_element << ","
         << snapshot.efficiency_score << ","
         << snapshot.eligible_for_refinement << ","
         << snapshot.saturated_soft << ","
         << snapshot.saturated_hard << ","
         << snapshot.consecutive_inefficient_steps << ","
         << snapshot.budget_blocked << ","
         << snapshot.refinement_rank << ","
         << snapshot.final_refinement_class << ","
         << snapshot.eligible_h_scale_applied << ","
         << snapshot.micro_refinement_active << ","
         << snapshot.budget_pressure_local << ","
         << snapshot.intra_patch_damped << ","
         << snapshot.local_patch_budget << ","
         << snapshot.patch_budget_exceeded << ","
         << snapshot.patch_generation_mode << ","
         << snapshot.target_patch_elements << ","
         << snapshot.actual_patch_elements << ","
         << snapshot.patch_budget_share << ","
         << snapshot.error_budget_share << ","
         << snapshot.quality_repair_budget_share << ","
         << snapshot.similar_patch_group << ","
         << snapshot.similarity_score << ","
         << snapshot.similarity_score_to_neighbors << ","
         << snapshot.is_consistency_anchor << ","
         << snapshot.consistency_clamped << ","
         << snapshot.budget_before_consistency << ","
         << snapshot.budget_after_consistency << ","
         << snapshot.target_h_before_consistency << ","
         << snapshot.target_h_after_consistency << ","
         << snapshot.group_anchor_patch << ","
         << snapshot.group_budget_ratio << ","
         << snapshot.target_patch_growth << ","
         << snapshot.target_patch_h << ","
         << snapshot.applied_patch_h << ","
         << snapshot.quadtree_cap_applied << ","
         << snapshot.max_quadtree_depth_applied << ","
         << snapshot.transition_floor << ","
         << snapshot.transition_underresolved << ","
         << snapshot.interior_transition_imbalance << ","
         << snapshot.curve_domain_balance << ","
         << snapshot.curve_interior_coupling << ","
         << snapshot.boundary_element_count << ","
         << snapshot.boundary_quality_min << ","
         << snapshot.boundary_quality_mean << ","
         << snapshot.boundary_good_ratio_ge_0_60 << ","
         << snapshot.transition_band_elements << ","
         << snapshot.transition_band_quality_min << ","
         << snapshot.transition_band_quality_mean << ","
         << snapshot.transition_band_q_ge_0_60 << ","
         << snapshot.interior_core_elements << ","
         << snapshot.interior_core_quality_min << ","
         << snapshot.interior_core_quality_mean << ","
         << snapshot.interior_core_q_ge_0_60 << ","
         << snapshot.generated_faces_structural << ","
         << snapshot.generation_status_structural << std::endl;
  }
}

void GeneratorAdaptive::WriteCurveAdaptationLog(int step, bool accepted) const {
  if (!WRITE_ADAPTATION_DEBUG) {
    return;
  }

  std::ostringstream name;
  name << NAME_MODEL << "_passo_" << step << "_curve_adaptation_"
       << (accepted ? "accepted" : "rejected") << ".csv";
  EnsureOutputParentDirectories(name.str());
  std::ofstream file(name.str().c_str());
  if (!file.good()) {
    return;
  }

  file << "curve_id,factor,num_points,policy,adjacent_patch_ids,adjacent_patch_weights,"
          "target_curve_points,actual_curve_points,curve_point_growth_factor,"
          "curve_budget_source_patches,geometric_target_points,min_adjacent_target_h,"
          "max_adjacent_target_elements,sensitive_curve,curve_patch_balance_ratio,"
          "curve_consistency_clamped,target_points_before_consistency,target_points_after_consistency"
       << std::endl;
  for (const auto& snapshot : current_curve_adaptation_) {
    file << snapshot.curve_id << "," << snapshot.factor << ","
         << snapshot.num_points << "," << snapshot.policy << ","
         << snapshot.adjacent_patch_ids << ","
         << snapshot.adjacent_patch_weights << ","
         << snapshot.target_points << "," << snapshot.actual_points << ","
         << snapshot.point_growth_factor << ","
         << snapshot.curve_budget_source_patches << ","
         << snapshot.geometric_target_points << ","
         << snapshot.min_adjacent_target_h << ","
         << snapshot.max_adjacent_target_elements << ","
         << snapshot.sensitive_curve << ","
         << snapshot.curve_patch_balance_ratio << ","
         << snapshot.curve_consistency_clamped << ","
         << snapshot.target_points_before_consistency << ","
         << snapshot.target_points_after_consistency << std::endl;
  }
}

void GeneratorAdaptive::WriteAcceptanceLog(int step, double score,
                                           const MeshQualityStats& stats,
                                           double error_improvement,
                                           double element_growth_factor,
                                           bool accepted) const {
  if (!WRITE_ADAPTATION_DEBUG) {
    return;
  }

  std::ostringstream name;
  name << NAME_MODEL << "_passo_" << step << "_acceptance_"
       << (accepted ? "accepted" : "rejected") << ".log";
  EnsureOutputParentDirectories(name.str());
  std::ofstream file(name.str().c_str());
  if (!file.good()) {
    return;
  }

  file << "step=" << step << std::endl;
  file << "accepted=" << (accepted ? 1 : 0) << std::endl;
  file << "acceptance_mode=" << ACCEPTANCE_MODE << std::endl;
  file << "acceptance_score=" << score << std::endl;
  file << "error_improvement=" << error_improvement << std::endl;
  file << "quality_min=" << stats.quality_min << std::endl;
  file << "quality_mean=" << stats.quality_mean << std::endl;
  file << "poor_ratio=" << stats.poor_ratio << std::endl;
  file << "q_ge_0_60=" << stats.good_ratio_ge_0_60 << std::endl;
  file << "quality_min_3d=" << stats.quality_min_3d << std::endl;
  file << "quality_mean_3d=" << stats.quality_mean_3d << std::endl;
  file << "poor_ratio_3d=" << stats.poor_ratio_3d << std::endl;
  file << "q_ge_0_60_3d=" << stats.good_ratio_ge_0_60_3d << std::endl;
  file << "quality_min_uv=" << stats.quality_min_uv << std::endl;
  file << "quality_mean_uv=" << stats.quality_mean_uv << std::endl;
  file << "poor_ratio_uv=" << stats.poor_ratio_uv << std::endl;
  file << "q_ge_0_60_uv=" << stats.good_ratio_ge_0_60_uv << std::endl;
  file << "angle_quality_mean_3d=" << stats.angle_quality_mean_3d << std::endl;
  file << "edge_ratio_mean_3d=" << stats.edge_ratio_mean_3d << std::endl;
  file << "min_angle_mean_3d=" << stats.min_angle_mean_3d << std::endl;
  file << "min_angle_p05_3d=" << stats.min_angle_p05_3d << std::endl;
  file << "max_angle_mean_3d=" << stats.max_angle_mean_3d << std::endl;
  file << "max_angle_p95_3d=" << stats.max_angle_p95_3d << std::endl;
  file << "error_global_definition=surface_curvature_3d_relative" << std::endl;
  file << "element_growth_factor=" << element_growth_factor << std::endl;
  file << "budget_exceeded=" << (current_budget_exceeded_ ? 1 : 0) << std::endl;
  file << "budget_policy="
       << ((ADAPTIVE_RETRY_COUNT > 0) ? "retry_with_shrink" : "direct_control")
       << std::endl;
  file << "budget_retry_count=" << current_budget_retry_count_ << std::endl;
  file << "budget_limited=" << (current_budget_limited_ ? 1 : 0) << std::endl;
  file << "budget_final_status=" << current_budget_final_status_ << std::endl;
  file << "element_budget_abs=" << current_element_budget_abs_ << std::endl;
  file << "element_budget_growth_factor=" << current_element_budget_growth_factor_limit_
       << std::endl;
  file << "element_budget_soft_limit=" << current_element_budget_soft_limit_
       << std::endl;
  file << "candidate_elements_before_budget_retry="
       << current_candidate_elements_before_budget_retry_ << std::endl;
  file << "top_patch_count_applied=" << current_top_patch_count_ << std::endl;
  file << "step_mode=" << current_step_mode_ << std::endl;
  file << "budget_pressure=" << current_budget_pressure_ << std::endl;
  file << "eligible_patch_count=" << current_eligible_patch_count_ << std::endl;
  file << "eligible_patch_elements_sum=" << current_eligible_patch_elements_sum_
       << std::endl;
  file << "eligible_patch_mean_factor=" << current_eligible_patch_mean_factor_
       << std::endl;
  file << "eligible_patch_mean_h_scale=" << current_eligible_patch_mean_h_scale_
       << std::endl;
  file << "retry_enabled=" << (ADAPTIVE_RETRY_COUNT > 0 ? 1 : 0) << std::endl;
  file << "local_budget_mode=" << PATCH_LOCAL_ELEMENT_BUDGET_MODE << std::endl;
  file << "target_element_growth=" << current_target_element_growth_ << std::endl;
  file << "target_elements_step=" << current_target_elements_step_ << std::endl;
  file << "actual_elements_step="
       << current_candidate_elements_before_budget_retry_ << std::endl;
  file << "overshoot_ratio=" << current_overshoot_ratio_ << std::endl;
  file << "actual_element_growth=" << element_growth_factor << std::endl;
  file << "step_efficiency=" << current_step_efficiency_ << std::endl;
  file << "local_progress_coverage=" << current_local_progress_coverage_
       << std::endl;
  file << "improved_top_patches=" << current_improved_top_patches_ << std::endl;
  file << "top_patch_candidate_count=" << current_top_patch_candidate_count_
       << std::endl;
  file << "generation_control_mode=" << current_generation_control_mode_
       << std::endl;
  file << "step_purpose=" << current_step_purpose_ << std::endl;
  file << "consistency_regularization_active="
       << current_consistency_regularization_active_ << std::endl;
  file << "patch_budget_dispersion=" << current_patch_budget_dispersion_
       << std::endl;
  file << "patch_h_dispersion=" << current_patch_h_dispersion_ << std::endl;
  file << "actual_patch_element_dispersion="
       << current_actual_patch_element_dispersion_ << std::endl;
  file << "curve_balance_dispersion=" << current_curve_balance_dispersion_
       << std::endl;
  file << "curve_domain_penalty=" << current_curve_domain_penalty_ << std::endl;
  file << "transition_coherence_penalty="
       << current_transition_coherence_penalty_ << std::endl;
  file << "underresolved_transition_patch_count="
       << current_underresolved_transition_patch_count_ << std::endl;
  file << "underresolved_transition_patch_count_real="
       << current_underresolved_transition_patch_count_real_ << std::endl;
  file << "underresolved_transition_fraction_real="
       << current_underresolved_transition_fraction_real_ << std::endl;
  file << "transition_imbalance_mean_real="
       << current_transition_imbalance_mean_real_ << std::endl;
  file << "transition_imbalance_max_real="
       << current_transition_imbalance_max_real_ << std::endl;
  file << "consistency_penalty=" << current_patch_consistency_penalty_
       << std::endl;
  file << "budget_driver_mode=" << current_budget_driver_mode_ << std::endl;
  file << "weights[error/quality_min/poor_ratio/growth]="
       << ACCEPTANCE_WEIGHT_ERROR << "/" << ACCEPTANCE_WEIGHT_QUALITY_MIN << "/"
       << ACCEPTANCE_WEIGHT_POOR_RATIO << "/" << ACCEPTANCE_WEIGHT_GROWTH
       << std::endl;
}

void GeneratorAdaptive::UpdatePatchHistory(
    const Geometry* geometry, const MeshAdaptive* mesh,
    const std::vector<double>& patch_factors,
    const std::vector<double>& current_errors, bool accepted_step) {
  if (!accepted_step || !geometry || !mesh) {
    return;
  }

  const unsigned int patch_count = geometry->GetNumberPatches();
  if (patch_history_.size() != patch_count) {
    patch_history_.assign(static_cast<size_t>(patch_count), PatchHistoryState{});
  }

  for (unsigned int patch_index = 0; patch_index < patch_count; ++patch_index) {
    const SubMesh* sub_mesh = mesh->GetSubMeshAdaptiveByPosition(patch_index);
    const unsigned long long current_elements =
        sub_mesh ? static_cast<unsigned long long>(sub_mesh->GetNumberElements())
                 : 0ULL;

    double quality_min = 0.0;
    double quality_mean = 0.0;
    double poor_ratio = 0.0;
    if (sub_mesh && sub_mesh->GetNumberElements() > 0) {
      unsigned long long poor_count = 0ULL;
      for (unsigned int elem_idx = 0; elem_idx < sub_mesh->GetNumberElements();
           ++elem_idx) {
        const TriangleAdaptive* tri =
            static_cast<const TriangleAdaptive*>(sub_mesh->GetElement(elem_idx));
        if (!tri) {
          continue;
        }
        const double quality =
            TriangleQualityFromNodes(tri->GetNoh(1), tri->GetNoh(2), tri->GetNoh(3));
        quality_mean += quality;
        quality_min =
            (elem_idx == 0) ? quality : std::min(quality_min, quality);
        if (quality < 0.30) {
          ++poor_count;
        }
      }
      quality_mean /= static_cast<double>(sub_mesh->GetNumberElements());
      poor_ratio =
          static_cast<double>(poor_count) /
          static_cast<double>(sub_mesh->GetNumberElements());
    }

    PatchHistoryState& history = patch_history_[patch_index];
    const double previous_error = history.prev_patch_error;
    const unsigned long long previous_elements = history.prev_patch_elements;
    const double current_error =
        patch_index < current_errors.size() ? current_errors[patch_index] : 0.0;
    const double error_drop_abs =
        previous_error > 0.0 ? (previous_error - current_error) : 0.0;
    const unsigned long long elements_delta =
        current_elements > previous_elements ? current_elements - previous_elements
                                             : 0ULL;
    const double error_drop_per_element =
        elements_delta > 0ULL
            ? error_drop_abs / static_cast<double>(elements_delta)
            : error_drop_abs;

    history.prev_patch_error = current_error;
    history.prev_patch_elements = current_elements;
    history.prev_patch_quality_min = quality_min;
    history.prev_patch_quality_mean = quality_mean;
    history.prev_patch_poor_ratio = poor_ratio;
    history.prev_patch_factor =
        patch_index < patch_factors.size() ? patch_factors[patch_index] : 1.0;
    history.prev_patch_error_drop_abs = error_drop_abs;
    history.prev_patch_error_drop_per_element = error_drop_per_element;
    history.prev_patch_efficiency =
        std::max(0.0, error_drop_per_element);
    history.prev_patch_generated_faces = current_elements;
    history.prev_patch_template_score_mean = 0.0;
    history.prev_patch_budget_share =
        patch_index < current_patch_adaptation_.size()
            ? current_patch_adaptation_[patch_index].patch_budget_share
            : 0.0;
    history.prev_patch_target_elements =
        patch_index < current_patch_adaptation_.size()
            ? current_patch_adaptation_[patch_index].target_patch_elements
            : 0.0;
    history.prev_patch_improved = error_drop_abs > 0.0 ? 1 : 0;
    history.prev_patch_decision_class =
        patch_index < current_patch_adaptation_.size()
            ? current_patch_adaptation_[patch_index].final_refinement_class
            : "freeze";
    const bool inefficient_now =
        history.prev_patch_generated_faces >=
            static_cast<unsigned long long>(
                std::max(1.0, QUADTREE_SATURATION_ELEMENTS_THRESHOLD)) &&
        history.prev_patch_efficiency <=
            QUADTREE_SATURATION_EFFICIENCY_THRESHOLD;
    history.consecutive_inefficient_steps =
        inefficient_now ? (history.consecutive_inefficient_steps + 1U) : 0U;
    history.saturated_last_step =
        inefficient_now &&
        history.consecutive_inefficient_steps >=
            static_cast<unsigned int>(QUADTREE_HARD_SATURATION_STEPS);
  }
}

#if USE_OPENMP
SubMesh* GeneratorAdaptive::GeneratorInitialMeshOmp(
    PatchCoons* patch, Performer::IdManager* id_manager) {
  CurveAdaptive* c1 = patch->GetCurve(0);
  CurveAdaptive* c2 = patch->GetCurve(1);
  CurveAdaptive* c3 = patch->GetCurve(2);
  CurveAdaptive* c4 = patch->GetCurve(3);

  // 1. verifica quais curvas ainda não foram discretizadas
  if (c1->GetNumBerPoints())
    c1 = nullptr;  // c1 já foi trabalhada no patch vizinho
  if (c2->GetNumBerPoints())
    c2 = nullptr;  // c2 já foi trabalhada no patch vizinho
  if (c3->GetNumBerPoints())
    c3 = nullptr;  // c3 já foi trabalhada no patch vizinho
  if (c4->GetNumBerPoints())
    c4 = nullptr;  // c4 já foi trabalhada no patch vizinho

  SubMesh* sub_mesh = new SubMesh;

  //========================= Malha Grosseira
  //====================================
  // 2. divide cada patch em 9 regiões e gera os nós dos extremos de cada região

  for (double v = 0.0; v <= 1.0; v += 1) {
    for (double u = 0.0; u <= 1.0; u += 1) {
      PointAdaptive* point = new NodeAdaptive(patch->Parameterize(u, v));
          point->SetId(CheckedLegacyId(id_manager->next(0)));
      bool inserted = false;

      if (v == 0 && c1)  // p está na curva 1 (c1 = nullptr)
        c1->InsertPoint(point), inserted = true;
      else if (v == 1 && c3)  // p está na curva 3
        c3->InsertPoint(point), inserted = true;

      if (u == 0 && c4)  // p está na curva 4
        c4->InsertPoint(point), inserted = true;
      else if (u == 1 && c2)  // p está na curva 2
        c2->InsertPoint(point), inserted = true;

      sub_mesh->SetNoh(static_cast<NodeAdaptive*>(point));
      if (!inserted) delete point;  // only submesh needs the data copy
    }
  }

  // Coarse initial mesh: 4 triangles per patch with a central node.
  PointAdaptive* center_point = new NodeAdaptive(patch->Parameterize(0.5, 0.5));
    center_point->SetId(CheckedLegacyId(id_manager->next(0)));
  sub_mesh->SetNoh(static_cast<NodeAdaptive*>(center_point));

  ElementAdaptive* e1 = new TriangleAdaptive(
      sub_mesh->GetNoh(0), sub_mesh->GetNoh(1), sub_mesh->GetNoh(4));
  (static_cast<TriangleAdaptive*>(e1))->SetParametersN1(make_tuple(0, 0));
  (static_cast<TriangleAdaptive*>(e1))->SetParametersN2(make_tuple(1, 0));
  (static_cast<TriangleAdaptive*>(e1))->SetParametersN3(make_tuple(0.5, 0.5));
    e1->SetId(CheckedLegacyId(id_manager->next(1)));
  sub_mesh->SetElement(e1);

  ElementAdaptive* e2 = new TriangleAdaptive(
      sub_mesh->GetNoh(1), sub_mesh->GetNoh(3), sub_mesh->GetNoh(4));
  (static_cast<TriangleAdaptive*>(e2))->SetParametersN1(make_tuple(1, 0));
  (static_cast<TriangleAdaptive*>(e2))->SetParametersN2(make_tuple(1, 1));
  (static_cast<TriangleAdaptive*>(e2))->SetParametersN3(make_tuple(0.5, 0.5));
    e2->SetId(CheckedLegacyId(id_manager->next(1)));
  sub_mesh->SetElement(e2);

  ElementAdaptive* e3 = new TriangleAdaptive(
      sub_mesh->GetNoh(3), sub_mesh->GetNoh(2), sub_mesh->GetNoh(4));
  (static_cast<TriangleAdaptive*>(e3))->SetParametersN1(make_tuple(1, 1));
  (static_cast<TriangleAdaptive*>(e3))->SetParametersN2(make_tuple(0, 1));
  (static_cast<TriangleAdaptive*>(e3))->SetParametersN3(make_tuple(0.5, 0.5));
    e3->SetId(CheckedLegacyId(id_manager->next(1)));
  sub_mesh->SetElement(e3);

  ElementAdaptive* e4 = new TriangleAdaptive(
      sub_mesh->GetNoh(2), sub_mesh->GetNoh(0), sub_mesh->GetNoh(4));
  (static_cast<TriangleAdaptive*>(e4))->SetParametersN1(make_tuple(0, 1));
  (static_cast<TriangleAdaptive*>(e4))->SetParametersN2(make_tuple(0, 0));
  (static_cast<TriangleAdaptive*>(e4))->SetParametersN3(make_tuple(0.5, 0.5));
    e4->SetId(CheckedLegacyId(id_manager->next(1)));
  sub_mesh->SetElement(e4);
  //==============================================================================*/

  // 5. define a submalha do patch
  patch->SetSubMesh(sub_mesh);
  sub_mesh->SetPatch(patch);

  return sub_mesh;
}

double GeneratorAdaptive::CalculateErrorGlobalOmp(MeshAdaptive* mesh,
                                                  Timer* timer,
                                                  [[maybe_unused]] int rank,
                                                  int size_thread) {
  unsigned int Ns = 0;  // número de submalhas
  double Nj = 0.0;      // erro global da malha

  Ns = mesh->GetNumberSubMeshesAdaptive();
  std::vector<double> sub_errors(Ns, 0.0);
  unsigned long long gauss_nodes_total = 0ULL;
  unsigned long long mean_nodes_total = 0ULL;
  // Calcula o erro global de cada submalha (OMP)
#pragma omp parallel num_threads(size_thread) firstprivate(Ns) reduction(+ : Nj, gauss_nodes_total, mean_nodes_total)
  {
#if USE_MPI
    timer->InitTimerParallel(RANK_MPI, omp_get_thread_num(),
                             7);  // calculo do erro global
#else
    timer->InitTimerParallel(0, omp_get_thread_num(),
                             7);  // calculo do erro global
#endif  // USE_MPI
#pragma omp for
    for (unsigned int i = 0; i < Ns; ++i) {
      SubMesh* sub = mesh->GetSubMeshAdaptiveByPosition(i);
      unsigned int Nv = sub->GetNumberNos();
      double curvPower = 0.0;
      double Njs = 0.0;

      // Calcula o erro relativo para cada nó e soma a Nj
      // #pragma omp parallel for num_threads(nThreads) firstprivate(Ns)
      // reduction(+ :Nj)
      for (unsigned int j = 0; j < Nv; ++j) {
#if USE_MPI
        timer->EndTimerParallel(RANK_MPI, omp_get_thread_num(),
                                7);  // calculo do erro global
        timer->InitTimerParallel(RANK_MPI, omp_get_thread_num(),
                                 6);  // MediaGauss
#else
        timer->EndTimerParallel(0, omp_get_thread_num(),
                                7);  // calculo do erro global
        timer->InitTimerParallel(0, omp_get_thread_num(), 6);  // MediaGauss
#endif  // USE_MPI

        PointAdaptive* n = sub->GetNoh(j);
        Patch* p = sub->GetPatch();
        CurvatureAnalytical ka(*(static_cast<NodeAdaptive*>(n)),
                               *(static_cast<PatchCoons*>(p)));
        CurvatureDiscrete kd(*(static_cast<NodeAdaptive*>(n)));
        double Ga = ka.CalculateGaussCurvature();
        double Gd = kd.CalculateGaussCurvature();
        double Ha = ka.CalculateMeanCurvature();
        double Hd = kd.CalculateMeanCurvature();

        // tratamento para erro -nan
        if (std::isnan(Gd)) {
          cout << "-nan Gd" << endl;
          Ga = 0.0;
          Gd = 0.0;
        }

        // atualiza as curvaturas do nó ( para que não sejam recalculadas na
        // adaptação das curvas e do domínio )
        ((NodeAdaptive*)n)->SetGa(Ga);
        ((NodeAdaptive*)n)->SetGd(Gd);
        ((NodeAdaptive*)n)->SetHa(Ha);
        ((NodeAdaptive*)n)->SetHd(Hd);
#if USE_MPI
        timer->EndTimerParallel(RANK_MPI, omp_get_thread_num(),
                                6);  // MediaGauss
        timer->InitTimerParallel(RANK_MPI, omp_get_thread_num(),
                                 7);  // calculo do erro global
#else
        timer->EndTimerParallel(0, omp_get_thread_num(), 6);   // MediaGauss
        timer->InitTimerParallel(0, omp_get_thread_num(),
                                 7);  // calculo do erro global
#endif  // USE_MPI

        double power = 0.0;
        double diff = 0.0;
        if (fabs(Ga) >= TOLERANCE) {
          ++gauss_nodes_total;
          diff = Gd - Ga;
          power = pow(diff, 2);
          Njs += power;
          curvPower += pow(Ga, 2);
        } else if (fabs(Ha) >= TOLERANCE) {
          ++mean_nodes_total;
          diff = Hd - Ha;
          power = pow(diff, 2);
          Njs += power;
          curvPower += pow(Ha, 2);
        }
      }

      if (Njs > 0.0 && curvPower > 0.0 && Nv > 0U) {
        Njs = (double)sqrt(Njs / curvPower) / Nv;
      }

      sub_errors[i] = Njs;
      if (!std::isnan(Njs)) {
        Nj += Njs;
      } else
        cout << "Njs -nan" << endl;

    }  // Parallel for

#if USE_MPI
    timer->EndTimerParallel(RANK_MPI, omp_get_thread_num(),
                            7);  // calculo do erro global
#else
    timer->EndTimerParallel(0, omp_get_thread_num(),
                            7);          // calculo do erro global
#endif  // USE_MPI
  }

  if (Ns > 0) {
    Nj /= Ns;
  }
  last_submesh_error_ = std::move(sub_errors);

  return Nj;
}

int GeneratorAdaptive::GeneratorOmp(Model& model, Timer* timer, int id_range,
                                    [[maybe_unused]] int size_rank,
                                    int size_thread)

{
  this->id_manager_ = nullptr;
  this->id_off_set_ = 0;
  this->id_range_ = id_range;
  this->communicator_ = new ApMeshCommunicator(true);

#if USE_MPI
  [[maybe_unused]] Int nProcesses = 1;
  [[maybe_unused]] Int rank = 0;
  nProcesses = this->communicator_->numProcesses();
  rank = this->communicator_->rank();
#endif  // #if USE_MPI

  Geometry* geo = model.GetGeometry();
  int sizePatch = geo->GetNumberPatches();

  MeshAdaptive* malha = new MeshAdaptive;
  malha->ResizeSubMeshAdaptiveByPosition(sizePatch);

  this->step_ = 0;

  // sizeThread = static_cast<Parallel::TMCommunicator
  // *>(this->comm)->getMaxThreads();

  ptr_aux.resize(size_thread, nullptr);

  if (this->id_managers_.empty()) {
    this->id_managers_.resize(size_thread, nullptr);
  }

#pragma omp parallel num_threads(size_thread) shared(malha, geo, sizePatch)
  {
    Int id = communicator_->threadId();

    if (!this->id_managers_[id]) {
      this->id_managers_[id] = this->MakeIdManagerOmp(communicator_, id);
    }

    timer->InitTimerParallel(0, id, 2);  // Malha inicial

    // 1. Gera a malha inicial
#pragma omp for
    for (int i = 0; i < sizePatch; ++i) {
      PatchCoons* patch = static_cast<PatchCoons*>(geo->GetPatch(i));
      SubMesh* sub = this->GeneratorInitialMeshOmp(
          static_cast<PatchCoons*>(patch), this->id_managers_[id]);
      malha->InsertSubMeshAdaptiveByPosition(sub, i);
    }

    timer->EndTimerParallel(0, id, 2);  // Malha inicial
  }

  // 2. Insere a malha inicial no modelo ( que guarda todas as malhas geradas )
  model.InsertMeshAdaptive(malha);

  // 3. Calcula o erro global para a malha inicial
  this->error_local_process_ =
      this->CalculateErrorGlobalOmp(malha, timer, 0, size_thread);

#if USE_PRINT_ERRO
  cout << "ERRO  " << this->step_ << " = " << this->error_local_process_
       << endl;
#endif  // #if USE_PRINT_ERRO

#if USE_SAVE_MESH
  WriteMesh(malha, step_);
#endif  // #USE_SAVE_MESH

  this->error_local_process_ = 1.0;

  // 4. enquanto o erro global de uma malha gerada não for menor que o desejado
  while (this->error_local_process_ > EPSYLON) {
    if (step_ >= 2) {
      break;
    }

    this->step_++;

    // 4.1. Aloca uma nova malha
    malha = new MeshAdaptive;
    malha->ResizeSubMeshAdaptiveByPosition(sizePatch);

    list<PointAdaptive*> novosPontos[geo->GetNumberCurves()];

    // map<Ponto *, Ponto *> mapaPontos;

    int sizeCurvas = geo->GetNumberCurves();

    //
    //        sizeThread = 1;
    // #pragma omp parallel num_threads(sizeThread) shared(geo, sizeCurvas,
    // sizePatch, malha, novosPontos)
    //        {
    //            Int id = comm->threadId();
    //            //           this->idManagers[id] =
    //            this->makeIdManagerOmp(comm, id);

    //            // 4.2. Adapta as curvas pela curvatura da curva
    // #pragma omp for firstprivate(ptr_aux)
    //            for (int i = 0; i < sizeCurvas; ++i )
    //            {
    //                novosPontos[i] =
    //                AdaptadorPorCurvatura::adaptaCurvaByCurvaOmp(
    //                geo->getCurva( i ), this->idManagers[id], 1);
    //                geo->getCurva(i)->setPontos(novosPontos[i]);
    //                novosPontos[i] =
    //                AdaptadorPorCurvatura::adaptaCurvaBySuperficieOmp(
    //                geo->getCurva( i ), this->idManagers[id], 1);
    //                geo->getCurva(i)->setPontos(novosPontos[i]);
    //                ((CurvaParametrica*)geo->getCurva(i))->ordenaLista ( );
    //            }

    //        }

    // 4.2. Adapta as curvas pela curvatura da curva / 4.3. Atualiza a
    // discretização das curvas
    map<PointAdaptive*, PointAdaptive*> mapaPontos;
    timer->InitTimerParallel(0, 0, 3);  // adpt. das curvas

    for (int i = 0; i < sizeCurvas; ++i) {
      novosPontos[i] = adapter_.AdaptCurveByCurve(geo->GetCurve(i), mapaPontos,
                                                  this->id_managers_[0], 1);
      geo->GetCurve(i)->SetPoints(novosPontos[i]);
      novosPontos[i] = adapter_.AdaptCurveBySurface(
          geo->GetCurve(i), mapaPontos, this->id_managers_[0], 1);
      geo->GetCurve(i)->SetPoints(novosPontos[i]);
      // ((CurvaParametrica*)geo->getCurva(i))->ordenaLista ( );
    }

    timer->EndTimerParallel(0, 0, 3);  // adpt. das cruvas

#pragma omp parallel num_threads(size_thread) shared(geo, sizePatch, malha)
    {
      Int id = communicator_->threadId();
      //((Performer::RangedIdManager *)this->idManagers[id])->setMin(1,0) ;

      timer->InitTimerParallel(0, id, 4);  // adpt. do domínio

      // 4.3. Adapta as patches
#pragma omp for
      for (int i = 0; i < sizePatch; ++i) {
        PatchCoons* p = static_cast<PatchCoons*>(geo->GetPatch(i));
        SubMesh* sub = adapter_.AdaptDomainOmp(p, this->id_managers_[id], 1);
        sub->SetPatch(p);
        malha->InsertSubMeshAdaptiveByPosition(sub, i);
        geo->GetPatch(i)->SetSubMesh(malha->GetSubMeshAdaptiveByPosition(i));
      }

      timer->EndTimerParallel(0, id, 4);  // adpt. do domínio
    }

    //        // 4.5. Atualiza os patches
    //        for ( unsigned int i = 0; i < geo->getNumDePatches ( ); ++i )
    //        {
    //            geo->getPatch( i )->setMalha(malha->getSubMalha( i ));
    //        }

    // 4.6. Insere a malha gerada no modelo ( que guarda todas as malhas geradas
    // )
    model.InsertMeshAdaptive(malha);

    // 4.7. Escreve um artigo "neutral file" da malha gerada

#if USE_SAVE_MESH
    WriteMesh(malha, step_);
#endif  // #USE_SAVE_MESH

    // 4.7. Calcula o erro global para a malha
    this->error_local_process_ =
        this->CalculateErrorGlobalOmp(malha, timer, 0, size_thread);

#if USE_PRINT_ERRO
    cout << "ERRO  " << this->step_ << " = " << this->error_local_process_
         << endl;
#endif  // #if USE_PRINT_COMENT
  }

  return 0;
}

void GeneratorAdaptive::AdaptDomainOmp(Geometry* geo, MeshAdaptive* malha,
                                       Timer* timer, int sizeThread,
                                       int sizePatch) {
  const std::vector<double> patch_factors = ComputePatchAdaptationFactors(geo);
  const bool few_patch_low_error =
      current_policy_.stable_mode && geo != nullptr &&
      geo->GetNumberPatches() <= 4U && !error_step_.empty() &&
      error_step_.back() < 0.12 && step_ <= 3;
  last_domain_patch_factors_.assign(patch_factors.begin(), patch_factors.end());

#pragma omp parallel num_threads(sizeThread) shared(geo, sizePatch, malha)
  {
    Int id = communicator_->threadId();

#if USE_MPI
    timer->InitTimerParallel(RANK_MPI, id, 4);  // Adaptação do domínio
#else
    timer->InitTimerParallel(0, id, 4);  // Adaptação do domínio
#endif  // USE_MPI

    // 4.3. Adapta as patches
#pragma omp for
    for (int i = 0; i < sizePatch; ++i) {
      PatchCoons* p = static_cast<PatchCoons*>(geo->GetPatch(i));
      double domain_factor = patch_factors[i];
      std::string patch_generation_mode = "normal";
      bool soft_saturated = false;
      bool hard_saturated = false;
      if (QUADTREE_SATURATION_MODE == 1 && step_ > 1 &&
          static_cast<size_t>(i) < patch_history_.size()) {
        const PatchHistoryState& history = patch_history_[static_cast<size_t>(i)];
        if (history.prev_patch_generated_faces >
                static_cast<unsigned long long>(
                    std::max(1.0, QUADTREE_SATURATION_ELEMENTS_THRESHOLD)) &&
            history.prev_patch_efficiency <=
                QUADTREE_SATURATION_EFFICIENCY_THRESHOLD) {
          soft_saturated = true;
          domain_factor = ClampValue(domain_factor * QUADTREE_SATURATION_H_SCALE,
                                     PATCH_FACTOR_MIN, PATCH_FACTOR_MAX);
        }
        if (QUADTREE_HARD_SATURATION_MODE == 1 &&
            history.consecutive_inefficient_steps >=
                static_cast<unsigned int>(QUADTREE_HARD_SATURATION_STEPS)) {
          hard_saturated = true;
          domain_factor =
              std::max(domain_factor, QUADTREE_HARD_SATURATION_FACTOR_MIN);
        }
      }
      if (STEP_ELEMENT_BUDGET_MODE == 1 && step_ > 1 &&
          current_budget_retry_count_ > 0U) {
        domain_factor = ClampValue(
            domain_factor *
                std::pow(STEP_ELEMENT_BUDGET_RETRY_H_SCALE,
                         static_cast<int>(current_budget_retry_count_)),
            PATCH_FACTOR_MIN, PATCH_FACTOR_MAX);
      }
      const int refinement_rank =
          (static_cast<size_t>(i) < current_patch_adaptation_.size())
              ? current_patch_adaptation_[static_cast<size_t>(i)].refinement_rank
              : -1;
      const bool step2_quality_critical =
          (step_ == 2 && static_cast<size_t>(i) < current_patch_adaptation_.size() &&
           (current_patch_adaptation_[static_cast<size_t>(i)].quality_min < 0.25 ||
            current_patch_adaptation_[static_cast<size_t>(i)].poor_ratio > 0.18));
      if (step_ > 1 && PATCH_GENERATION_CONTROL_MODE == 1) {
        patch_generation_mode =
            (STEP2_MICRO_REFINEMENT_MODE == 1) ? "micro_refinement" : "normal";
        if (PATCH_GENERATION_TOP_RANK_ONLY_MODE == 1 &&
            (static_cast<size_t>(i) >= current_patch_eligible_mask_.size() ||
             current_patch_eligible_mask_[static_cast<size_t>(i)] == 0)) {
          patch_generation_mode = "budget_constrained";
          domain_factor = std::max(domain_factor, STEP2_NONELIGIBLE_FACTOR_MIN);
        }
      }
      if (step_ > 1 && !few_patch_low_error &&
          static_cast<size_t>(i) < current_patch_eligible_mask_.size() &&
          current_patch_eligible_mask_[static_cast<size_t>(i)] == 1) {
        double eligible_h_scale = 1.0;
        if (STEP2_MICRO_REFINEMENT_MODE == 1) {
          eligible_h_scale *= STEP2_ELIGIBLE_H_SCALE;
        }
        if (static_cast<size_t>(i) < patch_history_.size() &&
            patch_history_[static_cast<size_t>(i)].prev_patch_elements >=
                static_cast<unsigned long long>(
                    std::max(1.0, STEP2_ELIGIBLE_ELEMENTS_THRESHOLD))) {
          eligible_h_scale *= 1.05;
        }
        if (STEP_ELEMENT_BUDGET_PRESSURE_MODE == 1 &&
            current_budget_pressure_ > STEP_ELEMENT_BUDGET_PRESSURE_START) {
          eligible_h_scale *= STEP_ELEMENT_BUDGET_PRESSURE_H_SCALE;
        }
        if (PATCH_GENERATION_CONTROL_MODE == 1) {
          eligible_h_scale *= PATCH_GENERATION_MICRO_H_SCALE;
        }
        domain_factor = ClampValue(domain_factor * eligible_h_scale, PATCH_FACTOR_MIN,
                                   PATCH_FACTOR_MAX);
        if (static_cast<size_t>(i) < current_patch_adaptation_.size()) {
          current_patch_adaptation_[static_cast<size_t>(i)].eligible_h_scale_applied =
              eligible_h_scale;
        }
      }
      if (step_ == 2 && refinement_rank > 0 &&
          refinement_rank <= static_cast<int>(current_top_patch_count_ + 1U)) {
        if (!few_patch_low_error) {
          domain_factor =
              std::min(domain_factor, step2_quality_critical ? 0.96 : 1.0);
          if (step2_quality_critical) {
            patch_generation_mode = "quality_stabilized";
          }
        }
      }
      if (static_cast<size_t>(i) < current_patch_saturated_soft_mask_.size()) {
        current_patch_saturated_soft_mask_[static_cast<size_t>(i)] =
            soft_saturated ? 1 : 0;
      }
      if (static_cast<size_t>(i) < current_patch_saturated_hard_mask_.size()) {
        current_patch_saturated_hard_mask_[static_cast<size_t>(i)] =
            hard_saturated ? 1 : 0;
      }
      if (step_ > 1 && static_cast<size_t>(i) < patch_history_.size() &&
          static_cast<size_t>(i) < current_patch_adaptation_.size()) {
        const PatchHistoryState& history = patch_history_[static_cast<size_t>(i)];
        const double efficiency_norm =
            Clamp01(history.prev_patch_efficiency /
                    std::max(PATCH_EFFICIENCY_TARGET, 1.0e-9));
        const double rank_weight =
            (refinement_rank > 0)
                ? std::max(
                      0.35,
                      1.0 + PATCH_LOCAL_ELEMENT_BUDGET_SCALE_BY_RANK *
                                (1.0 - (static_cast<double>(refinement_rank - 1) /
                                        std::max(1.0, static_cast<double>(
                                                          current_top_patch_count_ > 1
                                                              ? current_top_patch_count_ - 1
                                                              : 1U)))))
                : 0.5;
        const double efficiency_weight =
            1.0 + PATCH_LOCAL_ELEMENT_BUDGET_SCALE_BY_EFFICIENCY * efficiency_norm;
        const double local_patch_budget =
            std::min(PATCH_LOCAL_BUDGET_HARD_CAP,
                     PATCH_LOCAL_ELEMENT_BUDGET_BASE * rank_weight *
                         efficiency_weight);
        current_patch_adaptation_[static_cast<size_t>(i)].local_patch_budget =
            local_patch_budget;
        if (!few_patch_low_error) {
          current_patch_adaptation_[static_cast<size_t>(i)].target_patch_elements =
              local_patch_budget;
        } else if (static_cast<size_t>(i) < current_patch_target_elements_.size()) {
          current_patch_adaptation_[static_cast<size_t>(i)].target_patch_elements =
              current_patch_target_elements_[static_cast<size_t>(i)];
        }
        current_patch_adaptation_[static_cast<size_t>(i)].actual_patch_elements =
            static_cast<double>(history.prev_patch_generated_faces);
        const bool patch_budget_exceeded =
            PATCH_LOCAL_ELEMENT_BUDGET_MODE == 1 &&
            history.prev_patch_generated_faces >
                static_cast<unsigned long long>(std::max(1.0, local_patch_budget));
        current_patch_adaptation_[static_cast<size_t>(i)].patch_budget_exceeded =
            patch_budget_exceeded ? 1 : 0;
        if (patch_budget_exceeded) {
          patch_generation_mode = "budget_constrained";
          domain_factor = ClampValue(domain_factor * PATCH_LOCAL_BUDGET_H_SCALE,
                                     PATCH_FACTOR_MIN, PATCH_FACTOR_MAX);
        }
        if (!few_patch_low_error && PATCH_GENERATION_CONTROL_MODE == 1 &&
            current_target_element_growth_ > 0.0 &&
            history.prev_patch_elements > 0ULL &&
            current_target_element_growth_ <= 1.0) {
          domain_factor =
              ClampValue(domain_factor * STEP_TARGET_GROWTH_H_SCALE,
                         PATCH_FACTOR_MIN, PATCH_FACTOR_MAX);
          // Em adaptive_stable, evitar trocar o modo para 'budget_constrained'
          // após rejeições consecutivas (isso tem correlação com malhas
          // complementares erradas em patches curvos).
          if (!(current_policy_.stable_mode && stable_consecutive_reject_count_ > 0U)) {
            patch_generation_mode = "budget_constrained";
          }
        }
        if (!few_patch_low_error && STEP_ELEMENT_BUDGET_PRESSURE_MODE == 1 &&
            current_budget_pressure_ >= PATCH_GENERATION_BUDGET_PRESSURE_START) {
          domain_factor =
              ClampValue(domain_factor * PATCH_GENERATION_CONSTRAINED_H_SCALE,
                         PATCH_FACTOR_MIN, PATCH_FACTOR_MAX);
          patch_generation_mode = "budget_constrained";
        }
        current_patch_adaptation_[static_cast<size_t>(i)].patch_generation_mode =
            patch_generation_mode;
      }
      last_domain_patch_factors_[static_cast<size_t>(i)] = domain_factor;
      const int quadtree_depth_cap =
          (static_cast<size_t>(i) < current_patch_quadtree_depth_cap_.size())
              ? current_patch_quadtree_depth_cap_[static_cast<size_t>(i)]
              : -1;
      double patch_curve_support_ratio = 1.0;
      double target_patch_h =
          (static_cast<size_t>(i) < current_patch_target_h_.size())
              ? current_patch_target_h_[static_cast<size_t>(i)]
              : -1.0;
      // adaptive_stable guard (OpenMP): não permitir coarsen visível após
      // uma rejeição anterior (ver versão sequencial do mesmo bloco).
      if (current_policy_.stable_mode && stable_consecutive_reject_count_ > 0U &&
          this->step_ >= 4 && static_cast<size_t>(i) < last_candidate_patch_h_.size()) {
        const double prev_h = last_candidate_patch_h_[static_cast<size_t>(i)];
        const bool top_priority_curve_patch =
            (static_cast<size_t>(i) < current_patch_adaptation_.size() &&
             current_patch_adaptation_[static_cast<size_t>(i)].refinement_rank > 0 &&
             current_patch_adaptation_[static_cast<size_t>(i)].refinement_rank <= 2);
        if (prev_h > 0.0 && top_priority_curve_patch) {
          target_patch_h = std::min(target_patch_h, prev_h * 0.96);
        } else if (prev_h > 0.0 && target_patch_h > prev_h) {
          target_patch_h = prev_h;
        }
      }
      if (current_policy_.stable_mode && step_ >= 2 && target_patch_h <= 0.0) {
        SubMesh* previous_sub_mesh = p->GetSubMesh();
        if (previous_sub_mesh != nullptr &&
            previous_sub_mesh->GetNumberElements() > 0U) {
          double patch_area_est = 0.0;
          for (unsigned int e = 0; e < previous_sub_mesh->GetNumberElements(); ++e) {
            ElementAdaptive* element = previous_sub_mesh->GetElement(e);
            if (element != nullptr) {
              patch_area_est += std::fabs(element->GetArea());
            }
          }
          if (patch_area_est > 0.0) {
            const double h_ref =
                std::sqrt(patch_area_est /
                          static_cast<double>(previous_sub_mesh->GetNumberElements()));
            const double stable_scale = 0.78;
            target_patch_h = std::max(1.0e-4, h_ref * stable_scale);
          }
        }
      }
      if (current_policy_.stable_mode && step_ == 2 && !few_patch_low_error) {
        domain_factor =
            ClampValue(domain_factor * 1.10, PATCH_FACTOR_MIN, PATCH_FACTOR_MAX);
        if (target_patch_h > 0.0) {
          target_patch_h *= 1.18;
        }
        patch_generation_mode = "step2_global_stabilization";
      }
      if (current_policy_.stable_mode && step_ >= 2 && target_patch_h > 0.0 &&
          !few_patch_low_error) {
        patch_curve_support_ratio = 1.0;
        for (unsigned int c = 0; c < p->GetNumBerCurves(); ++c) {
          CurveAdaptive* curve = p->GetCurve(c);
          if (curve == nullptr) {
            continue;
          }
          unsigned int curve_idx = std::numeric_limits<unsigned int>::max();
          for (unsigned int g = 0; g < geo->GetNumberCurves(); ++g) {
            if (geo->GetCurve(g) == curve) {
              curve_idx = g;
              break;
            }
          }
          if (curve_idx >= current_curve_adaptation_.size()) {
            continue;
          }
          const CurveAdaptationSnapshot& snapshot =
              current_curve_adaptation_[curve_idx];
          if (snapshot.geometric_target_points <= 0) {
            continue;
          }
          const double ratio =
              static_cast<double>(snapshot.actual_points) /
              static_cast<double>(std::max(1, snapshot.geometric_target_points));
          patch_curve_support_ratio =
              std::min(patch_curve_support_ratio, ratio);
        }
        if (patch_curve_support_ratio < 0.85) {
          const double h_boost =
              ClampValue(1.0 + (0.85 - patch_curve_support_ratio) * 0.80, 1.0,
                         1.35);
          domain_factor =
              ClampValue(domain_factor * h_boost, PATCH_FACTOR_MIN, PATCH_FACTOR_MAX);
          target_patch_h *= h_boost;
          patch_generation_mode = "curve_transition_balanced";
        }
      }
      constexpr double kStableRejectCoarsenCurveRatioFloorOmp = 0.90;
      double curve_domain_balance_omp = 1.0;
      int transition_underresolved_omp = 0;
      if (static_cast<size_t>(i) < current_patch_adaptation_.size()) {
        curve_domain_balance_omp =
            current_patch_adaptation_[static_cast<size_t>(i)].curve_domain_balance;
        transition_underresolved_omp =
            current_patch_adaptation_[static_cast<size_t>(i)].transition_underresolved;
      }
      const bool stable_curve_allows_reject_coarsen_omp =
          patch_curve_support_ratio >= kStableRejectCoarsenCurveRatioFloorOmp &&
          curve_domain_balance_omp >= 0.90 && transition_underresolved_omp == 0;
      int gen_depth_cap = quadtree_depth_cap;
      if (current_policy_.stable_mode && stable_consecutive_reject_count_ > 0U &&
          this->step_ >= 4 && quadtree_depth_cap >= 0 &&
          stable_curve_allows_reject_coarsen_omp) {
        const int reduce =
            std::min(quadtree_depth_cap, static_cast<int>(this->step_ - 3));
        gen_depth_cap = std::max(0, quadtree_depth_cap - reduce);
      }
      if (current_policy_.stable_mode && stable_consecutive_reject_count_ > 0U &&
          this->step_ >= 4 && static_cast<size_t>(i) < current_patch_adaptation_.size() &&
          current_patch_adaptation_[static_cast<size_t>(i)].refinement_rank > 0 &&
          current_patch_adaptation_[static_cast<size_t>(i)].refinement_rank <= 2) {
        gen_depth_cap = (gen_depth_cap >= 0) ? std::max(gen_depth_cap, 4) : 4;
      }
      double gen_factor = domain_factor;
      double gen_h = target_patch_h;
      const bool stable_mesh_damp_omp =
          current_policy_.stable_mode && stable_curve_allows_reject_coarsen_omp &&
          (this->step_ >= 5 ||
           (this->step_ >= 3 && stable_consecutive_reject_count_ > 0U));
      if (stable_mesh_damp_omp) {
        constexpr double kStableLateFactorMul = 0.92;
        constexpr double kStableLateFactorFloor = 0.72;
        double mul = kStableLateFactorMul;
        double floor_v = kStableLateFactorFloor;
        if (stable_consecutive_reject_count_ > 0U && this->step_ >= 3) {
          const double ex = std::pow(
              0.972, static_cast<double>(
                         std::min(9U, stable_consecutive_reject_count_)));
          mul *= ex;
          floor_v = std::max(0.66, floor_v * ex);
        }
        if (this->step_ > 3 && stable_consecutive_reject_count_ > 0U) {
          const double st = std::pow(0.988, static_cast<double>(this->step_ - 3));
          mul *= st;
          floor_v = std::max(0.64, floor_v * st);
        }
        gen_factor = std::max(floor_v, domain_factor * mul);
        gen_factor = ClampValue(gen_factor, floor_v, PATCH_FACTOR_MAX);
        if (gen_h > 0.0) {
          gen_h *= (stable_consecutive_reject_count_ > 0U && this->step_ >= 3)
                       ? (0.88 * std::pow(0.985, static_cast<double>(std::min(
                                                      9U, stable_consecutive_reject_count_))))
                       : 0.88;
        }
      }
      if (static_cast<size_t>(i) < current_patch_adaptation_.size()) {
        current_patch_adaptation_[static_cast<size_t>(i)].applied_patch_h = gen_h;
      }
      SubMesh* sub = adapter_.AdaptDomainOmp(
          p, this->id_managers_[id], gen_factor, gen_h, gen_depth_cap,
          ADAPTIVE_CURRENT_STEP, i, ADAPTIVE_CURRENT_RETRY);
      if (static_cast<size_t>(i) < current_patch_adaptation_.size()) {
        current_patch_adaptation_[static_cast<size_t>(i)].actual_patch_elements =
            sub ? static_cast<double>(sub->GetNumberElements()) : 0.0;
      }
      if (static_cast<size_t>(i) < current_patch_eligible_mask_.size() &&
          current_patch_eligible_mask_[static_cast<size_t>(i)] == 1 && sub) {
#pragma omp atomic
        current_eligible_patch_count_ += 1U;
#pragma omp atomic
        current_eligible_patch_mean_factor_ += domain_factor;
#pragma omp atomic
        current_eligible_patch_mean_h_scale_ +=
            (static_cast<size_t>(i) < current_patch_adaptation_.size())
                ? current_patch_adaptation_[static_cast<size_t>(i)].eligible_h_scale_applied
                : 1.0;
#pragma omp atomic
        current_eligible_patch_elements_sum_ +=
            static_cast<unsigned long long>(sub->GetNumberElements());
      }
      sub->SetPatch(p);
      malha->InsertSubMeshAdaptiveByPosition(sub, i);
      geo->GetPatch(i)->SetSubMesh(malha->GetSubMeshAdaptiveByPosition(i));
    }
#if USE_MPI
    timer->EndTimerParallel(RANK_MPI, id, 4);  // Adaptação do domínio
#else
    timer->EndTimerParallel(0, id, 4);   // Adaptação do domínio
#endif  // USE_MPI
  }
}

#endif  // USE_OPENMP

SubMesh* GeneratorAdaptive::InitialMesh(PatchCoons* patch,
                                        Performer::IdManager* idManager) {
  CurveAdaptive* c1 = patch->GetCurve(0);
  CurveAdaptive* c2 = patch->GetCurve(1);
  CurveAdaptive* c3 = patch->GetCurve(2);
  CurveAdaptive* c4 = patch->GetCurve(3);

  // 1. verifica quais curvas ainda não foram discretizadas
  if (c1->GetNumBerPoints())
    c1 = nullptr;  // c1 já foi trabalhada no patch vizinho
  if (c2->GetNumBerPoints())
    c2 = nullptr;  // c2 já foi trabalhada no patch vizinho
  if (c3->GetNumBerPoints())
    c3 = nullptr;  // c3 já foi trabalhada no patch vizinho
  if (c4->GetNumBerPoints())
    c4 = nullptr;  // c4 já foi trabalhada no patch vizinho

  SubMesh* sub = new SubMesh;

  //========================= Malha Grosseira
  //====================================
  // 2. divide cada patch em 9 regiões e gera os nós dos extremos de cada região
  for (double v = 0.0; v <= 1.0; v += 1) {
    for (double u = 0.0; u <= 1.0; u += 1) {
      //			cout << "u = " << u << " v = " << v << endl;
      PointAdaptive* p = new NodeAdaptive(patch->Parameterize(u, v));
      p->SetId(CheckedLegacyId(idManager->next(0)));
      bool inserted = false;

      //			cout << "ponto " << p->id << " " <<  p->x << " "
      //<< p->y << " " << p->z << endl; 			cout << "===="
      //<< endl;

      if (v == 0 && c1)  // p está na curva 1 (c1 = nullptr)
        c1->InsertPoint(p), inserted = true;
      else if (v == 1 && c3)  // p está na curva 3
        c3->InsertPoint(p), inserted = true;

      if (u == 0 && c4)  // p está na curva 4
        c4->InsertPoint(p), inserted = true;
      else if (u == 1 && c2)  // p está na curva 2
        c2->InsertPoint(p), inserted = true;

      sub->SetNoh(static_cast<NodeAdaptive*>(p));
      if (!inserted) delete p;
    }
  }

  // Coarse initial mesh: 4 triangles per patch with a central node.
  auto center =
      std::make_unique<NodeAdaptive>(patch->Parameterize(0.5, 0.5));
  center->SetId(CheckedLegacyId(idManager->next(0)));
  sub->AddNoh(std::move(center));

  ElementAdaptive* e1 =
      new TriangleAdaptive(sub->GetNoh(0), sub->GetNoh(1), sub->GetNoh(4));
  ((TriangleAdaptive*)e1)->SetParametersN1(make_tuple(0, 0));
  ((TriangleAdaptive*)e1)->SetParametersN2(make_tuple(1, 0));
  ((TriangleAdaptive*)e1)->SetParametersN3(make_tuple(0.5, 0.5));
      e1->SetId(CheckedLegacyId(idManager->next(1)));
  sub->SetElement(e1);

  ElementAdaptive* e2 =
      new TriangleAdaptive(sub->GetNoh(1), sub->GetNoh(3), sub->GetNoh(4));
  ((TriangleAdaptive*)e2)->SetParametersN1(make_tuple(1, 0));
  ((TriangleAdaptive*)e2)->SetParametersN2(make_tuple(1, 1));
  ((TriangleAdaptive*)e2)->SetParametersN3(make_tuple(0.5, 0.5));
      e2->SetId(CheckedLegacyId(idManager->next(1)));
  sub->SetElement(e2);

  ElementAdaptive* e3 =
      new TriangleAdaptive(sub->GetNoh(3), sub->GetNoh(2), sub->GetNoh(4));
  ((TriangleAdaptive*)e3)->SetParametersN1(make_tuple(1, 1));
  ((TriangleAdaptive*)e3)->SetParametersN2(make_tuple(0, 1));
  ((TriangleAdaptive*)e3)->SetParametersN3(make_tuple(0.5, 0.5));
      e3->SetId(CheckedLegacyId(idManager->next(1)));
  sub->SetElement(e3);

  ElementAdaptive* e4 =
      new TriangleAdaptive(sub->GetNoh(2), sub->GetNoh(0), sub->GetNoh(4));
  ((TriangleAdaptive*)e4)->SetParametersN1(make_tuple(0, 1));
  ((TriangleAdaptive*)e4)->SetParametersN2(make_tuple(0, 0));
  ((TriangleAdaptive*)e4)->SetParametersN3(make_tuple(0.5, 0.5));
      e4->SetId(CheckedLegacyId(idManager->next(1)));
  sub->SetElement(e4);
  //==============================================================================*/

  // 5. define a submalha do patch
  patch->SetSubMesh(sub);
  sub->SetPatch(patch);

  return sub;
}

// gera a malha inicial e insere na lista de malhas do modelo
// a lista de pontos da curva é preenchida durante a geração da malha inicial

// calcula o erro global da malha
double GeneratorAdaptive::ErrorGlobal(MeshAdaptive* malha, Timer* timer,
                                      [[maybe_unused]] int rank,
                                      [[maybe_unused]] int sizeThread) {
  unsigned int Ns = 0;  // número de submalhas
  unsigned int Nv = 0;  // número de vértices
  double Njs = 0;       // erro global da submalha
  double curvPower = 0.0;
  double Nj = 0;  // erro global da malha
  SubMesh* sub = 0;

  Ns = malha->GetNumberSubMeshesAdaptive();
  std::vector<double> sub_errors(Ns, 0.0);
  unsigned long long gauss_nodes_total = 0ULL;
  unsigned long long mean_nodes_total = 0ULL;
#if USE_MPI
#if USE_OPENMP
  // Seção de código para MPI + OpenMP
  // calculo do erro global
  timer->InitTimerParallel(RANK_MPI, omp_get_thread_num(), 7);
#else
  // Seção de código para MPI sem OpenMP
  // calculo do erro global
  timer->InitTimerParallel(RANK_MPI, 0, 7);
#endif
#else
#if USE_OPENMP
  // Seção de código para OpenMP sem MPI
  // calculo do erro global
  timer->InitTimerParallel(0, omp_get_thread_num(), 7);
#else
  // Seção de código sem MPI e sem OpenMP
  // calculo do erro global
  timer->InitTimerParallel(0, 0, 7);
#endif
#endif

  // Calcula o erro global de cada submalha
  for (unsigned int i = 0; i < Ns; ++i) {
    sub = malha->GetSubMeshAdaptiveByPosition(i);
    Nv = sub->GetNumberNos();
    Njs = 0.0;
    curvPower = 0.0;

    // Calcula o erro relativo para cada nó e soma a Nj
    for (unsigned int j = 0; j < Nv; ++j) {
#if USE_MPI
#if USE_OPENMP
      // Seção de código para MPI + OpenMP
      // calculo do erro global
      timer->EndTimerParallel(RANK_MPI, omp_get_thread_num(), 7);
      // MediaGauss
      timer->InitTimerParallel(RANK_MPI, omp_get_thread_num(), 6);
#else
      // Seção de código para MPI sem OpenMP
      // calculo do erro global
      timer->EndTimerParallel(RANK_MPI, 0, 7);
      // MediaGauss
      timer->InitTimerParallel(RANK_MPI, 0, 6);
#endif
#else
#if USE_OPENMP
      // Seção de código para OpenMP sem MPI
      // calculo do erro global
      timer->EndTimerParallel(0, omp_get_thread_num(), 7);
      // MediaGauss
      timer->InitTimerParallel(0, omp_get_thread_num(), 6);
#else
      // Seção de código sem MPI e sem OpenMP
      // calculo do erro global
      timer->EndTimerParallel(0, 0, 7);
      // MediaGauss
      timer->InitTimerParallel(0, 0, 6);
#endif
#endif

      PointAdaptive* point_adaptive = sub->GetNoh(j);
      Patch* p = sub->GetPatch();
      CurvatureAnalytical ka(*(static_cast<NodeAdaptive*>(point_adaptive)),
                             *(static_cast<PatchCoons*>(p)));
      CurvatureDiscrete kd(*(static_cast<NodeAdaptive*>(point_adaptive)));
      double Ga = ka.CalculateGaussCurvature();
      double Gd = kd.CalculateGaussCurvature();
      double Ha = ka.CalculateMeanCurvature();
      double Hd = kd.CalculateMeanCurvature();
      // atualiza as curvaturas do nó ( para que não sejam recalculadas na
      // adaptação das curvas e do domínio )
      (static_cast<NodeAdaptive*>(point_adaptive))->SetGa(Ga);
      (static_cast<NodeAdaptive*>(point_adaptive))->SetGd(Gd);
      (static_cast<NodeAdaptive*>(point_adaptive))->SetHa(Ha);
      (static_cast<NodeAdaptive*>(point_adaptive))->SetHd(Hd);

#if USE_MPI
#if USE_OPENMP
      // Seção de código para MPI + OpenMP
      // MediaGauss
      timer->EndTimerParallel(RANK_MPI, omp_get_thread_num(), 6);
      // calculo do erro global
      timer->InitTimerParallel(RANK_MPI, omp_get_thread_num(), 7);
#else
      // Seção de código para MPI sem OpenMP
      // MediaGauss
      timer->EndTimerParallel(RANK_MPI, 0, 6);
      // calculo do erro global
      timer->InitTimerParallel(RANK_MPI, 0, 7);
#endif
#else
#if USE_OPENMP
      // Seção de código para OpenMP sem MPI
      // MediaGauss
      timer->EndTimerParallel(0, omp_get_thread_num(), 6);
      // calculo do erro global
      timer->InitTimerParallel(0, omp_get_thread_num(), 7);
#else
      // Seção de código sem MPI e sem OpenMP
      // calculo do erro global
      timer->EndTimerParallel(0, 0, 7);
      // MediaGauss
      timer->InitTimerParallel(0, 0, 6);
#endif
#endif

      double power = 0.0;
      double diff = 0.0;
      if (fabs(Ga) >= TOLERANCE) {
        ++gauss_nodes_total;
        diff = Gd - Ga;
        power = pow(diff, 2);
        Njs += power;
        curvPower += pow(Ga, 2);
      } else if (fabs(Ha) >= TOLERANCE) {
        ++mean_nodes_total;
        diff = Hd - Ha;
        power = pow(diff, 2);
        Njs += power;
        curvPower += pow(Ha, 2);
      }
    }

    if (Njs > 0.0 && curvPower > 0.0 && Nv > 0U) {
      Njs = (double)sqrt(Njs / curvPower) / Nv;
    }

    Nj += Njs;
    sub_errors[i] = Njs;
  }

  if (Ns > 0) {
    Nj /= Ns;
  }
  last_submesh_error_ = std::move(sub_errors);
  current_gauss_nodes_ = gauss_nodes_total;
  current_mean_nodes_ = mean_nodes_total;

#if USE_MPI
#if USE_OPENMP
  // Seção de código para MPI + OpenMP
  // calculo do erro global
  timer->EndTimerParallel(RANK_MPI, omp_get_thread_num(), 7);
#else
  // Seção de código para MPI sem OpenMP
  // calculo do erro global
  timer->EndTimerParallel(RANK_MPI, 0, 7);
#endif
#else
#if USE_OPENMP
  // Seção de código para OpenMP sem MPI
  // calculo do erro global
  timer->EndTimerParallel(0, omp_get_thread_num(), 7);
#else
  // Seção de código sem MPI e sem OpenMP
  // calculo do erro global
  timer->EndTimerParallel(0, 0, 7);
#endif
#endif

  return Nj;
}

std::unique_ptr<Performer::IdManager> GeneratorAdaptive::MakeIdManager(
    const Parallel::TMCommunicator* comm, Int id) const {
  UInt numProcs = comm->numProcesses();
  UInt rank = comm->rank();

  ULInt procOffset = rank * this->id_range_;

  this->id_off_set_ = numProcs * this->id_range_;
  ULInt tidrange = this->id_range_ / comm->getMaxThreads();

  auto manager = std::make_unique<Performer::RangedIdManager>(1, 1, 1, 1, 2);

  ULInt threadOffset = id * tidrange;

  manager->setRange(tidrange);
  manager->setOffset(this->id_off_set_);
  manager->setMin(0,
                  /*this->idManager->getId(0)*/ 0 + procOffset + threadOffset);
  manager->setMin(1,
                  /*this->idManager->getId(1)*/ 0 + procOffset + threadOffset);

  return manager;
}

std::unique_ptr<Performer::IdManager> GeneratorAdaptive::MakeIdManagerOmp(
    const Parallel::TMCommunicator* comm, Int id) const {
  ULInt iNoh, iElemet;
  if (this->id_managers_[id]) {
    iNoh = this->id_managers_[id]->getId(0);
    iElemet = this->id_managers_[id]->getId(1);
  } else {
    iNoh = 0;
    iElemet = 0;
  }

  UInt numProcs = comm->numProcesses();
  UInt rank = comm->rank();

  ULInt procOffset = rank * this->id_range_;

  this->id_off_set_ = numProcs * this->id_range_;
  ULInt tidrange = this->id_range_ / comm->getMaxThreads();

  auto manager = std::make_unique<Performer::RangedIdManager>(1, 1, 1, 1, 2);

  ULInt threadOffset = id * tidrange;

  manager->setRange(tidrange);
  manager->setOffset(this->id_off_set_);
  manager->setMin(0, iNoh + procOffset + threadOffset);
  manager->setMin(1, iElemet + procOffset + threadOffset);

  return manager;
}

std::unique_ptr<Performer::IdManager> GeneratorAdaptive::MakeIdManagerElementOmp(
    const Parallel::TMCommunicator* comm, Int id) const {
  ULInt iNoh;
  if (this->id_managers_[id]) {
    iNoh = this->id_managers_[id]->getId(0);
  } else {
    iNoh = 0;
  }

  UInt numProcs = comm->numProcesses();
  UInt rank = comm->rank();

  ULInt procOffset = rank * this->id_range_;

  this->id_off_set_ = numProcs * this->id_range_;
  ULInt tidrange = this->id_range_ / comm->getMaxThreads();

  auto manager = std::make_unique<Performer::RangedIdManager>(1, 1, 1, 1, 2);

  ULInt threadOffset = id * tidrange;

  manager->setRange(tidrange);
  manager->setOffset(this->id_off_set_);
  manager->setMin(0, iNoh + procOffset + threadOffset);
  manager->setMin(1, 0 + procOffset + threadOffset);

  return manager;
}

void GeneratorAdaptive::SaveErrorMesh(MeshAdaptive* malha) {
  cout << "Salvando a Malha com " << malha->GetNumberSubMeshesAdaptive()
       << " subMalhas" << endl;

  unsigned int Ns = 0;  // número de submalhas
  unsigned int Nv = 0;  // número de vértices
  double Njs = 0;       // erro global da submalha
  double curvPower = 0.0;
  // double Nj = 0; // erro global da malha
  SubMesh* sub = 0;

  // Escreve arquivo com as curvaturas
  stringstream nome;
  nome << step_;
  nome << "erro";
  nome << step_;
  nome << ".log";

  ofstream arquivo(nome.str().c_str());

  Ns = malha->GetNumberSubMeshesAdaptive();

  // Calcula o erro global de cada submalha
  for (unsigned int i = 0; i < Ns; ++i) {
    sub = malha->GetSubMeshAdaptiveByPosition(i);
    Nv = sub->GetNumberNos();
    Njs = 0.0;
    curvPower = 0.0;
    // Calcula o erro relativo para cada nó e soma a Nj
    for (unsigned int j = 0; j < Nv; ++j) {
      PointAdaptive* n = sub->GetNoh(j);
      Patch* p = sub->GetPatch();
      CurvatureAnalytical ka(*(static_cast<NodeAdaptive*>(n)),
                             *(static_cast<PatchCoons*>(p)));
      CurvatureDiscrete kd(*(static_cast<NodeAdaptive*>(n)));
      double Ga = ka.CalculateGaussCurvature();
      double Gd = kd.CalculateGaussCurvature();
      double Ha = ka.CalculateMeanCurvature();
      double Hd = kd.CalculateMeanCurvature();
      // atualiza as curvaturas do nó ( para que não sejam recalculadas na
      // adaptação das curvas e do domínio )
      ((NodeAdaptive*)n)->SetGa(Ga);
      ((NodeAdaptive*)n)->SetGd(Gd);
      ((NodeAdaptive*)n)->SetHa(Ha);
      ((NodeAdaptive*)n)->SetHd(Hd);

      arquivo << "P " << n->GetId() << ": ( " << n->GetX() << ", " << n->GetY()
              << ", " << n->GetZ() << ")" << endl;
      tuple<double, double> t_n = ((PatchHermite*)p)->FindUV(*n);
      arquivo << "\tu = " << get<0>(t_n) << " v = " << get<1>(t_n) << endl;
      unsigned int num = ((NodeAdaptive*)n)->GetNumberElements();
      arquivo << "\t" << num << " elementos incidentes:";
      for (unsigned int incident_index = 0; incident_index < num;
           ++incident_index) {
        ElementAdaptive* elem =
            ((NodeAdaptive*)n)->GetElement(incident_index);
        arquivo << " T-" << elem->GetId();
      }
      arquivo << endl;
      arquivo << "\tGd = " << ((NodeAdaptive*)n)->GetGd()
              << " Ga = " << ((NodeAdaptive*)n)->GetGa() << endl;
      arquivo << "\tHd = " << ((NodeAdaptive*)n)->GetHd()
              << " Ha = " << ((NodeAdaptive*)n)->GetHa() << endl;

      double power = 0.0;
      double diff = 0.0;

      if (fabs(Ga) >= TOLERANCE) {
        diff = Gd - Ga;
        power = pow(diff, 2);
        Njs += power;
        curvPower += pow(Ga, 2);
        arquivo << "\tCd = " << ((NodeAdaptive*)n)->GetGd()
                << " Ca = " << ((NodeAdaptive*)n)->GetGa() << endl;
        arquivo << "\t|Cd - Ca| = " << fabs(diff) << endl;
        if (fabs(diff) <= TOLERANCE)
          arquivo << "\tdiferença menor que tolerância!!" << endl;
      } else if (fabs(Ha) >= TOLERANCE) {
        diff = Hd - Ha;
        power = pow(diff, 2);
        Njs += power;
        curvPower += pow(Ha, 2);
        arquivo << "\tCd = " << ((NodeAdaptive*)n)->GetHd()
                << " Ca = " << ((NodeAdaptive*)n)->GetHa() << endl;
        arquivo << "\t|Cd - Ca| = " << fabs(diff) << endl;
        if (fabs(diff) <= TOLERANCE)
          arquivo << "\tdiferença menor que tolerância!!" << endl;
      }

      arquivo << endl;
    }

    if (Njs > 0.0 && curvPower > 0.0) {
      Njs = (double)sqrt(Njs / curvPower) / Nv;
    }
  }

  arquivo.close();

  cout << "Malha salva com sucesso!!!" << endl;
}

void GeneratorAdaptive::WriteMesh(MeshAdaptive* malha, int output_step) {
  stringstream nome;
  nome << output_step;
  nome << "malha";
  nome << output_step;
  nome << ".obj";

  ofstream obj(nome.str().c_str());
  obj << "# Wavefront OBJ generated by ap_mesh" << endl;
  obj << "# step " << output_step << endl;

  std::unordered_map<ULInt, unsigned long int> node_index;
  unsigned long int next_index = 1;

  for (unsigned int i = 0; i < malha->GetNumberSubMeshesAdaptive(); i++) {
    SubMesh* sub = malha->GetSubMeshAdaptiveByPosition(i);
    for (unsigned int j = 0; j < sub->GetNumberNos(); j++) {
      NodeAdaptive* n = sub->GetNoh(j);
      auto it = node_index.find(n->GetId());
      if (it == node_index.end()) {
        node_index[n->GetId()] = next_index++;
        obj << "v " << n->GetX() << " " << n->GetY() << " " << n->GetZ()
            << endl;
      }
    }
  }

  for (unsigned int i = 0; i < malha->GetNumberSubMeshesAdaptive(); i++) {
    SubMesh* sub = malha->GetSubMeshAdaptiveByPosition(i);
    for (unsigned int j = 0; j < sub->GetNumberElements(); j++) {
      TriangleAdaptive* t = (TriangleAdaptive*)sub->GetElement(j);
      obj << "f " << node_index[t->GetNoh(1).GetId()] << " "
          << node_index[t->GetNoh(2).GetId()] << " "
          << node_index[t->GetNoh(3).GetId()] << endl;
    }
  }

  obj.flush();
  obj.close();

  //  cout << "escreveu o arquivo para o step_ " << output_step << endl;
}

void GeneratorAdaptive::WriteMesh(MeshAdaptive* malha, int output_step,
                                  vector<double> step_error, int rank) {
  stringstream nome;
  stringstream nome_obj;
  if (rank == -1) {
    nome << NAME_MODEL;
    nome << "_passo_";
    nome << output_step;
    nome << "_malha_";
    nome << output_step;
    nome << ".pos";

    nome_obj << NAME_MODEL;
    nome_obj << "_passo_";
    nome_obj << output_step;
    nome_obj << "_malha_";
    nome_obj << output_step;
    nome_obj << ".obj";
  } else {
    nome << NAME_MODEL;
    nome << "_n.process_";
    nome << NUMBER_PROCESS;
    nome << "_passo_";
    nome << output_step;
    nome << "_malha_";
    nome << output_step;
    nome << "_rank_";
    nome << rank;
    nome << ".pos";

    nome_obj << NAME_MODEL;
    nome_obj << "_n.process_";
    nome_obj << NUMBER_PROCESS;
    nome_obj << "_passo_";
    nome_obj << output_step;
    nome_obj << "_malha_";
    nome_obj << output_step;
    nome_obj << "_rank_";
    nome_obj << rank;
    nome_obj << ".obj";
  }

  EnsureOutputParentDirectories(nome.str());
  ofstream arq(nome.str().c_str());

  arq << "%HEADER" << endl
      << "Arquivo gerado pelo gerador de malhas de superficie" << endl;

  unsigned long int Nv, Nt;
  Nv = Nt = 0;

  for (unsigned int i = 0; i < malha->GetNumberSubMeshesAdaptive(); i++) {
    SubMesh* sub = malha->GetSubMeshAdaptiveByPosition(i);

    Nv += sub->GetNumberNos();
    Nt += sub->GetNumberElements();
  }

  arq << "%HEADER.VERSION" << endl
      << "0-005 - Oct/93" << endl
      << endl
      << "%HEADER.ANALYSIS" << endl
      << "\'shell\'" << endl
      << endl;

  arq << "erro global em cada step_" << endl;
  int n_pas = 0;
  for (vector<double>::iterator it = step_error.begin(); it != step_error.end();
       it++) {
    arq << "step_: " << n_pas << " erro:" << (*it) << endl;
    n_pas++;
  }
  arq << endl;

  arq << "%NODE" << endl << Nv << endl << endl;

  arq << "%NODE.COORD" << endl << Nv << endl;

  for (unsigned int i = 0; i < malha->GetNumberSubMeshesAdaptive(); i++) {
    SubMesh* sub = malha->GetSubMeshAdaptiveByPosition(i);

    for (unsigned int j = 0; j < sub->GetNumberNos(); j++) {
      NodeAdaptive* n = sub->GetNoh(j);

      arq << n->GetId() << " " << n->GetX() << " " << n->GetY() << " "
          << n->GetZ() << endl;
    }
  }

  arq << endl;

  arq << "%MATERIAL" << endl
      << "1" << endl
      << endl
      << "%MATERIAL.LABEL" << endl
      << "1" << endl
      << "1\t\'m1\'" << endl
      << endl
      << "%MATERIAL.ISOTROPIC" << endl
      << "1" << endl
      << "1\t1000.0\t0.0" << endl
      << endl
      << "%THICKNESS" << endl
      << "1" << endl
      << "1\t1.0" << endl
      << endl
      << "%INTEGRATION.ORDER" << endl
      << "1" << endl
      << "1\t3\t1\t1\t3\t1\t1" << endl
      << endl;

  arq << "%ELEMENT" << endl << Nt << endl << endl;

  arq << "%ELEMENT.T3" << endl << Nt << endl;

  for (unsigned int i = 0; i < malha->GetNumberSubMeshesAdaptive(); i++) {
    SubMesh* sub = malha->GetSubMeshAdaptiveByPosition(i);

    for (unsigned int j = 0; j < sub->GetNumberElements(); j++) {
      TriangleAdaptive* t = (TriangleAdaptive*)sub->GetElement(j);

      arq << t->GetId() << " "
          << "1 1 1 " << t->GetNoh(1).GetId() << " " << t->GetNoh(2).GetId()
          << " " << t->GetNoh(3).GetId() << endl;
    }
  }

  arq << endl;
  arq << "%END";

  arq.flush();

  arq.close();
  std::remove(nome.str().c_str());

  // Também exporta no formato Wavefront OBJ.
  ofstream obj(nome_obj.str().c_str());
  obj << "# Wavefront OBJ generated by ap_mesh" << endl;
  obj << "# step " << output_step << endl;

  std::unordered_map<ULInt, unsigned long int> node_index;
  unsigned long int next_index = 1;

  for (unsigned int i = 0; i < malha->GetNumberSubMeshesAdaptive(); i++) {
    SubMesh* sub = malha->GetSubMeshAdaptiveByPosition(i);

    for (unsigned int j = 0; j < sub->GetNumberNos(); j++) {
      NodeAdaptive* n = sub->GetNoh(j);
      auto it = node_index.find(n->GetId());
      if (it == node_index.end()) {
        node_index[n->GetId()] = next_index++;
        obj << "v " << n->GetX() << " " << n->GetY() << " " << n->GetZ()
            << endl;
      }
    }
  }

  for (unsigned int i = 0; i < malha->GetNumberSubMeshesAdaptive(); i++) {
    SubMesh* sub = malha->GetSubMeshAdaptiveByPosition(i);

    for (unsigned int j = 0; j < sub->GetNumberElements(); j++) {
      TriangleAdaptive* t = (TriangleAdaptive*)sub->GetElement(j);
      obj << "f " << node_index[t->GetNoh(1).GetId()] << " "
          << node_index[t->GetNoh(2).GetId()] << " "
          << node_index[t->GetNoh(3).GetId()] << endl;
    }
  }

  obj.flush();
  obj.close();

  cout << "escreveu o arquivo para o step_ " << output_step << endl;

  // Análise dos Elementos da Malha Gerada

  // cout<< "INIT >> ANÁLISE DOS ELEMENTOS DA MALHA GERADA"<< endl;
  stringstream nameFile;

  nameFile << NAME_MODEL;
  nameFile << "_n.process_";
  nameFile << NUMBER_PROCESS;
  nameFile << "_passo_";
  nameFile << output_step;
  nameFile << "_qualite_";
  nameFile << output_step;
  nameFile << "_rank_";
  nameFile << rank;
  nameFile << ".log";

  EnsureOutputParentDirectories(nameFile.str());
  ofstream file(nameFile.str().c_str());

  file << "File Analise qualidade" << endl << endl;

  std::vector<double> vec_0_10;
  std::vector<double> vec_10_20;
  std::vector<double> vec_20_30;
  std::vector<double> vec_30_40;
  std::vector<double> vec_40_50;
  std::vector<double> vec_50_60;
  std::vector<double> vec_60_70;
  std::vector<double> vec_70_80;
  std::vector<double> vec_80_90;
  std::vector<double> vec_90_100;

  for (unsigned int i = 0; i < malha->GetNumberSubMeshesAdaptive(); i++) {
    SubMesh* sub = malha->GetSubMeshAdaptiveByPosition(i);

    for (unsigned int j = 0; j < sub->GetNumberElements(); j++) {
      TriangleAdaptive* t = (TriangleAdaptive*)sub->GetElement(j);

      double value = t->CalculateQualityTriangle();

      if (0.0 <= value && value <= 0.1) {
        vec_0_10.push_back(value);
      } else if (0.1 < value && value <= 0.2) {
        vec_10_20.push_back(value);
      } else if (0.2 < value && value <= 0.3) {
        vec_20_30.push_back(value);
      } else if (0.3 < value && value <= 0.4) {
        vec_30_40.push_back(value);
      } else if (0.4 < value && value <= 0.5) {
        vec_40_50.push_back(value);
      } else if (0.5 < value && value <= 0.6) {
        vec_50_60.push_back(value);
      } else if (0.6 < value && value <= 0.7) {
        vec_60_70.push_back(value);
      } else if (0.7 < value && value <= 0.8) {
        vec_70_80.push_back(value);
      } else if (0.8 < value && value <= 0.9) {
        vec_80_90.push_back(value);
      } else if (0.9 < value && value <= 1) {
        vec_90_100.push_back(value);
      }
    }
  }

  file << "Quantidade elementos em cada caso de 0 a 1" << endl;
  file << vec_0_10.size() << endl;
  file << vec_10_20.size() << endl;
  file << vec_20_30.size() << endl;
  file << vec_30_40.size() << endl;
  file << vec_40_50.size() << endl;
  file << vec_50_60.size() << endl;
  file << vec_60_70.size() << endl;
  file << vec_70_80.size() << endl;
  file << vec_80_90.size() << endl;
  file << vec_90_100.size() << endl << endl;

  file << "Porcetagem elementos em cada caso de 0 a 1" << endl;

  const std::size_t population =
      vec_0_10.size() + vec_10_20.size() + vec_20_30.size() +
      vec_30_40.size() + vec_40_50.size() + vec_50_60.size() +
      vec_60_70.size() + vec_70_80.size() + vec_80_90.size() +
      vec_90_100.size();
  const double population_as_double = static_cast<double>(population);

  file << static_cast<double>(vec_0_10.size()) * 100.0 /
              population_as_double
       << endl;
  file << static_cast<double>(vec_10_20.size()) * 100.0 /
              population_as_double
       << endl;
  file << static_cast<double>(vec_20_30.size()) * 100.0 /
              population_as_double
       << endl;
  file << static_cast<double>(vec_30_40.size()) * 100.0 /
              population_as_double
       << endl;
  file << static_cast<double>(vec_40_50.size()) * 100.0 /
              population_as_double
       << endl;
  file << static_cast<double>(vec_50_60.size()) * 100.0 /
              population_as_double
       << endl;
  file << static_cast<double>(vec_60_70.size()) * 100.0 /
              population_as_double
       << endl;
  file << static_cast<double>(vec_70_80.size()) * 100.0 /
              population_as_double
       << endl;
  file << static_cast<double>(vec_80_90.size()) * 100.0 /
              population_as_double
       << endl;
  file << static_cast<double>(vec_90_100.size()) * 100.0 /
              population_as_double
       << endl;

  file.flush();

  file.close();

  // cout<< "END >> ANÁLISE DOS ELEMENTOS DA MALHA GERADA"<< endl;
}

void GeneratorAdaptive::WriteQualityMesh(MeshAdaptive* malha, int output_step,
                                         vector<double> step_error,
                                         int rank) {
  stringstream nameFile;
  nameFile << NAME_MODEL;
  nameFile << "_n.process_";
  nameFile << NUMBER_PROCESS;
  nameFile << "_passo_";
  nameFile << output_step;
  nameFile << "_qualite_";
  nameFile << output_step;
  nameFile << "_rank_";
  nameFile << rank;
  nameFile << ".log";

  EnsureOutputParentDirectories(nameFile.str());
  ofstream file(nameFile.str().c_str());

  const unsigned int num_submeshes = malha->GetNumberSubMeshesAdaptive();
  unsigned long long total_nodes = 0;
  unsigned long long total_elements = 0;

  std::vector<unsigned long long> nodes_per_submesh;
  std::vector<unsigned long long> elements_per_submesh;
  nodes_per_submesh.reserve(num_submeshes);
  elements_per_submesh.reserve(num_submeshes);

  std::vector<double> qualities_3d;
  std::vector<double> qualities_uv;
  std::vector<double> min_angles_3d;
  std::vector<double> max_angles_3d;
  std::vector<double> angle_qualities_3d;
  std::vector<double> edge_ratios_3d;
  std::vector<double> areas;
  std::vector<unsigned long long> histogram_3d(10, 0ULL);
  std::vector<unsigned long long> histogram_uv(10, 0ULL);

  std::map<std::pair<ULInt, ULInt>, unsigned int> edge_use_count;

  auto add_edge = [&edge_use_count](ULInt a, ULInt b) {
    if (a > b) std::swap(a, b);
    edge_use_count[std::make_pair(a, b)]++;
  };

  for (unsigned int i = 0; i < num_submeshes; i++) {
    SubMesh* sub = malha->GetSubMeshAdaptiveByPosition(i);
    const unsigned long long sub_nodes = sub->GetNumberNos();
    const unsigned long long sub_elements = sub->GetNumberElements();

    total_nodes += sub_nodes;
    total_elements += sub_elements;
    nodes_per_submesh.push_back(sub_nodes);
    elements_per_submesh.push_back(sub_elements);

    for (unsigned int j = 0; j < sub->GetNumberElements(); j++) {
      TriangleAdaptive* t = static_cast<TriangleAdaptive*>(sub->GetElement(j));

      const double quality_3d = t->CalculateQualityTriangle();
      const double quality_uv = TriangleQualityFromParameters(
          t->GetParametersN1(), t->GetParametersN2(), t->GetParametersN3());
      const TriangleShapeMetrics shape_3d = ComputeTriangleShapeMetrics3D(
          t->GetNoh(1), t->GetNoh(2), t->GetNoh(3));
      qualities_3d.push_back(quality_3d);
      qualities_uv.push_back(quality_uv);
      min_angles_3d.push_back(shape_3d.min_angle_deg);
      max_angles_3d.push_back(shape_3d.max_angle_deg);
      angle_qualities_3d.push_back(shape_3d.angle_quality);
      edge_ratios_3d.push_back(shape_3d.edge_ratio);
      areas.push_back(t->GetArea());

      int bin_3d = static_cast<int>(quality_3d * 10.0);
      if (bin_3d < 0) bin_3d = 0;
      if (bin_3d > 9) bin_3d = 9;
      histogram_3d[static_cast<size_t>(bin_3d)]++;

      int bin_uv = static_cast<int>(quality_uv * 10.0);
      if (bin_uv < 0) bin_uv = 0;
      if (bin_uv > 9) bin_uv = 9;
      histogram_uv[static_cast<size_t>(bin_uv)]++;

      const ULInt id1 = t->GetNoh(1).GetId();
      const ULInt id2 = t->GetNoh(2).GetId();
      const ULInt id3 = t->GetNoh(3).GetId();
      add_edge(id1, id2);
      add_edge(id2, id3);
      add_edge(id3, id1);
    }
  }

  auto get_mean = [](const std::vector<double>& v) {
    if (v.empty()) return 0.0;
    return std::accumulate(v.begin(), v.end(), 0.0) /
           static_cast<double>(v.size());
  };

  auto get_stddev = [](const std::vector<double>& v, double mean) {
    if (v.size() < 2) return 0.0;
    double acc = 0.0;
    for (double x : v) {
      const double d = x - mean;
      acc += d * d;
    }
    return std::sqrt(acc / static_cast<double>(v.size() - 1));
  };

  auto get_percentile = [](std::vector<double> v, double p) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const double pos = p * static_cast<double>(v.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(pos));
    const size_t hi = static_cast<size_t>(std::ceil(pos));
    if (lo == hi) return v[lo];
    const double w = pos - static_cast<double>(lo);
    return v[lo] * (1.0 - w) + v[hi] * w;
  };

  auto minmax_quality_3d =
      std::minmax_element(qualities_3d.begin(), qualities_3d.end());
  const double quality_min = qualities_3d.empty() ? 0.0 : *(minmax_quality_3d.first);
  const double quality_max = qualities_3d.empty() ? 0.0 : *(minmax_quality_3d.second);
  const double quality_mean = get_mean(qualities_3d);
  const double quality_std = get_stddev(qualities_3d, quality_mean);
  const double quality_p05 = get_percentile(qualities_3d, 0.05);
  const double quality_p50 = get_percentile(qualities_3d, 0.50);
  const double quality_p95 = get_percentile(qualities_3d, 0.95);

  auto minmax_quality_uv =
      std::minmax_element(qualities_uv.begin(), qualities_uv.end());
  const double quality_min_uv =
      qualities_uv.empty() ? 0.0 : *(minmax_quality_uv.first);
  const double quality_max_uv =
      qualities_uv.empty() ? 0.0 : *(minmax_quality_uv.second);
  const double quality_mean_uv = get_mean(qualities_uv);
  const double quality_std_uv = get_stddev(qualities_uv, quality_mean_uv);
  const double quality_p05_uv = get_percentile(qualities_uv, 0.05);
  const double quality_p50_uv = get_percentile(qualities_uv, 0.50);
  const double quality_p95_uv = get_percentile(qualities_uv, 0.95);
  const double min_angle_mean_3d = get_mean(min_angles_3d);
  const double min_angle_p05_3d = get_percentile(min_angles_3d, 0.05);
  const double max_angle_mean_3d = get_mean(max_angles_3d);
  const double max_angle_p95_3d = get_percentile(max_angles_3d, 0.95);
  const double angle_quality_mean_3d = get_mean(angle_qualities_3d);
  const double edge_ratio_mean_3d = get_mean(edge_ratios_3d);

  const double area_mean = get_mean(areas);
  const double area_std = get_stddev(areas, area_mean);
  auto minmax_area = std::minmax_element(areas.begin(), areas.end());
  const double area_min = areas.empty() ? 0.0 : *(minmax_area.first);
  const double area_max = areas.empty() ? 0.0 : *(minmax_area.second);

  unsigned long long poor_quality = 0;
  unsigned long long good_quality_ge_0_60 = 0;
  unsigned long long excellent_quality = 0;
  for (double q : qualities_3d) {
    if (q < 0.3) poor_quality++;
    if (q >= 0.6) good_quality_ge_0_60++;
    if (q >= 0.8) excellent_quality++;
  }

  unsigned long long poor_quality_uv = 0;
  unsigned long long good_quality_ge_0_60_uv = 0;
  unsigned long long excellent_quality_uv = 0;
  for (double q : qualities_uv) {
    if (q < 0.3) poor_quality_uv++;
    if (q >= 0.6) good_quality_ge_0_60_uv++;
    if (q >= 0.8) excellent_quality_uv++;
  }

  unsigned long long boundary_edges = 0;
  unsigned long long nonmanifold_edges = 0;
  for (const auto& e : edge_use_count) {
    if (e.second == 1) boundary_edges++;
    if (e.second > 2) nonmanifold_edges++;
  }

  const double error_current =
      (output_step >= 0 && output_step < static_cast<int>(step_error.size()))
          ? step_error[static_cast<size_t>(output_step)]
          : this->error_local_process_;
  const double error_prev =
      (output_step > 0 && (output_step - 1) < static_cast<int>(step_error.size()))
          ? step_error[static_cast<size_t>(output_step - 1)]
          : (!step_error.empty() ? step_error.back() : -1.0);
  const double error_delta_abs =
      (error_current >= 0.0 && error_prev >= 0.0) ? (error_prev - error_current)
                                                  : 0.0;
  const double error_delta_pct =
      (error_current >= 0.0 && error_prev > 0.0)
          ? (error_delta_abs / error_prev) * 100.0
          : 0.0;
  const unsigned long long elements_prev =
      (output_step > 0 && (output_step - 1) < static_cast<int>(elements_step_.size()))
          ? elements_step_[static_cast<size_t>(output_step - 1)]
          : total_elements;
  const unsigned long long elements_delta =
      total_elements > elements_prev ? total_elements - elements_prev : 0ULL;
  const double error_drop_per_element =
      (output_step >= 0 && output_step < static_cast<int>(error_drop_per_element_step_.size()))
          ? error_drop_per_element_step_[static_cast<size_t>(output_step)]
          : 0.0;
  const double element_growth_factor =
      (output_step >= 0 && output_step < static_cast<int>(element_growth_factor_step_.size()))
          ? element_growth_factor_step_[static_cast<size_t>(output_step)]
          : 1.0;
  const double error_normalized =
      (output_step >= 0 && output_step < static_cast<int>(error_normalized_step_.size()))
          ? error_normalized_step_[static_cast<size_t>(output_step)]
          : ((!step_error.empty() && step_error.front() > 0.0)
                 ? (error_current / step_error.front())
                 : 0.0);
  const double error_total_reduction_pct =
      (output_step >= 0 &&
       output_step < static_cast<int>(error_total_reduction_pct_step_.size()))
          ? error_total_reduction_pct_step_[static_cast<size_t>(output_step)]
          : ((!step_error.empty() && step_error.front() > 0.0)
                 ? ((step_error.front() - error_current) / step_error.front()) * 100.0
                 : 0.0);
  const double acceptance_score =
      (output_step >= 0 && output_step < static_cast<int>(acceptance_score_step_.size()))
          ? acceptance_score_step_[static_cast<size_t>(output_step)]
          : 0.0;
  const unsigned long long gauss_nodes =
      (output_step >= 0 && output_step < static_cast<int>(gauss_nodes_step_.size()))
          ? gauss_nodes_step_[static_cast<size_t>(output_step)]
          : 0ULL;
  const unsigned long long mean_nodes =
      (output_step >= 0 && output_step < static_cast<int>(mean_nodes_step_.size()))
          ? mean_nodes_step_[static_cast<size_t>(output_step)]
          : 0ULL;
  const unsigned int retry_attempt =
      (output_step >= 0 && output_step < static_cast<int>(retry_attempt_step_.size()))
          ? retry_attempt_step_[static_cast<size_t>(output_step)]
          : 0U;
  const double element_budget_abs =
      (output_step >= 0 && output_step < static_cast<int>(element_budget_abs_step_.size()))
          ? element_budget_abs_step_[static_cast<size_t>(output_step)]
          : current_element_budget_abs_;
  const double element_budget_growth_factor_limit =
      (output_step >= 0 &&
       output_step < static_cast<int>(element_budget_growth_factor_limit_step_.size()))
          ? element_budget_growth_factor_limit_step_[static_cast<size_t>(output_step)]
          : current_element_budget_growth_factor_limit_;
  const double element_budget_soft_limit =
      (output_step >= 0 && output_step < static_cast<int>(element_budget_soft_limit_step_.size()))
          ? element_budget_soft_limit_step_[static_cast<size_t>(output_step)]
          : current_element_budget_soft_limit_;
  const unsigned long long candidate_elements_before_budget_retry =
      (output_step >= 0 &&
       output_step <
           static_cast<int>(candidate_elements_before_budget_retry_step_.size()))
          ? candidate_elements_before_budget_retry_step_[static_cast<size_t>(output_step)]
          : current_candidate_elements_before_budget_retry_;
  const unsigned int budget_retry_count =
      (output_step >= 0 && output_step < static_cast<int>(budget_retry_count_step_.size()))
          ? budget_retry_count_step_[static_cast<size_t>(output_step)]
          : current_budget_retry_count_;
  const int budget_limited =
      (output_step >= 0 && output_step < static_cast<int>(budget_limited_step_.size()))
          ? budget_limited_step_[static_cast<size_t>(output_step)]
          : (current_budget_limited_ ? 1 : 0);
  const std::string budget_final_status =
      (output_step >= 0 && output_step < static_cast<int>(budget_final_status_step_.size()))
          ? budget_final_status_step_[static_cast<size_t>(output_step)]
          : current_budget_final_status_;
  const std::string step_mode =
      (output_step >= 0 && output_step < static_cast<int>(step_mode_step_.size()))
          ? step_mode_step_[static_cast<size_t>(output_step)]
          : current_step_mode_;
  const double budget_pressure =
      (output_step >= 0 && output_step < static_cast<int>(budget_pressure_step_.size()))
          ? budget_pressure_step_[static_cast<size_t>(output_step)]
          : current_budget_pressure_;
  const unsigned int eligible_patch_count =
      (output_step >= 0 && output_step < static_cast<int>(eligible_patch_count_step_.size()))
          ? eligible_patch_count_step_[static_cast<size_t>(output_step)]
          : current_eligible_patch_count_;
  const unsigned long long eligible_patch_elements_sum =
      (output_step >= 0 &&
       output_step < static_cast<int>(eligible_patch_elements_sum_step_.size()))
          ? eligible_patch_elements_sum_step_[static_cast<size_t>(output_step)]
          : current_eligible_patch_elements_sum_;
  const double eligible_patch_mean_factor =
      (output_step >= 0 &&
       output_step < static_cast<int>(eligible_patch_mean_factor_step_.size()))
          ? eligible_patch_mean_factor_step_[static_cast<size_t>(output_step)]
          : current_eligible_patch_mean_factor_;
  const double eligible_patch_mean_h_scale =
      (output_step >= 0 &&
       output_step < static_cast<int>(eligible_patch_mean_h_scale_step_.size()))
          ? eligible_patch_mean_h_scale_step_[static_cast<size_t>(output_step)]
          : current_eligible_patch_mean_h_scale_;
  const double applied_relaxation =
      (output_step >= 0 && output_step < static_cast<int>(applied_relaxation_step_.size()))
          ? applied_relaxation_step_[static_cast<size_t>(output_step)]
          : ADAPTATION_RELAXATION;
  const double applied_max_delta =
      (output_step >= 0 && output_step < static_cast<int>(applied_max_delta_step_.size()))
          ? applied_max_delta_step_[static_cast<size_t>(output_step)]
          : ADAPTATION_MAX_DELTA;
  const int retry_enabled =
      (output_step >= 0 && output_step < static_cast<int>(retry_enabled_step_.size()))
          ? retry_enabled_step_[static_cast<size_t>(output_step)]
          : (ADAPTIVE_RETRY_COUNT > 0 ? 1 : 0);
  const int local_budget_mode =
      (output_step >= 0 && output_step < static_cast<int>(local_budget_mode_step_.size()))
          ? local_budget_mode_step_[static_cast<size_t>(output_step)]
          : PATCH_LOCAL_ELEMENT_BUDGET_MODE;
  const double target_element_growth =
      (output_step >= 0 && output_step < static_cast<int>(target_element_growth_step_.size()))
          ? target_element_growth_step_[static_cast<size_t>(output_step)]
          : current_target_element_growth_;
  const double actual_element_growth =
      (output_step >= 0 && output_step < static_cast<int>(actual_element_growth_step_.size()))
          ? actual_element_growth_step_[static_cast<size_t>(output_step)]
          : element_growth_factor;
  const double step_efficiency =
      (output_step >= 0 && output_step < static_cast<int>(step_efficiency_step_.size()))
          ? step_efficiency_step_[static_cast<size_t>(output_step)]
          : current_step_efficiency_;
  const std::string generation_control_mode =
      (output_step >= 0 &&
       output_step < static_cast<int>(generation_control_mode_step_.size()))
          ? generation_control_mode_step_[static_cast<size_t>(output_step)]
          : current_generation_control_mode_;
  const double target_elements_step =
      (output_step >= 0 && output_step < static_cast<int>(target_elements_step_.size()))
          ? target_elements_step_[static_cast<size_t>(output_step)]
          : current_target_elements_step_;
  const unsigned long long actual_elements_step =
      (output_step >= 0 && output_step < static_cast<int>(actual_elements_step_.size()))
          ? actual_elements_step_[static_cast<size_t>(output_step)]
          : total_elements;
  const double target_growth_step =
      (output_step >= 0 && output_step < static_cast<int>(target_growth_step_.size()))
          ? target_growth_step_[static_cast<size_t>(output_step)]
          : current_target_element_growth_;
  const double actual_growth_step =
      (output_step >= 0 && output_step < static_cast<int>(actual_growth_step_.size()))
          ? actual_growth_step_[static_cast<size_t>(output_step)]
          : element_growth_factor;
  const std::string budget_driver_mode =
      (output_step >= 0 && output_step < static_cast<int>(budget_driver_mode_step_.size()))
          ? budget_driver_mode_step_[static_cast<size_t>(output_step)]
          : current_budget_driver_mode_;
  const double local_progress_coverage =
      (output_step >= 0 && output_step < static_cast<int>(local_progress_coverage_step_.size()))
          ? local_progress_coverage_step_[static_cast<size_t>(output_step)]
          : current_local_progress_coverage_;
  const unsigned int improved_top_patches =
      (output_step >= 0 && output_step < static_cast<int>(improved_top_patches_step_.size()))
          ? improved_top_patches_step_[static_cast<size_t>(output_step)]
          : current_improved_top_patches_;
  const unsigned int top_patch_candidate_count =
      (output_step >= 0 && output_step < static_cast<int>(top_patch_candidate_count_step_.size()))
          ? top_patch_candidate_count_step_[static_cast<size_t>(output_step)]
          : current_top_patch_candidate_count_;
  const double patch_budget_dispersion =
      (output_step >= 0 &&
       output_step < static_cast<int>(patch_budget_dispersion_step_.size()))
          ? patch_budget_dispersion_step_[static_cast<size_t>(output_step)]
          : current_patch_budget_dispersion_;
  const double patch_h_dispersion =
      (output_step >= 0 && output_step < static_cast<int>(patch_h_dispersion_step_.size()))
          ? patch_h_dispersion_step_[static_cast<size_t>(output_step)]
          : current_patch_h_dispersion_;
  const double actual_patch_element_dispersion =
      (output_step >= 0 &&
       output_step < static_cast<int>(actual_patch_element_dispersion_step_.size()))
          ? actual_patch_element_dispersion_step_[static_cast<size_t>(output_step)]
          : current_actual_patch_element_dispersion_;
  const double curve_balance_dispersion =
      (output_step >= 0 &&
       output_step < static_cast<int>(curve_balance_dispersion_step_.size()))
          ? curve_balance_dispersion_step_[static_cast<size_t>(output_step)]
          : current_curve_balance_dispersion_;
  const double patch_consistency_penalty =
      (output_step >= 0 &&
       output_step < static_cast<int>(patch_consistency_penalty_step_.size()))
          ? patch_consistency_penalty_step_[static_cast<size_t>(output_step)]
          : current_patch_consistency_penalty_;
  const int consistency_regularization_active =
      (output_step >= 0 &&
       output_step < static_cast<int>(consistency_regularization_active_step_.size()))
          ? consistency_regularization_active_step_[static_cast<size_t>(output_step)]
          : current_consistency_regularization_active_;

  const auto min_nodes_sub =
      nodes_per_submesh.empty()
          ? 0ULL
          : *std::min_element(nodes_per_submesh.begin(), nodes_per_submesh.end());
  const auto max_nodes_sub =
      nodes_per_submesh.empty()
          ? 0ULL
          : *std::max_element(nodes_per_submesh.begin(), nodes_per_submesh.end());
  const auto min_elem_sub =
      elements_per_submesh.empty()
          ? 0ULL
          : *std::min_element(elements_per_submesh.begin(),
                              elements_per_submesh.end());
  const auto max_elem_sub =
      elements_per_submesh.empty()
          ? 0ULL
          : *std::max_element(elements_per_submesh.begin(),
                              elements_per_submesh.end());

  const double mean_nodes_sub = nodes_per_submesh.empty()
                                    ? 0.0
                                    : static_cast<double>(total_nodes) /
                                          static_cast<double>(nodes_per_submesh.size());
  const double mean_elem_sub = elements_per_submesh.empty()
                                   ? 0.0
                                   : static_cast<double>(total_elements) /
                                         static_cast<double>(elements_per_submesh.size());

  file << "Analise da malha por rodada" << endl << endl;
  file << "step=" << output_step << ", rank=" << rank << endl;
  file << "nodes_total=" << total_nodes << endl;
  file << "elements_total=" << total_elements << endl;
  file << "submeshes_total=" << num_submeshes << endl;
  file << "nodes_per_submesh[min/mean/max]=" << min_nodes_sub << "/"
       << mean_nodes_sub << "/" << max_nodes_sub << endl;
  file << "elements_per_submesh[min/mean/max]=" << min_elem_sub << "/"
       << mean_elem_sub << "/" << max_elem_sub << endl;
  file << "error_global_current=" << error_current << endl;
  file << "error_global_previous=" << error_prev << endl;
  file << "error_global_surface_curvature_current=" << error_current << endl;
  file << "error_global_surface_curvature_previous=" << error_prev << endl;
  file << "error_global_definition=surface_curvature_3d_relative" << endl;
  file << "error_improvement_abs=" << error_delta_abs << endl;
  file << "error_improvement_pct=" << error_delta_pct << endl;
  file << "elements_previous=" << elements_prev << endl;
  file << "elements_delta=" << elements_delta << endl;
  file << "element_growth_factor=" << element_growth_factor << endl;
  file << "error_improvement_per_element=" << error_drop_per_element << endl;
  file << "error_normalized=" << error_normalized << endl;
  file << "error_total_reduction_pct=" << error_total_reduction_pct << endl;
  file << "acceptance_score=" << acceptance_score << endl;
  file << "curvature_mode_gauss_nodes=" << gauss_nodes << endl;
  file << "curvature_mode_mean_nodes=" << mean_nodes << endl;
  file << "curvature_mode_gauss_ratio="
       << ((gauss_nodes + mean_nodes) > 0ULL
               ? static_cast<double>(gauss_nodes) /
                     static_cast<double>(gauss_nodes + mean_nodes)
               : 0.0)
       << endl;
  file << "curvature_mode_mean_ratio="
       << ((gauss_nodes + mean_nodes) > 0ULL
               ? static_cast<double>(mean_nodes) /
                     static_cast<double>(gauss_nodes + mean_nodes)
               : 0.0)
       << endl;
  file << "error_floor=" << ERROR_FLOOR << endl;
  file << "min_error_drop_abs=" << MIN_ERROR_DROP_ABS << endl;
  file << "min_error_drop_per_element=" << MIN_ERROR_DROP_PER_ELEMENT << endl;
  file << "retry_attempt=" << retry_attempt << endl;
  file << "retry_enabled=" << retry_enabled << endl;
  file << "local_budget_mode=" << local_budget_mode << endl;
  file << "target_element_growth=" << target_element_growth << endl;
  file << "target_elements_step=" << target_elements_step << endl;
  file << "actual_elements_step=" << actual_elements_step << endl;
  file << "target_growth_step=" << target_growth_step << endl;
  file << "actual_growth_step=" << actual_growth_step << endl;
  file << "actual_element_growth=" << actual_element_growth << endl;
  file << "step_efficiency=" << step_efficiency << endl;
  file << "local_progress_coverage=" << local_progress_coverage << endl;
  file << "improved_top_patches=" << improved_top_patches << endl;
  file << "top_patch_candidate_count=" << top_patch_candidate_count << endl;
  file << "generation_control_mode=" << generation_control_mode << endl;
  file << "consistency_regularization_active="
       << consistency_regularization_active << endl;
  file << "patch_budget_dispersion=" << patch_budget_dispersion << endl;
  file << "patch_h_dispersion=" << patch_h_dispersion << endl;
  file << "actual_patch_element_dispersion="
       << actual_patch_element_dispersion << endl;
  file << "curve_balance_dispersion=" << curve_balance_dispersion << endl;
  file << "patch_consistency_penalty=" << patch_consistency_penalty << endl;
  file << "budget_driver_mode=" << budget_driver_mode << endl;
  file << "adaptation_relaxation=" << applied_relaxation << endl;
  file << "adaptation_max_delta=" << applied_max_delta << endl;
  file << "adaptive_retry_count=" << ADAPTIVE_RETRY_COUNT << endl;
  file << "adaptive_retry_shrink=" << ADAPTIVE_RETRY_SHRINK << endl;
  file << "max_retry_element_growth_factor="
       << MAX_RETRY_ELEMENT_GROWTH_FACTOR << endl;
  file << "max_retry_element_growth_abs=" << MAX_RETRY_ELEMENT_GROWTH_ABS
       << endl;
  file << "stop_reason=" << stop_reason_ << endl;
  file << "laplacian_smoothing_iterations="
       << static_cast<unsigned int>(SMOOTHING_LAPLACIAN_NUMBER) << endl;
  file << "laplacian_smoothing_factor=" << SMOOTHING_LAPLACIAN_FACTOR << endl;
  file << endl;

  file << "quality[min/p05/p50/p95/max]=" << quality_min << "/" << quality_p05
       << "/" << quality_p50 << "/" << quality_p95 << "/" << quality_max
       << endl;
  file << "quality_mean=" << quality_mean << endl;
  file << "quality_stddev=" << quality_std << endl;
  file << "quality_poor_ratio_lt_0.30="
       << (total_elements > 0
               ? (100.0 * static_cast<double>(poor_quality) /
                  static_cast<double>(total_elements))
               : 0.0)
       << endl;
  file << "quality_good_ratio_ge_0.60="
       << (total_elements > 0
               ? (100.0 * static_cast<double>(good_quality_ge_0_60) /
                  static_cast<double>(total_elements))
               : 0.0)
       << endl;
  file << "quality_excellent_ratio_ge_0.80="
       << (total_elements > 0
               ? (100.0 * static_cast<double>(excellent_quality) /
                  static_cast<double>(total_elements))
               : 0.0)
       << endl;
  file << "angle_quality_mean_3d=" << angle_quality_mean_3d << endl;
  file << "edge_ratio_mean_3d=" << edge_ratio_mean_3d << endl;
  file << "min_angle_mean_3d=" << min_angle_mean_3d << endl;
  file << "min_angle_p05_3d=" << min_angle_p05_3d << endl;
  file << "max_angle_mean_3d=" << max_angle_mean_3d << endl;
  file << "max_angle_p95_3d=" << max_angle_p95_3d << endl;
  file << endl;

  file << "quality_uv[min/p05/p50/p95/max]=" << quality_min_uv << "/"
       << quality_p05_uv << "/" << quality_p50_uv << "/" << quality_p95_uv
       << "/" << quality_max_uv << endl;
  file << "quality_uv_mean=" << quality_mean_uv << endl;
  file << "quality_uv_stddev=" << quality_std_uv << endl;
  file << "quality_uv_poor_ratio_lt_0.30="
       << (total_elements > 0
               ? (100.0 * static_cast<double>(poor_quality_uv) /
                  static_cast<double>(total_elements))
               : 0.0)
       << endl;
  file << "quality_uv_good_ratio_ge_0.60="
       << (total_elements > 0
               ? (100.0 * static_cast<double>(good_quality_ge_0_60_uv) /
                  static_cast<double>(total_elements))
               : 0.0)
       << endl;
  file << "quality_uv_excellent_ratio_ge_0.80="
       << (total_elements > 0
               ? (100.0 * static_cast<double>(excellent_quality_uv) /
                  static_cast<double>(total_elements))
               : 0.0)
       << endl;
  file << endl;

  file << "area[min/mean/max]=" << area_min << "/" << area_mean << "/"
       << area_max << endl;
  file << "area_stddev=" << area_std << endl;
  file << endl;

  file << "topology[unique_edges/boundary_edges/nonmanifold_edges]="
       << edge_use_count.size() << "/" << boundary_edges << "/"
       << nonmanifold_edges << endl;
  file << endl;

  file << "quality_histogram_counts_3d" << endl;
  for (size_t i = 0; i < histogram_3d.size(); ++i) {
    const double a = static_cast<double>(i) / 10.0;
    const double b = static_cast<double>(i + 1) / 10.0;
    file << "[" << a << "," << b << "]=" << histogram_3d[i] << endl;
  }
  file << endl;

  file << "quality_histogram_percent_3d" << endl;
  for (size_t i = 0; i < histogram_3d.size(); ++i) {
    const double a = static_cast<double>(i) / 10.0;
    const double b = static_cast<double>(i + 1) / 10.0;
    const double pct =
        total_elements > 0
            ? (100.0 * static_cast<double>(histogram_3d[i]) /
               static_cast<double>(total_elements))
            : 0.0;
    file << "[" << a << "," << b << "]=" << pct << endl;
  }
  file << endl;

  file << "quality_histogram_counts_uv" << endl;
  for (size_t i = 0; i < histogram_uv.size(); ++i) {
    const double a = static_cast<double>(i) / 10.0;
    const double b = static_cast<double>(i + 1) / 10.0;
    file << "[" << a << "," << b << "]=" << histogram_uv[i] << endl;
  }
  file << endl;

  file << "quality_histogram_percent_uv" << endl;
  for (size_t i = 0; i < histogram_uv.size(); ++i) {
    const double a = static_cast<double>(i) / 10.0;
    const double b = static_cast<double>(i + 1) / 10.0;
    const double pct =
        total_elements > 0
            ? (100.0 * static_cast<double>(histogram_uv[i]) /
               static_cast<double>(total_elements))
            : 0.0;
    file << "[" << a << "," << b << "]=" << pct << endl;
  }

  file.flush();
  file.close();

  stringstream csvName;
  csvName << NAME_MODEL;
  csvName << "_n.process_";
  csvName << NUMBER_PROCESS;
  csvName << "_metrics_rank_";
  csvName << rank;
  csvName << ".csv";

  EnsureOutputParentDirectories(csvName.str());
  bool csv_exists = false;
  bool csv_has_current_header = false;
  {
    std::ifstream in(csvName.str().c_str());
    csv_exists = in.good();
    if (csv_exists) {
      std::string header_line;
      std::getline(in, header_line);
      csv_has_current_header =
          (header_line.find("error_global_surface_curvature") != std::string::npos) &&
          (header_line.find("quality_uv_min") != std::string::npos) &&
          (header_line.find("angle_quality_mean_3d") != std::string::npos);
    }
  }

  const bool rewrite_csv = csv_exists && !csv_has_current_header;
  std::ofstream csv(
      csvName.str().c_str(),
      rewrite_csv ? std::ios::out : (std::ios::out | std::ios::app));
  if (!csv_exists || rewrite_csv) {
    csv << "step,rank,nodes,elements,error,error_prev,error_global_surface_curvature,error_global_surface_curvature_prev,error_improvement_abs,"
           "error_improvement_pct,error_normalized,error_total_reduction_pct,elements_prev,elements_delta,element_growth_factor,"
           "error_improvement_per_element,acceptance_score,gauss_nodes,mean_nodes,gauss_ratio,mean_ratio,error_floor,min_error_drop_abs,"
           "min_error_drop_per_element,retry_attempt,adaptation_relaxation,"
           "retry_enabled,local_budget_mode,target_element_growth,target_elements_step,actual_elements_step,target_growth_step,actual_growth_step,actual_element_growth,step_efficiency,local_progress_coverage,improved_top_patches,top_patch_candidate_count,generation_control_mode,consistency_regularization_active,patch_budget_dispersion,patch_h_dispersion,actual_patch_element_dispersion,curve_balance_dispersion,patch_consistency_penalty,budget_driver_mode,"
           "adaptation_max_delta,adaptive_retry_count,adaptive_retry_shrink,max_retry_element_growth_factor,max_retry_element_growth_abs,"
           "element_budget_abs,element_budget_growth_factor,element_budget_soft_limit,candidate_elements_before_budget_retry,budget_retry_count,budget_limited,budget_final_status,"
           "step_mode,budget_pressure,eligible_patch_count,eligible_patch_elements_sum,eligible_patch_mean_factor,eligible_patch_mean_h_scale,"
           "step_purpose,underresolved_transition_fraction_real,transition_imbalance_max_real,"
           "stop_reason,quality_min,quality_p05,quality_p50,"
           "quality_p95,quality_max,quality_mean,quality_std,poor_ratio_lt_0_30,"
           "q_ge_0_60,excellent_ratio_ge_0_80,"
           "angle_quality_mean_3d,edge_ratio_mean_3d,min_angle_mean_3d,min_angle_p05_3d,max_angle_mean_3d,max_angle_p95_3d,"
           "quality_uv_min,quality_uv_p05,quality_uv_p50,quality_uv_p95,quality_uv_max,quality_uv_mean,quality_uv_std,quality_uv_poor_ratio_lt_0_30,quality_uv_q_ge_0_60,quality_uv_excellent_ratio_ge_0_80,"
           "area_min,area_mean,area_max,area_std,"
           "unique_edges,boundary_edges,nonmanifold_edges,laplacian_iterations,"
           "laplacian_factor"
        << endl;
  }
  csv << output_step << "," << rank << "," << total_nodes << "," << total_elements
      << "," << error_current << "," << error_prev << "," << error_current << ","
      << error_prev << "," << error_delta_abs
      << "," << error_delta_pct << "," << error_normalized << ","
      << error_total_reduction_pct << "," << elements_prev << ","
      << elements_delta << "," << element_growth_factor << ","
      << error_drop_per_element << "," << acceptance_score << ","
      << gauss_nodes << "," << mean_nodes << ","
      << ((gauss_nodes + mean_nodes) > 0ULL
              ? static_cast<double>(gauss_nodes) /
                    static_cast<double>(gauss_nodes + mean_nodes)
              : 0.0)
      << ","
      << ((gauss_nodes + mean_nodes) > 0ULL
              ? static_cast<double>(mean_nodes) /
                    static_cast<double>(gauss_nodes + mean_nodes)
              : 0.0)
      << ","
      << ERROR_FLOOR << "," << MIN_ERROR_DROP_ABS << ","
      << MIN_ERROR_DROP_PER_ELEMENT << "," << retry_attempt << ","
      << retry_enabled << "," << local_budget_mode << ","
      << target_element_growth << "," << target_elements_step << ","
      << actual_elements_step << "," << target_growth_step << ","
      << actual_growth_step << "," << actual_element_growth << ","
      << step_efficiency << "," << local_progress_coverage << ","
      << improved_top_patches << "," << top_patch_candidate_count << ","
      << generation_control_mode << ","
      << consistency_regularization_active << ","
      << patch_budget_dispersion << ","
      << patch_h_dispersion << ","
      << actual_patch_element_dispersion << ","
      << curve_balance_dispersion << ","
      << patch_consistency_penalty << ","
      << budget_driver_mode << ","
      << applied_relaxation << "," << applied_max_delta << ","
      << ADAPTIVE_RETRY_COUNT << "," << ADAPTIVE_RETRY_SHRINK << ","
      << MAX_RETRY_ELEMENT_GROWTH_FACTOR << ","
      << MAX_RETRY_ELEMENT_GROWTH_ABS << ","
      << element_budget_abs << "," << element_budget_growth_factor_limit << ","
      << element_budget_soft_limit << ","
      << candidate_elements_before_budget_retry << "," << budget_retry_count
      << "," << budget_limited << "," << budget_final_status << ","
      << step_mode << "," << budget_pressure << "," << eligible_patch_count
      << "," << eligible_patch_elements_sum << ","
      << eligible_patch_mean_factor << "," << eligible_patch_mean_h_scale << ","
      << (output_step >= 0 && output_step < static_cast<int>(step_purpose_step_.size())
              ? step_purpose_step_[static_cast<size_t>(output_step)]
              : current_step_purpose_)
      << ","
      << (output_step >= 0 &&
                  output_step < static_cast<int>(underresolved_transition_fraction_step_.size())
              ? underresolved_transition_fraction_step_[static_cast<size_t>(output_step)]
              : current_underresolved_transition_fraction_real_)
      << ","
      << (output_step >= 0 && output_step < static_cast<int>(transition_imbalance_max_step_.size())
              ? transition_imbalance_max_step_[static_cast<size_t>(output_step)]
              : current_transition_imbalance_max_real_)
      << "," << stop_reason_ << "," << quality_min << "," << quality_p05
      << "," << quality_p50 << "," << quality_p95 << "," << quality_max << ","
      << quality_mean << "," << quality_std << ","
      << (total_elements > 0
              ? (100.0 * static_cast<double>(poor_quality) /
                 static_cast<double>(total_elements))
              : 0.0)
      << ","
      << (total_elements > 0
              ? (100.0 * static_cast<double>(good_quality_ge_0_60) /
                 static_cast<double>(total_elements))
              : 0.0)
      << ","
      << (total_elements > 0
              ? (100.0 * static_cast<double>(excellent_quality) /
                 static_cast<double>(total_elements))
              : 0.0)
      << "," << angle_quality_mean_3d << "," << edge_ratio_mean_3d << ","
      << min_angle_mean_3d << "," << min_angle_p05_3d << ","
      << max_angle_mean_3d << "," << max_angle_p95_3d
      << "," << quality_min_uv << "," << quality_p05_uv << ","
      << quality_p50_uv << "," << quality_p95_uv << "," << quality_max_uv
      << "," << quality_mean_uv << "," << quality_std_uv << ","
      << (total_elements > 0
              ? (100.0 * static_cast<double>(poor_quality_uv) /
                 static_cast<double>(total_elements))
              : 0.0)
      << ","
      << (total_elements > 0
              ? (100.0 * static_cast<double>(good_quality_ge_0_60_uv) /
                 static_cast<double>(total_elements))
              : 0.0)
      << ","
      << (total_elements > 0
              ? (100.0 * static_cast<double>(excellent_quality_uv) /
                 static_cast<double>(total_elements))
              : 0.0)
      << "," << area_min << "," << area_mean << "," << area_max << ","
      << area_std << "," << edge_use_count.size() << "," << boundary_edges
      << "," << nonmanifold_edges << ","
      << static_cast<unsigned int>(SMOOTHING_LAPLACIAN_NUMBER) << ","
      << SMOOTHING_LAPLACIAN_FACTOR << endl;
  csv.flush();
  csv.close();
}

void GeneratorAdaptive::WriteCompatibilitySummary(const Geometry* geometry) const {
  if (geometry == nullptr || NAME_MODEL.empty()) {
    return;
  }

  std::unordered_map<unsigned long, std::vector<const CurveAdaptive*>> groups;
  unsigned int multi_incident_features = 0U;
  for (unsigned int curve_index = 0; curve_index < geometry->GetNumberCurves();
       ++curve_index) {
    const CurveAdaptive* curve = geometry->GetCurve(curve_index);
    if (curve == nullptr) {
      continue;
    }
    groups[curve->GetId()].push_back(curve);
    if (curve->GetNumBerPatches() > 1U) {
      ++multi_incident_features;
    }
  }

  double compatibility_error_mean = 0.0;
  double compatibility_error_max = 0.0;
  unsigned int compared_groups = 0U;
  unsigned int desynchronized_groups = 0U;
  if (ENABLE_SHARED_CURVE_SYNC == 0) {
    for (const auto& entry : groups) {
      if (entry.second.size() <= 1U) {
        continue;
      }
      const double group_error = ComputeCompatibilityErrorForCurveGroup(entry.second);
      compatibility_error_mean += group_error;
      compatibility_error_max = std::max(compatibility_error_max, group_error);
      ++compared_groups;
      if (group_error > 0.0) {
        ++desynchronized_groups;
      }
    }
    if (compared_groups > 0U) {
      compatibility_error_mean /= static_cast<double>(compared_groups);
    }
  }

  std::ostringstream name;
  name << NAME_MODEL << "_compatibility_summary.csv";
  EnsureOutputParentDirectories(name.str());
  const bool exists = std::ifstream(name.str().c_str()).good();
  std::ofstream file(name.str().c_str(), std::ios::app);
  if (!exists) {
    file << "model,shared_curve_sync_enabled,hybrid_reconstruction_enabled,steps_recorded,"
            "curve_objects,curve_groups,multi_incident_features,compared_groups,"
            "desynchronized_groups,compat_error_mean,compat_error_max"
         << std::endl;
  }
  file << NAME_MODEL << "," << ENABLE_SHARED_CURVE_SYNC << ","
       << ENABLE_HYBRID_RECONSTRUCTION << "," << error_step_.size() << ","
       << geometry->GetNumberCurves() << "," << groups.size() << ","
       << multi_incident_features << "," << compared_groups << ","
       << desynchronized_groups << "," << compatibility_error_mean << ","
       << compatibility_error_max << std::endl;
}

void GeneratorAdaptive::WriteRuntimeSummary(const Timer* timer) const {
  if (timer == nullptr || NAME_MODEL.empty() || WRITE_RUNTIME_SUMMARY == 0) {
    return;
  }

  const auto all_times = timer->GetTimerParallel();
  const unsigned int measured_steps =
      error_step_.size() > 1U ? static_cast<unsigned int>(error_step_.size() - 1U)
                              : 0U;

  std::ostringstream name;
  name << NAME_MODEL << "_runtime_summary.csv";
  EnsureOutputParentDirectories(name.str());
  const bool exists = std::ifstream(name.str().c_str()).good();
  std::ofstream file(name.str().c_str(), std::ios::app);
  if (!exists) {
    file << "model,shared_curve_sync_enabled,hybrid_reconstruction_enabled,"
            "measured_steps,stage_id,stage_name,total_seconds,per_iteration_ms"
         << std::endl;
  }

  for (int type = 0; type <= 11; ++type) {
    double total_seconds = 0.0;
    for (const auto& rank_times : all_times) {
      for (const auto& thread_times : rank_times) {
        if (type < static_cast<int>(thread_times.size())) {
          total_seconds = std::max(total_seconds, thread_times[static_cast<size_t>(type)]);
        }
      }
    }
    if (type == 10) {
      double input_read_seconds = 0.0;
      double save_output_seconds = 0.0;
      for (const auto& rank_times : all_times) {
        for (const auto& thread_times : rank_times) {
          if (5 < static_cast<int>(thread_times.size())) {
            input_read_seconds =
                std::max(input_read_seconds, thread_times[5]);
          }
          if (11 < static_cast<int>(thread_times.size())) {
            save_output_seconds =
                std::max(save_output_seconds, thread_times[11]);
          }
        }
      }
      total_seconds =
          std::max(0.0, total_seconds - input_read_seconds - save_output_seconds);
    }
    const double per_iteration_ms =
        measured_steps > 0U
            ? (1000.0 * total_seconds / static_cast<double>(measured_steps))
            : 0.0;
    file << NAME_MODEL << "," << ENABLE_SHARED_CURVE_SYNC << ","
         << ENABLE_HYBRID_RECONSTRUCTION << "," << measured_steps << ","
         << type << "," << TimerTypeName(type) << "," << total_seconds << ","
         << per_iteration_ms << std::endl;
  }
}

void GeneratorAdaptive::SaveMesh(std::unique_ptr<MeshAdaptive> malha, int output_step) {
  save_mesh_.emplace_back(output_step, std::move(malha));
  MeshAdaptive* saved_mesh = save_mesh_.back().second.get();
  if (!saved_mesh) {
    return;
  }

#if USE_MPI
  if (WRITE_MESH == std::string("q")) {
    PrintElments(saved_mesh, output_step, this->error_step_, RANK_MPI);
    WriteQualityMesh(saved_mesh, output_step, this->error_step_, RANK_MPI);
  } else if (WRITE_MESH == std::string("m")) {
    PrintElments(saved_mesh, output_step, this->error_step_, RANK_MPI);
    WriteMesh(saved_mesh, output_step, this->error_step_, RANK_MPI);
  } else if (WRITE_MESH == std::string("h")) {
    PrintElments(saved_mesh, output_step, this->error_step_, RANK_MPI);
    WriteMesh(saved_mesh, output_step, this->error_step_, RANK_MPI);
    WriteQualityMesh(saved_mesh, output_step, this->error_step_, RANK_MPI);
  }
#else
  if (WRITE_MESH == std::string("q")) {
    PrintElments(saved_mesh, output_step, this->error_step_);
    WriteQualityMesh(saved_mesh, output_step, this->error_step_);
  } else if (WRITE_MESH == std::string("m")) {
    PrintElments(saved_mesh, output_step, this->error_step_);
    WriteMesh(saved_mesh, output_step, this->error_step_);
  } else if (WRITE_MESH == std::string("h")) {
    PrintElments(saved_mesh, output_step, this->error_step_);
    WriteMesh(saved_mesh, output_step, this->error_step_);
    WriteQualityMesh(saved_mesh, output_step, this->error_step_);
  }
#endif
}

void GeneratorAdaptive::SaveErrorMesh(MeshAdaptive* malha, int output_step) {
  save_error_mesh_.push_back(make_pair(output_step, malha));
}

void escreveElementos(int step_, SubMesh* sub, int i) {
  stringstream nome;
  nome << step_;
  nome << "submalha-";
  nome << i;
  nome << ".log";

  ofstream arq(nome.str().c_str());

  for (unsigned int k = 0; k < sub->GetNumberElements(); ++k) {
    ElementAdaptive* elem = sub->GetElement(k);

    NodeAdaptive n1(elem->GetNoh(1));
    NodeAdaptive n2(elem->GetNoh(2));
    NodeAdaptive n3(elem->GetNoh(3));

    tuple<double, double> t1 = ((TriangleAdaptive*)elem)->GetParametersN1();
    tuple<double, double> t2 = ((TriangleAdaptive*)elem)->GetParametersN2();
    tuple<double, double> t3 = ((TriangleAdaptive*)elem)->GetParametersN3();

    arq << "T-" << elem->GetId() << ":\n\t"
        << "área = " << elem->GetArea() << ";\n\t"
        << "normal = " << elem->GetVectorNormal().GetX() << ", "
        << elem->GetVectorNormal().GetY() << ", "
        << elem->GetVectorNormal().GetZ() << "\n\t"
        << "n-" << n1.GetId() << "( " << get<0>(t1) << " , " << get<1>(t1)
        << ") ângulo = " << elem->GetAngle(n1) << ";\n\t"
        << "n-" << n2.GetId() << "( " << get<0>(t2) << " , " << get<1>(t2)
        << ") ângulo = " << elem->GetAngle(n2) << ";\n\t"
        << "n-" << n3.GetId() << "( " << get<0>(t3) << " , " << get<1>(t3)
        << ") ângulo = " << elem->GetAngle(n3) << endl;
  }

  arq.flush();

  arq.close();

  cout << "escreveu o arquivo com os elementos da submalha " << i
       << " para o step_ " << step_ << endl;
}

void GeneratorAdaptive::GeneratorInitialMesh(Geometry* geometry,
                                             MeshAdaptive* mesh, Timer* timer,
                                             int size_thread, int size_patch) {
  (void)timer;
  (void)size_thread;
#if USE_OPENMP
#pragma omp parallel num_threads(size_thread) shared(mesh, geometry, size_patch)
  {
    Int id = communicator_->threadId();

    if (!this->id_managers_[id]) {
      this->id_managers_[id] = this->MakeIdManagerOmp(communicator_, id);
    }
#if USE_MPI
    timer->InitTimerParallel(RANK_MPI, id, 2);  // Malha inicial
#else
    timer->InitTimerParallel(0, id, 2);  // Malha inicial
#endif  // USE_MPI

    // 1. Gera a mesh inicial (ordem estável por fronteira; um único executor
    //    evita corrida em curvas partilhadas quando NUM_THREADS > 1).
#pragma omp single
    {
      std::vector<int> patch_order =
          InitialMeshPatchProcessOrder(geometry, size_patch);
      Performer::IdManager* id_mgr = this->id_managers_[id].get();
      for (int k = 0; k < size_patch; ++k) {
        const int i = patch_order[static_cast<size_t>(k)];
        PatchCoons* patch = static_cast<PatchCoons*>(geometry->GetPatch(i));
        SubMesh* sub_mesh = this->GeneratorInitialMeshOmp(patch, id_mgr);
        mesh->InsertSubMeshAdaptiveByPosition(sub_mesh, i);
      }
    }

#if USE_MPI
    timer->EndTimerParallel(RANK_MPI, id, 2);  // Malha inicial
#else
    timer->EndTimerParallel(0, id, 2);   // Malha inicial
#endif  // USE_MPI
  }
#else
  {
    std::vector<int> patch_order =
        InitialMeshPatchProcessOrder(geometry, size_patch);
    for (int k = 0; k < size_patch; ++k) {
      const int i = patch_order[static_cast<size_t>(k)];
      PatchCoons* patch = static_cast<PatchCoons*>(geometry->GetPatch(i));
      SubMesh* sub_mesh = this->InitialMesh(static_cast<PatchCoons*>(patch),
                                            this->id_managers_[0].get());
      mesh->InsertSubMeshAdaptiveByPosition(sub_mesh, i);
    }
  }
#endif  // USE_OPENMP
}

void GeneratorAdaptive::PrintElments(MeshAdaptive* malha, int output_step,
                                     [[maybe_unused]] vector<double> step_error,
                                     int rank) {
  [[maybe_unused]] unsigned long int Nv = 0, Nt = 0;

  for (unsigned int i = 0; i < malha->GetNumberSubMeshesAdaptive(); i++) {
    SubMesh* sub = malha->GetSubMeshAdaptiveByPosition(i);

    Nv += sub->GetNumberNos();
    Nt += sub->GetNumberElements();
  }

  cout << "#elementos_" << NAME_MODEL << "_n.process_" << NUMBER_PROCESS
       << "_passo_" << output_step << "_rank_" << rank << " = " << Nt << endl;
}
