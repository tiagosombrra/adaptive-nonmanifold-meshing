#include "../../include/adapter/adapter.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <vector>

extern double TOLERANCE;
extern double TOLERANCE_AFT;
extern double TOLERANCE_CURVATURE;
extern double RATIO_AFT;
extern double SMOOTHING_LAPLACIAN_NUMBER;
extern double SMOOTHING_LAPLACIAN_FACTOR;
extern double DISCRETIZATION_CURVE_FACTOR;
extern double DISCRETIZATION_CURVE_FACTOR_INTERNAL;
extern std::string NAME_MODEL;
extern std::string ADAPTIVE_MODE;
extern double ADAPTIVE_INTENSITY;
extern double ADAPTIVE_QUALITY_PRIORITY;
extern int WRITE_ADAPTATION_DEBUG;
extern int PATCH_QUADTREE_DEPTH_STEP1;
extern int PATCH_QUADTREE_DEPTH_STEPN;
extern int ADAPTER_RESOLVE_NEGATIVE_QUADTREE_DEPTH;

namespace {
unsigned long CheckedLegacyId(const Data::ULInt id) {
  if (id > static_cast<Data::ULInt>(
               std::numeric_limits<unsigned long>::max())) {
    throw std::overflow_error("generated identifier exceeds platform limit");
  }
  return static_cast<unsigned long>(id);
}

double ClampValue(const double value, const double lower, const double upper) {
  return std::max(lower, std::min(value, upper));
}

// Profundidade efetiva na quadtree do AFT: valores < 0 do gerador significam
// "não definido"; opcionalmente mapeamos para PATCH_QUADTREE_DEPTH_* (config).
int ResolveQuadtreeDepthCap(int max_quadtree_depth, int adaptive_step) {
  if (max_quadtree_depth >= 0) {
    return max_quadtree_depth;
  }
  if (ADAPTER_RESOLVE_NEGATIVE_QUADTREE_DEPTH == 0) {
    return -1;
  }
  const int d = (adaptive_step <= 1) ? PATCH_QUADTREE_DEPTH_STEP1
                                     : PATCH_QUADTREE_DEPTH_STEPN;
  return std::max(0, std::min(32, d));
}

struct FaceMetricStats {
  unsigned long long count = 0;
  double quality_min = 0.0;
  double quality_mean = 0.0;
  double poor_ratio = 0.0;
  double h_min = 0.0;
  double h_mean = 0.0;
  double h_max = 0.0;
};

struct RegionQualityStatsLocal {
  unsigned long long element_count = 0ULL;
  double quality_min = 0.0;
  double quality_mean = 0.0;
  double poor_ratio = 0.0;
  double good_ratio_ge_0_60 = 0.0;
  double area_min = 0.0;
  double area_mean = 0.0;
  double area_max = 0.0;
  double aspect_ratio_mean = 0.0;
};

struct RegionalPatchStats {
  RegionQualityStatsLocal boundary;
  RegionQualityStatsLocal transition;
  RegionQualityStatsLocal interior;
  double curve_interior_coupling = 1.0;
  double good_ratio_ge_0_60 = 0.0;
};

struct ParamVertexKey {
  long long x;
  long long y;

  bool operator==(const ParamVertexKey& other) const {
    return x == other.x && y == other.y;
  }
};

struct ParamVertexKeyHasher {
  std::size_t operator()(const ParamVertexKey& key) const {
    return std::hash<long long>()((key.x << 1) ^ key.y);
  }
};

struct EdgeKey {
  ParamVertexKey a;
  ParamVertexKey b;

  bool operator==(const EdgeKey& other) const {
    return a == other.a && b == other.b;
  }
};

struct EdgeKeyHasher {
  std::size_t operator()(const EdgeKey& key) const {
    const std::size_t ha = ParamVertexKeyHasher()(key.a);
    const std::size_t hb = ParamVertexKeyHasher()(key.b);
    return ha ^ (hb << 1);
  }
};

ParamVertexKey MakeVertexKey(Vertex* vertex) {
  return ParamVertexKey{static_cast<long long>(std::llround(vertex->getX() * 1e9)),
                        static_cast<long long>(std::llround(vertex->getY() * 1e9))};
}

EdgeKey MakeEdgeKey(Vertex* v1, Vertex* v2) {
  const ParamVertexKey a = MakeVertexKey(v1);
  const ParamVertexKey b = MakeVertexKey(v2);
  if ((a.x < b.x) || (a.x == b.x && a.y <= b.y)) {
    return EdgeKey{a, b};
  }
  return EdgeKey{b, a};
}

double FaceBoundaryDistance(Face* face) {
  const double cx =
      (face->getV1()->getX() + face->getV2()->getX() + face->getV3()->getX()) /
      3.0;
  const double cy =
      (face->getV1()->getY() + face->getV2()->getY() + face->getV3()->getY()) /
      3.0;
  return std::min(std::min(cx, 1.0 - cx), std::min(cy, 1.0 - cy));
}

double FaceAspectRatio(Face* face) {
  const double h1 = face->getV1()->distance(face->getV2());
  const double h2 = face->getV2()->distance(face->getV3());
  const double h3 = face->getV3()->distance(face->getV1());
  const double hmin = std::min(h1, std::min(h2, h3));
  const double hmax = std::max(h1, std::max(h2, h3));
  if (hmin <= TOLERANCE_AFT) {
    return hmax > 0.0 ? 1.0e6 : 0.0;
  }
  return hmax / hmin;
}

void FinalizeRegionStats(RegionQualityStatsLocal& stats) {
  if (stats.element_count == 0ULL) {
    stats.quality_min = 0.0;
    stats.area_min = 0.0;
    return;
  }
  const double total = static_cast<double>(stats.element_count);
  stats.quality_mean /= total;
  stats.poor_ratio /= total;
  stats.good_ratio_ge_0_60 /= total;
  stats.area_mean /= total;
  stats.aspect_ratio_mean /= total;
}

void AccumulateRegionStats(RegionQualityStatsLocal& stats, Face* face) {
  const double quality = face->quality();
  const double area = std::fabs(face->orientedSurface());
  const double aspect = FaceAspectRatio(face);
  stats.quality_min =
      (stats.element_count == 0ULL) ? quality : std::min(stats.quality_min, quality);
  stats.quality_mean += quality;
  if (quality < 0.30) {
    stats.poor_ratio += 1.0;
  }
  if (quality >= 0.60) {
    stats.good_ratio_ge_0_60 += 1.0;
  }
  stats.area_min =
      (stats.element_count == 0ULL) ? area : std::min(stats.area_min, area);
  stats.area_mean += area;
  stats.area_max = std::max(stats.area_max, area);
  stats.aspect_ratio_mean += aspect;
  ++stats.element_count;
}

RegionalPatchStats ComputeRegionalPatchStats(const FaceList& faces,
                                             const double target_patch_h) {
  RegionalPatchStats stats;
  if (faces.empty()) {
    return stats;
  }

  std::unordered_map<EdgeKey, unsigned int, EdgeKeyHasher> edge_use_count;
  for (FaceList::const_iterator iter = faces.begin(); iter != faces.end(); ++iter) {
    Face* face = *iter;
    Vertex* verts[3] = {face->getV1(), face->getV2(), face->getV3()};
    for (int e = 0; e < 3; ++e) {
      ++edge_use_count[MakeEdgeKey(verts[e], verts[(e + 1) % 3])];
    }
  }

  const double transition_band_width = std::max(0.08, std::min(
      0.32, (target_patch_h > 0.0) ? target_patch_h * 1.75 : 0.14));
  for (FaceList::const_iterator iter = faces.begin(); iter != faces.end(); ++iter) {
    Face* face = *iter;
    Vertex* verts[3] = {face->getV1(), face->getV2(), face->getV3()};
    unsigned int boundary_edges = 0U;
    for (int e = 0; e < 3; ++e) {
      if (edge_use_count[MakeEdgeKey(verts[e], verts[(e + 1) % 3])] == 1U) {
        ++boundary_edges;
      }
    }

    if (boundary_edges > 0U) {
      AccumulateRegionStats(stats.boundary, face);
      continue;
    }

    const double boundary_distance = FaceBoundaryDistance(face);
    const double aspect = FaceAspectRatio(face);
    const bool transition_band_face =
        boundary_distance <= transition_band_width || aspect > 1.8;
    if (transition_band_face) {
      AccumulateRegionStats(stats.transition, face);
    } else {
      AccumulateRegionStats(stats.interior, face);
    }
  }

  FinalizeRegionStats(stats.boundary);
  FinalizeRegionStats(stats.transition);
  FinalizeRegionStats(stats.interior);

  const double total =
      static_cast<double>(stats.boundary.element_count + stats.transition.element_count +
                          stats.interior.element_count);
  if (total > 0.0) {
    const double good_count =
        stats.boundary.good_ratio_ge_0_60 * static_cast<double>(stats.boundary.element_count) +
        stats.transition.good_ratio_ge_0_60 * static_cast<double>(stats.transition.element_count) +
        stats.interior.good_ratio_ge_0_60 * static_cast<double>(stats.interior.element_count);
    stats.good_ratio_ge_0_60 = good_count / total;
  }

  const double transition_quality = stats.transition.good_ratio_ge_0_60;
  const double interior_quality = stats.interior.good_ratio_ge_0_60;
  const double transition_aspect_penalty = ClampValue(
      std::max(0.0, stats.transition.aspect_ratio_mean - 1.8) / 2.0, 0.0, 1.0);
  stats.curve_interior_coupling = ClampValue(
      0.50 * transition_quality + 0.35 * interior_quality +
          0.15 * (1.0 - transition_aspect_penalty),
      0.0, 1.0);
  return stats;
}

FaceMetricStats ComputeFaceMetricStats(const FaceList& faces) {
  FaceMetricStats stats;
  if (faces.empty()) {
    return stats;
  }

  stats.count = static_cast<unsigned long long>(faces.size());
  stats.quality_min = std::numeric_limits<double>::max();
  stats.h_min = std::numeric_limits<double>::max();

  unsigned long long poor_count = 0;
  for (FaceList::const_iterator iter = faces.begin(); iter != faces.end(); ++iter) {
    const double quality = (*iter)->quality();
    const double h_value = (*iter)->h;
    stats.quality_min = std::min(stats.quality_min, quality);
    stats.quality_mean += quality;
    stats.h_min = std::min(stats.h_min, h_value);
    stats.h_max = std::max(stats.h_max, h_value);
    stats.h_mean += h_value;
    if (quality < 0.35) {
      ++poor_count;
    }
  }

  stats.quality_mean /= static_cast<double>(stats.count);
  stats.h_mean /= static_cast<double>(stats.count);
  stats.poor_ratio = static_cast<double>(poor_count) / static_cast<double>(stats.count);
  if (stats.quality_min == std::numeric_limits<double>::max()) {
    stats.quality_min = 0.0;
  }
  if (stats.h_min == std::numeric_limits<double>::max()) {
    stats.h_min = 0.0;
  }
  return stats;
}

void AppendDomainDebugLog(int adaptive_step, int retry_index, int patch_index,
                          double factor_disc_global,
                          const FaceMetricStats& raw_stats,
                          const FaceMetricStats& smoothed_stats,
                          const FaceMetricStats& generated_stats,
                          const Par2DJMesh::AFT::AdvancingFrontRunStats& aft_stats,
                          const RegionalPatchStats& regional_stats,
                          int local_regen_attempt = 0,
                          const std::string& generation_status = "ok",
                          double degeneracy_score = 0.0,
                          double transition_floor = 0.0,
                          int transition_underresolved = 0,
                          double interior_transition_imbalance = 0.0,
                          double curve_domain_balance = 1.0) {
  if (!WRITE_ADAPTATION_DEBUG || NAME_MODEL.empty()) {
    return;
  }

  std::ostringstream name;
  name << NAME_MODEL << "_domain_adaptation.csv";
  const bool exists = std::ifstream(name.str().c_str()).good();
  std::ofstream file(name.str().c_str(), std::ios::app);
  if (!exists) {
    file << "step,retry,patch_id,factor_disc_global,old_faces,raw_h_min,raw_h_mean,raw_h_max,"
            "raw_quality_min,raw_quality_mean,raw_poor_ratio,smoothed_h_min,smoothed_h_mean,"
            "smoothed_h_max,smoothed_quality_min,smoothed_quality_mean,smoothed_poor_ratio,"
            "generated_faces,generated_quality_min,generated_quality_mean,generated_poor_ratio,"
            "quadtree_leaves,template_count,template_low_score_count,template_score_min,"
            "template_score_mean,low_quality_subdivide_hits,quality_pre_aft_min,quality_pre_aft_mean,"
            "quality_post_aft_min,quality_post_aft_mean,quality_post_smoothing_min,quality_post_smoothing_mean,"
            "poor_ratio_pre_aft,poor_ratio_post_aft,poor_ratio_post_smoothing,"
            "degenerate_pre_aft_count,degenerate_post_aft_count,degenerate_post_smoothing_count,"
            "boundary_element_ratio,transition_element_ratio,internal_element_ratio,"
            "postprocess_vertices_considered,postprocess_vertices_moved,"
            "local_regen_attempt,generation_status,degeneracy_score,"
            "transition_floor,transition_underresolved,interior_transition_imbalance,"
            "curve_domain_balance,curve_interior_coupling,good_ratio_ge_0_60,"
            "boundary_elements,boundary_quality_min,boundary_quality_mean,boundary_q_ge_0_60,boundary_area_mean,boundary_aspect_mean,"
            "transition_elements,transition_quality_min,transition_quality_mean,transition_q_ge_0_60,transition_area_mean,transition_aspect_mean,"
            "interior_elements,interior_quality_min,interior_quality_mean,interior_q_ge_0_60,interior_area_mean,interior_aspect_mean"
         << std::endl;
  }

  file << adaptive_step << "," << retry_index << "," << patch_index << ","
       << factor_disc_global << "," << raw_stats.count << "," << raw_stats.h_min << ","
       << raw_stats.h_mean << "," << raw_stats.h_max << "," << raw_stats.quality_min << ","
       << raw_stats.quality_mean << "," << raw_stats.poor_ratio << ","
       << smoothed_stats.h_min << "," << smoothed_stats.h_mean << ","
       << smoothed_stats.h_max << "," << smoothed_stats.quality_min << ","
       << smoothed_stats.quality_mean << "," << smoothed_stats.poor_ratio << ","
       << generated_stats.count << "," << generated_stats.quality_min << ","
       << generated_stats.quality_mean << "," << generated_stats.poor_ratio << ","
       << aft_stats.quadtree_leaf_count << "," << aft_stats.quadtree_template_count << ","
       << aft_stats.quadtree_low_score_count << "," << aft_stats.quadtree_template_score_min
       << "," << aft_stats.quadtree_template_score_mean << ","
       << aft_stats.quadtree_low_quality_subdivide_hits << ","
       << aft_stats.quality_pre_aft_min << "," << aft_stats.quality_pre_aft_mean << ","
       << aft_stats.quality_post_aft_min << "," << aft_stats.quality_post_aft_mean << ","
       << aft_stats.quality_post_smoothing_min << "," << aft_stats.quality_post_smoothing_mean << ","
       << aft_stats.poor_ratio_pre_aft << "," << aft_stats.poor_ratio_post_aft << ","
       << aft_stats.poor_ratio_post_smoothing << ","
       << aft_stats.degenerate_pre_aft_count << "," << aft_stats.degenerate_post_aft_count
       << "," << aft_stats.degenerate_post_smoothing_count << ","
       << aft_stats.boundary_element_ratio << "," << aft_stats.transition_element_ratio
       << "," << aft_stats.internal_element_ratio << ","
       << aft_stats.postprocess_vertices_considered << ","
       << aft_stats.postprocess_vertices_moved << ","
       << local_regen_attempt << "," << generation_status << ","
       << degeneracy_score << "," << transition_floor << ","
       << transition_underresolved << "," << interior_transition_imbalance << ","
       << curve_domain_balance << "," << regional_stats.curve_interior_coupling << ","
       << regional_stats.good_ratio_ge_0_60 << ","
       << regional_stats.boundary.element_count << ","
       << regional_stats.boundary.quality_min << ","
       << regional_stats.boundary.quality_mean << ","
       << regional_stats.boundary.good_ratio_ge_0_60 << ","
       << regional_stats.boundary.area_mean << ","
       << regional_stats.boundary.aspect_ratio_mean << ","
       << regional_stats.transition.element_count << ","
       << regional_stats.transition.quality_min << ","
       << regional_stats.transition.quality_mean << ","
       << regional_stats.transition.good_ratio_ge_0_60 << ","
       << regional_stats.transition.area_mean << ","
       << regional_stats.transition.aspect_ratio_mean << ","
       << regional_stats.interior.element_count << ","
       << regional_stats.interior.quality_min << ","
       << regional_stats.interior.quality_mean << ","
       << regional_stats.interior.good_ratio_ge_0_60 << ","
       << regional_stats.interior.area_mean << ","
       << regional_stats.interior.aspect_ratio_mean
       << std::endl;
}

bool IsAdaptiveStableMode() { return ADAPTIVE_MODE == "adaptive_stable"; }

int StablePatchMinElements() {
  return std::max(8, static_cast<int>(std::round(6.0 + 8.0 * ADAPTIVE_QUALITY_PRIORITY)));
}

double StablePatchQualityMin() {
  return ClampValue(0.16 + 0.12 * ADAPTIVE_QUALITY_PRIORITY, 0.16, 0.30);
}

double StablePatchPoorRatioMax() {
  return ClampValue(0.45 - 0.25 * ADAPTIVE_QUALITY_PRIORITY, 0.15, 0.45);
}

double EstimateExpectedFaceCount(const double patch_area,
                                 const double target_patch_h) {
  if (patch_area <= 0.0 || target_patch_h <= 0.0) {
    return 0.0;
  }
  return patch_area / std::max(1.0e-9, target_patch_h * target_patch_h);
}

double ComputeTransitionFloor(const double target_patch_h,
                              const double internal_ratio) {
  if (target_patch_h <= 0.0) {
    return 0.10;
  }
  const double h_term = ClampValue((0.18 - target_patch_h) / 0.18, 0.0, 1.0);
  const double interior_term = ClampValue((internal_ratio - 0.60) / 0.35, 0.0, 1.0);
  return ClampValue(0.08 + 0.08 * h_term + 0.07 * interior_term, 0.08, 0.24);
}

double ComputeInteriorTransitionImbalance(
    const Par2DJMesh::AFT::AdvancingFrontRunStats& aft_stats,
    const double transition_floor) {
  const double transition_gap =
      std::max(0.0, transition_floor - aft_stats.transition_element_ratio);
  const double internal_excess =
      std::max(0.0, aft_stats.internal_element_ratio - 0.65);
  return transition_gap + 0.5 * internal_excess;
}

double ComputeCurveBalanceFloor(const double target_patch_h,
                                const double internal_ratio) {
  if (target_patch_h <= 0.0) {
    return 0.70;
  }
  const double h_term = ClampValue((0.18 - target_patch_h) / 0.18, 0.0, 1.0);
  const double interior_term = ClampValue((internal_ratio - 0.58) / 0.35, 0.0, 1.0);
  return ClampValue(0.65 + 0.20 * h_term + 0.10 * interior_term, 0.65, 0.95);
}

double ComputeCurveDomainBalance(PatchCoons* coons_patch, const double target_patch_h) {
  if (coons_patch == nullptr || target_patch_h <= 0.0 ||
      coons_patch->GetNumBerCurves() == 0U) {
    return 1.0;
  }

  double min_ratio = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < coons_patch->GetNumBerCurves(); ++i) {
    CurveAdaptive* curve = coons_patch->GetCurve(i);
    if (curve == nullptr) {
      continue;
    }
    const double curve_length = std::max(1.0e-9, curve->GetLength());
    const int actual_points = std::max(2, static_cast<int>(curve->GetNumBerPoints()));
    const double desired_spacing = std::max(1.0e-6, target_patch_h * 1.35);
    const int needed_points =
        std::max(2, static_cast<int>(std::ceil(curve_length / desired_spacing)) + 1);
    const double ratio =
        static_cast<double>(actual_points) / static_cast<double>(needed_points);
    min_ratio = std::min(min_ratio, ratio);
  }

  if (min_ratio == std::numeric_limits<double>::max()) {
    return 1.0;
  }
  return ClampValue(min_ratio, 0.0, 2.0);
}

bool IsTransitionUnderresolved(
    const Par2DJMesh::AFT::AdvancingFrontRunStats& aft_stats,
    const double transition_floor) {
  const bool interior_dominant = aft_stats.internal_element_ratio > 0.66;
  return interior_dominant &&
         (aft_stats.transition_element_ratio + 1.0e-9 < transition_floor);
}

double ComputePatchDegeneracyScore(
    const FaceMetricStats& generated_stats,
    const Par2DJMesh::AFT::AdvancingFrontRunStats& aft_stats,
    const double patch_area = 0.0, const double target_patch_h = -1.0,
    const double transition_floor = 0.0, const double curve_domain_balance = 1.0,
    const double curve_balance_floor = 0.0) {
  const double min_elements_score =
      generated_stats.count < static_cast<unsigned long long>(StablePatchMinElements())
          ? 1.0
          : 0.0;
  const double quality_score =
      std::max(0.0, StablePatchQualityMin() - aft_stats.quality_post_aft_min);
  const double poor_ratio_score =
      std::max(0.0, aft_stats.poor_ratio_post_aft - StablePatchPoorRatioMax());
  const double template_score =
      (aft_stats.quadtree_low_score_count > 0 &&
       aft_stats.quadtree_template_score_min < (0.50 - 0.10 * ADAPTIVE_INTENSITY))
          ? 1.0
          : 0.0;
  const double expected_faces = EstimateExpectedFaceCount(patch_area, target_patch_h);
  const double underresolved_score =
      (expected_faces >= static_cast<double>(StablePatchMinElements()) * 1.5 &&
       generated_stats.count <
           static_cast<unsigned long long>(std::floor(expected_faces * 0.45)))
          ? 1.0
          : 0.0;
  const double transition_score =
      IsTransitionUnderresolved(aft_stats, transition_floor) ? 1.0 : 0.0;
  const double transition_imbalance_score =
      std::max(0.0, ComputeInteriorTransitionImbalance(aft_stats, transition_floor));
  const double curve_balance_score =
      std::max(0.0, curve_balance_floor - curve_domain_balance);
  return min_elements_score + 2.0 * quality_score + poor_ratio_score +
         template_score + underresolved_score + transition_score +
         transition_imbalance_score + 1.2 * curve_balance_score;
}

bool PatchNeedsFallback(const FaceMetricStats& generated_stats,
                        const Par2DJMesh::AFT::AdvancingFrontRunStats& aft_stats,
                        const double patch_area = 0.0,
                        const double target_patch_h = -1.0,
                        const double transition_floor = 0.0,
                        const double curve_domain_balance = 1.0,
                        const double curve_balance_floor = 0.0) {
  const double expected_faces = EstimateExpectedFaceCount(patch_area, target_patch_h);
  const bool underresolved =
      expected_faces >= static_cast<double>(StablePatchMinElements()) * 1.5 &&
      generated_stats.count <
          static_cast<unsigned long long>(std::floor(expected_faces * 0.45));
  return generated_stats.count <
             static_cast<unsigned long long>(StablePatchMinElements()) ||
         aft_stats.quality_post_aft_min < StablePatchQualityMin() ||
         aft_stats.poor_ratio_post_aft > StablePatchPoorRatioMax() ||
         underresolved ||
         (curve_domain_balance + 1.0e-9 < curve_balance_floor &&
          aft_stats.internal_element_ratio > 0.60) ||
         IsTransitionUnderresolved(aft_stats, transition_floor) ||
         (aft_stats.quadtree_low_score_count > 0 &&
          aft_stats.quadtree_template_score_min < (0.50 - 0.10 * ADAPTIVE_INTENSITY));
}

void SmoothDomainTargetSizes(FaceList& mesh_old) {
  std::vector<Face*> faces(mesh_old.begin(), mesh_old.end());
  if (faces.size() < 2) {
    return;
  }

  std::unordered_map<EdgeKey, std::vector<int>, EdgeKeyHasher> edge_to_faces;
  edge_to_faces.reserve(faces.size() * 3);

  for (std::size_t i = 0; i < faces.size(); ++i) {
    Face* face = faces[i];
    edge_to_faces[MakeEdgeKey(face->getV1(), face->getV2())].push_back(
        static_cast<int>(i));
    edge_to_faces[MakeEdgeKey(face->getV2(), face->getV3())].push_back(
        static_cast<int>(i));
    edge_to_faces[MakeEdgeKey(face->getV3(), face->getV1())].push_back(
        static_cast<int>(i));
  }

  std::vector<std::vector<int>> neighbors(faces.size());
  std::vector<int> boundary_degree(faces.size(), 0);

  for (const auto& item : edge_to_faces) {
    const std::vector<int>& incident = item.second;
    if (incident.size() == 1) {
      ++boundary_degree[incident.front()];
      continue;
    }
    for (std::size_t i = 0; i < incident.size(); ++i) {
      for (std::size_t j = i + 1; j < incident.size(); ++j) {
        neighbors[incident[i]].push_back(incident[j]);
        neighbors[incident[j]].push_back(incident[i]);
      }
    }
  }

  for (std::size_t i = 0; i < neighbors.size(); ++i) {
    std::sort(neighbors[i].begin(), neighbors[i].end());
    neighbors[i].erase(std::unique(neighbors[i].begin(), neighbors[i].end()),
                       neighbors[i].end());
  }

  std::vector<double> values(faces.size(), 0.0);
  std::vector<double> proposed(faces.size(), 0.0);
  for (std::size_t i = 0; i < faces.size(); ++i) {
    values[i] = faces[i]->h;
  }

  const double ratio_limit = 1.35;
  for (int pass = 0; pass < 3; ++pass) {
    for (std::size_t i = 0; i < faces.size(); ++i) {
      if (neighbors[i].empty()) {
        proposed[i] = values[i];
        continue;
      }

      double neighbor_sum = 0.0;
      double neighbor_min = values[neighbors[i].front()];
      for (int neighbor : neighbors[i]) {
        neighbor_sum += values[neighbor];
        neighbor_min = std::min(neighbor_min, values[neighbor]);
      }

      const double neighbor_avg =
          neighbor_sum / static_cast<double>(neighbors[i].size());
      const double boundary_distance = FaceBoundaryDistance(faces[i]);
      const bool near_boundary = boundary_degree[i] > 0 || boundary_distance < 0.14;
      const double blend = near_boundary ? 0.18 : 0.42;
      const double target = 0.5 * neighbor_avg + 0.5 * neighbor_min;
      proposed[i] = (1.0 - blend) * values[i] + blend * target;
    }

    values.swap(proposed);

    for (const auto& item : edge_to_faces) {
      const std::vector<int>& incident = item.second;
      if (incident.size() != 2) {
        continue;
      }
      const int a = incident[0];
      const int b = incident[1];
      double& ha = values[a];
      double& hb = values[b];
      if (ha > hb * ratio_limit) {
        ha = hb * ratio_limit;
      } else if (hb > ha * ratio_limit) {
        hb = ha * ratio_limit;
      }
    }
  }

  std::vector<int> depth(faces.size(), std::numeric_limits<int>::max());
  std::queue<int> frontier;
  for (std::size_t i = 0; i < faces.size(); ++i) {
    if (boundary_degree[i] > 0) {
      depth[i] = 0;
      frontier.push(static_cast<int>(i));
    }
  }

  while (!frontier.empty()) {
    const int current = frontier.front();
    frontier.pop();
    for (int neighbor : neighbors[current]) {
      if (depth[neighbor] > depth[current] + 1) {
        depth[neighbor] = depth[current] + 1;
        frontier.push(neighbor);
      }
    }
  }

  std::vector<int> order(faces.size(), 0);
  for (std::size_t i = 0; i < faces.size(); ++i) {
    order[i] = static_cast<int>(i);
  }
  std::sort(order.begin(), order.end(),
            [&depth](const int a, const int b) { return depth[a] < depth[b]; });

  const double growth_per_layer = 1.18;
  for (int index : order) {
    if (depth[index] <= 0 || neighbors[index].empty()) {
      continue;
    }

    double parent_limit = values[index];
    bool found_parent = false;
    for (int neighbor : neighbors[index]) {
      if (depth[neighbor] == depth[index] - 1) {
        const double allowed = values[neighbor] * growth_per_layer;
        parent_limit = found_parent ? std::min(parent_limit, allowed) : allowed;
        found_parent = true;
      }
    }

    if (found_parent) {
      values[index] = std::min(values[index], parent_limit);
    }
  }

  for (std::size_t i = 0; i < faces.size(); ++i) {
    faces[i]->h = values[i];
  }
}
}  // namespace

#if USE_OPENMP

list<PointAdaptive*> Adapter::AdaptCurveByCurveOmp(
    CurveAdaptive* curve, Performer::IdManager* id_manager,
    double factor_disc_global) {
  // Parameters generated by rediscretization.
  list<double> parameters;

  // Curve points.
  list<PointAdaptive*> points = curve->GetPoints();
  auto point_current = points.begin();
  auto point_next = points.begin();
  ++point_next;

  // Temporary midpoint (must not consume IDs).
  PointAdaptive midpoint;

  static_cast<CurveAdaptiveParametric*>(curve)->SortPointsByParameters();

  // Initialize the binary tree for the whole curve.
  BinTree bin_tree;

  // For each curve segment.
  while (point_next != points.end()) {
    const double length_old =
        curve->CalculateLengthPoints(*(*point_current), *(*point_next));

    midpoint =
        static_cast<CurveAdaptiveParametric*>(curve)->CalculateMidpointByPoints(
            *(*point_current), *(*point_next));

    // IMPORTANT: Midpoint is temporary; do not assign persistent IDs here.
    // midpoint.SetId(id_manager->next(0));  // Removed

    const double midpoint_segment =
        static_cast<CurveAdaptiveParametric*>(curve)->FindParameterByPoint(
            midpoint);

    const double ka_midpoint = curve->CalculateCurvature(midpoint_segment);

    // NOTE: Original code used CalculateCurvature(0/1); keep behavior.
    const double kd_average =
        (curve->CalculateCurvature(0) + curve->CalculateCurvature(1)) / 2.0;

    const double factor_disc = DISCRETIZATION_CURVE_FACTOR;

    const double lenght_new =
        CalculateNewSize(ka_midpoint, kd_average, factor_disc, length_old);

    const double lenght_par = lenght_new / curve->GetLength();

    bin_tree.Subdivide(midpoint_segment, lenght_par * factor_disc_global,
                       static_cast<CurveAdaptiveParametric*>(curve));

    ++point_next;
    ++point_current;
  }

  // Restrict binary tree.
  while (bin_tree.Restrict(static_cast<CurveAdaptiveParametric*>(curve))) {
  }

  // Update curve parameters.
  parameters = bin_tree.Rediscretization();
  static_cast<CurveAdaptiveParametric*>(curve)->UpdateParameters(parameters);

  list<PointAdaptive*> list_new_points;

  // In OMP variants (no map_points), keep original behavior (reuse endpoints).
  NodeAdaptive* point_front = static_cast<NodeAdaptive*>(points.front());
  point_front->SetId(id_manager->next(0));

  NodeAdaptive* point_back = static_cast<NodeAdaptive*>(points.back());
  point_back->SetId(id_manager->next(0));

  list_new_points.push_front(point_front);

  for (auto it = ++parameters.begin(); it != --parameters.end(); ++it) {
    NodeAdaptive* p = new NodeAdaptive(
        static_cast<CurveAdaptiveParametric*>(curve)->FindPointByParameter(*it));
    p->SetId(id_manager->next(0));
    list_new_points.push_back(p);
  }

  list_new_points.push_back(point_back);

  return list_new_points;
}

list<PointAdaptive*> Adapter::AdaptCurveBySurfaceOmp(
    CurveAdaptive* curve, Performer::IdManager* id_manager,
    double factor_disc_global) {
  // Parameters generated by rediscretization.
  list<double> parameters;

  // Curve points.
  list<PointAdaptive*> points = curve->GetPoints();
  auto point_current = points.begin();
  auto point_next = points.begin();
  ++point_next;

  // Temporary midpoint (must not consume IDs).
  PointAdaptive midpoint;

  static_cast<CurveAdaptiveParametric*>(curve)->SortPointsByParameters();

  // Initialize the binary tree for the whole curve.
  BinTree bin_tree;

  // For each curve segment.
  while (point_next != points.end()) {
    const double length_old =
        curve->CalculateLengthPoints(*(*point_current), *(*point_next));

    midpoint =
        static_cast<CurveAdaptiveParametric*>(curve)->CalculateMidpointByPoints(
            *(*point_current), *(*point_next));

    // Compute analytical curvature over surface patches.
    // For shared curves, use the most curved adjacent patch (max |curvature|)
    // to avoid cancellation when many patches meet the same curve.
    const unsigned int num_patches = curve->GetNumBerPatches();

    double ka_midpoint = 0.0;
    double kd_average = 0.0;
    bool use_mean = false;

    if (num_patches > 1) {
      double best_curvature = 0.0;
      bool best_uses_mean = false;
      bool found = false;

      for (unsigned int i = 0; i < num_patches; ++i) {
        CurvatureAnalytical ka(
            midpoint, *(static_cast<PatchCoons*>(curve->GetPatch(i))));
        const double ga = ka.CalculateGaussCurvature();
        const double me = ka.CalculateMeanCurvature();
        const bool local_use_mean = fabs(ga) < TOLERANCE;
        const double local_curvature = local_use_mean ? me : ga;

        if (!found || fabs(local_curvature) > fabs(best_curvature)) {
          best_curvature = local_curvature;
          best_uses_mean = local_use_mean;
          found = true;
        }
      }

      ka_midpoint = best_curvature;
      use_mean = best_uses_mean;
    } else {
      CurvatureAnalytical ka(
          midpoint, *(static_cast<PatchCoons*>(curve->GetPatch(0))));
      const double ga = ka.CalculateGaussCurvature();

      if (fabs(ga) < TOLERANCE) {
        ka_midpoint = ka.CalculateMeanCurvature();
        use_mean = true;
        kd_average =
            (static_cast<NodeAdaptive*>((*point_current))->GetHd() +
             static_cast<NodeAdaptive*>((*point_next))->GetHd()) /
            2.0;
      } else {
        ka_midpoint = ga;
        use_mean = false;
        kd_average =
            (static_cast<NodeAdaptive*>((*point_current))->GetGd() +
             static_cast<NodeAdaptive*>((*point_next))->GetGd()) /
            2.0;
      }
    }

    // Match kd_average with the curvature type.
    if (use_mean) {
      kd_average =
          (static_cast<NodeAdaptive*>((*point_current))->GetHd() +
           static_cast<NodeAdaptive*>((*point_next))->GetHd()) /
          2.0;
    } else {
      kd_average =
          (static_cast<NodeAdaptive*>((*point_current))->GetGd() +
           static_cast<NodeAdaptive*>((*point_next))->GetGd()) /
          2.0;
    }

    const double factor_disc = CalculateCurvatureDrivenFactor(
        ka_midpoint, kd_average, DISCRETIZATION_CURVE_FACTOR, num_patches);

    const double lenght_new =
        CalculateNewSize(ka_midpoint, kd_average, factor_disc, length_old);

    const double lenght_par = lenght_new / curve->GetLength();

    const double midpoint_segment =
        static_cast<CurveAdaptiveParametric*>(curve)->FindParameterByPoint(
            midpoint);

    bin_tree.Subdivide(midpoint_segment, lenght_par * factor_disc_global,
                       static_cast<CurveAdaptiveParametric*>(curve));

    ++point_next;
    ++point_current;
  }

  // Restrict binary tree.
  while (bin_tree.Restrict(static_cast<CurveAdaptiveParametric*>(curve))) {
  }

  // Update curve parameters.
  parameters = bin_tree.Rediscretization();
  static_cast<CurveAdaptiveParametric*>(curve)->UpdateParameters(parameters);

  list<PointAdaptive*> list_new_points;

  // In OMP variants (no map_points), keep original behavior (reuse endpoints).
  NodeAdaptive* point_front = static_cast<NodeAdaptive*>(points.front());
  point_front->SetId(id_manager->next(0));

  NodeAdaptive* point_back = static_cast<NodeAdaptive*>(points.back());
  point_back->SetId(id_manager->next(0));

  list_new_points.push_back(point_front);

  for (auto it = ++parameters.begin(); it != --parameters.end(); ++it) {
    NodeAdaptive* p = new NodeAdaptive(
        static_cast<CurveAdaptiveParametric*>(curve)->FindPointByParameter(*it));
    p->SetId(id_manager->next(0));
    list_new_points.push_back(p);
  }

  list_new_points.push_back(point_back);

  return list_new_points;
}

SubMesh* Adapter::AdaptDomainOmp(PatchCoons* coons_patch,
                                 Performer::IdManager* id_manager,
                                 double factor_disc_global,
                                 double target_patch_h,
                                 int max_quadtree_depth, int adaptive_step,
                                 int patch_index, int retry_index) {
  SubMesh* sub_mesh_new = new SubMesh;

  const double factor_disc = DISCRETIZATION_CURVE_FACTOR_INTERNAL;

  // Advancing front (ratio, tolerance, smoothing iterations, smoothing factor).
  AdvancingFront avanco(RATIO_AFT, TOLERANCE_AFT, SMOOTHING_LAPLACIAN_NUMBER,
                        SMOOTHING_LAPLACIAN_FACTOR);
  avanco.SetAdaptiveContext(adaptive_step, patch_index);
  const int resolved_depth_omp =
      ResolveQuadtreeDepthCap(max_quadtree_depth, adaptive_step);
  if (avanco.getQuadtree()) {
    avanco.getQuadtree()->setMaxLevelCap(resolved_depth_omp);
  }

  map<Vertex*, NodeAdaptive*> vertex_to_node;

  // 1) For each curve of the patch.
  for (unsigned int i = 0; i < coons_patch->GetNumBerCurves(); ++i) {
    CurveAdaptive* curve = coons_patch->GetCurve(i);

    if (i == 0 || i == 1) {
      auto last_parameter =
          static_cast<CurveAdaptiveParametric*>(curve)->parameters_.end();
      --last_parameter;

      int parameter_index = 0;

      for (auto it =
               static_cast<CurveAdaptiveParametric*>(curve)->parameters_.begin();
           it != last_parameter; ++it) {
        Vertex* vertex = nullptr;

        if (i == 0) {
          vertex = avanco.getBoundary()->addVertex(
              *it, 0.0, static_cast<CurveAdaptiveParametric*>(curve));
        } else {
          vertex = avanco.getBoundary()->addVertex(
              1.0, *it, static_cast<CurveAdaptiveParametric*>(curve));
        }

        NodeAdaptive* node_original =
            static_cast<NodeAdaptive*>(curve->GetPoint(parameter_index));
        auto node_copy = std::make_unique<NodeAdaptive>(*node_original);
        node_copy->SetId(node_original->GetId());

        NodeAdaptive* node_ptr = node_copy.get();
        vertex_to_node[vertex] = node_ptr;
        sub_mesh_new->AddNoh(std::move(node_copy));

        ++parameter_index;
      }
    } else if (i == 2 || i == 3) {
      auto last_parameter =
          static_cast<CurveAdaptiveParametric*>(curve)->parameters_.rend();
      --last_parameter;

      int parameter_index =
          static_cast<CurveAdaptiveParametric*>(curve)->GetNumBerPoints() - 1;

      for (auto it =
               static_cast<CurveAdaptiveParametric*>(curve)->parameters_.rbegin();
           it != last_parameter; ++it) {
        Vertex* vertex = nullptr;

        if (i == 2) {
          vertex = avanco.getBoundary()->addVertex(
              *it, 1.0, static_cast<CurveAdaptiveParametric*>(curve));
        } else {
          vertex = avanco.getBoundary()->addVertex(
              0.0, *it, static_cast<CurveAdaptiveParametric*>(curve));
        }

        NodeAdaptive* node_original =
            static_cast<NodeAdaptive*>(curve->GetPoint(parameter_index));
        auto node_copy = std::make_unique<NodeAdaptive>(*node_original);
        node_copy->SetId(node_original->GetId());

        NodeAdaptive* node_ptr = node_copy.get();
        vertex_to_node[vertex] = node_ptr;
        sub_mesh_new->AddNoh(std::move(node_copy));

        --parameter_index;
      }
    }
  }

  avanco.getBoundary()->close(static_cast<CurveAdaptiveParametric*>(
      coons_patch->GetCurve(coons_patch->GetNumBerCurves() - 1)));

  SubMesh* sub_mesh_old = coons_patch->GetSubMesh();

  FaceList mesh_old;
  double area_total = 0.0;

  for (unsigned int i = 0; i < sub_mesh_old->GetNumberElements(); ++i) {
    TriangleAdaptive* tri =
        static_cast<TriangleAdaptive*>(sub_mesh_old->GetElement(i));

    Vertex* v1 = new Vertex(get<0>(tri->GetParametersN1()),
                            get<1>(tri->GetParametersN1()));
    Vertex* v2 = new Vertex(get<0>(tri->GetParametersN2()),
                            get<1>(tri->GetParametersN2()));
    Vertex* v3 = new Vertex(get<0>(tri->GetParametersN3()),
                            get<1>(tri->GetParametersN3()));

    Face* face = new Face(v1, v2, v3, tri->GetId());
    area_total += tri->GetArea();
    mesh_old.push_back(face);
  }

  auto face_it = mesh_old.begin();

  for (unsigned int i = 0; i < sub_mesh_old->GetNumberElements(); ++i) {
    TriangleAdaptive* tri =
        static_cast<TriangleAdaptive*>(sub_mesh_old->GetElement(i));
    Face* face = *face_it;

    const double length_old = sqrt(tri->GetArea() / area_total);

    double ka = 0.0;
    double kd = 0.0;

    if (fabs(tri->GetNoh(1).GetGa()) >= TOLERANCE &&
        fabs(tri->GetNoh(2).GetGa()) >= TOLERANCE &&
        fabs(tri->GetNoh(3).GetGa()) >= TOLERANCE) {
      ka = (tri->GetNoh(1).GetGa() + tri->GetNoh(2).GetGa() +
            tri->GetNoh(3).GetGa()) /
           3.0;
      kd = (tri->GetNoh(1).GetGd() + tri->GetNoh(2).GetGd() +
            tri->GetNoh(3).GetGd()) /
           3.0;
    } else {
      ka = (tri->GetNoh(1).GetHa() + tri->GetNoh(2).GetHa() +
            tri->GetNoh(3).GetHa()) /
           3.0;
      kd = (tri->GetNoh(1).GetHd() + tri->GetNoh(2).GetHd() +
            tri->GetNoh(3).GetHd()) /
           3.0;
    }

    const double effective_factor_disc =
        CalculateCurvatureDrivenFactor(ka, kd, factor_disc, 1);
    const double lenght_new =
        CalculateNewSize(ka, kd, effective_factor_disc, length_old);

    face->h = lenght_new * factor_disc_global;
    if (target_patch_h > 0.0) {
      face->h = std::max(face->h, target_patch_h);
    }

    ++face_it;
  }

  const FaceMetricStats raw_stats = ComputeFaceMetricStats(mesh_old);

  SmoothDomainTargetSizes(mesh_old);
  const FaceMetricStats smoothed_stats = ComputeFaceMetricStats(mesh_old);

  if (!avanco.execute(mesh_old)) {
    while (!mesh_old.empty()) {
      Face* face = mesh_old.front();
      mesh_old.pop_front();
      delete face;
    }
    throw std::runtime_error(
        "advancing-front generation failed while adapting a patch domain");
  }

  FaceList new_mesh = avanco.getMesh();
  const FaceMetricStats generated_stats = ComputeFaceMetricStats(new_mesh);
  AppendDomainDebugLog(adaptive_step, retry_index, patch_index,
                       factor_disc_global, raw_stats, smoothed_stats,
                       generated_stats, avanco.GetRunStats());
  VertexList new_vertices = avanco.getInnerVertices();

  while (!new_vertices.empty()) {
    Vertex* vertex = new_vertices.front();
    new_vertices.pop_front();

    auto node = std::make_unique<NodeAdaptive>(
        coons_patch->Parameterize(vertex->getX(), vertex->getY()));
    node->SetId(id_manager->next(0));

    NodeAdaptive* node_ptr = node.get();
    vertex_to_node[vertex] = node_ptr;
    sub_mesh_new->AddNoh(std::move(node));
  }

  while (!new_mesh.empty()) {
    Face* face = new_mesh.front();
    new_mesh.pop_front();

    // Triangles reference the nodes owned by sub_mesh_new.
    ElementAdaptive* element = new TriangleAdaptive(
        static_cast<NodeAdaptive*>(vertex_to_node[face->getV1()]),
        static_cast<NodeAdaptive*>(vertex_to_node[face->getV2()]),
        static_cast<NodeAdaptive*>(vertex_to_node[face->getV3()]));

    element->SetId(id_manager->next(1));

    static_cast<TriangleAdaptive*>(element)->SetParametersN1(
        make_tuple(face->getV1()->getX(), face->getV1()->getY()));
    static_cast<TriangleAdaptive*>(element)->SetParametersN2(
        make_tuple(face->getV2()->getX(), face->getV2()->getY()));
    static_cast<TriangleAdaptive*>(element)->SetParametersN3(
        make_tuple(face->getV3()->getX(), face->getV3()->getY()));

    sub_mesh_new->AddElement(std::unique_ptr<ElementAdaptive>(element));
  }

  // Delete old mesh faces (original behavior).
  while (!mesh_old.empty()) {
    Face* face = mesh_old.front();
    mesh_old.pop_front();
    delete face;
  }

  return sub_mesh_new;
}

#endif  // USE_OPENMP

list<PointAdaptive*> Adapter::AdaptCurveByCurve(
    CurveAdaptive* curve, map<PointAdaptive*, PointAdaptive*>& map_points,
    Performer::IdManager* id_manager, double factor_disc_global) {
  list<double> parameters;

  list<PointAdaptive*> points = curve->GetPoints();
  auto point_current = points.begin();
  auto point_next = points.begin();
  ++point_next;

  PointAdaptive midpoint;

  static_cast<CurveAdaptiveParametric*>(curve)->SortPointsByParameters();

  BinTree bin_tree;

  while (point_next != points.end()) {
    const double length_old =
        curve->CalculateLengthPoints(*(*point_current), *(*point_next));

    midpoint =
        static_cast<CurveAdaptiveParametric*>(curve)->CalculateMidpointByPoints(
            *(*point_current), *(*point_next));

    const double midpoint_segment =
        static_cast<CurveAdaptiveParametric*>(curve)->FindParameterByPoint(
            midpoint);

    const double ka_midpoint = curve->CalculateCurvature(midpoint_segment);

    // NOTE: Keep original behavior (CalculateCurvature(0/1)).
    const double kd_average =
        (curve->CalculateCurvature(0) + curve->CalculateCurvature(1)) / 2.0;

    const double factor_disc = DISCRETIZATION_CURVE_FACTOR;

    const double lenght_new =
        CalculateNewSize(ka_midpoint, kd_average, factor_disc, length_old);

    const double lenght_par = lenght_new / curve->GetLength();

    bin_tree.Subdivide(midpoint_segment, lenght_par * factor_disc_global,
                       static_cast<CurveAdaptiveParametric*>(curve));

    ++point_next;
    ++point_current;
  }

  while (bin_tree.Restrict(static_cast<CurveAdaptiveParametric*>(curve))) {
  }

  parameters = bin_tree.Rediscretization();
  static_cast<CurveAdaptiveParametric*>(curve)->UpdateParameters(parameters);

  list<PointAdaptive*> list_new_points;

  // IMPORTANT: Do not change IDs of original endpoints here.
  NodeAdaptive* point_front = static_cast<NodeAdaptive*>(points.front());
  NodeAdaptive* point_back = static_cast<NodeAdaptive*>(points.back());

  NodeAdaptive* new_front = nullptr;
  NodeAdaptive* new_back = nullptr;

  // Reuse or clone front.
  {
    auto it = map_points.find(point_front);
    if (it == map_points.end()) {
      new_front = new NodeAdaptive(*point_front);
      new_front->SetId(CheckedLegacyId(id_manager->next(0)));
      map_points.emplace(point_front, new_front);
    } else {
      new_front = static_cast<NodeAdaptive*>(it->second);
    }
  }

  // Reuse or clone back.
  {
    auto it = map_points.find(point_back);
    if (it == map_points.end()) {
      new_back = new NodeAdaptive(*point_back);
      new_back->SetId(CheckedLegacyId(id_manager->next(0)));
      map_points.emplace(point_back, new_back);
    } else {
      new_back = static_cast<NodeAdaptive*>(it->second);
    }
  }

  list_new_points.push_back(new_front);

  for (auto it = ++parameters.begin(); it != --parameters.end(); ++it) {
    NodeAdaptive* node = new NodeAdaptive(
        static_cast<CurveAdaptiveParametric*>(curve)->FindPointByParameter(*it));
    node->SetId(CheckedLegacyId(id_manager->next(0)));
    list_new_points.push_back(node);
  }

  list_new_points.push_back(new_back);

  // Allow subsequent passes to hit the cache using the current endpoints too.
  map_points[new_front] = new_front;
  map_points[new_back] = new_back;

  return list_new_points;
}

list<PointAdaptive*> Adapter::AdaptCurveBySurface(
    CurveAdaptive* curve, map<PointAdaptive*, PointAdaptive*>& map_points,
    Performer::IdManager* id_manager, double factor_disc_global) {
  list<double> parameters;

  list<PointAdaptive*> points = curve->GetPoints();
  auto point_current = points.begin();
  auto point_next = points.begin();
  ++point_next;

  PointAdaptive midpoint;

  static_cast<CurveAdaptiveParametric*>(curve)->SortPointsByParameters();

  BinTree bin_tree;

  while (point_next != points.end()) {
    const double length_old =
        curve->CalculateLengthPoints(*(*point_current), *(*point_next));

    midpoint =
        static_cast<CurveAdaptiveParametric*>(curve)->CalculateMidpointByPoints(
            *(*point_current), *(*point_next));

    const unsigned int num_patches = curve->GetNumBerPatches();

    double ka_midpoint = 0.0;
    double kd_average = 0.0;
    bool use_mean = false;

    if (num_patches > 1) {
      double best_curvature = 0.0;
      bool best_uses_mean = false;
      bool found = false;

      for (unsigned int i = 0; i < num_patches; ++i) {
        CurvatureAnalytical ka(
            midpoint, *(static_cast<PatchCoons*>(curve->GetPatch(i))));
        const double ga = ka.CalculateGaussCurvature();
        const double me = ka.CalculateMeanCurvature();
        const bool local_use_mean = fabs(ga) < TOLERANCE;
        const double local_curvature = local_use_mean ? me : ga;

        if (!found || fabs(local_curvature) > fabs(best_curvature)) {
          best_curvature = local_curvature;
          best_uses_mean = local_use_mean;
          found = true;
        }
      }

      ka_midpoint = best_curvature;
      use_mean = best_uses_mean;
    } else {
      CurvatureAnalytical ka(
          midpoint, *(static_cast<PatchCoons*>(curve->GetPatch(0))));
      const double ga = ka.CalculateGaussCurvature();

      if (fabs(ga) < TOLERANCE) {
        ka_midpoint = ka.CalculateMeanCurvature();
        use_mean = true;
        kd_average =
            (static_cast<NodeAdaptive*>((*point_current))->GetHd() +
             static_cast<NodeAdaptive*>((*point_next))->GetHd()) /
            2.0;
      } else {
        ka_midpoint = ga;
        use_mean = false;
        kd_average =
            (static_cast<NodeAdaptive*>((*point_current))->GetGd() +
             static_cast<NodeAdaptive*>((*point_next))->GetGd()) /
            2.0;
      }
    }

    // Match kd_average with curvature type.
    if (use_mean) {
      kd_average =
          (static_cast<NodeAdaptive*>((*point_current))->GetHd() +
           static_cast<NodeAdaptive*>((*point_next))->GetHd()) /
          2.0;
    } else {
      kd_average =
          (static_cast<NodeAdaptive*>((*point_current))->GetGd() +
           static_cast<NodeAdaptive*>((*point_next))->GetGd()) /
          2.0;
    }

    const double factor_disc = CalculateCurvatureDrivenFactor(
        ka_midpoint, kd_average, DISCRETIZATION_CURVE_FACTOR, num_patches);

    const double lenght_new =
        CalculateNewSize(ka_midpoint, kd_average, factor_disc, length_old);

    const double lenght_par = lenght_new / curve->GetLength();

    const double midpoint_segment =
        static_cast<CurveAdaptiveParametric*>(curve)->FindParameterByPoint(
            midpoint);

    bin_tree.Subdivide(midpoint_segment, lenght_par * factor_disc_global,
                       static_cast<CurveAdaptiveParametric*>(curve));

    ++point_next;
    ++point_current;
  }

  while (bin_tree.Restrict(static_cast<CurveAdaptiveParametric*>(curve))) {
  }

  parameters = bin_tree.Rediscretization();
  static_cast<CurveAdaptiveParametric*>(curve)->UpdateParameters(parameters);

  list<PointAdaptive*> list_new_points;

  // IMPORTANT: Do not change IDs of original endpoints here.
  NodeAdaptive* point_front = static_cast<NodeAdaptive*>(points.front());
  NodeAdaptive* point_back = static_cast<NodeAdaptive*>(points.back());

  NodeAdaptive* new_front = nullptr;
  NodeAdaptive* new_back = nullptr;

  // Reuse or clone front.
  {
    auto it = map_points.find(point_front);
    if (it == map_points.end()) {
      new_front = new NodeAdaptive(*point_front);
      new_front->SetId(CheckedLegacyId(id_manager->next(0)));
      map_points.emplace(point_front, new_front);
    } else {
      new_front = static_cast<NodeAdaptive*>(it->second);
    }
  }

  // Reuse or clone back.
  {
    auto it = map_points.find(point_back);
    if (it == map_points.end()) {
      new_back = new NodeAdaptive(*point_back);
      new_back->SetId(CheckedLegacyId(id_manager->next(0)));
      map_points.emplace(point_back, new_back);
    } else {
      new_back = static_cast<NodeAdaptive*>(it->second);
    }
  }

  list_new_points.push_back(new_front);

  for (auto it = ++parameters.begin(); it != --parameters.end(); ++it) {
    NodeAdaptive* node = new NodeAdaptive(
        static_cast<CurveAdaptiveParametric*>(curve)->FindPointByParameter(*it));
    node->SetId(CheckedLegacyId(id_manager->next(0)));
    list_new_points.push_back(node);
  }

  list_new_points.push_back(new_back);

  // Allow subsequent passes to hit the cache using the current endpoints too.
  map_points[new_front] = new_front;
  map_points[new_back] = new_back;

  return list_new_points;
}

SubMesh* Adapter::AdaptDomain(PatchCoons* coons_patch,
                              Performer::IdManager* id_manager,
                              double factor_disc_global,
                              double target_patch_h,
                              int max_quadtree_depth, int adaptive_step,
                              int patch_index, int retry_index) {
  PrepareStructuralStatsContext(adaptive_step, retry_index);
  const int resolved_quadtree_depth =
      ResolveQuadtreeDepthCap(max_quadtree_depth, adaptive_step);
  struct AttemptResult {
    std::unique_ptr<SubMesh> sub_mesh;
    FaceMetricStats raw_stats;
    FaceMetricStats smoothed_stats;
    FaceMetricStats generated_stats;
    RegionalPatchStats regional_stats;
    Par2DJMesh::AFT::AdvancingFrontRunStats aft_stats;
    double patch_area = 0.0;
    double degeneracy_score = 0.0;
    double transition_floor = 0.0;
    int transition_underresolved = 0;
    double interior_transition_imbalance = 0.0;
    double curve_domain_balance = 1.0;
    double curve_balance_floor = 0.0;
    std::string status = "ok";
    bool success = false;
  };

  auto run_attempt = [&](double local_factor, double local_target_h,
                         int local_depth, int local_attempt) -> AttemptResult {
    AttemptResult result;
    result.sub_mesh = std::make_unique<SubMesh>();
    const double factor_disc = DISCRETIZATION_CURVE_FACTOR_INTERNAL;
    AdvancingFront avanco(RATIO_AFT, TOLERANCE_AFT,
                          static_cast<unsigned int>(SMOOTHING_LAPLACIAN_NUMBER),
                          SMOOTHING_LAPLACIAN_FACTOR);
    avanco.SetAdaptiveContext(adaptive_step, patch_index);
    if (avanco.getQuadtree()) {
      avanco.getQuadtree()->setMaxLevelCap(local_depth);
    }

    map<Vertex*, NodeAdaptive*> vertex_to_node;
    for (unsigned int i = 0; i < coons_patch->GetNumBerCurves(); ++i) {
      CurveAdaptive* curve = coons_patch->GetCurve(i);
      static_cast<CurveAdaptiveParametric*>(curve)->SortPointsByParameters();
      if (i == 0 || i == 1) {
        auto last_parameter =
            static_cast<CurveAdaptiveParametric*>(curve)->parameters_.end();
        --last_parameter;
        int parameter_index = 0;
        for (auto it =
                 static_cast<CurveAdaptiveParametric*>(curve)->parameters_.begin();
             it != last_parameter; ++it) {
          Vertex* vertex =
              (i == 0)
                  ? avanco.getBoundary()->addVertex(
                        *it, 0.0, static_cast<CurveAdaptiveParametric*>(curve))
                  : avanco.getBoundary()->addVertex(
                        1.0, *it, static_cast<CurveAdaptiveParametric*>(curve));
          NodeAdaptive* node_original =
              static_cast<NodeAdaptive*>(curve->GetPoint(parameter_index));
          auto node_copy = std::make_unique<NodeAdaptive>(*node_original);
          node_copy->SetId(node_original->GetId());
          NodeAdaptive* node_ptr = node_copy.get();
          vertex_to_node[vertex] = node_ptr;
          result.sub_mesh->AddNoh(std::move(node_copy));
          ++parameter_index;
        }
      } else if (i == 2 || i == 3) {
        auto last_parameter =
            static_cast<CurveAdaptiveParametric*>(curve)->parameters_.rend();
        --last_parameter;
        int parameter_index =
            static_cast<CurveAdaptiveParametric*>(curve)->GetNumBerPoints() - 1;
        for (auto it =
                 static_cast<CurveAdaptiveParametric*>(curve)->parameters_.rbegin();
             it != last_parameter; ++it) {
          Vertex* vertex =
              (i == 2)
                  ? avanco.getBoundary()->addVertex(
                        *it, 1.0, static_cast<CurveAdaptiveParametric*>(curve))
                  : avanco.getBoundary()->addVertex(
                        0.0, *it, static_cast<CurveAdaptiveParametric*>(curve));
          NodeAdaptive* node_original =
              static_cast<NodeAdaptive*>(curve->GetPoint(parameter_index));
          auto node_copy = std::make_unique<NodeAdaptive>(*node_original);
          node_copy->SetId(node_original->GetId());
          NodeAdaptive* node_ptr = node_copy.get();
          vertex_to_node[vertex] = node_ptr;
          result.sub_mesh->AddNoh(std::move(node_copy));
          --parameter_index;
        }
      }
    }

    avanco.getBoundary()->close(static_cast<CurveAdaptiveParametric*>(
        coons_patch->GetCurve(coons_patch->GetNumBerCurves() - 1)));

    SubMesh* sub_mesh_old = coons_patch->GetSubMesh();
    FaceList mesh_old;
    double area_total = 0.0;
    for (unsigned int i = 0; i < sub_mesh_old->GetNumberElements(); ++i) {
      TriangleAdaptive* tri =
          static_cast<TriangleAdaptive*>(sub_mesh_old->GetElement(i));
      Vertex* v1 =
          new Vertex(get<0>(tri->GetParametersN1()), get<1>(tri->GetParametersN1()));
      Vertex* v2 =
          new Vertex(get<0>(tri->GetParametersN2()), get<1>(tri->GetParametersN2()));
      Vertex* v3 =
          new Vertex(get<0>(tri->GetParametersN3()), get<1>(tri->GetParametersN3()));
      Face* face = new Face(v1, v2, v3, tri->GetId());
      area_total += tri->GetArea();
      mesh_old.push_back(face);
    }
    result.patch_area = area_total;

    auto face_it = mesh_old.begin();
    for (unsigned int i = 0; i < sub_mesh_old->GetNumberElements(); ++i) {
      TriangleAdaptive* tri =
          static_cast<TriangleAdaptive*>(sub_mesh_old->GetElement(i));
      Face* face = *face_it;
      const double length_old = sqrt(tri->GetArea() / area_total);
      double ka = 0.0;
      double kd = 0.0;
      if (fabs(tri->GetNoh(1).GetGa()) >= TOLERANCE &&
          fabs(tri->GetNoh(2).GetGa()) >= TOLERANCE &&
          fabs(tri->GetNoh(3).GetGa()) >= TOLERANCE) {
        ka = (tri->GetNoh(1).GetGa() + tri->GetNoh(2).GetGa() +
              tri->GetNoh(3).GetGa()) /
             3.0;
        kd = (tri->GetNoh(1).GetGd() + tri->GetNoh(2).GetGd() +
              tri->GetNoh(3).GetGd()) /
             3.0;
      } else {
        ka = (tri->GetNoh(1).GetHa() + tri->GetNoh(2).GetHa() +
              tri->GetNoh(3).GetHa()) /
             3.0;
        kd = (tri->GetNoh(1).GetHd() + tri->GetNoh(2).GetHd() +
              tri->GetNoh(3).GetHd()) /
             3.0;
      }
      const double effective_factor_disc =
          CalculateCurvatureDrivenFactor(ka, kd, factor_disc, 1);
      const double lenght_new =
          CalculateNewSize(ka, kd, effective_factor_disc, length_old);
      face->h = lenght_new * local_factor;
      if (local_target_h > 0.0) {
        face->h = std::max(face->h, local_target_h);
      }
      ++face_it;
    }

    result.raw_stats = ComputeFaceMetricStats(mesh_old);
    SmoothDomainTargetSizes(mesh_old);
    result.smoothed_stats = ComputeFaceMetricStats(mesh_old);

    if (!avanco.execute(mesh_old)) {
      while (!mesh_old.empty()) {
        Face* face = mesh_old.front();
        mesh_old.pop_front();
        delete face;
      }
      result.status = "generation_failed";
      return result;
    }

    FaceList new_mesh = avanco.getMesh();
    result.generated_stats = ComputeFaceMetricStats(new_mesh);
    result.regional_stats = ComputeRegionalPatchStats(new_mesh, local_target_h);
    result.aft_stats = avanco.GetRunStats();
    result.curve_domain_balance =
        ComputeCurveDomainBalance(coons_patch, local_target_h);
    result.transition_floor = ComputeTransitionFloor(
        local_target_h, result.aft_stats.internal_element_ratio);
    result.curve_balance_floor = ComputeCurveBalanceFloor(
        local_target_h, result.aft_stats.internal_element_ratio);
    result.transition_underresolved =
        IsTransitionUnderresolved(result.aft_stats, result.transition_floor) ? 1 : 0;
    result.interior_transition_imbalance = ComputeInteriorTransitionImbalance(
        result.aft_stats, result.transition_floor);
    result.degeneracy_score = ComputePatchDegeneracyScore(
        result.generated_stats, result.aft_stats, result.patch_area, local_target_h,
        result.transition_floor, result.curve_domain_balance,
        result.curve_balance_floor);
    result.status = PatchNeedsFallback(result.generated_stats, result.aft_stats,
                                       result.patch_area, local_target_h,
                                       result.transition_floor,
                                       result.curve_domain_balance,
                                       result.curve_balance_floor)
                        ? (result.transition_underresolved == 1
                               || result.curve_domain_balance + 1.0e-9 <
                                      result.curve_balance_floor
                               ? "curve_transition_needs_retry"
                               : "degenerate_needs_retry")
                        : "ok";

    const double expected_faces =
        EstimateExpectedFaceCount(result.patch_area, local_target_h);
    if (IsAdaptiveStableMode() && adaptive_step >= 2 && expected_faces > 0.0) {
      const bool transition_sensitive = (result.transition_underresolved == 1) ||
                                        (result.curve_domain_balance + 1.0e-9 <
                                         result.curve_balance_floor);
      const double cap_local_retry = transition_sensitive ? 1.15 : 1.35;
      const bool overshoot_local =
          static_cast<double>(result.generated_stats.count) >
          (expected_faces * cap_local_retry);
      if (overshoot_local && transition_sensitive) {
        result.status = "curve_transition_needs_retry";
      }
    }

    AppendDomainDebugLog(adaptive_step, retry_index, patch_index, local_factor,
                         result.raw_stats, result.smoothed_stats,
                         result.generated_stats, result.aft_stats,
                         result.regional_stats,
                         local_attempt, result.status, result.degeneracy_score,
                         result.transition_floor, result.transition_underresolved,
                         result.interior_transition_imbalance,
                         result.curve_domain_balance);

    VertexList new_vertices = avanco.getInnerVertices();
    while (!new_vertices.empty()) {
      Vertex* vertex = new_vertices.front();
      new_vertices.pop_front();
      auto node = std::make_unique<NodeAdaptive>(
          coons_patch->Parameterize(vertex->getX(), vertex->getY()));
      node->SetId(CheckedLegacyId(id_manager->next(0)));
      NodeAdaptive* node_ptr = node.get();
      vertex_to_node[vertex] = node_ptr;
      result.sub_mesh->AddNoh(std::move(node));
    }

    while (!new_mesh.empty()) {
      Face* face = new_mesh.front();
      new_mesh.pop_front();
      ElementAdaptive* element = new TriangleAdaptive(
          static_cast<NodeAdaptive*>(vertex_to_node[face->getV1()]),
          static_cast<NodeAdaptive*>(vertex_to_node[face->getV2()]),
          static_cast<NodeAdaptive*>(vertex_to_node[face->getV3()]));
      element->SetId(CheckedLegacyId(id_manager->next(1)));
      static_cast<TriangleAdaptive*>(element)->SetParametersN1(
          make_tuple(face->getV1()->getX(), face->getV1()->getY()));
      static_cast<TriangleAdaptive*>(element)->SetParametersN2(
          make_tuple(face->getV2()->getX(), face->getV2()->getY()));
      static_cast<TriangleAdaptive*>(element)->SetParametersN3(
          make_tuple(face->getV3()->getX(), face->getV3()->getY()));
      result.sub_mesh->SetElement(element);
    }

    while (!mesh_old.empty()) {
      Face* face = mesh_old.front();
      mesh_old.pop_front();
      delete face;
    }
    result.success = true;
    return result;
  };

  AttemptResult best =
      run_attempt(factor_disc_global, target_patch_h, resolved_quadtree_depth, 0);
  // Em adaptive_stable, permitir fallback estrutural tambem em passos tardios.
  // Isso ajuda patches curvos que entram em "degenerate_needs_retry" no step 4/5.
  if (IsAdaptiveStableMode() && adaptive_step >= 1 &&
      (!best.success || PatchNeedsFallback(best.generated_stats, best.aft_stats,
                                           best.patch_area, target_patch_h,
                                           best.transition_floor,
                                           best.curve_domain_balance,
                                           best.curve_balance_floor))) {
    const bool early_step = adaptive_step <= 1;
    const bool transition_retry_initial =
        (best.status == "curve_transition_needs_retry");
    int max_structural_attempts = transition_retry_initial ? 2 : 1;
    if (adaptive_step >= 4) {
      max_structural_attempts = std::max(max_structural_attempts, 3);
    }
    for (int attempt = 1; attempt <= max_structural_attempts; ++attempt) {
      const bool transition_retry = (best.status == "curve_transition_needs_retry");
      const double fallback_factor =
          transition_retry
              ? factor_disc_global *
                    (early_step ? (1.10 + 0.06 * (attempt - 1))
                                : (1.22 + 0.08 * (attempt - 1)))
              : factor_disc_global *
                    (early_step ? (0.90 - 0.05 * ADAPTIVE_QUALITY_PRIORITY)
                                : (0.94 - 0.04 * ADAPTIVE_QUALITY_PRIORITY));
      const double fallback_target_h =
          target_patch_h > 0.0
              ? target_patch_h *
                    (transition_retry
                         ? (early_step ? (0.82 - 0.04 * (attempt - 1))
                                       : (0.72 - 0.05 * (attempt - 1)))
                         : (early_step ? (0.82 - 0.05 * ADAPTIVE_INTENSITY)
                                       : (0.90 - 0.04 * ADAPTIVE_INTENSITY)))
              : target_patch_h;
      int fallback_depth;
      if (resolved_quadtree_depth < 0) {
        fallback_depth =
            transition_retry ? (early_step ? 5 : 6) : (early_step ? 5 : 4);
      } else {
        fallback_depth =
            resolved_quadtree_depth +
            (transition_retry ? (early_step ? 2 : 3) : (early_step ? 2 : 1));
        fallback_depth = std::max(0, std::min(32, fallback_depth));
      }
      AttemptResult fallback =
          run_attempt(fallback_factor, fallback_target_h, fallback_depth, attempt);
      const bool best_underresolved =
          best.transition_underresolved == 1 ||
          (best.curve_domain_balance + 1.0e-9 < best.curve_balance_floor);
      const bool fallback_underresolved =
          fallback.transition_underresolved == 1 ||
          (fallback.curve_domain_balance + 1.0e-9 < fallback.curve_balance_floor);
      const bool prefer_fallback_structural =
          fallback.success &&
          (!best.success ||
           (best_underresolved && !fallback_underresolved) ||
           (best_underresolved == fallback_underresolved &&
            (fallback.interior_transition_imbalance + 1.0e-9 <
                 best.interior_transition_imbalance ||
             fallback.generated_stats.count + 1ULL < best.generated_stats.count)));
      if (prefer_fallback_structural ||
          (fallback.success &&
           (!best.success ||
            fallback.degeneracy_score < best.degeneracy_score ||
            fallback.generated_stats.quality_min > best.generated_stats.quality_min))) {
        best = std::move(fallback);
      }
      if (best.success &&
          best.transition_underresolved == 0 &&
          !(best.curve_domain_balance + 1.0e-9 < best.curve_balance_floor)) {
        break;
      }
    }
  }

  if (!best.success) {
    throw std::runtime_error(
        "all advancing-front attempts failed while adapting a patch domain");
  }
  if (patch_index >= 0) {
    if (static_cast<size_t>(patch_index) >= latest_patch_structural_stats_.size()) {
      latest_patch_structural_stats_.resize(static_cast<size_t>(patch_index) + 1U);
    }
    DomainStructuralStats stats;
    stats.transition_floor = best.transition_floor;
    stats.transition_underresolved = best.transition_underresolved;
    stats.interior_transition_imbalance = best.interior_transition_imbalance;
    stats.curve_domain_balance = best.curve_domain_balance;
    stats.curve_interior_coupling = best.regional_stats.curve_interior_coupling;
    stats.good_ratio_ge_0_60 = best.regional_stats.good_ratio_ge_0_60;
    stats.generated_faces = best.generated_stats.count;
    stats.boundary_region = Adapter::RegionQualityStats{
        best.regional_stats.boundary.element_count,
        best.regional_stats.boundary.quality_min,
        best.regional_stats.boundary.quality_mean,
        best.regional_stats.boundary.poor_ratio,
        best.regional_stats.boundary.good_ratio_ge_0_60,
        best.regional_stats.boundary.area_min,
        best.regional_stats.boundary.area_mean,
        best.regional_stats.boundary.area_max,
        best.regional_stats.boundary.aspect_ratio_mean};
    stats.transition_region = Adapter::RegionQualityStats{
        best.regional_stats.transition.element_count,
        best.regional_stats.transition.quality_min,
        best.regional_stats.transition.quality_mean,
        best.regional_stats.transition.poor_ratio,
        best.regional_stats.transition.good_ratio_ge_0_60,
        best.regional_stats.transition.area_min,
        best.regional_stats.transition.area_mean,
        best.regional_stats.transition.area_max,
        best.regional_stats.transition.aspect_ratio_mean};
    stats.interior_region = Adapter::RegionQualityStats{
        best.regional_stats.interior.element_count,
        best.regional_stats.interior.quality_min,
        best.regional_stats.interior.quality_mean,
        best.regional_stats.interior.poor_ratio,
        best.regional_stats.interior.good_ratio_ge_0_60,
        best.regional_stats.interior.area_min,
        best.regional_stats.interior.area_mean,
        best.regional_stats.interior.area_max,
        best.regional_stats.interior.aspect_ratio_mean};
    stats.generation_status = best.status;
    latest_patch_structural_stats_[static_cast<size_t>(patch_index)] = stats;
  }
  return best.sub_mesh.release();
}

void Adapter::PrepareStructuralStatsContext(int adaptive_step, int retry_index) {
  if (adaptive_step != last_structural_step_ || retry_index != last_structural_retry_) {
    latest_patch_structural_stats_.clear();
    last_structural_step_ = adaptive_step;
    last_structural_retry_ = retry_index;
  }
}

const Adapter::DomainStructuralStats* Adapter::GetPatchStructuralStats(
    int patch_index) const {
  if (patch_index < 0 ||
      static_cast<size_t>(patch_index) >= latest_patch_structural_stats_.size()) {
    return nullptr;
  }
  return &latest_patch_structural_stats_[static_cast<size_t>(patch_index)];
}

double Adapter::CalculateNewSize(const double ka, const double kd,
                                 const double factor_disc,
                                 const double length_old) {
  // Scenario 1: ka is very close to kd.
  if (((ka - TOLERANCE_CURVATURE) < kd) && (kd < (ka + TOLERANCE_CURVATURE))) {
    // If near planar, coarsen.
    if (fabs(ka) < TOLERANCE_CURVATURE) {
      return length_old * factor_disc;
    }
    // Otherwise, discretization already reflects the surface well.
    return length_old;
  }

  // Other scenarios: refine.
  return length_old / factor_disc;
}

double Adapter::CalculateCurvatureDrivenFactor(const double ka, const double kd,
                                               const double base_factor,
                                               const unsigned int num_patches) {
  const double mismatch = fabs(ka - kd);
  const double reference =
      std::max(std::max(fabs(ka), fabs(kd)), TOLERANCE_CURVATURE);
  const double mismatch_ratio = mismatch / reference;

  // Shared curves should feel the most curved adjacent patch more strongly,
  // otherwise a straight common edge tends to stop at only one bisection.
  const double mismatch_scale = 1.0 + ClampValue(mismatch_ratio, 0.0, 2.5);
  const double adjacency_scale =
      1.0 + 0.35 * std::max(0, static_cast<int>(num_patches) - 1);

  return ClampValue(base_factor * mismatch_scale * adjacency_scale,
                    base_factor, base_factor * 4.0);
}
