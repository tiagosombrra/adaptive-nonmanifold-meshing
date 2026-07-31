#ifndef ADAPTER_ADAPTER_H
#define ADAPTER_ADAPTER_H

#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "../crab_mesh/aft/advancing_front.h"
#include "../crab_mesh/aft/boundary.h"
#include "../crab_mesh/aft/edge.h"
#include "../crab_mesh/aft/face.h"
#include "../crab_mesh/aft/vertex.h"
#include "../crab_mesh/performer/ranged_id_manager.h"
#include "../curvature/curvature_analytical.h"
#include "../curvature/curvature_discrete.h"
#include "../data/curve/curve_adaptive_parametric.h"
#include "../data/definitions.h"
#include "../data/mesh/sub_mesh.h"
#include "../data/patch/patch_coons.h"
#include "../data/tree/bin_tree.h"
#include "../parallel/TMCommunicator.h"

class Adapter {
 public:
  struct RegionQualityStats {
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

  struct DomainStructuralStats {
    double transition_floor = 0.0;
    int transition_underresolved = 0;
    double interior_transition_imbalance = 0.0;
    double curve_domain_balance = 1.0;
    double curve_interior_coupling = 1.0;
    double good_ratio_ge_0_60 = 0.0;
    unsigned long long generated_faces = 0ULL;
    RegionQualityStats boundary_region;
    RegionQualityStats transition_region;
    RegionQualityStats interior_region;
    std::string generation_status = "unknown";
  };

#if USE_OPENMP
  list<PointAdaptive*> AdaptCurveByCurveOmp(CurveAdaptive* curve,
                                            Performer::IdManager* id_manager,
                                            double factor_disc_global = 1.0);
  list<PointAdaptive*> AdaptCurveBySurfaceOmp(CurveAdaptive* curve,
                                              Performer::IdManager* id_manager,
                                              double factor_disc_global = 1.0);
  SubMesh* AdaptDomainOmp(PatchCoons* coons_patch,
                          Performer::IdManager* id_manager,
                          double factor_disc_global = 1.0,
                          double target_patch_h = -1.0,
                          int max_quadtree_depth = -1,
                          int adaptive_step = -1, int patch_index = -1,
                          int retry_index = 0);
#endif  // #USE_OPENMP

  list<PointAdaptive*> AdaptCurveByCurve(
      CurveAdaptive* curve, map<PointAdaptive*, PointAdaptive*>& map_points,
      Performer::IdManager* id_manager, double factor_disc_global = 1.0);
  list<PointAdaptive*> AdaptCurveBySurface(
      CurveAdaptive* curve, map<PointAdaptive*, PointAdaptive*>& map_points,
      Performer::IdManager* id_manager, double factor_disc_global = 1.0);
 SubMesh* AdaptDomain(PatchCoons* coons_patch,
                       Performer::IdManager* id_manager,
                       double factor_disc_global = 1.0,
                       double target_patch_h = -1.0,
                       int max_quadtree_depth = -1,
                       int adaptive_step = -1, int patch_index = -1,
                       int retry_index = 0);
 const DomainStructuralStats* GetPatchStructuralStats(int patch_index) const;

 private:
  double CalculateNewSize(const double ka, const double kd,
                          const double factor_disc, const double length_old);
  double CalculateCurvatureDrivenFactor(const double ka, const double kd,
                                        const double base_factor,
                                        const unsigned int num_patches = 1);
  void PrepareStructuralStatsContext(int adaptive_step, int retry_index);

  int last_structural_step_ = std::numeric_limits<int>::min();
  int last_structural_retry_ = std::numeric_limits<int>::min();
  std::vector<DomainStructuralStats> latest_patch_structural_stats_;
};
#endif  // ADAPTER_ADAPTER_H
