#ifndef CURVATURE_CURVATURE_ANALYTICAL_H
#define CURVATURE_CURVATURE_ANALYTICAL_H

#include <cmath>
#include <tuple>

#include "../data/patch/patch_coons.h"
#include "../data/point_adaptive.h"
#include "../data/vector_adaptive.h"
#include "curvature.h"

class CurvatureAnalytical : public Curvature {
 public:
  CurvatureAnalytical(const PointAdaptive& v, PatchCoons& p);
  double CalculateMeanCurvature() override;
  double CalculateGaussCurvature() override;

 private:
  VectorAdaptive qu_;
  VectorAdaptive qv_;
  VectorAdaptive quu_;
  VectorAdaptive quv_;
  VectorAdaptive qvv_;
  VectorAdaptive prod_;
  double a_;
  double b_;
  double c_;
};

#endif  // CURVATURE_CURVATURE_ANALYTICAL_H
