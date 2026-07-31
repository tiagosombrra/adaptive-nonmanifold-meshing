#ifndef CURVATURE_CURVATURE_H
#define CURVATURE_CURVATURE_H

class Curvature {
 public:
  virtual double CalculateMeanCurvature() = 0;
  virtual double CalculateGaussCurvature() = 0;
};

#endif  // CURVATURE_CURVATURE_H
