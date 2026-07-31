#ifndef DATA_CURVE_CURVE_ADAPTIVE_PARAMETRIC_HERMITE_H
#define DATA_CURVE_CURVE_ADAPTIVE_PARAMETRIC_HERMITE_H

#include "curve_adaptive_parametric.h"

class CurveAdaptiveParametricHermite : public CurveAdaptiveParametric {
 public:
  CurveAdaptiveParametricHermite();
  CurveAdaptiveParametricHermite(const PointAdaptive point0,
                                 const PointAdaptive point1,
                                 const VectorAdaptive vector0,
                                 const VectorAdaptive vector1);
  explicit CurveAdaptiveParametricHermite(CurveAdaptiveParametricHermite*);
  ~CurveAdaptiveParametricHermite();
  double CalculateCurvature(double t) override;

  VectorAdaptive GetVector0() const;
  void SetVector0(const VectorAdaptive& vector0);
  VectorAdaptive GetVector1() const;
  void SetVector1(const VectorAdaptive& vector1);

 protected:
  // vetor tangente no ponto inicial
  VectorAdaptive vector0_;
  // vetor tangente no ponto final
  VectorAdaptive vector1_;
};
#endif  // DATA_CURVE_CURVE_ADAPTIVE_PARAMETRIC_HERMITE_H
