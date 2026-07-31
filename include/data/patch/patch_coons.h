#ifndef DATA_PATCH_PATCH_COONS_H
#define DATA_PATCH_PATCH_COONS_H

#include <tuple>
#include <vector>

#include "patch.h"

class CurveAdaptive;
class PointAdaptive;
class VectorAdaptive;

class PatchCoons : public Patch {
 public:
  PatchCoons();
  explicit PatchCoons(PatchCoons* patch_coons);
  explicit PatchCoons(const std::vector<CurveAdaptive*>& curves);
  ~PatchCoons() override;

  void InsertCurve(CurveAdaptive* curve);
  void ReplaceCurve(unsigned int position, CurveAdaptive* curve);
  unsigned int GetNumBerCurves() const noexcept;

  CurveAdaptive* GetCurve(unsigned int position) noexcept;
  const CurveAdaptive* GetCurve(unsigned int position) const noexcept;

  virtual std::tuple<double, double> FindUV(const PointAdaptive& p) = 0;
  virtual PointAdaptive Parameterize(double u, double v) = 0;

  virtual VectorAdaptive Qu(double u, double v) = 0;
  virtual VectorAdaptive Qv(double u, double v) = 0;
  virtual VectorAdaptive Quu(double u, double v) = 0;
  virtual VectorAdaptive Quv(double u, double v) = 0;
  virtual VectorAdaptive Qvu(double u, double v) = 0;
  virtual VectorAdaptive Qvv(double u, double v) = 0;

  virtual VectorAdaptive Qu(const PointAdaptive& point) = 0;
  virtual VectorAdaptive Qv(const PointAdaptive& point) = 0;
  virtual VectorAdaptive Quu(const PointAdaptive& point) = 0;
  virtual VectorAdaptive Quv(const PointAdaptive& point) = 0;
  virtual VectorAdaptive Qvu(const PointAdaptive& point) = 0;
  virtual VectorAdaptive Qvv(const PointAdaptive& point) = 0;

 protected:
  std::vector<CurveAdaptive*> curves_;  // non-owning (Geometry owns curves)
};

#endif  // DATA_PATCH_PATCH_COONS_H
