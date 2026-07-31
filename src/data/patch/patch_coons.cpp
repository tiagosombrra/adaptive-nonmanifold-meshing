#include "data/patch/patch_coons.h"

PatchCoons::PatchCoons() = default;

PatchCoons::PatchCoons(PatchCoons* patch_coons) : Patch(patch_coons) {}

PatchCoons::PatchCoons(const std::vector<CurveAdaptive*>& curves)
    : curves_(curves) {}

PatchCoons::~PatchCoons() = default;

void PatchCoons::InsertCurve(CurveAdaptive* curve) {
  curves_.push_back(curve);
}

void PatchCoons::ReplaceCurve(unsigned int position, CurveAdaptive* curve) {
  if (position < curves_.size()) {
    curves_[position] = curve;
  }
}

unsigned int PatchCoons::GetNumBerCurves() const noexcept {
  return static_cast<unsigned int>(curves_.size());
}

CurveAdaptive* PatchCoons::GetCurve(unsigned int position) noexcept {
  return (position < curves_.size()) ? curves_[position] : nullptr;
}

const CurveAdaptive* PatchCoons::GetCurve(unsigned int position) const noexcept {
  return (position < curves_.size()) ? curves_[position] : nullptr;
}
