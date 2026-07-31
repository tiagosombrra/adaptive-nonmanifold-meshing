#include "data/geometry.h"

#include <unordered_set>

#include "data/curve/curve_adaptive.h"
#include "data/curve/curve_adaptive_parametric_bezier.h"
#include "data/patch/patch.h"
#include "data/point_adaptive.h"
#include "data/vector_adaptive.h"

Geometry::Geometry() = default;

Geometry::~Geometry() = default;

Geometry::Geometry(Geometry&&) noexcept = default;
Geometry& Geometry::operator=(Geometry&&) noexcept = default;

void Geometry::InsertCurve(std::unique_ptr<CurveAdaptive> curve) {
  curves_.push_back(std::move(curve));
}

void Geometry::InsertCurve(CurveAdaptive* curve) {
  // Takes ownership of curve (must be allocated with new).
  // (English comment)
  curves_.emplace_back(curve);
}

unsigned int Geometry::GetNumberCurves() const noexcept {
  return static_cast<unsigned int>(curves_.size());
}

CurveAdaptive* Geometry::GetCurve(unsigned int position) noexcept {
  return (position < curves_.size()) ? curves_[position].get() : nullptr;
}

const CurveAdaptive* Geometry::GetCurve(unsigned int position) const noexcept {
  return (position < curves_.size()) ? curves_[position].get() : nullptr;
}

void Geometry::InsertPatch(std::unique_ptr<Patch> patch) {
  patches_.push_back(std::move(patch));
}

void Geometry::InsertPatch(Patch* patch) {
  // Takes ownership of patch (must be allocated with new).
  // (English comment)
  patches_.emplace_back(patch);
}

unsigned int Geometry::GetNumberPatches() const noexcept {
  return static_cast<unsigned int>(patches_.size());
}

Patch* Geometry::GetPatch(unsigned int position) noexcept {
  return (position < patches_.size()) ? patches_[position].get() : nullptr;
}

const Patch* Geometry::GetPatch(unsigned int position) const noexcept {
  return (position < patches_.size()) ? patches_[position].get() : nullptr;
}

CurveAdaptiveParametricBezier* Geometry::VerifyCurveGeometry(
    PointAdaptive* p0, PointAdaptive* p1, PointAdaptive* p2, PointAdaptive* p3) {
  for (auto& c : curves_) {
    auto* bez = dynamic_cast<CurveAdaptiveParametricBezier*>(c.get());
    if (!bez) continue;

    const bool same_direction =
        bez->GetPoint0().operator==(p0) && bez->GetPoint1().operator==(p1) &&
        bez->GetPoint2().operator==(p2) && bez->GetPoint3().operator==(p3);
    const bool reversed_direction =
        bez->GetPoint0().operator==(p3) && bez->GetPoint1().operator==(p2) &&
        bez->GetPoint2().operator==(p1) && bez->GetPoint3().operator==(p0);

    if (same_direction || reversed_direction) {
      return bez;
    }
  }
  return nullptr;
}

PointAdaptive* Geometry::AddPoint(std::unique_ptr<PointAdaptive> point) {
  points_.push_back(std::move(point));
  return points_.back().get();
}

VectorAdaptive* Geometry::AddVector(std::unique_ptr<VectorAdaptive> vector) {
  vectors_.push_back(std::move(vector));
  return vectors_.back().get();
}
