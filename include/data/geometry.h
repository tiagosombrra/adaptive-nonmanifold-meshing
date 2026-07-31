#ifndef GEOMETRIA_H
#define GEOMETRIA_H

#include <memory>
#include <vector>

class CurveAdaptive;
class CurveAdaptiveParametricBezier;
class Patch;
class PointAdaptive;
class VectorAdaptive;

class Geometry {
 public:
  Geometry();
  ~Geometry();  // Defined in .cpp (complete types there).

  Geometry(const Geometry&) = delete;
  Geometry& operator=(const Geometry&) = delete;

  Geometry(Geometry&&) noexcept;
  Geometry& operator=(Geometry&&) noexcept;

  // Curves
  void InsertCurve(std::unique_ptr<CurveAdaptive> curve);
  void InsertCurve(CurveAdaptive* curve);  // takes ownership

  unsigned int GetNumberCurves() const noexcept;

  CurveAdaptive* GetCurve(unsigned int position) noexcept;
  const CurveAdaptive* GetCurve(unsigned int position) const noexcept;

  // Patches
  void InsertPatch(std::unique_ptr<Patch> patch);
  void InsertPatch(Patch* patch);  // takes ownership

  unsigned int GetNumberPatches() const noexcept;

  Patch* GetPatch(unsigned int position) noexcept;
  const Patch* GetPatch(unsigned int position) const noexcept;

  CurveAdaptiveParametricBezier* VerifyCurveGeometry(
      PointAdaptive* p0, PointAdaptive* p1, PointAdaptive* p2, PointAdaptive* p3);

  // Points/Vectors (geometry owns them to avoid leaks)
  PointAdaptive* AddPoint(std::unique_ptr<PointAdaptive> point);
  VectorAdaptive* AddVector(std::unique_ptr<VectorAdaptive> vector);

 private:
  std::vector<std::unique_ptr<PointAdaptive>> points_;
  std::vector<std::unique_ptr<VectorAdaptive>> vectors_;
  std::vector<std::unique_ptr<CurveAdaptive>> curves_;
  std::vector<std::unique_ptr<Patch>> patches_;
};

#endif  // GEOMETRIA_H
