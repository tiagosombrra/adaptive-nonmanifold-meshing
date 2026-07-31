#ifndef INPUT_OUTPUT_MODELS_3D_H
#define INPUT_OUTPUT_MODELS_3D_H

#include "../data/curve/curve_adaptive_parametric_bezier.h"
#include "../data/curve/curve_adaptive_parametric_hermite.h"
#include "../data/geometry.h"
#include "../data/patch/patch_bezier.h"
#include "../data/patch/patch_hermite.h"
#include "../data/vector_adaptive.h"
#include "../data/vertex_adaptive.h"
#include <memory>
#include <utility>

class Models3d {
 public:
  Models3d();
  Geometry *ModelPlanBook(Geometry *geometry);
  Geometry *ModelTresPatches(Geometry *geometry);
  Geometry *ModelDoisPatchesPlanosCurva1(Geometry *geometry);
  Geometry *ModelDoisPatchesPlanosCurva(Geometry *geometry);
  Geometry *ModelCurvaBezier(Geometry *geometry);
  Geometry *ModelParaboloide(Geometry *geometry);
  Geometry *ModelPneu(Geometry *geometry);
  Geometry *ModelLadoDescendente(Geometry *geometry);
  Geometry *ModelBaseCircular(Geometry *geometry);
  Geometry *ModelBaseQuadrada(Geometry *geometry);
  Geometry *ModelBordaCurva(Geometry *geometry);
  Geometry *ModelDoisPatches(Geometry *geometry);
  Geometry *ModelNariz(Geometry *geometry);
  Geometry *ModelUtahteapot(Geometry *geometry);

 private:
  template <typename... Args>
  static PointAdaptive *AddPoint(Geometry *geometry, Args &&...args) {
    return geometry->AddPoint(
        std::make_unique<VertexAdaptive>(std::forward<Args>(args)...));
  }

  template <typename... Args>
  static VectorAdaptive *AddVector(Geometry *geometry, Args &&...args) {
    return geometry->AddVector(
        std::make_unique<VectorAdaptive>(std::forward<Args>(args)...));
  }

  template <typename CurveT, typename... Args>
  static CurveAdaptive *MakeCurve(Geometry *geometry, Args &&...args) {
    auto curve = std::make_unique<CurveT>(std::forward<Args>(args)...);
    CurveAdaptive *raw = curve.get();
    geometry->InsertCurve(std::move(curve));
    return raw;
  }

  template <typename PatchT, typename... Args>
  static Patch *MakePatch(Geometry *geometry, Args &&...args) {
    auto patch = std::make_unique<PatchT>(std::forward<Args>(args)...);
    Patch *raw = patch.get();
    geometry->InsertPatch(std::move(patch));
    return raw;
  }
};

#endif  // INPUT_OUTPUT_MODELS_3D_H
