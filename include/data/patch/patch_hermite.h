#ifndef DATA_PATCH_PATCH_HERMITE_H
#define DATA_PATCH_PATCH_HERMITE_H

#include "../../data/curve/curve_adaptive_parametric_hermite.h"
#include "../../data/definitions.h"
#include "patch_coons.h"

// As curvas devem ser definidas da esquerda para a direita, de baixo para
// cima em relação ao Patch !!!
class PatchHermite : public PatchCoons {
 public:
  PatchHermite();
  explicit PatchHermite(PatchHermite* patch_hermite);
  // Ordem das curvas:
  //		C3
  //	C4		C2
  //		C1
  PatchHermite(CurveAdaptive* curve1, CurveAdaptive* curve2,
               CurveAdaptive* curve3, CurveAdaptive* curve4,
               VectorAdaptive tw00, VectorAdaptive tw10, VectorAdaptive tw01,
               VectorAdaptive tw11, bool signal_curve1 = true,
               bool signal_curve2 = true, bool signal_curve3 = true,
               bool signal_curve4 = true);
  ~PatchHermite() = default;

  PointAdaptive CalculatePointUV();
  void PrintAllMatrixPatchHermite();
  // encontra as coordenadas parâmetricas u, v de um dado ponto p no patch,
  // ou a projeção desse ponto na superfície
  // tuple<double, double> FindUV(const Ponto& p);
  tuple<double, double> FindUV(const PointAdaptive& point) override;

  // encontra o ponto p na curva dado um parâmetro p
  PointAdaptive Parameterize(double u, double v) override;

  // calcula as derivadas parciais nas direçõe u e v
  VectorAdaptive Qu(double u, double v) override;
  VectorAdaptive Qv(double u, double v) override;
  VectorAdaptive Quu(double u, double v) override;
  VectorAdaptive Quv(double u, double v) override;
  VectorAdaptive Qvu(double u, double v) override;
  VectorAdaptive Qvv(double u, double v) override;

  // calcula as derivadas parciais de um ponto p
  VectorAdaptive Qu(const PointAdaptive& point) override;
  VectorAdaptive Qv(const PointAdaptive& point) override;
  VectorAdaptive Quu(const PointAdaptive& point) override;
  VectorAdaptive Quv(const PointAdaptive& point) override;
  VectorAdaptive Qvu(const PointAdaptive& point) override;
  VectorAdaptive Qvv(const PointAdaptive& point) override;

  Matrix1x4 GetU() const;
  Matrix4x4 GetGx() const;
  Matrix4x4 GetGy() const;
  Matrix4x4 GetGz() const;
  Matrix4x1 GetV() const;
  Matrix4x4 GetH() const;

  PointAdaptive GetPt00() const;
  PointAdaptive GetPt01() const;
  PointAdaptive GetPt10() const;
  PointAdaptive GetPt11() const;

  VectorAdaptive GetQu00() const;
  VectorAdaptive GetQu01() const;
  VectorAdaptive GetQu10() const;
  VectorAdaptive GetQu11() const;

  VectorAdaptive GetQv00() const;
  VectorAdaptive GetQv01() const;
  VectorAdaptive GetQv10() const;
  VectorAdaptive GetQv11() const;

  VectorAdaptive GetTw00() const;
  VectorAdaptive GetTw01() const;
  VectorAdaptive GetTw10() const;
  VectorAdaptive GetTw11() const;

 protected:
  Matrix4x4 StartHermiteMatrix();

  PointAdaptive pt00_;
  PointAdaptive pt01_;
  VectorAdaptive qv00_;
  VectorAdaptive qv01_;
  PointAdaptive pt10_;
  PointAdaptive pt11_;
  VectorAdaptive qv10_;
  VectorAdaptive qv11_;
  VectorAdaptive qu00_;
  VectorAdaptive qu01_;
  VectorAdaptive tw00_;
  VectorAdaptive tw01_;
  VectorAdaptive qu10_;
  VectorAdaptive qu11_;
  VectorAdaptive tw10_;
  VectorAdaptive tw11_;

  Matrix4x4 mat_geo_gx_;  // matriz geométrica para coordenada x
  Matrix4x4 mat_geo_gy_;  // matriz geométrica para coordenada y
  Matrix4x4 mat_geo_gz_;  // matriz geométrica para coordenada z

  Matrix4x4 mat_base_;    // matriz de Hermite
  Matrix4x1 mat_base_v_;  // matriz do parâmetro v
  Matrix1x4 mat_base_u_;  // matriz do parâmetro u

  bool signal_curve1_;
  bool signal_curve2_;
  bool signal_curve3_;
  bool signal_curve4_;
};
#endif  // DATA_PATCH_PATCH_HERMITE_H
