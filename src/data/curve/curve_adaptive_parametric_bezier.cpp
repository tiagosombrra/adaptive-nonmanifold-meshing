#include "../../../include/data/curve/curve_adaptive_parametric_bezier.h"

CurveAdaptiveParametricBezier::CurveAdaptiveParametricBezier()
    : CurveAdaptiveParametric() {
  // 1. Preenche a matriz 'M' de 'CurvaParamétrica com a matriz de Bezier
  //
  //    M->setElement( 0, 0,-1 ); M->setElement( 0, 1, 3 ); M->setElement( 0,
  //    2,-3 ); M->setElement( 0, 3, 1 ); M->setElement( 1, 0, 3 );
  //    M->setElement( 1, 1,-6 ); M->setElement( 1, 2, 3 ); M->setElement( 1, 3,
  //    0 ); M->setElement( 2, 0,-3 ); M->setElement( 2, 1, 3 ); M->setElement(
  //    2, 2, 0 ); M->setElement( 2, 3, 0 ); M->setElement( 3, 0, 1 );
  //    M->setElement( 3, 1, 0 ); M->setElement( 3, 2, 0 ); M->setElement( 3, 3,
  //    0 );

  mat_base_(0, 0) = -1;
  mat_base_(0, 1) = 3;
  mat_base_(0, 2) = -3;
  mat_base_(0, 3) = 1;
  mat_base_(1, 0) = 3;
  mat_base_(1, 1) = -6;
  mat_base_(1, 2) = 3;
  mat_base_(1, 3) = 0;
  mat_base_(2, 0) = -3;
  mat_base_(2, 1) = 3;
  mat_base_(2, 2) = 0;
  mat_base_(2, 3) = 0;
  mat_base_(3, 0) = 1;
  mat_base_(3, 1) = 0;
  mat_base_(3, 2) = 0;
  mat_base_(3, 3) = 0;

  // 2. Preenche as matrizes geométricas com G de Bezier
  //
  //    Gx->setElement( 0, 0, point_0_.x );
  //    Gx->setElement( 1, 0, point_1_.x );
  //    Gx->setElement( 2, 0, P2.x );
  //    Gx->setElement( 3, 0, P3.x );

  mat_geo_gx_(0, 0) = point0_.GetX();
  mat_geo_gx_(1, 0) = point1_.GetX();
  mat_geo_gx_(2, 0) = point2_.GetX();
  mat_geo_gx_(3, 0) = point3_.GetX();

  //    Gy->setElement( 0, 0, point_0_.y );
  //    Gy->setElement( 1, 0, point_1_.y );
  //    Gy->setElement( 2, 0, P2.y );
  //    Gy->setElement( 3, 0, P3.y );

  mat_geo_gy_(0, 0) = point0_.GetY();
  mat_geo_gy_(1, 0) = point1_.GetY();
  mat_geo_gy_(2, 0) = point2_.GetY();
  mat_geo_gy_(3, 0) = point3_.GetY();

  //    Gz->setElement( 0, 0, point_0_.z );
  //    Gz->setElement( 1, 0, point_1_.z );
  //    Gz->setElement( 2, 0, P2.z );
  //    Gz->setElement( 3, 0, P3.z );

  mat_geo_gz_(0, 0) = point0_.GetZ();
  mat_geo_gz_(1, 0) = point1_.GetZ();
  mat_geo_gz_(2, 0) = point2_.GetZ();
  mat_geo_gz_(3, 0) = point3_.GetZ();

  Matrix4x1 m;
  m = this->GetMatBase() * this->GetMatGeoGx();
  // delete &this->Gx;
  this->SetMatGeoGx(m);

  m = this->GetMatBase() * this->GetMatGeoGy();
  // delete &this->Gx;
  this->SetMatGeoGy(m);

  m = this->GetMatBase() * this->GetMatGeoGz();
  // delete &this->Gx;
  this->SetMatGeoGz(m);

  // delete &M;

  // IMPORTANTE: Ao usar esse construtor, não esquecer de setar os pontos
  // inicial e final e os vetores tangentes inicial e final !!!
}

CurveAdaptiveParametricBezier::CurveAdaptiveParametricBezier(
    const PointAdaptive point0, const PointAdaptive point1,
    const PointAdaptive point2, const PointAdaptive point3)
    : CurveAdaptiveParametric(point0, point1),
      point2_(point2),
      point3_(point3) {
  // 1. Preenche a matriz 'M' de 'CurvaParamétrica com a matriz de Bezier

  //    M->setElement( 0, 0,-1 ); M->setElement( 0, 1, 3 ); M->setElement( 0,
  //    2,-3 ); M->setElement( 0, 3, 1 ); M->setElement( 1, 0, 3 );
  //    M->setElement( 1, 1,-6 ); M->setElement( 1, 2, 3 ); M->setElement( 1, 3,
  //    0 ); M->setElement( 2, 0,-3 ); M->setElement( 2, 1, 3 ); M->setElement(
  //    2, 2, 0 ); M->setElement( 2, 3, 0 ); M->setElement( 3, 0, 1 );
  //    M->setElement( 3, 1, 0 ); M->setElement( 3, 2, 0 ); M->setElement( 3, 3,
  //    0 );

  mat_base_(0, 0) = -1;
  mat_base_(0, 1) = 3;
  mat_base_(0, 2) = -3;
  mat_base_(0, 3) = 1;
  mat_base_(1, 0) = 3;
  mat_base_(1, 1) = -6;
  mat_base_(1, 2) = 3;
  mat_base_(1, 3) = 0;
  mat_base_(2, 0) = -3;
  mat_base_(2, 1) = 3;
  mat_base_(2, 2) = 0;
  mat_base_(2, 3) = 0;
  mat_base_(3, 0) = 1;
  mat_base_(3, 1) = 0;
  mat_base_(3, 2) = 0;
  mat_base_(3, 3) = 0;

  // 2. Preenche as matrizes geométricas com G de Bezier

  //    Gx->setElement( 0, 0, point_0_.x );
  //    Gx->setElement( 1, 0, point_1_.x );
  //    Gx->setElement( 2, 0, P2.x );
  //    Gx->setElement( 3, 0, P3.x );

  mat_geo_gx_(0, 0) = point0_.GetX();
  mat_geo_gx_(1, 0) = point1_.GetX();
  mat_geo_gx_(2, 0) = point2.GetX();
  mat_geo_gx_(3, 0) = point3.GetX();

  //    Gy->setElement( 0, 0, point_0_.y );
  //    Gy->setElement( 1, 0, point_1_.y );
  //    Gy->setElement( 2, 0, P2.y );
  //    Gy->setElement( 3, 0, P3.y );

  mat_geo_gy_(0, 0) = point0_.GetY();
  mat_geo_gy_(1, 0) = point1_.GetY();
  mat_geo_gy_(2, 0) = point2.GetY();
  mat_geo_gy_(3, 0) = point3.GetY();

  //    Gz->setElement( 0, 0, point_0_.z );
  //    Gz->setElement( 1, 0, point_1_.z );
  //    Gz->setElement( 2, 0, P2.z );
  //    Gz->setElement( 3, 0, P3.z );

  mat_geo_gz_(0, 0) = point0_.GetZ();
  mat_geo_gz_(1, 0) = point1_.GetZ();
  mat_geo_gz_(2, 0) = point2.GetZ();
  mat_geo_gz_(3, 0) = point3.GetZ();

  Matrix4x1 mat_geo;
  mat_geo = this->GetMatBase() * this->GetMatGeoGx();
  // delete &this->Gx;
  this->SetMatGeoGx(mat_geo);

  mat_geo = this->GetMatBase() * this->GetMatGeoGy();
  // delete &this->Gx;
  this->SetMatGeoGy(mat_geo);

  mat_geo = this->GetMatBase() * this->GetMatGeoGz();
  // delete &this->Gx;
  this->SetMatGeoGz(mat_geo);

  // 3. Calcula o comprimento da curva
  //
  CalculateLengthCurve();
  //
  ///////////////
}

CurveAdaptiveParametricBezier::CurveAdaptiveParametricBezier(
    CurveAdaptiveParametricBezier *curve_adaptive_parametric_bezier)
    : CurveAdaptiveParametric(curve_adaptive_parametric_bezier) {
  this->point2_ = curve_adaptive_parametric_bezier->point2_;
  this->point3_ = curve_adaptive_parametric_bezier->point3_;
}

CurveAdaptiveParametricBezier::~CurveAdaptiveParametricBezier() {}

double CurveAdaptiveParametricBezier::CalculateCurvature(double t) {
  // B'(t)= 3(1-t)²(point_1_-point_0_)+6t(1-t)(P2-point_1_)+3t²(P3-P2)
  double v0 = 3 * pow((1 - t), 2) * (point1_.GetX() - point0_.GetX()) +
              6 * t * (1 - t) * (point2_.GetX() - point1_.GetX()) +
              3 * pow(t, 2) * (point3_.GetX() - point2_.GetX());
  double v1 = 3 * pow((1 - t), 2) * (point1_.GetY() - point0_.GetY()) +
              6 * t * (1 - t) * (point2_.GetY() - point1_.GetY()) +
              3 * pow(t, 2) * (point3_.GetY() - point2_.GetY());
  double v2 = 3 * pow((1 - t), 2) * (point1_.GetZ() - point0_.GetZ()) +
              6 * t * (1 - t) * (point2_.GetZ() - point1_.GetZ()) +
              3 * pow(t, 2) * (point3_.GetZ() - point2_.GetZ());

  // B''(t)= 6(1-t)(P2-2P1+point_0_)+6t(P3-2P2+point_1_)
  double v00 =
      6 * (1 - t) * (point2_.GetX() - 2 * point1_.GetX() + point0_.GetX()) +
      6 * t * (point3_.GetX() - 2 * point2_.GetX() + point1_.GetX());
  double v10 =
      6 * (1 - t) * (point2_.GetY() - 2 * point1_.GetY() + point0_.GetY()) +
      6 * t * (point3_.GetY() - 2 * point2_.GetY() + point1_.GetY());
  double v20 =
      6 * (1 - t) * (point2_.GetZ() - 2 * point1_.GetZ() + point0_.GetZ()) +
      6 * t * (point3_.GetZ() - 2 * point2_.GetZ() + point1_.GetZ());

  VectorAdaptive vector_d1(v0, v1, v2);
  VectorAdaptive vector_d2(v00, v10, v20);

  double k = ((vector_d1.operator*(vector_d2)).CalculateModule()) /
             (pow(vector_d1.CalculateModule(), 3));

  return k;
}

PointAdaptive CurveAdaptiveParametricBezier::GetPoint2() const {
  return this->point2_;
}

PointAdaptive CurveAdaptiveParametricBezier::GetPoint3() const {
  return this->point3_;
}

void CurveAdaptiveParametricBezier::SetPoint2(const PointAdaptive &point) {
  this->point2_ = point;
}

void CurveAdaptiveParametricBezier::SetPoint3(const PointAdaptive &point) {
  this->point3_ = point;
}
