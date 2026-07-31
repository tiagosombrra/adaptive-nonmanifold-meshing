#include "../../../include/data/patch/patch_bezier.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iostream>

extern double TOLERANCE;  // distância máxima entre dois pontos
extern unsigned int I_MAX;
extern std::atomic<unsigned long long> FINDUV_TOTAL_CALLS;
extern std::atomic<unsigned long long> FINDUV_IMAX_EXITS;

namespace {
// Newton amortecido: reduz oscilacao; tolerancia em (du,dv) mais realista que TOLERANCE.
constexpr double kFindUvDamping = 0.65;
constexpr double kFindUvParamTol = 1e-5;
// Parada por erro 3D no patch (fracao de TOLERANCE usada na malha).
constexpr double kFindUvSpatialFactor = 50.0;
}  // namespace

PatchBezier::PatchBezier()
    : PatchCoons(),
      signal_curve1_(true),
      signal_curve2_(true),
      signal_curve3_(true),
      signal_curve4_(true),
      area_(0),
      ka_medio_(0),
      segment_medio_(0),
      area_triangle_(0),
      number_triangle_(0),
      id_process_(0),
      id_patch_bezier_(0) {}

PatchBezier::PatchBezier(PatchBezier* patch_bezier) : PatchCoons(patch_bezier) {
  this->pt03_ = patch_bezier->pt03_;
  this->pt13_ = patch_bezier->pt13_;
  this->pt23_ = patch_bezier->pt23_;
  this->pt33_ = patch_bezier->pt33_;
  this->pt02_ = patch_bezier->pt02_;
  this->pt12_ = patch_bezier->pt12_;
  this->pt22_ = patch_bezier->pt22_;
  this->pt32_ = patch_bezier->pt32_;
  this->pt01_ = patch_bezier->pt01_;
  this->pt11_ = patch_bezier->pt11_;
  this->pt21_ = patch_bezier->pt21_;
  this->pt31_ = patch_bezier->pt31_;
  this->pt00_ = patch_bezier->pt00_;
  this->pt10_ = patch_bezier->pt10_;
  this->pt20_ = patch_bezier->pt20_;
  this->pt30_ = patch_bezier->pt30_;

  this->mat_geo_gx_ = patch_bezier->mat_geo_gx_;
  this->mat_geo_gy_ = patch_bezier->mat_geo_gy_;
  this->mat_geo_gz_ = patch_bezier->mat_geo_gz_;
  this->mat_base_u_ = patch_bezier->mat_base_u_;
  this->mat_base_v_ = patch_bezier->mat_base_v_;
}

// Ordem das curvas:
//		C3
//	C4		C2
//		C1
PatchBezier::PatchBezier(CurveAdaptive* curve1, CurveAdaptive* curve2,
                         CurveAdaptive* curve3, CurveAdaptive* curve4,
                         PointAdaptive pt11, PointAdaptive pt21,
                         PointAdaptive pt12, PointAdaptive pt22,
                         bool signal_curve1, bool signal_curve2,
                         bool signal_curve3, bool signal_curve4)
    : PatchCoons() {
  this->signal_curve1_ = signal_curve1;
  this->signal_curve2_ = signal_curve2;
  this->signal_curve3_ = signal_curve3;
  this->signal_curve4_ = signal_curve4;

// As curvas devem ser definidas da esquerda para a direita, de baixo para
// cima em relação ao Patch !!! Vizinhos que partilham a mesma curva (mesmos
// quatro controlos, direto ou invertido — ver Geometry::VerifyCurveGeometry)
// ficam ligados à mesma CurveAdaptive; a malha grosseira respeita ordem de
// processamento dos patches (GeneratorInitialMesh).
  // 1. Inclui as curvas na lista de curvas de CoonsPatch
  //
  this->curves_.push_back(curve1);
  this->curves_.push_back(curve2);
  this->curves_.push_back(curve3);
  this->curves_.push_back(curve4);
  //
  // 2. Coloca o Patch na lista das curvas
  //
  static_cast<CurveAdaptiveParametricBezier*>(curve1)->InsertPatch(this);
  static_cast<CurveAdaptiveParametricBezier*>(curve2)->InsertPatch(this);
  static_cast<CurveAdaptiveParametricBezier*>(curve3)->InsertPatch(this);
  static_cast<CurveAdaptiveParametricBezier*>(curve4)->InsertPatch(this);
  //
  // 3. Seta os atributos de acordo com as curvas
  //
  this->pt00_ =
      static_cast<CurveAdaptiveParametricBezier*>(curve1)->GetPoint0();
  this->pt10_ =
      static_cast<CurveAdaptiveParametricBezier*>(curve1)->GetPoint1();
  this->pt20_ =
      static_cast<CurveAdaptiveParametricBezier*>(curve1)->GetPoint2();

  this->pt30_ =
      static_cast<CurveAdaptiveParametricBezier*>(curve2)->GetPoint0();
  this->pt31_ =
      static_cast<CurveAdaptiveParametricBezier*>(curve2)->GetPoint1();
  this->pt32_ =
      static_cast<CurveAdaptiveParametricBezier*>(curve2)->GetPoint2();

  this->pt13_ =
      static_cast<CurveAdaptiveParametricBezier*>(curve3)->GetPoint1();
  this->pt23_ =
      static_cast<CurveAdaptiveParametricBezier*>(curve3)->GetPoint2();
  this->pt33_ =
      static_cast<CurveAdaptiveParametricBezier*>(curve3)->GetPoint3();

  this->pt01_ =
      static_cast<CurveAdaptiveParametricBezier*>(curve4)->GetPoint1();
  this->pt02_ =
      static_cast<CurveAdaptiveParametricBezier*>(curve4)->GetPoint2();
  this->pt03_ =
      static_cast<CurveAdaptiveParametricBezier*>(curve4)->GetPoint3();

  this->pt11_ = pt11;
  this->pt21_ = pt21;
  this->pt12_ = pt12;
  this->pt22_ = pt22;
  //
  // 4. Aloca espaço para as matrizes
  //
  this->mat_base_u_.setZero(1, 4);
  this->mat_base_v_.setZero(4, 1);
  this->mat_geo_gx_.setZero(4, 4);
  this->mat_geo_gy_.setZero(4, 4);
  this->mat_geo_gz_.setZero(4, 4);
  //
  // 5. Preenche as matrizes geométricas com G de Bezier
  //
  // Gx:
  // 2x2 superior esquerdo
  mat_geo_gx_(0, 0) = this->pt00_.GetX();
  mat_geo_gx_(0, 1) = this->pt01_.GetX();
  mat_geo_gx_(1, 0) = this->pt10_.GetX();
  mat_geo_gx_(1, 1) = this->pt11_.GetX();
  // 2x2 inferior esquerdo
  mat_geo_gx_(2, 0) = this->pt20_.GetX();
  mat_geo_gx_(2, 1) = this->pt21_.GetX();
  mat_geo_gx_(3, 0) = this->pt30_.GetX();
  mat_geo_gx_(3, 1) = this->pt31_.GetX();
  // 2x2 superior direito
  mat_geo_gx_(0, 2) = this->pt02_.GetX();
  mat_geo_gx_(0, 3) = this->pt03_.GetX();
  mat_geo_gx_(1, 2) = this->pt12_.GetX();
  mat_geo_gx_(1, 3) = this->pt13_.GetX();
  // 2x2 inferior direito
  mat_geo_gx_(2, 2) = this->pt22_.GetX();
  mat_geo_gx_(2, 3) = this->pt23_.GetX();
  mat_geo_gx_(3, 2) = this->pt32_.GetX();
  mat_geo_gx_(3, 3) = this->pt33_.GetX();
  //
  // Gy:
  // 2x2 superior esquerdo
  this->mat_geo_gy_(0, 0) = this->pt00_.GetY();
  mat_geo_gy_(0, 1) = this->pt01_.GetY();
  mat_geo_gy_(1, 0) = this->pt10_.GetY();
  mat_geo_gy_(1, 1) = this->pt11_.GetY();
  // 2x2 inferior esquerdo
  mat_geo_gy_(2, 0) = this->pt20_.GetY();
  mat_geo_gy_(2, 1) = this->pt21_.GetY();
  mat_geo_gy_(3, 0) = this->pt30_.GetY();
  mat_geo_gy_(3, 1) = this->pt31_.GetY();
  // 2x2 superior direito
  mat_geo_gy_(0, 2) = this->pt02_.GetY();
  mat_geo_gy_(0, 3) = this->pt03_.GetY();
  mat_geo_gy_(1, 2) = this->pt12_.GetY();
  mat_geo_gy_(1, 3) = this->pt13_.GetY();
  // 2x2 inferior direito
  mat_geo_gy_(2, 2) = this->pt22_.GetY();
  mat_geo_gy_(2, 3) = this->pt23_.GetY();
  mat_geo_gy_(3, 2) = this->pt32_.GetY();
  mat_geo_gy_(3, 3) = this->pt33_.GetY();
  //
  // Gz:
  // 2x2 superior esquerdo
  mat_geo_gz_(0, 0) = this->pt00_.GetZ();
  mat_geo_gz_(0, 1) = this->pt01_.GetZ();
  mat_geo_gz_(1, 0) = this->pt10_.GetZ();
  mat_geo_gz_(1, 1) = this->pt11_.GetZ();
  // 2x2 inferior esquerdo
  mat_geo_gz_(2, 0) = this->pt20_.GetZ();
  mat_geo_gz_(2, 1) = this->pt21_.GetZ();
  mat_geo_gz_(3, 0) = this->pt30_.GetZ();
  mat_geo_gz_(3, 1) = this->pt31_.GetZ();
  // 2x2 superior direito
  mat_geo_gz_(0, 2) = this->pt02_.GetZ();
  mat_geo_gz_(0, 3) = this->pt03_.GetZ();
  mat_geo_gz_(1, 2) = this->pt12_.GetZ();
  mat_geo_gz_(1, 3) = this->pt13_.GetZ();
  // 2x2 inferior direito
  mat_geo_gz_(2, 2) = this->pt22_.GetZ();
  mat_geo_gz_(2, 3) = this->pt23_.GetZ();
  mat_geo_gz_(3, 2) = this->pt32_.GetZ();
  mat_geo_gz_(3, 3) = this->pt33_.GetZ();
  //
  // 6.Preenche a Matriz de Bezier
  //
  this->mat_base_ = StartBezierMatrix();
  //
  this->mat_geo_gx_ = this->GetB() * this->GetGx() * this->GetB();
  this->mat_geo_gy_ = this->GetB() * this->GetGy() * this->GetB();
  this->mat_geo_gz_ = this->GetB() * this->GetGz() * this->GetB();
}

// Esse patch acha que é isolado! As curvas que apontam para ele devem ser
// definidas externamente.
PatchBezier ::PatchBezier(const PointAdaptive pt00, const PointAdaptive pt01,
                          const PointAdaptive pt02, const PointAdaptive pt03,
                          const PointAdaptive pt10, const PointAdaptive pt11,
                          const PointAdaptive pt12, const PointAdaptive pt13,
                          const PointAdaptive pt20, const PointAdaptive pt21,
                          const PointAdaptive pt22, const PointAdaptive pt23,
                          const PointAdaptive pt30, const PointAdaptive pt31,
                          const PointAdaptive pt32, const PointAdaptive pt33,
                          bool signal_curve1, bool signal_curve2,
                          bool signal_curve3, bool signal_curve4)
    : PatchCoons(),
      pt03_(pt03),
      pt13_(pt13),
      pt23_(pt23),
      pt33_(pt33),
      pt02_(pt02),
      pt12_(pt12),
      pt22_(pt22),
      pt32_(pt32),
      pt01_(pt01),
      pt11_(pt11),
      pt21_(pt21),
      pt31_(pt31),
      pt00_(pt00),
      pt10_(pt10),
      pt20_(pt20),
      pt30_(pt30) {
  this->signal_curve1_ = signal_curve1;
  this->signal_curve2_ = signal_curve2;
  this->signal_curve3_ = signal_curve3;
  this->signal_curve4_ = signal_curve4;

  // 1. Inclui as curvas na lista de curvas de CoonsPatch
  //
  this->curves_.push_back(
      new CurveAdaptiveParametricBezier(pt00_, pt10_, pt20_, pt30_));
  this->curves_.push_back(
      new CurveAdaptiveParametricBezier(pt30_, pt31_, pt32_, pt33_));
  this->curves_.push_back(
      new CurveAdaptiveParametricBezier(pt03_, pt13_, pt23_, pt33_));
  this->curves_.push_back(
      new CurveAdaptiveParametricBezier(pt00_, pt01_, pt02_, pt03_));
  //
  // 2. Aloca espaço para as matrizes
  //
  this->mat_base_u_.setZero(1, 4);
  this->mat_base_v_.setZero(4, 1);
  this->mat_geo_gx_.setZero(4, 4);
  this->mat_geo_gy_.setZero(4, 4);
  this->mat_geo_gz_.setZero(4, 4);
  //
  ////////////////////////////////////

  // 3. Preenche as matrizes geométricas com G de Bezier
  //
  // 2x2 superior esquerdo
  mat_geo_gx_(0, 0) = this->pt00_.GetX();
  mat_geo_gx_(0, 1) = this->pt01_.GetX();
  mat_geo_gx_(1, 0) = this->pt10_.GetX();
  mat_geo_gx_(1, 1) = this->pt11_.GetX();
  // 2x2 inferior esquerdo
  mat_geo_gx_(2, 0) = this->pt20_.GetX();
  mat_geo_gx_(2, 1) = this->pt21_.GetX();
  mat_geo_gx_(3, 0) = this->pt30_.GetX();
  mat_geo_gx_(3, 1) = this->pt31_.GetX();
  // 2x2 superior direito
  mat_geo_gx_(0, 2) = this->pt02_.GetX();
  mat_geo_gx_(0, 3) = this->pt03_.GetX();
  mat_geo_gx_(1, 2) = this->pt12_.GetX();
  mat_geo_gx_(1, 3) = this->pt13_.GetX();
  // 2x2 inferior direito
  mat_geo_gx_(2, 2) = this->pt22_.GetX();
  mat_geo_gx_(2, 3) = this->pt23_.GetX();
  mat_geo_gx_(3, 2) = this->pt32_.GetX();
  mat_geo_gx_(3, 3) = this->pt33_.GetX();
  //
  // Gy:
  // 2x2 superior esquerdo
  mat_geo_gy_(0, 0) = this->pt00_.GetY();
  mat_geo_gy_(0, 1) = this->pt01_.GetY();
  mat_geo_gy_(1, 0) = this->pt10_.GetY();
  mat_geo_gy_(1, 1) = this->pt11_.GetY();
  // 2x2 inferior esquerdo
  mat_geo_gy_(2, 0) = this->pt20_.GetY();
  mat_geo_gy_(2, 1) = this->pt21_.GetY();
  mat_geo_gy_(3, 0) = this->pt30_.GetY();
  mat_geo_gy_(3, 1) = this->pt31_.GetY();
  // 2x2 superior direito
  mat_geo_gy_(0, 2) = this->pt02_.GetY();
  mat_geo_gy_(0, 3) = this->pt03_.GetY();
  mat_geo_gy_(1, 2) = this->pt12_.GetY();
  mat_geo_gy_(1, 3) = this->pt13_.GetY();
  // 2x2 inferior direito
  mat_geo_gy_(2, 2) = this->pt22_.GetY();
  mat_geo_gy_(2, 3) = this->pt23_.GetY();
  mat_geo_gy_(3, 2) = this->pt32_.GetY();
  mat_geo_gy_(3, 3) = this->pt33_.GetY();
  //
  // Gz:
  // 2x2 superior esquerdo
  mat_geo_gz_(0, 0) = this->pt00_.GetZ();
  mat_geo_gz_(0, 1) = this->pt01_.GetZ();
  mat_geo_gz_(1, 0) = this->pt10_.GetZ();
  mat_geo_gz_(1, 1) = this->pt11_.GetZ();
  // 2x2 inferior esquerdo
  mat_geo_gz_(2, 0) = this->pt20_.GetZ();
  mat_geo_gz_(2, 1) = this->pt21_.GetZ();
  mat_geo_gz_(3, 0) = this->pt30_.GetZ();
  mat_geo_gz_(3, 1) = this->pt31_.GetZ();
  // 2x2 superior direito
  mat_geo_gz_(0, 2) = this->pt02_.GetZ();
  mat_geo_gz_(0, 3) = this->pt03_.GetZ();
  mat_geo_gz_(1, 2) = this->pt12_.GetZ();
  mat_geo_gz_(1, 3) = this->pt13_.GetZ();
  // 2x2 inferior direito
  mat_geo_gz_(2, 2) = this->pt22_.GetZ();
  mat_geo_gz_(2, 3) = this->pt23_.GetZ();
  mat_geo_gz_(3, 2) = this->pt32_.GetZ();
  mat_geo_gz_(3, 3) = this->pt33_.GetZ();
  //
  // 6.Preenche a Matriz de Bezier
  //
  this->mat_base_ = StartBezierMatrix();
  //
  this->mat_geo_gx_ = this->GetB() * this->GetGx() * this->GetB();
  this->mat_geo_gy_ = this->GetB() * this->GetGy() * this->GetB();
  this->mat_geo_gz_ = this->GetB() * this->GetGz() * this->GetB();
}

PatchBezier::~PatchBezier() {
  // delete &mat_base_v_;
  // delete &mat_base_u_;
}

PointAdaptive PatchBezier::CalculatePointUV() {
  PointAdaptive point;

  // C = ( U * ( B * ( G * ( Bt * V ) ) ) )
  point.SetX((this->GetU() * ((this->GetGx() * (this->GetV()))))(0, 0));
  point.SetY((this->GetU() * ((this->GetGy() * (this->GetV()))))(0, 0));
  point.SetZ((this->GetU() * ((this->GetGz() * (this->GetV()))))(0, 0));

  return point;
}

void PatchBezier::PrintAllMatrixPatchBezier() {
  cout << "U:\n" << mat_base_u_;
  cout << "B:\n" << mat_base_;
  cout << "Gx:\n" << mat_geo_gx_;
  cout << "Gy:\n" << mat_geo_gy_;
  cout << "Gz:\n" << mat_geo_gz_;
  cout << "V:\n" << mat_base_v_;
}

// encontra o parâmetro t de um dado ponto p na curva
tuple<double, double> PatchBezier::FindUV(const PointAdaptive& point) {
  FINDUV_TOTAL_CALLS.fetch_add(1ULL, std::memory_order_relaxed);

  unsigned int iter = 0;

  // chute inicial
  double u_i = 0.5;
  double v_i = 0.5;

  double delta_u = 0.0;
  double delta_v = 0.0;

  PointAdaptive p_i;
  const double spatial_tol =
      std::max(1e-12, kFindUvSpatialFactor * std::fabs(TOLERANCE));

  while (true) {
    VectorAdaptive Tu = -(this->Qu(u_i, v_i));
    VectorAdaptive Tv = -(this->Qv(u_i, v_i));

    p_i = this->Parameterize(u_i, v_i);

    Matrix<double, 3, 3> matrix;
    matrix(0, 0) = Tu.GetX();
    matrix(0, 1) = Tv.GetX();
    matrix(0, 2) = p_i.GetX() - point.GetX();
    matrix(1, 0) = Tu.GetY();
    matrix(1, 1) = Tv.GetY();
    matrix(1, 2) = p_i.GetY() - point.GetY();
    matrix(2, 0) = Tu.GetZ();
    matrix(2, 1) = Tv.GetZ();
    matrix(2, 2) = p_i.GetZ() - point.GetZ();

    int k = 0;
    double pivot = matrix(0, 0);

    while ((fabs(pivot) < TOLERANCE) and (k < 2)) {
      ++k;
      pivot = matrix(k, 0);
    }

    matrix.row(k).swap(matrix.row(0));

    if (fabs(pivot) < TOLERANCE) {
      cout << "Erro! Não é possível encontrar as coordenadas paramétricas no "
              "ponto p"
           << point.GetId() << " (" << point.GetX() << ", " << point.GetY()
           << ", " << point.GetZ() << ")" << endl;

      return make_tuple(-1.0, -1.0);
    }

    double A_10 = matrix(1, 0);
    double A_20 = matrix(2, 0);

    for (short j = 0; j < 3; ++j) {
      matrix(0, j) = static_cast<double>(matrix(0, j)) / pivot;
      matrix(1, j) = matrix(1, j) - A_10 * (matrix(0, j));
      matrix(2, j) = matrix(2, j) - A_20 * (matrix(0, j));
    }

    pivot = matrix(1, 1);

    if (fabs(pivot) < TOLERANCE) {
      pivot = matrix(2, 1);
      matrix.row(2).swap(matrix.row(1));
    }

    double A_01 = matrix(0, 1);
    double A_21 = matrix(2, 1);

    for (short j = 0; j < 3; ++j) {
      matrix(1, j) = static_cast<double>(matrix(1, j) / pivot);
      matrix(0, j) = matrix(0, j) - A_01 * (matrix(1, j));
      matrix(2, j) = matrix(2, j) - A_21 * (matrix(1, j));
    }

    delta_u = matrix(0, 2);
    delta_v = matrix(1, 2);

    if (std::isnan(delta_u) || std::isnan(delta_v)) {
      break;
    }

    u_i += kFindUvDamping * delta_u;
    v_i += kFindUvDamping * delta_v;
    u_i = std::clamp(u_i, 0.0, 1.0);
    v_i = std::clamp(v_i, 0.0, 1.0);

    p_i = this->Parameterize(u_i, v_i);
    const double dist = p_i.CalculateDistance(point);

    const bool param_ok =
        (fabs(delta_u) < kFindUvParamTol && fabs(delta_v) < kFindUvParamTol);
    if (param_ok || dist < spatial_tol) {
      break;
    }

    ++iter;
    if (iter >= I_MAX) {
      const unsigned long long hit =
          1ULL + FINDUV_IMAX_EXITS.fetch_add(1ULL, std::memory_order_relaxed);
#if USE_PRINT_COMENT
      if (hit <= 5ULL || (hit % 2000ULL) == 0ULL) {
        std::cout << "[FindUV Bezier] I_MAX id=" << point.GetId()
                  << " iter=" << iter << " |du|=" << fabs(delta_u)
                  << " |dv|=" << fabs(delta_v) << " u=" << u_i << " v=" << v_i
                  << " dist=" << dist << " spatial_tol=" << spatial_tol
                  << std::endl;
      }
#endif
      break;
    }
  }

  if (u_i <= TOLERANCE)
    u_i = 0.0;
  else if (u_i >= 1.0 - TOLERANCE)
    u_i = 1.0;

  if (v_i <= TOLERANCE)
    v_i = 0.0;
  else if (v_i >= 1.0 - TOLERANCE)
    v_i = 1.0;

  if (std::isnan(u_i) || std::isnan(v_i)) {
    cout << "-nan u_i e v_i" << endl;
  }

  return make_tuple(u_i, v_i);
}

// encontra as coordenadas 3D de um ponto p de parâmetros u, v
PointAdaptive PatchBezier::Parameterize(double u, double v) {
  //	cout << "Parameterize ( " << u << ", " << v << ")" << endl;
  // Parameterize:
  //
  //  -> ALTERA a matriz U
  //  -> ALTERA a matriz V
  //  -> usa 'calculaPonto_u_v ( )'
  //

  this->mat_base_u_(0, 0) = u * u * u;
  this->mat_base_u_(0, 1) = u * u;
  this->mat_base_u_(0, 2) = u;
  this->mat_base_u_(0, 3) = 1;

  this->mat_base_v_(0, 0) = v * v * v;
  this->mat_base_v_(1, 0) = v * v;
  this->mat_base_v_(2, 0) = v;
  this->mat_base_v_(3, 0) = 1;

  return CalculatePointUV();
}

// calcula o vetor tangente na direção u nas coordenadas paramétricas u, v
VectorAdaptive PatchBezier::Qu(double u, double v) {
  // Qt:
  //
  //  -> ALOCA um Ponto (mas destroi aqui mesmo)
  //	 -> ALOCA um Vetor (retorna ele)
  //  -> ALTERA a matriz U !!!
  //  -> ALTERA a matriz V !!!
  //

  PointAdaptive P;

  this->mat_base_u_(0, 0) = 3 * u * u;
  this->mat_base_u_(0, 1) = 2 * u;
  this->mat_base_u_(0, 2) = 1;
  this->mat_base_u_(0, 3) = 0;

  this->mat_base_v_(0, 0) = v * v * v;
  this->mat_base_v_(1, 0) = v * v;
  this->mat_base_v_(2, 0) = v;
  this->mat_base_v_(3, 0) = 1;

  P = CalculatePointUV();
  VectorAdaptive V(P);

  return V;
}

// calcula o vetor tangente na direção v nas coordenadas paramétricas u, v
VectorAdaptive PatchBezier::Qv(double u, double v) {
  // Qt:
  //
  //  -> ALOCA um Ponto (mas destroi aqui mesmo)
  //	 -> ALOCA um Vetor (retorna ele)
  //  -> ALTERA a matriz U !!!
  //  -> ALTERA a matriz V !!!
  //

  PointAdaptive P;

  this->mat_base_u_(0, 0) = u * u * u;
  this->mat_base_u_(0, 1) = u * u;
  this->mat_base_u_(0, 2) = u;
  this->mat_base_u_(0, 3) = 1;

  this->mat_base_v_(0, 0) = 3 * v * v;
  this->mat_base_v_(1, 0) = 2 * v;
  this->mat_base_v_(2, 0) = 1;
  this->mat_base_v_(3, 0) = 0;

  P = CalculatePointUV();
  VectorAdaptive V(P);

  return V;
}

// calcula a derivada parcial Quu nas coordenadas paramétricas u, v
VectorAdaptive PatchBezier::Quu(double u, double v) {
  // Qt:
  //
  //  -> ALOCA um Ponto (mas destroi aqui mesmo)
  //	 -> ALOCA um Vetor (retorna ele)
  //  -> ALTERA a matriz U !!!
  //  -> ALTERA a matriz V !!!
  //

  PointAdaptive P;

  this->mat_base_u_(0, 0) = 6 * u;
  this->mat_base_u_(0, 1) = 2;
  this->mat_base_u_(0, 2) = 0;
  this->mat_base_u_(0, 3) = 0;

  this->mat_base_v_(0, 0) = v * v * v;
  this->mat_base_v_(1, 0) = v * v;
  this->mat_base_v_(2, 0) = v;
  this->mat_base_v_(3, 0) = 1;

  P = CalculatePointUV();
  VectorAdaptive V(P);

  return V;
}

// calcula a derivada parcial Quv nas coordenadas paramétricas u, v
VectorAdaptive PatchBezier::Quv(double u, double v) {
  // Qt:
  //
  //  -> ALOCA um Ponto (mas destroi aqui mesmo)
  //	 -> ALOCA um Vetor (retorna ele)
  //  -> ALTERA a matriz U !!!
  //  -> ALTERA a matriz V !!!
  //

  PointAdaptive P;

  this->mat_base_u_(0, 0) = 3 * u * u;
  this->mat_base_u_(0, 1) = 2 * u;
  this->mat_base_u_(0, 2) = 1;
  this->mat_base_u_(0, 3) = 0;

  this->mat_base_v_(0, 0) = 3 * v * v;
  this->mat_base_v_(1, 0) = 2 * v;
  this->mat_base_v_(2, 0) = 1;
  this->mat_base_v_(3, 0) = 0;

  P = CalculatePointUV();
  VectorAdaptive V(P);

  return V;
}

// calcula a derivada parcial Qvu nas coordenadas paramétricas u, v
VectorAdaptive PatchBezier::Qvu(double u, double v) { return Quv(u, v); }

// calcula a derivada parcial Qvv nas coordenadas paramétricas u, v
VectorAdaptive PatchBezier::Qvv(double u, double v) {
  // Qt:
  //
  //  -> ALOCA um Ponto (mas destroi aqui mesmo)
  //	 -> ALOCA um Vetor (retorna ele)
  //  -> ALTERA a matriz U !!!
  //  -> ALTERA a matriz V !!!
  //

  PointAdaptive P;

  this->mat_base_u_(0, 0) = u * u * u;
  this->mat_base_u_(0, 1) = u * u;
  this->mat_base_u_(0, 2) = u;
  this->mat_base_u_(0, 3) = 1;

  this->mat_base_v_(0, 0) = 6 * v;
  this->mat_base_v_(1, 0) = 2;
  this->mat_base_v_(2, 0) = 0;
  this->mat_base_v_(3, 0) = 0;

  P = CalculatePointUV();
  VectorAdaptive V(P);

  return V;
}

// calcula o vetor tangente na direção u para o ponto p
VectorAdaptive PatchBezier::Qu(const PointAdaptive& p) {
  tuple<double, double> t = this->FindUV(p);
  return this->Qu(get<0>(t), get<1>(t));
}

// calcula o vetor tangente na direção v para o ponto p
VectorAdaptive PatchBezier::Qv(const PointAdaptive& p) {
  tuple<double, double> t = this->FindUV(p);
  return this->Qv(get<0>(t), get<1>(t));
}

// calcula a derivada parcial Quu para o ponto p
VectorAdaptive PatchBezier::Quu(const PointAdaptive& p) {
  tuple<double, double> t = this->FindUV(p);
  return this->Quu(get<0>(t), get<1>(t));
}

// calcula a derivada parcial Quv para o ponto p
VectorAdaptive PatchBezier::Quv(const PointAdaptive& p) {
  tuple<double, double> t = this->FindUV(p);
  return this->Quv(get<0>(t), get<1>(t));
}

// calcula a derivada parcial Qvu para o ponto p
VectorAdaptive PatchBezier::Qvu(const PointAdaptive& p) {
  tuple<double, double> t = this->FindUV(p);
  return this->Qvu(get<0>(t), get<1>(t));
}

// calcula a derivada parcial Qvv para o ponto p
VectorAdaptive PatchBezier::Qvv(const PointAdaptive& p) {
  tuple<double, double> t = this->FindUV(p);
  return this->Qvv(get<0>(t), get<1>(t));
}

Matrix1x4 PatchBezier::GetU() const { return this->mat_base_u_; }

Matrix4x4 PatchBezier::GetGx() const { return this->mat_geo_gx_; }

Matrix4x4 PatchBezier::GetGy() const { return this->mat_geo_gy_; }

Matrix4x4 PatchBezier::GetGz() const { return this->mat_geo_gz_; }

Matrix4x1 PatchBezier::GetV() const { return this->mat_base_v_; }

Matrix4x4 PatchBezier::GetB() const { return this->mat_base_; }

PointAdaptive PatchBezier::GetPt00() const { return this->pt00_; }

PointAdaptive PatchBezier::GetPt01() const { return this->pt01_; }

PointAdaptive PatchBezier::GetPt02() const { return this->pt02_; }

PointAdaptive PatchBezier::GetPt03() const { return this->pt03_; }

PointAdaptive PatchBezier::GetPt10() const { return this->pt10_; }

PointAdaptive PatchBezier::GetPt11() const { return this->pt11_; }

PointAdaptive PatchBezier::GetPt12() const { return this->pt12_; }

PointAdaptive PatchBezier::GetPt13() const { return this->pt13_; }

PointAdaptive PatchBezier::GetPt20() const { return this->pt20_; }

PointAdaptive PatchBezier::GetPt21() const { return this->pt21_; }

PointAdaptive PatchBezier::GetPt22() const { return this->pt22_; }

PointAdaptive PatchBezier::GetPt23() const { return this->pt23_; }

PointAdaptive PatchBezier::GetPt30() const { return this->pt30_; }

PointAdaptive PatchBezier::GetPt31() const { return this->pt31_; }

PointAdaptive PatchBezier::GetPt32() const { return this->pt32_; }

PointAdaptive PatchBezier::GetPt33() const { return this->pt33_; }

void PatchBezier::SetPt00(const PointAdaptive point) { pt00_ = point; }

void PatchBezier::SetPt01(const PointAdaptive point) { pt01_ = point; }

void PatchBezier::SetPt02(const PointAdaptive point) { pt02_ = point; }

void PatchBezier::SetPt03(const PointAdaptive point) { pt03_ = point; }

void PatchBezier::SetPt10(const PointAdaptive point) { pt10_ = point; }

void PatchBezier::SetPt11(const PointAdaptive point) { pt11_ = point; }

void PatchBezier::SetPt12(const PointAdaptive point) { pt12_ = point; }

void PatchBezier::SetPt13(const PointAdaptive point) { pt13_ = point; }

void PatchBezier::SetPt20(const PointAdaptive point) { pt20_ = point; }

void PatchBezier::SetPt21(const PointAdaptive point) { pt21_ = point; }

void PatchBezier::SetPt22(const PointAdaptive point) { pt22_ = point; }

void PatchBezier::SetPt23(const PointAdaptive point) { pt23_ = point; }

void PatchBezier::SetPt30(const PointAdaptive point) { pt30_ = point; }

void PatchBezier::SetPt31(const PointAdaptive point) { pt31_ = point; }

void PatchBezier::SetPt32(const PointAdaptive point) { pt32_ = point; }

void PatchBezier::SetPt33(const PointAdaptive point) { pt33_ = point; }

double PatchBezier::GetNumberTriangle() const { return number_triangle_; }

void PatchBezier::SetNumberTriangle(double value) { number_triangle_ = value; }

double PatchBezier::GetAreaTriangle() const { return area_triangle_; }

void PatchBezier::SetAreaTriangle(double value) { area_triangle_ = value; }

double PatchBezier::GetSegmentMedio() const { return segment_medio_; }

void PatchBezier::SetSegmentMedio(double value) { segment_medio_ = value; }

double PatchBezier::GetKaMedio() const { return ka_medio_; }

void PatchBezier::SetKaMedio(double value) { ka_medio_ = value; }

double PatchBezier::GetArea() const { return area_; }

void PatchBezier::SetArea(double value) { area_ = value; }

int PatchBezier::GetIdProcess() const { return id_process_; }

void PatchBezier::SetIdProcess(int value) { id_process_ = value; }

int PatchBezier::GetIdPatchBezier() const { return id_patch_bezier_; }

void PatchBezier::SetIdPatchBezier(int value) { id_patch_bezier_ = value; }

Matrix4x4 PatchBezier::StartBezierMatrix() {
  Matrix4x4 matrix_bezier;

  matrix_bezier(0, 0) = -1;
  matrix_bezier(0, 1) = 3;
  matrix_bezier(0, 2) = -3;
  matrix_bezier(0, 3) = 1;
  matrix_bezier(1, 0) = 3;
  matrix_bezier(1, 1) = -6;
  matrix_bezier(1, 2) = 3;
  matrix_bezier(1, 3) = 0;
  matrix_bezier(2, 0) = -3;
  matrix_bezier(2, 1) = 3;
  matrix_bezier(2, 2) = 0;
  matrix_bezier(2, 3) = 0;
  matrix_bezier(3, 0) = 1;
  matrix_bezier(3, 1) = 0;
  matrix_bezier(3, 2) = 0;
  matrix_bezier(3, 3) = 0;

  return matrix_bezier;
}
