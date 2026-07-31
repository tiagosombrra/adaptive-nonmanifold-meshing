#include "../../include/curvature/curvature_analytical.h"

extern double TOLERANCE;

// Dill, J. C. (1981). An Application of Color Graphics to the Display of
// Surface Curvature. Comp. Graph., Vol. 15, pag. 153-161. SIGGRAPH 81.

CurvatureAnalytical::CurvatureAnalytical(const PointAdaptive& v,
                                         PatchCoons& p) {
  std::tuple<double, double> t = p.FindUV(v);

  this->qu_ = p.Qu(std::get<0>(t), std::get<1>(t));
  this->qv_ = p.Qv(std::get<0>(t), std::get<1>(t));
  this->quu_ = p.Quu(std::get<0>(t), std::get<1>(t));
  this->quv_ = p.Quv(std::get<0>(t), std::get<1>(t));
  this->qvv_ = p.Qvv(std::get<0>(t), std::get<1>(t));
  // Vetor * Vetor -> produto vetorial
  this->prod_ = qu_ * qv_;

  // #pragma omp critical
  //     {
  //         if (std::isnan(get < 0 > ( t )) || std::isnan(get < 1 > ( t ))) {
  //             cout<<"-nan t"<<endl;
  //         }
  //         cout <<
  //         "=========================================================="
  //         << endl; cout << "P - " << v.id << ": ( " << get < 0 > ( t ) << ",
  //         "
  //         << get < 1 > ( t ) << ")" << endl; cout << "Qu ( " << Qu.x << ", "
  //         << Qu.y << ", " << Qu.z << " )" << endl; cout << "Qv ( " << Qv.x <<
  //         ", " << Qv.y << ", " << Qv.z << " )" << endl; cout << "Quu ( " <<
  //         Quu.x << ", " << Quu.y << ", " << Quu.z << " )" << endl; cout <<
  //         "Quv ( " << Quv.x << ", " << Quv.y << ", " << Quv.z << " )" <<
  //         endl; cout << "Qvv ( " << Qvv.x << ", " << Qvv.y << ", " << Qvv.z
  //         << " )"
  //         << endl; if ( 0 == prod.CalculateModule() )
  //             cout << "Qu X Qv = 0 no ponto p" << v.id << " (" << v.x << ", "
  //             << v.y << ", " << v.z << ")" << endl;
  //     }
  //  Vetor ^ Vetor -> produto escalar
  this->a_ = prod_ ^ quu_;
  this->b_ = prod_ ^ quv_;
  this->c_ = prod_ ^ qvv_;

  //	cout << "A = " << this->A << endl;
  //	cout << "B = " << this->B << endl;
  //	cout << "C = " << this->C << endl;
}

double CurvatureAnalytical::CalculateMeanCurvature() {
  // Vetor ^ Vetor -> produto escalar
  double prodModule = prod_.CalculateModule();

  if (prodModule <= TOLERANCE) return 0.0;  // regra de L'Hôpital

  double qvModule = qv_.CalculateModule();
  double quModule = qu_.CalculateModule();

  // H = ( A.|Qv|² - 2.B.Qu.Qv + C.|Qu|² ) / ( 2.| Qu x Qv |³ )
  double resultado =
      (static_cast<double>(a_ * qvModule * qvModule - 2 * b_ * (qu_ ^ qv_) +
                           c_ * quModule * quModule) /
       (2 * pow(prodModule, 3)));

  // cout << "Ha = " << resultado << endl;
  // cout << "==========================================================" <<
  // endl;

  return (fabs(resultado) <= TOLERANCE) ? 0.0 : resultado;
}

double CurvatureAnalytical::CalculateGaussCurvature() {
  double prodModule = prod_.CalculateModule();

  if (prodModule <= TOLERANCE) return 0.0;  // regra de L'Hôpital

  // K = ( A.C - B² ) / | Qu x Qv |⁴
  double resultado =
      static_cast<double>(this->a_ * this->c_ - (this->b_ * this->b_)) /
      pow(prodModule, 4);

  // cout << "Ga = " << resultado << endl;

  return (fabs(resultado) <= TOLERANCE) ? 0.0 : resultado;
}
