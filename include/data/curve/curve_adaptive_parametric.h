#ifndef DATA_CURVE_CURVE_ADAPTIVE_PARAMETRIC_H
#define DATA_CURVE_CURVE_ADAPTIVE_PARAMETRIC_H

#include <iostream>
#include <list>

#include "../../crab_mesh/numerical/bisection_equation_root.h"
#include "../../crab_mesh/numerical/equation_root_function.h"
#include "../../data/definitions.h"
#include "../vector_adaptive.h"
#include "../vertex_adaptive.h"
#include "curve_adaptive.h"

class CurveAdaptiveParametric : public CurveAdaptive {
 public:
  // IMPORTANTE: Ao usar o construtor default, não esquecer de setar
  // os pontos inicial e final e os vetores tangentes inicial e final !!!
  CurveAdaptiveParametric();
  // Esse construtor é mais recomendado !!! Evita erros ao esquecer de setar
  // os valores indicados na observação do construtor default !!!
  CurveAdaptiveParametric(const PointAdaptive point0,
                          const PointAdaptive point1);
  explicit CurveAdaptiveParametric(CurveAdaptiveParametric*);
  ~CurveAdaptiveParametric();

  // calcula o comprimento de curva de p1 a p2
  double CalculateLengthPoints(const PointAdaptive& point1,
                               const PointAdaptive& point2) override;
  // calcula o comprimento de curva até p
  double CalculateLengthPoint(const PointAdaptive& point) override;
  // calcula comprimento total "L" da curva
  void CalculateLengthCurve() override;
  // calcula a curvatuta da curva
  double CalculateCurvature(double) override;
  // calcula o comprimento de curva entre os parâmetros t1 e t2
  double CalculateParametricLength(double parameter1, double parameter2);
  // encontra o parâmetro t de um dado ponto p na curva, ou do ponto na
  // curva mais próximo de p
  double FindParameterByPoint(const PointAdaptive& point);
  // encontra o ponto p na curva dado um parâmetro t
  PointAdaptive FindPointByParameter(double t);
  // calcula o gradiente no parâmetro t
  VectorAdaptive CalculateGradientByParameter(double t);
  // retorna o ponto que fica na metade da curva
  PointAdaptive CalculateMidpointByPoints(const PointAdaptive& point1,
                                          const PointAdaptive& point2);
  // retorna o ponto que fica na metade da curva
  double CalculateMidparameterByParamters(const double parameter1,
                                          const double parameter2);
  // ordena a lista de pontos de acordo com suas coordenadas paramétricas
  void SortPointsByParameters();
  void UpdateParameters(const list<double> new_parameters);

  Matrix4x1 GetMatGeoGx() const;
  void SetMatGeoGx(const Matrix4x1& mat_geo_gx);

  Matrix4x1 GetMatGeoGy() const;
  void SetMatGeoGy(const Matrix4x1& mat_geo_gy);

  Matrix4x1 GetMatGeoGz() const;
  void SetMatGeoGz(const Matrix4x1& mat_geo_gz);

  Matrix4x4 GetMatBase() const;
  void SetMatBase(Matrix4x4& mat_base);

  Matrix1x4 GetMatParameters() const;
  void SetMatParameters(const Matrix1x4& mat_parameters);

  PointAdaptive GetPoint0() const;
  void SetPoint0(const PointAdaptive& point0);

  PointAdaptive GetPoint1() const;
  void SetPoint1(const PointAdaptive& point1);

  void SetPoint0Point1(const PointAdaptive& point0,
                       const PointAdaptive& point1);

  // parâmetros dos pontos
  list<double> parameters_;

 protected:
  // retorna o ponto calculado por T x M x Gx,y,z
  PointAdaptive CalculatePointT();
  double CalculateLength(double t1, double t2, int parts, int points);
  double CalculateMidpointBisection(double parameter1, double parameter2);

  // ponto inicial
  PointAdaptive point0_;
  // ponto final
  PointAdaptive point1_;
  // matrizes geométricas
  Matrix4x1 mat_geo_gx_;
  Matrix4x1 mat_geo_gy_;
  Matrix4x1 mat_geo_gz_;
  // matriz característica da curva
  Matrix4x4 mat_base_;
  // matriz dos parâmetros
  Matrix1x4 mat_parameters_;
};
#endif  // DATA_CURVE_CURVE_ADAPTIVE_PARAMETRIC_H
