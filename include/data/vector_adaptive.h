#ifndef DATA_VECTOR_ADAPTIVE_H
#define DATA_VECTOR_ADAPTIVE_H

#include <cmath>

#include "point_adaptive.h"

class VectorAdaptive {
 public:
  VectorAdaptive();
  VectorAdaptive(double x, double y, double z);
  VectorAdaptive(const VectorAdaptive& vector);
  explicit VectorAdaptive(const PointAdaptive& point);
  // constrói um vetor entre dois pontos
  VectorAdaptive(const PointAdaptive& p, const PointAdaptive& q);

  // soma vetorial
  VectorAdaptive operator+(const VectorAdaptive& vector) const;
  // diferença vetorial
  VectorAdaptive operator-(const VectorAdaptive& vector) const;
  // vetor inverso
  VectorAdaptive operator-() const;
  // produto escalar
  double operator^(const VectorAdaptive& vector) const;
  // produto por escalar
  VectorAdaptive operator^(const double value) const;
  // produto vetorial
  VectorAdaptive operator*(const VectorAdaptive& vector) const;

  const VectorAdaptive& operator=(const VectorAdaptive& vector);
  const VectorAdaptive& operator=(const PointAdaptive& point);

  double CalculateModule() const;
  // ângulo entre dois vetores
  double CalculateAngle(VectorAdaptive& vector) const;
  // vetor unitário
  VectorAdaptive CalculateUnitVector();

  double GetX() const;
  void SetX(double x);

  double GetY() const;
  void SetY(double y);

  double GetZ() const;
  void SetZ(double z);

 protected:
  // coordenadas
  double x_;
  double y_;
  double z_;
};
#endif  // DATA_VECTOR_ADAPTIVE_H
