/* Classe que define um triangulo
 */
#include "../../include/data/triangle_adaptive.h"

#include <iostream>

extern double TOLERANCE;

TriangleAdaptive::TriangleAdaptive()
    : ElementAdaptive(),
      n1_(nullptr),
      n2_(nullptr),
      n3_(nullptr),
      angle_n1_(0),
      angle_n2_(0),
      angle_n3_(0) {}

TriangleAdaptive::TriangleAdaptive(NodeAdaptive* n1, NodeAdaptive* n2,
                                   NodeAdaptive* n3)
    : ElementAdaptive() {
  this->n1_ = n1;
  this->n2_ = n2;
  this->n3_ = n3;

  this->n1_->InsertElement(this);
  this->n2_->InsertElement(this);
  this->n3_->InsertElement(this);

  this->angle_n1_ = CalculateAngleN1();
  this->angle_n2_ = CalculateAngleN2();
  this->angle_n3_ = CalculateAngleN3();

  TriangleAdaptive::CalculateNormal();
  TriangleAdaptive::CalculateArea();
}

TriangleAdaptive::~TriangleAdaptive() {}

// retorna o i-ésimo nó
NodeAdaptive TriangleAdaptive::GetNoh(unsigned const int position) const {
  if (position == 1) {
    return *(this->n1_);
  } else if (position == 2) {
    return *(this->n2_);
  } else if (position == 3) {
    return *(this->n3_);
  } else {
    std::cout << "Índice inválido para Noh Triangulo::GetNoh ( " << position
              << " )!" << std::endl;
  }

  NodeAdaptive n;

  return n;
}

// calcula o ângulo no nó 1
double TriangleAdaptive::CalculateAngleN1() {
  VectorAdaptive vector1(*(this->n1_), *(this->n2_));
  VectorAdaptive vector2(*(this->n1_), *(this->n3_));

  return vector1.CalculateAngle(vector2);
}

// calcula o ângulo no nó 2
double TriangleAdaptive::CalculateAngleN2() {
  VectorAdaptive vector1(*(this->n2_), *(this->n3_));
  VectorAdaptive vector2(*(this->n2_), *(this->n1_));

  return vector1.CalculateAngle(vector2);
}

// calcula o ângulo no nó 3
double TriangleAdaptive::CalculateAngleN3() {
  VectorAdaptive vector1(*(this->n3_), *(this->n1_));
  VectorAdaptive vector2(*(this->n3_), *(this->n2_));

  return vector1.CalculateAngle(vector2);
}

// retorna o ângulo do nó n
double TriangleAdaptive::GetAngle(const NodeAdaptive& node) {
  if (this->n1_->CalculateDistance(node) <= TOLERANCE) {
    return this->angle_n1_;
  }

  if (this->n2_->CalculateDistance(node) <= TOLERANCE) {
    return this->angle_n2_;
  }

  if (this->n3_->CalculateDistance(node) <= TOLERANCE) {
    return this->angle_n3_;
  }

  return -1;  // erro
}

// calcula a área do triângulo
void TriangleAdaptive::CalculateArea() {
  this->area_ = 0.5 * (this->vector_normal_.CalculateModule());
}

// calcula a normal do triângulo
void TriangleAdaptive::CalculateNormal() {
  VectorAdaptive vector1(*(this->n1_), *(this->n2_));
  VectorAdaptive vector2(*(this->n1_), *(this->n3_));

  this->vector_normal_ = vector1 * vector2;
}

void TriangleAdaptive::ReplaceNode(const NodeAdaptive* old_node,
                                   NodeAdaptive* new_node) {
  if (this->n1_ == old_node) {
    this->n1_ = new_node;
  } else if (this->n2_ == old_node) {
    this->n2_ = new_node;
  } else if (this->n3_ == old_node) {
    this->n3_ = new_node;
  }
}

double TriangleAdaptive::CalculateQualityTriangle() {
  double a1 = n1_->CalculateDistance(static_cast<PointAdaptive>(*n2_));
  double b2 = n2_->CalculateDistance(static_cast<PointAdaptive>(*n3_));
  double c3 = n3_->CalculateDistance(static_cast<PointAdaptive>(*n1_));

  double s = 0.5 * (a1 + b2 + c3);
  double d = (s - a1) * (s - b2) * (s - c3);

  double ray_inscribed = std::sqrt(d / s);
  double ray_circumscribed = 0.25 * a1 * b2 * c3 / sqrt(s * d);

  return 2.0 * ray_inscribed / ray_circumscribed;
}

const std::tuple<double, double>& TriangleAdaptive::GetParametersN1() const {
  return parameters_n1_;
}

void TriangleAdaptive::SetParametersN1(
    const std::tuple<double, double>& parameters_n1) {
  parameters_n1_ = parameters_n1;
}

const std::tuple<double, double>& TriangleAdaptive::GetParametersN2() const {
  return parameters_n2_;
}

void TriangleAdaptive::SetParametersN2(
    const std::tuple<double, double>& parameters_n2) {
  parameters_n2_ = parameters_n2;
}

const std::tuple<double, double>& TriangleAdaptive::GetParametersN3() const {
  return parameters_n3_;
}

void TriangleAdaptive::SetParametersN3(
    const std::tuple<double, double>& parameters_n3) {
  parameters_n3_ = parameters_n3;
}
