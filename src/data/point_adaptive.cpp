#include "../../include/data/point_adaptive.h"

#include <iostream>

extern double EPSYLON;

PointAdaptive::PointAdaptive() : id_(-1), x_(0), y_(0), z_(0) {}

PointAdaptive::PointAdaptive(double x, double y, double z)
    : id_(0), x_(x), y_(y), z_(z) {}

PointAdaptive::PointAdaptive(double x, double y, double z, unsigned long id)
    : id_(id), x_(x), y_(y), z_(z) {}

unsigned long PointAdaptive::GetId() const { return id_; }

void PointAdaptive::SetId(unsigned long id) { id_ = id; }

double PointAdaptive::GetX() const { return x_; }

void PointAdaptive::SetX(double x) { x_ = x; }

double PointAdaptive::GetY() const { return y_; }

double PointAdaptive::GetZ() const { return z_; }

void PointAdaptive::SetZ(double z) { z_ = z; }

void PointAdaptive::SetY(double y) { y_ = y; }

bool PointAdaptive::operator==(const PointAdaptive& point) const {
  if ((fabs(this->x_ - point.x_) <= EPSYLON) &&
      (fabs(this->y_ - point.y_) <= EPSYLON) &&
      (fabs(this->z_ - point.z_) <= EPSYLON)) {
    return true;
  }

  return false;
}

bool PointAdaptive::operator==(const PointAdaptive* point) const {
  if ((fabs(this->x_ - point->x_) <= EPSYLON) &&
      (fabs(this->y_ - point->y_) <= EPSYLON) &&
      (fabs(this->z_ - point->z_) <= EPSYLON))

  {
    return true;
  }

  return false;
}

double PointAdaptive::CalculateDistance(const PointAdaptive& point) const {
  return (sqrt(pow((point.x_ - this->x_), 2.0) +
               pow((point.y_ - this->y_), 2.0) +
               pow((point.z_ - this->z_), 2.0)));
}

void PointAdaptive::PrintPoint() const {
  std::cout << "Ponto " << this->id_ << " = ( " << this->x_ << ", " << this->y_
            << ", " << this->z_ << ")" << std::endl;
}
