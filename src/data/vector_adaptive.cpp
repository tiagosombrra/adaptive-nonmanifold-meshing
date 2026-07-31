#include "../../include/data/vector_adaptive.h"

VectorAdaptive::VectorAdaptive() : x_(0), y_(0), z_(0) {}

VectorAdaptive::VectorAdaptive(double x, double y, double z)
    : x_(x), y_(y), z_(z) {}

VectorAdaptive::VectorAdaptive(const VectorAdaptive& vector)
    : x_(vector.x_), y_(vector.y_), z_(vector.z_) {}

VectorAdaptive::VectorAdaptive(const PointAdaptive& point)
    : x_(point.GetX()), y_(point.GetY()), z_(point.GetZ()) {}

VectorAdaptive::VectorAdaptive(const PointAdaptive& p, const PointAdaptive& q)
    : x_(q.GetX() - p.GetX()),
      y_(q.GetY() - p.GetY()),
      z_(q.GetZ() - p.GetZ()) {}

VectorAdaptive VectorAdaptive::operator+(const VectorAdaptive& vector) const {
  VectorAdaptive soma(this->x_ + vector.x_, this->y_ + vector.y_,
                      this->z_ + vector.z_);

  return soma;
}

VectorAdaptive VectorAdaptive::operator-(const VectorAdaptive& vector) const {
  VectorAdaptive subt(this->x_ - vector.x_, this->y_ - vector.y_,
                      this->z_ - vector.z_);

  return subt;
}

VectorAdaptive VectorAdaptive::operator-() const {
  VectorAdaptive invr(-1 * this->x_, -1 * this->y_, -1 * this->z_);

  return invr;
}

double VectorAdaptive::operator^(const VectorAdaptive& vector) const {
  return (this->x_ * vector.x_ + this->y_ * vector.y_ + this->z_ * vector.z_);
}

VectorAdaptive VectorAdaptive::operator^(const double value) const {
  VectorAdaptive prod(value * this->x_, value * this->y_, value * this->z_);
  return prod;
}

VectorAdaptive VectorAdaptive::operator*(const VectorAdaptive& vector) const {
  VectorAdaptive prod(this->y_ * vector.z_ - this->z_ * vector.y_,
                      this->z_ * vector.x_ - this->x_ * vector.z_,
                      this->x_ * vector.y_ - this->y_ * vector.x_);

  return prod;
}

const VectorAdaptive& VectorAdaptive::operator=(const VectorAdaptive& vector) {
  this->x_ = vector.x_;
  this->y_ = vector.y_;
  this->z_ = vector.z_;

  return *this;
}

const VectorAdaptive& VectorAdaptive::operator=(const PointAdaptive& point) {
  this->x_ = point.GetX();
  this->y_ = point.GetY();
  this->z_ = point.GetZ();

  return *this;
}

double VectorAdaptive::CalculateModule() const {
  return (
      sqrt(this->x_ * this->x_ + this->y_ * this->y_ + this->z_ * this->z_));
}

double VectorAdaptive::CalculateAngle(VectorAdaptive& vector) const {
  VectorAdaptive c(this->x_, this->y_, this->z_);

  double cos = static_cast<double>(c ^ vector) /
               (c.CalculateModule() * vector.CalculateModule());

  return (acos((cos > 1.0) ? 1.0 : ((cos < -1.0) ? -1.0 : cos)));
}

VectorAdaptive VectorAdaptive::CalculateUnitVector() {
  double module = this->CalculateModule();

  VectorAdaptive unit_vector(static_cast<double>(this->x_) / module,
                             static_cast<double>(this->y_) / module,
                             static_cast<double>(this->z_) / module);

  return unit_vector;
}

double VectorAdaptive::GetX() const { return x_; }

void VectorAdaptive::SetX(double x) { x_ = x; }

double VectorAdaptive::GetY() const { return y_; }

void VectorAdaptive::SetY(double y) { y_ = y; }

double VectorAdaptive::GetZ() const { return z_; }

void VectorAdaptive::SetZ(double z) { z_ = z; }
