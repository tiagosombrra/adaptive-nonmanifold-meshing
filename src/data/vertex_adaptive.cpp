#include "../../include/data/vertex_adaptive.h"

VertexAdaptive::VertexAdaptive() {}

VertexAdaptive::VertexAdaptive(double x, double y, double z)
    : PointAdaptive(x, y, z) {}

VertexAdaptive::VertexAdaptive(PointAdaptive *point) {
  this->x_ = point->GetX();
  this->y_ = point->GetY();
  this->z_ = point->GetZ();
}
