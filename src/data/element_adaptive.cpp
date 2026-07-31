#include "../../include/data/element_adaptive.h"

ElementAdaptive::ElementAdaptive() : id_(0), area_(0) {}

ElementAdaptive::~ElementAdaptive() {}

void ElementAdaptive::SetId(const unsigned long id) { this->id_ = id; }

unsigned long ElementAdaptive::GetId() const { return this->id_; }

double ElementAdaptive::GetArea() const { return this->area_; }

VectorAdaptive ElementAdaptive::GetVectorNormal() const {
  return this->vector_normal_;
}
