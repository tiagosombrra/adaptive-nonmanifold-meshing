#include "../../include/parallel/Transferable.h"

using namespace Parallel;

Parallel::Transferable::Transferable(UInt type_value) {
  this->setType(type_value);
}

Parallel::Transferable::~Transferable() {}

void Parallel::Transferable::setType(UInt type_value) {
  this->type = type_value;
}

UInt Parallel::Transferable::getType() { return this->type; }

void Parallel::Transferable::free(Package &p) { delete[] p.second; }
