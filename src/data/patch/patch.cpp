#include "data/patch/patch.h"

Patch::Patch() = default;

Patch::Patch(Patch* patch) : id_(patch ? patch->GetId() : 0) {}

Patch::~Patch() = default;

Patch::Patch(Patch&&) noexcept = default;
Patch& Patch::operator=(Patch&&) noexcept = default;

unsigned int Patch::GetId() const noexcept {
  return id_;
}

void Patch::SetId(unsigned int id) noexcept {
  id_ = id;
}

SubMesh* Patch::GetSubMesh() const noexcept {
  return sub_mesh_;
}

void Patch::SetSubMesh(SubMesh* sub_mesh) noexcept {
  sub_mesh_ = sub_mesh;
}
