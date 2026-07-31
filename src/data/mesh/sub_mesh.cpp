#include "data/mesh/sub_mesh.h"

#include "data/node_adaptive.h"
#include "data/element_adaptive.h"
#include <memory>

SubMesh::SubMesh() = default;
SubMesh::~SubMesh() = default;

SubMesh::SubMesh(SubMesh&&) noexcept = default;
SubMesh& SubMesh::operator=(SubMesh&&) noexcept = default;

Patch* SubMesh::GetPatch() const noexcept {
  return patch_;
}

void SubMesh::SetPatch(Patch* patch) noexcept {
  patch_ = patch;
}

NodeAdaptive* SubMesh::GetNoh(unsigned int position) noexcept {
  return (position < nos_.size()) ? nos_[position].get() : nullptr;
}

const NodeAdaptive* SubMesh::GetNoh(unsigned int position) const noexcept {
  return (position < nos_.size()) ? nos_[position].get() : nullptr;
}

ElementAdaptive* SubMesh::GetElement(unsigned int position) noexcept {
  return (position < elements_.size()) ? elements_[position].get() : nullptr;
}

const ElementAdaptive* SubMesh::GetElement(unsigned int position) const noexcept {
  return (position < elements_.size()) ? elements_[position].get() : nullptr;
}

void SubMesh::AddNoh(std::unique_ptr<NodeAdaptive> noh) {
  nos_.push_back(std::move(noh));
}

void SubMesh::AddElement(std::unique_ptr<ElementAdaptive> element) {
  elements_.push_back(std::move(element));
}

void SubMesh::SetNoh(NodeAdaptive* noh) {
  if (!noh) return;
  auto copy = std::make_unique<NodeAdaptive>(*noh);
  copy->SetId(noh->GetId());
  nos_.push_back(std::move(copy));
}

void SubMesh::SetElement(ElementAdaptive* element) {
  // Takes ownership of element (must be allocated with new).
  // (English comment)
  elements_.emplace_back(element);
}

unsigned int SubMesh::GetNumberNos() const noexcept {
  return static_cast<unsigned int>(nos_.size());
}

unsigned int SubMesh::GetNumberElements() const noexcept {
  return static_cast<unsigned int>(elements_.size());
}
