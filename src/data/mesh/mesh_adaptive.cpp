#include "data/mesh/mesh_adaptive.h"

#include "data/mesh/sub_mesh.h"

#include <cstddef>
#include <utility>

MeshAdaptive::MeshAdaptive() = default;
MeshAdaptive::~MeshAdaptive() = default;

MeshAdaptive::MeshAdaptive(MeshAdaptive&&) noexcept = default;
MeshAdaptive& MeshAdaptive::operator=(MeshAdaptive&&) noexcept = default;

void MeshAdaptive::InsertSubMeshAdaptiveByPosition(SubMesh* sub_mesh_adaptive,
                                                   const signed int position) {
  if (position < 0) {
    // Invalid index; ignore to keep behavior safe.
    // (English comment)
    return;
  }

  const std::size_t idx = static_cast<std::size_t>(position);

  if (idx >= sub_meshes_.size()) {
    sub_meshes_.resize(idx + 1);  // new slots become nullptr unique_ptr
  }

  // Takes ownership. Replaces existing item (old one is deleted automatically).
  // (English comment)
  sub_meshes_[idx].reset(sub_mesh_adaptive);
}

void MeshAdaptive::InsertSubMeshAdaptive(SubMesh* sub_mesh_adaptive) {
  // Takes ownership.
  // (English comment)
  sub_meshes_.emplace_back(sub_mesh_adaptive);
}

void MeshAdaptive::RemoveSubMeshAdaptive() {
  if (!sub_meshes_.empty()) {
    // Automatically deletes the last SubMesh if not null.
    // (English comment)
    sub_meshes_.pop_back();
  }
}

unsigned int MeshAdaptive::GetNumberSubMeshesAdaptive() const noexcept {
  return static_cast<unsigned int>(sub_meshes_.size());
}

SubMesh* MeshAdaptive::GetSubMeshAdaptiveByPosition(
    const unsigned int position) noexcept {
  return (position < sub_meshes_.size()) ? sub_meshes_[position].get()
                                         : nullptr;
}

const SubMesh* MeshAdaptive::GetSubMeshAdaptiveByPosition(
    const unsigned int position) const noexcept {
  return (position < sub_meshes_.size()) ? sub_meshes_[position].get()
                                         : nullptr;
}

void MeshAdaptive::ResizeSubMeshAdaptiveByPosition(const signed int new_size) {
  if (new_size < 0) {
    // Negative size doesn't make sense; clear to a safe state.
    // (English comment)
    sub_meshes_.clear();
    return;
  }

  sub_meshes_.resize(static_cast<std::size_t>(new_size));
}
