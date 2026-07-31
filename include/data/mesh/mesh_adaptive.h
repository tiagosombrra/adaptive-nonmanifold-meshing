#ifndef DATA_MESH_MESH_ADAPTIVE_H
#define DATA_MESH_MESH_ADAPTIVE_H

#include <memory>
#include <vector>

// Forward declaration to reduce header dependencies.
// (English comment)
class SubMesh;

class MeshAdaptive {
 public:
  MeshAdaptive();
  ~MeshAdaptive();  // Defined in .cpp (SubMesh complete type there)

  // Non-copyable because we own resources.
  // (English comment)
  MeshAdaptive(const MeshAdaptive&) = delete;
  MeshAdaptive& operator=(const MeshAdaptive&) = delete;

  MeshAdaptive(MeshAdaptive&&) noexcept;
  MeshAdaptive& operator=(MeshAdaptive&&) noexcept;

  // Ownership: this class takes ownership of the raw pointer.
  // (English comment)
  void InsertSubMeshAdaptiveByPosition(SubMesh* sub_mesh_adaptive,
                                       const signed int position);
  void InsertSubMeshAdaptive(SubMesh* sub_mesh_adaptive);

  void RemoveSubMeshAdaptive();

  unsigned int GetNumberSubMeshesAdaptive() const noexcept;

  SubMesh* GetSubMeshAdaptiveByPosition(const unsigned int position) noexcept;
  const SubMesh* GetSubMeshAdaptiveByPosition(
      const unsigned int position) const noexcept;

  void ResizeSubMeshAdaptiveByPosition(const signed int new_size);

 protected:
  std::vector<std::unique_ptr<SubMesh>> sub_meshes_;
};

#endif  // DATA_MESH_MESH_ADAPTIVE_H
