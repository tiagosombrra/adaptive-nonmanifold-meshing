#ifndef DATA_MESH_SUB_MESH_H
#define DATA_MESH_SUB_MESH_H

#include <memory>
#include <vector>

class Patch;
class NodeAdaptive;
class ElementAdaptive;

class SubMesh {
 public:
  SubMesh();
  ~SubMesh();  // Defined in .cpp to keep forward declarations valid.

  // Non-copyable to avoid double free.
  // (English comment)
  SubMesh(const SubMesh&) = delete;
  SubMesh& operator=(const SubMesh&) = delete;

  SubMesh(SubMesh&&) noexcept;
  SubMesh& operator=(SubMesh&&) noexcept;

  Patch* GetPatch() const noexcept;
  void SetPatch(Patch* patch) noexcept;

  NodeAdaptive* GetNoh(unsigned int position) noexcept;
  const NodeAdaptive* GetNoh(unsigned int position) const noexcept;

  ElementAdaptive* GetElement(unsigned int position) noexcept;
  const ElementAdaptive* GetElement(unsigned int position) const noexcept;

  void AddNoh(std::unique_ptr<NodeAdaptive> noh);
  void AddElement(std::unique_ptr<ElementAdaptive> element);

  // Compatibility API: takes ownership of raw pointers.
  // (English comment)
  void SetNoh(NodeAdaptive* noh);
  void SetElement(ElementAdaptive* element);

  unsigned int GetNumberNos() const noexcept;
  unsigned int GetNumberElements() const noexcept;

 private:
  Patch* patch_ = nullptr;  // non-owning

  std::vector<std::unique_ptr<NodeAdaptive>> nos_;
  std::vector<std::unique_ptr<ElementAdaptive>> elements_;
};

#endif  // DATA_MESH_SUB_MESH_H
