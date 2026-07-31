#ifndef DATA_PATCH_PATCH_H
#define DATA_PATCH_PATCH_H

class SubMesh;

class Patch {
 public:
  Patch();
  explicit Patch(Patch* patch);
  virtual ~Patch();  // In .cpp

  Patch(const Patch&) = delete;
  Patch& operator=(const Patch&) = delete;

  Patch(Patch&&) noexcept;
  Patch& operator=(Patch&&) noexcept;

  unsigned int GetId() const noexcept;
  void SetId(unsigned int id) noexcept;

  SubMesh* GetSubMesh() const noexcept;
  void SetSubMesh(SubMesh* sub_mesh) noexcept;

 protected:
  unsigned int id_ = 0;
  SubMesh* sub_mesh_ = nullptr;  // non-owning (MeshAdaptive owns it)
};

#endif  // DATA_PATCH_PATCH_H
