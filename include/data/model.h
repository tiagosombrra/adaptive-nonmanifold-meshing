#ifndef DATA_MODEL_H
#define DATA_MODEL_H

#include <vector>

#include "../data/mesh/mesh_adaptive.h"
#include "geometry.h"

class Model {
 public:
  Model();
  explicit Model(Geometry* geometry);
  ~Model();

  void SetGeometry(Geometry* geometry);
  Geometry* GetGeometry();
  void InsertMeshAdaptive(MeshAdaptive* mesh_adaptive);
  unsigned int GetNumberMeshes() const;
  MeshAdaptive* GetMeshAdaptive(const unsigned int position);

 protected:
  Geometry* geometry_;
  std::vector<MeshAdaptive*> meshes_;
};

#endif  // DATA_MODEL_H
