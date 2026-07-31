#ifndef INPUT_OUTPUT_WRITE_OBJ_FILE_H
#define INPUT_OUTPUT_WRITE_OBJ_FILE_H

#include <ctime>
#include <fstream>
#include <sstream>
#include <vector>

#include "../data/mesh/mesh_adaptive.h"
#include "../data/mesh/sub_mesh.h"
#include "../data/triangle_adaptive.h"

extern std::string NAME_MODEL;

class WriteOBJFile {
 public:
  WriteOBJFile();
  bool WriteMeshOBJFile(MeshAdaptive* mesh, unsigned int step, int process);
  static void WriteCurvaturePatches(const std::vector<double>& patches,
                                    double max_value);
  ~WriteOBJFile();
};

#endif  // INPUT_OUTPUT_WRITE_OBJ_FILE_H
