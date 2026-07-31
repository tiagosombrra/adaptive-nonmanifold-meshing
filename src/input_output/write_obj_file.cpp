#include "../../include/input_output/write_obj_file.h"

WriteOBJFile::WriteOBJFile() = default;

WriteOBJFile::~WriteOBJFile() = default;

void WriteOBJFile::WriteCurvaturePatches(
    const std::vector<double>& patches, double max_value) {
  std::stringstream name_file;
  name_file << NAME_MODEL + "_analise_curvature_patches.log";

  std::ofstream file(name_file.str());

  file << "File Analise Curvature" << std::endl << std::endl;

  std::vector<double> vec_0_10;
  std::vector<double> vec_10_20;
  std::vector<double> vec_20_30;
  std::vector<double> vec_30_40;
  std::vector<double> vec_40_50;
  std::vector<double> vec_50_60;
  std::vector<double> vec_60_70;
  std::vector<double> vec_70_80;
  std::vector<double> vec_80_90;
  std::vector<double> vec_90_100;

  for (auto it = patches.cbegin(); it != patches.cend(); ++it) {
    const double value = (*it) / max_value;
    if (0.0 <= value && value <= 0.1) {
      vec_0_10.push_back(value);
    } else if (0.1 < value && value <= 0.2) {
      vec_10_20.push_back(value);
    } else if (0.2 < value && value <= 0.3) {
      vec_20_30.push_back(value);
    } else if (0.3 < value && value <= 0.4) {
      vec_30_40.push_back(value);
    } else if (0.4 < value && value <= 0.5) {
      vec_40_50.push_back(value);
    } else if (0.5 < value && value <= 0.6) {
      vec_50_60.push_back(value);
    } else if (0.6 < value && value <= 0.7) {
      vec_60_70.push_back(value);
    } else if (0.7 < value && value <= 0.8) {
      vec_70_80.push_back(value);
    } else if (0.8 < value && value <= 0.9) {
      vec_80_90.push_back(value);
    } else if (0.9 < value && value <= 1) {
      vec_90_100.push_back(value);
    }
  }

  const double patch_count = static_cast<double>(patches.size());
  file << static_cast<double>(vec_0_10.size()) / patch_count << std::endl;
  file << static_cast<double>(vec_10_20.size()) / patch_count << std::endl;
  file << static_cast<double>(vec_20_30.size()) / patch_count << std::endl;
  file << static_cast<double>(vec_30_40.size()) / patch_count << std::endl;
  file << static_cast<double>(vec_40_50.size()) / patch_count << std::endl;
  file << static_cast<double>(vec_50_60.size()) / patch_count << std::endl;
  file << static_cast<double>(vec_60_70.size()) / patch_count << std::endl;
  file << static_cast<double>(vec_70_80.size()) / patch_count << std::endl;
  file << static_cast<double>(vec_80_90.size()) / patch_count << std::endl;
  file << static_cast<double>(vec_90_100.size()) / patch_count << std::endl;

  file.flush();
  file.close();
}

bool WriteOBJFile::WriteMeshOBJFile(MeshAdaptive* mesh, unsigned int step,
                                    int process) {
  std::stringstream name_file;
  name_file << "mesh_";
  name_file << step;
  name_file << "_process_";
  name_file << process;
  name_file << ".obj";

  std::ofstream file(name_file.str());

  file << " File Wavefront OBJ generated apMesh" << std::endl << std::endl;

  const std::time_t timestamp = std::time(nullptr);
  std::tm* now = std::localtime(&timestamp);
  file << "# File created: " << (now->tm_year + 1900) << '-'
       << (now->tm_mon + 1) << '-' << now->tm_mday << std::endl;

  unsigned long int Nv, Nt;
  Nv = Nt = 0;

  for (unsigned int i = 0; i < mesh->GetNumberSubMeshesAdaptive(); ++i) {
    SubMesh* sub = mesh->GetSubMeshAdaptiveByPosition(i);

    Nv += sub->GetNumberNos();
    Nt += sub->GetNumberElements();
  }

  file << "# of vertices" << std::endl << Nv << std::endl << std::endl;

  for (unsigned int i = 0; i < mesh->GetNumberSubMeshesAdaptive(); ++i) {
    SubMesh* sub = mesh->GetSubMeshAdaptiveByPosition(i);

    for (unsigned int j = 0; j < sub->GetNumberNos(); ++j) {
      NodeAdaptive* n = sub->GetNoh(j);
      file << "v " << n->GetX() << " " << n->GetY() << " " << n->GetZ()
           << std::endl;
    }
  }

  file << "# of faces " << std::endl << Nt << std::endl;

  for (unsigned int i = 0; i < mesh->GetNumberSubMeshesAdaptive(); ++i) {
    SubMesh* sub = mesh->GetSubMeshAdaptiveByPosition(i);

    for (unsigned int j = 0; j < sub->GetNumberElements(); ++j) {
      auto* triangle =
          static_cast<TriangleAdaptive*>(sub->GetElement(j));
      file << "f " << triangle->GetNoh(1).GetId() << " "
           << triangle->GetNoh(2).GetId() << " "
           << triangle->GetNoh(3).GetId() << std::endl;
    }
  }

  file.flush();
  file.close();

  return true;
}
