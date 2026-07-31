#include "config/runtime_config.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace {

bool RejectsSemanticallyEmptyModel(const std::string& case_name,
                                   const std::string& model_contents) {
  const std::filesystem::path root =
      std::filesystem::temp_directory_path() /
      ("ap_mesh_runtime_config_" + case_name);
  std::error_code cleanup_error;
  std::filesystem::remove_all(root, cleanup_error);
  std::filesystem::create_directories(root);

  const std::filesystem::path model = root / "model.bp";
  const std::filesystem::path config = root / "case.conf";
  {
    std::ofstream output(model);
    output << model_contents;
  }
  {
    std::ofstream output(config);
    output << "INPUT_MODEL=" << model.generic_string() << "\n"
           << "OUTPUT_PREFIX=" << (root / "output").generic_string() << "\n"
           << "NUM_PROCESSES=1\n"
           << "NUM_THREADS=1\n";
  }

  bool rejected = false;
  try {
    static_cast<void>(apmesh::LoadRuntimeConfig(config.generic_string()));
  } catch (const apmesh::ConfigError& exception) {
    rejected = std::string(exception.what()).find("contains no model data") !=
               std::string::npos;
  }

  std::filesystem::remove_all(root, cleanup_error);
  return rejected;
}

}  // namespace

int main() {
  const std::vector<std::string> configurations = {
      "configs/book/ablation_full.conf",
      "configs/book/ablation_no_hybrid.conf",
      "configs/book/ablation_no_sync.conf",
      "configs/book/article.conf",
      "configs/book/smoke.conf",
      "configs/decor_shelf/ablation_full.conf",
      "configs/decor_shelf/ablation_no_hybrid.conf",
      "configs/decor_shelf/ablation_no_sync.conf",
      "configs/decor_shelf/article.conf",
      "configs/decor_shelf/smoke.conf",
      "configs/eistute/ablation_full.conf",
      "configs/eistute/ablation_no_hybrid.conf",
      "configs/eistute/ablation_no_sync.conf",
      "configs/eistute/article.conf",
      "configs/eistute/smoke.conf",
  };

  try {
    for (const std::string& path : configurations) {
      const apmesh::RuntimeConfig config = apmesh::LoadRuntimeConfig(path);
      if (config.input_model.empty() || config.output_prefix.empty() ||
          config.num_processes < 1 || config.num_threads < 1) {
        std::cerr << "Invalid parsed configuration: " << path << std::endl;
        return 1;
      }
    }
  } catch (const std::exception& exception) {
    std::cerr << exception.what() << std::endl;
    return 1;
  }

  const std::vector<std::pair<std::string, std::string>> invalid_models = {
      {"empty", ""},
      {"comments_only", "  \n# no geometry\n// no patches\n\t\n"},
  };
  for (const auto& [case_name, contents] : invalid_models) {
    if (!RejectsSemanticallyEmptyModel(case_name, contents)) {
      std::cerr << "Semantically empty model was accepted: " << case_name
                << std::endl;
      return 1;
    }
  }

  std::cout << "Validated maintained configurations: "
            << configurations.size() << "; rejected semantic-empty models: "
            << invalid_models.size() << std::endl;
  return 0;
}
