#include "../../include/input_output/file.h"

File::File(const char* name) {
  INPUT_MODEL.open(name);
  if (INPUT_MODEL.fail()) {
    std::cout << "não abriu o arquivo em disco!" << std::endl;
  } else {
    name_ = name;
  }
}

File::~File() { INPUT_MODEL.close(); }

// ler as linhas que definem as curvas
void File::ReadCurves(const std::string& reading) {
  // "P" de point_1_ ou P2, os pontos inicial e final da curva
  // "D" de DP1 ou DP2, as derivadas nos pontos inicial e final
  if (!reading.empty() && (reading[0] == 'P' || reading[0] == 'D')) {
    curves_.push_back(reading);
  }
}

// ler as linhas que definem os patches
void File::ReadPatches(const std::string& reading) {
  // "D" de DEFINE_PATCH
  if (!reading.empty() && reading[0] == 'D') {
    patches_.push_back(reading);
  }
}

// converta uma string em um char*, por causa do strtok() do C
char* File::ConvertString(const std::string& font) {
  auto* destiny = new char[font.size() + 1];
  std::memcpy(destiny, font.c_str(), font.size() + 1);
  return destiny;
}

std::string File::GetName() const { return name_; }

// criar as curvas
void File::CreateCurvesTo() {
  char* temp = nullptr;
  char* str = nullptr;
  double pt0[3];   // ponto inicial
  double pt1[3];   // ponto final
  double dpt0[3];  // derivada no ponto inicial
  double dpt1[3];  // derivada no ponto final

  auto itr = curves_.begin();
  const auto fim = curves_.end();

  // leia quatro strings da list para definir uma curva
  while (itr != fim) {
    // lê o ponto inicial
    temp = ConvertString(*itr);
    (void)strtok(temp, " <");
    str = strtok(nullptr, "<,");
    pt0[0] = atof(str);
    str = strtok(nullptr, ",,");
    pt0[1] = atof(str);
    str = strtok(nullptr, ",>");
    pt0[2] = atof(str);
    ++itr;

    delete[] temp;

    // lê o ponto final
    temp = ConvertString(*itr);
    (void)strtok(temp, " <");
    str = strtok(nullptr, "<,");
    pt1[0] = atof(str);
    str = strtok(nullptr, ",,");
    pt1[1] = atof(str);
    str = strtok(nullptr, ",>");
    pt1[2] = atof(str);
    ++itr;

    delete[] temp;

    // lê a derivada no ponto inicial
    temp = ConvertString(*itr);
    (void)strtok(temp, " <");
    str = strtok(nullptr, "<,");
    dpt0[0] = atof(str);
    str = strtok(nullptr, ",,");
    dpt0[1] = atof(str);
    str = strtok(nullptr, ",>");
    dpt0[2] = atof(str);
    ++itr;

    delete[] temp;

    // lê a derivada no ponto final
    temp = ConvertString(*itr);
    (void)strtok(temp, " <");
    str = strtok(nullptr, "<,");
    dpt1[0] = atof(str);
    str = strtok(nullptr, ",,");
    dpt1[1] = atof(str);
    str = strtok(nullptr, ",>");
    dpt1[2] = atof(str);
    ++itr;

    delete[] temp;

    // substituir pelo construtor de curvas
    std::cout << "\nContrui uma curva com ponto inicial ( " << pt0[0] << ", "
              << pt0[1] << ", " << pt0[2] << ")\n"
              << "ponto final: (" << pt1[0] << ", " << pt1[1] << ", "
              << pt1[2] << ")\n"
              << "Derivada no ponto inicial: (" << dpt0[0] << ", " << dpt0[1]
              << ", " << dpt0[2] << ")\n"
              << "Derivada no ponto final: (" << dpt1[0] << ", " << dpt1[1]
              << ", " << dpt1[2] << ")" << std::endl;
  }
}

// criar os patches
void File::CreatePatchesTo() {
  char* temp = nullptr;
  char* str = nullptr;

  auto itr = patches_.begin();
  const auto fim = patches_.end();

  while (itr != fim) {
    temp = ConvertString(*itr);
    (void)strtok(temp, " <");
    str = strtok(nullptr, "<,");
    std::cout << str << std::endl;
    str = strtok(nullptr, ",,");
    std::cout << str << std::endl;
    str = strtok(nullptr, ",,");
    std::cout << str << std::endl;
    str = strtok(nullptr, ",>");
    std::cout << str << std::endl;
    ++itr;

    delete[] temp;
  }
}

// ler um arquivo para definir um Modelo
void File::ReadFileTo() {
  std::string line;

  const std::string init_curves = "CURVAS_HERMITE";
  bool read_curves = false;
  const std::string end_of_curves = "FIM_CURVAS_HERMITE";

  const std::string init_patches = "PATCHS_HERMITE";
  bool read_patches = false;
  const std::string end_of_patches = "FIM_DE_PATCHS_HERMITES";

  while (std::getline(INPUT_MODEL, line)) {
    if (line == init_curves) {
      read_curves = true;
      continue;
    }
    if (line == end_of_curves) {
      read_curves = false;
      continue;
    }
    if (line.empty()) {
      continue;
    }
    if (read_curves) {
      ReadCurves(line);
      continue;
    }
    if (line == init_patches) {
      read_patches = true;
      continue;
    }
    if (line == end_of_patches) {
      break;
    }
    if (read_patches) {
      ReadPatches(line);
    }
  }

  CreateCurvesTo();
  CreatePatchesTo();
}
