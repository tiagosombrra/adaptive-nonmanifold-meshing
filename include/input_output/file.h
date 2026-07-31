#ifndef INPUT_OUTPUT_FILE_H
#define INPUT_OUTPUT_FILE_H

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <list>
#include <string>

#include "../data/model.h"

class File {
 public:
  explicit File(const char* name);
  ~File();

  std::string GetName() const;
  void ReadFileTo();

 protected:
  void ReadCurves(const std::string& reading);   // lê todas as curvas do arquivo
  void ReadPatches(const std::string& reading);  // lê todos os patches do arquivo
  void CreateCurvesTo();                         // cria as curvas a partir da list
  void CreatePatchesTo();                        // cria os patches a partir da list
  char* ConvertString(const std::string& font);

  std::string name_;
  std::ifstream INPUT_MODEL;
  std::list<std::string> curves_;
  std::list<std::string> patches_;
};
#endif  // INPUT_OUTPUT_FILE_H
