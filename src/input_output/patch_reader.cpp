#include "../../include/input_output/patch_reader.h"

#include <cmath>
#include <memory>
#include <vector>

#include "../../include/data/vertex_adaptive.h"

namespace {

// Solda controlos repetidos entre patches (.bp estilo blocos) sem depender só de
// igualdade exacta bit-a-bit. Tolerância fixa e estrita: independente de EPSYLON
// do adaptador (que pode ser 1e-2), para não fundir controlos vizinhos legítimos.
constexpr double kBpControlMergeTol = 1e-7;

VertexAdaptive* InternControlPoint(
    const PointAdaptive& src,
    std::vector<std::unique_ptr<VertexAdaptive>>& storage,
    std::vector<VertexAdaptive*>& registry) {
  for (VertexAdaptive* v : registry) {
    if (fabs(v->GetX() - src.GetX()) <= kBpControlMergeTol &&
        fabs(v->GetY() - src.GetY()) <= kBpControlMergeTol &&
        fabs(v->GetZ() - src.GetZ()) <= kBpControlMergeTol) {
      return v;
    }
  }
  auto up =
      std::make_unique<VertexAdaptive>(src.GetX(), src.GetY(), src.GetZ());
  up->SetId(src.GetId());
  VertexAdaptive* raw = up.get();
  storage.push_back(std::move(up));
  registry.push_back(raw);
  return raw;
}

}  // namespace

PatchReader::PatchReader() : patch_(nullptr), patch_hermite_(nullptr) {}

std::list<PatchHermite*> PatchReader::LoaderBPFileHermite() {
#ifdef __APPLE__
  string filename =
      "/Users/tiagosombra/Dropbox/tiago/ufc/MestradoTiago/GitHub/TMeshSurf/"
      "apMesh/Modelos/blend/18_mountain.bp";
#else
  string filename = INPUT_MODEL;  //"../../INPUT_MODEL/mountain.bp"
#endif  // #ifdef __APPLE__

  patch_hermite_ = new PatchHermite();

  if (!filename.empty()) {
    std::ifstream fin(filename);

    if (fin) {
      while (fin) {
        std::string line;
        std::vector<double> v;
        std::vector<PointAdaptive> vectorControlPoints;
        std::vector<long> p;
        long id_ponto = 0;

        while (std::getline(fin, line)) {
          if (line[0] == 'v') {
            line = line.erase(0, 1);
            std::istringstream iss(line);
            // int id_ponto;
            // iss >> id_ponto;

            // line = line.erase(0, 1);
            double n;
            while (iss >> n) {
              v.push_back(n);
            }

            PointAdaptive ponto(v[0], v[1], v[2], id_ponto);
            id_ponto++;
            vectorControlPoints.push_back(ponto);
            v.erase(v.begin(), v.end());
          }

          if (line[0] == 'p') {
            line = line.erase(0, 1);
            std::istringstream iss(line);
            long n;
            while (iss >> n) {
              p.push_back(n);
            }

            for (unsigned int i = 0; i < p.size(); i++) {
              //                            if (i == 0) {
              //                                patchHermite->setPt00(getPointVectorControlPoints(vectorControlPoints,
              //                                p[i]));
              //                            } else if (i == 1) {
              //                                patchHermite->setPt10(getPointVectorControlPoints(vectorControlPoints,
              //                                p[i]));
              //                            } else if (i == 2) {
              //                                patchHermite->setPt20(getPointVectorControlPoints(vectorControlPoints,
              //                                p[i]));
              //                            } else if (i == 3) {
              //                                patchHermite->setPt30(getPointVectorControlPoints(vectorControlPoints,
              //                                p[i]));
              //                            } else if (i == 4) {
              //                                patchHermite->setPt01(getPointVectorControlPoints(vectorControlPoints,
              //                                p[i]));
              //                            } else if (i == 5) {
              //                                patchHermite->setPt11(getPointVectorControlPoints(vectorControlPoints,
              //                                p[i]));
              //                            } else if (i == 6) {
              //                                patchHermite->setPt21(getPointVectorControlPoints(vectorControlPoints,
              //                                p[i]));
              //                            } else if (i == 7) {
              //                                patchHermite->setPt31(getPointVectorControlPoints(vectorControlPoints,
              //                                p[i]));
              //                            } else if (i == 8) {
              //                                patchHermite->setPt02(getPointVectorControlPoints(vectorControlPoints,
              //                                p[i]));
              //                            } else if (i == 9) {
              //                                patchHermite->setPt12(getPointVectorControlPoints(vectorControlPoints,
              //                                p[i]));
              //                            } else if (i == 10) {
              //                                patchHermite->setPt22(getPointVectorControlPoints(vectorControlPoints,
              //                                p[i]));
              //                            } else if (i == 11) {
              //                                patchHermite->setPt32(getPointVectorControlPoints(vectorControlPoints,
              //                                p[i]));
              //                            } else if (i == 12) {
              //                                patchHermite->setPt03(getPointVectorControlPoints(vectorControlPoints,
              //                                p[i]));
              //                            } else if (i == 13) {
              //                                patchHermite->setPt13(getPointVectorControlPoints(vectorControlPoints,
              //                                p[i]));
              //                            } else if (i == 14) {
              //                                patchHermite->setPt23(getPointVectorControlPoints(vectorControlPoints,
              //                                p[i]));
              //                            } else if (i == 15) {
              //                                patchHermite->setPt33(getPointVectorControlPoints(vectorControlPoints,
              //                                p[i]));
              //                            }
            }

            p.erase(p.begin(), p.end());
            patches_hermite_.push_back(patch_hermite_);
            patch_hermite_ = new PatchHermite();
          }
        }
      }
    } else {
      std::cout << "Error: file .bp is not readable." << std::endl;
    }

    fin.close();

  } else {
    std::cout << "Erro ao abrir o arquivo .bp" << std::endl;
  }

  return patches_hermite_;
}

std::list<PatchBezier*> PatchReader::LoaderBPFile(std::string filename) {
  patch_ = new PatchBezier();

  if (!filename.empty()) {
    std::ifstream fin(filename);
    // boost::filesystem::ifstream fin(filename);

    if (fin) {
      while (fin) {
        std::string line;
        std::vector<double> v;
        std::vector<PointAdaptive> vectorControlPoints;
        std::vector<long> p;
        long id_ponto = 0;

        while (std::getline(fin, line)) {
          if (line[0] == 'v') {
            line = line.erase(0, 1);
            std::istringstream iss(line);
            // boost::archive::text_iarchive iss(&line);
            double n;
            while (iss >> n) {
              v.push_back(n);
            }

            PointAdaptive ponto(v[0], v[1], v[2], id_ponto);
            id_ponto++;
            vectorControlPoints.push_back(ponto);
            v.erase(v.begin(), v.end());
          }

          if (line[0] == 'p') {
            line = line.erase(0, 1);
            std::istringstream iss(line);
            long n;
            while (iss >> n) {
              p.push_back(n);
            }

            for (unsigned int i = 0; i < p.size(); i++) {
              if (i == 0) {
                patch_->SetPt00(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 1) {
                patch_->SetPt10(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 2) {
                patch_->SetPt20(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 3) {
                patch_->SetPt30(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 4) {
                patch_->SetPt01(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 5) {
                patch_->SetPt11(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 6) {
                patch_->SetPt21(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 7) {
                patch_->SetPt31(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 8) {
                patch_->SetPt02(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 9) {
                patch_->SetPt12(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 10) {
                patch_->SetPt22(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 11) {
                patch_->SetPt32(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 12) {
                patch_->SetPt03(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 13) {
                patch_->SetPt13(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 14) {
                patch_->SetPt23(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 15) {
                patch_->SetPt33(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              }
            }

            p.erase(p.begin(), p.end());
            patches_.push_back(patch_);
            patch_ = new PatchBezier();
          }
        }
      }
    } else {
      std::cout << "Error: file .bp is not readable." << std::endl;
    }

    fin.close();

  } else {
    std::cout << "Erro ao abrir o arquivo .bp" << std::endl;
  }

  return patches_;
}

std::list<PatchBezier*> PatchReader::LoaderOBJFile() {
#ifdef __APPLE__
  string filename =
      "/Users/tiagosombra/Dropbox/tiago/ufc/MestradoTiago/GitHub/TMeshSurf/"
      "genBezierPatches/projeto/linux/saida/patch_estimativa_2.bp";
#else
  string filename = "../../INPUT_MODEL/uteapot.obj";
#endif  // #ifdef __APPLE__

  patch_ = new PatchBezier();

  if (!filename.empty()) {
    std::ifstream fin(filename);

    if (fin) {
      while (fin) {
        std::string line;
        std::vector<double> v;
        std::vector<PointAdaptive> vectorControlPoints;
        std::vector<long> p;
        long id_Ponto = 0;

        while (std::getline(fin, line)) {
          if (line[0] == 'v') {
            line = line.erase(0, 1);
            std::istringstream iss(line);

            double n;
            while (iss >> n) {
              v.push_back(n);
            }

            PointAdaptive ponto(v[0], v[1], v[2], id_Ponto);
            id_Ponto++;
            vectorControlPoints.push_back(ponto);
            v.erase(v.begin(), v.end());
          }

          if (line[0] == 's' and line[1] == 'u' and line[2] == 'r' and
              line[3] == 'f') {
            line = line.erase(0, 12);
            std::istringstream iss(line);
            long n;
            while (iss >> n) {
              p.push_back(abs(n));
            }

            for (unsigned int i = 0; i < p.size(); i++) {
              if (i == 0) {
                patch_->SetPt00(vectorControlPoints.at(i));
              } else if (i == 1) {
                patch_->SetPt10(vectorControlPoints.at(i));
              } else if (i == 2) {
                patch_->SetPt20(vectorControlPoints.at(i));
              } else if (i == 3) {
                patch_->SetPt30(vectorControlPoints.at(i));
              } else if (i == 4) {
                patch_->SetPt01(vectorControlPoints.at(i));
              } else if (i == 5) {
                patch_->SetPt11(vectorControlPoints.at(i));
              } else if (i == 6) {
                patch_->SetPt21(vectorControlPoints.at(i));
              } else if (i == 7) {
                patch_->SetPt31(vectorControlPoints.at(i));
              } else if (i == 8) {
                patch_->SetPt02(vectorControlPoints.at(i));
              } else if (i == 9) {
                patch_->SetPt12(vectorControlPoints.at(i));
              } else if (i == 10) {
                patch_->SetPt22(vectorControlPoints.at(i));
              } else if (i == 11) {
                patch_->SetPt32(vectorControlPoints.at(i));
              } else if (i == 12) {
                patch_->SetPt03(vectorControlPoints.at(i));
              } else if (i == 13) {
                patch_->SetPt13(vectorControlPoints.at(i));
              } else if (i == 14) {
                patch_->SetPt23(vectorControlPoints.at(i));
              } else if (i == 15) {
                patch_->SetPt33(vectorControlPoints.at(i));
              }
            }

            p.erase(p.begin(), p.end());
            patches_.push_back(patch_);
            patch_ = new PatchBezier();
          }
        }
      }
    } else {
      std::cout << "Error: file .obj is not readable." << std::endl;
    }

    fin.close();

  } else {
    std::cout << "Erro ao abrir o arquivo .obj" << std::endl;
  }

  return patches_;
}

PatchReader::~PatchReader() {
  // Loaders keep one empty look-ahead patch after the final parsed record.
  // Parsed patches are returned to the caller; only the look-ahead objects
  // remain owned by the reader.
  delete patch_;
  delete patch_hermite_;
}
std::list<PatchBezier*> PatchReader::OrderVectorToListBezierPatches(
    std::vector<double> vecDouble) {
  for (unsigned int i = 0; i < vecDouble.size(); i += i + 16) {
    patch_ = new PatchBezier();
  }

  return patches_;
}

PointAdaptive PatchReader::GetPointVectorControlPoints(
    std::vector<PointAdaptive> vectorPoints, unsigned long idPoint) {
  for (std::vector<PointAdaptive>::iterator it = vectorPoints.begin();
       it != vectorPoints.end(); ++it) {
    if ((*it).GetId() == idPoint) {
      return (*it);
    }
  }

  PointAdaptive ponto;
  return ponto;
}

std::list<PatchBezier*> PatchReader::ParsePatchesBezier() {
#if USE_INTERFACEQT

  QFileDialog dialog;
  QWidget* wid = new QWidget();
  QString filename = dialog.getOpenFileName(
      wid, QObject::tr("Adicionar um conjunto de patch"), QDir::currentPath(),
      QObject::tr("Arquivo de cenas (*.patches *.pt)"));
  dialog.close();

#else
  // setar manuamente a localização do arquivo de INPUT_MODEL gerado pelo
  // genMesh(.pt / .patches)

#ifdef __APPLE__
  string filename = "../../INPUT_MODEL/four_patches.pt";
#else
  string filename = "../../INPUT_MODEL/cone1.pt";
#endif  // #ifdef __APPLE__

#endif  // #if USE_INTERFACEQT

#if USE_INTERFACEQT
  if (!filename.isNull()) {
    std::ifstream fin(filename.toUtf8().constData());

#else
  if (!filename.empty()) {
    std::ifstream fin(filename);

#endif  // #if USE_INTERFACEQT

    int count_points = 0;
    long id_point = 0;
    // bool delPatch = true;

    if (fin) {
      while (fin) {
        std::string line;
        std::vector<double> v;

        while (std::getline(fin, line)) {
          //  delPatch = true;

          if (line[0] == 'p' && line[1] == 'a') {
            //  delPatch = false;

            line = line.erase(0, 6);
            std::istringstream iss(line);
            int id_patch;
            iss >> id_patch;
            patch_->SetIdPatchBezier(id_patch);
          }

          if (line[0] == 'p' && line[1] == 'c') {
            //  delPatch = false;

            std::string name_patch = "";
            for (unsigned int i = 0; i < line.size(); i++) {
              if (line[i] != ' ') {
                name_patch.push_back(line[i]);
              } else {
                break;
              }
            }

            line = line.erase(0, 4);
            std::istringstream iss(line);
            double n;

            while (iss >> n) {
              v.push_back(n);
            }
            PointAdaptive point3D(v[0], v[1], v[2], id_point);

            id_point++;

            v.erase(v.begin(), v.end());

            if (name_patch == "pc00") {
              patch_->SetPt00(point3D);
              count_points++;

            } else if (name_patch == "pc10") {
              patch_->SetPt10(point3D);
              count_points++;

            } else if (name_patch == "pc20") {
              patch_->SetPt20(point3D);
              count_points++;

            } else if (name_patch == "pc30") {
              patch_->SetPt30(point3D);
              count_points++;

            } else if (name_patch == "pc01") {
              patch_->SetPt01(point3D);
              count_points++;

            } else if (name_patch == "pc11") {
              patch_->SetPt11(point3D);
              count_points++;

            } else if (name_patch == "pc21") {
              patch_->SetPt21(point3D);
              count_points++;

            } else if (name_patch == "pc31") {
              patch_->SetPt31(point3D);
              count_points++;

            } else if (name_patch == "pc02") {
              patch_->SetPt02(point3D);
              count_points++;

            } else if (name_patch == "pc12") {
              patch_->SetPt12(point3D);
              count_points++;

            } else if (name_patch == "pc22") {
              patch_->SetPt22(point3D);
              count_points++;

            } else if (name_patch == "pc32") {
              patch_->SetPt32(point3D);
              count_points++;

            } else if (name_patch == "pc03") {
              patch_->SetPt03(point3D);
              count_points++;

            } else if (name_patch == "pc13") {
              patch_->SetPt13(point3D);
              count_points++;

            } else if (name_patch == "pc23") {
              patch_->SetPt23(point3D);
              count_points++;

            } else if (name_patch == "pc33") {
              patch_->SetPt33(point3D);
              count_points++;
            }

            name_patch = "";
          }

          if (count_points == 16) {
            count_points = 0;
            patches_.push_back(patch_);
            // patch = nullptr;
            patch_ = new PatchBezier();
          }
        }
      }
    } else
      std::cout << "Error: file .obj is not readable." << std::endl;

    fin.close();

  } else {
    std::cout << "Erro ao abrir o arquivo .obj" << std::endl;
  }

#if USE_INTERFACEQT
  delete wid;
#endif  // #if USE_INTERFACEQT
  return patches_;
}

std::list<PatchBezier*> PatchReader::LoaderRibFile() {
#ifdef __APPLE__
  string filename =
      "/Users/tiagosombra/Dropbox/tiago/ufc/GitHub/TMeshSurfAux_Documentos/"
      "TMeshSurf_Aux/reunioes/objs/"
      "four_patches.pt";
#else
  string filename = "../INPUT_MODEL/uteapot.rib";
#endif  // #ifdef __APPLE__

  std::vector<double> v;

  if (!filename.empty()) {
    std::ifstream fin(filename);

    if (fin) {
      while (fin) {
        std::string line;

        while (std::getline(fin, line)) {
          if (line[0] == 'P' && line[1] == 'a' && line[2] == 't' &&
              line[3] == 'c' && line[4] == 'h') {
            line = line.erase(0, 21);

            std::istringstream iss(line);
            double n;

            while (iss >> n) {
              v.push_back(n);
            }
          }
        }
      }
    } else
      std::cout << "Error: file .obj is not readable." << std::endl;

    fin.close();

  } else {
    std::cout << "Erro ao abrir o arquivo .obj" << std::endl;
  }

  cout << "tamanho de v: " << v.size() / (16 * 3) << endl;

  return patches_;
}

std::list<PatchBezier*> PatchReader::LoaderBezierPatchFile(string fileName) {
  patch_ = new PatchBezier();

  if (!fileName.empty()) {
    std::ifstream fin(fileName);

    if (fin) {
      while (fin) {
        std::string line;
        std::vector<double> v;
        std::vector<PointAdaptive> vectorControlPoints;
        std::vector<long> p;
        long id_ponto = 0;

        while (std::getline(fin, line)) {
          if (line[0] == 'v') {
            line = line.erase(0, 1);
            std::istringstream iss(line);
            // int id_ponto;
            // iss >> id_ponto;

            // line = line.erase(0, 1);
            double n;
            while (iss >> n) {
              v.push_back(n);
            }

            PointAdaptive ponto(v[0], v[1], v[2], id_ponto);
            id_ponto++;
            vectorControlPoints.push_back(ponto);
            v.erase(v.begin(), v.end());
          }

          if (line[0] == 'p') {
            line = line.erase(0, 1);
            std::istringstream iss(line);
            long n;
            while (iss >> n) {
              p.push_back(n);
            }

            for (unsigned int i = 0; i < p.size(); i++) {
              if (i == 0) {
                patch_->SetPt00(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 1) {
                patch_->SetPt10(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 2) {
                patch_->SetPt20(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 3) {
                patch_->SetPt30(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 4) {
                patch_->SetPt01(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 5) {
                patch_->SetPt11(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 6) {
                patch_->SetPt21(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 7) {
                patch_->SetPt31(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 8) {
                patch_->SetPt02(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 9) {
                patch_->SetPt12(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 10) {
                patch_->SetPt22(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 11) {
                patch_->SetPt32(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 12) {
                patch_->SetPt03(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 13) {
                patch_->SetPt13(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 14) {
                patch_->SetPt23(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              } else if (i == 15) {
                patch_->SetPt33(
                    GetPointVectorControlPoints(vectorControlPoints, p[i]));
              }
            }

            p.erase(p.begin(), p.end());
            patches_.push_back(patch_);
            patch_ = new PatchBezier();
          }
        }
      }
    } else {
      std::cout << "Error: file .bp is not readable." << std::endl;
    }

    fin.close();

  } else {
    std::cout << "Erro ao abrir o arquivo .bp" << std::endl;
  }

  return patches_;
}

Geometry* PatchReader::ReaderFilePatches(Geometry* geometry, string file_name) {
  PatchReader* patch_reader = new PatchReader();
  std::list<PatchBezier*> patches =
      patch_reader->LoaderBezierPatchFile(file_name);
  delete patch_reader;

  PointAdaptive* p00;
  PointAdaptive* p01;
  PointAdaptive* p02;
  PointAdaptive* p03;
  PointAdaptive* p10;
  PointAdaptive* p11;
  PointAdaptive* p12;
  PointAdaptive* p13;
  PointAdaptive* p20;
  PointAdaptive* p21;
  PointAdaptive* p22;
  PointAdaptive* p23;
  PointAdaptive* p30;
  PointAdaptive* p31;
  PointAdaptive* p32;
  PointAdaptive* p33;
  CurveAdaptive* patch_c1;
  CurveAdaptive* patch_c2;
  CurveAdaptive* patch_c3;
  CurveAdaptive* patch_c4;

  std::vector<std::unique_ptr<VertexAdaptive>> interned_vertices;
  std::vector<VertexAdaptive*> intern_registry;
  intern_registry.reserve(1024);

  for (std::list<PatchBezier*>::iterator it = patches.begin();
       it != patches.end(); it++) {
    PatchBezier* parsed_patch = *it;
    p00 = InternControlPoint(parsed_patch->GetPt00(), interned_vertices,
                             intern_registry);
    p10 = InternControlPoint(parsed_patch->GetPt10(), interned_vertices,
                             intern_registry);
    p20 = InternControlPoint(parsed_patch->GetPt20(), interned_vertices,
                             intern_registry);
    p30 = InternControlPoint(parsed_patch->GetPt30(), interned_vertices,
                             intern_registry);

    p01 = InternControlPoint(parsed_patch->GetPt01(), interned_vertices,
                             intern_registry);
    p11 = InternControlPoint(parsed_patch->GetPt11(), interned_vertices,
                             intern_registry);
    p21 = InternControlPoint(parsed_patch->GetPt21(), interned_vertices,
                             intern_registry);
    p31 = InternControlPoint(parsed_patch->GetPt31(), interned_vertices,
                             intern_registry);

    p02 = InternControlPoint(parsed_patch->GetPt02(), interned_vertices,
                             intern_registry);
    p12 = InternControlPoint(parsed_patch->GetPt12(), interned_vertices,
                             intern_registry);
    p22 = InternControlPoint(parsed_patch->GetPt22(), interned_vertices,
                             intern_registry);
    p32 = InternControlPoint(parsed_patch->GetPt32(), interned_vertices,
                             intern_registry);

    p03 = InternControlPoint(parsed_patch->GetPt03(), interned_vertices,
                             intern_registry);
    p13 = InternControlPoint(parsed_patch->GetPt13(), interned_vertices,
                             intern_registry);
    p23 = InternControlPoint(parsed_patch->GetPt23(), interned_vertices,
                             intern_registry);
    p33 = InternControlPoint(parsed_patch->GetPt33(), interned_vertices,
                             intern_registry);

    patch_c1 = geometry->VerifyCurveGeometry(p00, p10, p20, p30);
    if (patch_c1 == nullptr) {
      patch_c1 = new CurveAdaptiveParametricBezier(*p00, *p10, *p20, *p30);
      geometry->InsertCurve(patch_c1);
    }

    patch_c2 = geometry->VerifyCurveGeometry(p30, p31, p32, p33);
    if (patch_c2 == nullptr) {
      patch_c2 = new CurveAdaptiveParametricBezier(*p30, *p31, *p32, *p33);
      geometry->InsertCurve(patch_c2);
    }

    patch_c3 = geometry->VerifyCurveGeometry(p03, p13, p23, p33);
    if (patch_c3 == nullptr) {
      patch_c3 = new CurveAdaptiveParametricBezier(*p03, *p13, *p23, *p33);
      geometry->InsertCurve(patch_c3);
    }

    patch_c4 = geometry->VerifyCurveGeometry(p00, p01, p02, p03);
    if (patch_c4 == nullptr) {
      patch_c4 = new CurveAdaptiveParametricBezier(*p00, *p01, *p02, *p03);
      geometry->InsertCurve(patch_c4);
    }

    delete parsed_patch;
    (*it) = new PatchBezier(patch_c1, patch_c2, patch_c3, patch_c4, *p11,
                            *p21, *p12, *p22);

    geometry->InsertPatch((*it));
  }

  return geometry;
}
