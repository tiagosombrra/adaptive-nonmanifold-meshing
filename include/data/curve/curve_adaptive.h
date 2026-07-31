#ifndef DATA_CURVE_CURVE_ADAPTIVE_H
#define DATA_CURVE_CURVE_ADAPTIVE_H

#include <list>
#include <vector>

#include "../node_adaptive.h"
#include "../patch/patch.h"
#include "../triangle_adaptive.h"

class CurveAdaptive {
 public:
  CurveAdaptive();
  explicit CurveAdaptive(CurveAdaptive*);
  virtual ~CurveAdaptive();

  void SetId(unsigned int id);
  unsigned long GetId() const;
  double GetLength() const;
  void InsertPoint(PointAdaptive* point);
  unsigned int GetNumBerPoints() const;
  // retorna o i-ésimo ponto
  PointAdaptive* GetPoint(const unsigned int position);
  // troca a lista de pontos
  void SetPoints(const std::list<PointAdaptive*>& new_points);
  std::list<PointAdaptive*>& GetPoints();
  void InsertPatch(Patch* patch);
  void ClearPatches();
  unsigned int GetNumBerPatches() const;
  // retorna o i-ésimo patch
  Patch* GetPatch(const unsigned int position);
  // verifica se uma curva é de borda
  bool CheckIsOnBorder();

  // calcula o comprimento de curva de p1 a p2
  virtual double CalculateLengthPoints(const PointAdaptive&,
                                       const PointAdaptive&);
  // calcula o comprimento de curva até p
  virtual double CalculateLengthPoint(const PointAdaptive&);
  // calcula o comprimento total "L" da curva
  virtual void CalculateLengthCurve();
  // calcula a curvatuta da curva
  virtual double CalculateCurvature(double);

 protected:
  // identificador da curva
  unsigned long id_;
  // comprimento total da curva
  double length_;
  // será usada na discretização
  std::list<PointAdaptive*> points_;
  // uma curva só com um patch é de borda
  std::vector<Patch*> patches_;
};
#endif  // DATA_CURVE_CURVE_ADAPTIVE_H
