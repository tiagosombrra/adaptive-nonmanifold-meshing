#ifndef DATA_TRIANGLE_ADAPTIVE_H
#define DATA_TRIANGLE_ADAPTIVE_H

#include <tuple>

#include "element_adaptive.h"
#include "node_adaptive.h"
#include "vector_adaptive.h"

class TriangleAdaptive : public ElementAdaptive {
 public:
  TriangleAdaptive();
  // será construído no sentido n1->n2->n3
  TriangleAdaptive(NodeAdaptive* n1, NodeAdaptive* n2, NodeAdaptive* n3);
  virtual ~TriangleAdaptive();

  void CalculateArea() override;    // OBS: Não normalizar a normal!!!!
  void CalculateNormal() override;  // OBS: Não normalizar a normal!

  NodeAdaptive GetNoh(unsigned const int position) const override;
  double GetAngle(const NodeAdaptive& node) override;

  void ReplaceNode(const NodeAdaptive* old_node, NodeAdaptive* new_node);
  double CalculateQualityTriangle();

  double CalculateAngleN1();  // calcula o ângulo no nó 1
  double CalculateAngleN2();  // calcula o ângulo no nó 2
  double CalculateAngleN3();  // calcula o ângulo no nó 3

  const std::tuple<double, double>& GetParametersN1() const;
  void SetParametersN1(const std::tuple<double, double>& parameters_n1);

  const std::tuple<double, double>& GetParametersN2() const;
  void SetParametersN2(const std::tuple<double, double>& parameters_n2);

  const std::tuple<double, double>& GetParametersN3() const;
  void SetParametersN3(const std::tuple<double, double>& parameters_n3);

 protected:
  NodeAdaptive* n1_;
  NodeAdaptive* n2_;
  NodeAdaptive* n3_;

  double angle_n1_;  // o ângulo no nó 1
  double angle_n2_;  // o ângulo no nó 2
  double angle_n3_;  // o ângulo no nó 3

  std::tuple<double, double> parameters_n1_;  // parametro do nó 1
  std::tuple<double, double> parameters_n2_;  // parametro do nó 2
  std::tuple<double, double> parameters_n3_;  // parametro do nó 3
};

#endif  // DATA_TRIANGLE_ADAPTIVE_H
