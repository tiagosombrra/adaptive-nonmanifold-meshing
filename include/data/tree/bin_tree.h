#ifndef DATA_TREE_BIN_TREE_H
#define DATA_TREE_BIN_TREE_H

#include <list>

#include "../curve/curve_adaptive_parametric.h"

class BinTree {
 public:
  BinTree(double initial_param_coord = 0.0, double final_param_coord = 1.0,
          BinTree* bin_tree = nullptr);
  ~BinTree();
  unsigned short level_;        // nível d
  double initial_param_coord_;  // coordenada paramétrica do início da célula
  double final_param_coord_;    // coordenada paramétrica do final da célula

  BinTree* bt_father_;
  BinTree* bt_right_child_;
  BinTree* bt_left_child_;
  BinTree* bt_left_neighbor_;
  BinTree* bt_right_neighbor_;

  bool IsLeaf();     // diz se uma célula é folha
  bool IsRoot();     // diz se uma célula é a raiz
  double GetSize();  // retorna o tamanho da célula
  bool Restrict(CurveAdaptiveParametric* curve);
  void Subdivide(CurveAdaptiveParametric* curve);
  // subdivide uma célula e define suas duas células filhas
  void Subdivide(double t, double t_par, CurveAdaptiveParametric* curve);
  // retorna uma célula que contém ti <= t <=tf
  BinTree* Locate(double t);
  void Traverse(BinTree* bin_tree, list<double>& coordinates);
  // retorna as coordenadas das folhas
  list<double> Rediscretization();
};

#endif  // DATA_TREE_BIN_TREE_H
