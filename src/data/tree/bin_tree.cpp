/* Classe para leitura de arquivo
 */
#include "../../../include/data/tree/bin_tree.h"

extern double TOLERANCE;

BinTree::BinTree(double initial_param_coord, double final_param_coord,
                 BinTree* bin_tree) {
  this->initial_param_coord_ = initial_param_coord;
  this->final_param_coord_ = final_param_coord;
  this->bt_father_ = bin_tree;
  if (bin_tree) {
    this->level_ = bin_tree->level_ + 1;
  } else {
    this->level_ = 0;
  }
  this->bt_left_child_ = this->bt_right_child_ = nullptr;
  this->bt_left_neighbor_ = this->bt_right_neighbor_ = nullptr;
}

BinTree::~BinTree() {
  if (this->bt_left_child_) {
    delete (this->bt_left_child_);
  }

  if (this->bt_right_child_) {
    delete (this->bt_right_child_);
  }
}

// diz se uma célula é folha
bool BinTree::IsLeaf() {
  if (this->bt_left_child_ || this->bt_right_child_) {
    return false;
  }

  return true;
}

// diz se uma célula é a raiz
bool BinTree::IsRoot() {
  if (this->bt_father_) {
    return false;
  }

  return true;
}

// retorna o tamanho da célula
double BinTree::GetSize() {
  return (this->final_param_coord_ - this->initial_param_coord_);
}

bool BinTree::Restrict(CurveAdaptiveParametric* curve) {
  if (!this->IsLeaf()) {
    return this->bt_left_child_->Restrict(curve);
  }

  if (!this->bt_right_neighbor_) {
    return false;
  }

  if (this->bt_right_neighbor_->level_ > this->level_ + 1) {
    this->Subdivide(curve);
    this->bt_left_child_->Restrict(curve);
    return true;
  } else if (this->bt_right_neighbor_->level_ < this->level_ - 1) {
    this->bt_right_neighbor_->Subdivide(curve);
    this->Restrict(curve);
    return true;
  }

  return this->bt_right_neighbor_->Restrict(curve);
}

void BinTree::Subdivide(CurveAdaptiveParametric* curve) {
  double new_t;  // t que divide a curva ao meio
  new_t = curve->CalculateMidparameterByParamters(this->initial_param_coord_,
                                                  this->final_param_coord_);

  BinTree* bt_left = new BinTree(this->initial_param_coord_, new_t, this);
  this->bt_left_child_ = bt_left;

  BinTree* bt_right = new BinTree(new_t, this->final_param_coord_, this);
  this->bt_right_child_ = bt_right;

  this->bt_left_child_->bt_left_neighbor_ = this->bt_left_neighbor_;
  this->bt_left_child_->bt_right_neighbor_ = this->bt_right_child_;

  this->bt_right_child_->bt_left_neighbor_ = this->bt_left_child_;
  this->bt_right_child_->bt_right_neighbor_ = this->bt_right_neighbor_;

  if (this->bt_left_neighbor_) {
    this->bt_left_neighbor_->bt_right_neighbor_ = this->bt_left_child_;
  }
  if (this->bt_right_neighbor_) {
    this->bt_right_neighbor_->bt_left_neighbor_ = this->bt_right_child_;
  }
}

// Subdivide uma célula e define suas duas células filhas
void BinTree::Subdivide(double t, double t_par,
                        CurveAdaptiveParametric* curve) {
  if (this->IsLeaf()) {
    if ((this->GetSize() - t_par) < TOLERANCE) {
      return;
    }
    Subdivide(curve);
  }

  double middle = curve->CalculateMidparameterByParamters(
      this->initial_param_coord_, this->final_param_coord_);

  if (t <= middle + TOLERANCE) {
    this->bt_left_child_->Subdivide(t, t_par, curve);
  }

  if (t >= middle - TOLERANCE) {
    this->bt_right_child_->Subdivide(t, t_par, curve);
  }
}

// retorna uma célula que contém ti <= t <=tf
BinTree* BinTree::Locate(double t) {
  BinTree* bt = this;
  double middle = 0.0;

  while (!bt->IsLeaf()) {
    middle = 0.5 * bt->GetSize();
    if (t <= bt->initial_param_coord_ + middle)
      bt = bt->bt_left_child_;
    else
      bt = bt->bt_right_child_;
  }

  return bt;
}

// percorre a árvore em pré-ordem
void BinTree::Traverse(BinTree* bin_tree, list<double>& coordinates) {
  if (bin_tree->IsLeaf()) {
    coordinates.push_back(bin_tree->initial_param_coord_);
  }
  if (bin_tree->bt_left_child_) {
    this->Traverse(bin_tree->bt_left_child_, coordinates);
  }
  if (bin_tree->bt_right_child_) {
    this->Traverse(bin_tree->bt_right_child_, coordinates);
  }
}

// retorna as coordenadas das folhas
// essa lista deve ser usada pelo adaptador de curva para gerar a nova
// lista de pontos da curva rediscretizada
list<double> BinTree::Rediscretization() {
  // essa lista deve ser usada pelo adaptador de curva para gerar a nova
  // lista de pontos da curva rediscretizada
  list<double> coordinates;

  this->Traverse(this, coordinates);

  coordinates.push_back(1.0);

  return coordinates;
}
