#ifndef CURVATURE_CURVATURE_DISCRETE_H
#define CURVATURE_CURVATURE_DISCRETE_H

#include <cmath>
#include <cstdlib>
#include <list>

#include "../data/element_adaptive.h"
#include "../data/node_adaptive.h"
#include "adjacent.h"
#include "curvature.h"

class CurvatureDiscrete : public Curvature {
 public:
  explicit CurvatureDiscrete(const NodeAdaptive& noh);
  double CalculateMeanCurvature() override;
  double CalculateGaussCurvature() override;

 private:
  void AdjacencySort(const NodeAdaptive& noh);

  NodeAdaptive noh_;
  // lista dos elementos de 'm' incidentes em 'n'
  std::list<ElementAdaptive*> elements_;
  // área dos elementos incidentes em 'n'
  double a_;
  // assume M_PI ou 2 * M_PI
  double factor_disc_;
  // soma dos ângulos em 'n'
  double sum_phi_;
  Adjacent adjacent_;
};

#endif  // CURVATURE_CURVATURE_DISCRETE_H
