#ifndef CURVATURE_ADJACENT_H
#define CURVATURE_ADJACENT_H

#include <cmath>
#include <iostream>
#include <list>

#include "../data/node_adaptive.h"
#include "../data/triangle_adaptive.h"

// OBS: Essa classe foi implementada para triâgulos !!!
class Adjacent {
 public:
  static bool ConfirmLeftAdjacency(const NodeAdaptive& noh,
                                   const ElementAdaptive& first_element,
                                   const ElementAdaptive& last_element);
  static bool ConfirmRightAdjacency(const NodeAdaptive& noh,
                                    const ElementAdaptive& first_element,
                                    const ElementAdaptive& last_element);
  static ElementAdaptive* GetElementLeft(
      const NodeAdaptive& noh, ElementAdaptive* element,
      std::list<ElementAdaptive*>& list_element);
  static ElementAdaptive* GetElementRight(
      const NodeAdaptive& noh, ElementAdaptive* element,
      std::list<ElementAdaptive*>& list_element);
  static int ConcavityElement(const NodeAdaptive& noh,
                              const ElementAdaptive& first_element,
                              const ElementAdaptive& next_element);
  static double AngleElement(const ElementAdaptive& first_element,
                             const ElementAdaptive& next_element);
};
#endif  // CURVATURE_ADJACENT_H
