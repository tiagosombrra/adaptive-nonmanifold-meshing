#include "../../include/curvature/adjacent.h"

extern double TOLERANCE;

// P de Primeiro, U de Último !!! :D
// confirma que U é adjacente à esquerda de P
bool Adjacent::ConfirmLeftAdjacency(const NodeAdaptive& noh,
                                    const ElementAdaptive& first_element,
                                    const ElementAdaptive& last_element) {
  //	quem é o noh para P? ( n1, n2 ou n3? )
  //	se noh = P.n1 então verifique se U possui a sequência P.n1 P.n3

  if (noh == first_element.GetNoh(1)) {
    if (((first_element.GetNoh(1) == last_element.GetNoh(1)) and
         (first_element.GetNoh(3) == last_element.GetNoh(2))) or

        ((first_element.GetNoh(1) == last_element.GetNoh(3)) and
         (first_element.GetNoh(3) == last_element.GetNoh(1))) or

        ((first_element.GetNoh(1) == last_element.GetNoh(2)) and
         (first_element.GetNoh(3) == last_element.GetNoh(3))))
      return true;
  }

  //	se noh = P.n2 então verifique se U possui a sequência P.n2 P.n1
  if (noh == first_element.GetNoh(2)) {
    if (((first_element.GetNoh(2) == last_element.GetNoh(1)) and
         (first_element.GetNoh(1) == last_element.GetNoh(2))) or

        ((first_element.GetNoh(2) == last_element.GetNoh(3)) and
         (first_element.GetNoh(1) == last_element.GetNoh(1))) or

        ((first_element.GetNoh(2) == last_element.GetNoh(2)) and
         (first_element.GetNoh(1) == last_element.GetNoh(3))))
      return true;
  }

  //	se noh = P.n3 então verifique se U possui a sequência P.n3 P.n2
  if (noh == first_element.GetNoh(3)) {
    if (((first_element.GetNoh(3) == last_element.GetNoh(1)) and
         (first_element.GetNoh(2) == last_element.GetNoh(2))) or

        ((first_element.GetNoh(3) == last_element.GetNoh(3)) and
         (first_element.GetNoh(2) == last_element.GetNoh(1))) or

        ((first_element.GetNoh(3) == last_element.GetNoh(2)) and
         (first_element.GetNoh(2) == last_element.GetNoh(3))))
      return true;
  }

  return false;
}

// confirma que U é adjacente à direita de P
bool Adjacent::ConfirmRightAdjacency(const NodeAdaptive& noh,
                                     const ElementAdaptive& first_element,
                                     const ElementAdaptive& last_element) {
  //	quem é o noh para P? ( n1, n2 ou n3? )
  //	se noh = P.n1 então verifique se U possui a sequência P.n2 P.n1
  if (noh == first_element.GetNoh(1)) {
    if (((last_element.GetNoh(1) == first_element.GetNoh(1)) and
         (last_element.GetNoh(3) == first_element.GetNoh(2))) or

        ((last_element.GetNoh(3) == first_element.GetNoh(1)) and
         (last_element.GetNoh(2) == first_element.GetNoh(2))) or

        ((last_element.GetNoh(2) == first_element.GetNoh(1)) and
         (last_element.GetNoh(1) == first_element.GetNoh(2))))
      return true;
  }

  //	se noh = P.n2 então verifique se U possui a sequência P.n3 P.n2
  if (noh == first_element.GetNoh(2)) {
    if (((last_element.GetNoh(1) == first_element.GetNoh(2)) and
         (last_element.GetNoh(3) == first_element.GetNoh(3))) or

        ((last_element.GetNoh(3) == first_element.GetNoh(2)) and
         (last_element.GetNoh(2) == first_element.GetNoh(3))) or

        ((last_element.GetNoh(2) == first_element.GetNoh(2)) and
         (last_element.GetNoh(1) == first_element.GetNoh(3))))
      return true;
  }

  //	se noh = P.n3 então verifique se U possui a sequência P.n1 P.n3
  if (noh == first_element.GetNoh(3)) {
    if (((last_element.GetNoh(1) == first_element.GetNoh(3)) and
         (last_element.GetNoh(3) == first_element.GetNoh(1))) or

        ((last_element.GetNoh(3) == first_element.GetNoh(3)) and
         (last_element.GetNoh(2) == first_element.GetNoh(1))) or

        ((last_element.GetNoh(2) == first_element.GetNoh(3)) and
         (last_element.GetNoh(1) == first_element.GetNoh(1))))
      return true;
  }

  return false;
}

ElementAdaptive* Adjacent::GetElementLeft(
    const NodeAdaptive& noh, ElementAdaptive* element,
    std::list<ElementAdaptive*>& list_element) {
  ElementAdaptive* element_result = nullptr;
  std::list<ElementAdaptive*>::iterator element_iterator;

  for (element_iterator = list_element.begin();
       element_iterator != list_element.end(); ++element_iterator) {
    if (Adjacent::ConfirmLeftAdjacency(noh, *element, *(*element_iterator))) {
      element_result = *element_iterator;
      list_element.erase(element_iterator);
      break;
    }
  }

  return element_result;
}

// busca na lista um elemento adjacente à direita de elem
ElementAdaptive* Adjacent::GetElementRight(
    const NodeAdaptive& noh, ElementAdaptive* element,
    std::list<ElementAdaptive*>& list_element) {
  ElementAdaptive* element_result = nullptr;
  std::list<ElementAdaptive*>::iterator element_iterator;

  for (element_iterator = list_element.begin();
       element_iterator != list_element.end(); ++element_iterator) {
    if (Adjacent::ConfirmRightAdjacency(noh, *element, *(*element_iterator))) {
      element_result = *element_iterator;
      list_element.erase(element_iterator);
      break;
    }
  }

  return element_result;
}

// verifica a concavidade de dois elementos adjacentes
int Adjacent::ConcavityElement(const NodeAdaptive& noh,
                               const ElementAdaptive& first_element,
                               const ElementAdaptive& next_element) {
  VectorAdaptive vector;

  if (noh == next_element.GetNoh(1))
    vector = VectorAdaptive(next_element.GetNoh(1), next_element.GetNoh(3));
  else if (noh == next_element.GetNoh(2))
    vector = VectorAdaptive(next_element.GetNoh(2), next_element.GetNoh(1));
  else
    vector = VectorAdaptive(next_element.GetNoh(3), next_element.GetNoh(2));

  double d;
  d = first_element.GetVectorNormal() ^ vector;

  if (fabs(d) <= TOLERANCE) return 0;

  if (d > 0)
    return -1;
  else
    return 1;
}

double Adjacent::AngleElement(const ElementAdaptive& first_element,
                              const ElementAdaptive& next_element) {
  VectorAdaptive first_vector = first_element.GetVectorNormal();
  VectorAdaptive next_vector = next_element.GetVectorNormal();

  return first_vector.CalculateAngle(next_vector);
}
