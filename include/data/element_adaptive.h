#ifndef DATA_ELEMENT_ADAPTIVE_H
#define DATA_ELEMENT_ADAPTIVE_H

#include "vector_adaptive.h"

class NodeAdaptive;

class ElementAdaptive {
 public:
  ElementAdaptive();
  virtual ~ElementAdaptive();

  unsigned long GetId() const;
  void SetId(const unsigned long id);
  double GetArea() const;
  VectorAdaptive GetVectorNormal() const;

  virtual void CalculateArea() = 0;
  virtual void CalculateNormal() = 0;
  virtual NodeAdaptive GetNoh(unsigned const int) const = 0;
  virtual double GetAngle(const NodeAdaptive& n) = 0;

 protected:
  unsigned long id_;
  double area_;
  VectorAdaptive vector_normal_;
};

#endif  // DATA_ELEMENT_ADAPTIVE_H
