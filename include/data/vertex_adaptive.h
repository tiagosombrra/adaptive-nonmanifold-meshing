#ifndef DATA_VERTEX_ADAPTIVE_H
#define DATA_VERTEX_ADAPTIVE_H

#include "point_adaptive.h"

class VertexAdaptive : public PointAdaptive {
 public:
  VertexAdaptive();
  VertexAdaptive(double x, double y, double z);
  explicit VertexAdaptive(PointAdaptive* point);
};
#endif  // DATA_VERTEX_ADAPTIVE_H
