#ifndef _QUADTREE_H_
#define _QUADTREE_H_

#include "../../data/definitions.h"
#include "boundary.h"
#include "edge.h"
#include "face.h"
#include "quadtree_cell.h"
#include "shape.h"
#include "vertex.h"

using namespace Par2DJMesh;
using namespace Par2DJMesh::AFT;
using namespace Par2DJMesh::Basics;

namespace Par2DJMesh {
namespace AFT {
struct QuadtreeTemplateStats {
  unsigned long long template_count[6];
  double template_score_sum[6];
  double template_score_min[6];
  unsigned long long total_templates;
  unsigned long long low_score_templates;
  double overall_score_sum;
  double overall_score_min;
  unsigned long long low_quality_subdivide_hits;

  QuadtreeTemplateStats();
};

class Quadtree : public Shape {
 private:
  double factor;
  long int maxLevelCap;

  QuadtreeCell *root;

  QuadtreeCellList leaves;

  Boundary *boundary;

  // para a geracao baseada em templates
  EdgeList front;
  EdgeList edges;
  VertexList vertices;
  FaceList mesh;
  long int lastVertexId;
  long int lastEdgeId;
  long int lastFaceId;
  QuadtreeTemplateStats template_stats;

 public:
  Quadtree(Boundary *boundary = NULL, double factor = 0.85);
  ~Quadtree();

  void setFactor(double factor);
  double getFactor();
  void setMaxLevelCap(long int max_level_cap);
  long int getMaxLevelCap() const;

  void setBoundary(Boundary *boundary);
  Boundary *getBoundary();

  void setRoot(QuadtreeCell *root);
  QuadtreeCell *getRoot();

  // retira cell como folha e adiciona os filhos de cell
  void addLeaves(QuadtreeCell *cell);
  QuadtreeCellList getLeaves();

  int getNumCells();

  Vertex *getMin();
  Vertex *getMax();

  long int getCellId();

  void findCell(Edge *e);

  bool in(Vertex *v);
  bool on(Vertex *v);
  bool out(Vertex *v);

  enum MethodStatus generate(const FaceList &oldmesh);
  enum MethodStatus refineToLevel();
  enum MethodStatus refineAccordingToNeighbors();

  bool execute(const FaceList &oldmesh);

  // para a geracao baseada em templates
  long int vertexId();
  long int edgeId();
  long int faceId();

  EdgeList getFront();
  EdgeList getEdges();
  VertexList getVertices();
  FaceList getMesh();

  void add(Vertex *v);
  void add(Edge *e);
  void add(Face *f);
  void addFront(Edge *e);
  void ResetTemplateStats();
  void RecordTemplateChoice(int type, double score);
  void RecordLowQualitySubdivision();
  const QuadtreeTemplateStats& GetTemplateStats() const;

  enum MethodStatus makeTemplateBasedMesh();

  string getText() override;

  // #if USE_OPENGL
  //     void highlight();
  //     void unhighlight();

  //    void draw();
  // #endif //#if USE_OPENGL
};
}  // namespace AFT
}  // namespace Par2DJMesh

#endif  // #ifndef _QUADTREE_H_
