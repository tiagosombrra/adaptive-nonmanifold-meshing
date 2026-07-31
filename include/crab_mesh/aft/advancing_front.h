#ifndef ADVANCING_FRONT_H_
#define ADVANCING_FRONT_H_

#include "../../data/definitions.h"
#include "boundary.h"
#include "edge.h"
#include "face.h"
#include "quadtree.h"
#include "quadtree_cell.h"
#include "shape.h"
#include "vertex.h"

using namespace Par2DJMesh;
using namespace Par2DJMesh::AFT;
using namespace Par2DJMesh::Basics;

extern double TOLERANCE_AFT;
extern std::string USE_TEMPLATE;

namespace Par2DJMesh {
namespace AFT {
struct AdvancingFrontRunStats {
  unsigned long long postprocess_vertices_moved;
  unsigned long long postprocess_vertices_considered;
  unsigned long long quadtree_leaf_count;
  unsigned long long quadtree_template_count;
  unsigned long long quadtree_low_score_count;
  unsigned long long quadtree_low_quality_subdivide_hits;
  unsigned long long degenerate_pre_aft_count;
  unsigned long long degenerate_post_aft_count;
  unsigned long long degenerate_post_smoothing_count;
  double quadtree_template_score_min;
  double quadtree_template_score_mean;
  double quality_pre_aft_min;
  double quality_pre_aft_mean;
  double quality_post_aft_min;
  double quality_post_aft_mean;
  double quality_post_smoothing_min;
  double quality_post_smoothing_mean;
  double poor_ratio_pre_aft;
  double poor_ratio_post_aft;
  double poor_ratio_post_smoothing;
  double boundary_element_ratio;
  double transition_element_ratio;
  double internal_element_ratio;
  int adaptive_step;
  int patch_index;

  AdvancingFrontRunStats();
};

class AdvancingFront : public Shape {
 protected:
  long int lastVertexId;
  long int lastEdgeId;
  long int lastFaceId;

  bool boundarySorted;

  // quantidade de vezes que smoothing + local back-tracking, na
  // fase de improvement serao feitos
  double phi;
  unsigned int numImproves;

  Boundary *boundary;

  Quadtree *quadtree;

  FaceList mesh;

  EdgeList front;
  EdgeList innerEdges;
  EdgeList edges;
  EdgeList rejected;

  VertexList frontVertices;
  VertexList innerVertices;
  VertexList vertices;

 protected:
  void makeInitialFront();
  void sortFront();

  // Testa intersecoes com arestas jah existentes
  // e com as faces jah existentes
  bool interceptionTest(Edge *e, Vertex *candidate, bool inFaceTest = true,
                        bool onlyFrontEdges = false);
  bool interceptionTest(Edge *e);

  Vertex *makeIdealVertex(Edge *e, double &h);
  virtual bool findBestVertex(Edge *e, Vertex *&best, bool geometryPhase);

  void insertInFront(Edge *last, Edge *e);

  Edge *findEdge(Vertex *v1, Vertex *v2);
  EdgeList findAdjacentEdges(Vertex *v);
  FaceList findAdjacentFaces(const FaceList &faces, Vertex *v);

  void removeFromFront(Vertex *v1, Vertex *v2 = NULL, Vertex *v3 = NULL);

  virtual enum MethodStatus makeMesh(bool frontBased,
                                     bool geometryPhase = true);

  void fillMesh();

  bool laplacianSmoothing(bool &changed);
  bool qualityShapeSmoothing(bool &changed);
  bool poorRegionShapeSmoothing(bool &changed);
  void ResetRunStats();
  void UpdateRunStatsFromQuadtree();
  void CaptureStageStats(const FaceList& faces, int stage);

 public:
  AdvancingFront(double factor = 0.85, double tolerance = 1.e-8,
                 unsigned int numImproves = 5, double phi = 0.5);
  AdvancingFront(Boundary *boundary, Quadtree *quadtree,
                 double tolerance = 1.e-8, unsigned int numImproves = 5);

  virtual ~AdvancingFront();

  //    static double getTolerance();

  void setBoundarySorted(bool boundarySorted);
  bool isBoundarySorted();

  void setNumImproves(unsigned int numImproves);
  unsigned int getNumImproves();

  void setBoundary(Boundary *boundary);
  Boundary *getBoundary();

  void setQuadtree(Quadtree *quadtree);
  Quadtree *getQuadtree();

  VertexList getVertices();
  VertexList getInnerVertices();

  EdgeList getEdges();
  EdgeList getInnerEdges();

  FaceList getMesh();

  void addVertices(VertexList vertices);
  void addEdges(EdgeList edges);
  void addEdgesToFront(EdgeList front);
  void addMesh(FaceList mesh);

  bool belongsToAdvFront(Edge *e);

  virtual enum MethodStatus makeGeometryBasedMesh();
  virtual enum MethodStatus makeTopologyBasedMesh();
  enum MethodStatus improveMesh();
  const AdvancingFrontRunStats& GetRunStats() const;
  void SetAdaptiveContext(int adaptive_step, int patch_index);

  bool execute(const FaceList &oldmesh);

  string getText();

  // #if USE_OPENGL
  //     void highlight();
  //     void unhighlight();

  //    void draw();
  //    void drawNormals();
  // #endif //#if USE_OPENGL
 private:
  AdvancingFrontRunStats run_stats;
};
}  // namespace AFT
}  // namespace Par2DJMesh

#endif  // ADVANCING_FRONT_H_
