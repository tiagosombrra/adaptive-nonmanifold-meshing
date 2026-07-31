#include "../../../include/crab_mesh/aft/advancing_front.h"

#include <cfloat>
#include <algorithm>
#include <map>

extern int AFT_LOCAL_POSTPROCESS_PASSES;
extern double AFT_LOCAL_POSTPROCESS_QUALITY_THRESHOLD;
extern double AFT_LOCAL_POSTPROCESS_BLEND;

namespace {
double ComputeLocalQualityScore(const FaceList &faces) {
  if (faces.empty()) {
    return 0.0;
  }

  double averageQuality = 0.0;
  double minQuality = DBL_MAX;

  for (FaceList::const_iterator iter = faces.begin(); iter != faces.end();
       ++iter) {
    const double quality = (*iter)->quality();
    averageQuality += quality;
    minQuality = std::min(minQuality, quality);
  }

  averageQuality /= static_cast<double>(faces.size());
  return 0.6 * averageQuality + 0.4 * minQuality;
}

bool FaceOppositeEdge(Face *face, Vertex *vertex, Vertex *&a, Vertex *&b) {
  if (face->getV1() == vertex) {
    a = face->getV2();
    b = face->getV3();
    return true;
  }
  if (face->getV2() == vertex) {
    a = face->getV3();
    b = face->getV1();
    return true;
  }
  if (face->getV3() == vertex) {
    a = face->getV1();
    b = face->getV2();
    return true;
  }
  return false;
}

struct StageFaceStats {
  double quality_min = 0.0;
  double quality_mean = 0.0;
  double poor_ratio = 0.0;
  double boundary_ratio = 0.0;
  double transition_ratio = 0.0;
  double internal_ratio = 0.0;
  unsigned long long degenerate_count = 0ULL;
};

StageFaceStats ComputeStageFaceStats(const FaceList& faces) {
  StageFaceStats stats;
  if (faces.empty()) {
    return stats;
  }

  struct EdgeKey {
    Vertex* a;
    Vertex* b;
    bool operator<(const EdgeKey& other) const {
      return (a < other.a) || (a == other.a && b < other.b);
    }
  };

  std::map<EdgeKey, unsigned int> edge_use_count;
  for (FaceList::const_iterator iter = faces.begin(); iter != faces.end(); ++iter) {
    Face* face = *iter;
    Vertex* verts[3] = {face->getV1(), face->getV2(), face->getV3()};
    for (int e = 0; e < 3; ++e) {
      Vertex* v1 = verts[e];
      Vertex* v2 = verts[(e + 1) % 3];
      EdgeKey key = (v1 < v2) ? EdgeKey{v1, v2} : EdgeKey{v2, v1};
      ++edge_use_count[key];
    }
  }

  stats.quality_min = DBL_MAX;
  unsigned long long poor_count = 0ULL;
  unsigned long long boundary_faces = 0ULL;
  unsigned long long transition_faces = 0ULL;
  unsigned long long internal_faces = 0ULL;

  for (FaceList::const_iterator iter = faces.begin(); iter != faces.end(); ++iter) {
    Face* face = *iter;
    const double quality = face->quality();
    stats.quality_min = std::min(stats.quality_min, quality);
    stats.quality_mean += quality;
    if (quality < 0.35) {
      ++poor_count;
    }
    if (quality < 0.05 || face->orientedSurface() <= TOLERANCE_AFT) {
      ++stats.degenerate_count;
    }

    Vertex* verts[3] = {face->getV1(), face->getV2(), face->getV3()};
    unsigned int boundary_edges = 0U;
    for (int e = 0; e < 3; ++e) {
      Vertex* v1 = verts[e];
      Vertex* v2 = verts[(e + 1) % 3];
      EdgeKey key = (v1 < v2) ? EdgeKey{v1, v2} : EdgeKey{v2, v1};
      if (edge_use_count[key] == 1U) {
        ++boundary_edges;
      }
    }

    if (boundary_edges > 0U) {
      ++boundary_faces;
    } else {
      const double h1 = verts[0]->distance(verts[1]);
      const double h2 = verts[1]->distance(verts[2]);
      const double h3 = verts[2]->distance(verts[0]);
      const double hmin = std::min(h1, std::min(h2, h3));
      const double hmax = std::max(h1, std::max(h2, h3));
      if (hmin > TOLERANCE_AFT && hmax / hmin > 1.8) {
        ++transition_faces;
      } else {
        ++internal_faces;
      }
    }
  }

  const double total = static_cast<double>(faces.size());
  stats.quality_mean /= total;
  stats.poor_ratio = static_cast<double>(poor_count) / total;
  stats.boundary_ratio = static_cast<double>(boundary_faces) / total;
  stats.transition_ratio = static_cast<double>(transition_faces) / total;
  stats.internal_ratio = static_cast<double>(internal_faces) / total;
  if (stats.quality_min == DBL_MAX) {
    stats.quality_min = 0.0;
  }
  return stats;
}
}  // namespace

namespace Par2DJMesh {
namespace AFT {
AdvancingFrontRunStats::AdvancingFrontRunStats()
    : postprocess_vertices_moved(0),
      postprocess_vertices_considered(0),
      quadtree_leaf_count(0),
      quadtree_template_count(0),
      quadtree_low_score_count(0),
      quadtree_low_quality_subdivide_hits(0),
      degenerate_pre_aft_count(0),
      degenerate_post_aft_count(0),
      degenerate_post_smoothing_count(0),
      quadtree_template_score_min(0.0),
      quadtree_template_score_mean(0.0),
      quality_pre_aft_min(0.0),
      quality_pre_aft_mean(0.0),
      quality_post_aft_min(0.0),
      quality_post_aft_mean(0.0),
      quality_post_smoothing_min(0.0),
      quality_post_smoothing_mean(0.0),
      poor_ratio_pre_aft(0.0),
      poor_ratio_post_aft(0.0),
      poor_ratio_post_smoothing(0.0),
      boundary_element_ratio(0.0),
      transition_element_ratio(0.0),
      internal_element_ratio(0.0),
      adaptive_step(-1),
      patch_index(-1) {}
}  // namespace AFT
}  // namespace Par2DJMesh

AdvancingFront::AdvancingFront(double factor, [[maybe_unused]] double tolerance,
                               unsigned int improvement_count,
                               double smoothing_factor)
    : Shape() {
  boundary = new Boundary();
  quadtree = new Quadtree(boundary, factor);

  // cout << "AdvancingFront tolerance: " << tolerance << endl;
  // Shape::setTolerance(tolerance);

  boundarySorted = true;
  this->numImproves = improvement_count;
  this->phi = smoothing_factor;

  lastVertexId = lastEdgeId = lastFaceId = 0;
}

AdvancingFront::AdvancingFront(Boundary *initial_boundary,
                               Quadtree *initial_quadtree,
                               [[maybe_unused]] double tolerance,
                               unsigned int improvement_count)
    : Shape() {
  setBoundary(initial_boundary);
  setQuadtree(initial_quadtree);

  // cout << "AdvancingFront tolerance: " << tolerance << endl;
  // Shape::setTolerance(tolerance);

  setBoundarySorted(true);
  setNumImproves(improvement_count);

  lastVertexId = lastEdgeId = lastFaceId = 0;
  phi = 0.5;
}

AdvancingFront::~AdvancingFront() {
  while (!mesh.empty()) {
    Face *f = mesh.front();
    mesh.pop_front();

    f->setVertices(NULL, NULL, NULL);

    delete f;
  }

  while (!innerEdges.empty()) {
    Edge *e = innerEdges.front();
    innerEdges.pop_front();

    e->setVertices(NULL, NULL);

    delete e;
  }

  while (!innerVertices.empty()) {
    Vertex *v = innerVertices.front();
    innerVertices.pop_front();

    delete v;
  }

  vertices.clear();
  frontVertices.clear();

  edges.clear();
  rejected.clear();
  front.clear();

  delete quadtree;

  quadtree = NULL;
}

void AdvancingFront::makeInitialFront() {
  front = boundary->getEdges();

  for (EdgeList::iterator iter = front.begin(); iter != front.end(); iter++) {
    /*//cout << "aresta (" <<
        (*iter)->getV1()->getX() << ", " << (*iter)->getV1()->getY() << ") e ("
       <<
        (*iter)->getV2()->getX() << ", " << (*iter)->getV2()->getY() << ")" <<
       endl;*/

    vertices.push_back((*iter)->getV1());

    if ((*iter)->getV1()->getId() > lastVertexId) {
      lastVertexId = (*iter)->getV1()->getId();
    }

    edges.push_back((*iter));

    if ((*iter)->getId() > lastEdgeId) {
      lastEdgeId = (*iter)->getId();
    }
  }

  EdgeList quadtreeFront = quadtree->getFront();

  while (!quadtreeFront.empty()) {
    Edge *e = quadtreeFront.front();
    quadtreeFront.pop_front();

    front.push_back(e);

    bool found = false;

    for (VertexList::iterator iter = vertices.begin(); iter != vertices.end();
         iter++) {
      if ((*iter) == e->getV1()) {
        found = true;

        break;
      }
    }

    if (!found) {
      vertices.push_back(e->getV1());

      if (e->getV1()->getId() > lastVertexId) {
        lastVertexId = e->getV1()->getId();
      }
    }

    edges.push_back(e);

    if (e->getId() > lastEdgeId) {
      lastEdgeId = e->getId();
    }
  }

  lastVertexId = quadtree->vertexId() - 1;
  lastEdgeId = quadtree->edgeId() - 1;
  lastFaceId = quadtree->faceId() - 1;
}

void AdvancingFront::sortFront() {
  EdgeList sorted, inBoundary;

  while (!front.empty()) {
    Edge *least = front.front();

    if (least->isInBoundary()) {
      front.pop_front();

      inBoundary.push_back(least);

      continue;
    }

    EdgeList::iterator leastIter = front.begin();

    for (EdgeList::iterator iter = ++front.begin(); iter != front.end();
         iter++) {
      if ((*iter)->length() < least->length()) {
        least = (*iter);
        leastIter = iter;
      }
    }

    front.erase(leastIter);

    sorted.push_back(least);
  }

  front.swap(sorted);

  while (!inBoundary.empty()) {
    Edge *least = inBoundary.front();
    EdgeList::iterator leastIter = inBoundary.begin();

    for (EdgeList::iterator iter = ++inBoundary.begin();
         iter != inBoundary.end(); iter++) {
      if ((*iter)->length() < least->length()) {
        least = (*iter);
        leastIter = iter;
      }
    }

    inBoundary.erase(leastIter);

    sorted.push_back(least);
  }

  front.insert(front.begin(), sorted.begin(), sorted.end());
}

bool AdvancingFront::interceptionTest(Edge *e, Vertex *candidate,
                                      bool inFaceTest, bool onlyFrontEdges) {
  bool intercepts = false;

  EdgeList *testEdges = &edges;

  if (onlyFrontEdges) {
    testEdges = &front;
  }

  for (EdgeList::iterator iter = testEdges->begin(); iter != testEdges->end();
       iter++) {
    if (((*iter)->intercept(e->getV1(), candidate)) ||
        ((*iter)->intercept(candidate, e->getV2())) ||
        false)  //((*iter)->intercept(candidate)))
    {
      intercepts = true;

      break;
    }
  }

  if (inFaceTest && !intercepts) {
    // static Face f;
    Face f;
    // cout << "inte teste" << endl;

    f.setVertices(e->getV1(), e->getV2(), candidate);

    for (VertexList::iterator iter = frontVertices.begin();
         iter != frontVertices.end(); iter++) {
      // #pragma omp critical
      //             {

      //            if (!(*iter)) {
      //               cout<<"iter: "<<(*iter)->getId()<<" thread:
      //               "<<omp_get_thread_num()<<endl;
      //            }
      //            }

      if (((*iter) == e->getV1()) || ((*iter) == e->getV2()) ||
          ((*iter) == candidate)) {
        continue;
      }

      if (((*iter)->matches(e->getV1())) || ((*iter)->matches(e->getV2())) ||
          ((*iter)->matches(candidate))) {
        continue;
      }

      // if (f->in((*iter)))
      if (!f.out((*iter))) {
        intercepts = true;

        break;
      }
    }

    f.setVertices(NULL, NULL, NULL);

    // cout << "fim inte teste" << endl;
  }

  return intercepts;
}

bool AdvancingFront::interceptionTest(Edge *e) {
  for (EdgeList::iterator iter = edges.begin(); iter != edges.end(); iter++) {
    if ((*iter) == e) {
      continue;
    }

    if ((*iter)->equals(e)) {
      continue;
    }

    if ((*iter)->intercept(e->getV1(), e->getV2())) {
      return true;
    }
  }

  return false;
}

Vertex *AdvancingFront::makeIdealVertex(Edge *e, double &h) {
  Vertex *v = e->normal();

  h *= e->getCell()->height();

  v->scalarMultiplication(h);
  v->sum(e->getMid());

  // #if DEBUG_MODE
  //     v->h = h;
  // #endif //#if DEBUG_MODE

  return v;
}

bool AdvancingFront::findBestVertex(Edge *e, Vertex *&best,
                                    bool geometryPhase) {
  bool vertexExistedBefore = true;

  double h = 1.0;

  best = (geometryPhase) ? makeIdealVertex(e, h) : NULL;

  VertexList bestVertices;

  // debug markos
  // std::cerr << std::boolalpha << "geometryPhase = " << geometryPhase << ";
  // frontVertices.size() = " << frontVertices.size() << "; "; int matches = 0,
  // outbox = 0, outdistance = 0, outnormal = 0, intercepted = 0; endebug markos

  for (VertexList::iterator iter = frontVertices.begin();
       iter != frontVertices.end(); iter++) {
    Vertex *candidate = (*iter);

    if ((candidate == e->getV1()) || (candidate == e->getV2()) ||
        (candidate->matches(e->getV1())) || (candidate->matches(e->getV2()))) {
      // matches++;
      continue;
    }

    if (geometryPhase) {
      if ((candidate->getX() < best->getX() - h) ||
          (candidate->getX() > best->getX() + h) ||
          (candidate->getY() < best->getY() - h) ||
          (candidate->getY() > best->getY() + h)) {
        // outbox++;
        continue;
      }

      if (candidate->distance(best) > h) {
        // outdistance++;
        continue;
      }

      // testa se best estah perto demais de e
      double dist = e->straightDistance(candidate);

      if (dist < 0.0) {
        // cout << "debug " << e->getId() << endl;

        delete best;

        best = NULL;

        // std::cerr << "matches = " << matches << "; outbox = " << outbox << ";
        // outdistance = " << outdistance << "; outnormal = " << outnormal <<
        //            "; intercepted = " << intercepted << ";
        //            bestVertices.size() = " << bestVertices.size() << ";
        //            return = false, NULL; reason = dist < 0.0" << std::endl;

        return false;
      } else if (dist <= h * 0.1) {
        // continue;

        // std::cerr << "matches = " << matches << "; outbox = " << outbox << ";
        // outdistance = " << outdistance << "; outnormal = " << outnormal <<
        //            "; intercepted = " << intercepted << ";
        //            bestVertices.size() = " << bestVertices.size() << ";
        //            return = false,  NULL; reason = dist < h*0.1" <<
        //            std::endl;

        bestVertices.clear();

        delete best;

        best = NULL;

        return false;
      }
    } else {
      if (!(e->accordingToNormal(candidate))) {
        // outnormal++;
        continue;
      }
    }

    // bool intercept = interceptionTest(e, candidate, false);
    // bool intercept = geometryPhase ? interceptionTest(e, candidate, false) :
    // interceptionTest(e, candidate, false, true);
    bool intercept = interceptionTest(e, candidate, true, false);

    if (!intercept) {
      bestVertices.push_back(candidate);
    }
    // else
    //{
    // intercepted++;
    //}
  }

  // std::cerr << "matches = " << matches << "; outbox = " << outbox << ";
  // outdistance = " << outdistance << "; outnormal = " << outnormal <<
  //            "; intercepted = " << intercepted << "; bestVertices.size() = "
  //            << bestVertices.size() << "; return = ";

  if (geometryPhase && bestVertices.empty()) {
    bool intercept = interceptionTest(e, best);

    if (intercept) {
      delete best;

      best = NULL;

      // std::cerr << "false,  NULL; reason = intercept == true" << std::endl;

      return false;
    } else {
      bool tooClose = false;

      // testa se best estah perto demais de alguma outra edge da fronteira

      for (EdgeList::iterator iter = edges.begin(); iter != edges.end();
           iter++) {
        if ((*iter)->intercept(best)) {
          tooClose = true;

          break;
        }
      }

      if (tooClose) {
        delete best;

        best = NULL;

        // std::cerr << "false,  NULL; reason = tooClose == true" << std::endl;

        return false;
      } else {
        best->setId(++lastVertexId);

        frontVertices.push_back(best);
        innerVertices.push_back(best);
        vertices.push_back(best);
      }
    }

    // std::cerr << "false, NOTNULL; reason = new vertex not close to existing"
    // << std::endl;

    vertexExistedBefore = false;
  } else if (!bestVertices.empty()) {
    if (geometryPhase) {
      delete best;
    }

    best = bestVertices.front();
    bestVertices.pop_front();

    Vertex *v1 = new Vertex(e->getV1()->getX() - best->getX(),
                            e->getV1()->getY() - best->getY());
    Vertex *v2 = new Vertex(e->getV2()->getX() - best->getX(),
                            e->getV2()->getY() - best->getY());

    double maxAngle = v1->angle(v2);

    while (!bestVertices.empty()) {
      Vertex *candidate = bestVertices.front();
      bestVertices.pop_front();

      v1->setPosition(e->getV1()->getX() - candidate->getX(),
                      e->getV1()->getY() - candidate->getY());
      v2->setPosition(e->getV2()->getX() - candidate->getX(),
                      e->getV2()->getY() - candidate->getY());

      double angle = v1->angle(v2);

      if (angle > maxAngle) {
        maxAngle = angle;
        best = candidate;
      } else if (fabs(maxAngle - angle) < TOLERANCE_AFT) {
        // caso em que os 2 vertices, candidate e best sao geometricamente
        // iguais, mas topologicamente diferentes
        Edge *candidateEdge1, *candidateEdge2;
        candidateEdge1 = candidateEdge2 = NULL;

        for (EdgeList::iterator iter = front.begin(); iter != front.end();
             iter++) {
          Edge *eAux = (*iter);

          if (eAux->getV2() == candidate) {
            candidateEdge1 = eAux;
          }

          if (eAux->getV1() == candidate) {
            candidateEdge2 = eAux;
          }

          if (candidateEdge1 && candidateEdge2) {
            break;
          }
        }

        if ((candidateEdge1) && (candidateEdge2) &&
            (candidateEdge1->accordingToNormal(e->getV1(), true)) &&
            (candidateEdge1->accordingToNormal(e->getV2(), true)) &&
            (candidateEdge2->accordingToNormal(e->getV1(), true)) &&
            (candidateEdge2->accordingToNormal(e->getV2(), true))) {
          maxAngle = angle;
          best = candidate;
        }
      }
    }

    delete v1;
    delete v2;

    // std::cerr << "true, NOTNULL; reason = existing vertex" << std::endl;
  } else {
    // std::cerr << "false, NOTNULL; reason = new vertex" << std::endl;

    vertexExistedBefore = false;
  }

  return vertexExistedBefore;
}

void AdvancingFront::insertInFront(Edge *last, Edge *e) {
  if (boundarySorted) {
    if (last) {
      bool inserted = false;

      for (EdgeList::iterator iter = front.begin(); iter != front.end();
           iter++) {
        if ((*iter) == last) {
          for (iter++; iter != front.end(); iter++) {
            if ((*iter)->length() > e->length()) {
              front.insert(iter, e);

              inserted = true;

              break;
            }
          }

          break;
        }
      }

      if (!inserted) {
        front.push_back(e);
      }
    } else {
      bool inserted = false;

      for (EdgeList::iterator iter = front.begin(); iter != front.end();
           iter++) {
        if ((*iter)->length() > e->length()) {
          front.insert(iter, e);

          inserted = true;

          break;
        }
      }

      if (!inserted) {
        front.push_back(e);
      }
    }
  } else {
    front.push_back(e);
  }
}

Edge *AdvancingFront::findEdge(Vertex *v1, Vertex *v2) {
  Edge *e = NULL;

  for (EdgeList::iterator iter = edges.begin(); iter != edges.end(); iter++) {
    if ((*iter)->equals(v1, v2)) {
      e = (*iter);
      break;
    }
  }

  return e;
}

EdgeList AdvancingFront::findAdjacentEdges(Vertex *v) {
  EdgeList adjacency;

  EdgeSet adjacentEdges = v->getAdjacentEdges();

  adjacency.insert(adjacency.end(), adjacentEdges.begin(), adjacentEdges.end());

  return adjacency;
}

FaceList AdvancingFront::findAdjacentFaces(const FaceList &faces, Vertex *v) {
  FaceList adjacency;

  for (FaceList::const_iterator iter = faces.begin(); iter != faces.end();
       iter++) {
    if (((*iter)->getV1() == v) || ((*iter)->getV2() == v) ||
        ((*iter)->getV3() == v)) {
      adjacency.push_back((*iter));
    }
  }

  return adjacency;
}

void AdvancingFront::removeFromFront(Vertex *v1, Vertex *v2, Vertex *v3) {
  // verifica se v1 eh adjacente a alguma aresta livre, ou seja,
  // da fronteira corrente. se for adjacente, nao remove. se nao for
  // adjacente, remove
  EdgeSet adjacentEdges = v1->getAdjacentEdges();

  bool remove = true;

  for (EdgeSet::iterator iter = adjacentEdges.begin();
       iter != adjacentEdges.end(); iter++) {
    if ((*iter)->isFree()) {
      remove = false;

      break;
    }
  }

  if (remove) {
    frontVertices.remove(v1);
  }

  // verifica se v2 eh adjacente a alguma aresta livre, ou seja,
  // da fronteira corrente. se for adjacente, nao remove. se nao for
  // adjacente, remove
  adjacentEdges = v2->getAdjacentEdges();

  remove = true;

  for (EdgeSet::iterator iter = adjacentEdges.begin();
       iter != adjacentEdges.end(); iter++) {
    if ((*iter)->isFree()) {
      remove = false;

      break;
    }
  }

  if (remove) {
    frontVertices.remove(v2);
  }

  // verifica se v3 eh adjacente a alguma aresta livre, ou seja,
  // da fronteira corrente. se for adjacente, nao remove. se nao for
  // adjacente, remove
  adjacentEdges = v3->getAdjacentEdges();

  remove = true;

  for (EdgeSet::iterator iter = adjacentEdges.begin();
       iter != adjacentEdges.end(); iter++) {
    if ((*iter)->isFree()) {
      remove = false;

      break;
    }
  }

  if (remove) {
    frontVertices.remove(v3);
  }
}

enum MethodStatus AdvancingFront::makeMesh(bool frontBased,
                                           bool geometryPhase) {
  // cout << "AdvancingFront::makeMesh(frontBased = " << frontBased << ",
  // geometryPhase = " << geometryPhase << ")" << endl;

  frontVertices.clear();

  for (EdgeList::iterator iter = front.begin(); iter != front.end(); iter++) {
    frontVertices.push_back((*iter)->getV1());
  }

  Edge *last = NULL;

  if (frontBased && geometryPhase) {
    last = new Edge();
    front.push_back(last);
  }

  while (true) {
    Edge *e = NULL;

    if (frontBased) {
      if (front.empty()) {
        break;
      }

      e = front.front();
      front.pop_front();
    } else {
      if (rejected.empty()) {
        break;
      }

      e = rejected.front();
      rejected.pop_front();
    }

    if (frontBased && (e == last)) {
      delete last;
      last = NULL;

      continue;
    }

    Vertex *best = NULL;
    bool existed = findBestVertex(e, best, geometryPhase);

    if (!best) {
      if (frontBased) {
        rejected.push_back(e);

        continue;
      } else {
        // #if USE_OPENGL
        //                 e->setColor(1.0, 0.0, 0.0);

        //                for (EdgeList::iterator iter = rejected.begin();
        //                     iter != rejected.end(); iter++)
        //                {
        //                    (*iter)->setColor(1.0, 0.0, 1.0);
        //                }
        // #endif //#if USE_OPENGL
        rejected.push_front(e);

        return (geometryPhase) ? ADVF_GEOMETRY_EDGE_REJECTED_TWICE
                               : ADVF_TOPOLOGY_EDGE_REJECTED_TWICE;
      }
    }

    if (geometryPhase && !existed && !quadtree->in(best)) {
      if (frontBased) {
        rejected.push_back(e);
      } else {
        front.push_back(e);
      }

      vertices.remove(best);
      innerVertices.remove(best);
      frontVertices.remove(best);

      delete best;

      continue;
    }

    Edge *e1, *e2;
    e1 = e2 = NULL;

    if (existed) {
      e1 = findEdge(e->getV1(), best);
      e2 = findEdge(best, e->getV2());
    }

    if (e1) {
      if (frontBased) {
        front.remove(e1);
      }

      rejected.remove(e1);

      e1->setFree(false);
    } else {
      e1 = new Edge(e->getV1(), best, ++lastEdgeId);

      e->getV1()->addAdjacentEdge(e1);
      best->addAdjacentEdge(e1);

      // #if USE_OPENGL
      //             e1->setColor(0.0, 0.0, 1.0);
      // #endif //#if USE_OPENGL

      innerEdges.push_back(e1);
      edges.push_back(e1);

      if (frontBased) {
        insertInFront(last, e1);
      } else {
        rejected.push_back(e1);
      }

      quadtree->findCell(e1);
    }

    if (e2) {
      if (frontBased) {
        front.remove(e2);
      }

      rejected.remove(e2);

      e2->setFree(false);
    } else {
      e2 = new Edge(best, e->getV2(), ++lastEdgeId);

      e->getV2()->addAdjacentEdge(e2);
      best->addAdjacentEdge(e2);

      // #if USE_OPENGL
      //             e2->setColor(0.0, 0.0, 1.0);
      // #endif //#if USE_OPENGL

      innerEdges.push_back(e2);
      edges.push_back(e2);

      if (frontBased) {
        insertInFront(last, e2);
      } else {
        rejected.push_back(e2);
      }

      quadtree->findCell(e2);
    }

    e->setFree(false);

    removeFromFront(e->getV1(), e->getV2(), best);

    Face *f = new Face(e->getV1(), e->getV2(), best, ++lastFaceId);

    mesh.push_back(f);
  }

  return geometryPhase ? ADVF_GEOMETRY_MESH_DONE : ADVF_TOPOLOGY_MESH_DONE;
}

void AdvancingFront::fillMesh() {
  /*for (FaceList::iterator iter = mesh.begin();
        iter != mesh.end(); iter++)
  {
      //cout << "triangulo "
            << "(" << (*iter)->getV1()->getX() << ", " <<
  (*iter)->getV1()->getY() << ") "
            << "(" << (*iter)->getV2()->getX() << ", " <<
  (*iter)->getV2()->getY() << ") "
            << "(" << (*iter)->getV3()->getX() << ", " <<
  (*iter)->getV3()->getY() << ") "
            <<  endl;
  }*/

  VertexList quadVertices = quadtree->getVertices();

  while (!quadVertices.empty()) {
    Vertex *v = quadVertices.front();
    quadVertices.pop_front();

    innerVertices.push_back(v);

    bool found = false;

    for (VertexList::iterator iter = vertices.begin(); iter != vertices.end();
         iter++) {
      if ((*iter) == v) {
        found = true;

        break;
      }
    }

    if (!found) {
      vertices.push_back(v);
    }
  }

  EdgeList quadEdges = quadtree->getEdges();

  while (!quadEdges.empty()) {
    Edge *e = quadEdges.front();
    quadEdges.pop_front();

    innerEdges.push_back(e);

    bool found = false;

    for (EdgeList::iterator iter = edges.begin(); iter != edges.end(); iter++) {
      if ((*iter) == e) {
        found = true;

        break;
      }
    }

    if (!found) {
      edges.push_back(e);
    }
  }

  FaceList quadMesh = quadtree->getMesh();

  mesh.insert(mesh.end(), quadMesh.begin(), quadMesh.end());
}

bool AdvancingFront::laplacianSmoothing(bool &changed) {
  changed = false;

  static const double qualityTolerance = 1.0e-8;

  for (VertexList::iterator iter = innerVertices.begin();
       iter != innerVertices.end(); iter++) {
    Vertex *v = (*iter);

    double weight = 1.0;

    double oldx = v->getX();
    double oldy = v->getY();

    double numx, numy, den;
    numx = numy = den = 0.0;

    EdgeList adjacency = findAdjacentEdges(v);
    FaceList faces = findAdjacentFaces(mesh, v);

    if (faces.empty()) {
      continue;
    }

    double oldAverageQuality = 0.0;
    double oldMinQuality = DBL_MAX;
    unsigned int faceCount = 0;

    for (FaceList::iterator faceIter = faces.begin(); faceIter != faces.end();
         ++faceIter) {
      const double quality = (*faceIter)->quality();
      oldAverageQuality += quality;
      oldMinQuality = std::min(oldMinQuality, quality);
      ++faceCount;
    }

    oldAverageQuality /= static_cast<double>(faceCount);
    const double oldScore = 0.6 * oldAverageQuality + 0.4 * oldMinQuality;

    while (!adjacency.empty()) {
      Edge *e = adjacency.front();

      adjacency.pop_front();

      Vertex *v2 = (e->getV1() == v) ? e->getV2() : e->getV1();

      numx += weight * (v2->getX() - oldx);
      numy += weight * (v2->getY() - oldy);

      den += weight;
    }

    den = (den > TOLERANCE_AFT) ? phi / den : 0;

    numx *= den;
    numy *= den;

    double newx = oldx + numx;
    double newy = oldy + numy;

    v->setPosition(newx, newy);

    bool negativeSurface = false;
    double newAverageQuality = 0.0;
    double newMinQuality = DBL_MAX;

    for (FaceList::iterator iter2 = faces.begin(); iter2 != faces.end();
         iter2++) {
      if ((*iter2)->orientedSurface() <= TOLERANCE_AFT) {
        negativeSurface = true;
        break;
      }

      const double quality = (*iter2)->quality();
      newAverageQuality += quality;
      newMinQuality = std::min(newMinQuality, quality);
    }

    if (negativeSurface) {
      v->setPosition(oldx, oldy);
      continue;
    }

    newAverageQuality /= static_cast<double>(faceCount);
    const double newScore = 0.6 * newAverageQuality + 0.4 * newMinQuality;

    if (newScore > oldScore + qualityTolerance) {
      changed = true;
    } else {
      v->setPosition(oldx, oldy);
    }
  }

  // para ajeitar o mid e o vector das innerEdges
  for (EdgeList::iterator iter = innerEdges.begin(); iter != innerEdges.end();
       iter++) {
    (*iter)->setVertices((*iter)->getV1(), (*iter)->getV2());
  }

  return true;
}

void AdvancingFront::ResetRunStats() { run_stats = AdvancingFrontRunStats(); }

void AdvancingFront::SetAdaptiveContext(int adaptive_step_value,
                                        int patch_index_value) {
  run_stats.adaptive_step = adaptive_step_value;
  run_stats.patch_index = patch_index_value;
}

void AdvancingFront::UpdateRunStatsFromQuadtree() {
  if (!quadtree) {
    return;
  }
  run_stats.quadtree_leaf_count =
      static_cast<unsigned long long>(quadtree->getLeaves().size());
  const QuadtreeTemplateStats& template_stats = quadtree->GetTemplateStats();
  run_stats.quadtree_template_count = template_stats.total_templates;
  run_stats.quadtree_low_score_count = template_stats.low_score_templates;
  run_stats.quadtree_low_quality_subdivide_hits =
      template_stats.low_quality_subdivide_hits;
  run_stats.quadtree_template_score_min =
      template_stats.total_templates > 0 ? template_stats.overall_score_min : 0.0;
  run_stats.quadtree_template_score_mean =
      template_stats.total_templates > 0
          ? template_stats.overall_score_sum /
                static_cast<double>(template_stats.total_templates)
          : 0.0;
}

void AdvancingFront::CaptureStageStats(const FaceList& faces, int stage) {
  const StageFaceStats stats = ComputeStageFaceStats(faces);
  if (stage == 0) {
    run_stats.quality_pre_aft_min = stats.quality_min;
    run_stats.quality_pre_aft_mean = stats.quality_mean;
    run_stats.poor_ratio_pre_aft = stats.poor_ratio;
    run_stats.degenerate_pre_aft_count = stats.degenerate_count;
  } else if (stage == 1) {
    run_stats.quality_post_aft_min = stats.quality_min;
    run_stats.quality_post_aft_mean = stats.quality_mean;
    run_stats.poor_ratio_post_aft = stats.poor_ratio;
    run_stats.boundary_element_ratio = stats.boundary_ratio;
    run_stats.transition_element_ratio = stats.transition_ratio;
    run_stats.internal_element_ratio = stats.internal_ratio;
    run_stats.degenerate_post_aft_count = stats.degenerate_count;
  } else if (stage == 2) {
    run_stats.quality_post_smoothing_min = stats.quality_min;
    run_stats.quality_post_smoothing_mean = stats.quality_mean;
    run_stats.poor_ratio_post_smoothing = stats.poor_ratio;
    run_stats.degenerate_post_smoothing_count = stats.degenerate_count;
  }
}

bool AdvancingFront::qualityShapeSmoothing(bool &changed) {
  changed = false;

  static const double qualityTolerance = 1.0e-8;
  static const double sqrt3Half = 0.8660254037844386;

  for (VertexList::iterator iter = innerVertices.begin();
       iter != innerVertices.end(); ++iter) {
    Vertex *v = (*iter);
    FaceList faces = findAdjacentFaces(mesh, v);

    if (faces.empty()) {
      continue;
    }

    const double oldx = v->getX();
    const double oldy = v->getY();
    const double oldScore = ComputeLocalQualityScore(faces);

    double targetX = 0.0;
    double targetY = 0.0;
    double totalWeight = 0.0;

    for (FaceList::iterator faceIter = faces.begin(); faceIter != faces.end();
         ++faceIter) {
      Face *face = (*faceIter);
      Vertex *a = NULL;
      Vertex *b = NULL;

      if (!FaceOppositeEdge(face, v, a, b)) {
        continue;
      }

      const double dx = b->getX() - a->getX();
      const double dy = b->getY() - a->getY();
      const double edgeLen = sqrt(dx * dx + dy * dy);

      if (edgeLen <= TOLERANCE_AFT) {
        continue;
      }

      const double midx = 0.5 * (a->getX() + b->getX());
      const double midy = 0.5 * (a->getY() + b->getY());
      const double nx = -dy / edgeLen;
      const double ny = dx / edgeLen;
      const double orientation = v->orientedSurface(a, b);
      const double sign = (orientation >= 0.0) ? 1.0 : -1.0;

      const double idealX = midx + sign * sqrt3Half * edgeLen * nx;
      const double idealY = midy + sign * sqrt3Half * edgeLen * ny;
      const double qualityPenalty = std::max(0.05, 1.0 - face->quality());
      const double weight = qualityPenalty * qualityPenalty;

      targetX += weight * idealX;
      targetY += weight * idealY;
      totalWeight += weight;
    }

    if (totalWeight <= TOLERANCE_AFT) {
      continue;
    }

    targetX /= totalWeight;
    targetY /= totalWeight;

    double blend = std::min(0.6, std::max(0.3, phi + 0.15));
    bool accepted = false;

    for (int trial = 0; trial < 4; ++trial) {
      const double candidateX = oldx + blend * (targetX - oldx);
      const double candidateY = oldy + blend * (targetY - oldy);

      v->setPosition(candidateX, candidateY);

      bool negativeSurface = false;
      for (FaceList::iterator faceIter = faces.begin(); faceIter != faces.end();
           ++faceIter) {
        if ((*faceIter)->orientedSurface() <= TOLERANCE_AFT) {
          negativeSurface = true;
          break;
        }
      }

      if (!negativeSurface) {
        const double newScore = ComputeLocalQualityScore(faces);
        if (newScore > oldScore + qualityTolerance) {
          changed = true;
          accepted = true;
          break;
        }
      }

      v->setPosition(oldx, oldy);
      blend *= 0.5;
    }

    if (!accepted) {
      v->setPosition(oldx, oldy);
    }
  }

  for (EdgeList::iterator iter = innerEdges.begin(); iter != innerEdges.end();
       ++iter) {
    (*iter)->setVertices((*iter)->getV1(), (*iter)->getV2());
  }

  return true;
}

void AdvancingFront::setBoundarySorted(bool is_boundary_sorted) {
  this->boundarySorted = is_boundary_sorted;
}

void AdvancingFront::setNumImproves(unsigned int improvement_count) {
  this->numImproves = improvement_count;
}

unsigned int AdvancingFront::getNumImproves() { return numImproves; }

bool AdvancingFront::isBoundarySorted() { return boundarySorted; }

// double AdvancingFront::getTolerance()
//{
//    return Shape::tolerance;
//}

void AdvancingFront::setBoundary(Boundary *new_boundary) {
  this->boundary = new_boundary;
}

Boundary *AdvancingFront::getBoundary() { return boundary; }

void AdvancingFront::setQuadtree(Quadtree *new_quadtree) {
  this->quadtree = new_quadtree;
}

Quadtree *AdvancingFront::getQuadtree() { return quadtree; }

VertexList AdvancingFront::getVertices() { return vertices; }

VertexList AdvancingFront::getInnerVertices() { return innerVertices; }

EdgeList AdvancingFront::getEdges() { return edges; }

EdgeList AdvancingFront::getInnerEdges() { return innerEdges; }

FaceList AdvancingFront::getMesh() { return mesh; }

void AdvancingFront::addVertices(VertexList new_vertices) {
  if (!lastVertexId) {
    lastVertexId = boundary->getLastVertexId();
  }

  while (!new_vertices.empty()) {
    Vertex *v = new_vertices.front();
    new_vertices.pop_front();

    v->setId(++lastVertexId);

    this->vertices.push_back(v);
    this->innerVertices.push_back(v);
    this->frontVertices.push_back(v);
  }
}

void AdvancingFront::addEdges(EdgeList new_edges) {
  if (!lastEdgeId) {
    lastEdgeId = boundary->getLastEdgeId();
  }

  while (!new_edges.empty()) {
    Edge *e = new_edges.front();
    new_edges.pop_front();

    e->setId(++lastEdgeId);

    this->edges.push_back(e);
    this->innerEdges.push_back(e);
  }
}

void AdvancingFront::addMesh(FaceList new_faces) {
  while (!new_faces.empty()) {
    Face *f = new_faces.front();
    new_faces.pop_front();

    f->setId(++lastFaceId);

    this->mesh.push_back(f);
  }
}

bool AdvancingFront::belongsToAdvFront(Edge *e) {
  for (EdgeList::iterator iter = front.begin(); iter != front.end(); iter++) {
    if ((*iter) == e) {
      return true;
    }
  }

  return false;
}

enum MethodStatus AdvancingFront::makeGeometryBasedMesh() {
  if (front.size() < 3) {
    return ADVF_EMPTY;
  }

  if (front.size() == 3) {
    Vertex *v = (*(++front.begin()))->getV1();
    Face *f = new Face(front.front()->getV1(), v, front.back()->getV1(),
                       ++lastFaceId);
    mesh.push_back(f);

    return ADVF_GEOMETRY_MESH_DONE;
  }

  if (boundarySorted) {
    sortFront();
  }

  enum MethodStatus status = makeMesh(true);

  if (status == ADVF_GEOMETRY_MESH_DONE) {
    status = makeMesh(false);
  }

  return status;
}

enum MethodStatus AdvancingFront::makeTopologyBasedMesh() {
  // cout << "AdvancingFront::makeTopologyBasedMesh()" << endl;

  if (rejected.empty()) {
    return ADVF_TOPOLOGY_MESH_DONE;
  }

  while (!rejected.empty()) {
    Edge *e = rejected.front();
    rejected.pop_front();

    // #if USE_OPENGL
    //         if (boundary->belongs(e))
    //         {
    //             //figura
    //             //e->setColor(1.0, 1.0, 1.0);
    //             e->setColor(0.0, 0.0, 0.0);
    //             //endfigura
    //         }
    //         else
    //         {
    //             e->setColor(0.0, 0.0, 1.0);
    //         }
    // #endif //#if USE_OPENGL

    front.push_back(e);
  }

  enum MethodStatus status = makeMesh(true, false);

  if (!rejected.empty()) {
    // cout << "rejected nao tah vazio, mas era para estar" << endl;
  }

  if (status == ADVF_TOPOLOGY_MESH_DONE) {
    status = makeMesh(false, false);
  }

  return status;
}

enum MethodStatus AdvancingFront::improveMesh() {
  for (unsigned int i = 0; i < numImproves; i++) {
    bool movedSomeVertex;
    bool shapeOptimized;

    laplacianSmoothing(movedSomeVertex);
    qualityShapeSmoothing(shapeOptimized);

    bool localImproved = false;
    for (int pass = 0; pass < AFT_LOCAL_POSTPROCESS_PASSES; ++pass) {
      bool passChanged = false;
      poorRegionShapeSmoothing(passChanged);
      localImproved = localImproved || passChanged;
      if (!passChanged) {
        break;
      }
    }

    if (!movedSomeVertex && !shapeOptimized && !localImproved) {
      break;
    }
  }

  if (run_stats.adaptive_step == 2 &&
      (run_stats.quality_post_aft_min < 0.22 ||
       run_stats.poor_ratio_post_aft > 0.20)) {
    for (int pass = 0; pass < std::max(2, AFT_LOCAL_POSTPROCESS_PASSES + 1);
         ++pass) {
      bool passChanged = false;
      poorRegionShapeSmoothing(passChanged);
      if (!passChanged) {
        break;
      }
    }
  }

  return IMPR_IMPROVEMENT_DONE;
}

bool AdvancingFront::execute(const FaceList &oldmesh) {
  enum MethodStatus status;
  const int adaptive_step_value = run_stats.adaptive_step;
  const int patch_index_value = run_stats.patch_index;
  ResetRunStats();
  run_stats.adaptive_step = adaptive_step_value;
  run_stats.patch_index = patch_index_value;

  // FaceList emptymesh;

  status = quadtree->generate(oldmesh);

  // cout << methodNotices[status] << endl;

  if (status != QUAD_INITIAL_TREE_DONE) {
    return false;
  }

#if USE_PRINT_COMENT
  cout << "gerou a quadtree" << endl;
#endif  // #if USE_PRINT_COMENT

  /*if (quadtree->refineToLevel() != QUAD_REFINE_TO_LEVEL_DONE)
  {
      return false;
  }*/

  status = quadtree->refineAccordingToNeighbors();

  // cout << methodNotices[status] << endl;

  if (status != QUAD_REFINE_ACCORDING_TO_NEIGHBORS_DONE) {
    return false;
  }

#if USE_PRINT_COMENT
  cout << "refinou a quadtree" << endl;
#endif  // #if USE_PRINT_COMENT

  if (USE_TEMPLATE == std::string("y")) {
    status = quadtree->makeTemplateBasedMesh();
    // cout << methodNotices[status] << endl;
    if (status != QUAD_MAKE_TEMPLATE_BASED_MESH_DONE) {
      return false;
    }
  }
  CaptureStageStats(quadtree->getMesh(), 0);

#if USE_PRINT_COMENT
  cout << "gerou malha por templates" << endl;
#endif  // #if USE_PRINT_COMENT

  makeInitialFront();

  status = makeGeometryBasedMesh();

#if USE_PRINT_COMENT
  cout << "gerou malha por avanco de fronteira - geometria" << endl;
#endif  // #if USE_PRINT_COMENT

  if ((status != ADVF_GEOMETRY_MESH_DONE) &&
      (status != ADVF_GEOMETRY_EDGE_REJECTED_TWICE)) {
    return false;
  }

  if (status == ADVF_GEOMETRY_EDGE_REJECTED_TWICE) {
    status = makeTopologyBasedMesh();

    if (status != ADVF_TOPOLOGY_MESH_DONE) {
      return false;
    }
  }

#if USE_PRINT_COMENT
  cout << "gerou malha por avanco de fronteira - topologia" << endl;
#endif  // #if USE_PRINT_COMENT

  fillMesh();
  UpdateRunStatsFromQuadtree();
  CaptureStageStats(mesh, 1);

  status = improveMesh();

  if (status != IMPR_IMPROVEMENT_DONE) {
    return false;
  }
  CaptureStageStats(mesh, 2);

#if USE_PRINT_COMENT
  cout << "melhorou a malha" << endl;
#endif  // #if USE_PRINT_COMENT

  return true;
}

bool AdvancingFront::poorRegionShapeSmoothing(bool &changed) {
  changed = false;

  if (AFT_LOCAL_POSTPROCESS_PASSES <= 0 ||
      AFT_LOCAL_POSTPROCESS_QUALITY_THRESHOLD <= 0.0) {
    return true;
  }

  static const double qualityTolerance = 1.0e-8;
  static const double sqrt3Half = 0.8660254037844386;

  for (VertexList::iterator iter = innerVertices.begin();
       iter != innerVertices.end(); ++iter) {
    Vertex *v = (*iter);
    FaceList faces = findAdjacentFaces(mesh, v);
    if (faces.empty()) {
      continue;
    }

    bool touchesPoorFace = false;
    for (FaceList::iterator faceIter = faces.begin(); faceIter != faces.end();
         ++faceIter) {
      if ((*faceIter)->quality() < AFT_LOCAL_POSTPROCESS_QUALITY_THRESHOLD) {
        touchesPoorFace = true;
        break;
      }
    }
    if (!touchesPoorFace) {
      continue;
    }

    ++run_stats.postprocess_vertices_considered;
    const double oldx = v->getX();
    const double oldy = v->getY();
    const double oldScore = ComputeLocalQualityScore(faces);

    double targetX = 0.0;
    double targetY = 0.0;
    double totalWeight = 0.0;

    for (FaceList::iterator faceIter = faces.begin(); faceIter != faces.end();
         ++faceIter) {
      Face *face = (*faceIter);
      Vertex *a = NULL;
      Vertex *b = NULL;

      if (!FaceOppositeEdge(face, v, a, b)) {
        continue;
      }

      const double quality = face->quality();
      if (quality >= AFT_LOCAL_POSTPROCESS_QUALITY_THRESHOLD) {
        continue;
      }

      const double dx = b->getX() - a->getX();
      const double dy = b->getY() - a->getY();
      const double edgeLen = sqrt(dx * dx + dy * dy);
      if (edgeLen <= TOLERANCE_AFT) {
        continue;
      }

      const double midx = 0.5 * (a->getX() + b->getX());
      const double midy = 0.5 * (a->getY() + b->getY());
      const double nx = -dy / edgeLen;
      const double ny = dx / edgeLen;
      const double orientation = v->orientedSurface(a, b);
      const double sign = (orientation >= 0.0) ? 1.0 : -1.0;
      const double idealX = midx + sign * sqrt3Half * edgeLen * nx;
      const double idealY = midy + sign * sqrt3Half * edgeLen * ny;
      const double deficit =
          std::max(0.0, AFT_LOCAL_POSTPROCESS_QUALITY_THRESHOLD - quality);
      const double weight = std::max(0.05, deficit * deficit);
      targetX += weight * idealX;
      targetY += weight * idealY;
      totalWeight += weight;
    }

    if (totalWeight <= TOLERANCE_AFT) {
      continue;
    }

    targetX /= totalWeight;
    targetY /= totalWeight;

    double blend = AFT_LOCAL_POSTPROCESS_BLEND;
    bool accepted = false;
    for (int trial = 0; trial < 4; ++trial) {
      v->setPosition(oldx + blend * (targetX - oldx),
                     oldy + blend * (targetY - oldy));

      bool negativeSurface = false;
      for (FaceList::iterator faceIter = faces.begin(); faceIter != faces.end();
           ++faceIter) {
        if ((*faceIter)->orientedSurface() <= TOLERANCE_AFT) {
          negativeSurface = true;
          break;
        }
      }

      if (!negativeSurface) {
        const double newScore = ComputeLocalQualityScore(faces);
        if (newScore > oldScore + qualityTolerance) {
          changed = true;
          accepted = true;
          ++run_stats.postprocess_vertices_moved;
          break;
        }
      }

      v->setPosition(oldx, oldy);
      blend *= 0.5;
    }

    if (!accepted) {
      v->setPosition(oldx, oldy);
    }
  }

  for (EdgeList::iterator iter = innerEdges.begin(); iter != innerEdges.end();
       ++iter) {
    (*iter)->setVertices((*iter)->getV1(), (*iter)->getV2());
  }

  return true;
}

const AdvancingFrontRunStats& AdvancingFront::GetRunStats() const {
  return run_stats;
}

string AdvancingFront::getText() {
  string s;

  return s;
}

// #if USE_OPENGL
//  void AdvancingFront::highlight()
//{

//}

// void AdvancingFront::unhighlight()
//{

//}

// void AdvancingFront::draw()
//{
//    static Edge e;

//    //figura
//    //e.setColor(0.0, 0.0, 1.0);
//    //endfigura

//    if (!edges.empty())
//    {
//        for (FaceList::iterator iter = mesh.begin();
//             iter != mesh.end(); iter++)
//        {
//            (*iter)->draw();
//        }

//        for (EdgeList::iterator iter = edges.begin();
//             iter != edges.end(); iter++)
//        {
//            (*iter)->draw();
//        }
//    }
//    else
//    {
//        for (FaceList::iterator iter = mesh.begin();
//             iter != mesh.end(); iter++)
//        {
//            e.setVertices((*iter)->getV1(), (*iter)->getV2());
//            e.draw();
//            e.setVertices((*iter)->getV2(), (*iter)->getV3());
//            e.draw();
//            e.setVertices((*iter)->getV3(), (*iter)->getV1());
//            e.draw();
//        }
//    }

//    for (VertexList::iterator iter = vertices.begin();
//         iter != vertices.end(); iter++)
//    {
//        (*iter)->highlight();

//        (*iter)->draw();

//        (*iter)->unhighlight();
//    }

//    e.setVertices(NULL, NULL);

// #if DEBUG_MODE
//     if (!vertices.empty())
//     {
//         vertices.back()->drawCircle();
//     }
// #endif //#if DEBUG_MODE
// }

// void AdvancingFront::drawNormals()
//{
//    for (EdgeList::iterator iter = edges.begin();
//         iter != edges.end(); iter++)
//    {
//        (*iter)->drawNormal();
//    }
//}
// #endif //#if USE_OPENGL
