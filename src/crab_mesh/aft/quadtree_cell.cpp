#include "../../../include/crab_mesh/aft/quadtree_cell.h"

#include <array>
#include <cfloat>
#include <cmath>
#include <vector>

extern double QUADTREE_FACE_QUALITY_THRESHOLD;
extern double QUADTREE_LOW_QUALITY_H_FACTOR;

namespace {
double TemplateTriangleQuality(Vertex* a, Vertex* b, Vertex* c) {
  const double abx = b->getX() - a->getX();
  const double aby = b->getY() - a->getY();
  const double bcx = c->getX() - b->getX();
  const double bcy = c->getY() - b->getY();
  const double cax = a->getX() - c->getX();
  const double cay = a->getY() - c->getY();

  const double lab = sqrt(abx * abx + aby * aby);
  const double lbc = sqrt(bcx * bcx + bcy * bcy);
  const double lca = sqrt(cax * cax + cay * cay);
  const double perimeter = lab + lbc + lca;
  if (perimeter <= DBL_EPSILON) {
    return 0.0;
  }

  const double area2 = fabs((b->getX() - a->getX()) * (c->getY() - a->getY()) -
                            (b->getY() - a->getY()) * (c->getX() - a->getX()));
  if (area2 <= DBL_EPSILON) {
    return 0.0;
  }

  const double area = 0.5 * area2;
  const double r = (2.0 * area) / perimeter;
  const double R = (lab * lbc * lca) / (4.0 * area);
  if (R <= DBL_EPSILON) {
    return 0.0;
  }

  return (2.0 * r) / R;
}

double TemplateQualityScore(
    const std::vector<std::array<Vertex*, 3>>& triangles) {
  if (triangles.empty()) {
    return 0.0;
  }

  double min_quality = DBL_MAX;
  double sum_quality = 0.0;
  for (const auto& triangle : triangles) {
    const double quality =
        TemplateTriangleQuality(triangle[0], triangle[1], triangle[2]);
    min_quality = std::min(min_quality, quality);
    sum_quality += quality;
  }

  return 0.75 * min_quality +
         0.25 * (sum_quality / static_cast<double>(triangles.size()));
}

double TemplateQualityScoreFromFaces(Face** faces, const int num_faces) {
  if (num_faces <= 0) {
    return 0.0;
  }

  double min_quality = DBL_MAX;
  double sum_quality = 0.0;
  for (int i = 0; i < num_faces; ++i) {
    const double quality = faces[i]->quality();
    min_quality = std::min(min_quality, quality);
    sum_quality += quality;
  }

  return 0.75 * min_quality + 0.25 * (sum_quality / static_cast<double>(num_faces));
}
}  // namespace

QuadtreeCell::QuadtreeCell(long int cell_id,
                           // MainDrive *mainDrive,
                           Quadtree *owner_tree, Vertex *minimum,
                           Vertex *maximum, QuadtreeCell *parent_cell,
                           unsigned long int cell_level)
    : Shape(cell_id) {
  setQuadtree(owner_tree);
  setBox(minimum, maximum);
  setParent(parent_cell);
  setLevel(cell_level);

  children[QUAD_BOTTOM_LEFT] = children[QUAD_BOTTOM_RIGHT] =
      children[QUAD_TOP_RIGHT] = children[QUAD_TOP_LEFT] = NULL;

  mids = NULL;
  // #if USE_OPENGL
  //     setColor(tree->getR(), tree->getG(), tree->getB());
  //     highlighted = highlightedAsNeighbor = false;
  // #endif //#if USE_OPENGL

  for (int i = 0; i < 8; i++) {
    v[i] = NULL;
    e[i] = NULL;
  }
}

QuadtreeCell::~QuadtreeCell() {
  if (mids) {
    delete mids[QUAD_BOTTOM];
    delete mids[QUAD_RIGHT];
    delete mids[QUAD_TOP];
    delete mids[QUAD_LEFT];
    delete mids[QUAD_MID];

    delete[] mids;
  }

  if (subdivided()) {
    delete children[QUAD_BOTTOM_LEFT];
    delete children[QUAD_BOTTOM_RIGHT];
    delete children[QUAD_TOP_RIGHT];
    delete children[QUAD_TOP_LEFT];
  }

  cell_edges.clear();

  neighbors[QUAD_BOTTOM].clear();
  neighbors[QUAD_RIGHT].clear();
  neighbors[QUAD_TOP].clear();
  neighbors[QUAD_LEFT].clear();
}

Vertex **QuadtreeCell::makeMids() {
  Vertex **m = NULL;

  if (min && max) {
    /*
     *        -----mids[2]---max
     *       |                |
     *    mids[3]  mids[4]  mids[1]
     *       |                |
     *      min----mids[0]----
     *
     */

    double x = (min->getX() + max->getX()) / 2.0;
    double y = (min->getY() + max->getY()) / 2.0;

    m = new Vertex *[5];
    m[QUAD_BOTTOM] = new Vertex(x, min->getY());
    m[QUAD_RIGHT] = new Vertex(max->getX(), y);
    m[QUAD_TOP] = new Vertex(x, max->getY());
    m[QUAD_LEFT] = new Vertex(min->getX(), y);
    m[QUAD_MID] = new Vertex(x, y);
  }

  return m;
}

void QuadtreeCell::subdivide(bool setChildrenEdges) {
  /*
   *        -------------------
   *       | chil[3] | chil[2] |
   *       |---------|---------|
   *       | chil[0] | chil[1] |
   *        -------------------
   *
   */

  mids = makeMids();

  children[QUAD_BOTTOM_LEFT] =
      new QuadtreeCell(tree->getCellId(), /*mainDrive, */ tree, min,
                       mids[QUAD_MID], this, level + 1);
  children[QUAD_BOTTOM_RIGHT] =
      new QuadtreeCell(tree->getCellId(), /*mainDrive, */ tree,
                       mids[QUAD_BOTTOM], mids[QUAD_RIGHT], this, level + 1);
  children[QUAD_TOP_RIGHT] =
      new QuadtreeCell(tree->getCellId(), /*mainDrive, */ tree, mids[QUAD_MID],
                       max, this, level + 1);
  children[QUAD_TOP_LEFT] =
      new QuadtreeCell(tree->getCellId(), /*mainDrive, */ tree, mids[QUAD_LEFT],
                       mids[QUAD_TOP], this, level + 1);

  setChildrenNeighbors();

  if (setChildrenEdges) {
    this->setChildrenEdges();
  }

  tree->addLeaves(this);

  neighbors[QUAD_BOTTOM].clear();
  neighbors[QUAD_RIGHT].clear();
  neighbors[QUAD_TOP].clear();
  neighbors[QUAD_LEFT].clear();
}

void QuadtreeCell::setChildrenNeighbors() {
  children[QUAD_BOTTOM_LEFT]->neighbors[QUAD_RIGHT].push_back(
      children[QUAD_BOTTOM_RIGHT]);
  children[QUAD_BOTTOM_LEFT]->neighbors[QUAD_TOP].push_back(
      children[QUAD_TOP_LEFT]);

  children[QUAD_BOTTOM_RIGHT]->neighbors[QUAD_TOP].push_back(
      children[QUAD_TOP_RIGHT]);
  children[QUAD_BOTTOM_RIGHT]->neighbors[QUAD_LEFT].push_back(
      children[QUAD_BOTTOM_LEFT]);

  children[QUAD_TOP_RIGHT]->neighbors[QUAD_BOTTOM].push_back(
      children[QUAD_BOTTOM_RIGHT]);
  children[QUAD_TOP_RIGHT]->neighbors[QUAD_LEFT].push_back(
      children[QUAD_TOP_LEFT]);

  children[QUAD_TOP_LEFT]->neighbors[QUAD_BOTTOM].push_back(
      children[QUAD_BOTTOM_LEFT]);
  children[QUAD_TOP_LEFT]->neighbors[QUAD_RIGHT].push_back(
      children[QUAD_TOP_RIGHT]);

  for (QuadtreeCellList::iterator iter = neighbors[QUAD_BOTTOM].begin();
       iter != neighbors[QUAD_BOTTOM].end(); iter++) {
    QuadtreeCell *q = (*iter);

    q->neighbors[QUAD_TOP].remove(this);

    if (q->min->getX() < children[QUAD_BOTTOM_LEFT]->max->getX()) {
      children[QUAD_BOTTOM_LEFT]->neighbors[QUAD_BOTTOM].push_back(q);
      q->neighbors[QUAD_TOP].push_back(children[QUAD_BOTTOM_LEFT]);
    }

    if (q->max->getX() > children[QUAD_BOTTOM_RIGHT]->min->getX()) {
      children[QUAD_BOTTOM_RIGHT]->neighbors[QUAD_BOTTOM].push_back(q);
      q->neighbors[QUAD_TOP].push_back(children[QUAD_BOTTOM_RIGHT]);
    }
  }

  for (QuadtreeCellList::iterator iter = neighbors[QUAD_RIGHT].begin();
       iter != neighbors[QUAD_RIGHT].end(); iter++) {
    QuadtreeCell *q = (*iter);

    q->neighbors[QUAD_LEFT].remove(this);

    if (q->min->getY() < children[QUAD_BOTTOM_RIGHT]->max->getY()) {
      children[QUAD_BOTTOM_RIGHT]->neighbors[QUAD_RIGHT].push_back(q);
      q->neighbors[QUAD_LEFT].push_back(children[QUAD_BOTTOM_RIGHT]);
    }

    if (q->max->getY() > children[QUAD_TOP_RIGHT]->min->getY()) {
      children[QUAD_TOP_RIGHT]->neighbors[QUAD_RIGHT].push_back(q);
      q->neighbors[QUAD_LEFT].push_back(children[QUAD_TOP_RIGHT]);
    }
  }

  for (QuadtreeCellList::iterator iter = neighbors[QUAD_TOP].begin();
       iter != neighbors[QUAD_TOP].end(); iter++) {
    QuadtreeCell *q = (*iter);

    q->neighbors[QUAD_BOTTOM].remove(this);

    if (q->max->getX() > children[QUAD_TOP_RIGHT]->min->getX()) {
      children[QUAD_TOP_RIGHT]->neighbors[QUAD_TOP].push_back(q);
      q->neighbors[QUAD_BOTTOM].push_back(children[QUAD_TOP_RIGHT]);
    }

    if (q->min->getX() < children[QUAD_TOP_LEFT]->max->getX()) {
      children[QUAD_TOP_LEFT]->neighbors[QUAD_TOP].push_back(q);
      q->neighbors[QUAD_BOTTOM].push_back(children[QUAD_TOP_LEFT]);
    }
  }

  for (QuadtreeCellList::iterator iter = neighbors[QUAD_LEFT].begin();
       iter != neighbors[QUAD_LEFT].end(); iter++) {
    QuadtreeCell *q = (*iter);

    q->neighbors[QUAD_RIGHT].remove(this);

    if (q->max->getY() > children[QUAD_TOP_LEFT]->min->getY()) {
      children[QUAD_TOP_LEFT]->neighbors[QUAD_LEFT].push_back(q);
      q->neighbors[QUAD_RIGHT].push_back(children[QUAD_TOP_LEFT]);
    }

    if (q->min->getY() < children[QUAD_BOTTOM_LEFT]->max->getY()) {
      children[QUAD_BOTTOM_LEFT]->neighbors[QUAD_LEFT].push_back(q);
      q->neighbors[QUAD_RIGHT].push_back(children[QUAD_BOTTOM_LEFT]);
    }
  }
}

void QuadtreeCell::setChildrenEdges() {
  for (EdgeList::iterator iter = cell_edges.begin();
       iter != cell_edges.end(); iter++) {
    Edge *edge = (*iter);

    tree->findCell(edge);
  }
}

Edge *QuadtreeCell::makeE(int i) {
  int v1 = -1, v2 = -1;

  switch (i) {
    case 0:
      v1 = v[1] ? 1 : 2;
      v2 = 0;
      break;
    case 1:
      v1 = 2;
      v2 = 1;
      break;
    case 2:
      v1 = v[3] ? 3 : 4;
      v2 = 2;
      break;
    case 3:
      v1 = 4;
      v2 = 3;
      break;
    case 4:
      v1 = v[5] ? 5 : 6;
      v2 = 4;
      break;
    case 5:
      v1 = 6;
      v2 = 5;
      break;
    case 6:
      v1 = v[7] ? 7 : 0;
      v2 = 6;
      break;
    case 7:
      v1 = 0;
      v2 = 7;
      break;
  }

  return makeE(v1, v2);
}

Edge *QuadtreeCell::makeE(int i, int j) { return makeE(v[i], v[j]); }

Edge *QuadtreeCell::makeE(Vertex *v1, Vertex *v2) {
  Edge *new_edge = new Edge(v1, v2, tree->edgeId());

  v1->addAdjacentEdge(new_edge);
  v2->addAdjacentEdge(new_edge);

  new_edge->setFree(false);

  return new_edge;
}

void QuadtreeCell::makeMeshType1() {
  //	    v[6] -------------e[4]------------- v[4]
  //        | \                                 |
  //        |   \                               |
  //        |     \                             |
  //        |       \                           |
  //        |         \                         |
  //        |           \                       |
  //        |             \                     |
  //      e[6]              \                 e[2]
  //        |                 \                 |
  //        |                   \               |
  //        |                     \             |
  //        |                       \           |
  //        |                         \         |
  //        |                           \       |
  //        |                             \     |
  //		v[0] ------------ e[0] ------------ v[2]

  std::vector<std::array<Vertex*, 3>> diagonal_faces = {
      {v[0], v[2], v[6]},
      {v[4], v[6], v[2]},
  };
  const double diagonal_score = TemplateQualityScore(diagonal_faces);

  const double center_x = 0.5 * (min->getX() + max->getX());
  const double center_y = 0.5 * (min->getY() + max->getY());
  const double dx = 0.10 * (max->getX() - min->getX());
  const double dy = 0.10 * (max->getY() - min->getY());

  double best_x = center_x;
  double best_y = center_y;
  double best_center_score = -DBL_MAX;

  std::vector<std::pair<double, double>> candidates = {
      {center_x, center_y},
      {center_x - dx, center_y},
      {center_x + dx, center_y},
      {center_x, center_y - dy},
      {center_x, center_y + dy},
      {center_x - 0.6 * dx, center_y - 0.6 * dy},
      {center_x + 0.6 * dx, center_y - 0.6 * dy},
      {center_x + 0.6 * dx, center_y + 0.6 * dy},
      {center_x - 0.6 * dx, center_y + 0.6 * dy},
  };

  for (const auto& candidate : candidates) {
    Vertex probe(candidate.first, candidate.second);
    std::vector<std::array<Vertex*, 3>> center_faces = {
        {v[0], v[2], &probe},
        {v[2], v[4], &probe},
        {v[4], v[6], &probe},
        {v[6], v[0], &probe},
    };
    const double score = TemplateQualityScore(center_faces);
    if (score > best_center_score) {
      best_center_score = score;
      best_x = candidate.first;
      best_y = candidate.second;
    }
  }

  if (best_center_score > diagonal_score + 1.0e-6) {
    const int numEdges = 4;
    const int numFaces = 4;
    const int numVertices = 1;

    Edge** edges = new Edge*[numEdges];
    Face** faces = new Face*[numFaces];
    Vertex** vertices = new Vertex*[numVertices];

    vertices[0] = new Vertex(best_x, best_y, tree->vertexId());

    edges[0] = makeE(v[0], vertices[0]);
    edges[1] = makeE(v[2], vertices[0]);
    edges[2] = makeE(v[4], vertices[0]);
    edges[3] = makeE(v[6], vertices[0]);

    faces[0] = new Face(v[0], v[2], vertices[0], tree->faceId());
    faces[1] = new Face(v[2], v[4], vertices[0], tree->faceId());
    faces[2] = new Face(v[4], v[6], vertices[0], tree->faceId());
    faces[3] = new Face(v[6], v[0], vertices[0], tree->faceId());

    for (int i = 0; i < numEdges; i++) {
      tree->add(edges[i]);
      edges[i]->setCell(this);
    }

    for (int i = 0; i < numFaces; i++) {
      tree->add(faces[i]);
    }

    for (int i = 0; i < numVertices; i++) {
      tree->add(vertices[i]);
    }

    tree->RecordTemplateChoice(1, TemplateQualityScoreFromFaces(faces, numFaces));

    delete[] edges;
    delete[] faces;
    delete[] vertices;
    return;
  }

  int numEdges = 1;
  int numFaces = 2;

  Edge **edges = new Edge *[numEdges];
  Face **faces = new Face *[numFaces];

  edges[0] = makeE(2, 6);

  faces[0] = new Face(v[0], v[2], v[6], tree->faceId());
  faces[1] = new Face(v[4], v[6], v[2], tree->faceId());

  for (int i = 0; i < numEdges; i++) {
    tree->add(edges[i]);

    edges[i]->setCell(this);
  }

  for (int i = 0; i < numFaces; i++) {
    tree->add(faces[i]);
  }

  tree->RecordTemplateChoice(1, TemplateQualityScoreFromFaces(faces, numFaces));

  delete[] edges;
  delete[] faces;
}

void QuadtreeCell::makeMeshType2(int c) {
  //	v[6] ------------ e[4] -------- --- v[4]
  //        |  \                                |
  //        |      \                            |
  //        |          \                        |
  //        |              \                  e[3]
  //        |                  \                |
  //        |                      \            |
  //        |                          \        |
  //      e[6]                             *  v[3]     c == QUAD_R
  //        |                          /        |
  //        |                      /            |
  //        |                  /                |
  //        |              /                  e[2]
  //        |          /                        |
  //        |      /                            |
  //        |  /                                |
  //		v[0] ------------ e[0] ------------ v[2]

  int numEdges = 2;
  int numFaces = 3;

  Edge **edges = new Edge *[numEdges];
  Face **faces = new Face *[numFaces];

  int base = 0;

  switch (c) {
    case QUAD_B:
      base = 1;
      break;
    case QUAD_R:
      base = 3;
      break;
    case QUAD_T:
      base = 5;
      break;
    case QUAD_L:
      base = 7;
      break;
  }

  edges[0] = makeE(base, (base + 5) % 8);
  edges[1] = makeE(base, (base + 3) % 8);

  faces[0] =
      new Face(v[(base + 7) % 8], v[base], v[(base + 5) % 8], tree->faceId());
  faces[1] =
      new Face(v[(base + 5) % 8], v[base], v[(base + 3) % 8], tree->faceId());
  faces[2] =
      new Face(v[(base + 3) % 8], v[base], v[(base + 1) % 8], tree->faceId());

  for (int i = 0; i < numEdges; i++) {
    tree->add(edges[i]);

    edges[i]->setCell(this);
  }

  for (int i = 0; i < numFaces; i++) {
    tree->add(faces[i]);
  }

  tree->RecordTemplateChoice(2, TemplateQualityScoreFromFaces(faces, numFaces));

  delete[] edges;
  delete[] faces;
}

void QuadtreeCell::makeMeshType3(int c) {
  //                    c == QUAD_T
  //	v[6] --- e[5] --- v[5] --- e[4] --- v[4]
  //        | \              |             /    |
  //        |   \            |           /      |
  //        |     \          |         /        |
  //        |       \        |       /          |
  //        |         \      |     /            |
  //        |           \    |   /              |
  //        |             \  | /                |
  //      e[6]               *                e[2]
  //        |             /  | \                |
  //        |           /    |   \              |
  //        |         /      |     \            |
  //        |       /        |       \          |
  //        |     /          |         \        |
  //        |   /            |           \      |
  //        | /              |             \    |
  //		v[0] --- e[0] --- v[1] --- e[1] --- v[2]
  //                    c == QUAD_B

  int numEdges = 6;
  int numFaces = 6;
  int numVertices = 1;

  Edge **edges = new Edge *[numEdges];
  Face **faces = new Face *[numFaces];
  Vertex **vertices = new Vertex *[numVertices];

  int base = 0;

  switch (c) {
    case QUAD_B:
    case QUAD_T:
      base = 5;
      break;
    case QUAD_R:
    case QUAD_L:
      base = 7;
      break;
  }

  const double center_x = 0.5 * (min->getX() + max->getX());
  const double center_y = 0.5 * (min->getY() + max->getY());
  const double dx = 0.10 * (max->getX() - min->getX());
  const double dy = 0.10 * (max->getY() - min->getY());

  double best_x = center_x;
  double best_y = center_y;
  double best_score = -DBL_MAX;

  std::vector<std::pair<double, double>> candidates = {
      {center_x, center_y},
      {center_x - dx, center_y},
      {center_x + dx, center_y},
      {center_x, center_y - dy},
      {center_x, center_y + dy},
      {center_x - 0.6 * dx, center_y - 0.6 * dy},
      {center_x + 0.6 * dx, center_y - 0.6 * dy},
      {center_x + 0.6 * dx, center_y + 0.6 * dy},
      {center_x - 0.6 * dx, center_y + 0.6 * dy},
  };

  for (const auto& candidate : candidates) {
    Vertex probe(candidate.first, candidate.second);
    std::vector<std::array<Vertex*, 3>> probe_faces = {
        {v[base], v[(base + 1) % 8], &probe},
        {v[(base + 1) % 8], v[(base + 3) % 8], &probe},
        {v[(base + 3) % 8], v[(base + 4) % 8], &probe},
        {v[(base + 4) % 8], v[(base + 5) % 8], &probe},
        {v[(base + 5) % 8], v[(base + 7) % 8], &probe},
        {v[(base + 7) % 8], v[base], &probe},
    };
    const double score = TemplateQualityScore(probe_faces);
    if (score > best_score) {
      best_score = score;
      best_x = candidate.first;
      best_y = candidate.second;
    }
  }

  vertices[0] = new Vertex(best_x, best_y, tree->vertexId());

  edges[0] = makeE(v[base], vertices[0]);
  edges[1] = makeE(v[(base + 1) % 8], vertices[0]);
  edges[2] = makeE(v[(base + 3) % 8], vertices[0]);
  edges[3] = makeE(v[(base + 4) % 8], vertices[0]);
  edges[4] = makeE(v[(base + 5) % 8], vertices[0]);
  edges[5] = makeE(v[(base + 7) % 8], vertices[0]);

  faces[0] = new Face(v[base], v[(base + 1) % 8], vertices[0], tree->faceId());
  faces[1] = new Face(v[(base + 1) % 8], v[(base + 3) % 8], vertices[0],
                      tree->faceId());
  faces[2] = new Face(v[(base + 3) % 8], v[(base + 4) % 8], vertices[0],
                      tree->faceId());
  faces[3] = new Face(v[(base + 4) % 8], v[(base + 5) % 8], vertices[0],
                      tree->faceId());
  faces[4] = new Face(v[(base + 5) % 8], v[(base + 7) % 8], vertices[0],
                      tree->faceId());
  faces[5] = new Face(v[(base + 7) % 8], v[base], vertices[0], tree->faceId());

  for (int i = 0; i < numEdges; i++) {
    tree->add(edges[i]);

    edges[i]->setCell(this);
  }

  for (int i = 0; i < numFaces; i++) {
    tree->add(faces[i]);
  }

  for (int i = 0; i < numVertices; i++) {
    tree->add(vertices[i]);
  }

  tree->RecordTemplateChoice(3, TemplateQualityScoreFromFaces(faces, numFaces));

  delete[] edges;
  delete[] faces;
  delete[] vertices;
}

void QuadtreeCell::makeMeshType4(int c) {
  //	v[6] --- e[5] --- v[5] --- e[4] --- v[4]
  //        | \             |  \                |
  //        |   \           |    \              |
  //        |     \         |      \            |
  //        |       \       |        \        e[3]
  //        |         \     |          \        |
  //        |           \   |            \      |
  //        |             \ |              \    |
  //      e[6]              * --------------  v[3]   c == QUAD_R
  //        |             /   \                 |
  //        |           /       \               |
  //        |         /           \             |
  //        |       /               \         e[2]
  //        |     /                   \         |
  //        |   /                       \       |
  //        | /                           \     |
  //		v[0] ------------ e[0] ------------ v[2]

  int numEdges = 6;
  int numFaces = 6;
  int numVertices = 1;

  Edge **edges = new Edge *[numEdges];
  Face **faces = new Face *[numFaces];
  Vertex **vertices = new Vertex *[numVertices];

  int base = 0;

  switch (c) {
    case QUAD_B:
      base = 1;
      break;
    case QUAD_R:
      base = 3;
      break;
    case QUAD_T:
      base = 5;
      break;
    case QUAD_L:
      base = 7;
      break;
  }

  const double center_x = 0.5 * (min->getX() + max->getX());
  const double center_y = 0.5 * (min->getY() + max->getY());
  const double dx = 0.12 * (max->getX() - min->getX());
  const double dy = 0.12 * (max->getY() - min->getY());

  std::pair<double, double> bias(0.0, 0.0);
  switch (c) {
    case QUAD_B:
      bias = {0.0, 0.5 * dy};
      break;
    case QUAD_R:
      bias = {-0.5 * dx, 0.0};
      break;
    case QUAD_T:
      bias = {0.0, -0.5 * dy};
      break;
    case QUAD_L:
      bias = {0.5 * dx, 0.0};
      break;
  }

  double best_x = center_x + bias.first;
  double best_y = center_y + bias.second;
  double best_score = -DBL_MAX;

  std::vector<std::pair<double, double>> candidates = {
      {center_x + bias.first, center_y + bias.second},
      {center_x + bias.first - 0.7 * dx, center_y + bias.second},
      {center_x + bias.first + 0.7 * dx, center_y + bias.second},
      {center_x + bias.first, center_y + bias.second - 0.7 * dy},
      {center_x + bias.first, center_y + bias.second + 0.7 * dy},
  };

  for (const auto& candidate : candidates) {
    Vertex probe(candidate.first, candidate.second);
    std::vector<std::array<Vertex*, 3>> probe_faces = {
        {v[base], v[(base + 2) % 8], &probe},
        {v[(base + 2) % 8], v[(base + 3) % 8], &probe},
        {v[(base + 3) % 8], v[(base + 5) % 8], &probe},
        {v[(base + 5) % 8], v[(base + 7) % 8], &probe},
        {v[(base + 7) % 8], v[base], &probe},
    };
    const double score = TemplateQualityScore(probe_faces);
    if (score > best_score) {
      best_score = score;
      best_x = candidate.first;
      best_y = candidate.second;
    }
  }

  vertices[0] = new Vertex(best_x, best_y, tree->vertexId());

  edges[0] = makeE(base, (base + 2) % 8);
  edges[1] = makeE(v[base], vertices[0]);
  edges[2] = makeE(v[(base + 2) % 8], vertices[0]);
  edges[3] = makeE(v[(base + 3) % 8], vertices[0]);
  edges[4] = makeE(v[(base + 5) % 8], vertices[0]);
  edges[5] = makeE(v[(base + 7) % 8], vertices[0]);

  faces[0] =
      new Face(v[base], v[(base + 1) % 8], v[(base + 2) % 8], tree->faceId());
  faces[1] = new Face(v[base], v[(base + 2) % 8], vertices[0], tree->faceId());
  faces[2] = new Face(v[(base + 2) % 8], v[(base + 3) % 8], vertices[0],
                      tree->faceId());
  faces[3] = new Face(v[(base + 3) % 8], v[(base + 5) % 8], vertices[0],
                      tree->faceId());
  faces[4] = new Face(v[(base + 5) % 8], v[(base + 7) % 8], vertices[0],
                      tree->faceId());
  faces[5] = new Face(v[(base + 7) % 8], v[base], vertices[0], tree->faceId());

  for (int i = 0; i < numEdges; i++) {
    tree->add(edges[i]);

    edges[i]->setCell(this);
  }

  for (int i = 0; i < numFaces; i++) {
    tree->add(faces[i]);
  }

  for (int i = 0; i < numVertices; i++) {
    tree->add(vertices[i]);
  }

  tree->RecordTemplateChoice(4, TemplateQualityScoreFromFaces(faces, numFaces));

  delete[] edges;
  delete[] faces;
  delete[] vertices;
}

void QuadtreeCell::makeMeshType5(int c) {
  //	v[6] --- e[5] --- v[5] --- e[4] --- v[4]
  //        |             /  | \                |
  //        |           /    |  \               |
  //        |         /      |   \              |
  //      e[6]      /        |    \             |
  //        |     /          |     \            |
  //        |   /            |      \           |
  //        | /              |       \          |
  //      v[7]               |        \       e[2]  c == QUAD_R
  //        | \              |         \        |
  //        |   \            |          \       |
  //        |     \          |           \      |
  //      e[7]      \        |            \     |
  //        |         \      |             \    |
  //        |           \    |              \   |
  //        |             \  |               \  |
  //		v[0] --- e[0] --- v[1] --- e[1] --- v[2]

  int numEdges = 4;
  int numFaces = 5;

  Edge **edges = new Edge *[numEdges];
  Face **faces = new Face *[numFaces];

  int base = 0;

  switch (c) {
    case QUAD_B:
      base = 3;
      break;
    case QUAD_R:
      base = 5;
      break;
    case QUAD_T:
      base = 7;
      break;
    case QUAD_L:
      base = 1;
      break;
  }

  edges[0] = makeE(base, (base + 2) % 8);
  edges[1] = makeE(base, (base + 4) % 8);
  edges[2] = makeE(base, (base + 5) % 8);
  edges[3] = makeE((base + 2) % 8, (base + 4) % 8);

  faces[0] =
      new Face(v[(base + 2) % 8], v[base], v[(base + 1) % 8], tree->faceId());
  faces[1] =
      new Face(v[(base + 4) % 8], v[base], v[(base + 2) % 8], tree->faceId());
  faces[2] =
      new Face(v[(base + 5) % 8], v[base], v[(base + 4) % 8], tree->faceId());
  faces[3] =
      new Face(v[(base + 7) % 8], v[base], v[(base + 5) % 8], tree->faceId());
  faces[4] = new Face(v[(base + 3) % 8], v[(base + 4) % 8], v[(base + 2) % 8],
                      tree->faceId());

  for (int i = 0; i < numEdges; i++) {
    tree->add(edges[i]);

    edges[i]->setCell(this);
  }

  for (int i = 0; i < numFaces; i++) {
    tree->add(faces[i]);
  }

  tree->RecordTemplateChoice(5, TemplateQualityScoreFromFaces(faces, numFaces));

  delete[] edges;
  delete[] faces;
}

void QuadtreeCell::makeMeshType6() {
  //	v[6] --- e[5] --- v[5] --- e[4] --- v[4]
  //        |              / | \                |
  //        |            /   |   \              |
  //        |          /     |     \            |
  //      e[6]       /       |       \        e[3]
  //        |      /         |         \        |
  //        |    /           |           \      |
  //        |  /             |             \    |
  //      v[7]               |                v[3]
  //        |  \             |             /    |
  //        |    \           |           /      |
  //        |      \         |         /        |
  //      e[7]       \       |       /        e[2]
  //        |          \     |     /            |
  //        |            \   |   /              |
  //        |              \ | /                |
  //		v[0] --- e[0] --- v[1] --- e[1] --- v[2]

  int numEdges = 5;
  int numFaces = 6;

  Edge **edges = new Edge *[numEdges];
  Face **faces = new Face *[numFaces];

  const double q13 = TemplateTriangleQuality(v[1], v[3], v[7]) +
                     TemplateTriangleQuality(v[3], v[5], v[7]);
  const double q15 = TemplateTriangleQuality(v[1], v[3], v[5]) +
                     TemplateTriangleQuality(v[1], v[5], v[7]);

  if (q13 > q15) {
    edges[0] = makeE(3, 7);
    edges[1] = makeE(1, 3);
    edges[2] = makeE(3, 5);
    edges[3] = makeE(1, 7);
    edges[4] = makeE(5, 7);

    faces[0] = new Face(v[0], v[1], v[7], tree->faceId());
    faces[1] = new Face(v[2], v[3], v[1], tree->faceId());
    faces[2] = new Face(v[4], v[5], v[3], tree->faceId());
    faces[3] = new Face(v[6], v[7], v[5], tree->faceId());
    faces[4] = new Face(v[1], v[3], v[7], tree->faceId());
    faces[5] = new Face(v[3], v[5], v[7], tree->faceId());
  } else {
    edges[0] = makeE(1, 5);
    edges[1] = makeE(1, 3);
    edges[2] = makeE(3, 5);
    edges[3] = makeE(1, 7);
    edges[4] = makeE(7, 5);

    faces[0] = new Face(v[0], v[1], v[7], tree->faceId());
    faces[1] = new Face(v[2], v[3], v[1], tree->faceId());
    faces[2] = new Face(v[4], v[5], v[3], tree->faceId());
    faces[3] = new Face(v[6], v[7], v[5], tree->faceId());
    faces[4] = new Face(v[1], v[3], v[5], tree->faceId());
    faces[5] = new Face(v[1], v[5], v[7], tree->faceId());
  }

  for (int i = 0; i < numEdges; i++) {
    tree->add(edges[i]);

    edges[i]->setCell(this);
  }

  for (int i = 0; i < numFaces; i++) {
    tree->add(faces[i]);
  }

  tree->RecordTemplateChoice(6, TemplateQualityScoreFromFaces(faces, numFaces));

  delete[] edges;
  delete[] faces;
}

void QuadtreeCell::setQuadtree(Quadtree *owner_tree) {
  this->tree = owner_tree;
}

void QuadtreeCell::setLevel(unsigned long int cell_level) {
  this->level = cell_level;
}

unsigned long int QuadtreeCell::getLevel() { return level; }

double QuadtreeCell::getSize() { return size; }

void QuadtreeCell::setParent(QuadtreeCell *parent_cell) {
  this->parent = parent_cell;
}

QuadtreeCell *QuadtreeCell::getParent() { return parent; }

void QuadtreeCell::setChild(int position, QuadtreeCell *child) {
  if ((position >= 0) && (position <= 3)) {
    children[position] = child;
  }
}

QuadtreeCell *QuadtreeCell::getChild(int position) {
  return ((position >= 0) && (position <= 3)) ? children[position] : NULL;
}

void QuadtreeCell::setBox(Vertex *minimum, Vertex *maximum) {
  /*static*/ double spanX, spanY;

  this->min = minimum;
  this->max = maximum;

  spanX = maximum->getX() - minimum->getX();
  spanY = maximum->getY() - minimum->getY();

  size = (spanX > spanY) ? spanX : spanY;
}

void QuadtreeCell::setNeighbors(int direction,
                                QuadtreeCellList adjacent_cells) {
  if ((direction < QUAD_BOTTOM) || (direction > QUAD_LEFT)) {
    return;
  }

  this->neighbors[direction] = adjacent_cells;
}

QuadtreeCellList QuadtreeCell::getNeighbors(int direction) {
  QuadtreeCellList l;

  return (((direction < 0) || (direction > 3)) ? l : neighbors[direction]);
}

int QuadtreeCell::getNumCells() {
  int numCells = 1;

  if (subdivided()) {
    numCells += children[QUAD_BOTTOM_LEFT]->getNumCells();
    numCells += children[QUAD_BOTTOM_RIGHT]->getNumCells();
    numCells += children[QUAD_TOP_RIGHT]->getNumCells();
    numCells += children[QUAD_TOP_LEFT]->getNumCells();
  }

  return numCells;
}

VertexList QuadtreeCell::getVertices() {
  VertexList vertices;

  for (EdgeList::iterator iter = cell_edges.begin();
       iter != cell_edges.end(); iter++) {
    bool found1, found2;
    found1 = found2 = false;

    Vertex *v1 = (*iter)->getV1();
    Vertex *v2 = (*iter)->getV2();

    for (VertexList::iterator iter2 = vertices.begin(); iter2 != vertices.end();
         iter2++) {
      if ((*iter2) == v1) {
        found1 = true;
      }

      if ((*iter2) == v2) {
        found2 = true;
      }

      if (found1 && found2) {
        break;
      }
    }

    if (!found1) {
      vertices.push_back(v1);
    }

    if (!found2) {
      vertices.push_back(v2);
    }
  }

  return vertices;
}

EdgeList QuadtreeCell::getEdges() { return cell_edges; }

bool QuadtreeCell::shouldSubdivide(Edge *edge) {
  if (tree && tree->getMaxLevelCap() >= 0 &&
      static_cast<long int>(level) >= tree->getMaxLevelCap()) {
    return false;
  }
  return (size - edge->getLen() * tree->getFactor() >= TOLERANCE_AFT);
}

bool QuadtreeCell::shouldSubdivide(Face *f) {
  if (tree && tree->getMaxLevelCap() >= 0 &&
      static_cast<long int>(level) >= tree->getMaxLevelCap()) {
    return false;
  }
  // Subdivisão de acordo com as curvaturas dos vértices de um elemento
  if ((f->getId() == 2) || (f->getId() == 4)) {
    // cout << "TRIANGULO " << f->getId() << endl;
    // cout << "size = " << size << endl;
    // cout << "f->h = " << f->h << endl;
    // cout << "size - f->h*tree->getFactor() = " << size -
    // f->h*tree->getFactor() << endl; cout << "deve subdividir = " << (size -
    // f->h*tree->getFactor() >= Shape::tolerance) << endl;
  }
  double target_h = f->h;
  if (QUADTREE_FACE_QUALITY_THRESHOLD > 0.0 &&
      f->quality() < QUADTREE_FACE_QUALITY_THRESHOLD) {
    target_h *= QUADTREE_LOW_QUALITY_H_FACTOR;
  }
  const bool should_subdivide =
      (size - target_h * tree->getFactor() >= TOLERANCE_AFT);
  if (should_subdivide && QUADTREE_FACE_QUALITY_THRESHOLD > 0.0 &&
      f->quality() < QUADTREE_FACE_QUALITY_THRESHOLD &&
      QUADTREE_LOW_QUALITY_H_FACTOR < 1.0) {
    tree->RecordLowQualitySubdivision();
  }
  return should_subdivide;
}

void QuadtreeCell::addEdge(Edge *edge) {
  bool existed = false;

  for (EdgeList::iterator iter = cell_edges.begin();
       iter != cell_edges.end(); iter++) {
    if ((*iter)->equals(edge)) {
      existed = true;
      break;
    }
  }

  if (!existed) {
    cell_edges.push_back(edge);
  }
}

void QuadtreeCell::removeEdge(Edge *edge) {
  cell_edges.remove(edge);

  if (parent) {
    parent->removeEdge(edge);
  }
}

void QuadtreeCell::clearEdges() { cell_edges.clear(); }

QuadtreeCell *QuadtreeCell::findCell(Edge *edge) {
  if (out(edge->getMid())) {
    return NULL;
  }

  if (subdivided()) {
    QuadtreeCell *q = children[QUAD_BOTTOM_LEFT]->findCell(edge);

    if (!q) {
      q = children[QUAD_BOTTOM_RIGHT]->findCell(edge);
    }

    if (!q) {
      q = children[QUAD_TOP_RIGHT]->findCell(edge);
    }

    if (!q) {
      q = children[QUAD_TOP_LEFT]->findCell(edge);
    }

    return q;
  }

  return this;
}

QuadtreeCell *QuadtreeCell::findCell(Face *f) {
  if (out(f->getMid())) {
    return NULL;
  }

  if (subdivided()) {
    QuadtreeCell *q = children[QUAD_BOTTOM_LEFT]->findCell(f);

    if (!q) {
      q = children[QUAD_BOTTOM_RIGHT]->findCell(f);
    }

    if (!q) {
      q = children[QUAD_TOP_RIGHT]->findCell(f);
    }

    if (!q) {
      q = children[QUAD_TOP_LEFT]->findCell(f);
    }

    return q;
  }

  return this;
}

bool QuadtreeCell::in(Vertex *vertex) {
  return (vertex->getX() > min->getX() && vertex->getX() < max->getX() &&
          vertex->getY() > min->getY() && vertex->getY() < max->getY());
}

bool QuadtreeCell::on(Vertex *vertex) {
  return (!in(vertex) && !(out(vertex)));
}

bool QuadtreeCell::out(Vertex *vertex) {
  // cout << "v = (" << v->getX() << ", " << v->getY() << ") " << endl;
  // cout << "box = (" << min->getX() << ", " << min->getY() << "), (" <<
  // max->getX() << ", " << max->getY() << ")" << endl;

  return ((vertex->getX() <= min->getX() - TOLERANCE_AFT) ||
          (vertex->getX() >= max->getX() + TOLERANCE_AFT) ||
          (vertex->getY() <= min->getY() - TOLERANCE_AFT) ||
          (vertex->getY() >= max->getY() + TOLERANCE_AFT));
  /*return ((v->getX() < min->getX()) ||
          (v->getX() > max->getX()) ||
          (v->getY() < min->getY()) ||
          (v->getY() > max->getY()));*/
}

double QuadtreeCell::height() {
  return size;  //(max->getX() - min->getX());
}

double QuadtreeCell::surface() { return size * size; }

Vertex *QuadtreeCell::getMin() { return min; }

Vertex *QuadtreeCell::getMax() { return max; }

bool QuadtreeCell::subdivide(Edge *edge) {
  if (out(edge->getMid())) {
    return false;
  }

  if (!shouldSubdivide(edge)) {
    return false;
  }

  if (!subdivided()) {
    subdivide();
  }

  for (int i = 0; i < 4; i++) {
    children[i]->subdivide(edge);
  }

  return true;

  /*QuadtreeCell *q = findCell(e);

  if (!q)
  {
      return false;
  }

  if (!shouldSubdivide(e))
  {
      return false;
  }

  if (!subdivided())
  {
      subdivide();
  }

  if (subdivided())
  {
      if (children[QUAD_BOTTOM_LEFT]->subdivide(e))
      {
          return true;
      }

      if (children[QUAD_BOTTOM_RIGHT]->subdivide(e))
      {
          return true;
      }

      if (children[QUAD_TOP_RIGHT]->subdivide(e))
      {
          return true;
      }

      if (children[QUAD_TOP_LEFT]->subdivide(e))
      {
          return true;
      }
  }

  return false;*/
}

bool QuadtreeCell::subdivide(Face *f) {
  if (f->getId() == 4) {
    // cout << " ****** TRIANGULO 4 *******" << endl;
  }

  if (out(f->getMid())) {
    return false;
  }

  if (!shouldSubdivide(f)) {
    return false;
  }

  if (f->getId() == 4) {
    // cout << "SUBDIVIDE" << endl;
  }

  if (!subdivided()) {
    subdivide();
  }

  for (int i = 0; i < 4; i++) {
    children[i]->subdivide(f);
  }

  return true;

  /*QuadtreeCell *q = findCell(f);

  if (!q)
  {
      return false;
  }

  if (!shouldSubdivide(f))
  {
      return false;
  }

  if (!subdivided())
  {
      subdivide();
  }

  if (subdivided())
  {
      if (children[QUAD_BOTTOM_LEFT]->subdivide(f))
      {
          return true;
      }

      if (children[QUAD_BOTTOM_RIGHT]->subdivide(f))
      {
          return true;
      }

      if (children[QUAD_TOP_RIGHT]->subdivide(f))
      {
          return true;
      }

      if (children[QUAD_TOP_LEFT]->subdivide(f))
      {
          return true;
      }
  }

  return false;*/
}

void QuadtreeCell::subdivideToLevel(unsigned long int target_level) {
  if ((parent) && (this->level <= target_level) && (cell_edges.empty())) {
    EdgeList pEdges = parent->cell_edges;

    for (EdgeList::iterator iter = pEdges.begin(); iter != pEdges.end();
         iter++) {
      tree->findCell((*iter));
    }
  }

  if (this->level < target_level) {
    if (!subdivided()) {
      subdivide();
    }

    if (subdivided()) {
      children[QUAD_BOTTOM_LEFT]->subdivideToLevel(target_level);
      children[QUAD_BOTTOM_RIGHT]->subdivideToLevel(target_level);
      children[QUAD_TOP_RIGHT]->subdivideToLevel(target_level);
      children[QUAD_TOP_LEFT]->subdivideToLevel(target_level);
    }
  }
}

bool QuadtreeCell::subdivideAccordingToNeighbors() {
  if ((neighbors[QUAD_BOTTOM].size() > 2) ||
      (neighbors[QUAD_RIGHT].size() > 2) || (neighbors[QUAD_TOP].size() > 2) ||
      (neighbors[QUAD_LEFT].size() > 2)) {
    subdivide();

    return true;
  }

  for (int dir = 0; dir < 4; ++dir) {
    for (QuadtreeCellList::iterator iter = neighbors[dir].begin();
         iter != neighbors[dir].end(); ++iter) {
      if ((*iter) && ((*iter)->getLevel() > this->level + 1)) {
        subdivide();
        return true;
      }
    }
  }

  return false;
}

bool QuadtreeCell::subdivided() { return ((bool)children[QUAD_BOTTOM_LEFT]); }

void QuadtreeCell::sortNeighbors() {
  if (subdivided()) {
    for (int i = 0; i < 4; i++) {
      children[i]->sortNeighbors();
    }

    return;
  }

  for (int i = 0; i < 4; i++) {
    if (neighbors[i].size() > 1) {
      QuadtreeCell *f = neighbors[i].front();
      QuadtreeCell *b = neighbors[i].back();

      bool change = false;

      switch (i) {
        case QUAD_B:
          change = b->getMin()->getX() < f->getMin()->getX();
          break;
        case QUAD_R:
          change = b->getMin()->getY() < f->getMin()->getY();
          break;
        case QUAD_T:
          change = f->getMin()->getX() < b->getMin()->getX();
          break;
        case QUAD_L:
          change = f->getMin()->getY() < b->getMin()->getY();
          break;
      }

      if (change) {
        neighbors[i].clear();

        neighbors[i].push_back(b);
        neighbors[i].push_back(f);
      }
    }
  }
}

bool QuadtreeCell::inBoundary() {
  return (neighbors[QUAD_B].empty() || neighbors[QUAD_R].empty() ||
          neighbors[QUAD_T].empty() || neighbors[QUAD_L].empty());
}

void QuadtreeCell::makeOuterVertices() {
  if (subdivided()) {
    for (int i = 0; i < 4; i++) {
      children[i]->makeOuterVertices();
    }

    return;
  }

  if (inBoundary()) {
    return;
  }

  if (!v[0]) {
    v[0] = new Vertex(min->getX(), min->getY(), tree->vertexId());

    if ((neighbors[QUAD_L].size() == 2) ||
        (neighbors[QUAD_L].back()->level == level) ||
        (neighbors[QUAD_L].back()->neighbors[QUAD_R].front() == this)) {
      neighbors[QUAD_L].back()->v[2] = v[0];
    } else {
      neighbors[QUAD_L].back()->v[3] = v[0];
    }

    if ((neighbors[QUAD_B].size() == 2) ||
        (neighbors[QUAD_B].front()->level == level) ||
        (neighbors[QUAD_B].front()->neighbors[QUAD_T].back() == this)) {
      neighbors[QUAD_B].front()->v[6] = v[0];
    } else {
      neighbors[QUAD_B].front()->v[5] = v[0];
    }

    if (neighbors[QUAD_B].front()->neighbors[QUAD_L].front() ==
        neighbors[QUAD_L].back()->neighbors[QUAD_B].back()) {
      neighbors[QUAD_B].front()->neighbors[QUAD_L].front()->v[4] = v[0];
    }

    tree->add(v[0]);
  }

  if ((!v[1]) && (neighbors[QUAD_B].size() > 1)) {
    v[1] = new Vertex(0.5 * (min->getX() + max->getX()), min->getY(),
                      tree->vertexId());

    neighbors[QUAD_B].front()->v[4] = v[1];
    neighbors[QUAD_B].back()->v[6] = v[1];

    tree->add(v[1]);
  }

  if (!v[2]) {
    v[2] = new Vertex(max->getX(), min->getY(), tree->vertexId());

    if ((neighbors[QUAD_B].size() == 2) ||
        (neighbors[QUAD_B].back()->level == level) ||
        (neighbors[QUAD_B].back()->neighbors[QUAD_T].front() == this)) {
      neighbors[QUAD_B].back()->v[4] = v[2];
    } else {
      neighbors[QUAD_B].back()->v[5] = v[2];
    }

    if ((neighbors[QUAD_R].size() == 2) ||
        (neighbors[QUAD_R].front()->level == level) ||
        (neighbors[QUAD_R].front()->neighbors[QUAD_L].back() == this)) {
      neighbors[QUAD_R].front()->v[0] = v[2];
    } else {
      neighbors[QUAD_R].front()->v[7] = v[2];
    }

    if (neighbors[QUAD_R].front()->neighbors[QUAD_B].front() ==
        neighbors[QUAD_B].back()->neighbors[QUAD_R].back()) {
      neighbors[QUAD_R].front()->neighbors[QUAD_B].front()->v[6] = v[2];
    }

    tree->add(v[2]);
  }

  if ((!v[3]) && (neighbors[QUAD_R].size() > 1)) {
    v[3] = new Vertex(max->getX(), 0.5 * (min->getY() + max->getY()),
                      tree->vertexId());

    neighbors[QUAD_R].front()->v[6] = v[3];
    neighbors[QUAD_R].back()->v[0] = v[3];

    tree->add(v[3]);
  }

  if (!v[4]) {
    v[4] = new Vertex(max->getX(), max->getY(), tree->vertexId());

    if ((neighbors[QUAD_R].size() == 2) ||
        (neighbors[QUAD_R].back()->level == level) ||
        (neighbors[QUAD_R].back()->neighbors[QUAD_L].front() == this)) {
      neighbors[QUAD_R].back()->v[6] = v[4];
    } else {
      neighbors[QUAD_R].back()->v[7] = v[4];
    }

    if ((neighbors[QUAD_T].size() == 2) ||
        (neighbors[QUAD_T].front()->level == level) ||
        (neighbors[QUAD_T].front()->neighbors[QUAD_B].back() == this)) {
      neighbors[QUAD_T].front()->v[2] = v[4];
    } else {
      neighbors[QUAD_T].front()->v[1] = v[4];
    }

    if (neighbors[QUAD_T].front()->neighbors[QUAD_R].front() ==
        neighbors[QUAD_R].back()->neighbors[QUAD_T].back()) {
      neighbors[QUAD_T].front()->neighbors[QUAD_R].front()->v[0] = v[4];
    }

    tree->add(v[4]);
  }

  if ((!v[5]) && (neighbors[QUAD_T].size() > 1)) {
    v[5] = new Vertex(0.5 * (min->getX() + max->getX()), max->getY(),
                      tree->vertexId());

    neighbors[QUAD_T].front()->v[0] = v[5];
    neighbors[QUAD_T].back()->v[2] = v[5];

    tree->add(v[5]);
  }

  if (!v[6]) {
    v[6] = new Vertex(min->getX(), max->getY(), tree->vertexId());

    if ((neighbors[QUAD_T].size() == 2) ||
        (neighbors[QUAD_T].back()->level == level) ||
        (neighbors[QUAD_T].back()->neighbors[QUAD_B].front() == this)) {
      neighbors[QUAD_T].back()->v[0] = v[6];
    } else {
      neighbors[QUAD_T].back()->v[1] = v[6];
    }

    if ((neighbors[QUAD_L].size() == 2) ||
        (neighbors[QUAD_L].front()->level == level) ||
        (neighbors[QUAD_L].front()->neighbors[QUAD_R].back() == this)) {
      neighbors[QUAD_L].front()->v[4] = v[6];
    } else {
      neighbors[QUAD_L].front()->v[3] = v[6];
    }

    if (neighbors[QUAD_L].front()->neighbors[QUAD_T].front() ==
        neighbors[QUAD_T].back()->neighbors[QUAD_L].back()) {
      neighbors[QUAD_L].front()->neighbors[QUAD_T].front()->v[2] = v[6];
    }

    tree->add(v[6]);
  }

  if ((!v[7]) && (neighbors[QUAD_L].size() > 1)) {
    v[7] = new Vertex(min->getX(), 0.5 * (min->getY() + max->getY()),
                      tree->vertexId());

    neighbors[QUAD_L].front()->v[2] = v[7];
    neighbors[QUAD_L].back()->v[4] = v[7];

    tree->add(v[7]);
  }
}

void QuadtreeCell::makeOuterEdges() {
  if (subdivided()) {
    for (int i = 0; i < 4; i++) {
      children[i]->makeOuterEdges();
    }

    return;
  }

  if (inBoundary()) {
    return;
  }

  ////cout << "criando arestas" << endl;

  if (!e[0]) {
    e[0] = makeE(0);

    if (neighbors[QUAD_B].front()->inBoundary()) {
      tree->addFront(e[0]);
    } else {
      if ((neighbors[QUAD_B].size() == 1) &&
          (neighbors[QUAD_B].front()->level < level) &&
          (neighbors[QUAD_B].front()->neighbors[QUAD_T].back() == this)) {
        neighbors[QUAD_B].front()->e[5] = e[0];
      } else {
        neighbors[QUAD_B].front()->e[4] = e[0];
      }
    }

    tree->add(e[0]);

    // if (this->level > neighbors[QUAD_B].front()->level)
    if (this->level < neighbors[QUAD_B].front()->level) {
      e[0]->setCell(this);
    } else {
      e[0]->setCell(neighbors[QUAD_B].front());
    }
  }

  ////cout << "aresta 0" << endl;

  if ((!e[1]) && (neighbors[QUAD_B].size() == 2)) {
    e[1] = makeE(1);

    if (neighbors[QUAD_B].back()->inBoundary()) {
      tree->addFront(e[1]);
    } else {
      neighbors[QUAD_B].back()->e[4] = e[1];
    }

    tree->add(e[1]);

    // e[1]->setCell(this);
    e[1]->setCell(neighbors[QUAD_B].back());
  }

  ////cout << "aresta 1" << endl;

  if (!e[2]) {
    e[2] = makeE(2);

    if (neighbors[QUAD_R].front()->inBoundary()) {
      tree->addFront(e[2]);
    } else {
      if ((neighbors[QUAD_R].size() == 1) &&
          (neighbors[QUAD_R].front()->level < level) &&
          (neighbors[QUAD_R].front()->neighbors[QUAD_L].back() == this)) {
        neighbors[QUAD_R].front()->e[7] = e[2];
      } else {
        neighbors[QUAD_R].front()->e[6] = e[2];
      }
    }

    tree->add(e[2]);

    // if (this->level > neighbors[QUAD_R].front()->level)
    if (this->level < neighbors[QUAD_R].front()->level) {
      e[2]->setCell(this);
    } else {
      e[2]->setCell(neighbors[QUAD_R].front());
    }
  }

  ////cout << "aresta 2" << endl;

  if ((!e[3]) && (neighbors[QUAD_R].size() == 2)) {
    e[3] = makeE(3);

    if (neighbors[QUAD_R].back()->inBoundary()) {
      tree->addFront(e[3]);
    } else {
      neighbors[QUAD_R].back()->e[6] = e[3];
    }

    tree->add(e[3]);

    // e[3]->setCell(this);
    e[3]->setCell(neighbors[QUAD_R].back());
  }

  ////cout << "aresta 3" << endl;

  if (!e[4]) {
    e[4] = makeE(4);

    if (neighbors[QUAD_T].front()->inBoundary()) {
      tree->addFront(e[4]);
    } else {
      if ((neighbors[QUAD_T].size() == 1) &&
          (neighbors[QUAD_T].front()->level < level) &&
          (neighbors[QUAD_T].front()->neighbors[QUAD_B].back() == this)) {
        neighbors[QUAD_T].front()->e[1] = e[4];
      } else {
        neighbors[QUAD_T].front()->e[0] = e[4];
      }
    }

    tree->add(e[4]);

    // if (this->level > neighbors[QUAD_T].front()->level)
    if (this->level < neighbors[QUAD_T].front()->level) {
      e[4]->setCell(this);
    } else {
      e[4]->setCell(neighbors[QUAD_T].front());
    }
  }

  ////cout << "aresta 4" << endl;

  if ((!e[5]) && (neighbors[QUAD_T].size() == 2)) {
    e[5] = makeE(5);

    if (neighbors[QUAD_T].back()->inBoundary()) {
      tree->addFront(e[5]);
    } else {
      neighbors[QUAD_T].back()->e[0] = e[5];
    }

    tree->add(e[5]);

    // e[5]->setCell(this);
    e[5]->setCell(neighbors[QUAD_T].back());
  }

  ////cout << "aresta 5" << endl;

  if (!e[6]) {
    e[6] = makeE(6);

    if (neighbors[QUAD_L].front()->inBoundary()) {
      tree->addFront(e[6]);
    } else {
      if ((neighbors[QUAD_L].size() == 1) &&
          (neighbors[QUAD_L].front()->level < level) &&
          (neighbors[QUAD_L].front()->neighbors[QUAD_R].back() == this)) {
        neighbors[QUAD_L].front()->e[3] = e[6];
      } else {
        neighbors[QUAD_L].front()->e[2] = e[6];
      }
    }

    tree->add(e[6]);

    // if (this->level > neighbors[QUAD_L].front()->level)
    if (this->level < neighbors[QUAD_L].front()->level) {
      e[6]->setCell(this);
    } else {
      e[6]->setCell(neighbors[QUAD_L].front());
    }
  }

  ////cout << "aresta 6" << endl;

  if ((!e[7]) && (neighbors[QUAD_L].size() == 2)) {
    e[7] = makeE(7);

    if (neighbors[QUAD_L].back()->inBoundary()) {
      tree->addFront(e[7]);
    } else {
      neighbors[QUAD_L].back()->e[2] = e[7];
    }

    tree->add(e[7]);

    // e[7]->setCell(this);
    e[7]->setCell(neighbors[QUAD_L].back());
  }

  ////cout << "aresta 7" << endl;
}

void QuadtreeCell::makeTemplateBasedMesh() {
  if (subdivided()) {
    for (int i = 0; i < 4; i++) {
      children[i]->makeTemplateBasedMesh();
    }

    return;
  }

  if (inBoundary()) {
    return;
  }

  // gera a malha interna baseada em templates.

  // algumas restricoes:
  // os vertices utilizados por celulas vizinhas tem que ser os mesmos;
  // as arestas utilizadas por celulas vizinhas tem que ser as mesmas;
  // cada vertice, aresta, ou face criada tem que ser inserido na arvore
  // quando uma celula tem um vizinho que faz parte da borda, as arestas
  //	adjacente aos dois vizinhos tem que ser inseridas na fronteira da
  //	arvore
  // as faces criadas tem que estar de acordo com a regra da mao direita
  // as arestas criadas devem ter o lado esquerdo para fora da celula

  // algumas observacoes:
  // se os metodos sortNeighbors(), makeOuterVertices() e makeOuterEdges()
  // tiverem sido chamados, basta criar os vertices, arestas e triangulos
  // internos a celula, pois os vertices e as arestas adjacentes a duas
  // celulas jah foram criadas, da seguinte forma:
  //
  //	v[6] --- e[5] --- v[5] --- e[4] --- v[4]
  //        |                                   |
  //        |                                   |
  //        |                                   |
  //      e[6]                                e[3]
  //        |                                   |
  //        |                                   |
  //        |                                   |
  //      v[7]                                v[3]
  //        |                                   |
  //        |                                   |
  //        |                                   |
  //      e[7]                                e[2]
  //        |                                   |
  //        |                                   |
  //        |                                   |
  //	v[0] --- e[0] --- v[1] --- e[1] --- v[2]
  //
  // onde v[] sao os vertices, e e[] sao as arestas.
  //
  // o caso acima mostrado eh quando todas as celulas adjacentes sao
  // mais subdividas que a mostrada. no caso em que uma celula adjacente
  // tem o mesmo nivel, ou um nivel menor, temos o seguinte padrao:
  //
  //	v[6] --- e[5] --- v[5] --- e[4] --- v[4]
  //        |                                   |
  //        |                                   |
  //        |                                   |
  //      e[6]                                  |
  //        |                                   |
  //        |                                   |
  //        |                                   |
  //      v[7]                                e[2]
  //        |                                   |
  //        |                                   |
  //        |                                   |
  //      e[7]                                  |
  //        |                                   |
  //        |                                   |
  //        |                                   |
  //	v[0] --- e[0] --- v[1] --- e[1] --- v[2]
  //
  // nesse caso, a celula adjacente a direita eh de mesmo nivel ou de
  // nivel menor. entao, v[1] = NULL e e[1] = NULL. o mesmo acontece
  // para os outros casos, das celulas adjacentes abaixo, acima e a
  // direita

  int numNeighs[4], total = 0;

  for (int i = 0; i < 4; i++) {
    numNeighs[i] = static_cast<int>(neighbors[i].size());

    total += numNeighs[i];
  }

  if (total == 4) {
    // nenhuma das celulas adjacentes eh menor que essa

    ////cout << "tipo 1" << endl;

    makeMeshType1();
  } else if (total == 5) {
    // uma das celulas adjacentes eh menor que essa;

    ////cout << "tipo 2" << endl;

    for (int i = 0; i < 4; i++) {
      if (numNeighs[i] == 2) {
        makeMeshType2(i);

        break;
      }
    }
  } else if (total == 6) {
    // duas das celulas adjacentes sao vizinhas

    if (numNeighs[QUAD_B] == numNeighs[QUAD_T]) {
      ////cout << "tipo 3" << endl;

      if (numNeighs[QUAD_B] == 2) {
        makeMeshType3(QUAD_B);
      } else {
        makeMeshType3(QUAD_R);
      }
    } else {
      ////cout << "tipo 4" << endl;

      for (int i = 0; i < 4; i++) {
        if ((numNeighs[i] == numNeighs[(i + 1) % 4]) && (numNeighs[i] == 2)) {
          makeMeshType4(i);

          break;
        }
      }
    }
  } else if (total == 7) {
    ////cout << "tipo 5" << endl;

    for (int i = 0; i < 4; i++) {
      if (numNeighs[i] == 1) {
        makeMeshType5(i);

        break;
      }
    }
  } else {
    ////cout << "tipo 6" << endl;

    makeMeshType6();
  }

  ////cout << "fim criar triangulos template" << endl;
}

string QuadtreeCell::getText() {
  ostringstream stream;

  stream << id << " level " << level << " minLevel";

  string s = stream.str();

  return s;
}

// #if USE_OPENGL
//  void QuadtreeCell::highlight()
//{
//     highlight(true);
// }

// void QuadtreeCell::highlight(bool highlightEdges, bool
// dontHighlightNeighbors)
//{
//    highlighted = true;

//    if (!dontHighlightNeighbors && !highlightedAsNeighbor)
//    {
//        for (QuadtreeCellList::iterator iter = neighbors[QUAD_BOTTOM].begin();
//             iter != neighbors[QUAD_BOTTOM].end(); iter++)
//        {
//            (*iter)->highlightAsNeighbor();
//        }
//        for (QuadtreeCellList::iterator iter = neighbors[QUAD_RIGHT].begin();
//             iter != neighbors[QUAD_RIGHT].end(); iter++)
//        {
//            (*iter)->highlightAsNeighbor();
//        }
//        for (QuadtreeCellList::iterator iter = neighbors[QUAD_TOP].begin();
//             iter != neighbors[QUAD_TOP].end(); iter++)
//        {
//            (*iter)->highlightAsNeighbor();
//        }
//        for (QuadtreeCellList::iterator iter = neighbors[QUAD_LEFT].begin();
//             iter != neighbors[QUAD_LEFT].end(); iter++)
//        {
//            (*iter)->highlightAsNeighbor();
//        }
//    }
//}

// void QuadtreeCell::highlightAsNeighbor()
//{
//    highlightedAsNeighbor = true;

//    highlight(false);
//}

// void QuadtreeCell::unhighlight()
//{
//    unhighlight(true);
//}

// void QuadtreeCell::unhighlight(bool highlightEdges, bool
// dontHighlightNeighbors)
//{
//    highlighted = false;

//    if (!dontHighlightNeighbors && !highlightedAsNeighbor)
//    {
//        for (QuadtreeCellList::iterator iter = neighbors[QUAD_BOTTOM].begin();
//             iter != neighbors[QUAD_BOTTOM].end(); iter++)
//        {
//            (*iter)->unhighlightAsNeighbor();
//        }
//        for (QuadtreeCellList::iterator iter = neighbors[QUAD_RIGHT].begin();
//             iter != neighbors[QUAD_RIGHT].end(); iter++)
//        {
//            (*iter)->unhighlightAsNeighbor();
//        }
//        for (QuadtreeCellList::iterator iter = neighbors[QUAD_TOP].begin();
//             iter != neighbors[QUAD_TOP].end(); iter++)
//        {
//            (*iter)->unhighlightAsNeighbor();
//        }
//        for (QuadtreeCellList::iterator iter = neighbors[QUAD_LEFT].begin();
//             iter != neighbors[QUAD_LEFT].end(); iter++)
//        {
//            (*iter)->unhighlightAsNeighbor();
//        }
//    }
//}

// void QuadtreeCell::unhighlightAsNeighbor()
//{
//    unhighlight(false);

//    highlightedAsNeighbor = false;
//}

// void QuadtreeCell::draw()
//{
//    static Vertex v[4];
//    static Edge e[4];

//    v[0].setPosition(min->getX(), min->getY());
//    v[1].setPosition(max->getX(), min->getY());
//    v[2].setPosition(max->getX(), max->getY());
//    v[3].setPosition(min->getX(), max->getY());

//    e[0].setVertices(&v[0], &v[1]);
//    e[1].setVertices(&v[1], &v[2]);
//    e[2].setVertices(&v[2], &v[3]);
//    e[3].setVertices(&v[3], &v[0]);

//    e[0].setColor(r, g, b);
//    e[1].setColor(r, g, b);
//    e[2].setColor(r, g, b);
//    e[3].setColor(r, g, b);

//    if (highlighted)
//    {
//        if (highlightedAsNeighbor)
//        {
//            glColor3d(1.0 - r, 1.0 - g, 1.0 - b);
//        }
//        else
//        {
//            glColor3d(r*0.5, g*0.5, b*0.5);
//        }

//        glBegin(GL_QUADS);
//        glVertex2d(v[0].getX(), v[0].getY());
//        glVertex2d(v[1].getX(), v[1].getY());
//        glVertex2d(v[2].getX(), v[2].getY());
//        glVertex2d(v[3].getX(), v[3].getY());
//        glEnd();
//    }

//    //debug
//    if (!subdivided())
//    {
//        //endebug
//        e[0].draw();
//        e[1].draw();
//        e[2].draw();
//        e[3].draw();
//    }

//    e[0].setVertices(NULL, NULL);
//    e[1].setVertices(NULL, NULL);
//    e[2].setVertices(NULL, NULL);
//    e[3].setVertices(NULL, NULL);

//    if (subdivided())
//    {
//        children[QUAD_BOTTOM_LEFT]->draw();
//        children[QUAD_BOTTOM_RIGHT]->draw();
//        children[QUAD_TOP_RIGHT]->draw();
//        children[QUAD_TOP_LEFT]->draw();
//    }
//}
// #endif //#if USE_OPENGL
