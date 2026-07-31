#include "../../../include/crab_mesh/aft/vertex.h"

Vertex::Vertex(double initial_x, double initial_y, long int vertex_id)
    : Shape(vertex_id) {
  setPosition(initial_x, initial_y);

  // #if USE_OPENGL
  //     setSize(1.0);
  // #endif //#if USE_OPENGL

  // #if DEBUG_MODE
  //     h = 0.0;
  // #endif //#if DEBUG_MODE
}

Vertex::~Vertex() { adjacentEdges.clear(); }

void Vertex::setX(double new_x) { this->x = new_x; }

void Vertex::setY(double new_y) { this->y = new_y; }

void Vertex::setPosition(double new_x, double new_y) {
  setX(new_x);
  setY(new_y);
}

double Vertex::getX() { return x; }

double Vertex::getY() { return y; }

void Vertex::getPosition(double *out_x, double *out_y) {
  *out_x = this->x;
  *out_y = this->y;
}

// #if USE_OPENGL
//  void Vertex::setSize(double size)
//{
//     this->size = size;
// }
// #endif //#if USE_OPENGL

void Vertex::addAdjacentEdge(Edge *e) { adjacentEdges.insert(e); }

void Vertex::removeAdjacentEdge(Edge *e) { adjacentEdges.erase(e); }

EdgeSet Vertex::getAdjacentEdges() { return adjacentEdges; }

void Vertex::sum(Vertex *v) { setPosition(x + v->x, y + v->y); }

void Vertex::scalarMultiplication(double d) { setPosition(x * d, y * d); }

double Vertex::dot(Vertex *v) { return (x * v->x + y * v->y); }

double Vertex::cross(Vertex *v) { return (x * v->y - y * v->x); }

double Vertex::norm() { return sqrt(x * x + y * y); }

double Vertex::distance(Vertex *v) {
  return sqrt(((v->x) - x) * ((v->x) - x) + ((v->y) - y) * ((v->y) - y));
}

double Vertex::angle(Vertex *v) { return acos(dot(v) / (norm() * v->norm())); }

double Vertex::pseudoAngle() { return x / norm(); }

double Vertex::pseudoAngle(Vertex *v) { return dot(v) / (norm() * v->norm()); }

double Vertex::squarePseudoAngle() {
  if (x >= 0.0) {
    if (y >= 0.0) {
      if (x >= y) {
        return y / x;
      } else {
        return 2.0 - x / y;
      }
    } else {
      if (x >= fabs(y)) {
        return 8.0 + y / x;
      } else {
        return 6.0 - x / y;
      }
    }
  } else {
    if (y >= 0.0) {
      if (fabs(x) >= y) {
        return 4.0 + y / x;
      } else {
        return 2.0 - x / y;
      }
    } else {
      if (fabs(x) >= fabs(y)) {
        return 4.0 + y / x;
      } else {
        return 6.0 - x / y;
      }
    }
  }
}

double Vertex::orientedAngle() {
  /*static*/ Vertex v;
  /*static*/ double ang;

  v.setPosition(1.0, 0.0);

  ang = angle(&v);

  return (y >= 0.0) ? ang : 2.0 * M_PI - ang;
}

double Vertex::orientedAngle(Vertex *v) {
  return v->orientedAngle() - orientedAngle();
}

bool Vertex::left(Vertex *v) { return (cross(v) > 0.0); }

bool Vertex::right(Vertex *v) { return (cross(v) < 0.0); }

double Vertex::surface(Vertex *v1, Vertex *v2) {
  return fabs(orientedSurface(v1, v2));
}

double Vertex::orientedSurface(Vertex *v1, Vertex *v2) {
  //    if (!v1 || !v2) {
  // #pragma omp critical
  //        cout<<"Orientação superfície: "<<(    x * v1->y - v1->x * y +
  //                                              v1->x * v2->y - v2->x * v1->y
  //                                              + v2->x * y - x *
  //                                              v2->y)/2.0<<omp_get_thread_num()<<endl;

  //        return 1.0;
  //    }else{
  // #pragma omp critical
  //        cout<<"Orientação superfície1: "<<(    x * v1->y - v1->x * y +
  //                                              v1->x * v2->y - v2->x * v1->y
  //                                              + v2->x * y - x *
  //                                              v2->y)/2.0<<omp_get_thread_num()<<endl;
  return (x * v1->y - v1->x * y + v1->x * v2->y - v2->x * v1->y + v2->x * y -
          x * v2->y) /
         2.0;
  //   }
}

bool Vertex::counterClockWise(Vertex *v1, Vertex *v2) {
  return orientedSurface(v1, v2) > 0.0;
}

bool Vertex::clockWise(Vertex *v1, Vertex *v2) {
  return orientedSurface(v1, v2) < 0.0;
}

bool Vertex::equals(double other_x, double other_y) {
  return ((this->x > other_x - TOLERANCE_AFT) &&
          (this->x < other_x + TOLERANCE_AFT) &&
          (this->y > other_y - TOLERANCE_AFT) &&
          (this->y < other_y + TOLERANCE_AFT));
}

bool Vertex::matches(Vertex *v) { return ((x == v->x) && (y == v->y)); }

// #if USE_OPENGL
//  void Vertex::highlight()
//{
//     setSize(5.0);

//    //debug
//    for (EdgeSet::iterator iter = adjacentEdges.begin();
//         iter != adjacentEdges.end(); iter++)
//    {
//        (*iter)->highlight(false);
//    }
//    //endebug
//}

// void Vertex::unhighlight()
//{
//    setSize(1.0);

//    //debug
//    for (EdgeSet::iterator iter = adjacentEdges.begin();
//         iter != adjacentEdges.end(); iter++)
//    {
//        (*iter)->unhighlight(false);
//    }
//    //endebug
//}
// #endif //#if USE_OPENGL

string Vertex::getText() {
  ostringstream stream;

  stream << id << " " << x << " " << y;

  string s = stream.str();

  return s;
}

// #if USE_OPENGL
//  void Vertex::draw()
//{
//     glPointSize(size);

//    glColor3d(r, g, b);
//    glBegin(GL_POINTS);
//    glVertex2d(x, y);
//    glEnd();

//    glPointSize(1.0);
//}

// #if DEBUG_MODE
//  void Vertex::drawCircle()
//{
//     int numLines = 40;

//    double angle = 2.0*M_PI/(double)numLines;

//    glColor3d(1.0, 0.0, 1.0);

//    glBegin(GL_LINE_LOOP);
//    for(int i = 1; i <= numLines; i++)
//    {
//        glVertex2d(x + cos(i*angle)*h, y + sin(i*angle)*h);
//    }
//    glEnd();
//}
// #endif //#if DEBUG_MODE

// #endif //#if USE_OPENGL
