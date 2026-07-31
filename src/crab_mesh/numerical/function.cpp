#include "../../../include/crab_mesh/numerical/function.h"

using namespace Data;
using namespace Data::Numerical;

double Data::Numerical::Function::f(const double x[1]) { return this->f(x[0]); }
