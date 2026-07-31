#ifndef _DATA_NUMERICAL_EQUATION_ROOT_H_
#define _DATA_NUMERICAL_EQUATION_ROOT_H_

#include "../../data/definitions.h"
#include "equation_root_function.h"

namespace Data {
namespace Numerical {
class EquationRoot {
 public:
  virtual double execute(EquationRootFunction *function) = 0;
};
}  // namespace Numerical
}  // namespace Data

#endif  // #ifndef _DATA_NUMERICAL_EQUATION_ROOT_H_
