#ifndef _DATA_NUMERICAL_FUNCTION_H_
#define _DATA_NUMERICAL_FUNCTION_H_

#include "../../data/definitions.h"
#include "multi_variable_function.h"

namespace Data {
namespace Numerical {
class Function : public Data::Numerical::MultiVariableFunction<1> {
 public:
  virtual double f(double x) = 0;

  virtual double f(const double x[1]);
};
}  // namespace Numerical
}  // namespace Data

#endif  // #ifndef _DATA_NUMERICAL_FUNCTION_H_
