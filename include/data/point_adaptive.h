#ifndef Ponto_h
#define Ponto_h

#include <cmath>

class PointAdaptive {
 public:
  PointAdaptive();
  PointAdaptive(double x, double y, double z);
  PointAdaptive(double x, double y, double z, unsigned long id);
  // IMPORTANT: virtual destructor for polymorphic delete
  virtual ~PointAdaptive() = default;

  bool operator==(const PointAdaptive& point) const;
  bool operator==(const PointAdaptive* point) const;
  double CalculateDistance(const PointAdaptive& point) const;
  void PrintPoint() const;

  unsigned long GetId() const;
  void SetId(unsigned long id);

  double GetX() const;
  void SetX(double x);

  double GetY() const;
  void SetY(double y);

  double GetZ() const;
  void SetZ(double z);

 protected:
  unsigned long id_;
  double x_;
  double y_;
  double z_;
};
#endif
