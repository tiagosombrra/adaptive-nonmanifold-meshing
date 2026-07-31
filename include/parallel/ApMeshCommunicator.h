#ifndef APMESHCOMMUNICATOR_H
#define APMESHCOMMUNICATOR_H

#include "TMCommunicator.h"

class ApMeshCommunicator : public Parallel::TMCommunicator {
 public:
  explicit ApMeshCommunicator(bool sharedParallelismEnabled);
  ~ApMeshCommunicator();
  bool isMaster() const override;
  using Parallel::TMCommunicator::unpack;
  Parallel::Transferable *unpack(Parallel::Package &p) const override;
};

#endif  // APMESHCOMMUNICATOR_H
