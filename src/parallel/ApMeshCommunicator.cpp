#include "../../include/parallel/ApMeshCommunicator.h"

ApMeshCommunicator::ApMeshCommunicator(bool shared_parallelism_enabled)
    : Parallel::TMCommunicator::TMCommunicator(shared_parallelism_enabled) {}

ApMeshCommunicator::~ApMeshCommunicator() {}

bool ApMeshCommunicator::isMaster() const {
  return (this->rank() == this->root());
}

Parallel::Transferable *ApMeshCommunicator::unpack(
    [[maybe_unused]] Parallel::Package &p) const {
  return NULL;
}
