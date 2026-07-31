#include "../../include/parallel/ThreadManager.h"

Parallel::ThreadManager::ThreadManager(bool shared_parallelism_enabled) {
  this->setSharedParallelismEnabled(shared_parallelism_enabled);
}

Parallel::ThreadManager::~ThreadManager() {}

void Parallel::ThreadManager::setSharedParallelismEnabled(
    bool shared_parallelism_enabled) {
  this->sharedParallelismEnabled = shared_parallelism_enabled;
}

bool Parallel::ThreadManager::isSharedParallelismEnabled() const {
  return this->sharedParallelismEnabled;
}
