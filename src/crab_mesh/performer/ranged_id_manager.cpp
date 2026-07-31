#include "../../../include/crab_mesh/performer/ranged_id_manager.h"

using namespace Performer;

Performer::RangedIdManager::RangedIdManager(
    ULInt id, ULInt min, ULInt initial_offset, ULInt initial_range,
    UInt manager_size)
    : Performer::IdManager(id, manager_size) {
  this->offset = this->range = 0;

  this->mins = new ULInt[this->size];

  for (UInt i = 0; i < this->size; i++) {
    this->setMin(i, min);
  }

  this->setRange(initial_range);
  this->setOffset(initial_offset);
}

Performer::RangedIdManager::~RangedIdManager() { delete[] this->mins; }

void Performer::RangedIdManager::setId(UInt i, ULInt id) {
  Performer::IdManager::setId(i, id);

  this->adjust(i);
}

void Performer::RangedIdManager::setMin(UInt i, ULInt min) {
  this->mins[i] = min;

  this->adjust(i);
}

void Performer::RangedIdManager::setMin(ULInt min) { this->setMin(0, min); }

ULInt Performer::RangedIdManager::getMin(UInt i) const { return this->mins[i]; }

ULInt Performer::RangedIdManager::getMin() const { return this->getMin(0); }

void Performer::RangedIdManager::setOffset(ULInt new_offset) {
  this->offset = new_offset;

  this->adjustAll();
}

ULInt Performer::RangedIdManager::getOffset() const { return this->offset; }

void Performer::RangedIdManager::setRange(ULInt new_range) {
  this->range = new_range;

  this->adjustAll();
}

ULInt Performer::RangedIdManager::getRange() const { return this->range; }

ULInt Performer::RangedIdManager::next(UInt i) {
  if (this->ids[i] >= this->mins[i] + this->range) {
    this->mins[i] += this->offset;
    this->ids[i] = this->mins[i];
  }
  // #pragma omp critical
  //     {
  //         std::ofstream ofs;
  //         ofs.open ("backtrace", std::ofstream::out | std::ofstream::app);
  //         //std::cout<<"i:"<<i<<" this->ids[i]:"<<this->ids[i]<<"
  //         thread:"<<omp_get_thread_num()<<endl; ofs<<"i:"<<i<<"
  //         this->ids[i]:"<<this->ids[i]<<"
  //         thread:"<<omp_get_thread_num()<<endl;
  //     }
  return this->ids[i]++;
}

ULInt Performer::RangedIdManager::next() { return this->next(0); }

void Performer::RangedIdManager::adjust(UInt i) {
  if (this->ids[i] < this->mins[i]) {
    this->ids[i] = this->mins[i];
  } else if (this->ids[i] >= this->mins[i] + this->range) {
    ULInt numOffsets =
        (this->offset > 0) ? (this->ids[i] - this->mins[i]) / this->offset : 0;

    this->mins[i] += numOffsets * this->offset;

    if (this->ids[i] >= this->mins[i] + this->range) {
      this->mins[i] += this->offset;
      this->ids[i] = this->mins[i];
    }
  }
}

void Performer::RangedIdManager::adjustAll() {
  if (this->offset < this->range) {
    this->offset = this->range;
  }

  for (UInt i = 0; i < this->size; i++) {
    this->adjust(i);
  }
}
