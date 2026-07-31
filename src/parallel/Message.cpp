#include "../../include/parallel/Message.h"

using namespace Parallel;

Parallel::Message::Message() : Transferable(0) { this->setMessage(0); }

Parallel::Message::Message(Int message_value) : Transferable(0) {
  this->setMessage(message_value);
}

Parallel::Message::~Message() {}

void Parallel::Message::setMessage(Int message_value) {
  this->message = message_value;
}

Int Parallel::Message::getMessage() const { return this->message; }
