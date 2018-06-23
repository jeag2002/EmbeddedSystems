#include "fifo.h"

void fifo_init(FIFO* f, uint32_t size, uint8_t* data) {
    f->size     = size;
    f->data     = data;
    f->status   = FIFO_GOOD;
    f->putIndex = 0;
    f->getIndex = 0;
    f->used     = 0;
}

uint32_t fifo_isFull(FIFO* f) {
    return (f->used >= f->size);
}

uint32_t fifo_isEmpty(FIFO* f) {
    return (f->used == 0);
}

uint8_t fifo_get(FIFO* f) {
    uint8_t c;
    if (f->used > 0) {
        c = f->data[f->getIndex];
        f->getIndex = (f->getIndex+1) % f->size;
        f->used--;
        return c;
    }
    else {
        f->status = FIFO_UNDERFLOW;
        return 0;
    }
}

void fifo_put(FIFO* f, uint8_t c) {
    if (f->used >= f->size)
        f->status = FIFO_OVERFLOW;
    else {
        f->data[f->putIndex] = c;
        f->putIndex = (f->putIndex+1) % f->size;
        f->used++;
    }
}

uint8_t fifo_peek(FIFO* f) {
    return f->data[f->getIndex];
}

uint32_t fifo_available(FIFO* f) {
    return f->used;
}

void fifo_clear(FIFO* f) {
    f->status = FIFO_GOOD;
    f->putIndex = 0;
    f->getIndex = 0;
    f->used = 0;
}

uint8_t fifo_status(FIFO* f) {
    return f->status;
}
