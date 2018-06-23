#ifndef _FIFO_H_
#define _FIFO_H_

/* includes */
#include <stdint.h>

/* defines */
#define FIFO_GOOD       0x00
#define FIFO_OVERFLOW   0x01
#define FIFO_UNDERFLOW  0x02

/* typedefs */
typedef struct {
    volatile uint32_t size;
    volatile uint8_t* data;
    volatile uint8_t  status;
    volatile uint32_t putIndex;
    volatile uint32_t getIndex;
    volatile uint32_t used;
} FIFO;

/* functions */
void     fifo_init(FIFO* f, uint32_t size, uint8_t* data);
uint32_t fifo_isFull(FIFO* f);
uint32_t fifo_isEmpty(FIFO* f);
uint8_t  fifo_get(FIFO* f);
void     fifo_put(FIFO* f, uint8_t c);
uint8_t  fifo_peek(FIFO* f);
uint32_t fifo_available(FIFO* f);
void     fifo_clear(FIFO* f);
uint8_t  fifo_status(FIFO* f);

#endif // _FIFO_H_
