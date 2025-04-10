//ring_buffer.h
#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include "stdio.h"
#include "stdint.h"
#define RING_BUFFER_SIZE 256 // change for bigger ones later  



typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE]; // Defines buffer size as 256 bytes (power of 2 for efficient modulo)
    volatile uint16_t head;
    volatile uint16_t tail;

} ring_buffer;

//fuction declarations

void ring_buffer_init(ring_buffer *p_ring_buf);
uint8_t ring_buffer_put(ring_buffer *p_ring_buf, uint8_t data);
uint8_t ring_buffer_get(ring_buffer *p_ring_buf, uint8_t *data);
uint16_t ring_buffer_availible(ring_buffer *p_ring_buf);
void send_to_ring_buffer(void* r_buff, const char* format, uint32_t value);


#endif //RING_BUFFER_H 

