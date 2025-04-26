//ring_buffer.c
#include "ring_buffer.h"

void ring_buffer_init(ring_buffer *p_ring_buf){
    p_ring_buf->head = 0;
    p_ring_buf->tail = 0;
}

//add data to the ring buffer 
uint8_t ring_buffer_put(ring_buffer *p_ring_buf, uint8_t data){
    uint16_t next = (p_ring_buf->head + 1 ) % RING_BUFFER_SIZE; // increments by one and becomes 0 when reached buffer size (should wraparound at max int val) 
    if (next == p_ring_buf->tail){
        return 0; // when its full 
    }
    p_ring_buf->buffer[p_ring_buf->head]= data;
    p_ring_buf->head = next;
    return 1; // succcess
}

//get data from the ring buffer
uint8_t ring_buffer_get(ring_buffer *p_ring_buf, uint8_t *data){
    if (p_ring_buf->head == p_ring_buf->tail) {
        return 0; // Buffer empty
    }
    *data = p_ring_buf->buffer[p_ring_buf->tail];
    p_ring_buf->tail = (p_ring_buf->tail+1)%RING_BUFFER_SIZE;
    return 1; // read
}

//check if available data in the ring buffer
uint16_t ring_buffer_availible(ring_buffer *p_ring_buf){
    return (RING_BUFFER_SIZE + p_ring_buf->head - p_ring_buf->tail)%RING_BUFFER_SIZE;
}


// dont remember why I wrote this ...
void send_to_ring_buffer(void* r_buff, const char* format, uint32_t value) {
    char buffer[64]; // Adjust size based on your needs
    int length;

    // Format the string with the given value
    length = snprintf(buffer, sizeof(buffer), format, value);

    // Check if snprintf succeeded
    if (length < 0 || length >= sizeof(buffer)) {
        // Handle error (e.g., log or assert)
        return; // add error handling
    }

    // Send each character of the formatted string to the ring buffer
    for (int i = 0; i < length; i++) {
        ring_buffer_put(r_buff, (uint8_t)buffer[i]);
    }
}

