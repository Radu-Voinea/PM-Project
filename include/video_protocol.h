#pragma once

#include <cstdint>
#include <cstddef>

static const uint16_t UDP_PORT         = 4242;
static const size_t   MAX_PACKET_SIZE  = 1408;   // header + payload
static const size_t   HEADER_SIZE      = 8;
static const size_t   MAX_PAYLOAD_SIZE = MAX_PACKET_SIZE - HEADER_SIZE; // 1400
static const size_t   MAX_FRAME_SIZE   = 100 * 1024; // 100KB max JPEG frame

struct __attribute__((packed)) VideoPacketHeader {
    uint16_t frame_id;      // Rolling frame counter
    uint16_t chunk_index;   // 0-based index within frame
    uint16_t chunk_count;   // Total chunks in this frame
    uint16_t payload_size;  // Bytes of JPEG data in this packet
};
