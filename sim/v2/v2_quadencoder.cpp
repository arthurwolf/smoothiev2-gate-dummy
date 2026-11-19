#include "QuadEncoder.h"

namespace {
bool enabled = false;
uint16_t count = 0;
}

bool setup_quadrature_encoder(bool) { enabled = true; count = 0; return true; }
uint32_t read_quadrature_encoder() { return count; }
uint32_t get_quadrature_encoder_max_count() { return 65535; }
uint32_t get_quadrature_encoder_div() { return 2; }
bool sim_v2_encoder_advance(int32_t counts)
{
    if (!enabled) return false;
    count = static_cast<uint16_t>(static_cast<uint32_t>(count) + static_cast<uint32_t>(counts));
    return true;
}
