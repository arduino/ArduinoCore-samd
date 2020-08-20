#include <arm_math.h>

arm_rfft_fast_instance_f32 plan;

void setup() {
  arm_rfft_fast_init_f32(&plan, 256);
}

void loop() {
    float in[256] = { 0 }, out[256] = { 0 };
    arm_rfft_fast_f32(&plan, in, out, 0);
}
