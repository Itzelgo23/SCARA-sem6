#include <definitions.h>

float cm2deg(float cm, float pitch) {
    float mm = cm * 10.0;
    return (mm / pitch) * 360.0;
}