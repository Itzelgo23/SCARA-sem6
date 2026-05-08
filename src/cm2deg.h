#include "definitions.h"

float cm2deg(float cm, float pitch) {
    float mm = cm * 10.0;
    return (mm / pitch) * 360.0;
}

float deg2cm(float deg, float pitch) {
    float mm = (deg / 360.0) * pitch;
    return mm / 10.0;
}