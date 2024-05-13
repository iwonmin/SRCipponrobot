#include "header.h"
#pragma region sensors
#pragma endregion sensors

bool psd_side::refresh() {
        psd_side::now_distance = psd_side::GP2A_.getDistance();
        uint16_t difference = fabs(psd_side::now_distance - psd_side::prev_distance);
        if(difference > PSD_THRESHOLD) {
            psd_side::detection = 1;
        } else {
            psd_side::detection = 0;
            }
        psd_side::prev_distance = psd_side::now_distance;
        return psd_side::detection;
    }
psd_side::psd_side(PinName pin_, uint16_t mincm, uint16_t maxcm, float slope, float base)
:GP2A_(pin_, mincm, maxcm, slope, base) {
    psd_side::prev_distance = 0;
    psd_side::now_distance = 0;
    psd_side::detection = 0;
}
uint16_t psd_side::distance() {
    return GP2A_.getDistance();
}


/*
void irsafety() {
//if sum of ir value > 150000 (black=30000)
    uint16_t sum = irfl.read_u16(); // + irfm.read_u16(); ...
//expected direction of retreation
}
*/