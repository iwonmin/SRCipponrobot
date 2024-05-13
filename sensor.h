#include "mbed.h"
#include "rtos.h"
#include "GP2A.h"
#include "PinNames.h"
#define PSD_INTERVAL_us 0.1 // @@ dummy value, should be defined !!@@
#define PSD_THRESHOLD 5 // encounter distance(cm) diff, must be defined with experiment - threshold / inverval = speed
#define IR_THRESHOLD 30000 // 30000 넘으면 대충 검정임, 실험 필요!!
class psd_side {
    GP2A GP2A_;
    //Timeout tmo_;
    uint16_t prev_distance;
    uint16_t now_distance;
    // comparing 8 data to find effective change in distace
    bool detection;
    public:
        psd_side(PinName, uint16_t, uint16_t, float, float);
        bool refresh();
        uint16_t distance();

};

extern bool isSafe;
