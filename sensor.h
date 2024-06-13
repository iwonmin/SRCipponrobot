#include "mbed.h"
#include "rtos.h"
#include "GP2A.h"
#include "PinNames.h"
#define PSD_INTERVAL_us 0.1 // @@ dummy value, should be defined !!@@
#define PSD_THRESHOLD 5 // encounter distance(cm) diff, must be defined with experiment - threshold / inverval = speed
#define IR_THRESHOLD 30000 // 30000 넘으면 대충 검정임, 실험 필요!!
#define CIRCLE_DISTANCE 70 //cm
#define WALL_DISTANCE 70 //cm
#pragma region externs
extern DigitalIn irfl;
extern DigitalIn irfr;
extern DigitalIn irc;
extern DigitalIn irbl;
extern DigitalIn irbr;
extern GP2A psdf; //그냥 거리감지
extern GP2A psdb;
extern enum class CurrentPos;
#pragma endregion externs

class psd_side {
    GP2A GP2A_;
    uint16_t prev_distance;
    uint16_t now_distance;
    float filtered_distance;
    float alpha;
    bool detection;
    public:
        psd_side(PinName, uint16_t, uint16_t, float, float);
        bool refresh();
        float distance();
};

class irs {
    uint16_t ir_val[5]; //irfl, irfr, irc, irbl, irbr //미리 선언되어야 함.
    uint32_t ir_total;
    public:
        irs();
        enum class ColorOrient {
        //정면에 색영역
        FRONT,
        //색영역에 대해 왼쪽접선방향
        TAN_LEFT,
        //색영역에 대해 오른쪽접선방향
        TAN_RIGHT,
        //후면에 색영역
        BACK,
        //4개 인식됐을 떄
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT,  
        SAFE //저장되서 탈출해도 유지되는거 막는 값       
        };
        enum class Position {
        ClosetoLeftWall,
        CriticalLeftWall,
        ClosetoRightWall,
        CriticalRightWall,
        WallFront,
        WallBehind,
        ClosetoCenter,
        FartoCenter
        };
        ColorOrient Orient;
        Position CurrentPos;
        void refresh();
        void ColorOrient();
        void enumfucker(int);
        void SetPosition();
};
class EnemyFind {
    public:
        EnemyFind(irs::Position pos);
        void LeftWallTrack();
        void RightWallTrack();
        void CenterSpin();
        void FrontWall();
        void BehindWall();
};
/*
while(true):
    read_sensor_distances()
    avg_distance = calculate_average_distance()
    
    error = target_distance - avg_distance
    
    if abs(error) < threshold:
        set_motor_speeds(motor_speed, motor_speed)
    elif error > 0:
        set_motor_speeds(motor_speed + error, motor_speed - error)
    else:
        set_motor_speeds(motor_speed - error, motor_speed + error)
    
    wait(delay_time)
*/