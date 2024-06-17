#include "mbed.h"
#include "GP2A.h"
#include <string>
#include <stdlib.h>
#include "Thread.h"
#define MAXSPEED 0.5
#define ESCAPESPEED -0.5
#define PSD_INTERVAL_us 0.1 // @@ dummy value, should be defined !!@@
#define PSD_THRESHOLD 5 // encounter distance(cm) diff, must be defined with experiment - threshold / inverval = speed
#define IR_THRESHOLD 30000 // 30000 넘으면 대충 검정임, 실험 필요!!
#define CIRCLE_DISTANCE 70 //cm
#define WALL_DISTANCE 70 //cm
<<<<<<< Updated upstream
#pragma region external
extern DigitalOut DirL;
extern DigitalOut DirR;
extern PwmOut PwmL;
extern PwmOut PwmR;
extern GP2A psdf; //그냥 거리감지
extern GP2A psdb;
extern class Controller;
extern class psd_side;
=======
#pragma region externs
>>>>>>> Stashed changes
extern DigitalIn irfl;
extern DigitalIn irfr;
extern DigitalIn irc;
extern DigitalIn irbl;
extern DigitalIn irbr;
extern GP2A psdf; //그냥 거리감지
extern GP2A psdb;
#pragma endregion externs

class Controller
{
     public:   
        enum class RoboState
    {
        //초기
        START,
        //대기
        IDLE,
        //감지
        DETECT,
        //공격
        ATTACK,
        //탈출
        ESCAPE
    };
    //객체 생성시 실행되는 생성자 함수
    Controller();
//-------------------Get & Set methods----------------------//
    //현재 로봇의 상태 반환
    RoboState GetState();

    //로봇의 상태를 변환
    void SetState(RoboState state);

    //적 감지 여부 반환
    bool GetEnemyState();

    //적 감지 여부 변환
    void SetEnemyState(bool enemyState);

    //좌측 바퀴 속도 반환
    float GetSpeedL();
    
    //우측 바퀴 속도 반환
    float GetSpeedR();
    
    // 양쪽 바퀴 같은 속도 설정
    void SetSpeed(float speed);

    //양쪽 바퀴 다른 속도 설정
    void SetSpeed(float sL, float sR);
    
    //적과의 수평거리 반환 함수
    int GetHD();

    //적과의 수평거리 변환환
    void SetHD(int HD);
//--------------------State Machine methods----------------------//
    //초기상태 시 실행 함수
    void Start();

    //대기상태 시 실행 함수
    void Idle();

     //감지상태 시 실행 함수
    void Detect();

     //공격상태 시 실행 함수
    void Attack();

     //탈출상태 시 실행 함수
    void Escape();

    //주행 함수
    void Move(float sL, float sR);

    void EnemyDetect();
<<<<<<< Updated upstream

    class psd_side {
    GP2A GP2A_;
    uint16_t prev_distance;
    uint16_t now_distance;
    float filtered_distance;
    float alpha;

    public:
        bool detection;
        psd_side(PinName, uint16_t, uint16_t, float, float);
        bool refresh();
        float distance();
    };
    class irs {
    uint16_t ir_val[5]; //irfl, irfr, irc, irbl, irbr //미리 선언되어야 함.
    uint32_t ir_total; //
    public:
        enum class ColorOrient {
        FRONT, TAN_LEFT, TAN_RIGHT, BACK, FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT, SAFE    
        };
        enum class Position {
        ClosetoLeftWall, CriticalLeftWall, ClosetoRightWall, CriticalRightWall,
        WallFront, WallBehind, ClosetoCenter, FartoCenter
        };
        ColorOrient Orient;
        Position CurrentPos;
        Position GetPosition();
        void refresh();
        void IR_Escape(ColorOrient orient);
        void ColorOrient();
        void enumfucker(int);
        void SetPosition();
    };
    class EnemyFind {
    irs::Position pos;
    static void SetSpeed(float, float);
    static void SetState(RoboState);
    public:
        EnemyFind(irs::Position pos);
        void LeftWallTrack();
        void RightWallTrack();
        void CenterSpin();
        void FrontWall();
        void BehindWall();
    };
=======
>>>>>>> Stashed changes
//--------------------Private variables--------------------------//
    private:
    //로봇 상태
    RoboState robo_state;

    //적 감지 여부
    bool enemy = false;

    //적과 벌어진 거리
    int enemy_horizontal_distance;

    //위험 지역 여부
    bool isSafe = true;

    //좌측 바퀴 속력
    float speedL;

    //우측 바퀴 속력
    float speedR;
};

class psd_side {
    GP2A GP2A_;
    uint16_t prev_distance;
    uint16_t now_distance;
    float filtered_distance;
    float alpha;

    public:
        bool detection;
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
        Position GetPosition();
        void refresh();
        void IR_Escape(ColorOrient orient);
        void ColorOrient();
        void enumfucker(int);
        void SetPosition();
};
class EnemyFind:Controller {
    irs::Position pos;
    Controller controller;
    public:
        EnemyFind(irs::Position pos);
        void LeftWallTrack();
        void RightWallTrack();
        void CenterSpin();
        void FrontWall();
        void BehindWall();
};