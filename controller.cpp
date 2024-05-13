#include "controller.h"

#pragma region variables
DigitalOut DirL(PC_7);
DigitalOut DirR(PB_6);
PwmOut PwmL(PB_4);
PwmOut PwmR(PB_5);
#pragma endregion variables

Controller::Controller()
{
    SetState(RoboState::START);
};

Controller::RoboState Controller::GetState()
{
    return robo_state;
};

void Controller::SetState(RoboState state)
{
 robo_state = state;
};

float Controller::GetSpeedL()
{
    return speedL;
};

float Controller::GetSpeedR()
{
    return speedR;
};

void Controller::SetSpeed(float speed)
{
    speedL = speed;
    speedR = speed;
};
void Controller::SetSpeed(float sL,float sR)
{
    speedL = sL;
    speedR = sR;
};

bool Controller::GetEnemyState()
{
    return enemy;
};

void Controller::SetEnemyState()
{
    enemy = !enemy;
};

void Controller::Start()
{
    PwmL.period_us(66);
    PwmR.period_us(66);
    SetEnemyState();
    SetState(RoboState::IDLE);
};

void Controller::Idle()
{
    if(isSafe)
    {
        SetState(RoboState::DETECT);
    }else
    {
        SetState(RoboState::ESCAPE);
    }
};

void Controller::Detect()
{
    if(!GetEnemyState())
        {
            SetSpeed(0.5,-0.5);
        }
    else
        {
            SetSpeed(0);
            SetState(RoboState::ATTACK); 
        }
};

void Controller::Attack()
{
    SetSpeed(MAXSPEED);
};

void Controller::Escape()
{
    SetSpeed(ESCAPESPEED);
};

void Controller::Move(float sL, float sR){
    if(sL<0) DirL = 0;
    else DirL = 1;

    if(sR<0) DirR = 0;
    else DirR = 1;

    PwmL = abs(sL);
    PwmR = abs(sR);
};

// float lerp(float p1, float p2, float d1)
// {
// return (((1-d1) * p1) + (d1 * p2));
// }