#include "controller.h"
#pragma region variables
InterruptIn btn(BUTTON1);
DigitalOut DirL(PC_7);
DigitalOut DirR(PB_6);
PwmOut PwmL(PB_4);
PwmOut PwmR(PB_5);
GP2A psdlf(PA_0, 7, 80, 24.6, -0.297);
GP2A psdf(PB_0, 7, 80, 24.6, -0.297);
GP2A psdrf(PA_1, 7, 80, 24.6, -0.297);
GP2A psdlc(PA_4, 30, 150, 60, 0);
GP2A psdrc(PC_1, 30, 150, 60, 0);
GP2A psdlb(PC_3, 7, 80, 24.6, -0.297);
GP2A psdb(PC_2, 30, 150, 60, 0);
GP2A psdrb(PC_0, 7, 80, 24.6, -0.297);
BusIn ir(PA_5, PA_7, PA_8, PA_6, PB_1, PC_4); //irbr, irbl, irbc, irfc, irfr, irfl BusIn은 역순으로 합이 계산됨 ㅆㅂ;
// DigitalIn irfl(PC_4);
// DigitalIn irfr(PB_1);
// DigitalIn irfc(PA_6);
// DigitalIn irbc(PA_8);
// DigitalIn irbl(PA_7);
// DigitalIn irbr(PA_5);
Serial ebimu(PB_10,PC_5,115200);
Controller controller;
Thread Thread1;
Mutex mutex;
#pragma endregion variables
#pragma region Serial Variables
Serial pc(USBTX, USBRX, 115200);
RawSerial device(D8, D2, 9600); // Raspberry Pi
int bufferIndex = 0;
char buffer[8] = "";
char a = 0;
bool Incoming = false;
bool AsteriskReceived = false;
#pragma endregion Serial Variables


Controller::Controller() { 
    SetState(RoboState::START);
    btn.fall(&Starter);
}

Controller::RoboState Controller::GetState() { return robo_state; };

void Controller::SetState(RoboState state) { robo_state = state; };

float Controller::GetSpeedL() { return speedL; };

float Controller::GetSpeedR() { return speedR; };

void Controller::SetSpeed(float speed) {
  speedL = speed;
  speedR = speed;
};
void Controller::SetSpeed(float sL, float sR) {
  speedL = sL;
  speedR = sR;
};
//==================================flags startregion=================================//
bool Controller::GetEnemyState() { return enemy; }

void Controller::SetEnemyState(bool enemyState) { enemy = enemyState; }

bool Controller::GetStartFlag() {return StartFlag;}

void Controller::SetStartFlag(bool sf) {StartFlag=sf;}

bool Controller::GetAttackState() { return attack; }

void Controller::SetAttackState(bool attackState) { enemy = attackState; }

bool Controller::GetIrSafetyState() { return irSafe; }

void Controller::SetIrSafetyState(bool IrSafetyState) { irSafe = IrSafetyState; }

bool Controller::GetImuSafetyState() { return imuSafe; }

void Controller::SetImuSafetyState(bool ImuSafetyState) { imuSafe = ImuSafetyState; }

bool Controller::GetWallSafetyState() { return wallSafe; }

void Controller::SetWallSafetyState(bool WallSafetyState) { wallSafe = WallSafetyState; }

bool Controller::GetYellow() {return yellow;}

void Controller::SetYellow(bool y) {yellow = y;}
//==================================Volatile Variables=================================//

int Controller::GetHD() { return enemy_horizontal_distance; }

void Controller::SetHD(int HD) { enemy_horizontal_distance = HD; }

void Controller::CheckStartTime() {
    StartTime = Kernel::get_ms_count();
}

uint64_t Controller::GetStartTime() {
    return StartTime;
}
//==================================State Methods=================================//
void Controller::Start() {
    if(StartFlag) {
    PwmL.period_us(66);
    PwmR.period_us(66);
    ir.mode(PullDown);
    Thread1.start(ImuThread);
    Thread1.set_priority(osPriorityAboveNormal);
    ThisThread::sleep_for(50);
    SetState(RoboState::IDLE);
    }
};

void Controller::Idle() {
  if (imuSafe && irSafe && wallSafe) {
    SetState(RoboState::DETECT);
  } else if(!imuSafe || !irSafe || !wallSafe) {
    SetState(RoboState::ESCAPE);
  }
};

void Controller::Detect() {
    if (imuSafe && irSafe && wallSafe) {
        if(!GetYellow()){
            if(GetCurrentYaw()>=-90){
                SetSpeed(-0.5,0.5);
            }if(GetCurrentYaw()<-90)
            {
                SetSpeed(0);
                lastDirection = GetHD();
                SetState(RoboState::YELLOW);
            }
        }
        else if(GetYellow()){
            if(GetEnemyState() && abs(GetHD())<150) {
            SetSpeed(0);
            SetState(RoboState::ATTACK);
            } else if (!GetEnemyState() && GetHD() > 0) {
            SetSpeed(-0.5, 0.5);
            } else if (!GetEnemyState() && GetHD() < 0) {
            SetSpeed(0.5, -0.5);
            }
        }
    } else {
        SetState(RoboState::IDLE);
    }
};

void Controller::Attack() {//에다가 ir 위험 신호 받으면 Ir_Escape 실행할 수 있게 하기
    if(GetOrient() == ColorOrient::FRONT) SetAttackState(true);
    if (irSafe && imuSafe) {
        // if(psd_val[1] < 15) SetSpeed(1.0);
        // else SetSpeed(0.5);
        SetSpeed(0.5);
        if (!GetEnemyState()) {
        SetState(RoboState::IDLE);
        SetAttackState(false);
        }
    } else {
        SetState(RoboState::IDLE);
    }
};

void Controller::Yellow()
{
    if(lastDirection>=0)
    {
        if(GetCurrentYaw()>=-130)
        {
            SetSpeed(-0.5,0.5);
        }else if(GetCurrentYaw()<=-140)
        {
            SetSpeed(0.5,-0.5);
        }else if(GetCurrentYaw()<-130 && GetCurrentYaw()>-140)
        {
            if(ir[3].read())
            {
                SetSpeed(0.5);
            }else
            {
                SetSpeed(0);
                if(GetEnemyState()&& psd_val[1]<20)
                {
                    SetYellow(true);
                    SetState(RoboState::DETECT);
                }
            }
        }
    }else if(lastDirection<0){
        if(GetCurrentYaw()>=-40)
        {
            SetSpeed(-0.5,0.5);
        }else if(GetCurrentYaw()<=-50)
        {
            SetSpeed(0.5,-0.5);
        }else if(GetCurrentYaw()<-40 && GetCurrentYaw()>-50)
        {
            if(ir[3].read())
            {
                SetSpeed(0.5);
            }else
            {
                SetSpeed(0);
                if(GetEnemyState()&& psd_val[1]<75)
                {
                    SetYellow(true);
                    SetState(RoboState::DETECT);
                }
            }
        }
    }
}
void Controller::Escape() {
    if (!GetIrSafetyState()) {
        IrEscape(); 
    } else if (!GetImuSafetyState()) {
        ImuEscape();
    } else if (!GetWallSafetyState()) {
        PsdWallEscape();
    } 
    SetState(RoboState::IDLE);
};

void Controller::WallTwerk() {
    //각도 맞추기 -> 벽에 가까워지기 -> 각도 맞추기 -> 뒤로붙어서 pitch 변화
    if(abs((int)GetCurrentYaw())%90 < 10) {
        if((psd_val[5]+psd_val[7])/2 < 13) {
            if(pitch < 10) {
                SetSpeed(-0.15,-0.15);
            } else {
                if(Escape_Timer.read_ms() != 0) {Escape_Timer.start();}
                if(Escape_Timer.read_ms() <= 9000) {Escape_Timer.reset(); SetState(RoboState::DETECT);}
                SetSpeed(0,0);
                if(GetEnemyState() && psd_val[1] < 10) {SetState(RoboState::ATTACK);}
            }
        } else {
            SetSpeed(-0.3,-0.3);
        }
    } else {
        if(abs((int)GetCurrentYaw())%90 > 45 ) SetSpeed(-0.45, 0.45);
        else SetSpeed(0.45,-0.45);

    }
}

void Controller::Move(float sL, float sR) {
    if (sL < 0)
        DirL = 0;
    else
        DirL = 1;
    if (sR < 0)
        DirR = 0;
    else
        DirR = 1;

    PwmL = abs(sL);
    PwmR = abs(sR);
};

uint16_t Controller::PsdDistance(GP2A GP2A_, uint8_t i) {
  now_distance[i] = GP2A_.getDistance();
  filtered_distance[i] = now_distance[i] * alpha_psd + (1 - alpha_psd) * prev_distance[i];
  prev_distance[i] = now_distance[i];
  return filtered_distance[i];
}

void Controller::PsdRefresh() {
  psd_val[0] = PsdDistance(psdlf, 0);
  psd_val[1] = PsdDistance(psdf, 1);
  psd_val[2] = PsdDistance(psdrf, 2);
  psd_val[3] = PsdDistance(psdlc, 3);
  psd_val[4] = PsdDistance(psdrc, 4);
  psd_val[5] = PsdDistance(psdlb, 5);
  psd_val[6] = PsdDistance(psdb, 6);
  psd_val[7] = PsdDistance(psdrb, 7);
  PsdWallDetect();
}

void Controller::PsdWallDetect() {
    if (psd_val[0] <= 10 && psd_val[2] <= 10 && !GetEnemyState()) {
        FrontCollision = true; 
        SetWallSafetyState(false);
    } else if ((psd_val[0]+psd_val[2])/2 > 10) {
        FrontCollision = false;
        SetWallSafetyState(true);
    }
    if (psd_val[5] <= 10 && psd_val[7] <= 10) {
        BackCollision = true;
        SetWallSafetyState(false);
    } else if ((psd_val[5]+psd_val[7])/2 > 10) {
        BackCollision = false;
        SetWallSafetyState(true);
    }
    if (psd_val[0] <= 10 && psd_val[5] <= 10) {
        LeftCollision = true;
        SetWallSafetyState(false);
    } else if ((psd_val[0]+psd_val[5])/2 > 10) {
        LeftCollision = false;
        SetWallSafetyState(true);
    }
    if (psd_val[2] <= 10 && psd_val[7] <= 10) {
        RightCollision = true;
        SetWallSafetyState(false);
    } else if ((psd_val[2]+psd_val[7])/2 > 10) {
        RightCollision = false;
        SetWallSafetyState(true);
    }
}
void Controller::PsdWallEscape() {
  if (FrontCollision) {
    //전방에 벽 있으면 후진 후 돌기
    SetSpeed(-0.5, -0.5);
  }
  if (BackCollision) {
    //후방에 벽 있으면 전진 후 돌기
    SetSpeed(0.5, 0.5);
  }
  if (LeftCollision) {
    //왼쪽에 벽 있으면 90도 우회전
    SetSpeed(1.0, 0.3);
  }
  if (RightCollision) {
    //오른쪽에 벽 있으면 90도 좌회전
    SetSpeed(0.3, 1.0);
  }
}
/*
void Controller::IrRefresh() {
    //IR = false 이면 색영역
    ir_val[0] = irfl.read();
    ir_val[1] = irfr.read();
    ir_val[2] = irfc.read();
    ir_val[3] = irbc.read();
    ir_val[4] = irbl.read();
    ir_val[5] = irbr.read();
    ir_total = ir_val[0] + ir_val[1] + ir_val[2] + ir_val[3] + ir_val[4] + ir_val[5];
        //ir에서 피스톤질 모드::조금이라도 IR 있으면 일단 피하기->적 만나서 ATTACK 일때 색영역 Front 일때까지 밀면, 그때부터는 ir하나만걸려도 뒤로뺄예정.
    if (ir_total <= 3) { ColorOrient(); SetIrSafetyState(false);} //검정은 1로 뜸, 검정 영역 뜬 곳의 합이 3 이하라면?
    else { Orient = ColorOrient::SAFE; SetIrSafetyState(true);}
}
*/
void Controller::ColorOrient() {
    if (ir_total == 1) { 
        if (ir_val[0] == 1) {
            Orient = ColorOrient::BACK_RIGHT;
        } else if (ir_val[1]) {
            Orient = ColorOrient::BACK_LEFT;
        } else if (ir_val[4]) {
            Orient = ColorOrient::FRONT_RIGHT;
        } else if (ir_val[5]) {
            Orient = ColorOrient::FRONT_LEFT;
        } else {}
    } else if (ir_total == 2) {
        if (ir_val[4] && ir_val[5]) {
            Orient = ColorOrient::FRONT;
        } else if (ir_val[1] && ir_val[5]) {
            Orient = ColorOrient::TAN_LEFT;
        } else if (ir_val[0] && ir_val[1]) {
            Orient = ColorOrient::BACK;
        } else if (ir_val[0] && ir_val[4]) {
            Orient = ColorOrient::TAN_RIGHT;
        } 
    } else if (ir_total == 3) {
        if (ir_val[4] && ir_val[5]) {
            Orient = ColorOrient::FRONT;
        } else if (ir_val[1] && ir_val[5]) {
            Orient = ColorOrient::TAN_LEFT;
        } else if (ir_val[0] && ir_val[1]) {
            Orient = ColorOrient::BACK;
        } else if (ir_val[0] && ir_val[4]) {
            Orient = ColorOrient::TAN_RIGHT;
        } 
    } else if (ir_total == 4) {
        //2개씩만 인식 Front만 쓰지않을까? 영역은 주되 위험하지는 않은 상황
        SetIrSafetyState(true);
        if (ir_val[4] && ir_val[5]) {
            Orient = ColorOrient::FRONT;
        } else if (ir_val[1] && ir_val[5]) {
            Orient = ColorOrient::TAN_LEFT;
        } else if (ir_val[0] && ir_val[1]) {
            Orient = ColorOrient::BACK;
        } else if (ir_val[0] && ir_val[4]) {
            Orient = ColorOrient::TAN_RIGHT;
        } 
    } else if (ir_total >= 5) {
        SetIrSafetyState(true);
        Orient = ColorOrient::SAFE;
    } else {} //완전히 들어갔을 때, 적 찾다가 알아서 나갈것이므로 일단 비움
}
enum Controller::ColorOrient Controller::GetOrient() { return Orient;}

void Controller::IrRefresh_new() {
    ir_total = ir.read();
    switch(ir_total) {
        case 0b001111:
            Orient = ColorOrient::FRONT;
            SetIrSafetyState(true);
            break;
        case 0b001011:
            Orient = ColorOrient::FRONT;
            SetIrSafetyState(false);
            break;
        case 0b000011:
            Orient = ColorOrient::FRONT;
            SetIrSafetyState(false);
            break;
        case 0b110100:
            Orient = ColorOrient::BACK;
            SetIrSafetyState(false);
            break;
        case 0b110000:
            Orient = ColorOrient::BACK;
            SetIrSafetyState(false);
            break;
        case 0b011001:
            Orient = ColorOrient::TAN_LEFT;
            SetIrSafetyState(true);
            break;
        case 0b010101:
            Orient = ColorOrient::TAN_LEFT;
            SetIrSafetyState(true);
            break;
        case 0b010001:
            Orient = ColorOrient::TAN_LEFT;
            SetIrSafetyState(true);
            break;
        case 0b101010:
            Orient = ColorOrient::TAN_RIGHT;
            SetIrSafetyState(true);
            break;
        case 0b100110:
            Orient = ColorOrient::TAN_RIGHT;
            SetIrSafetyState(true);
            break;
        case 0b100010:
            Orient = ColorOrient::TAN_RIGHT;
            SetIrSafetyState(true);
            break;
        case 0b000101:
            Orient = ColorOrient::FRONT_LEFT;
            SetIrSafetyState(false);
            break;
        case 0b001001:
            Orient = ColorOrient::FRONT_LEFT;
            SetIrSafetyState(false);
            break;
        case 0b000001:
            Orient = ColorOrient::FRONT_LEFT;
            SetIrSafetyState(false);
            break;
        case 0b000110:
            Orient = ColorOrient::FRONT_RIGHT;
            SetIrSafetyState(false);
            break;
        case 0b001010:
            Orient = ColorOrient::FRONT_RIGHT;
            SetIrSafetyState(false);
            break;
        case 0b000010:
            Orient = ColorOrient::FRONT_RIGHT;
            SetIrSafetyState(false);
            break;
        case 0b011000:
            Orient = ColorOrient::BACK_LEFT;
            SetIrSafetyState(false);
            break;
        case 0b010100:
            Orient = ColorOrient::BACK_LEFT;
            SetIrSafetyState(false);
            break;
        case 0b010000:
            Orient = ColorOrient::BACK_LEFT;
            SetIrSafetyState(false);
            break;
        case 0b101000:
            Orient = ColorOrient::BACK_RIGHT;
            SetIrSafetyState(false);
            break;
        case 0b100100:
            Orient = ColorOrient::BACK_RIGHT;
            SetIrSafetyState(false);
            break;
        case 0b100000:
            Orient = ColorOrient::BACK_RIGHT;
            SetIrSafetyState(false);
            break;
        //완전히 들어가버린 색영역.. Criterion 선정 기준은 로봇은 항상 적을 찾으려고 노력할 것이므로, 적이 없어 벽을 보고 있다면(FrontCollision) 상태일때는 빠져나가기만 한다면 일단                  //다시 적 추적할 것. 그러므로 LEFT/RIGHT Collision도 걍 BACK으로 선정. 빨간 색영역에서는 Collision이 없을 것이며 그냥 후진하고, 파란색 색영역에서는 LEFT/RIGHT Collision이  뜨기 위해서는 
        //FRONT/BACK collision이 동반되므로, 정말 특별한 상황 아니면 쓸일 없을듯하다.
        case 0b000000: 
            if(FrontCollision) {
                Orient = ColorOrient::FRONT;
                SetIrSafetyState(false);
            } else if(BackCollision) {
                Orient = ColorOrient::BACK;
                SetIrSafetyState(false);
            } else if(LeftCollision) {
                Orient = ColorOrient::BACK_LEFT;
                SetIrSafetyState(false);
            } else if(RightCollision) {
                Orient = ColorOrient::BACK_RIGHT;
                SetIrSafetyState(false);
            } else {
                Orient = ColorOrient::FRONT;
                SetIrSafetyState(false);
            }
            break;
        default:
            Orient = ColorOrient::SAFE;
            SetIrSafetyState(true);
            break;
    }
}

void Controller::IrEscape() {
  if (Orient == ColorOrient::SAFE) {
    return;
  } else if (Orient == ColorOrient::FRONT) {
    SetSpeed(-0.5, -0.5);
  } else if (Orient == ColorOrient::TAN_LEFT) {
    // SetSpeed(0.5, -0.5);
  } else if (Orient == ColorOrient::TAN_RIGHT) {
    // SetSpeed(-0.5, 0.5);
  } else if (Orient == ColorOrient::BACK) {
    // 180, turn, recheck, and move
    SetSpeed(0.5, 0.5);
  } else if (Orient == ColorOrient::FRONT_LEFT) {
    // back, and turn
    SetSpeed(-0.5, 0.5);
  } else if (Orient == ColorOrient::FRONT_RIGHT) {
    // back, and turn
    SetSpeed(0.5, -0.5);
  } else if (Orient == ColorOrient::BACK_LEFT) {
    // back, and turn
    SetSpeed(0.5, -0.5);
  } else if (Orient == ColorOrient::BACK_LEFT) {
    // back, and turn
    SetSpeed(-0.5, 0.5);
  } else {}
}

//-----------------------MPU9250, IMU-----------------------------//
void Controller::ImuParse() {
    ebimu.putc(0x2A);
    ebimu.scanf("%s",data);
    controller.ImuChartoData();
    memset(data, NULL, 32*sizeof(char));
}

void Controller::ImuChartoData() {

    char* start = strchr(data, '*');
    if (start != NULL) {
        start++;

        char* token = strtok(start, ",");
        if (token != NULL) {
            roll = atof(token); 

            token = strtok(NULL, ",");
            if (token != NULL) {
                pitch = atof(token); 

                token = strtok(NULL, ",");
                if (token != NULL) {
                    rawYaw = atof(token); 
                     if (!isInitialized) {
                        // 최초 초기화 시 yaw 값을 0으로 설정
                        initialYaw = rawYaw;
                        isInitialized = true; // 초기화 완료 표시
                    }
                    yaw = NormalizeYaw(rawYaw - initialYaw); // 기준 yaw 값을 초기화한 값으로 설정하고 정규화              
                }
            }
        }
    }
}

void Controller::ImuDetect()  {
    if( psd_val[1] < 15 && pitch < -IMU_THRESHOLD) {
        SetImuSafetyState(false);
        ImuPitchLift = true;
        tilt_state = TiltState::FRONT;
    } else if(psd_val[0] < 9 && pitch < -IMU_THRESHOLD) {
        SetImuSafetyState(false);
        ImuPitchLift = true;
        tilt_state = TiltState::FRONT_LEFT;
    } else if(psd_val[2] < 9 && pitch <- IMU_THRESHOLD) {
        SetImuSafetyState(false);
        ImuPitchLift = true;
        tilt_state = TiltState::FRONT_RIGHT;
    } else if(psd_val[0] < 9 && roll < -IMU_THRESHOLD) {
        if(Escape_Timer.read_ms() == 0) Escape_Timer.start();
        if(psd_val[0] < 9 && roll > IMU_THRESHOLD && Escape_Timer.read_ms() > ESCAPE_TIME) {
            SetImuSafetyState(false);
            ImuRollLift = true;
            tilt_state = TiltState::SIDE_LEFT;
            Escape_Timer.reset();
        }
    } else if(psd_val[3] <= 30 && roll < -IMU_THRESHOLD) {
        if(Escape_Timer.read_ms() == 0) Escape_Timer.start();
        if(psd_val[3] <= 30 && roll < -IMU_THRESHOLD && Escape_Timer.read_ms() > ESCAPE_TIME) {
            SetImuSafetyState(false);
            ImuRollLift = true;
            tilt_state = TiltState::SIDE_LEFT;
            Escape_Timer.reset();
        }
    } else if(psd_val[2] < 9 && roll > IMU_THRESHOLD) {
        if(Escape_Timer.read_ms() == 0) Escape_Timer.start();
        if(psd_val[2] < 15 && roll > IMU_THRESHOLD && Escape_Timer.read_ms() > ESCAPE_TIME) {
            SetImuSafetyState(false);
            ImuRollLift = true;
            tilt_state = TiltState::SIDE_RIGHT;
            Escape_Timer.reset();
        }
    } else if(psd_val[4] <= 30 && roll > IMU_THRESHOLD) {
        if(Escape_Timer.read_ms() == 0) Escape_Timer.start();
        if(psd_val[4] <= 30 && roll > IMU_THRESHOLD && Escape_Timer.read_ms() > ESCAPE_TIME) {
            SetImuSafetyState(false);
            ImuRollLift = true;
            tilt_state = TiltState::SIDE_RIGHT;
            Escape_Timer.reset();
        }
    } else if (pitch > -IMU_THRESHOLD && ImuPitchLift) { 
        SetImuSafetyState(true);
        ImuPitchLift = false;
        tilt_state = TiltState::SAFE;
    } else if (roll > -IMU_THRESHOLD && roll < IMU_THRESHOLD && ImuRollLift) {
        SetImuSafetyState(true);
        ImuRollLift = false;
        tilt_state = TiltState::SAFE;
    }
    if(Escape_Timer.read_ms() > 10000) Escape_Timer.reset();
}

void Controller::ImuEscape() {
    switch (tilt_state) {
        case TiltState::FRONT:
            SetSpeed(1.0,1.0);
            // if(BackCollision) WallTwerk();//벽_타기_함수(); 또는 벽_타기 State, 여유가 없다면 backcollision 대신 긴 PSD의 거리 재기
            break;
        case TiltState::FRONT_LEFT:
            SetSpeed(0.5,1.0);
            break;
        case TiltState::FRONT_RIGHT:
            SetSpeed(1.0,0.5);
            break;
        case TiltState::SIDE_LEFT:
            SetSpeed(-1.0,-1.0);
            break;
        case TiltState::SIDE_RIGHT:
            SetSpeed(-1.0,-1.0);
            break;
        case TiltState::SAFE:
            return;
    }
}

float Controller::GetCurrentYaw()
{
    return yaw;
}

void Controller::SetCurrentYaw(float y)
{
    yaw = y;
}

float Controller::NormalizeYaw(float angle)
{
    if(angle>180) angle -=360;
    if(angle<-180) angle +=360;
    return angle;
}

//------------------------------Thread, Callbacks--------------------------------//
void ImuThread() {
    while(1) {
        mutex.lock();
        controller.ImuParse();
        controller.PsdRefresh();
        controller.ImuDetect();
        controller.IrRefresh_new();
        mutex.unlock();
        ThisThread::sleep_for(20);
    }
}

void EnemyDetect()
{
    a = device.getc();
    if(a=='[') {
        Incoming = true;
    } else if(a==']') {
        Incoming = false;
        bufferIndex = 0;
        if(AsteriskReceived) {
            controller.SetEnemyState(false);
        } else {
            controller.SetHD(atoi(buffer));
            controller.SetEnemyState(true);
        }
        memset(buffer,NULL,8*sizeof(char));
    } else if(a=='*') {
        AsteriskReceived = true;
    } else {
        AsteriskReceived = false;
        if(Incoming) {
            buffer[bufferIndex++] = a;
        }
    }
}
void Starter() {
    controller.StartFlag = true;
}

//---------------Tester Functions------------------//
void Controller::OrientViewer() {
    if((int)Orient == 0) {
        pc.printf("FRONT\r\n");
    } else if((int)Orient == 1) {
        pc.printf("TAN_LEFT\r\n");
    } else if((int)Orient == 2) {
        pc.printf("TAN_RIGHT\r\n");
    } else if((int)Orient == 3) {
        pc.printf("BACK\r\n");
    } else if((int)Orient == 4) {
        pc.printf("FRONT_LEFT\r\n");
    } else if((int)Orient == 5) {
        pc.printf("FRONT_RIGHT\r\n");
    } else if((int)Orient == 6) {
        pc.printf("BACK_LEFT\r\n");
    } else if((int)Orient == 7) {
        pc.printf("BACK_RIGHT\r\n");
    } else if((int)Orient == 8) {
        pc.printf("SAFE\r\n");
    }
}

void Controller::WallViewer() {
    if(controller.FrontCollision == true) pc.printf("Front ");
    else pc.printf("      ");
    if(controller.BackCollision == true) pc.printf("Back ");
    else pc.printf("     ");
    if(controller.LeftCollision == true) pc.printf("Left ");
    else pc.printf("     ");
    if(controller.RightCollision == true) pc.printf("Right ");
    else pc.printf("      ");
    pc.printf("\r\n");
}

void Controller::ImuViewer() {
    switch (tilt_state) {
        case TiltState::FRONT:
            pc.printf("FRONT\r\n");
            // pc.printf("1\r\n");
            break;
        case TiltState::FRONT_LEFT:
            pc.printf("FRONT_LEFT\r\n");
            // pc.printf("2\r\n");
            break;
        case TiltState::FRONT_RIGHT:
            pc.printf("FRONT_RIGHT\r\n");
            // pc.printf("3\r\n");
            break;
        case TiltState::SIDE_LEFT:
            pc.printf("SIDE_LEFT\r\n");
            // pc.printf("4\r\n");
            break;
        case TiltState::SIDE_RIGHT:
            pc.printf("SIDE_RIGHT\r\n");
            // pc.printf("5\r\n");
            break;
        case TiltState::SAFE:
            pc.printf("SAFE\r\n");
            // pc.printf("0\r\n");
            return;
    }
}
