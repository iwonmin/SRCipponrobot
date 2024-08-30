#include "controller.h"
char buffer[8] = "";
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
DigitalIn irfl(PC_4);
DigitalIn irfr(PB_1);
DigitalIn irfc(PA_6);
DigitalIn irbc(PA_8);//new pin!!
DigitalIn irbl(PA_7);
DigitalIn irbr(PA_5);
Serial ebimu(PB_10,PC_5,115200);
// MPU9250 mpu9250(D14, D15);
Controller controller;
Thread Thread1;
Thread Thread2;
Thread Thread4;
#pragma endregion variables
Mutex mutex;

#pragma region Serial Variables
// PC와의 통신을 위한 Serial 객체 생성
Serial pc(USBTX, USBRX, 115200);
// Raspberry Pi와의 통신 설정 (TX, RX, baud rate)
Serial device(D8, D2, 9600);
// 소수점 발견 여부를 추적하기 위한 변수
bool decimalPointSeen = false;
// 부호 정보를 추적하기 위한 변수
bool isNegative = false;

char distanceBuffer[256];

float initialYaw=0.0;
int bufferIndex = 0;
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
//   float speedL_i = GetSpeedL();
//   float speedR_i = GetSpeedR();
//   int interval_L = (sL - speedL_i) / 0.1f;
//   int interval_R = (sR - speedR_i) / 0.1f;
//   for (int i = 0; i <= abs(interval_L); i++) {
//     if (interval_L >= 0) {
//       speedL = speedL_i + 0.1 * i;
//     } else {
//       speedL = speedL_i - 0.1 * i;
//     }
//   }
//   for (int i = 0; i <= abs(interval_R); i++) {
//     if (interval_R >= 0) {
//       speedR = speedR_i + 0.1 * i;
//     } else {
//       speedR = speedR_i - 0.1 * i;
//     }
//   }
  speedL = sL;
  speedR = sR;
};
//==================================flags startregion=================================//
bool Controller::GetEnemyState() { return enemy; }

void Controller::SetEnemyState(bool enemyState) { enemy = enemyState; }

bool Controller::GetStartFlag() {return StartFlag;}

void Controller::SetStartFlag(bool sf){StartFlag=sf;}

bool Controller::GetAttackState() { return attack; }

void Controller::SetAttackState(bool attackState) { enemy = attackState; }

bool Controller::GetIrSafetyState() { return irSafe; }

void Controller::SetIrSafetyState(bool IrSafetyState) { irSafe = IrSafetyState; }

bool Controller::GetImuSafetyState() { return imuSafe; }

void Controller::SetImuSafetyState(bool ImuSafetyState) { imuSafe = ImuSafetyState; }

bool Controller::GetWallSafetyState() { return wallSafe; };

void Controller::SetWallSafetyState(bool WallSafetyState) { wallSafe = WallSafetyState; }
//==================================flags endregion=================================//

int Controller::GetHD() { return enemy_horizontal_distance; }

void Controller::SetHD(int HD) { enemy_horizontal_distance = HD; }

bool Controller::GetYellow(){return yellow;}

void Controller::SetYellow(bool y){yellow = y;}

int Controller::GetYA(){return yellowAngle;}

void Controller::SetYA(int YA){yellowAngle = YA;}

int Controller::GetYHD(){return yellow_horizontal_distance;}

void Controller::SetYHD(int YHD){yellow_horizontal_distance = YHD;}

void Controller::CheckStartTime() {
    StartTime = Kernel::get_ms_count();
}

uint64_t Controller::GetStartTime() {
    return StartTime;
}
void Controller::Start() {
    //pc.printf("Start\n");
    if(StartFlag) {
    PwmL.period_us(66);
    PwmR.period_us(66);
    Thread1.start(DetectThread2);
    Thread1.set_priority(osPriorityNormal);
    Thread2.start(ImuThread);
    Thread2.set_priority(osPriorityHigh);
    // Thread2.start(PsdThread);
    //Thread2.set_priority(osPriorityAboveNormal);
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
/*
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
            if(GetEnemyState()) {
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
            if(GetOrient()!=ColorOrient::FRONT)
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
    }else if(lastDirection<0){
        if(GetCurrentYaw()>=-40)
        {
            SetSpeed(-0.5,0.5);
        }else if(GetCurrentYaw()<=-50)
        {
            SetSpeed(0.5,-0.5);
        }else if(GetCurrentYaw()<-40 && GetCurrentYaw()>-50)
        {
            if(GetOrient()!=ColorOrient::FRONT)
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

/*

void Controller::EnemyDetect() {
 
  if (device.readable()) {
    char receivedChar = device.getc();
    if (receivedChar == '*') {
      SetEnemyState(false);
    } else {
      SetEnemyState(true);
    }
    // if (receivedChar == '/') {
    //   distanceBuffer[bufferIndex] = '\0';
    //   SetHD(atoi(distanceBuffer));
    //   pc.printf("Received Distance: %d\n", GetHD());
    //   bufferIndex = 0; // 버퍼 초기화
    // } else {
    //   distanceBuffer[bufferIndex] = receivedChar;
    //   bufferIndex++;
    //   if (bufferIndex >= sizeof(distanceBuffer) - 1) {
    //     bufferIndex = 0; // 버퍼가 가득 찬 경우 초기화
    //   }
    // }
    if(receivedChar=='\n')
    {
        distanceBuffer[bufferIndex]='\0';
        char *comma_ptr1=strchr(distanceBuffer,',');//첫번째 콤마 위치
        char *comma_ptr2=NULL;
        if(comma_ptr1 !=NULL)
        {
            *comma_ptr1='\0';//첫 번째 콤마 위치를 문자열 종료로 설정
            char* data1=distanceBuffer;//첫 번째 데이터

            if(*data1 != '*') SetHD(atoi(data1));
            comma_ptr2 = strchr(comma_ptr1+1,',');//두번째 콤마 위치
            
            if(comma_ptr2!=NULL)
            {
                *comma_ptr2 ='\0';//두 번째 콤마 위치를 문자열 종료로 설정
                char* data2=comma_ptr1+1;//두번째 데이터
                if(*data2!='*')
                {
                    SetYHD(atoi(data2));
                } else
                {
                    SetYHD(404);
                }                   
                char* data3=comma_ptr2+1;//세번째 데이터
                if(*data3!='*')
                {
                    SetYA(atof(data3));
                }else
                {
                    SetYA(404);
                } 
                    
                //데이터 출력
                //pc.printf("Data1: %s, Data2: %s, Data3: %s\r\n", data1, data2, data3);
                pc.printf("GHD: %d, YHD: %d, YA: %d, yaw: %.2f\r\n",GetHD(),GetYHD(),GetYA(),GetCurrentYaw());
            }else{
                pc.printf("Error: Only two data fields found!\r\n");
            }
        }else{
            pc.printf("Error: Only one data field found!\r\n");
        }
        bufferIndex =0; //인덱스 초기화
    }else{
        //버퍼가 가득 차지 않았을 경우에만 저장
        if(bufferIndex<sizeof(distanceBuffer)-1){
            distanceBuffer[bufferIndex++]=receivedChar;//버퍼에 데이터 저장
        }
    }
  }
}



uint16_t Controller::PsdDistance(GP2A GP2A_, uint8_t i) {
  now_distance[i] = GP2A_.getDistance();
  filtered_distance[i] = now_distance[i] * alpha_psd + (1 - alpha_psd) * prev_distance[i];
  uint16_t difference = fabs(filtered_distance[i] - prev_distance[i]);
  if (difference > PSD_THRESHOLD) {
    detection[i] = 1;
  } else {
    detection[i] = 0;
  }
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
/*
void Controller::PsdDetect() { //아직 안돌려봄
    if (FollowIndex == 0){
        MinValue = psd_val[6];
        MinIndex = 1;
        if (psdlc < MinValue) { MinValue = psd_val[3]; MinIndex = 2; }
        if (psdrc < MinValue) { MinValue = psd_val[4]; MinIndex = 3; }
        switch (MinIndex) {
            case 1: //in which case psdb is minimum
                    if(detection[3]) {
                        //psdlc detected, ccw
                        SetSpeed(-1.0,1.0);
                        FollowIndex = 2;
                    } else if(detection[4]) {
                        //psdrc detected, cw
                        SetSpeed(1.0,-1.0);
                        FollowIndex = 3;
                    } else if(detection[6]) {
                        //psdb detected, both but cw
                        SetSpeed(1.0,-1.0);
                        FollowIndex = 1;
                    }
                break;
            case 2: //in which case psdlc is minimum
                    if(detection[6]) {
                        //psdb detected, both but cw
                        SetSpeed(1.0,-1.0);
                        FollowIndex = 1;
                    } else if(detection[4]) {
                        //psdrc detected, cw
                        SetSpeed(1.0,-1.0);
                        FollowIndex = 3;
                    } else if(detection[3]) {
                        //psdlc detected, ccw
                        SetSpeed(-1.0,1.0);
                        FollowIndex = 2;
                    }             
                break;
            case 3: //in which case psdrc is minimum
                    if(detection[3]) {
                        //psdlc detected, both but cw
                        SetSpeed(1.0,-1.0);
                        FollowIndex = 2;
                    } else if(detection[6]) {
                        //psdb detected, ccw
                        SetSpeed(-1.0,1.0);
                        FollowIndex = 1;
                    } else if(detection[4]) {
                        //psdrc detected, cw
                        SetSpeed(1.0,-1.0);
                        FollowIndex = 3;
                    }
                break;
        }        
    }
    if (FollowIndex != 0){
        switch (MinIndex) {
            case 1: //in which case psdb is minimum
                    if(psd_val[3] == MinValue) {
                        //psdlc detected, ccw
                        SetSpeed(0,0);
                        FollowIndex = 0;
                    } else if(psd_val[4] == MinValue) {
                        //psdrc detected, cw
                        SetSpeed(0,0);
                        FollowIndex = 0;
                    } else if(psd_val[6] == MinValue) {
                        //psdb detected, both but cw
                        SetSpeed(0,0);
                        FollowIndex = 0;
                    }
                break;
            case 2: //in which case psdlc is minimum
                    if(psd_val[6] == MinValue) {
                        //psdb detected, both but cw
                        SetSpeed(0,0);
                        FollowIndex = 0;
                    } else if(psd_val[4]) {
                        //psdrc detected, cw
                        SetSpeed(0,0);
                        FollowIndex = 3;
                    } else if(psd_val[3] == MinValue) {
                        //psdlc detected, ccw
                        SetSpeed(0,0);
                        FollowIndex = 0;
                    }             
                break;
            case 3: //in which case psdrc is minimum
                    if(psd_val[3] == MinValue) {
                        //psdlc detected, both but cw
                        SetSpeed(0,0);
                        FollowIndex = 0;
                    } else if(psd_val[6] == MinValue) {
                        //psdb detected, ccw
                        SetSpeed(0,0);
                        FollowIndex = 0;
                    } else if(psd_val[4] == MinValue) {
                        //psdrc detected, cw
                        SetSpeed(0,0);
                        FollowIndex = 0;
                    }
                break;
        }        
    }
}
*/
void Controller::PsdWallDetect() {
    if (psd_val[0] <= 20 && psd_val[2] <= 20 && !GetEnemyState()) {
        FrontCollision = true; 
        SetWallSafetyState(false);
    } else if ((psd_val[0]+psd_val[2])/2 > 20) {
        FrontCollision = false;
        SetWallSafetyState(true);
    }
    if (psd_val[5] <= 20 && psd_val[7] <= 20) {
        BackCollision = true;
        SetWallSafetyState(false);
    } else if ((psd_val[5]+psd_val[7])/2 > 20) {
        BackCollision = false;
        SetWallSafetyState(true);
    }
    if (psd_val[0] <= 20 && psd_val[5] <= 20) {
        LeftCollision = true;
        SetWallSafetyState(false);
    } else if ((psd_val[0]+psd_val[5])/2 > 20) {
        LeftCollision = false;
        SetWallSafetyState(true);
    }
    if (psd_val[2] <= 20 && psd_val[7] <= 20) {
        RightCollision = true;
        SetWallSafetyState(false);
    } else if ((psd_val[2]+psd_val[7])/2 > 20) {
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
    ir_val[0] = irfl.read();
    ir_val[1] = irfr.read();
    ir_val[2] = irfc.read();
    ir_val[3] = irbc.read();
    ir_val[4] = irbl.read();
    ir_val[5] = irbr.read();
    ColorOrient_new();
}

void Controller::ColorOrient_new() {
    //0 b (irfl) (irfr) (irfc) (irbc) (irbl) (irbr) || gyumin == 1, white == 0;
    switch((ir_val[0] << 5) | (ir_val[1] << 4) | (ir_val[2] << 3) | (ir_val[3] << 2) | (ir_val[4] << 1) | ir_val[5]) {
        case 0b001111:
            Orient = ColorOrient::FRONT;
            SetIrSafetyState(true);
        case 0b001011:
            Orient = ColorOrient::FRONT;
            SetIrSafetyState(false);
        case 0b000011:
            Orient = ColorOrient::FRONT;
            SetIrSafetyState(false);
        case 0b110100:
            Orient = ColorOrient::BACK;
            SetIrSafetyState(false);
        case 0b110000:
            Orient = ColorOrient::BACK;
            SetIrSafetyState(false);
        case 0b011001:
            Orient = ColorOrient::TAN_LEFT;
            SetIrSafetyState(true);
        case 0b010101:
            Orient = ColorOrient::TAN_LEFT;
            SetIrSafetyState(true);
        case 0b010001:
            Orient = ColorOrient::TAN_LEFT;
            SetIrSafetyState(true);
        case 0b101010:
            Orient = ColorOrient::TAN_RIGHT;
            SetIrSafetyState(true);
        case 0b100110:
            Orient = ColorOrient::TAN_RIGHT;
            SetIrSafetyState(true);
        case 0b100010:
            Orient = ColorOrient::TAN_RIGHT;
            SetIrSafetyState(true);
        case 0b000101:
            Orient = ColorOrient::FRONT_LEFT;
            SetIrSafetyState(false);
        case 0b001001:
            Orient = ColorOrient::FRONT_LEFT;
            SetIrSafetyState(false);
        case 0b000001:
            Orient = ColorOrient::FRONT_LEFT;
            SetIrSafetyState(false);
        case 0b000110:
            Orient = ColorOrient::FRONT_RIGHT;
            SetIrSafetyState(false);
        case 0b001010:
            Orient = ColorOrient::FRONT_RIGHT;
            SetIrSafetyState(false);
        case 0b000010:
            Orient = ColorOrient::FRONT_RIGHT;
            SetIrSafetyState(false);
        case 0b011000:
            Orient = ColorOrient::BACK_LEFT;
            SetIrSafetyState(false);
        case 0b010100:
            Orient = ColorOrient::BACK_LEFT;
            SetIrSafetyState(false);
        case 0b010000:
            Orient = ColorOrient::BACK_LEFT;
            SetIrSafetyState(false);
        case 0b101000:
            Orient = ColorOrient::BACK_RIGHT;
            SetIrSafetyState(false);
        case 0b100100:
            Orient = ColorOrient::BACK_RIGHT;
            SetIrSafetyState(false);
        case 0b100000:
            Orient = ColorOrient::BACK_RIGHT;
            SetIrSafetyState(false);
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
        default:
            Orient = ColorOrient::SAFE;
            SetIrSafetyState(true);
        break;
    }
}
/*
Controller::Position Controller::GetPosition() { return CurrentPos; }
//Position::@@@@@@@@@@@@@@@@조건 너무 빈약, 고쳐야함.
//getDistance() 타이밍에 로봇 있을 때 거를 방안 찾아야함. 
//거리 함수 말고 전역 변수로 불러와야할 듯(controller)
//irs Colororient=>정확성 높음, 벽거리만 추가고려해서 바로 사용
void Controller::SetPosition() { 
    if (Orient == ColorOrient::TAN_LEFT && psd_val[3] < CIRCLE_DISTANCE) {
    CurrentPos = Position::ClosetoLeftWall;
    return;
    } else if (Orient == ColorOrient::TAN_RIGHT && psd_val[4] < CIRCLE_DISTANCE) {
    CurrentPos = Position::ClosetoRightWall;
    return;
    } else if (Orient == ColorOrient::FRONT_LEFT && psd_val[3] < CIRCLE_DISTANCE && psd_val[1] < WALL_DISTANCE) {
    CurrentPos = Position::CriticalLeftWall;
    //뒤로, 오른쪽으로 이동하는 것 필요
    } else if (Orient == ColorOrient::BACK_LEFT && psd_val[3] < CIRCLE_DISTANCE && psd_val[6] < WALL_DISTANCE) {
    CurrentPos = Position::CriticalLeftWall;
    //앞으로, 오른쪽으로 이동하는 것 필요
    } else if (Orient == ColorOrient::FRONT_RIGHT && psd_val[4] < CIRCLE_DISTANCE && psd_val[1] < WALL_DISTANCE) {
    CurrentPos = Position::CriticalRightWall;
    //뒤로, 왼쪽으로 이동하는 것 필요
    } else if (Orient == ColorOrient::BACK_RIGHT && psd_val[4] < CIRCLE_DISTANCE && psd_val[6] < WALL_DISTANCE) {
    CurrentPos = Position::CriticalRightWall;
    //앞으로, 왼쪽으로 이동하는 것 필요
    } else if (Orient != ColorOrient::SAFE && psd_val[0] > 90 && psd_val[7] > 90 || psd_val[2] > 90 && psd_val[5] > 90) {
    //일단 ir에 색은 감지되었지만 벽과의 거리가 생각보다 멀때 -> 중앙임
    CurrentPos = Position::ClosetoCenter;
    } else {
    // ir 영역 아닐떄, psd만 사용(부정확)
    if (psd_val[1] <= 10) {
      CurrentPos = Position::WallFront;
    } else if (psd_val[6] <= 10) {
      CurrentPos = Position::WallBehind;
    } else
      CurrentPos = Position::FartoCenter; // 색영역도 아닌데 안보임
  }
}
*/
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
/*
void Controller::IrEscapeWhenImuUnsafe() {
    if(Controller::ir_val[2]+Controller::ir_val[3]+Controller::ir_val[4] < 2) {
        //뒤쪽에 IR영역
        SetSpeed(0.5,-0.5);
    } else if(Controller::ir_val[0]+Controller::ir_val[1]+Controller::ir_val[2] < 2) {
        //앞쪽에 IR영역
        SetSpeed(-0.5, 0.5);
    } else if(Controller::ir_val[0]+Controller::ir_val[1]+Controller::ir_val[3] < 2) {
        //왼쪽에 IR영역
        SetSpeed(0.5, 0.5);
    } else if(Controller::ir_val[1]+Controller::ir_val[2]+Controller::ir_val[4] < 2) {
        //오른쪽에 IR영역
        SetSpeed(-0.5, -0.5);
    }
}
*/

//Imu Thread에서 새로운 Sub-Thread 실행, 여기서 IMU 값 새로 받아와야함
/*
void Controller::WallTwerk() {
    bool FinishMove = false;
    bool Orient = false;
    while(1) {
        EnemyDetect();
        if(!FinishMove) {
            if (!Orient) {
                if (abs(psd_val[5] - psd_val[7]) < 5) Orient = true;
                if (psd_val[5] > psd_val[7]) SetSpeed(-0.3, 0.3);
                if (psd_val[5] < psd_val[7]) SetSpeed(0.3,-0.3);
            }
            if (!BackCollision) { SetSpeed(-0.3); }
            else {
                SetSpeed(-0.1);
                if(mpu9250.pitch < -10.0f) FinishMove = true;
            }
        }
        if(GetEnemyState()) {
            SetState(RoboState::ATTACK);
            return;
        controller.ImuRefresh_MPU9250();
        }
    }
}
*/
void Controller::ImuDetect()  {
    if(GetEnemyState() && psd_val[1] < 15 && pitch < -IMU_THRESHOLD) {
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
*/
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
bool isInitialized = false;

float Controller::NormalizeYaw(float angle)
{
    if(angle>180) angle -=360;
    if(angle<-180) angle +=360;
    return angle;
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
                    float rawYaw = atof(token); 
                     if (!isInitialized) {
                        // 최초 초기화 시 yaw 값을 0으로 설정
                        initialYaw = rawYaw;
                        isInitialized = true; // 초기화 완료 표시
                    }
                    yaw = NormalizeYaw(rawYaw - initialYaw); // 기준 yaw 값을 초기화한 값으로 설정하고 정규화
/*
                    token = strtok(NULL, ",");
                    if (token != NULL) {
                        ax = atof(token);

                        token = strtok(NULL, ",");
                        if (token != NULL) {
                            ay = atof(token);

                            token = strtok(NULL, ",");
                            if (token != NULL) {
                                az = atof(token);
                        
                            }
                        }                       
                    }                     
                }
            }
        }
    }
}

void Controller::ImuParse() {
    ebimu.putc(0x2A);
    ebimu.scanf("%s",data);
    controller.ImuChartoData();
    memset(data, NULL, 32*sizeof(char));
}

//------------------------------Thread&NotController--------------------------------//
void ImuThread() {
    pc.printf("IMU Thread running\n");
    while(1) {
        mutex.lock();
        controller.ImuParse();
        controller.PsdRefresh();
        controller.IrRefresh();
        mutex.unlock();
        // controller.ImuDetect();
        //pc.printf("HD: %d,Roll: %.1f, Pitch: %.1f, Yaw:%.1f, Z AC: %.1f\r\n",controller.GetHD(),controller.roll, controller.pitch, controller.yaw, controller.az);
        ThisThread::sleep_for(20);
    }
}
void PsdThread() {
    while(1) {
        controller.PsdRefresh();
        controller.IrRefresh();
        // pc.printf("%d, %d, %d, %d, %d, %d, %d, %d\r\n",controller.psd_val[0],controller.psd_val[1],controller.psd_val[2],controller.psd_val[3],controller.psd_val[4],controller.psd_val[5],controller.psd_val[6],controller.psd_val[7]);
        // pc.printf("%d, %d, %d, %d, %d, %d \r\n",irfl.read(), irfr.read(), irfc.read(), irbc.read(), irbl.read(), irbr.read());
        // pc.printf("%d, %d, %d, %d, %d",controller.GetState(), controller.GetAttackState(), controller.GetImuSafetyState(), controller.GetIrSafetyState(), controller.GetWallSafetyState());
        // controller.OrientViewer();
        // controller.SetPosition();
        // controller.WallViewer();
        ThisThread::sleep_for(20); //임의
    }
}

void DetectThread()
{
    pc.printf("Detect Thread running\n");
    while(1)
    {        
        //pc.printf("Detecting\n");
        if (device.readable()) {
        char receivedChar = device.getc();
        if (receivedChar == '*') {
        controller.SetEnemyState(false);
        } else {
        controller.SetEnemyState(true);
        }
        if(receivedChar=='\n')
        {
            distanceBuffer[bufferIndex]='\0';
            char *comma_ptr1=strchr(distanceBuffer,',');//첫번째 콤마 위치
            char *comma_ptr2=NULL;
            if(comma_ptr1 !=NULL)
            {
                *comma_ptr1='\0';//첫 번째 콤마 위치를 문자열 종료로 설정
                char* data1=distanceBuffer;//첫 번째 데이터

                if(*data1 != '*') controller.SetHD(atoi(data1));
                comma_ptr2 = strchr(comma_ptr1+1,',');//두번째 콤마 위치
                
                if(comma_ptr2!=NULL)
                {
                    *comma_ptr2 ='\0';//두 번째 콤마 위치를 문자열 종료로 설정
                    char* data2=comma_ptr1+1;//두번째 데이터
                    if(*data2!='*')
                    {
                        controller.SetYHD(atoi(data2));
                    } else
                    {
                        controller.SetYHD(404);
                    }                   
                    char* data3=comma_ptr2+1;//세번째 데이터
                    if(*data3!='*')
                    {
                        controller.SetYA(atof(data3));
                    }else
                    {
                        controller.SetYA(404);
                    } 
                        
                    //데이터 출력
                   // pc.printf("Data1: %s, Data2: %s, Data3: %s\r\n", data1, data2, data3);
                    mutex.lock();
                    pc.printf("GHD: %d, YHD: %d, YA: %d yaw: %.2f\r\n",controller.GetHD(),controller.GetYHD(),controller.GetYA(), controller.GetCurrentYaw());
                    mutex.unlock();
                }else{
                    mutex.lock();
                    pc.printf("Error: Only two data fields found!\r\n");
                    mutex.unlock();
                }
            }else{
                mutex.lock();
                //pc.printf("Error: Only one data field found!\r\n");
                mutex.unlock();
            }
            bufferIndex =0; //인덱스 초기화
        }else{
            //버퍼가 가득 차지 않았을 경우에만 저장
            if(bufferIndex<sizeof(distanceBuffer)-1){
                distanceBuffer[bufferIndex++]=receivedChar;//버퍼에 데이터 저장
            }
        }
    }
    ThisThread::sleep_for(1);
    }
}

void DetectThread2()
{
    char buffer_D[1024];

    int index_D=0;

    bool receiving_D = false;
    pc.printf("DetecThread2 running\n");
    while(1)
    {
        if(device.readable())
        {
            char c = device.getc();
            if (c == '*') {
            controller.SetEnemyState(false);
            } else 
            {
                controller.SetEnemyState(true);
            }
            if(c=='[')
            {
                receiving_D=true;
                index_D=0;
            }
            if(receiving_D)
            {
                buffer[index_D++]=c;
                if(c==']')
                {
                    buffer[index_D++]='\0';
                    //pc.printf("Received Data: %s Yaw : %.2f\n",buffer, controller.GetCurrentYaw());
                    char *token = strtok(buffer + 1, "]"); // 대괄호 안의 값 추출
                    if (token != NULL) {
                        mutex.lock();
                        controller.SetHD(atoi(token)); // 값을 정수로 변환
                        mutex.unlock();
                    }
                    // pc.printf("HD : %d, Yaw: %.2f\n",controller.GetHD(),controller.GetCurrentYaw());
                    pc.printf("HD: %d\n", controller.GetHD());
                    receiving_D=false;
                }
            }
             if (index_D >= sizeof(buffer_D) - 1) {
            index_D = 0; // 버퍼 초기화
            receiving_D = false; // 수신 종료
            }               
        }
        // mutex.lock();
        // controller.ImuParse();
        // mutex.unlock();
        ThisThread::sleep_for(1);
    }
}
void Starter() {
    controller.StartFlag = true;
}

//---------------임시------------------//
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
float Controller::GetCurrentYaw()
{
    return yaw;
}

void Controller::SetCurrentYaw(float y)
{
    yaw = y;
}
