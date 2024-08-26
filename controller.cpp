#include "controller.h"
char buffer[8] = "";
#pragma region variables
InterruptIn btn(BUTTON1);
DigitalOut DirL(PC_7);
DigitalOut DirR(PB_6);
PwmOut PwmL(PB_4);
PwmOut PwmR(PB_5);
GP2A psdlf(PA_0, 7, 80, 24.6, -0.297);
GP2A psdf(PB_0, 30, 150, 60, 0);
GP2A psdrf(PA_1, 7, 80, 24.6, -0.297);
GP2A psdlc(PA_4, 30, 150, 60, 0);
GP2A psdrc(PC_1, 30, 150, 60, 0);
GP2A psdlb(PC_3, 7, 80, 24.6, -0.297);
GP2A psdb(PC_2, 30, 150, 60, 0);
GP2A psdrb(PC_0, 7, 80, 24.6, -0.297);
DigitalIn irfl(PC_4);
DigitalIn irfr(PB_1);
DigitalIn irfc(PA_6);
DigitalIn irbc(PC_5);//new pin!!
DigitalIn irbl(PA_7);
DigitalIn irbr(PA_5);

MPU9250 mpu9250(D14, D15);
Controller controller;
Thread Thread1;
Thread Thread2;
#pragma endregion variables

#pragma region Serial Variables
// PC와의 통신을 위한 Serial 객체 생성
Serial pc(USBTX, USBRX, 115200);
// Raspberry Pi와의 통신 설정 (TX, RX, baud rate)
Serial device(D8, D2, 9600);
// 소수점 발견 여부를 추적하기 위한 변수
bool decimalPointSeen = false;
// 부호 정보를 추적하기 위한 변수
bool isNegative = false;

char distanceBuffer[32];

int bufferIndex = 0;
#pragma endregion Serial Variables


Controller::Controller() { 
    SetState(RoboState::START);
    btn.fall(&Starter);
    PwmL.period_us(66);
    PwmR.period_us(66);
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
  float speedL_i = GetSpeedL();
  float speedR_i = GetSpeedR();
  int interval_L = (sL - speedL_i) / 0.1f;
  int interval_R = (sR - speedR_i) / 0.1f;
  for (int i = 0; i <= abs(interval_L); i++) {
    if (interval_L >= 0) {
      speedL = speedL_i + 0.1 * i;
    } else {
      speedL = speedL_i - 0.1 * i;
    }
  }
  for (int i = 0; i <= abs(interval_R); i++) {
    if (interval_R >= 0) {
      speedR = speedR_i + 0.1 * i;
    } else {
      speedR = speedR_i - 0.1 * i;
    }
  }
  // speedL = sL;
  // speedR = sR;
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

void Controller::Start() {
    if(StartFlag) {
    Thread1.start(ImuThread);
    Thread1.set_priority(osPriorityHigh);
    Thread2.start(PsdThread);
    Thread2.set_priority(osPriorityAboveNormal);
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
        if (GetEnemyState()) {
        SetSpeed(0);
        SetState(RoboState::ATTACK);
        } else if (!GetEnemyState() && GetHD() > 0) {
        SetSpeed(-0.5, 0.5);
        } else if (!GetEnemyState() && GetHD() < 0) {
        SetSpeed(0.5, -0.5);
        }
    } else {
        SetState(RoboState::IDLE);
    }
};

void Controller::Attack() {//에다가 ir 위험 신호 받으면 Ir_Escape 실행할 수 있게 하기
    if(GetOrient() == ColorOrient::FRONT) SetAttackState(true);
    if (irSafe && imuSafe) {
        SetSpeed(1.0);
        if (!GetEnemyState()) {
        SetState(RoboState::IDLE);
        SetAttackState(false);
        }
    } else {
        SetState(RoboState::IDLE);
    }
};

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
    if (receivedChar == '/') {
      distanceBuffer[bufferIndex] = '\0';
      SetHD(atoi(distanceBuffer));
      pc.printf("Received Distance: %d\n", GetHD());
      bufferIndex = 0; // 버퍼 초기화
    } else {
      distanceBuffer[bufferIndex] = receivedChar;
      bufferIndex++;
      if (bufferIndex >= sizeof(distanceBuffer) - 1) {
        bufferIndex = 0; // 버퍼가 가득 찬 경우 초기화
      }
    }
  }
}
*/

void Controller::EnemyDetect() {//실험용 짭
    // if(psd_val[1] <= 35) {
    //     SetEnemyState(true);
    //     SetHD(0);
    // } else { SetEnemyState(false); SetHD(20.f);}
    // SetEnemyState(true);
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
/*
void Controller::CenterSpin() {//굳이??싶은 함수
  SetSpeed(0.5, -0.5); //빙글빙글
  if (detection[0] || detection[2] || detection[5] || detection[7]) {
    SetSpeed(0, 0);
    ThisThread::sleep_for(50); // 90도 돌만큼의 시간
    SetState(RoboState::ATTACK);
  }
}

void Controller::FrontWall() {
  if (detection[2] || detection[4] == 1) {
    SetSpeed(-1.0, 1.0);
    ThisThread::sleep_for(50); //반시계 방향으로 135도 회전
    SetState(RoboState::ATTACK);
  } else if (detection[5] == 1 || detection[3] == 1) {
    SetSpeed(1.0, -1.0);
    ThisThread::sleep_for(50); //시계 방향으로 135도 회전
    SetState(RoboState::ATTACK);
  } else {
    SetSpeed(0.5, -0.5); // 180도 회전
  }
}

void Controller::BehindWall() {
  if (detection[2]) {
    SetSpeed(-1.0, 1.0);
    ThisThread::sleep_for(50); //반시계 방향 90도 회전
    SetState(RoboState::ATTACK);
  }
  if (detection[3]) {
    SetSpeed(1.0, -1.0);
    ThisThread::sleep_for(50); //시계 방향 90도 회전
    SetState(RoboState::ATTACK);
  }
}
*/
void Controller::SetupImu_MPU9250() {
  uint8_t whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250); // Read WHO_AM_I register for MPU-9250
    // pc.printf("I AM 0x%x\t", whoami); pc.printf("I SHOULD BE 0x71\n\r");
  mpu9250.resetMPU9250(); // Reset registers to default in preparation for
    // device calibration
  mpu9250.MPU9250SelfTest(mpu9250.SelfTest); // Start by performing self test and reporting values
  mpu9250.calibrateMPU9250(mpu9250.gyroBias, mpu9250.accelBias);
  // Calibrate gyro and accelerometers, load biases in bias registers
  mpu9250.initMPU9250();
//   mpu9250.initAK8963(mpu9250.magCalibration);
  mpu9250.getAres(); // Get accelerometer sensitivity
  mpu9250.getGres(); // Get gyro sensitivity
//   mpu9250.getMres(); // Get magnetometer sensitivity
  t.start();
}
void Controller::ImuRefresh_MPU9250() {
    // If intPin goes high, all data registers have new data
    t.reset();
    if(mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {  // On interrupt, check if data ready interrupt
        //pc.printf("imu main     ");
        mpu9250.readAccelData(mpu9250.accelCount);  // Read the x/y/z adc values   
        // Now we'll calculate the accleration value into actual g's
        mpu9250.ax = (float)mpu9250.accelCount[0]*mpu9250.aRes - mpu9250.accelBias[0];  // get actual g value, this depends on scale being set
        mpu9250.ay = (float)mpu9250.accelCount[1]*mpu9250.aRes - mpu9250.accelBias[1];   
        mpu9250.az = (float)mpu9250.accelCount[2]*mpu9250.aRes - mpu9250.accelBias[2];  
        mpu9250.readGyroData(mpu9250.gyroCount);  // Read the x/y/z adc values
        // Calculate the gyro value into actual degrees per second
        mpu9250.gx = (float)mpu9250.gyroCount[0]*mpu9250.gRes - mpu9250.gyroBias[0];  // get actual gyro value, this depends on scale being set
        mpu9250.gy = (float)mpu9250.gyroCount[1]*mpu9250.gRes - mpu9250.gyroBias[1];  
        // mpu9250.gz = (float)mpu9250.g+yroCount[2]*mpu9250.gRes - mpu9250.gyroBias[2];   
        // Read the x/y/z adc values   
        // // Calculate the magnetometer values in milliGauss
        // // Include factory calibration per data sheet and user environmental corrections
        mpu9250.readMagData(mpu9250.magCount);
        mpu9250.mx = (float)mpu9250.magCount[0]*mpu9250.mRes*mpu9250.magCalibration[0] - mpu9250.magbias[0];  // get actual magnetometer value, this depends on scale being set
        mpu9250.my = (float)mpu9250.magCount[1]*mpu9250.mRes*mpu9250.magCalibration[1] - mpu9250.magbias[1];  
        // mpu9250.mz = (float)mpu9250.magCount[2]*mpu9250.mRes*mpu9250.magCalibration[2] - mpu9250.magbias[2];   
    }
    mpu9250.deltat = t.read_us()/1000000.0f;
    accel_angle_x = atan2(mpu9250.ay, sqrt(mpu9250.ax * mpu9250.ax + mpu9250.az * mpu9250.az)) * (180.0f / PI); 
    accel_angle_y = atan2(mpu9250.ax, sqrt(mpu9250.ay * mpu9250.ay + mpu9250.az * mpu9250.az)) * (180.0f / PI);
    // mag_angle_z  = atan2(mpu9250.my*cos(mpu9250.pitch*PI/180.0f) - mpu9250.mz*sin(mpu9250.pitch*PI/180.0f), mpu9250.mx*cos(mpu9250.roll*PI/180.0f)+mpu9250.my*sin(mpu9250.pitch*PI/180.0f)*sin(mpu9250.roll*PI/180.0f)+mpu9250.mz*cos(mpu9250.pitch*PI/180.0f)*sin(mpu9250.roll*PI/180.0f)) * (180.0f / PI);
    // mag_angle_z = atan2(mpu9250.my, mpu9250.mx) * (180.0f / PI);
    //gyro값 넣기
    gyro_angle_x = mpu9250.roll + mpu9250.gx * mpu9250.deltat;
    gyro_angle_y = mpu9250.pitch + mpu9250.gy * mpu9250.deltat;
    // gyro_angle_z = mpu9250.yaw + mpu9250.gz * mpu9250.deltat;
    //alpha를 이용한 보정(상보)
    mpu9250.roll = alpha_imu * gyro_angle_x + (1.0-alpha_imu) * accel_angle_x;
    mpu9250.pitch = alpha_imu * gyro_angle_y + (1.0-alpha_imu) * accel_angle_y;
    // mpu9250.yaw = 0.95 * gyro_angle_z + (1.0-0.95) * mag_angle_z;
}

void Controller::ImuDetect_MPU9250()  {
    if (mpu9250.az >= MinStableZAccel && mpu9250.az <= MaxStableZAccel) {
        if (SettleTimer.read_ms() == 0) {
            SettleTimer.start();
        } else if (SettleTimer.read_ms() >= SettlingTime) { isZAccelSettled = true; }
    } else {
        isZAccelSettled = false;
        SettleTimer.stop();
        SettleTimer.reset();
    }
    if(GetEnemyState() && mpu9250.pitch < -IMU_THRESHOLD && isZAccelSettled) {
        SetImuSafetyState(false);
        tilt_state = TiltState::FRONT;
    } else if(/*GetEnemyState() &&*/ (psd_val[0] < 9) && mpu9250.pitch < -IMU_THRESHOLD) {
        SetImuSafetyState(false);
        tilt_state = TiltState::FRONT_LEFT;
    } else if((/*GetEnemyState() && */ psd_val[2] < 9) && mpu9250.pitch <- IMU_THRESHOLD) {
        SetImuSafetyState(false);
        tilt_state = TiltState::FRONT_RIGHT;
    } else if(psd_val[0] < 9 && mpu9250.roll < -IMU_THRESHOLD) {
        if(Escape_Timer.read_ms() == 0) Escape_Timer.start();
        if(psd_val[0] < 9 && mpu9250.roll > IMU_THRESHOLD && Escape_Timer.read_ms() > ESCAPE_TIME) {
            SetImuSafetyState(false);
            tilt_state = TiltState::SIDE_LEFT;
            Escape_Timer.reset();
        }
    } else if(psd_val[3] <= 30 && mpu9250.roll > IMU_THRESHOLD) {
        if(Escape_Timer.read_ms() == 0) Escape_Timer.start();
        if(psd_val[3] <= 30 && mpu9250.roll < -IMU_THRESHOLD && Escape_Timer.read_ms() > ESCAPE_TIME) {
            SetImuSafetyState(false);
            tilt_state = TiltState::SIDE_LEFT;
            Escape_Timer.reset();
        }
    } else if(psd_val[2] < 9 && mpu9250.roll < -IMU_THRESHOLD) {
        if(Escape_Timer.read_ms() == 0) Escape_Timer.start();
        if(psd_val[2] < 15 && mpu9250.roll > IMU_THRESHOLD && Escape_Timer.read_ms() > ESCAPE_TIME) {
            SetImuSafetyState(false);
            tilt_state = TiltState::SIDE_RIGHT;
            Escape_Timer.reset();
        }
    } else if(psd_val[4] <= 30 && mpu9250.roll < -IMU_THRESHOLD) {
        if(Escape_Timer.read_ms() == 0) Escape_Timer.start();
        if(psd_val[4] <= 30 && mpu9250.roll > IMU_THRESHOLD && Escape_Timer.read_ms() > ESCAPE_TIME) {
            SetImuSafetyState(false);
            tilt_state = TiltState::SIDE_RIGHT;
            Escape_Timer.reset();
        }
    } else if (mpu9250.pitch < IMU_THRESHOLD){ SetImuSafetyState(true); tilt_state = TiltState::SAFE;}
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
//------------------------------Thread&NotController--------------------------------//
void ImuThread() {
    // controller.SetupImu_MPU9250();
    while(1) {
        controller.ImuRefresh_MPU9250();
        controller.ImuDetect_MPU9250();
        // pc.printf("%.2f,%.2f,",mpu9250.pitch,mpu9250.az);
        // controller.ImuViewer();
        ThisThread::sleep_for(20);
    }
}
void PsdThread() {
    while(1) {
        controller.PsdRefresh();
        // controller.IrRefresh();
        // pc.printf("%d, %d, %d, %d, %d, %d, %d, %d\r\n",controller.psd_val[0],controller.psd_val[1],controller.psd_val[2],controller.psd_val[3],controller.psd_val[4],controller.psd_val[5],controller.psd_val[6],controller.psd_val[7]);
        // pc.printf("%d, %d, %d, %d, %d, %d \r\n",irfl.read(), irfr.read(), irfc.read(), irbc.read(), irbl.read(), irbr.read());
        pc.printf("%d, %d, %d, %d, %d",controller.GetState(), controller.GetAttackState(), controller.GetImuSafetyState(), controller.GetIrSafetyState(), controller.GetWallSafetyState());
        controller.OrientViewer((int)controller.GetOrient());
        // controller.SetPosition();
        // controller.WallViewer();
        ThisThread::sleep_for(20); //임의
    }
}

void Starter() {
    controller.StartFlag = true;
}
//Interrupt to Idle
//Imu ThresHold -> Idle -> Escape
//Enemy Lost -> Idle -> Detect
//Enemy Detected -> Idle -> Attack

//---------------임시------------------//
void Controller::OrientViewer(int orient) {
    if(orient == 0) {
        pc.printf("FRONT\r\n");
    } else if(orient == 1) {
        pc.printf("TAN_LEFT\r\n");
    } else if(orient == 2) {
        pc.printf("TAN_RIGHT\r\n");
    } else if(orient == 3) {
        pc.printf("BACK\r\n");
    } else if(orient == 4) {
        pc.printf("FRONT_LEFT\r\n");
    } else if(orient == 5) {
        pc.printf("FRONT_RIGHT\r\n");
    } else if(orient == 6) {
        pc.printf("BACK_LEFT\r\n");
    } else if(orient == 7) {
        pc.printf("BACK_RIGHT\r\n");
    } else if(orient == 8) {
        pc.printf("SAFE\r\n");
    }
}

void Controller::WallViewer() {
    pc.printf("Distance : %d ",psd_val[1]);
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