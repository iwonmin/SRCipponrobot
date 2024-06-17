#include "controller.h"
char buffer[8] = "";
#pragma region variables
DigitalOut DirL(PC_7);
DigitalOut DirR(PB_6);
PwmOut PwmL(PB_4);
PwmOut PwmR(PB_5);
GP2A psdf(PA_0,7,80,0.246,-0.297); //그냥 거리감지
GP2A psdb(PA_0,7,80,0.246,-0.297);
//detector psd
GP2A psdlf(PA_0,30,150,60,0); //PA_0 -> 핀 바꿔야함 !!!!
GP2A psdlc(PA_0,30,150,60,0);
GP2A psdlb(PA_0,30,150,60,0);
GP2A psdrf(PA_0,30,150,60,0);
GP2A psdrc(PA_0,30,150,60,0);
GP2A psdrb(PA_0,30,150,60,0);
//ir pin
DigitalIn irfl(PA_0);
DigitalIn irfr(PA_0);
DigitalIn irc(PA_0);
DigitalIn irbl(PA_0);
DigitalIn irbr(PA_0);
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

DigitalOut led1(LED1);

Controller::Controller()
{
    SetState(RoboState::START);
    Controller::alpha = 0.9;
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
    float speedL_i = GetSpeedL();
    float speedR_i = GetSpeedR();
    int interval_L= (sL-speedL_i)/0.1f;
    int interval_R= (sR-speedR_i)/0.1f;
    for(int i = 0; i<=abs(interval_L); i++){
        if(interval_L>=0){
            speedL = speedL_i + 0.1*i;
        }else{
            speedL = speedL_i - 0.1*i;
        } 
    }
    for(int i = 0; i<=abs(interval_R); i++){
        if(interval_R>=0){
            speedR = speedR_i + 0.1*i;
        }else{
            speedR = speedR_i - 0.1*i;
        } 
    }
    // speedL = sL;
    // speedR = sR;
};

bool Controller::GetEnemyState()
{
    return enemy;
};

void Controller::SetEnemyState(bool enemyState)
{
    enemy = enemyState;
};

int Controller::GetHD()
{
    return enemy_horizontal_distance;
};

void Controller::SetHD(int HD)
{
    enemy_horizontal_distance = HD;
}

void Controller::Start()
{
    PwmL.period_us(66);
    PwmR.period_us(66);
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
    if(isSafe){
        if(GetEnemyState())
        {            
            SetSpeed(0);
            SetState(RoboState::ATTACK);
        }else if(!GetEnemyState() && GetHD()>0){
            led1=0;
            SetSpeed(-0.5,0.5);
        }else if(!GetEnemyState() && GetHD()<0){
            led1=0;
            SetSpeed(0.5,-0.5);
        }
    }else{
        SetState(RoboState::IDLE);
    }
};

void Controller::Attack()
{
    if(isSafe){
        led1 = 1;
        SetSpeed(MAXSPEED);
        if(!GetEnemyState()){
            SetState(RoboState::IDLE);
        }
    }else
    {
        SetState(RoboState::IDLE);
    }
};

void Controller::Escape()
{
    SetSpeed(ESCAPESPEED);
    if(isSafe){
        SetState(RoboState::IDLE);
    }
};

void Controller::Move(float sL, float sR){
    if(sL<0) DirL = 0;
    else DirL = 1;

    if(sR<0) DirR = 0;
    else DirR = 1;

    PwmL = abs(sL);
    PwmR = abs(sR);
};

void Controller::EnemyDetect()
{  
    if (device.readable()) {
        char receivedChar = device.getc();
        if(receivedChar=='*'){
            SetEnemyState(false);
        }else{
            SetEnemyState(true);
      }    
        if(receivedChar=='/'){
            distanceBuffer[bufferIndex] = '\0';
            SetHD(atoi(distanceBuffer));
            pc.printf("Received Distance: %d\n", GetHD());
            bufferIndex = 0; // 버퍼 초기화
        }else{
            distanceBuffer[bufferIndex] = receivedChar;
            bufferIndex++;
            if (bufferIndex >= sizeof(distanceBuffer) - 1) {
                bufferIndex = 0; // 버퍼가 가득 찬 경우 초기화
            }
        }
   }
}

void Controller::PsdRefresh(GP2A GP2A_, uint8_t i) {
        Controller::now_distance[i] = GP2A_.getDistance();
        uint16_t difference = fabs(Controller::now_distance - Controller::prev_distance);
        if(difference > PSD_THRESHOLD) {
            Controller::detection[i] = 1;
        } else {
            Controller::detection[i] = 0;
            }
        Controller::prev_distance[i] = Controller::now_distance[i];
    }

float Controller::PsdDistance(GP2A GP2A_, uint8_t i) {
    Controller::now_distance[i] = GP2A_.getDistance();
    Controller::filtered_distance[i] = Controller::now_distance[i] * Controller::alpha + (1-Controller::alpha) * Controller::prev_distance[i];
    Controller::prev_distance[i] = Controller::now_distance[i];
    return Controller::filtered_distance[i];
}
//0 psdlf
//1 psdrf
//2 psdlc
//3 psdrc
//4 psdlb
//5 psdrb
//6 psdf
//7 psdb
void Controller::IrRefresh() {
    ir_val[0] = irfl.read();
    ir_val[1] = irfr.read();
    ir_val[2] = irc.read();
    ir_val[3] = irbl.read();
    ir_val[4] = irbr.read();
    ir_total = ir_val[0] + ir_val[1] + ir_val[2] + ir_val[3] + ir_val[4];
    if(ir_total < 3) Controller::ColorOrient();
}
void Controller::ColorOrient() {
    //5개 인식되었을떄
    if (ir_total == 0) { //뭐하지??
    } else if (ir_total == 1) {
        if(ir_val[0] == 1) {
            Controller::Orient = ColorOrient::BACK_RIGHT;
        } else if (ir_val[1] == 1) {
            Controller::Orient = ColorOrient::BACK_LEFT;
        } else if (ir_val[3] == 1) {
            Controller::Orient = ColorOrient::FRONT_RIGHT;
        } else if (ir_val[4] == 1) {
            Controller::Orient = ColorOrient::FRONT_LEFT;
        } else {}
    } else if (ir_total == 2) {
        if(ir_val[0] + ir_val[1] + ir_val[2] == 0) {
            Controller::Orient = ColorOrient::FRONT;
        } else if(ir_val[0] + ir_val[2] + ir_val[3] == 0) {
            Controller::Orient = ColorOrient::TAN_LEFT;
        } else if(ir_val[2] + ir_val[3] + ir_val[4] == 0) {
            Controller::Orient = ColorOrient::BACK;
        } else if(ir_val[1] + ir_val[2] + ir_val[4] == 0) {
            Controller::Orient = ColorOrient::TAN_RIGHT;
        } else {}
    } else Controller::Orient = ColorOrient::SAFE;
}
Controller::Position Controller::GetPosition() {
    return CurrentPos;
}
void Controller::SetPosition() { //@@@@@@@@@@@@@@@@조건 너무 빈약, 고쳐야함. getDistance() 타이밍에 로봇 있을 때 거를 방안 찾아야함. //거리 함수 말고 전역 변수로 불러와야할 듯(controller)
    //irs Colororient=>정확성 높음, 벽거리만 추가고려해서 바로 사용
    if(Controller::Orient == Controller::ColorOrient::TAN_LEFT && Controller::filtered_distance[2] < CIRCLE_DISTANCE) {
        Controller::CurrentPos = Position::ClosetoLeftWall;
        return;
        } else if(Controller::Orient == Controller::ColorOrient::TAN_RIGHT && Controller::filtered_distance[3] < CIRCLE_DISTANCE) {
        Controller::CurrentPos = Position::ClosetoRightWall;
        return;
        } else if(Controller::Orient == Controller::ColorOrient::FRONT_LEFT && Controller::filtered_distance[2] < CIRCLE_DISTANCE) {
        Controller::CurrentPos = Position::CriticalLeftWall;
        //뒤로, 오른쪽으로 이동하는 것 필요
        } else if(Controller::Orient == Controller::ColorOrient::BACK_LEFT && Controller::filtered_distance[2] < CIRCLE_DISTANCE) {
        Controller::CurrentPos = Position::CriticalLeftWall;
        //앞으로, 오른쪽으로 이동하는 것 필요
        } else if(Controller::Orient == Controller::ColorOrient::FRONT_RIGHT && Controller::filtered_distance[3] < CIRCLE_DISTANCE) {
        Controller::CurrentPos = Position::CriticalLeftWall;
        //뒤로, 왼쪽으로 이동하는 것 필요
        } else if(Controller::Orient == Controller::ColorOrient::BACK_RIGHT && Controller::filtered_distance[3] < CIRCLE_DISTANCE) {
        Controller::CurrentPos = Position::CriticalLeftWall;
        //앞으로, 왼쪽으로 이동하는 것 필요
        } else if(Controller::Orient != Controller::ColorOrient::SAFE && Controller::filtered_distance[2] > 220 && Controller::filtered_distance[2] < 250 && Controller::filtered_distance[3] > 120 && Controller::filtered_distance[3] < 150) {
        //일단 ir에 색은 감지되었지만 벽과의 거리가 생각보다 멀때 -> 중앙임
        Controller::CurrentPos = Position::ClosetoCenter;
        } else {
            //ir 영역 아닐떄, psd만 사용(부정확)
            if(psdf.getDistance() < 30) {
                Controller::CurrentPos = Position::WallFront;
            } else if (psdb.getDistance() < 30) {
                Controller::CurrentPos = Position::WallBehind;
            } else Controller::CurrentPos = Position::FartoCenter; // 색영역도 아닌데 안보임
        }
}
void Controller::IrEscape(enum ColorOrient orient) {
    if(orient==ColorOrient::SAFE) {
        return;
    } else if(orient==ColorOrient::FRONT) {
        
    } else if(orient==ColorOrient::TAN_LEFT) {
        
    } else if(orient==ColorOrient::TAN_RIGHT) {

    } else if(orient==ColorOrient::BACK) {

    } else if(orient==ColorOrient::FRONT_LEFT) {

    } else if(orient==ColorOrient::FRONT_RIGHT) {

    } else if(orient==ColorOrient::BACK_LEFT) {
        
    } else if(orient==ColorOrient::BACK_LEFT) {

    } else return;
}

void Controller::EnemyFind(Controller::Position pos) { 
    if(pos==Position::ClosetoLeftWall) {
        LeftWallTrack();
    } else if(pos==Position::ClosetoRightWall) {
        RightWallTrack();
    } else if(pos==Position::CriticalLeftWall) {
        //살짝 빠져나오는거 필요
        LeftWallTrack();
    } else if(pos==Position::CriticalRightWall) {
        //살짝 빠져나오는거 필요
        RightWallTrack();
    } else if(pos==Position::ClosetoCenter || pos==Position::FartoCenter) {
        //현재 거리값 대충 저장 후 빙글빙글 돌다가 갑자기 튀는 값 찾기
    } else if(pos==Position::WallFront) {
        FrontWall();
    } else if(pos==Position::WallBehind) {
        BehindWall();
    }
}

void Controller::LeftWallTrack() { // 왼쪽에 벽, psdlf, psdlc, psdlb 로 거리 따고 right로 추적
    uint16_t avg_distance = (Controller::filtered_distance[0] + Controller::filtered_distance[2] + Controller::filtered_distance[4])/3;// 나중에 제어 주기로 인해 새로고침된 전역변수로 바꾸기
    SetSpeed(0.5,0.5);
    if(avg_distance > WALL_DISTANCE+10) {
        SetSpeed(0.1,0.5);
        ThisThread::sleep_for(50);
    } else if(avg_distance < WALL_DISTANCE-10) {
        SetSpeed(0.5,0.1);
        ThisThread::sleep_for(50);
    }
    if(Controller::detection[5] == 1 || Controller::detection[3] == 1 || Controller::detection[1] == 1) {
        SetSpeed(1.0,-1.0);
        ThisThread::sleep_for(50); //90도 돌만큼의 시간
        SetState(RoboState::ATTACK);
    }
}
void Controller::RightWallTrack() { // 왼쪽에 벽, psdlf, psdlc, psdlb 로 거리 따고 right로 추적
    uint16_t avg_distance = (Controller::filtered_distance[1] + Controller::filtered_distance[3] + Controller::filtered_distance[5])/3;// 나중에 제어 주기로 인해 새로고침된 전역변수로 바꾸기
    SetSpeed(0.5,0.5);
    if(avg_distance > WALL_DISTANCE+10) {
        SetSpeed(0.5,0.1);
        ThisThread::sleep_for(50);
    } else if(avg_distance < WALL_DISTANCE-10) {
        SetSpeed(0.1,0.5);
        ThisThread::sleep_for(50);
    }
    if(Controller::detection[0] == 1 || Controller::detection[2] || Controller::detection[4] == 1) {
        SetSpeed(-1.0,1.0);
        ThisThread::sleep_for(50); //90도 돌만큼의 시간
        SetState(RoboState::ATTACK);
    }
    
}

void Controller::CenterSpin() {
    SetSpeed(0.5,-0.5); //빙글빙글
    if(Controller::detection[0] == 1 || Controller::detection[2] == 1 || Controller::detection[4] == 1 || Controller::detection[1] == 1 || Controller::detection[3] == 1 || Controller::detection[5] == 1) {
        SetSpeed(0,0);
        ThisThread::sleep_for(50); //90도 돌만큼의 시간
        SetState(RoboState::ATTACK);
    }
}

void Controller::FrontWall() {
    if(Controller::detection[0] == 1 || Controller::detection[2] || Controller::detection[4] == 1) {
        SetSpeed(-1.0,1.0);
        ThisThread::sleep_for(50); //90도 돌만큼의 시간
        SetState(RoboState::ATTACK);
    } else if(Controller::detection[5] == 1 || Controller::detection[3] == 1 || Controller::detection[1] == 1) {
        SetSpeed(1.0, -1.0);
        ThisThread::sleep_for(50); //90도 돌만큼의 시간
        SetState(RoboState::ATTACK);
    } else {
        SetSpeed(0.5,-0.5); // 180도 회전
    }
}

void Controller::BehindWall() {
    if(Controller::detection[0] == 1 || Controller::detection[2] || Controller::detection[4] == 1) {
        SetSpeed(-1.0,1.0);
        ThisThread::sleep_for(50); //90도 돌만큼의 시간
        SetState(RoboState::ATTACK);
    }
    if(Controller::detection[5] == 1 || Controller::detection[3] == 1 || Controller::detection[1] == 1) {
        SetSpeed(1.0, -1.0);
        ThisThread::sleep_for(50); //90도 돌만큼의 시간
        SetState(RoboState::ATTACK);
    }
}

