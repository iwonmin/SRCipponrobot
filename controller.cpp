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
<<<<<<< Updated upstream
Controller::psd_side psdlf(PA_0,30,150,60,0); //PA_0 -> 핀 바꿔야함 !!!!
Controller::psd_side psdlc(PA_0,30,150,60,0);
Controller::psd_side psdlb(PA_0,30,150,60,0);
Controller::psd_side psdrf(PA_0,30,150,60,0);
Controller::psd_side psdrc(PA_0,30,150,60,0);
Controller::psd_side psdrb(PA_0,30,150,60,0);
=======
psd_side psdlf(PA_0,30,150,60,0); //PA_0 -> sheep gu ra pin !!
psd_side psdlc(PA_0,30,150,60,0);
psd_side psdlb(PA_0,30,150,60,0);
psd_side psdrf(PA_0,30,150,60,0);
psd_side psdrc(PA_0,30,150,60,0);
psd_side psdrb(PA_0,30,150,60,0);
>>>>>>> Stashed changes
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

void Controller:: EnemyDetect()
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

<<<<<<< Updated upstream
bool Controller::psd_side::refresh() {
=======
uint16_t ir_val[5] = {}; //순서: irfl, irfr, irc, irbl, irbr

bool psd_side::refresh() {
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
Controller::psd_side::psd_side(PinName pin_, uint16_t mincm, uint16_t maxcm, float slope, float base):GP2A_(pin_, mincm, maxcm, slope, base) {
=======
psd_side::psd_side(PinName pin_, uint16_t mincm, uint16_t maxcm, float slope, float base)
:GP2A_(pin_, mincm, maxcm, slope, base) {
>>>>>>> Stashed changes
    psd_side::prev_distance = 0;
    psd_side::now_distance = 0;
    psd_side::detection = 0;
    psd_side::filtered_distance = 0.f;
    psd_side::alpha = 0.9;
}
<<<<<<< Updated upstream
float Controller::psd_side::distance() {
=======
float psd_side::distance() {
>>>>>>> Stashed changes
    psd_side::now_distance = GP2A_.getDistance();
    psd_side::filtered_distance = psd_side::now_distance * psd_side::alpha + (1-psd_side::alpha) * psd_side::prev_distance;
    psd_side::prev_distance = psd_side::now_distance;
    return psd_side::filtered_distance;
}

<<<<<<< Updated upstream
void Controller::irs::refresh() {
=======
irs::irs():ir_val{}{
    
}
void irs::refresh() {
>>>>>>> Stashed changes
    ir_val[0] = irfl.read();
    ir_val[1] = irfr.read();
    ir_val[2] = irc.read();
    ir_val[3] = irbl.read();
    ir_val[4] = irbr.read();
    ir_total = ir_val[0] + ir_val[1] + ir_val[2] + ir_val[3] + ir_val[4];
    if(ir_total < 3) irs::ColorOrient();
}
<<<<<<< Updated upstream
void Controller::irs::ColorOrient() {
=======
void irs::ColorOrient() {
>>>>>>> Stashed changes
    //5개 인식되었을떄
    if (ir_total == 0) { //뭐하지??
    } else if (ir_total == 1) {
        if(ir_val[0] == 1) {
            irs::Orient = ColorOrient::BACK_RIGHT;
        } else if (ir_val[1] == 1) {
            irs::Orient = ColorOrient::BACK_LEFT;
        } else if (ir_val[3] == 1) {
            irs::Orient = ColorOrient::FRONT_RIGHT;
        } else if (ir_val[4] == 1) {
            irs::Orient = ColorOrient::FRONT_LEFT;
        } else {}
    } else if (ir_total == 2) {
        if(ir_val[0] + ir_val[1] + ir_val[2] == 0) {
            irs::Orient = ColorOrient::FRONT;
        } else if(ir_val[0] + ir_val[2] + ir_val[3] == 0) {
            irs::Orient = ColorOrient::TAN_LEFT;
        } else if(ir_val[2] + ir_val[3] + ir_val[4] == 0) {
            irs::Orient = ColorOrient::BACK;
        } else if(ir_val[1] + ir_val[2] + ir_val[4] == 0) {
            irs::Orient = ColorOrient::TAN_RIGHT;
        } else {}
    } else irs::Orient = ColorOrient::SAFE;
}
<<<<<<< Updated upstream
Controller::irs::Position Controller::irs::GetPosition() {
    return CurrentPos;
}
void Controller::irs::SetPosition() { //@@@@@@@@@@@@@@@@조건 너무 빈약, 고쳐야함. getDistance() 타이밍에 로봇 있을 때 거를 방안 찾아야함. //거리 함수 말고 전역 변수로 불러와야할 듯(controller)
=======
/*
void irs::enumfucker(int orient) {
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
    }
}
*/
irs::Position irs::GetPosition() {
    return CurrentPos;
}
void irs::SetPosition() { //@@@@@@@@@@@@@@@@조건 너무 빈약, 고쳐야함. getDistance() 타이밍에 로봇 있을 때 거를 방안 찾아야함. //거리 함수 말고 전역 변수로 불러와야할 듯(controller)
>>>>>>> Stashed changes
    //irs Colororient=>정확성 높음, 벽거리만 추가고려해서 바로 사용
    if(irs::Orient == irs::ColorOrient::TAN_LEFT && psdlc.distance() < CIRCLE_DISTANCE) {
        irs::CurrentPos = Position::ClosetoLeftWall;
        return;
        } else if(irs::Orient == irs::ColorOrient::TAN_RIGHT && psdrc.distance() < CIRCLE_DISTANCE) {
        irs::CurrentPos = Position::ClosetoRightWall;
        return;
        } else if(irs::Orient == irs::ColorOrient::FRONT_LEFT && psdlc.distance() < CIRCLE_DISTANCE) {
        irs::CurrentPos = Position::CriticalLeftWall;
        //뒤로, 오른쪽으로 이동하는 것 필요
        } else if(irs::Orient == irs::ColorOrient::BACK_LEFT && psdlc.distance() < CIRCLE_DISTANCE) {
        irs::CurrentPos = Position::CriticalLeftWall;
        //앞으로, 오른쪽으로 이동하는 것 필요
        } else if(irs::Orient == irs::ColorOrient::FRONT_RIGHT && psdrc.distance() < CIRCLE_DISTANCE) {
        irs::CurrentPos = Position::CriticalLeftWall;
        //뒤로, 왼쪽으로 이동하는 것 필요
        } else if(irs::Orient == irs::ColorOrient::BACK_RIGHT && psdrc.distance() < CIRCLE_DISTANCE) {
        irs::CurrentPos = Position::CriticalLeftWall;
        //앞으로, 왼쪽으로 이동하는 것 필요
        } else if(irs::Orient != irs::ColorOrient::SAFE && psdlc.distance() > 120 && psdlc.distance() < 150 && psdrc.distance() > 120 && psdrc.distance() < 150) {
        //일단 ir에 색은 감지되었지만 벽과의 거리가 생각보다 멀때 -> 중앙임
        irs::CurrentPos = Position::ClosetoCenter;
        } else {
            //ir 영역 아닐떄, psd만 사용(부정확)
            if(psdf.getDistance() < 30) {
                irs::CurrentPos = Position::WallFront;
            } else if (psdb.getDistance() < 30) {
                irs::CurrentPos = Position::WallBehind;
            } else irs::CurrentPos = Position::FartoCenter; // 색영역도 아닌데 안보임
        }
}
<<<<<<< Updated upstream
void Controller::irs::IR_Escape(enum ColorOrient orient) {
=======
void irs::IR_Escape(enum ColorOrient orient) {
>>>>>>> Stashed changes
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

<<<<<<< Updated upstream
Controller::EnemyFind::EnemyFind(irs::Position pos) { //생성자에 위치 넣고 클래스 바로 삭제하기 -> 무한반복
=======
EnemyFind::EnemyFind(irs::Position pos):controller() { //생성자에 위치 넣고 클래스 바로 삭제하기 -> 무한반복
>>>>>>> Stashed changes
    if(pos==irs::Position::ClosetoLeftWall) {
        LeftWallTrack();
    } else if(pos==irs::Position::ClosetoRightWall) {
        RightWallTrack();
    } else if(pos==irs::Position::CriticalLeftWall) {
        //살짝 빠져나오는거 필요
        LeftWallTrack();
    } else if(pos==irs::Position::CriticalRightWall) {
        //살짝 빠져나오는거 필요
        RightWallTrack();
    } else if(pos==irs::Position::ClosetoCenter || pos==irs::Position::FartoCenter) {
        //현재 거리값 대충 저장 후 빙글빙글 돌다가 갑자기 튀는 값 찾기
    } else if(pos==irs::Position::WallFront) {
        FrontWall();
    } else if(pos==irs::Position::WallBehind) {
        BehindWall();
    }
}

<<<<<<< Updated upstream
void Controller::EnemyFind::LeftWallTrack() { // 왼쪽에 벽, psdlf, psdlc, psdlb 로 거리 따고 right로 추적
=======
void EnemyFind::LeftWallTrack() { // 왼쪽에 벽, psdlf, psdlc, psdlb 로 거리 따고 right로 추적
>>>>>>> Stashed changes
    uint16_t avg_distance = (psdlf.distance() + psdlc.distance() + psdlb.distance())/3;// 나중에 제어 주기로 인해 새로고침된 전역변수로 바꾸기
    SetSpeed(0.5,0.5);
    if(avg_distance > WALL_DISTANCE+10) {
        SetSpeed(0.1,0.5);
        ThisThread::sleep_for(50);
    } else if(avg_distance < WALL_DISTANCE-10) {
        SetSpeed(0.5,0.1);
        ThisThread::sleep_for(50);
    }
    if(psdrb.detection == 1 || psdrc.detection == 1 || psdrf.detection == 1) {
        SetSpeed(1.0,-1.0);
        ThisThread::sleep_for(50); //90도 돌만큼의 시간
        SetState(RoboState::ATTACK);
    }
}
<<<<<<< Updated upstream
void Controller::EnemyFind::RightWallTrack() { // 왼쪽에 벽, psdlf, psdlc, psdlb 로 거리 따고 right로 추적
=======
void EnemyFind::RightWallTrack() { // 왼쪽에 벽, psdlf, psdlc, psdlb 로 거리 따고 right로 추적
>>>>>>> Stashed changes
    uint16_t avg_distance = (psdrf.distance() + psdrc.distance() + psdrb.distance())/3;// 나중에 제어 주기로 인해 새로고침된 전역변수로 바꾸기
    SetSpeed(0.5,0.5);
    if(avg_distance > WALL_DISTANCE+10) {
        SetSpeed(0.5,0.1);
        ThisThread::sleep_for(50);
    } else if(avg_distance < WALL_DISTANCE-10) {
        SetSpeed(0.1,0.5);
        ThisThread::sleep_for(50);
    }
    if(psdlb.detection == 1 || psdlc.detection == 1 || psdlf.detection == 1) {
        SetSpeed(-1.0,1.0);
        ThisThread::sleep_for(50); //90도 돌만큼의 시간
        SetState(RoboState::ATTACK);
    }
    
}

<<<<<<< Updated upstream
void Controller::EnemyFind::CenterSpin() {
=======
void EnemyFind::CenterSpin() {
>>>>>>> Stashed changes
    SetSpeed(0.5,-0.5); //빙글빙글
    if(psdlb.detection == 1 || psdlc.detection == 1 || psdlf.detection == 1 || psdrb.detection == 1 || psdrc.detection == 1 || psdrf.detection == 1) {
        SetSpeed(0,0);
        ThisThread::sleep_for(50); //90도 돌만큼의 시간
        SetState(RoboState::ATTACK);
    }
}

<<<<<<< Updated upstream
void Controller::EnemyFind::FrontWall() {
=======
void EnemyFind::FrontWall() {
>>>>>>> Stashed changes
    if(psdlb.detection == 1 || psdlc.detection == 1 || psdlf.detection == 1) {
        SetSpeed(-1.0,1.0);
        ThisThread::sleep_for(50); //90도 돌만큼의 시간
        SetState(RoboState::ATTACK);
    } else if(psdrb.detection == 1 || psdrc.detection == 1 || psdrf.detection == 1) {
        SetSpeed(1.0, -1.0);
        ThisThread::sleep_for(50); //90도 돌만큼의 시간
        SetState(RoboState::ATTACK);
    } else {
        SetSpeed(0.5,-0.5); // 180도 회전
    }
}

<<<<<<< Updated upstream
void Controller::EnemyFind::BehindWall() {
=======
void EnemyFind::BehindWall() {
>>>>>>> Stashed changes
    if(psdlb.detection == 1 || psdlc.detection == 1 || psdlf.detection == 1) {
        SetSpeed(-1.0,1.0);
        ThisThread::sleep_for(50); //90도 돌만큼의 시간
        SetState(RoboState::ATTACK);
    }
    if(psdrb.detection == 1 || psdrc.detection == 1 || psdrf.detection == 1) {
        SetSpeed(1.0, -1.0);
        ThisThread::sleep_for(50); //90도 돌만큼의 시간
        SetState(RoboState::ATTACK);
    }
}
        // if (ch == '-') {
        //     isNegative = true; 
        //     if (isNegative) {
        //     //pc.printf("-"); // 부호 정보 출력
        // }// 부호가 음수인 경우 표시
        // } else if (ch == '.') {
        //     decimalPointSeen = true; // 소수점 발견 시, 이후의 숫자는 무시
        // } else if (ch >= '0' && ch <= '9' && !decimalPointSeen) {
        //     // 소수점 전의 숫자만 PC로 전송
        //     pc.putc(ch);
        // } else if (ch == '/') {            
        //     // '/' 문자를 만나면 새로운 줄로 이동하기 위해 줄바꿈 문자 출력
        //     //pc.printf("\r\n");
        //     decimalPointSeen = false; // 소수점 발견 여부 초기화
        //     isNegative = false; // 부호 정보 초기화
        //     //continue; // '/' 문자는 무시하고 다음 데이터로 이동
        // }
        // // Raspberry Pi로 받은 데이터를 다시 보냄 (필요한 경우)
        // device.putc(ch);
        // float hd = (int)(ch);
        // pc.printf("%f\r\n", hd);
        // SetHD(hd);