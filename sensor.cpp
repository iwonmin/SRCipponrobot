#include "sensor.h"

#include <cstdint>
#pragma region variables
//just psd
GP2A psdf(PA_0,7,80,0.246,-0.297); //그냥 거리감지
GP2A psdb(PA_0,7,80,0.246,-0.297);
//detector psd
psd_side psdlf(PA_0,30,150,60,0); //PA_0 -> sheep gu ra pin !!
psd_side psdlc(PA_0,30,150,60,0);
psd_side psdlb(PA_0,30,150,60,0);
psd_side psdrf(PA_0,30,150,60,0);
psd_side psdrc(PA_0,30,150,60,0);
psd_side psdrb(PA_0,30,150,60,0);
//ir pin
DigitalIn irfl(PA_0);
DigitalIn irfr(PA_0);
DigitalIn irc(PA_0);
DigitalIn irbl(PA_0);
DigitalIn irbr(PA_0);
Serial pc(USBTX, USBRX, 115200);
#pragma endregion variables

uint16_t ir_val[5] = {}; //순서: irfl, irfr, irc, irbl, irbr

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
    psd_side::filtered_distance = 0.f;
    psd_side::alpha = 0.9;
}
float psd_side::distance() {
    psd_side::now_distance = GP2A_.getDistance();
    psd_side::filtered_distance = psd_side::now_distance * psd_side::alpha + (1-psd_side::alpha) * psd_side::prev_distance;
    psd_side::prev_distance = psd_side::now_distance;
    return psd_side::filtered_distance;
}

irs::irs():ir_val{}{
    
}
void irs::refresh() {
    ir_val[0] = irfl.read();
    ir_val[1] = irfr.read();
    ir_val[2] = irc.read();
    ir_val[3] = irbl.read();
    ir_val[4] = irbr.read();
    ir_total = ir_val[0] + ir_val[1] + ir_val[2] + ir_val[3] + ir_val[4];
    if(ir_total < 3) irs::ColorOrient();
}
void irs::ColorOrient() {
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
void irs::IR_Escape(enum ColorOrient orient) {
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

EnemyFind::EnemyFind(irs::Position pos):controller() { //생성자에 위치 넣고 클래스 바로 삭제하기 -> 무한반복
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

void EnemyFind::LeftWallTrack() { // 왼쪽에 벽, psdlf, psdlc, psdlb 로 거리 따고 right로 추적
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
void EnemyFind::RightWallTrack() { // 왼쪽에 벽, psdlf, psdlc, psdlb 로 거리 따고 right로 추적
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

void EnemyFind::CenterSpin() {
    SetSpeed(0.5,-0.5); //빙글빙글
    if(psdlb.detection == 1 || psdlc.detection == 1 || psdlf.detection == 1 || psdrb.detection == 1 || psdrc.detection == 1 || psdrf.detection == 1) {
        SetSpeed(0,0);
        ThisThread::sleep_for(50); //90도 돌만큼의 시간
        SetState(RoboState::ATTACK);
    }
}

void EnemyFind::FrontWall() {
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

void EnemyFind::BehindWall() {
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