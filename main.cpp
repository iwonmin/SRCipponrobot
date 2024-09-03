#include "mbed.h"
#include "controller.h"

int main()
{
    device.attach(&sibal, SerialBase::RxIrq);
 while (true) {
        controller.CheckStartTime();
        switch(controller.GetState())
        {
            case Controller::RoboState::START:
            controller.Start();
            break;
            case Controller::RoboState::IDLE:
            controller.Idle();
            break;
            case Controller::RoboState::DETECT:
            controller.Detect();
            break;
            case Controller::RoboState::ATTACK:
            controller.Attack();
            break;
            case Controller::RoboState::ESCAPE:
            controller.Escape();
            break;
            case Controller::RoboState::YELLOW:
            controller.Yellow();
        }
        // pc.printf("EnemyDistance : %d, Yaw : %.1f, CenterPsd : %d, State : %d ImuState : ",controller.GetHD(),controller.GetCurrentYaw(),controller.psd_val[1], (int)controller.GetState());
        // controller.ImuViewer();
        pc.printf("%d, %.2f, %d\r\n",controller.GetHD(),controller.GetCurrentYaw(),controller.GetEnemyState());
        controller.Move(controller.GetSpeedL(),controller.GetSpeedR());
        ThisThread::sleep_until(controller.GetStartTime() + 10); //절대 시간으로 10ms 만큼 쉬기
    }
}
