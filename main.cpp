#include "mbed.h"
#include "controller.h"

int main()
{
    device.attach(&EnemyDetect, SerialBase::RxIrq);
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
        // pc.printf("%d, %.2f\r\n",controller.GetHD(), controller.GetCurrentYaw());
        // pc.printf("%d, %d, %d, %d\r\n",controller.psd_val[0], controller.psd_val[2], controller.psd_val[5], controller.psd_val[7]);
        // pc.printf("%d, %d, %d, %d\r\n",controller.psd_val[1], controller.psd_val[3], controller.psd_val[4], controller.psd_val[6]);
        // controller.WallViewer();
        // controller.OrientViewer();
        controller.Move(controller.GetSpeedL(),controller.GetSpeedR());
        ThisThread::sleep_until(controller.GetStartTime() + 10); //절대 시간으로 10ms 만큼 쉬기
    }
}
