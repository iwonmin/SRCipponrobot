#include "mbed.h"
#include "MPU9250.h"

Timer t;
float sum = 0;
uint32_t sumCount = 0;
char buffer[4] = {};
float tmp_angle_x, tmp_angle_y, tmp_angle_z;
float filltered_angle_x, filltered_angle_y, filltered_angle_z;
float tmp_acc_x, tmp_acc_y, tmp_acc_z;
float filtered_acc_x = 0, filtered_acc_y, filtered_acc_z;
float alpha = 0.90; 
float aalpha = 0.9;
int imu_count = 0;
MPU9250 mpu9250(D14,D15);
RawSerial pc(USBTX, USBRX, 115200);
void setup_mpu9250();
void imu_main();

void sendFloatAsBinary(float value) {
    // float 값을 바이트 배열로 변환
    char *byteArray = (char *)&value;
    
    // 변환된 바이트 배열을 시리얼 포트로 전송
    for (size_t i = 0; i < sizeof(float); i++) {
        pc.putc(byteArray[i]);
    }
}
int main() {
    setup_mpu9250();

    float ax = 0.;
    float ay = 0.;
    float angx = 0.;
    float angy = 0.;
    while(1) {
        imu_main();
        ax = mpu9250.ax;
        ay = filtered_acc_y;
        angx = filltered_angle_x;
        angy = filltered_angle_y;
        sendFloatAsBinary(ay);
        ThisThread::sleep_for(50);
        
    }
}

void setup_mpu9250(){
    uint8_t whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
    // pc.printf("I AM 0x%x\t", whoami); pc.printf("I SHOULD BE 0x71\n\r");

    mpu9250.resetMPU9250(); // Reset registers to default in preparation for device calibration
    mpu9250.MPU9250SelfTest(mpu9250.SelfTest); // Start by performing self test and reporting values 
    
    mpu9250.calibrateMPU9250(mpu9250.gyroBias, mpu9250.accelBias); 
    // Calibrate gyro and accelerometers, load biases in bias registers  

    mpu9250.initMPU9250();
    // -6050확인용 mpu9250.initAK8963(mpu9250.magCalibration);

    mpu9250.getAres(); // Get accelerometer sensitivity
    mpu9250.getGres(); // Get gyro sensitivity
    // -6050확인용 mpu9250.getMres(); // Get magnetometer sensitivity

}

void imu_main(){
    //while(true){
        // If intPin goes high, all data registers have new data
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
            mpu9250.gz = (float)mpu9250.gyroCount[2]*mpu9250.gRes - mpu9250.gyroBias[2];   
  
            // -6050확인용 mpu9250.readMagData(mpu9250.magCount);  // Read the x/y/z adc values   
            // // Calculate the magnetometer values in milliGauss
            // // Include factory calibration per data sheet and user environmental corrections
            // mpu9250.mx = (float)mpu9250.magCount[0]*mpu9250.mRes*mpu9250.magCalibration[0] - mpu9250.magbias[0];  // get actual magnetometer value, this depends on scale being set
            // mpu9250.my = (float)mpu9250.magCount[1]*mpu9250.mRes*mpu9250.magCalibration[1] - mpu9250.magbias[1];  
            // mpu9250.mz = (float)mpu9250.magCount[2]*mpu9250.mRes*mpu9250.magCalibration[2] - mpu9250.magbias[2];   
        }
        //pc.printf("%c\n",mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS));

        //pc.printf("no if\n");
        mpu9250.Now = t.read_us();
        mpu9250.deltat = (float)((mpu9250.Now - mpu9250.lastUpdate)/1000000.0f) ; 
        // set integration time by time elapsed since last filter update
        mpu9250.lastUpdate = mpu9250.Now;
    
        sum += mpu9250.deltat;
        sumCount++;
    
        // Serial print and/or display at 0.5 s rate independent of data rates
        //mpu9250.delt_t = t.read_ms() - mpu9250.count;

        mpu9250.roll = atan2(mpu9250.ay, sqrt(mpu9250.ax * mpu9250.ax + mpu9250.az * mpu9250.az)) * (180.0 / PI);
        mpu9250.pitch = atan2(-mpu9250.ax, sqrt(mpu9250.ay * mpu9250.ay + mpu9250.az * mpu9250.az)) * (180.0 / PI);

        

        //temp filter - gyro적분
        // tmp_angle_x = filltered_angle_x + mpu9250.gx * mpu9250.deltat;
        // tmp_angle_y = filltered_angle_y + mpu9250.gy * mpu9250.deltat;
        // tmp_angle_z = filltered_angle_z + mpu9250.gz * mpu9250.deltat;

        tmp_angle_x = filltered_angle_x + mpu9250.gx * 0.02;
        tmp_angle_y = filltered_angle_y + mpu9250.gy * 0.02;
        tmp_angle_z = filltered_angle_z + mpu9250.gz * 0.02;

        //alpha를 이용한 보정(상보)
        filltered_angle_x = alpha * tmp_angle_x + (1.0-alpha) * mpu9250.roll;
        filltered_angle_y = alpha * tmp_angle_y + (1.0-alpha) * mpu9250.pitch;
        // filltered_angle_z = tmp_angle_z;
        
        //LPF for accelerometer
        tmp_acc_x = mpu9250.ax;
        filtered_acc_x = filtered_acc_x * aalpha + (1.0-aalpha) * tmp_acc_x;
        tmp_acc_y = mpu9250.ay;
        filtered_acc_y = filtered_acc_y * aalpha + (1.0-aalpha) * tmp_acc_y;
        mpu9250.count = t.read_ms(); 

        if(mpu9250.count > 1<<21) {
            t.start(); // start the timer over again if ~30 minutes has passed
            mpu9250.count = 0;
            mpu9250.deltat= 0;
            //t.reset();
            mpu9250.lastUpdate = t.read_us();
        }
        sum = 0;
        sumCount = 0; 
    //}
    

}




/*
void imu_read(){
    // imu_th.set_priority(osPriorityNormal);
    t.start();
    setup_mpu9250();
    while (true){
        // ThisThread::sleep_for(7);
        Nowi_time = rtos::Kernel::get_ms_count();
        //pc.printf("%11u ms\n", -(chek_time - Nowi_time));

        imu_main();

        //chek_time = Nowi_time;
        Worki_time = rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count() + (imu_time-(Worki_time - Nowi_time)));
    }
}
*/