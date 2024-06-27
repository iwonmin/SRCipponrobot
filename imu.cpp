#include "mbed.h"
#include "MPU9250/MPU9250.h"
const float alpha_imu = 0.9f;

float gyro_angle_x, gyro_angle_y, gyro_angle_z;

float accel_angle_x, accel_angle_y, mag_angle_z;

int imu_count = 0;

Timer t;

float sum = 0;

uint32_t sumCount = 0;

uint64_t Now_time,Work_time,Nowm_time,Workm_time,Nowi_time,Worki_time;

uint16_t chek_time;
void SetupImu();
void ImuRefresh();
void ImuThread();
MPU9250 mpu9250(D14,D15);
Serial pc(USBTX, USBRX, 9600);
int main() {
    SetupImu();
    while (true){
        ImuRefresh();
        // pc.printf("%.2f,%.6f\r\n",mpu9250.gx,mpu9250.deltat);
        //pc.printf("roll : %.1f pitch : %.1f yaw : %.1f\r\n",mpu9250.roll,mpu9250.pitch,mpu9250.yaw);
        pc.printf("%.3f %.3f %.3f %.3f %.3f\r\n",gyro_angle_z,mpu9250.mx,mpu9250.my,mpu9250.mz,mpu9250.deltat);
        ThisThread::sleep_for(100);
    }
}


void SetupImu() {
    uint8_t whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250

    mpu9250.resetMPU9250(); // Reset registers to default in preparation for device calibration
    mpu9250.MPU9250SelfTest(mpu9250.SelfTest); // Start by performing self test and reporting values 
    
    mpu9250.calibrateMPU9250(mpu9250.gyroBias, mpu9250.accelBias); 
    // Calibrate gyro and accelerometers, load biases in bias registers  

    mpu9250.initMPU9250();
    mpu9250.initAK8963(mpu9250.magCalibration);

    mpu9250.getAres(); // Get accelerometer sensitivity
    mpu9250.getGres(); // Get gyro sensitivity
    mpu9250.getMres(); // Get magnetometer sensitivity
    t.start();
}

void ImuRefresh() {
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
        mpu9250.gz = (float)mpu9250.gyroCount[2]*mpu9250.gRes - mpu9250.gyroBias[2];   
        // Read the x/y/z adc values   
        // // Calculate the magnetometer values in milliGauss
        // // Include factory calibration per data sheet and user environmental corrections
        mpu9250.readMagData(mpu9250.magCount);
        mpu9250.mx = (float)mpu9250.magCount[0]*mpu9250.mRes*mpu9250.magCalibration[0] - mpu9250.magbias[0];  // get actual magnetometer value, this depends on scale being set
        mpu9250.my = (float)mpu9250.magCount[1]*mpu9250.mRes*mpu9250.magCalibration[1] - mpu9250.magbias[1];  
        mpu9250.mz = (float)mpu9250.magCount[2]*mpu9250.mRes*mpu9250.magCalibration[2] - mpu9250.magbias[2];   
    }
    mpu9250.deltat = t.read_us()/1000000.0f;
    accel_angle_x = atan2(mpu9250.ay, sqrt(mpu9250.ax * mpu9250.ax + mpu9250.az * mpu9250.az)) * (180.0f / PI); 
    accel_angle_y = atan2(mpu9250.ax, sqrt(mpu9250.ay * mpu9250.ay + mpu9250.az * mpu9250.az)) * (180.0f / PI);
    mag_angle_z  = atan2(mpu9250.my*cos(mpu9250.pitch*PI/180.0f) - mpu9250.mz*sin(mpu9250.pitch*PI/180.0f), mpu9250.mx*cos(mpu9250.roll*PI/180.0f)+mpu9250.my*sin(mpu9250.pitch*PI/180.0f)*sin(mpu9250.roll*PI/180.0f)+mpu9250.mz*cos(mpu9250.pitch*PI/180.0f)*sin(mpu9250.roll*PI/180.0f)) * (180.0f / PI);
    //gyro값 넣기
    gyro_angle_x = mpu9250.roll + mpu9250.gx * mpu9250.deltat;
    gyro_angle_y = mpu9250.pitch + mpu9250.gy * mpu9250.deltat;
    gyro_angle_z = mpu9250.yaw + mpu9250.gz * mpu9250.deltat;
    //alpha를 이용한 보정(상보)
    mpu9250.roll = alpha_imu * gyro_angle_x + (1.0-alpha_imu) * accel_angle_x;
    mpu9250.pitch = alpha_imu * gyro_angle_y + (1.0-alpha_imu) * accel_angle_y;
    mpu9250.yaw = alpha_imu * gyro_angle_z + (1.0-alpha_imu) * mag_angle_z;
}
//imu thread에 넣고 돌리는 코드
