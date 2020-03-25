#include "main.h"

//ACCEL_AVERAGE_DATA   Accel_Raw_Average_Data; 
//GYRO_RADIAN_DATA     Gyro_Radian_Data;
//MPU6050_ANGLE        MPU6050_Angle;

//void MPU6050_Data_Filter(void)  // this*0.01 + (��һ���õ�)last *0.99 Ҫ��
//{
//    unsigned int i=0;
//    static unsigned int first_flag = 0;
//    static unsigned int filter_cnt = 0;    //������ٶȼ��˲��Ĵ���
//    
//    long temp_accel_x = 0; //������ż��ٶȼ�X��ԭ�����ݵ��ۼӺ�
//    long temp_accel_y = 0; //������ż��ٶȼ�Y��ԭ�����ݵ��ۼӺ�
//    long temp_accel_z = 0; //������ż��ٶȼ�Z��ԭ�����ݵ��ۼӺ�
//    
//    static short accel_x_buffer[10] = {0}; //������ż��ٶȼ�X�����10�����ݵ�����
//    static short accel_y_buffer[10] = {0}; //������ż��ٶȼ�Y�����10�����ݵ�����
//    static short accel_z_buffer[10] = {0}; //������ż��ٶȼ�Z�����10�����ݵ�����
//    
//    if(first_flag == 0) //�����һ�ν����ú��������������ƽ����������г�ʼ��
//    {
//        first_flag = 1; //�Ժ��ٽ���
//        for(i=0;i<10;i++)
//        {
//            accel_x_buffer[i] = MPU6050_Raw_Data.Accel_X;
//            accel_y_buffer[i] = MPU6050_Raw_Data.Accel_Y;
//            accel_z_buffer[i] = MPU6050_Raw_Data.Accel_Z;
//        }
//    }
//    else  //������ǵ�һ����
//    {
//        accel_x_buffer[filter_cnt] = MPU6050_Raw_Data.Accel_X;
//        accel_y_buffer[filter_cnt] = MPU6050_Raw_Data.Accel_Y;
//        accel_z_buffer[filter_cnt] = MPU6050_Raw_Data.Accel_Z;   
//        
//        filter_cnt ++;
//        if(filter_cnt == 10)
//        {
//            filter_cnt = 0;
//        }        
//    }
//    
//    for(i=0;i<10;i++)
//    {
//        temp_accel_x += accel_x_buffer[i];
//        temp_accel_y += accel_y_buffer[i];
//        temp_accel_z += accel_z_buffer[i];
//    }
//    
//    Accel_Raw_Average_Data.X = (float)temp_accel_x / 10.0f;
//    Accel_Raw_Average_Data.Y = (float)temp_accel_y / 10.0f;
//    Accel_Raw_Average_Data.Z = (float)temp_accel_z / 10.0f;
//    
//    Gyro_Radian_Data.X = (float)(MPU6050_Real_Data.Gyro_X  * (3.14159265f/180.0f));
//    Gyro_Radian_Data.Y = (float)(MPU6050_Real_Data.Gyro_Y  * (3.14159265f/180.0f));
//    Gyro_Radian_Data.Z = (float)(MPU6050_Real_Data.Gyro_Z  * (3.14159265f/180.0f));
//}

//void MPU6050_Angle_Calculate( float gyro_x,
//                              float gyro_y,
//                              float gyro_z,
//                              float accel_x,
//                              float accel_y,
//                              float accel_z)
//{
//    static float q0 = 1.0f;
//    static float q1 = 0.0f;
//    static float q2 = 0.0f;
//    static float q3 = 0.0f;
//    
//    static float exInt = 0.0f;
//    static float eyInt = 0.0f;
//    static float ezInt = 0.0f;
//    
//    const float kp = 0.3f; //
//    const float ki = 0.00f; //0.0;
//    const float halfT = 0.001f; //�������ڵ�һ��ֵ
//    
//    float norm; //������ģ
//    float vx,vy,vz;
//    float ex,ey,ez;

//    float ax,ay,az; //���ٶ�������ģ�ı�ֵ 
//    float gx,gy,gz; //������

//    static float pre_ax = 0.0f;
//    static float pre_ay = 0.0f;
//    static float pre_az = 0.0f;
//    //���ٶ��˲�
//    accel_x = accel_x *0.02f + pre_ax * 0.98f;//cyq:0.02
//    pre_ax = accel_x;
//    
//    accel_y = accel_y *0.02f + pre_ay * 0.98f;
//    pre_ay = accel_y;

//    accel_z = accel_z *0.02f + pre_az * 0.98f;
//    pre_az = accel_z;    
//    
//    norm = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
//    ax = accel_x / norm;
//    ay = accel_y / norm;
//    az = accel_z / norm;
//    
//    vx = 2 * (q1*q3 - q0*q2);
//    vy = 2 * (q0*q1 + q2*q3);
//    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
//    
//    ex = (ay*vz - az*vy);
//    ey = (az*vx - ax*vz);
//    ez = (ax*vy - ay*vx);
//    
//    exInt += ki*ex;
//    eyInt += ki*ey;
//    ezInt += ki*ez;
//    
//    gx = gyro_x + kp*ex + exInt;
//    gy = gyro_y + kp*ey + eyInt;
//    gz = gyro_z + kp*ez + ezInt;
//    
//    q0 += (      - q1*gx - q2*gy - q3*gz)*halfT;
//    q1 += (q0*gx +         q2*gz - q3*gy)*halfT;
//    q2 += (q0*gy - q1*gz +         q3*gx)*halfT;
//    q3 += (q0*gz + q1*gy - q2*gx        )*halfT;

//    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
//    q0 = q0 / norm;
//    q1 = q1 / norm;
//    q2 = q2 / norm;
//    q3 = q3 / norm;
//    
//    MPU6050_Angle.Rool = asin(-2.0f * q1 * q3 + 2.0f * q0* q2) * (180.0f/3.14159265f); 
//    MPU6050_Angle.Pitch  = atan2(2.0f * q2 * q3 + 2.0f * q0 * q1, -2.0f * q1 * q1 - 2.0f * q2* q2 + 1.0f) * (180.0f/3.14159265f); 
//    MPU6050_Angle.Yaw = atan2( 2.0f * q1 * q2 + 2.0f * q0 * q3,1.0f - 2.0f * ( q2 * q2 + q3 * q2 ) ) * (180.0f/3.14159265f);//��׼

//}


