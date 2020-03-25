#include "main.h"
#include "imu.h"
#include "math.h"
#include "mpu6050_driver.h"

volatile float exInt, eyInt, ezInt;  // 误差积分
volatile float q0 = 1.0f;
volatile float q1 = 0.0f;
volatile float q2 = 0.0f;
volatile float q3 = 0.0f;

volatile float mygetqval[9];	//用于存放传感器转换结果的数组
static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;   //作用域仅在此文件中

static volatile float q[4]; //　四元数
volatile uint32_t lastUpdate, now; // 采样周期计数 单位 us
volatile float angle[3] = {0};
volatile float yaw_temp,pitch_temp,roll_temp;
volatile float last_yaw_temp,last_pitch_temp,last_roll_temp;
volatile float yaw_angle,pitch_angle,roll_angle; //使用到的角度值

// Fast inverse square-root
/**************************实现函数********************************************
*函数原型:	   float invSqrt(float x)
*功　　能:	   快速计算 1/Sqrt(x) 	
输入参数： 要计算的值
输出参数： 结果
*******************************************************************************/
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**************************实现函数********************************************
*函数原型:	   void Init_Quaternion
*功　　能:	 初始化四元数
输入参数： 当前的测量值。
输出参数：没有
*******************************************************************************/
//初始化IMU数据
#define BOARD_DOWN 1   //板子正面朝下摆放

void Init_Quaternion()//根据测量数据，初始化q0,q1,q2.q3，从而加快收敛速度
{
	int16_t hx,hy,hz;
	HMC58X3_getlastValues(&hx,&hy,&hz);
	#ifdef BOARD_DOWN
	if(hx<0 && hy <0)   //OK
	{
		if(fabs(hx/hy)>=1)
		{
			q0 = -0.005;
			q1 = -0.199;
			q2 = 0.979;
			q3 = -0.0089;
		}
		else
		{
			q0 = -0.008;
			q1 = -0.555;
			q2 = 0.83;
			q3 = -0.002;
		}
		
	}
	else if (hx<0 && hy > 0) //OK
	{
		if(fabs(hx/hy)>=1)   
		{
			q0 = 0.005;
			q1 = -0.199;
			q2 = -0.978;
			q3 = 0.012;
		}
		else
		{
			q0 = 0.005;
			q1 = -0.553;
			q2 = -0.83;
			q3 = -0.0023;
		}
		
	}
	else if (hx > 0 && hy > 0)   //OK
	{
		if(fabs(hx/hy)>=1)
		{
			q0 = 0.0012;
			q1 = -0.978;
			q2 = -0.199;
			q3 = -0.005;
		}
		else
		{
			q0 = 0.0023;
			q1 = -0.83;
			q2 = -0.553;
			q3 = 0.0023;
		}
		
	}
	else if (hx > 0 && hy < 0)     //OK
	{
		if(fabs(hx/hy)>=1)
		{
			q0 = 0.0025;
			q1 = 0.978;
			q2 = -0.199;
			q3 = 0.008;			
		}
		else
		{
			q0 = 0.0025;
			q1 = 0.83;
			q2 = -0.56;
			q3 = 0.0045;
		}		
	}
	#else
		if(hx<0 && hy <0)
	{
		if(fabs(hx/hy)>=1)
		{
			q0 = 0.195;
			q1 = -0.015;
			q2 = 0.0043;
			q3 = 0.979;
		}
		else
		{
			q0 = 0.555;
			q1 = -0.015;
			q2 = 0.006;
			q3 = 0.829;
		}
		
	}
	else if (hx<0 && hy > 0)
	{
		if(fabs(hx/hy)>=1)
		{
			q0 = -0.193;
			q1 = -0.009;
			q2 = -0.006;
			q3 = 0.979;
		}
		else
		{
			q0 = -0.552;
			q1 = -0.0048;
			q2 = -0.0115;
			q3 = 0.8313;
		}
		
	}
	else if (hx>0 && hy > 0)
	{
		if(fabs(hx/hy)>=1)
		{
			q0 = -0.9785;
			q1 = 0.008;
			q2 = -0.02;
			q3 = 0.195;
		}
		else
		{
			q0 = -0.9828;
			q1 = 0.002;
			q2 = -0.0167;
			q3 = 0.5557;
		}
		
	}
	else if (hx > 0 && hy < 0)
	{
		if(fabs(hx/hy)>=1)
		{
			q0 = -0.979;
			q1 = 0.0116;
			q2 = -0.0167;
			q3 = -0.195;			
		}
		else
		{
			q0 = -0.83;
			q1 = 0.014;
			q2 = -0.012;
			q3 = -0.556;
		}		
	}
	#endif
	
	//根据hx hy hz来判断q的值，取四个相近的值做逼近即可,初始值可以由欧拉角转换到四元数计算得到
	 
}


	


/**************************实现函数********************************************
*函数原型:	   void IMU_getValues(volatile float * values)
*功　　能:	 读取加速度 陀螺仪 磁力计 的当前值  
输入参数： 将结果存放的数组首地址
加速度值：原始数据，-8192-+8192
角速度值：deg/s
磁力计值：原始数据
输出参数：没有
*******************************************************************************/
void IMU_getValues(volatile float * values) {  
		int16_t accgyroval[6];
		int i;
	//读取加速度和陀螺仪的当前ADC
  MPU6050_getMotion6(&accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5]);
	MPU6050_Raw_Data.Accel_X = accgyroval[0];
	MPU6050_Raw_Data.Accel_Y = accgyroval[1];
	MPU6050_Raw_Data.Accel_Z = accgyroval[2];
	MPU6050_Raw_Data.Gyro_X = accgyroval[3];
	MPU6050_Raw_Data.Gyro_Y = accgyroval[4];
	MPU6050_Raw_Data.Gyro_Z = accgyroval[5];
	
	
    for(i = 0; i<6; i++) {
      if(i < 3) {
        values[i] = (float) accgyroval[i];
      }
      else {
        values[i] = ((float) accgyroval[i]) / 32.8f; //转成度每秒
		//这里已经将量程改成了 1000度每秒  32.8 对应 1度每秒
      }
    }
    HMC58X3_mgetValues(&values[6]);	//读取磁力计的ADC值
		MPU6050_Raw_Data.Mag_X = values[6];
		MPU6050_Raw_Data.Mag_Y = values[7];
		MPU6050_Raw_Data.Mag_Z = values[8];
}

/**************************实现函数********************************************
*函数原型:	   void IMU_AHRSupdate
*功　　能:	 更新AHRS 更新四元数 
输入参数： 当前的测量值。
输出参数：没有
*******************************************************************************/
#define Kp 2.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.01f   // integral gain governs rate of convergence of gyroscope biases
void IMU_AHRSupdate(void) {
    float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez,halfT;
    float tempq0,tempq1,tempq2,tempq3;

    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;   
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;   

    gx = mygetqval[3] * M_PI/180;
    gy = mygetqval[4] * M_PI/180;
    gz = mygetqval[5] * M_PI/180;
    ax = mygetqval[0];
    ay = mygetqval[1];
    az = mygetqval[2];
    mx = mygetqval[6];
    my = mygetqval[7];
    mz = mygetqval[8];		

    now = Get_Time_Micros();  //读取时间 单位是us   
    if(now<lastUpdate)
    {
    //halfT =  ((float)(now + (0xffffffff- lastUpdate)) / 2000000.0f);   //  uint 0.5s
    }
    else	
    {
        halfT =  ((float)(now - lastUpdate) / 2000000.0f);
    }
    lastUpdate = now;	//更新时间
    //快速求平方根算法
    norm = invSqrt(ax*ax + ay*ay + az*az);       
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    //把加计的三维向量转成单位向量。
    norm = invSqrt(mx*mx + my*my + mz*mz);          
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm; 
    // compute reference direction of flux
    hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
    hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
    hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz; 
    // estimated direction of gravity and flux (v and w)
    vx = 2.0f*(q1q3 - q0q2);
    vy = 2.0f*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
    wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
    wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
        exInt = exInt + ex * Ki * halfT;
        eyInt = eyInt + ey * Ki * halfT;	
        ezInt = ezInt + ez * Ki * halfT;
        // 用叉积误差来做PI修正陀螺零偏
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
    }
    // 四元数微分方程
    tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

    // 四元数规范化
    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;

}


/**************************实现函数********************************************
*函数原型:	   void IMU_getQ(float * q)
*功　　能:	 更新四元数 返回当前的四元数组值
输入参数： 将要存放四元数的数组首地址
输出参数：没有
*******************************************************************************/

void IMU_getQ(volatile float * q) {

    IMU_getValues(mygetqval);	 //获取原始数据,加速度计和磁力计是原始值，陀螺仪转换成了deg/s
    IMU_AHRSupdate();
    q[0] = q0; //返回当前值
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getYawPitchRoll(float * angles)
*功　　能:	 更新四元数 返回当前解算后的姿态数据
输入参数： 将要存放姿态角的数组首地址
输出参数：没有
*******************************************************************************/
void IMU_getYawPitchRoll(volatile float * angles) 
{  
    // volatile float gx=0.0, gy=0.0, gz=0.0; //估计重力方向
    IMU_getQ(q); //更新全局四元数
    //四元数转换成欧拉角，经过三角函数计算即可
    angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw        -pi----pi
    angles[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch    -pi/2    --- pi/2 
    angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll       -pi-----pi  
}

static int yaw_count = 0;
void GetPitchYawGxGyGz()
{
	MPU6050_Real_Data.Gyro_X = mygetqval[3];
	MPU6050_Real_Data.Gyro_Y = -mygetqval[4];
	MPU6050_Real_Data.Gyro_Z = mygetqval[5];

	last_yaw_temp = yaw_temp;
	yaw_temp = angle[0]; 
	if(yaw_temp-last_yaw_temp>=330)  //yaw轴角度经过处理后变成连续的
	{
		yaw_count--;
	}
	else if (yaw_temp-last_yaw_temp<=-330)
	{
		yaw_count++;
	}
	yaw_angle = yaw_temp + yaw_count*360;  //yaw轴角度
	pitch_angle = angle[1];
    roll_angle = angle[2];	
}
