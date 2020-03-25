#include "main.h"

volatile MPU6050_RAW_DATA    MPU6050_Raw_Data;    //原始数据
volatile MPU6050_REAL_DATA   MPU6050_Real_Data;
AHRS ahrs;
uint8_t mpu_buf[20]={0};       //save the data of acc gyro & mag using iic

int16_t MPU6050_FIFO[6][11] = {0};//[0]-[9]为最近10次数据 [10]为10次数据的平均值
int16_t HMC5883_FIFO[3][11] = {0};//[0]-[9]为最近10次数据 [10]为10次数据的平均值 注：磁传感器的采样频率慢，所以单独列出
MagMaxMinData_t MagMaxMinData;


float HMC5883_lastx,HMC5883_lasty,HMC5883_lastz;
//MPU6050 初始化，成功返回0  失败返回 0xff
int MPU6050_Init(void)
{
    unsigned char temp_data = 0x00;

    IIC_GPIO_Init();  //初始化IIC接口
    HEAT_Configuration();
    
    if(IIC_ReadData(MPU6050_DEVICE_ADDRESS,WHO_AM_I,&temp_data,1)==0) //确定IIC总线上挂接的是否是MPU6050
    {
        if(temp_data != MPU6050_ID)
        {
            printf("error 1A\r\n");
            return 0xff; //校验失败，返回0xff
        }
    }
    else
    {
        printf("error 1B\r\n");
        return 0xff; //读取失败 返回0xff
    }
    
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,PWR_MGMT_1,0x01) == 0xff)    //解除休眠状态
    {
        printf("error 1C\r\n");
        return 0xff;
    }
		

    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,CONFIG,0x03) == 0xff)         //Digital Low-Pass Filter:DLPF_CFG is 3, Fs is 1khz 
    {                                                                     //acc bandwidth 44Hz,gyro 42Hz
        printf("error 1E\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,GYRO_CONFIG,0x10) == 0xff)    //FS_SEL 3 : gyroscope full scale range is +-1000degs/s 
    {
        printf("error 1F\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,ACCEL_CONFIG,0x00) == 0xff)   //AFS_SEL 1: accelerometer full scale range is +-2g
    {
        printf("error 1G\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_PIN_CFG,0x02) == 0xff)    //logic level for the INT pin is active high
                                                                          //the INT pin emits a 50us long pulse, not latched    bypass mode enabled
    {
        printf("error 1H\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_ENABLE,0x00) == 0xff)      //disable data ready interrupt
    {
        printf("error 1I\r\n");
        return 0xff;
    }
		
		//设置mpu6050 IIC masters mode  disabled 不让mpu6050控制aux IIC接口
		if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,MPU6050_RA_USER_CTRL,0x00) == 0xff)      //disable data ready interrupt
    {
        printf("error 1I\r\n");
        return 0xff;
    }
		
		//设置IIC masters mode 为 bypass mode enabled，在INT_PIN_CFG中配置
		

		
//		//5883初始化
		if(HMC5883_Init() == 0xff)
		{
			  printf("error 1K\r\n");
        return 0xff;
		}
		
    Delay_ms(500);
    //MPU6050_GyroCalibration();
    return 0;
}

int MPU6050_EnableInt(void)
{
	  if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,SMPLRT_DIV,0x01)==0xff)      //Sample Rate: Gyro output rate / (1 + 1) = 500Hz
	  {
        printf("Cannot enable interrupt successfully.\r\n");
        return 0xff;
	  }
    printf("MPU6050 set sample rate done.\n");
	  Delay_ms(10);
	  if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_ENABLE,0x01) == 0xff)     //enable data ready interrupt 
    {
      printf("error 1I\r\n");
      return 0xff;
    } 
#if 0
    printf("MPU6050 enable interrupt done.\n"); 
//	unsigned char  buf[14] ={0};    
//    Delay_ms(2);
//	IIC_ReadData(MPU6050_DEVICE_ADDRESS,MPU6050_DATA_START,buf,14);    //dummy read to clear the data registers
//	Delay_ms(10);
#endif
	return 0;
}

void MPU6050_Initialize(void)
{
    while(MPU6050_Init() == 0xff) 
    {                       
       Delay_ms(200);              
    }
}

//MPU6050  数据读取，成功返回0  失败返回 0xff



int MPU6050_ReadData(uint8_t Slave_Addr, uint8_t Reg_Addr, uint8_t * Data, uint8_t Num)
{    
	//IIC_ReadData(MPU6050_DEVICE_ADDRESS,MPU6050_DATA_START,buf,14)
    if(IIC_ReadData(Slave_Addr,Reg_Addr,Data,Num) == 0xff)
    {
        printf("error 1J\r\n");
        return 0xff;
    }
   
    return 0;
}

/**********************************************************************************/
/*将MPU6050_ax,MPU6050_ay, MPU6050_az,MPU6050_gx, MPU6050_gy, MPU6050_gz处理后存储*/
/**********************************************************************************/
void MPU6050_DataSave(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz) //[0]-[9]为最近10次数据 [10]为10次数据的平均值
{
	uint8_t i = 0;
	int32_t sum=0;
	
	for(i=1;i<10;i++)
	{
		MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
		MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
		MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
		MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
		MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
		MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
	}
	
	MPU6050_FIFO[0][9]=ax;//将新的数据放置到 数据的最后面
	MPU6050_FIFO[1][9]=ay;
	MPU6050_FIFO[2][9]=az;
	MPU6050_FIFO[3][9]=gx;
	MPU6050_FIFO[4][9]=gy;
	MPU6050_FIFO[5][9]=gz;
	
	for(i=0;i<10;i++)//求当前数组的合，再取平均值
	{	
		 sum+=MPU6050_FIFO[0][i];
	}
	MPU6050_FIFO[0][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[1][i];
	}
	MPU6050_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[2][i];
	}
	MPU6050_FIFO[2][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[3][i];
	}
	MPU6050_FIFO[3][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[4][i];
	}
	MPU6050_FIFO[4][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[5][i];
	}
	MPU6050_FIFO[5][10]=sum/10;
	
}


int16_t MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz
				,MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz;
/**************************实现函数********************************************
*函数原型:		void MPU6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
*功　　能:	    读取 MPU6050的当前测量值
*******************************************************************************/
void MPU6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
  
	if(isMPU6050_is_DRY)
	{
		isMPU6050_is_DRY = 0;
		MPU6050_ReadData(MPU6050_DEVICE_ADDRESS,MPU6050_DATA_START,mpu_buf,14);  //
		HMC58X3_ReadData(&(mpu_buf[14]));  //14-19为陀螺仪数据
		MPU6050_Lastax=(((int16_t)mpu_buf[0]) << 8) | mpu_buf[1];
		MPU6050_Lastay=(((int16_t)mpu_buf[2]) << 8) | mpu_buf[3];
		MPU6050_Lastaz=(((int16_t)mpu_buf[4]) << 8) | mpu_buf[5];
		//跳过温度ADC
		MPU6050_Lastgx=(((int16_t)mpu_buf[8]) << 8) | mpu_buf[9];
		MPU6050_Lastgy=(((int16_t)mpu_buf[10]) << 8) | mpu_buf[11];
		MPU6050_Lastgz=(((int16_t)mpu_buf[12]) << 8) | mpu_buf[13];
			
		MPU6050_DataSave(MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz,MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz);  		
		*ax  =MPU6050_FIFO[0][10];
		*ay  =MPU6050_FIFO[1][10];
		*az = MPU6050_FIFO[2][10];
		*gx  =MPU6050_FIFO[3][10] - GyroSavedCaliData.GyroXOffset;
		*gy = MPU6050_FIFO[4][10] - GyroSavedCaliData.GyroYOffset;
		*gz = MPU6050_FIFO[5][10] - GyroSavedCaliData.GyroZOffset;
	} 
	else
	{       //读取上一次的值
		*ax = MPU6050_FIFO[0][10];//=MPU6050_FIFO[0][10];
		*ay = MPU6050_FIFO[1][10];//=MPU6050_FIFO[1][10];
		*az = MPU6050_FIFO[2][10];//=MPU6050_FIFO[2][10];
		*gx = MPU6050_FIFO[3][10]-GyroSavedCaliData.GyroXOffset;//=MPU6050_FIFO[3][10];
		*gy = MPU6050_FIFO[4][10]-GyroSavedCaliData.GyroYOffset;//=MPU6050_FIFO[4][10];
		*gz = MPU6050_FIFO[5][10]-GyroSavedCaliData.GyroZOffset;//=MPU6050_FIFO[5][10];
	}
}

void MPU6050_getlastMotion6(int16_t* ax, int16_t* ay, 
		int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
	*ax  =MPU6050_FIFO[0][10];
	*ay  =MPU6050_FIFO[1][10];
	*az = MPU6050_FIFO[2][10];
	*gx  =MPU6050_FIFO[3][10]-GyroSavedCaliData.GyroXOffset;
	*gy = MPU6050_FIFO[4][10]-GyroSavedCaliData.GyroYOffset;
	*gz = MPU6050_FIFO[5][10]-GyroSavedCaliData.GyroZOffset;
}

uint8_t HMC5883_Init(void)
{
    uint8_t tmp_ch = 0;
    if(MPU6050_ReadData(HMC5883_ADDRESS, HMC58X3_R_IDA, &tmp_ch, 1) == 0)
    {
        if(tmp_ch != HMC5883_DEVICE_ID_A)
        {
            printf("error 2A\r\n");
            return 0xff;
        }
    }
    else
    {
        printf("error 2B\r\n");
        return 0xff;
    }

    IIC_WriteData(HMC5883_ADDRESS, HMC58X3_R_CONFA,0x70);
    Delay_ms(5);
    IIC_WriteData(HMC5883_ADDRESS, HMC58X3_R_CONFB,0xA0);
    Delay_ms(5);
    IIC_WriteData(HMC5883_ADDRESS, HMC58X3_R_MODE,0x00);    //这里初始化为0x00 连续模式
    //wait the response of the hmc5883 stabalizes, 6 milliseconds  
    Delay_ms(6);
    //set DOR
    IIC_WriteData(HMC5883_ADDRESS, HMC58X3_R_CONFA,6<<2);   //75HZ更新
    Delay_ms(6);
    return 0;
}


/**************************实现函数********************************************
*函数原型:	  void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z)
*功　　能:	   写HMC5883L的寄存器读取5883寄存器的值
输入参数：    reg  寄存器地址
			  val   要写入的值	
输出参数：  无
*******************************************************************************/
void HMC58X3_ReadData(u8 *vbuff) {   
   MPU6050_ReadData(HMC5883_ADDRESS,HMC58X3_R_XM,vbuff,6);   //读取到磁力计数据
}


/**************************实现函数********************************************
*函数原型:	   void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
*功　　能:	   更新一组数据到FIFO数组
输入参数：  磁力计三个轴对应的ADC值
输出参数：  无
*******************************************************************************/
void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
{
	uint8_t i = 0;
	int32_t sum=0;

	for(i=1;i<10;i++)
	{
		HMC5883_FIFO[0][i-1]=HMC5883_FIFO[0][i];
		HMC5883_FIFO[1][i-1]=HMC5883_FIFO[1][i];
		HMC5883_FIFO[2][i-1]=HMC5883_FIFO[2][i];
	}
	HMC5883_FIFO[0][9]= x;//将新的数据放置到 数据的最后面
	HMC5883_FIFO[1][9]= y;
	HMC5883_FIFO[2][9]= z;
	
	for(i=0;i<10;i++)//求当前数组的合，再取平均值
	{	
		 sum+=HMC5883_FIFO[0][i];
	}
	HMC5883_FIFO[0][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=HMC5883_FIFO[1][i];
	}
	HMC5883_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=HMC5883_FIFO[2][i];
	}
	HMC5883_FIFO[2][10]=sum/10;
	//以上全部为未校准数据
	if(MagMaxMinData.MinMagX>HMC5883_FIFO[0][10])
	{
		MagMaxMinData.MinMagX=(int16_t)HMC5883_FIFO[0][10];
	}
	if(MagMaxMinData.MinMagY>HMC5883_FIFO[1][10])
	{
		MagMaxMinData.MinMagY=(int16_t)HMC5883_FIFO[1][10];
	}
	if(MagMaxMinData.MinMagZ>HMC5883_FIFO[2][10])
	{
		MagMaxMinData.MinMagZ=(int16_t)HMC5883_FIFO[2][10];
	}

	if(MagMaxMinData.MaxMagX<HMC5883_FIFO[0][10])
	{
		MagMaxMinData.MaxMagX=(int16_t)HMC5883_FIFO[0][10];		
	}
	if(MagMaxMinData.MaxMagY<HMC5883_FIFO[1][10])
	{
		MagMaxMinData.MaxMagY = HMC5883_FIFO[1][10];
	}
	if(MagMaxMinData.MaxMagZ<HMC5883_FIFO[2][10])
	{
		MagMaxMinData.MaxMagZ=(int16_t)HMC5883_FIFO[2][10];
	}		
}


/**************************实现函数********************************************
*函数原型:	  void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z)
*功　　能:	   写HMC5883L的寄存器
输入参数：    reg  寄存器地址
			  val   要写入的值	
输出参数：  无
*******************************************************************************/
void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z) 
{
    HMC58X3_ReadData(&mpu_buf[14]);
    HMC58X3_newValues((((int16_t)mpu_buf[18] << 8) | mpu_buf[19]), -(((int16_t)mpu_buf[14] << 8) | mpu_buf[15]), ((int16_t)mpu_buf[16] << 8) | mpu_buf[17]);
    *x = HMC5883_FIFO[0][10];
    *y = HMC5883_FIFO[1][10];
    *z = HMC5883_FIFO[2][10];
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_getValues(int16_t *x,int16_t *y,int16_t *z)
*功　　能:	   读取 磁力计的当前ADC值
输入参数：    三个轴对应的输出指针	
输出参数：  无
*******************************************************************************/
void HMC58X3_getlastValues(int16_t *x,int16_t *y,int16_t *z) 
{
    *x = HMC5883_FIFO[0][10];
    *y = HMC5883_FIFO[1][10]; 
    *z = HMC5883_FIFO[2][10]; 
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_mgetValues(volatile float *arry)
*功　　能:	   读取 校正后的 磁力计ADC值
输入参数：    输出数组指针	
输出参数：  无
*******************************************************************************/
void HMC58X3_mgetValues(volatile float *arry) 
{
    int16_t xr,yr,zr;
    HMC58X3_getRaw(&xr, &yr, &zr);
    arry[0]= HMC5883_lastx=((float)(xr - MagSavedCaliData.MagXOffset)) * MagSavedCaliData.MagXScale;
    arry[1]= HMC5883_lasty=((float)(yr - MagSavedCaliData.MagYOffset)) * MagSavedCaliData.MagYScale;
    arry[2]= HMC5883_lastz=((float)(zr - MagSavedCaliData.MagZOffset)) * MagSavedCaliData.MagZScale;
}






