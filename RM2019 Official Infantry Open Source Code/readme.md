该工程是RM2019机甲大师比赛外赠步兵机器人的嵌入式程序，使用的芯片型号为stm32f427IIH6，使用st公司标准库。
目录：
CMSIS：内核相关的文件
FWLIB：标准库文件
Project：工程文件
startup:芯片启动文件
user：用户编写的相关文件，主要编写的文件都在这个文件夹下

user/main.c\h :main函数所在的文件
user/AHRS：陀螺仪驱动以及姿态解算
user/APP：freeRTOS任务
user/DSP：DSP库
user/FreeRTOS:移植的freeRTOS文件
user/hardware：硬件层
user/user_lib：编写的数学函数