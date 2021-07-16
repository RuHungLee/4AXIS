# 4AXIS

7/16 首次試飛 , 在空中不會翻機 , 但因為平衡點不是在 setpoint 0 度 , 所以一起飛就會往一個方向飄走 , 沒辦法垂直起飛

開發版 : STM32F4

機架 : F450

姿態傳感器 : MPU9250

作業系統 : FreeRTOS

通訊模組 : ESP8266 , 利用 STM32 IDLE interrupt 搭配 DMA 收發 

使用驅動 : 
* STM32 HAL lib
* MPU DMP Driver ( https://github.com/fMeow/STM32_DMP_Driver )

