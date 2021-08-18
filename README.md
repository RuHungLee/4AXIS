# MyQuadcopter

Board : STM32F4

operating system : FreeRTOS

hardware sensors : ESP8266 , MPU9250 , TF02

control algorithm : double loop pid controller

三軸平衡使用角度角速度雙環 , z 軸定高使用高度速度雙環

地面工作站 : 

https://github.com/RuHungLee/MyDroneGCS

## Build and Program flash

```
make all
make flash 
```

## Debug

```
arm-none-eabi-gdb
target remote localhost:<port>
```


