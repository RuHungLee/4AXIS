# MyQuadcopter

Board : STM32F4

Operating system : FreeRTOS

Sensors : ESP8266 , MPU9250 , TF02

Control algorithm : double loop pid control

三軸平衡使用角度角速度雙環 , z 軸定高使用高度速度雙環

地面工作站 : 

https://github.com/RuHungLee/MyDroneGCS

## Hardware configuration

### Clock Tree Configuration

![Screenshot from 2021-08-18 23-18-30](https://user-images.githubusercontent.com/39644941/129925699-500b5714-8fbf-458c-a451-0db03a522c1d.png)

### Pinout & Configuration

![Screenshot from 2021-08-18 23-00-31](https://user-images.githubusercontent.com/39644941/129922624-bb8b14d3-2817-421e-85be-f0cf19bf4bf1.png)

### MPU9250

PB6 : I2C1_SCL

PB7 : I2C1_SDA

Clock Speed (Hz) : 100000

STM32 Receiving method : polling

### TF02

PD8 : USART3_TX

PB11 : USART3_RX

Baud Rate : 115200 Bits/s

STM32 Receiving method : polling

### ESP8266

PA0 : UART4_TX

PA1 : UART4_RX

Baud Rate : 115200 Bits/s

STM32 Receiving method : idle interrupt , dma

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


