# Web3D-MPU6050
ESP8266使用MPU6050解算欧拉角，发送至MQTT服务器，Web端展示3D实时姿态

## Web端

### 引用JS库：

```
基础功能：JQuery.js
订阅MQTT消息：MQTT.js
3D渲染：Three.js
```

### 参考代码：

[https://github.com/emqx/MQTT-Client-Examples/blob/master/mqtt-client-WebSocket/ws-mqtt.html](https://github.com/emqx/MQTT-Client-Examples/blob/master/mqtt-client-WebSocket/ws-mqtt.html)

[https://github.com/mrdoob/three.js/blob/master/examples/webgl_geometry_cube.html](https://github.com/mrdoob/three.js/blob/master/examples/webgl_geometry_cube.html)

## ESP8266端

使用PlatformIO IDE开发

### 引用库：

```
MQTT消息发布：knolleary/PubSubClient@^2.8
SSD1306显示相关：
	adafruit/Adafruit GFX Library@^1.10.12
	adafruit/Adafruit SSD1306@^2.5.0
	adafruit/Adafruit BusIO@^1.10.0
MPU6050相关：
	I2CDevLib
	I2CDevLib -> MPU6050
```

### 参考代码：

[https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050/examples/MPU6050_DMP6_ESPWiFi](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050/examples/MPU6050_DMP6_ESPWiFi)

### 硬件线路：

<img src="https://cdn.jsdelivr.net/gh/bochili/cdn3/202201112047676.png" style="zoom:50%;" />
