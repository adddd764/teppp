[TOC]

# ESP32-CAM拍照上传服务器

## 一、ESP32-CAM简介

[ESP32-CAM](https://makeradvisor.com/tools/esp32-cam/) 是一款非常小的摄像头模块，配备 ESP32-S 芯片。除了 OV2640 摄像头和几个用于连接外围设备的 GPIO 外，它还具有一个 microSD 卡插槽，可用于存储使用摄像头拍摄的图像或存储文件以提供给客户。

![img](https://www.seeedstudio.com/media/catalog/product/cache/ef3164306500b1080e8560b2e8b5cc0f/b/a/bazaar1003542_esp32cam2.jpg)

ESP32-CAM没有附带 USB 连接器，因此需要通过TTL转USBU0R和U0Tpins（串行引脚）。

**ESP32-CAM ==功能==的列表：**

- 最小的 802.11b/g/n Wi-Fi BT SoC 模块
- 低功耗 32 位 CPU，也可服务于应用处理器
- 高达 160MHz 的时钟速度，汇总计算能力高达 600 DMIPS
- 内置 520 KB SRAM，外部 4MPSRAM
- 支持 UART/SPI/I2C/PWM/ADC/DAC
- 支持 OV2640 和 OV7670 摄像头，内置闪光灯 lamp
- 支持图像 WiFi 上传
- 支持 TF 卡
- 支持多种睡眠模式
- 嵌入式 Lwip 和 FreeRTOS
- 支持 STA/AP/STA+AP作模式
- 支持 Smart Config/AirKiss 技术
- 支持串行端口本地和远程固件升级 （FOTA）

ESP32-CAM 引脚（AI-Thinker 模块）如下：

![img](https://www.seeedstudio.com/media/catalog/product/cache/ef3164306500b1080e8560b2e8b5cc0f/b/a/bazaar1003541_esp32cam3.jpg)

有三个**接地**引脚和两个电源引脚：任一3.3V或5V。

**GPIO 1**和**GPIO 3**是串行引脚。您需要这些 pin 才能将代码上传到您的开发板。此外GPIO 0也起着重要作用，因为它决定了 ESP32 是否处于闪烁模式。什么时候GPIO 0连接到**接地**，则 ESP32 处于闪烁模式。

**以下引脚内部连接到 microSD 读卡器：**

***

- GPIO 14：CLK
- GPIO 15：CMD
- GPIO 2：数据 0
- GPIO 4：数据 1（也连接到板载 LED）
- GPIO 12：数据 2
- GPIO 13：数据 3

***

<u>==**注意**==：烧录前需对ESP32-CAM模块进行烧录。</u>

## 二、项目实现流程

### （一） ESP32-CAM 构建视频流 Web 服务器

项目实现在 ESP32-CAM 构建视频流 Web 服务器基础上实现。了解如何使用 ESP32-CAM 板构建 Web 服务器，该服务器允许发送命令来拍照，并在浏览器中可视化保存在 SPIFFS 中的最新拍摄照片。

#### 1、ESP32-CAM 在 Web 服务器中拍照和显示实现

要执行此项目，需要以下部分：

实现效果如下：

![1745675318475](C:\Users\小小双的电脑\AppData\Roaming\Typora\typora-user-images\1745675318475.png)

#### 2、示例步骤实现

在此示例中，我们使用 Arduino IDE 对 ESP32-CAM 板进行编程。

在 Arduino IDE 中，转到 **ESP32** > **Camera** > **File** > **Examples** 并打开 **CameraWebServer** 示例。

加载如下代码：

```c
#include "esp_camera.h"
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#include "camera_pins.h"

// ===========================
// Enter your WiFi credentials
// ===========================
const char* ssid = "ESP8266";
const char* password = "04121835279";

void startCameraServer();
void setupLedFlash(int pin);

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable detector

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  gpio_config_t conf;
conf.mode = GPIO_MODE_INPUT;
conf.pull_up_en = GPIO_PULLUP_ENABLE;
conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
conf.intr_type = GPIO_INTR_DISABLE;
conf.pin_bit_mask = 1LL << 13;
gpio_config(&conf);
conf.pin_bit_mask = 1LL << 14;
gpio_config(&conf);

  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(config.pixel_format == PIXFORMAT_JPEG){
    if(psramFound()){
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV2640_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if(config.pixel_format == PIXFORMAT_JPEG){
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
  // Do nothing. Everything is done in another task by the web server
  delay(10000);
}

```

更改代码，插入自己的网络凭证：

```c
const char* ssid = "ESP8266";
const char* password = "04121835279";
```

然后，确保选择正确的摄像头模块——**AI-THINKER 模型**。

因此，注释所有其他模型并取消注释

```c
CAMERA_MODEL_AI_THINKER
```

```c
// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER
```

#### 3、ESP32-CAM 上传代码

使用 TTL转USB将 ESP32-CAM 板连接到的计算机。

![1745675888459](C:\Users\小小双的电脑\AppData\Roaming\Typora\typora-user-images\1745675888459.png)

确保跳线位于正确的位置以选择 5V。

**==重要：==** GPIO 0需要连接到接地以便能够上传代码。

| **ESP32-CAM 系列** | **FTDI 程序员** |
| ------------------ | --------------- |
| GND                | GND             |
| 5V                 | VCC(5V)         |
| U0R                | TXD             |
| U0T                | RXD             |
| GPIO               | 接地            |

#### 4、访问 Video Streaming Server

打开串口，ES32-CAM会自动生成一个IP地址：‘http"//172.20.10.2’

现在，可以在本地网络上访问摄像机流媒体服务器。打开浏览器并键入 ESP32-CAM IP 地址。按 **开始流式传输** 按钮开始视频流式传输。

![1745676206811](C:\Users\小小双的电脑\AppData\Roaming\Typora\typora-user-images\1745676206811.png)

您还可以选择通过单击拍照 **获取静止**图像 按钮。但此示例没有保存照片仅是视频流，接着在此示例上进行修改，实现ESP32-CAM拍照上传本地服务器。



### （二）ESP32-CAM拍照上传本地服务器

#### 1、添加拍照功能

在CameraWebServer示例上进行加工修改，可以连接到WiFi网络并将JPEG图像流传输到TCP服务器：

```c
#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include <vector>
 
const char *ssid = "ESP8266";   //wifi id
const char *password = "04121835279";  //wifi密码
const IPAddress serverIP(172,20,10,9); //欲访问的地址
uint16_t serverPort = 8940;         //服务器端口号
 
#define maxcache 1430
 
WiFiClient client; //声明一个客户端对象，用于与服务器进行连接
 
//CAMERA_MODEL_AI_THINKER类型摄像头的引脚定义
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
 
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
 
static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,  
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,
    
    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,
    
    .xclk_freq_hz = 20000000,  //帧率
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_VGA,    //图片格式
    .jpeg_quality = 12, //PEG图片质量（jpeg_quality），0-63，数字越小质量越高
    .fb_count = 1,
};
void wifi_init()
{
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false); //关闭STA模式下wifi休眠，提高响应速度
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi Connected!");
    Serial.print("IP Address:");
    Serial.println(WiFi.localIP());
}
esp_err_t camera_init() {
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.println("Camera Init Failed");
        return err;
    }
    sensor_t * s = esp_camera_sensor_get();
    //initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV2640_PID) {
    //        s->set_vflip(s, 1);//flip it back
    //        s->set_brightness(s, 1);//up the blightness just a bit
    //        s->set_contrast(s, 1);
    }
    Serial.println("Camera Init OK!");
    return ESP_OK;
}
 
void setup()
{
    Serial.begin(115200);
    wifi_init();
    camera_init();
}
 
void loop()
{
    Serial.println("Try To Connect TCP Server!");
    if (client.connect(serverIP, serverPort)) //尝试访问目标地址
    {
        Serial.println("Connect Tcp Server Success!");
        //client.println("Frame Begin");  //46 72 61 6D 65 20 42 65 67 69 6E // 0D 0A 代表换行  //向服务器发送数据
        while (1){       
          camera_fb_t * fb = esp_camera_fb_get();
          uint8_t * temp = fb->buf; //这个是为了保存一个地址，在摄像头数据发送完毕后需要返回，否则会出现板子发送一段时间后自动重启，不断重复
          if (!fb)
          {
              Serial.println( "Camera Capture Failed");
          }
          else
          { 
            //先发送Frame Begin 表示开始发送图片 然后将图片数据分包发送 每次发送1430 余数最后发送 
            //完毕后发送结束标志 Frame Over 表示一张图片发送完毕 
            client.print("FrameBegin"); //一张图片的起始标志
            // 将图片数据分段发送
            int leng = fb->len;
            int timess = leng/maxcache;
            int extra = leng%maxcache;
            for(int j = 0;j< timess;j++)
            {
              client.write(fb->buf, maxcache); 
              for(int i =0;i< maxcache;i++)
              {
                fb->buf++;
              }
            }
            client.write(fb->buf, extra);
            client.print("FrameOver");      // 一张图片的结束标志
            Serial.print("This Frame Length:");
            Serial.print(fb->len);
            Serial.println(".Succes To Send Image For TCP!");
            //return the frame buffer back to the driver for reuse
            fb->buf = temp; //将当时保存的指针重新返还
            esp_camera_fb_return(fb);  //这一步在发送完毕后要执行，具体作用还未可知。        
          }
          //等待服务端回应
          while (1) //如果已连接或有收到的未读取的数据
        {
            if (client.available()) //如果有数据可读取
            {
                String line = client.readStringUntil('\n'); //读取数据到换行符
                Serial.print("读取到数据：");
                Serial.println(line);
                break;
            }
        }
        }
        
        while (client.connected() || client.available()) //如果已连接或有收到的未读取的数据
        {
            if (client.available()) //如果有数据可读取
            {
                String line = client.readStringUntil('\n'); //读取数据到换行符
                Serial.print("ReceiveData：");
                Serial.println(line);
                client.print("--From ESP32--:Hello Server!");    
            }
        }
        Serial.println("close connect!");
        //client.stop(); //关闭客户端
        
    }
    else
    {
        Serial.println("Connect To Tcp Server Failed!After 10 Seconds Try Again!");
        client.stop(); //关闭客户端
    }
    delay(10000);
}


```

代码**实现功能**如下：

***

- 
  WiFi网络初始化：

连接到指定的WiFi网络（SSID："ESP8266"，密码："04121835279"）

禁用WiFi休眠模式以提高响应速度

通过串口监视器输出连接状态和本地IP地址

- 摄像头配置：


使用AI-Thinker摄像头模块（OV2640传感器）

配置为VGA分辨率(640x480)的JPEG格式

设置JPEG质量为12（0-63，数值越小质量越高）

帧率设置为20MHz

- TCP客户端功能：


尝试连接到指定的服务器IP(172.20.10.9)和端口(8940)

建立连接后开始传输摄像头捕获的图像帧

#### 2、创建接收服务端

接着，使用python创建接收端接收图片，并存储在本地文件夹中。

```python
import socket
import threading
import time
import os

# 协议标识符（建议使用无空格格式）
BEGIN_DATA = b'FrameBegin'
END_DATA = b'FrameOver'

# 图片存储配置（修改此处路径）
IMAGE_DIR = r'E:\esp32cam\photo'  # Windows 绝对路径
MAX_CACHE_SIZE = 1430  # 分包传输大小


class ImageHandler(threading.Thread):
    def __init__(self, client_socket, client_address):
        super().__init__()
        self.client_socket = client_socket
        self.client_address = client_address
        self.image_counter = 0
        self.lock = threading.Lock()  # 线程安全锁

    def run(self):
        print(f"[{time.time()}] 新连接: {self.client_address}")
        try:
            # 自动创建存储目录（若不存在）
            os.makedirs(IMAGE_DIR, exist_ok=True)

            # 初始化文件计数器（根据已有文件数量自动递增）
            with self.lock:
                existing_files = os.listdir(IMAGE_DIR)
                self.image_counter = len(existing_files) + 1

            while True:
                # 1. 接收开始标志
                begin_flag = self._recv_exact(len(BEGIN_DATA))
                if begin_flag != BEGIN_DATA:
                    continue  # 非图片数据包，跳过

                # 2. 接收图片数据（带超时机制）
                image_data = bytearray()
                timeout = time.time() + 10  # 10秒接收超时
                while time.time() < timeout:
                    chunk = self.client_socket.recv(MAX_CACHE_SIZE)
                    if not chunk:
                        break  # 连接断开
                    image_data += chunk
                    if image_data.endswith(END_DATA):
                        # 去除结束标志
                        image_data = image_data[:-len(END_DATA)]
                        break
                else:
                    print(f"[{time.time()}] 接收超时，关闭连接: {self.client_address}")
                    break  # 超时退出循环

                # 3. 保存图片
                filename = os.path.join(IMAGE_DIR, f"{self.image_counter:04d}.jpg")
                with open(filename, 'wb') as f:
                    f.write(image_data)
                print(f"[{time.time()}] 保存图片: {filename} ({len(image_data)} bytes)")

                # 4. 发送确认
                self.client_socket.send(b"ACK")

                # 5. 更新计数器
                with self.lock:
                    self.image_counter += 1

        except Exception as e:
            print(f"[{time.time()}] 错误: {str(e)}")
        finally:
            print(f"[{time.time()}] 关闭连接: {self.client_address}")
            self.client_socket.close()

    def _recv_exact(self, length):
        """确保精确接收指定长度的数据"""
        data = bytearray()
        while len(data) < length:
            packet = self.client_socket.recv(length - len(data))
            if not packet:
                return data  # 连接关闭时返回已接收数据
            data.extend(packet)
        return data


def main():
    # 创建TCP服务器
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 端口复用
    server.bind(('0.0.0.0', 8940))
    server.listen(5)
    print(f"[{time.time()}] 服务器启动，监听端口 8940...")

    try:
        while True:
            # 接受客户端连接
            client_sock, client_addr = server.accept()
            print(f"[{time.time()}] 连接来自: {client_addr}")

            # 创建独立线程处理客户端
            handler = ImageHandler(client_sock, client_addr)
            handler.start()

    except KeyboardInterrupt:
        print("[{}] 服务器关闭".format(time.time()))
    finally:
        server.close()


if __name__ == "__main__":
    main()
```

**主要功能**如下：

- TCP图片服务器：

监听8940端口，接收客户端连接

每个客户端连接由独立线程处理(ImageHandler)

支持多客户端同时连接

- 图片传输协议：


使用自定义协议标识符：FrameBegin表示图片开始，FrameOver表示图片结束

支持分包传输(最大1430字节/包)

接收完成后发送ACK确认

- 图片存储：


图片保存为JPG格式，按顺序编号(0001.jpg, 0002.jpg等)

存储路径可配置(默认为E:\esp32cam\photo)

#### 3、项目使用流程

- **部署Python服务器：**
  运行python接收端示例
  输出示例：
  [1713045345.123] 服务器启动，监听端口 8640...
  ![1745677516070](C:\Users\小小双的电脑\AppData\Roaming\Typora\typora-user-images\1745677516070.png)



- **烧录Arduino代码：**
  使用Arduino IDE选择ESP32开发板
  确认WiFi配置正确
  监控串口输出验证连接状态

![1745677347445](C:\Users\小小双的电脑\AppData\Roaming\Typora\typora-user-images\1745677347445.png)

接收到的图片将存入本地创建的esp32cam\photo目录下：

![1745677657659](C:\Users\小小双的电脑\AppData\Roaming\Typora\typora-user-images\1745677657659.png)



## 三、总结

本项目基于ESP32-CAM板进行，在CamareWebServer示例上进行功能改进与完善，实现了实时拍照上传本地服务器的功能。在本组互联网+项目内可担当实时害虫拍照并上传本地服务器，为yolo框架监测害虫提供便利。



## 附录

Arduino ESP32-CAM学习网址:[ESP32-CAM 在 Web 服务器中拍照和显示 |随机书教程](https://randomnerdtutorials.com/esp32-cam-take-photo-display-web-server/)

