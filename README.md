# **_“Surveillance Robot using ESP32 Cam Module”_**

### **Submitted By**

| Sr No |   PRN    |        Name         |
| :---: | :------: | :-----------------: |
|   1   | 21410038 | Mr. Shardul Shastri |
|   2   | 21410039 | Mr. Atharva Tambade |
|   3   | 21410043 | Mr. Ayush Shirbhate |
|   4   | 21410050 |   Mr. Vikas Gouda   |

**Under the guidance of**

_Ms. Deepika Chavan_

# **Department of Electronics Engineering WALCHAND COLLEGE OF ENGINEERING, SANGLI**

(Government-Aided Autonomous Institute)

# **2023-2024**

**(**This page is intentionally kept blank**)**

**Contents**

1. **List of Figures 04**

2. **List of Tables 04**

3. **Abstract 06**

4. **Acknowledgement 06**

5. **Chapter-1 07**

6. **Chapter-2 08**

7. **Chapter-3 10**

8. **Chapter-4 20**

9. **Cost Estimation 24**

10. **Appendices 25**

11. **References 29**(\#references)

# **LIST OF FIGURES**

Figure 2.1 Block Diagram of Surveillance Robot 09  
Figure 2.3.1 Arduino Uno 10  
Figure 2.3.2 ESP 32 Cam Module 12  
Figure 2.3.3 L298N Motor Driver 14  
Figure 2.3.4 Wheels and Chassis 14  
Figure 2.3.5 DC Motors 14  
Figure 2.4.1 Flow Chart 16  
Figure 3.2 Hardware of project 18

**LIST OF TABLES**

Table 1: Cost of Project 24

# **ABSTRACT**

A surveillance robot using ESP32-CAM is a system that utilizes the ESP32-CAM board

and a robot chassis to create a mobile surveillance device. The ESP32-CAM is a low-cost

development board that integrates a small camera module and Wi-Fi connectivity. The robot

chassis allows the device to move around and capture video in different locations. The system

can be controlled through a web interface hosted on the ESP32-CAM board. The web interface

allows the user to control the robot's movement, view live video streams, and take snapshots

of the video feed. Additionally, the system can be programmed to detect motion using

computer vision algorithms, such as object detection and tracking, and send alerts to the user.

The surveillance robot using ESP32-CAM has potential applications in home security,

monitoring of remote locations, and industrial surveillance. With its low cost and easy-to-use

interface, it provides a convenient solution for anyone who needs to monitor their surroundings

remotely.

# **ACKNOWLEDGEMENT**

We would like to express our gratitude to our mentor, Ms. Deepika Chavan Mam, for her invaluable guidance and support throughout the development of our surveillance car robot project using the ESP32 CAM module. We also thank the Electronics Department for providing resources and assistance. Special thanks to faculty, staff, teammates, and the ESP32 CAM module for their contributions. Dhukhi

# **CHAPTER 1**

1. ## **Introduction**

Our primary objective is to develop a mobile surveillance solution capable of autonomous navigation, obstacle detection, video capture, and real-time data transmission via IoT connectivity. By integrating these components, we aim to enhance security measures while providing users with convenient remote monitoring capabilities.

In today's dynamic security landscape, the need for autonomous surveillance systems is paramount. Our project endeavors to address this need by designing a surveillance robot car utilizing components such as the Arduino UNO, L298N Motor Driver, and incorporating IoT connectivity for remote control and data transmission

1. **Background**

In response to evolving security challenges, our project aims to pioneer a cutting-edge mobile surveillance solution. Leveraging advanced technologies including Arduino UNO, we're developing an autonomous robot capable of navigating, detecting obstacles, and capturing audio/video data. With IoT integration, our system enables real-time remote monitoring and data transmission, bolstering security measures while enhancing user convenience.

2. ### **Motivation**

Following are reasons due to which we find this project important:

- **Innovation:** Developing a hand gesture control robot is a cutting-edge project that involves the use of modern technologies and provides an opportunity to innovate in the field of robotics and automation.

- **Accessibility:** The project aims to create a more intuitive and user-friendly method of controlling robots that is accessible to a wider range of users without specialized training.

- **Skill Development:** Working on this project involves learning new skills and gaining valuable experience in coding, electronics, and sensor technology.

3. ### **Problem Description**

In contemporary security landscapes, traditional surveillance methods often fall short in providing comprehensive coverage and real-time monitoring capabilities. Manual surveillance systems are labor-intensive, prone to human error, and limited in scalability, making them inadequate for effectively addressing modern security challenges. Additionally, the reliance on fixed cameras and stationary monitoring stations limits the flexibility and adaptability required to cover dynamic environments or large areas efficiently. Consequently, there's a pressing need for a versatile and autonomous surveillance solution capable of navigating diverse terrains, detecting obstacles, and transmitting data in real time to enhance security measures while providing users with convenient remote monitoring capabilities. This project aims to bridge this gap by developing a state-of-the-art Surveillance Robot, integrating advanced components and IoT connectivity to revolutionize surveillance practices.

4. ### **Objectives**

The objectives of the project are:

1. Develop a cost-effective and user-friendly device
2. Implement real-time video capture capabilities within the surveillance system to enhance situational awareness and security monitoring.
3. Establish robust IOT connectivity to enable remote control and data transmission, facilitating seamless integration with existing security infrastructure.
4. Enable easy integration of additional features for future enhancement

# **CHAPTER 2**

1. ## **Technology and Literature Survey**

1. **“Intelligent combat robot 2015” by V. SHANKAR:** It has been described as developing a robotic vehicle for remote operation using RF technology and a wireless camera for monitoring purposes. The robot and camera can send real-time footage with night vision capabilities through a wireless network. This type of robot could be useful in war zones for spying purposes. In this technology, a robot can only be operated from a distance of ten meters. Robot with Bluetooth control: With the increased speed, a new classification technique was presented to improve the robot's range. The camera's link was frequently lost with this technology.

1. **Dr. S. Bhargavi and S. Manjunath Electronics and Communication:** The goal of this research is to reduce human casualties in terrorist attacks like the one on September 11, 2001\. The combat robot was created to deal with such heinous terror acts. This robot is radio-controlled self-powered and equipped with all of the controls found in a typical car. It's been outfitted with a wireless camera so that it can keep an eye on the adversary from afar if necessary. It can enter enemy territory and transmit all information to us via its small camera eyes. This spy robot can be deployed at high-end hotels, shopping malls, and jewelry showrooms, among places where intruders or terrorists may pose a threat.

1. ## **Block Diagram**

   Surveillance Robot project is divided into the following blocks:

![][image1]

                _Figure 2.1: Block Diagram of Surveillance Robot_

3. ## **Hardware Required**

   1. ### **Arduino Uno**

Arduino is an open-source platform used for building electronics projects. Arduino consists of both a physical programmable circuit board and an IDE that runs on your computer, used to write and upload computer code to the physical board. The Arduino IDE uses a simplified version of C++, making it easier to learn to program.

                                                                    *Figure 2.3.1: Arduino Uno*

### **2.3.2 ESP32 Cam Module**

The ESP32 Cam Module is a compact camera module capable of capturing images and streaming video over Wi-Fi. It integrates an ESP32 microcontroller and an OV2640 camera sensor, offering easy connectivity and high-quality imaging for mini surveillance or IoT projects.

                                                                  *Figure 2.3.2: ESP32 Cam Module*

### **2.3.3 Motor Driver**

A motor driver is an electronic circuit or device that controls the speed, direction, and torque of an electric motor. The use of a motor driver is essential in many applications where precise motor control is required. It is essential in many applications, including robotics, automation, and industrial control systems. In our project, we are using a driver motor to accelerate the wheels of the robot.

                             		  *Figure 2.3.3* Motor Driver

**2.3.4 Wheels and Chassis**

             Wheels and chassis are two important components of a vehicle that work together to provide stability, maneuverability, and support. Wheels are round structures that are typically made of metal or rubber and are attached to the vehicle's axles. The chassis is the framework of a vehicle that supports the body and engine.

In our project for making the main body of the robot, we are requiring 4 wheels and chassis  
 _Figure 2.3.4:_ Wheels and Chassis

### **2.3.5 DC Motor**

A DC (direct current) motor is a type of electric motor that converts electrical energy into mechanical energy through the use of a magnetic field. It operates by applying a voltage to the motor's terminals, which creates a magnetic field that interacts with the motor's armature, causing it to rotate. DC motors are commonly used in robotics and automation applications because they can be easily controlled and provide high torque at low speeds. They are also relatively simple and inexpensive compared to other types of motors. To use a DC motor in a hand gesture control robot, we need a motor driver circuit that can provide the appropriate voltage and current to the motor.

_Figure 2\. DC Motor_

_Figure 2\. DC Motor_

3. ### **Software Required**

   For coding and uploading the sketch, the Arduino IDE is used.

   4. ### **Flow Chart**

      **![][image2]**

      _Fig. 2.4.1 Flow Chart_

# **CHAPTER 3**

3. ### **Design and Implementation**

   1. **Components**

   #### Required Components for the schematic:

   - Arduino Uno.
   - ESP32 Cam Module
   - L298N Motor Driver
   - DC Motor
   - Connectors to join the different boards to form one functional device. Each of the hardware is dissected and was designed/implemented separately for its functionality and later incorporated as one whole application. This helped in the debugging processes. We can prepare by using this.

   2. ### **Hardware**

      _Fig. 3.2 Hardware Implementation_

4. ### **Working of Surveillance Robot using ESP 32 Cam:**

The surveillance robot car using ESP32 Cam module operates based on the following principles:

1. **Image and Video Capture (ESP32 Cam Module):**

   The ESP32 Cam module captures images and videos using its onboard camera sensor.

   It processes the captured media to extract visual information about the surroundings.

2. **Integration and Control:**

   The ESP32 Cam module is integrated into the surveillance robot car's control system.

3. **Data Processing and Transmission:**

   The captured images and videos are processed by the ESP32 microcontroller for surveillance analysis.

   Relevant data or alerts may be generated based on detected events or anomalies.

   The data may be transmitted wirelessly for remote monitoring and analysis, using Wi-Fi or other communication protocols supported by the ESP32.

Overall, the ESP32 Cam module captures visual data, allowing the surveillance robot car to monitor its surroundings effectively. This integrated system enables autonomous surveillance capabilities with potential applications in home security, monitoring, and surveillance tasks.

# **CHAPTER 4**

1. ### **Applications**

1. **Home Security:** Patrol homes and properties, detecting intruders and sending alerts to homeowners.
1. **Commercial Security:** Monitor business premises, warehouses, or retail stores to deter theft and unauthorized access.
1. **Search and Rescue:** Assist in disaster scenarios by navigating hazardous terrain to locate survivors and assess damage.
1. **Monitoring and Inspection:** Provide real-time feedback in environments like construction sites or industrial facilities to identify issues quickly.
1. **Education and Research:** Enhance learning in classrooms by teaching robotics, computer vision, and IoT concepts. Also, support research in robotics, artificial intelligence, and environmental monitoring.

   2. ### **Advantages**

1. **Compact Size**: ESP32 Cam's small design enables robots to navigate tight spaces effectively.
1. **Affordability:** It's cost-effective, making surveillance accessible to a wide range of users.
1. **Wireless Connectivity**: Supports Wi-Fi for remote monitoring and live video streaming.
1. **High-Quality Imaging:** Equipped with an OV2640 sensor for clear and detailed visuals.
1. **Versatility:** Customizable with additional sensors for various surveillance tasks.

### **4.3. Disadvantages**

1. **Limited range and accuracy:** The range and accuracy of hand gestures can be limited, which may make it difficult to control the robot precisely or from a distance.

   2. **Limited range of motion:** Hand gestures can only control the robot within a limited range of motion. This can make it difficult to control the robot in certain situations or to perform complex tasks

### **4.4 Conclusion**

In conclusion, the surveillance robot using ESP32-CAM is a promising project that can be used for various applications, such as monitoring and surveillance of homes or offices. The ESP32-CAM module provides a compact and cost-effective solution for capturing images and streaming video. By integrating it with a robot platform, it is possible to remotely control the robot and view live video footage from the camera. With the increasing demand for remote monitoring and surveillance. This project demonstrates the power of technological innovation in addressing contemporary challenges and contributing to a smarter and safer environment.

It has the potential to provide a practical and affordable solution for many different scenarios.

# **COST ESTIMATION**

| Sr. No.       | Name of Component | Quantity | Price Rs.  |
| ------------- | ----------------- | -------- | ---------- |
| **1**         | Arduino UNO       | 1        | **789**    |
| **2**         | Driver motor      | 1        | **115**    |
| **3**         | 3.7V Li battery   | 3        | **180**    |
| **4**         | Wheels            | 4        | **180**    |
| **5**         | DC motors         | 4        | **200**    |
| **6**         | ESP32 CAM module  | 1        | **472**    |
|               |                   |          |            |
| **Total Rs.** |                   |          | **1936/-** |

_Table 2: Cost of Project_

# **APPENDICES**

### **Program Code:**

#### ESP32CAM_Car.ino

\#include "esp_camera.h"  
\#include \<WiFi.h\>

\#define CAMERA_MODEL_AI_THINKER

const char\* ssid \= "WIFI_NAME";  
const char\* password \= "WIFI_PASSWORD";

\#if defined(CAMERA_MODEL_WROVER_KIT)  
\#define PWDN_GPIO_NUM \-1  
\#define RESET_GPIO_NUM \-1  
\#define XCLK_GPIO_NUM 21  
\#define SIOD_GPIO_NUM 26  
\#define SIOC_GPIO_NUM 27

\#define Y9_GPIO_NUM 35  
\#define Y8_GPIO_NUM 34  
\#define Y7_GPIO_NUM 39  
\#define Y6_GPIO_NUM 36  
\#define Y5_GPIO_NUM 19  
\#define Y4_GPIO_NUM 18  
\#define Y3_GPIO_NUM 5  
\#define Y2_GPIO_NUM 4  
\#define VSYNC_GPIO_NUM 25  
\#define HREF_GPIO_NUM 23  
\#define PCLK_GPIO_NUM 22

\#elif defined(CAMERA_MODEL_AI_THINKER)  
\#define PWDN_GPIO_NUM 32  
\#define RESET_GPIO_NUM \-1  
\#define XCLK_GPIO_NUM 0  
\#define SIOD_GPIO_NUM 26  
\#define SIOC_GPIO_NUM 27

\#define Y9_GPIO_NUM 35  
\#define Y8_GPIO_NUM 34  
\#define Y7_GPIO_NUM 39  
\#define Y6_GPIO_NUM 36  
\#define Y5_GPIO_NUM 21  
\#define Y4_GPIO_NUM 19  
\#define Y3_GPIO_NUM 18  
\#define Y2_GPIO_NUM 5  
\#define VSYNC_GPIO_NUM 25  
\#define HREF_GPIO_NUM 23  
\#define PCLK_GPIO_NUM 22

\#else  
\#error "Camera model not selected"  
\#endif

// GPIO Setting  
extern int gpLb \= 2; // Left 1  
extern int gpLf \= 14; // Left 2  
extern int gpRb \= 15; // Right 1  
extern int gpRf \= 13; // Right 2  
extern int gpLed \= 4; // Light  
extern String WiFiAddr \="";

void startCameraServer();

void setup() {  
 Serial.begin(115200);  
 Serial.setDebugOutput(true);  
 Serial.println();

pinMode(gpLb, OUTPUT); //Left Backward  
 pinMode(gpLf, OUTPUT); //Left Forward  
 pinMode(gpRb, OUTPUT); //Right Forward  
 pinMode(gpRf, OUTPUT); //Right Backward  
 pinMode(gpLed, OUTPUT); //Light

//initialize  
 digitalWrite(gpLb, LOW);  
 digitalWrite(gpLf, LOW);  
 digitalWrite(gpRb, LOW);  
 digitalWrite(gpRf, LOW);  
 digitalWrite(gpLed, LOW);

camera_config_t config;  
 config.ledc_channel \= LEDC_CHANNEL_0;  
 config.ledc_timer \= LEDC_TIMER_0;  
 config.pin_d0 \= Y2_GPIO_NUM;  
 config.pin_d1 \= Y3_GPIO_NUM;  
 config.pin_d2 \= Y4_GPIO_NUM;  
 config.pin_d3 \= Y5_GPIO_NUM;  
 config.pin_d4 \= Y6_GPIO_NUM;  
 config.pin_d5 \= Y7_GPIO_NUM;  
 config.pin_d6 \= Y8_GPIO_NUM;  
 config.pin_d7 \= Y9_GPIO_NUM;  
 config.pin_xclk \= XCLK_GPIO_NUM;  
 config.pin_pclk \= PCLK_GPIO_NUM;  
 config.pin_vsync \= VSYNC_GPIO_NUM;  
 config.pin_href \= HREF_GPIO_NUM;  
 config.pin_sscb_sda \= SIOD_GPIO_NUM;  
 config.pin_sscb_scl \= SIOC_GPIO_NUM;  
 config.pin_pwdn \= PWDN_GPIO_NUM;  
 config.pin_reset \= RESET_GPIO_NUM;  
 config.xclk_freq_hz \= 20000000;  
 config.pixel_format \= PIXFORMAT_JPEG;  
 //init with high specs to pre-allocate larger buffers  
 if(psramFound()){  
 config.frame_size \= FRAMESIZE_UXGA;  
 config.jpeg_quality \= 10;  
 config.fb_count \= 2;  
 } else {  
 config.frame_size \= FRAMESIZE_SVGA;  
 config.jpeg_quality \= 12;  
 config.fb_count \= 1;  
 }

// camera init  
 esp_err_t err \= esp_camera_init(\&config);  
 if (err \!= ESP_OK) {  
 Serial.printf("Camera init failed with error 0x%x", err);  
 return;  
 }

//drop down frame size for higher initial frame rate  
 sensor_t \* s \= esp_camera_sensor_get();  
 s-\>set_framesize(s, FRAMESIZE_CIF);

WiFi.begin(ssid, password);

while (WiFi.status() \!= WL_CONNECTED) {  
 delay(500);  
 Serial.print(".");  
 }  
 Serial.println("");  
 Serial.println("WiFi connected");

startCameraServer();

Serial.print("Camera Ready\! Use 'http://");  
 Serial.print(WiFi.localIP());  
 WiFiAddr \= WiFi.localIP().toString();  
 Serial.println("' to connect");  
}

void loop() {  
 // put your main code here, to run repeatedly:

}

App_http.cpp**:**

\#include "esp_http_server.h"  
\#include "esp_timer.h"  
\#include "esp_camera.h"  
\#include "img_converters.h"  
\#include "camera_index.h"  
\#include "Arduino.h"

extern int gpLb;  
extern int gpLf;  
extern int gpRb;  
extern int gpRf;  
extern int gpLed;  
extern String WiFiAddr;

void WheelAct(int nLf, int nLb, int nRf, int nRb);

typedef struct {  
 size_t size; //number of values used for filtering  
 size_t index; //current value index  
 size_t count; //value count  
 int sum;  
 int \* values; //array to be filled with values  
} ra_filter_t;

typedef struct {  
 httpd_req_t \*req;  
 size_t len;  
} jpg_chunking_t;

\#define PART_BOUNDARY "123456789000000000000987654321"  
static const char\* \_STREAM_CONTENT_TYPE \= "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;  
static const char\* \_STREAM_BOUNDARY \= "\\r\\n--" PART_BOUNDARY "\\r\\n";  
static const char\* \_STREAM_PART \= "Content-Type: image/jpeg\\r\\nContent-Length: %u\\r\\n\\r\\n";

static ra_filter_t ra_filter;  
httpd_handle_t stream_httpd \= NULL;  
httpd_handle_t camera_httpd \= NULL;

static ra_filter_t \* ra_filter_init(ra_filter_t \* filter, size_t sample_size){  
 memset(filter, 0, sizeof(ra_filter_t));

    filter-\>values \= (int \*)malloc(sample\_size \* sizeof(int));
    if(\!filter-\>values){
        return NULL;
    }
    memset(filter-\>values, 0, sample\_size \* sizeof(int));

    filter-\>size \= sample\_size;
    return filter;

}

static int ra_filter_run(ra_filter_t \* filter, int value){  
 if(\!filter-\>values){  
 return value;  
 }  
 filter-\>sum \-= filter-\>values\[filter-\>index\];  
 filter-\>values\[filter-\>index\] \= value;  
 filter-\>sum \+= filter-\>values\[filter-\>index\];  
 filter-\>index++;  
 filter-\>index \= filter-\>index % filter-\>size;  
 if (filter-\>count \< filter-\>size) {  
 filter-\>count++;  
 }  
 return filter-\>sum / filter-\>count;  
}

static size_t jpg_encode_stream(void \* arg, size_t index, const void\* data, size_t len){  
 jpg_chunking_t \*j \= (jpg_chunking_t \*)arg;  
 if(\!index){  
 j-\>len \= 0;  
 }  
 if(httpd_resp_send_chunk(j-\>req, (const char \*)data, len) \!= ESP_OK){  
 return 0;  
 }  
 j-\>len \+= len;  
 return len;  
}

static esp_err_t capture_handler(httpd_req_t \*req){  
 camera_fb_t \* fb \= NULL;  
 esp_err_t res \= ESP_OK;  
 int64_t fr_start \= esp_timer_get_time();

    fb \= esp\_camera\_fb\_get();
    if (\!fb) {
        Serial.printf("Camera capture failed");
        httpd\_resp\_send\_500(req);
        return ESP\_FAIL;
    }

    httpd\_resp\_set\_type(req, "image/jpeg");
    httpd\_resp\_set\_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");

    size\_t fb\_len \= 0;
    if(fb-\>format \== PIXFORMAT\_JPEG){
        fb\_len \= fb-\>len;
        res \= httpd\_resp\_send(req, (const char \*)fb-\>buf, fb-\>len);
    } else {
        jpg\_chunking\_t jchunk \= {req, 0};
        res \= frame2jpg\_cb(fb, 80, jpg\_encode\_stream, \&jchunk)?ESP\_OK:ESP\_FAIL;
        httpd\_resp\_send\_chunk(req, NULL, 0);
        fb\_len \= jchunk.len;
    }
    esp\_camera\_fb\_return(fb);
    int64\_t fr\_end \= esp\_timer\_get\_time();
    Serial.printf("JPG: %uB %ums", (uint32\_t)(fb\_len), (uint32\_t)((fr\_end \- fr\_start)/1000));
    return res;

}

static esp_err_t stream_handler(httpd_req_t \*req){  
 camera_fb_t \* fb \= NULL;  
 esp_err_t res \= ESP_OK;  
 size_t \_jpg_buf_len \= 0;  
 uint8_t \* \_jpg_buf \= NULL;  
 char \* part_buf\[64\];

    static int64\_t last\_frame \= 0;
    if(\!last\_frame) {
        last\_frame \= esp\_timer\_get\_time();
    }

    res \= httpd\_resp\_set\_type(req, \_STREAM\_CONTENT\_TYPE);
    if(res \!= ESP\_OK){
        return res;
    }

    while(true){
        fb \= esp\_camera\_fb\_get();
        if (\!fb) {
            Serial.printf("Camera capture failed");
            res \= ESP\_FAIL;
        } else {
            if(fb-\>format \!= PIXFORMAT\_JPEG){
                bool jpeg\_converted \= frame2jpg(fb, 80, &\_jpg\_buf, &\_jpg\_buf\_len);
                esp\_camera\_fb\_return(fb);
                fb \= NULL;
                if(\!jpeg\_converted){
                    Serial.printf("JPEG compression failed");
                    res \= ESP\_FAIL;
                }
            } else {
                \_jpg\_buf\_len \= fb-\>len;
                \_jpg\_buf \= fb-\>buf;
            }
        }
        if(res \== ESP\_OK){
            size\_t hlen \= snprintf((char \*)part\_buf, 64, \_STREAM\_PART, \_jpg\_buf\_len);
            res \= httpd\_resp\_send\_chunk(req, (const char \*)part\_buf, hlen);
        }
        if(res \== ESP\_OK){
            res \= httpd\_resp\_send\_chunk(req, (const char \*)\_jpg\_buf, \_jpg\_buf\_len);
        }
        if(res \== ESP\_OK){
            res \= httpd\_resp\_send\_chunk(req, \_STREAM\_BOUNDARY, strlen(\_STREAM\_BOUNDARY));
        }
        if(fb){
            esp\_camera\_fb\_return(fb);
            fb \= NULL;
            \_jpg\_buf \= NULL;
        } else if(\_jpg\_buf){
            free(\_jpg\_buf);
            \_jpg\_buf \= NULL;
        }
        if(res \!= ESP\_OK){
            break;
        }
        int64\_t fr\_end \= esp\_timer\_get\_time();

        int64\_t frame\_time \= fr\_end \- last\_frame;
        last\_frame \= fr\_end;
        frame\_time /= 1000;
        uint32\_t avg\_frame\_time \= ra\_filter\_run(\&ra\_filter, frame\_time);
        Serial.printf("MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps)"
            ,(uint32\_t)(\_jpg\_buf\_len),
            (uint32\_t)frame\_time, 1000.0 / (uint32\_t)frame\_time,
            avg\_frame\_time, 1000.0 / avg\_frame\_time
        );
    }

    last\_frame \= 0;
    return res;

}

static esp_err_t cmd_handler(httpd_req_t \*req){  
 char\* buf;  
 size_t buf_len;  
 char variable\[32\] \= {0,};  
 char value\[32\] \= {0,};

    buf\_len \= httpd\_req\_get\_url\_query\_len(req) \+ 1;
    if (buf\_len \> 1\) {
        buf \= (char\*)malloc(buf\_len);
        if(\!buf){
            httpd\_resp\_send\_500(req);
            return ESP\_FAIL;
        }
        if (httpd\_req\_get\_url\_query\_str(req, buf, buf\_len) \== ESP\_OK) {
            if (httpd\_query\_key\_value(buf, "var", variable, sizeof(variable)) \== ESP\_OK &&
                httpd\_query\_key\_value(buf, "val", value, sizeof(value)) \== ESP\_OK) {
            } else {
                free(buf);
                httpd\_resp\_send\_404(req);
                return ESP\_FAIL;
            }
        } else {
            free(buf);
            httpd\_resp\_send\_404(req);
            return ESP\_FAIL;
        }
        free(buf);
    } else {
        httpd\_resp\_send\_404(req);
        return ESP\_FAIL;
    }

    int val \= atoi(value);
    sensor\_t \* s \= esp\_camera\_sensor\_get();
    int res \= 0;

    if(\!strcmp(variable, "framesize")) {
        if(s-\>pixformat \== PIXFORMAT\_JPEG) res \= s-\>set\_framesize(s, (framesize\_t)val);
    }
    else if(\!strcmp(variable, "quality")) res \= s-\>set\_quality(s, val);
    else if(\!strcmp(variable, "contrast")) res \= s-\>set\_contrast(s, val);
    else if(\!strcmp(variable, "brightness")) res \= s-\>set\_brightness(s, val);
    else if(\!strcmp(variable, "saturation")) res \= s-\>set\_saturation(s, val);
    else if(\!strcmp(variable, "gainceiling")) res \= s-\>set\_gainceiling(s, (gainceiling\_t)val);
    else if(\!strcmp(variable, "colorbar")) res \= s-\>set\_colorbar(s, val);
    else if(\!strcmp(variable, "awb")) res \= s-\>set\_whitebal(s, val);
    else if(\!strcmp(variable, "agc")) res \= s-\>set\_gain\_ctrl(s, val);
    else if(\!strcmp(variable, "aec")) res \= s-\>set\_exposure\_ctrl(s, val);
    else if(\!strcmp(variable, "hmirror")) res \= s-\>set\_hmirror(s, val);
    else if(\!strcmp(variable, "vflip")) res \= s-\>set\_vflip(s, val);
    else if(\!strcmp(variable, "awb\_gain")) res \= s-\>set\_awb\_gain(s, val);
    else if(\!strcmp(variable, "agc\_gain")) res \= s-\>set\_agc\_gain(s, val);
    else if(\!strcmp(variable, "aec\_value")) res \= s-\>set\_aec\_value(s, val);
    else if(\!strcmp(variable, "aec2")) res \= s-\>set\_aec2(s, val);
    else if(\!strcmp(variable, "dcw")) res \= s-\>set\_dcw(s, val);
    else if(\!strcmp(variable, "bpc")) res \= s-\>set\_bpc(s, val);
    else if(\!strcmp(variable, "wpc")) res \= s-\>set\_wpc(s, val);
    else if(\!strcmp(variable, "raw\_gma")) res \= s-\>set\_raw\_gma(s, val);
    else if(\!strcmp(variable, "lenc")) res \= s-\>set\_lenc(s, val);
    else if(\!strcmp(variable, "special\_effect")) res \= s-\>set\_special\_effect(s, val);
    else if(\!strcmp(variable, "wb\_mode")) res \= s-\>set\_wb\_mode(s, val);
    else if(\!strcmp(variable, "ae\_level")) res \= s-\>set\_ae\_level(s, val);
    else {
        res \= \-1;
    }

    if(res){
        return httpd\_resp\_send\_500(req);
    }

    httpd\_resp\_set\_hdr(req, "Access-Control-Allow-Origin", "\*");
    return httpd\_resp\_send(req, NULL, 0);

}

static esp_err_t status_handler(httpd_req_t \*req){  
 static char json_response\[1024\];

    sensor\_t \* s \= esp\_camera\_sensor\_get();
    char \* p \= json\_response;
    \*p++ \= '{';

    p+=sprintf(p, "\\"framesize\\":%u,", s-\>status.framesize);
    p+=sprintf(p, "\\"quality\\":%u,", s-\>status.quality);
    p+=sprintf(p, "\\"brightness\\":%d,", s-\>status.brightness);
    p+=sprintf(p, "\\"contrast\\":%d,", s-\>status.contrast);
    p+=sprintf(p, "\\"saturation\\":%d,", s-\>status.saturation);
    p+=sprintf(p, "\\"special\_effect\\":%u,", s-\>status.special\_effect);
    p+=sprintf(p, "\\"wb\_mode\\":%u,", s-\>status.wb\_mode);
    p+=sprintf(p, "\\"awb\\":%u,", s-\>status.awb);
    p+=sprintf(p, "\\"awb\_gain\\":%u,", s-\>status.awb\_gain);
    p+=sprintf(p, "\\"aec\\":%u,", s-\>status.aec);
    p+=sprintf(p, "\\"aec2\\":%u,", s-\>status.aec2);
    p+=sprintf(p, "\\"ae\_level\\":%d,", s-\>status.ae\_level);
    p+=sprintf(p, "\\"aec\_value\\":%u,", s-\>status.aec\_value);
    p+=sprintf(p, "\\"agc\\":%u,", s-\>status.agc);
    p+=sprintf(p, "\\"agc\_gain\\":%u,", s-\>status.agc\_gain);
    p+=sprintf(p, "\\"gainceiling\\":%u,", s-\>status.gainceiling);
    p+=sprintf(p, "\\"bpc\\":%u,", s-\>status.bpc);
    p+=sprintf(p, "\\"wpc\\":%u,", s-\>status.wpc);
    p+=sprintf(p, "\\"raw\_gma\\":%u,", s-\>status.raw\_gma);
    p+=sprintf(p, "\\"lenc\\":%u,", s-\>status.lenc);
    p+=sprintf(p, "\\"hmirror\\":%u,", s-\>status.hmirror);
    p+=sprintf(p, "\\"dcw\\":%u,", s-\>status.dcw);
    p+=sprintf(p, "\\"colorbar\\":%u", s-\>status.colorbar);
    \*p++ \= '}';
    \*p++ \= 0;
    httpd\_resp\_set\_type(req, "application/json");
    httpd\_resp\_set\_hdr(req, "Access-Control-Allow-Origin", "\*");
    return httpd\_resp\_send(req, json\_response, strlen(json\_response));

}

static esp_err_t index_handler(httpd_req_t \*req){  
 httpd_resp_set_type(req, "text/html");  
 String page \= "";  
 page \+= "\<meta name=\\"viewport\\" content=\\"width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=0\\"\>\\n";  
 page \+= "\<script\>var xhttp \= new XMLHttpRequest();\</script\>";  
 page \+= "\<script\>function getsend(arg) { xhttp.open('GET', arg \+'?' \+ new Date().getTime(), true); xhttp.send() } \</script\>";  
 //page \+= "\<p align=center\>\<IMG SRC='http://" \+ WiFiAddr \+ ":81/stream' style='width:280px;'\>\</p\>\<br/\>\<br/\>";  
 page \+= "\<p align=center\>\<IMG SRC='http://" \+ WiFiAddr \+ ":81/stream' style='width:300px; transform:rotate(180deg);'\>\</p\>\<br/\>\<br/\>";

page \+= "\<p align=center\> \<button style=background-color:lightgrey;width:90px;height:80px onmousedown=getsend('go') onmouseup=getsend('stop') ontouchstart=getsend('go') ontouchend=getsend('stop') \>\<b\>Forward\</b\>\</button\> \</p\>";  
 page \+= "\<p align=center\>";  
 page \+= "\<button style=background-color:lightgrey;width:90px;height:80px; onmousedown=getsend('left') onmouseup=getsend('stop') ontouchstart=getsend('left') ontouchend=getsend('stop')\>\<b\>Left\</b\>\</button\>\&nbsp;";  
 page \+= "\<button style=background-color:indianred;width:90px;height:80px onmousedown=getsend('stop') onmouseup=getsend('stop')\>\<b\>Stop\</b\>\</button\>\&nbsp;";  
 page \+= "\<button style=background-color:lightgrey;width:90px;height:80px onmousedown=getsend('right') onmouseup=getsend('stop') ontouchstart=getsend('right') ontouchend=getsend('stop')\>\<b\>Right\</b\>\</button\>";  
 page \+= "\</p\>";

page \+= "\<p align=center\>\<button style=background-color:lightgrey;width:90px;height:80px onmousedown=getsend('back') onmouseup=getsend('stop') ontouchstart=getsend('back') ontouchend=getsend('stop') \>\<b\>Backward\</b\>\</button\>\</p\>";

page \+= "\<p align=center\>";  
 page \+= "\<button style=background-color:yellow;width:140px;height:40px onmousedown=getsend('ledon')\>\<b\>Light ON\</b\>\</button\>";  
 page \+= "\<button style=background-color:yellow;width:140px;height:40px onmousedown=getsend('ledoff')\>\<b\>Light OFF\</b\>\</button\>";  
 page \+= "\</p\>";

    return httpd\_resp\_send(req, \&page\[0\], strlen(\&page\[0\]));

}

static esp_err_t go_handler(httpd_req_t \*req){  
 WheelAct(HIGH, LOW, HIGH, LOW);  
 Serial.println("Go");  
 httpd_resp_set_type(req, "text/html");  
 return httpd_resp_send(req, "OK", 2);  
}  
static esp_err_t back_handler(httpd_req_t \*req){  
 WheelAct(LOW, HIGH, LOW, HIGH);  
 Serial.println("Back");  
 httpd_resp_set_type(req, "text/html");  
 return httpd_resp_send(req, "OK", 2);  
}

static esp_err_t left_handler(httpd_req_t \*req){  
 WheelAct(LOW, HIGH, HIGH, LOW);  
 Serial.println("Right");  
 httpd_resp_set_type(req, "text/html");  
 return httpd_resp_send(req, "OK", 2);  
}  
static esp_err_t right_handler(httpd_req_t \*req){

    WheelAct(HIGH, LOW, LOW, HIGH);
    Serial.println("Left");
    httpd\_resp\_set\_type(req, "text/html");
    return httpd\_resp\_send(req, "OK", 2);

}

static esp_err_t stop_handler(httpd_req_t \*req){  
 WheelAct(LOW, LOW, LOW, LOW);  
 Serial.println("Stop");  
 httpd_resp_set_type(req, "text/html");  
 return httpd_resp_send(req, "OK", 2);  
}

static esp_err_t ledon_handler(httpd_req_t \*req){  
 digitalWrite(gpLed, HIGH);  
 Serial.println("LED ON");  
 httpd_resp_set_type(req, "text/html");  
 return httpd_resp_send(req, "OK", 2);  
}  
static esp_err_t ledoff_handler(httpd_req_t \*req){  
 digitalWrite(gpLed, LOW);  
 Serial.println("LED OFF");  
 httpd_resp_set_type(req, "text/html");  
 return httpd_resp_send(req, "OK", 2);  
}

void startCameraServer(){  
 httpd_config_t config \= HTTPD_DEFAULT_CONFIG();

    httpd\_uri\_t go\_uri \= {
        .uri       \= "/go",
        .method    \= HTTP\_GET,
        .handler   \= go\_handler,
        .user\_ctx  \= NULL
    };

    httpd\_uri\_t back\_uri \= {
        .uri       \= "/back",
        .method    \= HTTP\_GET,
        .handler   \= back\_handler,
        .user\_ctx  \= NULL
    };

    httpd\_uri\_t stop\_uri \= {
        .uri       \= "/stop",
        .method    \= HTTP\_GET,
        .handler   \= stop\_handler,
        .user\_ctx  \= NULL
    };

    httpd\_uri\_t left\_uri \= {
        .uri       \= "/left",
        .method    \= HTTP\_GET,
        .handler   \= left\_handler,
        .user\_ctx  \= NULL
    };

    httpd\_uri\_t right\_uri \= {
        .uri       \= "/right",
        .method    \= HTTP\_GET,
        .handler   \= right\_handler,
        .user\_ctx  \= NULL
    };

    httpd\_uri\_t ledon\_uri \= {
        .uri       \= "/ledon",
        .method    \= HTTP\_GET,
        .handler   \= ledon\_handler,
        .user\_ctx  \= NULL
    };

    httpd\_uri\_t ledoff\_uri \= {
        .uri       \= "/ledoff",
        .method    \= HTTP\_GET,
        .handler   \= ledoff\_handler,
        .user\_ctx  \= NULL
    };

    httpd\_uri\_t index\_uri \= {
        .uri       \= "/",
        .method    \= HTTP\_GET,
        .handler   \= index\_handler,
        .user\_ctx  \= NULL
    };

    httpd\_uri\_t status\_uri \= {
        .uri       \= "/status",
        .method    \= HTTP\_GET,
        .handler   \= status\_handler,
        .user\_ctx  \= NULL
    };

    httpd\_uri\_t cmd\_uri \= {
        .uri       \= "/control",
        .method    \= HTTP\_GET,
        .handler   \= cmd\_handler,
        .user\_ctx  \= NULL
    };

    httpd\_uri\_t capture\_uri \= {
        .uri       \= "/capture",
        .method    \= HTTP\_GET,
        .handler   \= capture\_handler,
        .user\_ctx  \= NULL
    };

httpd_uri_t stream_uri \= {  
 .uri \= "/stream",  
 .method \= HTTP_GET,  
 .handler \= stream_handler,  
 .user_ctx \= NULL  
 };

    ra\_filter\_init(\&ra\_filter, 20);
    Serial.printf("Starting web server on port: '%d'", config.server\_port);
    if (httpd\_start(\&camera\_httpd, \&config) \== ESP\_OK) {
        httpd\_register\_uri\_handler(camera\_httpd, \&index\_uri);
        httpd\_register\_uri\_handler(camera\_httpd, \&go\_uri);
        httpd\_register\_uri\_handler(camera\_httpd, \&back\_uri);
        httpd\_register\_uri\_handler(camera\_httpd, \&stop\_uri);
        httpd\_register\_uri\_handler(camera\_httpd, \&left\_uri);
        httpd\_register\_uri\_handler(camera\_httpd, \&right\_uri);
        httpd\_register\_uri\_handler(camera\_httpd, \&ledon\_uri);
        httpd\_register\_uri\_handler(camera\_httpd, \&ledoff\_uri);
    }

    config.server\_port \+= 1;
    config.ctrl\_port \+= 1;
    Serial.printf("Starting stream server on port: '%d'", config.server\_port);
    if (httpd\_start(\&stream\_httpd, \&config) \== ESP\_OK) {
        httpd\_register\_uri\_handler(stream\_httpd, \&stream\_uri);
    }

}

void WheelAct(int nLf, int nLb, int nRf, int nRb)  
{  
 digitalWrite(gpLf, nLf);  
 digitalWrite(gpLb, nLb);  
 digitalWrite(gpRf, nRf);  
 digitalWrite(gpRb, nRb);  
}

# **References** {#references}

1. Montrose, M. I. (1996). Printed circuit board and design techniques for EMC compliance (Vol. 1, p. 996). Piscataway, NJ: IEEE Press.
2. Forouzan, B. A., & Fegan, S. C. (2006). TCP/IP protocol suite (Vol. 2). McGrawHill.
3. Balakrishnan, H., Stemm, M., Seshan, S., & Katz, R. H. (1997). Analyzing stability in wide-area network
4. performance. ACM SIGMETRICS Performance and Evaluation Review, 25(1), 2-12.
5. Briscoe, N. (2000). Understanding the OSI 7-layer model. PC Network Advisor, 120(2).
6. G.Huston, Analyzing the Internet BGP routing table, Internet ProtocolJournal 4 (1) (2001).
7. McKeown, N. (2009). Software-defined networking. INFOCOM keynote talk, 17(2), 30-32.
8. https://components101.com/microcontrollers/arduino-uno https://w98ujBhCgARIsAD7QeAhulaq0nKHQe9Tv1hL4a9KacWRvE2yUbH7dRY0wIJLfBpKXWlUC4waAt6qEALw_wcB
9. https://www.verical.com/datasheet/adafruit-brushless-dc-motors-

[image1]: data:image/png;base64,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
[image2]: data:image/png;base64,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
