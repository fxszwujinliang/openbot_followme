#include <Encoder.h>    //电机编码器，我用的霍尔编码器，因此这个文件我修改为使用c++编写的编码器驱动
   
// ---------------------------------------------------------------------------
// This Arduino sketch accompanies the OpenBot Android application. 
// By MoKamLeung October 2021
//
// The sketch has the following functinonalities:
//  - receive control commands from Android application (USB serial)
//. - produce low-level controls (PWM) for the vehicle
//  - toggle left and right indicator signals 
//  - wheel odometry based on optical speed sensors
//  - estimate battery voltage via voltage divider
//  - estimate distance based on sonar sensor 
//  - send sensor readings to Android application (USB serial)
//  - display vehicle status on OLED
//
// Dependencies: Install via "Tools --> Manage Libraries" (type library name in the search field)
//  - Interrupts: PinChangeInterrupt by Nico Hood (read speed sensors and sonar)
//  - OLED: Adafruit_SSD1306 & Adafruit_GFX (display vehicle status)
// Contributors:
//  - October 2020: OLED display support by Ingmar Stapel
// ---------------------------------------------------------------------------

// PIN_PWM_L1,PIN_PWM_L2,PIN_PWM_R1,PIN_PWM_R2  Low-level control of left DC motors via PWM 
// PIN_SPEED_L, PIN_SPEED_R                     Measure left and right wheel speed
// PIN_VIN                                      Measure battery voltage via voltage divider
// PIN_TRIGGER                                  Arduino pin tied to trigger pin on ultrasonic sensor.
// PIN_ECHO                                     Arduino pin tied to echo pin on ultrasonic sensor.
// PIN_LED_LB, PIN_LED_RB                       Toggle left and right rear LEDs (indicator signals) 

//------------------------------------------------------//
// DEFINITIONS
//------------------------------------------------------//

// DO NOT CHANGE!
#define DIY 0    


//------------------------------------------------------//
//SETTINGS
//------------------------------------------------------//

// Setup the OpenBot version (DIY,PCB_V1,PCB_V2)
#define OPENBOT DIY    //有这个宏定义，确保下面if语句判断为true

// Enable/Disable voltage divider (1,0)
#define HAS_VOLTAGE_DIVIDER 0     //无分压检测功能

// Enable/Disable indicators (1,0)
#define HAS_INDICATORS 0   //无显示模块

// Enable/Disable speed sensors (1,0)
#define HAS_SPEED_SENSORS 0  //这里的速度检测指的是利用码盘和激光测速模块自己编写代码测速，我的小车用的Encoder马达自带的正交编码器，因此这里定义为0

// Enable/Disable sonar (1,0) 声呐模块，超声波检测，如果没有配置该模块则电机将始终全速运行，速度不受控。
#define HAS_SONAR 1 

// Enable/Disable median filter for sonar measurements (1,0) 声呐检测的滤波算法，即把接收到的信号数据提取中间值，避免误差大.
#define USE_MEDIAN 1

// Enable/Disable OLED (1,0)
#define HAS_OLED 0   //无显示模块

// Enable/Disable no phone mode (1,0)
// In no phone mode:
// - the motors will turn at 75% speed
// - the speed will be reduced if an obstacle is detected by the sonar sensor
// - the car will turn, if an obstacle is detected within STOP_THRESHOLD
// WARNING: If the sonar sensor is not setup, the car will go full speed forward!
#define NO_PHONE_MODE 0    //这个宏定义是指不使用手机控制小车.我的小车需要手机进行对象跟踪的，这个宏定义为0.

//------------------------------------------------------//
// PINOUT
//------------------------------------------------------//

//Setup the pin definitions
#if (OPENBOT == DIY)
  #define PIN_PWM_L 5     //左电机的PWM引脚
  #define PIN_DIR_L 6    //左电机的Direction转向控制引脚 
  #define PIN_PWM_R 9    //右电机PWM引脚
  #define PIN_DIR_R 10    //右电机的转向
  //#define PIN_SPEED_L 2
  //#define PIN_SPEED_R 3
  //#define PIN_VIN A7
  #define PIN_TRIGGER 12   //超声波触发
  #define PIN_ECHO 11    //超声波接收
  //#define PIN_LED_LB 4  //LED显示
 // #define PIN_LED_RB 7

#endif

//------------------------------------------------------//
// INITIALIZATION
//------------------------------------------------------//

#include <limits.h>
const unsigned int STOP_THRESHOLD = 12; //cm

#if NO_PHONE_MODE
  int turn_direction = 0; // right
  const unsigned long TURN_DIRECTION_INTERVAL = 2000; // How frequently to change turn direction (ms).
  unsigned long turn_direction_timeout = 0;   // After timeout (ms), random turn direction is updated.
#endif

#if HAS_SPEED_SENSORS or HAS_SONAR
  #include <PinChangeInterrupt.h>
#endif

#if HAS_SONAR
  //Sonar sensor
  const float US_TO_CM = 0.01715; //cm/uS -> (343 * 100 / 1000000) / 2;
  const unsigned long PING_INTERVAL = 100; // How frequently to send out a ping (ms).
  unsigned long ping_timeout = 0;   // After timeout (ms), distance is set to maximum.
  volatile unsigned long start_time = 0; // Time when sending sonar pulse was sent
  volatile unsigned long echo_time = 0; // Time taken to receive echo
  unsigned int distance = UINT_MAX; //cm      65535 UINT_MAX
  unsigned int distance_estimate = UINT_MAX; //cm
  #if USE_MEDIAN
    const unsigned int distance_array_sz = 3;
    unsigned int distance_array[distance_array_sz]={};
    unsigned int distance_counter = 0;
  #endif
#else
  unsigned int distance_estimate = UINT_MAX; //cm
#endif

#if HAS_OLED
  #include <SPI.h>
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  
  const int OLED_RESET = -1; // not used
  Adafruit_SSD1306 display(OLED_RESET);
  
  // OLED Display SSD1306
  const unsigned int SCREEN_WIDTH = 128; // OLED display width, in pixels
  const unsigned int SCREEN_HEIGHT = 32; // OLED display height, in pixels
#endif

#if HAS_VOLTAGE_DIVIDER
  const unsigned int ADC_MAX = 1023;
  const unsigned int VREF = 5;
  //The voltage divider factor is computed as (R1+R2)/R2
  #if (OPENBOT == PCB_V1)
    const float VOLTAGE_DIVIDER_FACTOR = (100+33)/33;
  #else //DIY and PCB_V2
    const float VOLTAGE_DIVIDER_FACTOR = (20+10)/10;
  #endif
#endif

//Vehicle Control
int ctrl_left = 0;
int ctrl_right = 0;
//encoder measurement 编码器测速模块的变量定义
int counter_left = 0;
int counter_right = 0;
int oldstate_left = 0;
int oldstate_right = 0;
int newstate_left = 0;
int newstate_right = 0;

//创建Encoder对象，构造函数的传参就是编码器的AB两项输入引脚
Encoder LeftEnc(2, 3);   //这俩引脚需要有中断功能才行, Arduino Mega 2560 的中断引脚:2 (interrupt 0), 3 (interrupt 1),18 (interrupt 5), 19 (interrupt 4), 20 (interrupt 3), 21 (interrupt 2)
Encoder RightEnc(7, 8);  //括号内是中断号：18 (interrupt 5), 19 (interrupt 4)，看了库源码，好像不一定需要中断的（本次用的Arduino  nano）

long LoldPosition  = -999;
long RoldPosition  = -999;

float lrpm_factor = 0.014; // converting from quadrature encoder to equivalent optical wheel readings
float rrpm_factor = 0.01;


//Voltage measurement
const unsigned int VIN_ARR_SZ = 10;
unsigned int vin_counter = 0;
unsigned int vin_array[VIN_ARR_SZ]={0};


//Indicator Signal
const unsigned long INDICATOR_INTERVAL = 500; //Blinking rate of the indicator signal (ms).
unsigned long indicator_timeout = 0;
int indicator_val = 0;

//Serial communication
const unsigned long SEND_INTERVAL = 1000; // How frequently vehicle data is sent (ms).
unsigned long send_timeout = 0;
String inString = "";

//------------------------------------------------------//
// SETUP
//------------------------------------------------------//

void setup()
{
  //Outputs
  pinMode(PIN_PWM_L,OUTPUT);
  pinMode(PIN_DIR_L,OUTPUT);
  pinMode(PIN_PWM_R,OUTPUT);
  pinMode(PIN_DIR_R,OUTPUT);
  //pinMode(PIN_LED_LB,OUTPUT);
  //pinMode(PIN_LED_RB,OUTPUT);

  //Inputs
  //pinMode(PIN_VIN,INPUT);       
  //pinMode(PIN_SPEED_L,INPUT);  //本项目的速度检测用Encoder库，这里不必设置
  //pinMode(PIN_SPEED_R,INPUT);
  
  Serial.begin(115200,SERIAL_8N1); //8 data bits, no parity, 1 stop bit
  send_timeout = millis() + SEND_INTERVAL; //wait for one interval to get readings
  
  
  
  //Initialize with the I2C addr 0x3C
  #if HAS_OLED
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  #endif
  
  //Test sequence for indicator LEDs
  #if HAS_INDICATORS
    digitalWrite(PIN_LED_LB,LOW);
    digitalWrite(PIN_LED_RB,LOW);
    delay(500);
    digitalWrite(PIN_LED_LB,HIGH);
    delay(500);
    digitalWrite(PIN_LED_LB,LOW);
    digitalWrite(PIN_LED_RB,HIGH);
    delay(500);
    digitalWrite(PIN_LED_RB,LOW);
  #endif
}

//------------------------------------------------------//
// MAIN LOOP
//------------------------------------------------------//

void loop() {
  
  #if HAS_VOLTAGE_DIVIDER
    //Measure voltage
    vin_array[vin_counter%VIN_ARR_SZ] = analogRead(PIN_VIN);
    vin_counter++;
  #endif
  
  #if HAS_SONAR    //没有使用库函数，这里是自己写的算法
    //Measure distance every PING_INTERVAL
    if (millis() >= ping_timeout) {
      ping_timeout = ping_timeout + PING_INTERVAL;
      if (echo_time == 0) { //No echo received
        distance = UINT_MAX;
      }
      else
      {
        distance = echo_time * US_TO_CM;
      }
      #if USE_MEDIAN
        distance_array[distance_counter%distance_array_sz] = distance;
        distance_counter++;
        distance_estimate = get_median(distance_array, distance_array_sz);
      #else
        distance_estimate = distance;
      #endif
      send_ping();  //超声波测距，不用库，相当于重写，还用了模拟中断。函数定义在下面
    }
  #endif
  
  // Send vehicle measurments to serial every SEND_INTERVAL
  if (millis() >= send_timeout) {
    send_vehicle_data();
    send_timeout = millis() + SEND_INTERVAL;
  }

  #if HAS_INDICATORS
    // Check indicator signal every INDICATOR_INTERVAL
    if (millis() >= indicator_timeout) {
      update_indicators();
      indicator_timeout = millis() + INDICATOR_INTERVAL;
    }
  #endif
  
  #if (NO_PHONE_MODE)
    if (millis() > turn_direction_timeout)
    {
      turn_direction_timeout = millis() + TURN_DIRECTION_INTERVAL;
      turn_direction = random(2); //Generate random number in the range [0,1]
    }
    // drive forward
    if (distance_estimate > 4*STOP_THRESHOLD) {
      ctrl_left = distance_estimate;
      ctrl_right = ctrl_left;
      digitalWrite(PIN_LED_LB, LOW);
      digitalWrite(PIN_LED_RB, LOW);
    }
    // turn slightly  轻轻地转弯
    else if (distance_estimate > 2*STOP_THRESHOLD) {
      ctrl_left = distance_estimate;
      ctrl_right = ctrl_left - 3*STOP_THRESHOLD;
    }
    // turn strongly  强烈转弯
    else if (distance_estimate > STOP_THRESHOLD) {
      ctrl_left = 192;
      ctrl_right = - 192;
    }
    // drive backward slowly  慢慢向后
    else {   
        ctrl_left = -96;
        ctrl_right = -96;
        digitalWrite(PIN_LED_LB, HIGH);
        digitalWrite(PIN_LED_RB, HIGH);
    }
    // flip controls if needed and set indicator light
    if (ctrl_left != ctrl_right) {
      if (turn_direction > 0) {
        int temp = ctrl_left;
        ctrl_left = ctrl_right;
        ctrl_right = temp;
        digitalWrite(PIN_LED_LB, HIGH);
        digitalWrite(PIN_LED_RB, LOW);
      }
      else {
        digitalWrite(PIN_LED_LB, LOW);
        digitalWrite(PIN_LED_RB, HIGH);
      }
    }
    // enforce limits
    ctrl_left = ctrl_left > 0 ? max(64, min(ctrl_left, 192)) : min(-64, max(ctrl_left, -192));
    ctrl_right = ctrl_right > 0 ? max(64, min(ctrl_right, 192)) : min(-64, max(ctrl_right, -192));

  #else // Wait for messages from the phone 此处才是正式接收手机信息
    if (Serial.available() > 0) {
      //read_msg();
      check_for_msg();
    }
    if (distance_estimate < STOP_THRESHOLD) {
      if (ctrl_left > 0) ctrl_left = 0;
      if (ctrl_right > 0) ctrl_right = 0;
    }
  #endif
  
  update_left_motors();
  update_right_motors();
  
  //Measure speed, otherwise through interrupt
   update_speed_left();
   update_speed_right();

}


//------------------------------------------------------//
// FUNCTIONS
//------------------------------------------------------//

void update_left_motors() {    //如果左右马达旋转方向有误就会原地打转，可以把其中一对马达M+和M-调换一下即可
    if (ctrl_left < 0) {
      analogWrite(PIN_PWM_L,-ctrl_left);
      analogWrite(PIN_DIR_L,0);
    }
    else if (ctrl_left > 0) {
      analogWrite(PIN_PWM_L,0);
      analogWrite(PIN_DIR_L,ctrl_left);
      
    }
    else { //Motor brake
      analogWrite(PIN_PWM_L,255);
      analogWrite(PIN_DIR_L,255);
    }
}

void update_right_motors() {
    if (ctrl_right < 0) {
      analogWrite(PIN_PWM_R,-ctrl_right);
      analogWrite(PIN_DIR_R,0);
    }
    else if (ctrl_right > 0) {
      analogWrite(PIN_PWM_R,0);
      analogWrite(PIN_DIR_R,ctrl_right);
      
    }
    else { //Motor brake
      analogWrite(PIN_PWM_R,255);
      analogWrite(PIN_DIR_R,255);
    }
}


void update_speed_left() {
  long LnewPosition = LeftEnc.read();  
  if (LnewPosition != LoldPosition) {
    if (LnewPosition < LoldPosition) {
      counter_left--;
      }
    else if (LnewPosition > LoldPosition) {
      counter_left++;
       }
    }
    LoldPosition = LnewPosition;
  }

void update_speed_right() {
  long RnewPosition = RightEnc.read();
  if (RnewPosition != RoldPosition) {
    if (RnewPosition < RoldPosition) {
      counter_right--;
      }
    else if (RnewPosition > RoldPosition) {
      counter_right++;
       }
    }
    RoldPosition = RnewPosition;
  }

void process_ctrl_msg() {
  bool ctrl_rx = true;
  while (ctrl_rx) {
    if (Serial.available()) {
      char inChar = Serial.read();
      // comma indicates that inString contains the left ctrl
      if (inChar == ',') {
        ctrl_left = inString.toInt();
        // clear the string for new input:
        inString = "";
      }
      // new line indicates that inString contains the right ctrl
      else if (inChar == '\n') {
        ctrl_right = inString.toInt();
        // clear the string for new input:
        inString = "";
        // end of message
        ctrl_rx = false;
      }
      else {
        // As long as the incoming byte
        // is not a newline or comma,
        // convert the incoming byte to a char
        // and add it to the string
        inString += inChar;
      }
    }
  }
}

void process_indicator_msg() {
  bool indicator_rx = true;
  while (indicator_rx) {
    if (Serial.available()){
      char inChar = Serial.read();
      // new line indicates that inString contains the indicator signal
      if (inChar == '\n') {
        indicator_val = inString.toInt();
        // clear the string for new input:
        inString = "";
        // end of message
        indicator_rx = false;
      }
      else {
        // As long as the incoming byte
        // is not a newline
        // convert the incoming byte to a char
        // and add it to the string
        inString += inChar;
      }
    }
  }
}

void check_for_msg() {
  char inChar = Serial.read();
  switch (inChar) {
    case 'c':
      process_ctrl_msg();
      break;
    case 'i':
      process_indicator_msg();
      break;
  }
}

void send_vehicle_data() {
  float voltage_value = get_voltage();
  int ticks_left = counter_left;
  counter_left = 0;
  int ticks_right = counter_right;
  counter_right = 0;
  
  #if (NO_PHONE_MODE || HAS_OLED)
    float rpm_factor = 60.0*(1000.0/SEND_INTERVAL)/(DISK_HOLES);
    float rpm_left = ticks_left*rpm_factor;
    float rpm_right = ticks_right*rpm_factor;
  #endif
  #if (NO_PHONE_MODE)
    Serial.print("Voltage: "); Serial.println(voltage_value, 2);
    Serial.print("Left RPM: "); Serial.println(rpm_left, 0);
    Serial.print("Right RPM: "); Serial.println(rpm_right, 0);
    Serial.print("Distance: "); Serial.println(distance_estimate);
    Serial.println("------------------");
  #else
    Serial.print(voltage_value);
    Serial.print(",");
    Serial.print(counter_left*rrpm_factor);
    Serial.print(",");
    Serial.print(counter_right*rrpm_factor);
    Serial.print(",");
    Serial.print(distance_estimate);
    Serial.println();
    
  #endif 
  
  #if HAS_OLED
    // Set display information
    drawString(
      "Voltage:    " + String(voltage_value,2), 
      "Left RPM:  " + String(rpm_left,0), 
      "Right RPM: " + String(rpm_right, 0), 
      "Distance:   " + String(distance_estimate));
  #endif
}

 

#if HAS_VOLTAGE_DIVIDER
  float get_voltage () {
    unsigned long array_sum = 0;
    unsigned int array_size = min(VIN_ARR_SZ,vin_counter);
    for(unsigned int index = 0; index < array_size; index++)
    { 
      array_sum += vin_array[index]; 
    }
    return float(array_sum)/array_size/ADC_MAX*VREF*VOLTAGE_DIVIDER_FACTOR;
  }
#else
  float get_voltage () {
    return -1;
  }
#endif

#if HAS_INDICATORS
  void update_indicators() {
    if (indicator_val < 0) {
      digitalWrite(PIN_LED_LB, !digitalRead(PIN_LED_LB));
      digitalWrite(PIN_LED_RB, 0);
    }
    else if (indicator_val > 0) {
      digitalWrite(PIN_LED_LB, 0);
      digitalWrite(PIN_LED_RB, !digitalRead(PIN_LED_RB));
    }
    else {
      digitalWrite(PIN_LED_LB, 0);
      digitalWrite(PIN_LED_RB, 0);
    }
  }
#endif

#if HAS_OLED
// Function for drawing a string on the OLED display
void drawString(String line1, String line2, String line3, String line4) {
  display.clearDisplay();
  // set text color
  display.setTextColor(WHITE);
  // set text size
  display.setTextSize(1);
  // set text cursor position
  display.setCursor(1,0);
  // show text
  display.println(line1);
  display.setCursor(1,8);
  // show text
  display.println(line2);
  display.setCursor(1,16);
  // show text
  display.println(line3);
  display.setCursor(1,24);
  // show text
  display.println(line4);    
  display.display();
}
#endif

#if USE_MEDIAN
  unsigned int get_median(unsigned int a[], unsigned int sz) {
    //bubble sort
    for(unsigned int i=0; i<(sz-1); i++) {
      for(unsigned int j=0; j<(sz-(i+1)); j++) {
        if(a[j] > a[j+1]) {
            unsigned int t = a[j];
            a[j] = a[j+1];
            a[j+1] = t;
        }
      }
    }
    return a[sz/2];
  }
#endif

//------------------------------------------------------//
// INTERRUPT SERVICE ROUTINES (ISR)
//------------------------------------------------------//

#if HAS_SPEED_SENSORS
  // ISR: Increment speed sensor counter (right)
  void update_speed_left() {
    if (ctrl_left < 0) {
      counter_left--; 
    }
    else if (ctrl_left > 0) {
      counter_left++;
    }
  }
  
  // ISR: Increment speed sensor counter (right)
  void update_speed_right() {
    if (ctrl_right < 0) {
      counter_right--; 
    }
    else if (ctrl_right > 0){
      counter_right++;
    }
  }
#endif

#if HAS_SONAR
  // Send pulse by toggling trigger pin
  void send_ping() {
    detachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_ECHO));
    echo_time = 0;
    pinMode(PIN_TRIGGER,OUTPUT);
    digitalWrite(PIN_TRIGGER, LOW);
    delayMicroseconds(5);
    digitalWrite(PIN_TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIGGER, LOW);
    pinMode(PIN_ECHO,INPUT);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_ECHO), start_timer, RISING);
  }
  // ISR: Start timer to measure the time it takes for the pulse to return
  void start_timer() {
    start_time = micros();
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_ECHO), stop_timer, FALLING);
  }
  // ISR: Stop timer and record the time
  void stop_timer() {
    echo_time = micros() - start_time;
    }
#endif
