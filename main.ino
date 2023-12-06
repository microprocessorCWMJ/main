#define S2 31
#define S3 33      
#define sensorOut1 35 
#define S22 37
#define S32 39
#define sensorOut2 41
#define S23 43
#define S33 45      
#define sensorOut3 47 
#define S24 49
#define S34 51
#define sensorOut4 53

#define switch 23
#define parking_switch 25

#define LeftFrontpinTrig 22
#define LeftFrontpinEcho 24
#define LeftBehindpinTrig 26
#define LeftBehindpinEcho 28
#define RightFrontpinTrig 30
#define RightFrontpinEcho 32
#define RightBehindpinTrig 34
#define RightBehindpinEcho 36

#include <Wire.h>

const int MPU_ADDR = 0x68;  // Address of MPU6050(imu sensor) for I2C communication
const double RADIAN_TO_DEGREE = 180 / 3.14159;

// length of kickboard
const double kickboard_length_cm = 38;

//----------------------------------------------------For Driving Mode--------------------------------------------------//
// If users change the button to park, the driving mode must be false.
bool driving_mode = true;
// If the white color is repetitively detected, on_the_crosswalk flag is set.
// It names "on_the_crosswalk", but it is also set when the user is changing lanes repetitively.
bool on_the_crosswalk = true;
// If ultrasound detects objects repetitively, many_objects_around flag is set.
bool many_objects_around = false;
// If parking is completed, the parking completion flag is set.
bool parking_complete = false;
//----------------------------------------------------------------------------------------------------------------------//

//---------------------------------------------------For Parking Mode---------------------------------------------------//
bool untilted_parking = false;
bool parallel_with_beside_kickboard = false;
bool on_the_parking_line = false;
//----------------------------------------------------------------------------------------------------------------------//

class parking{
  private:
    int16_t AcX, AcY, AcZ;
    double angleAcY;
    
    int R_Min = 5; 
    int R_Max = 25; 
    int G_Min = 4; 
    int G_Max = 42;
    int B_Min = 4; 
    int B_Max = 45; 

    int Red1 = 0, Red2 = 0, Red3 = 0, Red4 = 0;
    int Green1 = 0, Green2 = 0, Green3 = 0, Green4 = 4;
    int Blue1 = 0, Blue2 = 0, Blue3 = 0, Blue4 = 0;

    int redValue1, redValue2, redValue3, redValue4;
    int greenValue1, greenValue2, greenValue3, greenValue4;
    int blueValue1, blueValue2, blueValue3, blueValue4;
    int Frequency;

    bool park_flag_Red, park_flag_Green, park_flag_Blue; //Total three colors
  
  public:
    int getRed(int x) {
      if(x==1){
        digitalWrite(S2,LOW);
        digitalWrite(S3,LOW);
        Frequency = pulseIn(sensorOut1, LOW);
      }
      else if(x==2){
        digitalWrite(S22,LOW);
        digitalWrite(S32,LOW);
        Frequency = pulseIn(sensorOut2, LOW);
      }
      else if(x==3){
        digitalWrite(S23,LOW);
        digitalWrite(S33,LOW);
        Frequency = pulseIn(sensorOut3, LOW);
      }
      else if(x==4){
        digitalWrite(S24,LOW);
        digitalWrite(S34,LOW);
        Frequency = pulseIn(sensorOut4, LOW);
      }

      return Frequency;
    }

    int getGreen(int x) {
      if(x==1){
        digitalWrite(S2,HIGH);
        digitalWrite(S3,HIGH);
        Frequency = pulseIn(sensorOut1, LOW);
      }
      else if(x==2){
        digitalWrite(S22,HIGH);
        digitalWrite(S32,HIGH);
        Frequency = pulseIn(sensorOut2, LOW);
      }
      else if(x==3){
        digitalWrite(S23,HIGH);
        digitalWrite(S33,HIGH);
        Frequency = pulseIn(sensorOut3, LOW);
      }
      else if(x==4){
        digitalWrite(S24,HIGH);
        digitalWrite(S34,HIGH);
        Frequency = pulseIn(sensorOut4, LOW);
      }
      return Frequency;
    }

    int getBlue(int x) {
      if(x==1){
      digitalWrite(S2,LOW);
      digitalWrite(S3,HIGH);
      Frequency = pulseIn(sensorOut1, LOW);
      }
      else if(x==2){
      digitalWrite(S22,LOW);
      digitalWrite(S32,HIGH);
      Frequency = pulseIn(sensorOut2, LOW);
      }
      else if(x==3){
      digitalWrite(S23,LOW);
      digitalWrite(S33,HIGH);
      Frequency = pulseIn(sensorOut3, LOW);
      }
      else if(x==4){
        digitalWrite(S24,LOW);
        digitalWrite(S34,HIGH);
        Frequency = pulseIn(sensorOut4, LOW);
      }
      return Frequency;
    }

    void park() {
      // When detect red line
      // I set the wide range of values. This must be changed.
      if(169<redValue1 && redValue1<209 && 135<greenValue1 && greenValue1<175 && 155<blueValue1 && blueValue1<195){
        if(169<redValue2 && redValue2<209 && 135<greenValue2 && greenValue2<175 && 155<blueValue2 && blueValue2<195){
          if(169<redValue3 && redValue3<209 && 135<greenValue3 && greenValue3<175 && 155<blueValue3 && blueValue3<195){
            if(169<redValue4 && redValue4<209 && 135<greenValue4 && greenValue4<175 && 155<blueValue4 && blueValue4<195){
              park_flag_Red = true;
            }
          }
        }
      }

      // When detect green line
      // I set the wide range of values. This must be changed.
      else if(169<redValue1 && redValue1<209 && 135<greenValue1 && greenValue1<175 && 155<blueValue1 && blueValue1<195){ 
        if(169<redValue2 && redValue2<209 && 135<greenValue2 && greenValue2<175 && 155<blueValue2 && blueValue2<195){
          if(169<redValue3 && redValue3<209 && 135<greenValue3 && greenValue3<175 && 155<blueValue3 && blueValue3<195){
            if(169<redValue4 && redValue4<209 && 135<greenValue4 && greenValue4<175 && 155<blueValue4 && blueValue4<195){
              park_flag_Green = true;
            }
          }
        }
      }

      // When detect blue line
      // I set the wide range of values. This must be changed.
      else if(169<redValue1 && redValue1<209 && 135<greenValue1 && greenValue1<175 && 155<blueValue1 && blueValue1<195){ 
        if(169<redValue2 && redValue2<209 && 135<greenValue2 && greenValue2<175 && 155<blueValue2 && blueValue2<195){
          if(169<redValue3 && redValue3<209 && 135<greenValue3 && greenValue3<175 && 155<blueValue3 && blueValue3<195){
            if(169<redValue4 && redValue4<209 && 135<greenValue4 && greenValue4<175 && 155<blueValue4 && blueValue4<195){
              park_flag_Blue = true;
            }
          }
        }
      }

      // When detected color is not R,G and B.
      else{
        park_flag_Red = false;
        park_flag_Green = false;
        park_flag_Blue = false;
      }
    }

    void detect_parking_line(){
      Red1 = getRed(1);
      Red2 = getRed(2);
      Red3 = getRed(3);
      Red4 = getRed(4);
      // all of the values need to be calibrated. This is temporay value.
      // We should add calibrating code or We should set the min & max value by hand. 
      redValue1 = map(Red1, R_Min,R_Max,255,0); 
      redValue2 = map(Red2, R_Min,R_Max,255,0);
      redValue3 = map(Red3, R_Min,R_Max,255,0);
      redValue4 = map(Red4, R_Min,R_Max,255,0);

      Green1 = getGreen(1);
      Green2 = getGreen(2);
      Green3 = getGreen(3);
      Green4 = getGreen(4);
      greenValue1 = map(Green1, G_Min,G_Max,255,0);
      greenValue2 = map(Green2, G_Min,G_Max,255,0);
      greenValue3 = map(Green3, G_Min,G_Max,255,0);
      greenValue4 = map(Green4, G_Min,G_Max,255,0);

      Blue1 = getBlue(1);
      Blue2 = getBlue(2);
      Blue3 = getBlue(3);
      Blue4 = getBlue(4);
      blueValue1 = map(Blue1, B_Min,B_Max,255,0);
      blueValue2 = map(Blue2, B_Min,B_Max,255,0); 
      blueValue3 = map(Blue3, B_Min,B_Max,255,0);
      blueValue4 = map(Blue4, B_Min,B_Max,255,0);
    
      park();
      
      if(!park_flag_Red && !park_flag_Green && !park_flag_Blue){
        on_the_parking_line = false;
      }
      else {
        on_the_parking_line = true;
      }
    }

    double measureDistanceCm(uint8_t pinTrig, uint8_t pinEcho){
      digitalWrite(pinTrig, LOW);
      delayMicroseconds(5);
      digitalWrite(pinTrig, HIGH);
      delayMicroseconds(10);
      digitalWrite(pinTrig, LOW);
      
      double duration = pulseIn(pinEcho, HIGH);
      double cm = 0.0343 * (duration/2);
      return cm;
    }

    void measureAngle_for_parking(){
      if(on_the_parking_line){

        double distance_LeftFront = measureDistanceCm(LeftFrontpinTrig, LeftFrontpinEcho);
        double distance_LeftBehind = measureDistanceCm(LeftBehindpinTrig, LeftBehindpinEcho);
        double distance_RightFront = measureDistanceCm(RightFrontpinTrig, RightFrontpinEcho);
        double distance_RightBehind = measureDistanceCm(RightBehindpinTrig, RightBehindpinEcho);

        //They will be used when there are kickboards on your both side.
        bool Left_parallel, Right_parallel;

        // To prevent distance errors.
        if(distance_LeftFront < 1000 && distance_LeftBehind < 1000 && distance_RightFront < 1000 && distance_RightBehind < 1000){

          // Assume that no kickboards beside you
          if (distance_LeftFront >= 300 && distance_LeftBehind >= 300 && distance_RightFront >= 300 && distance_RightBehind >= 300){
            parallel_with_beside_kickboard = true;
          }

          // Assume that no kickboard on your right side
          else if(distance_RightFront >= 300 && distance_RightBehind >= 300){
            if (distance_LeftFront >= distance_LeftBehind){
              double distance_Leftdiff = distance_LeftFront - distance_LeftBehind;
              double cos_angle_Left = acos(distance_Leftdiff / kickboard_length_cm);
              cos_angle_Left = cos_angle_Left * RADIAN_TO_DEGREE;

              // Crieria of wrong parking angle: 75 degree (It needs to be updated)
              if (cos_angle_Left <= 75) {
                parallel_with_beside_kickboard = false;
              }
              else {
                parallel_with_beside_kickboard = true;
              }
            }

            else{
              double distance_Leftdiff = distance_LeftBehind - distance_LeftFront;
              double cos_angle_Left = acos(distance_Leftdiff / kickboard_length_cm);
              cos_angle_Left = cos_angle_Left * RADIAN_TO_DEGREE;
              if (cos_angle_Left <= 75){
                parallel_with_beside_kickboard = false;
              }
              else {
                parallel_with_beside_kickboard = true;
              }
            }
          }

          // Assume that no kickboard on your left side
          else if(distance_LeftFront >= 300 && distance_LeftBehind >= 300){
            if (distance_RightFront >= distance_RightBehind){
              double distance_Rightdiff = distance_RightFront - distance_RightBehind;
              double cos_angle_Right = acos(distance_Rightdiff / kickboard_length_cm);
              cos_angle_Right = cos_angle_Right * RADIAN_TO_DEGREE;
              if (cos_angle_Right <= 75) {
                parallel_with_beside_kickboard = false;
              }
              else {
                parallel_with_beside_kickboard = true;
              }
            }

            else{
              double distance_Rightdiff = distance_RightBehind - distance_RightFront;
              double cos_angle_Right = acos(distance_Rightdiff / kickboard_length_cm);
              cos_angle_Right = cos_angle_Right * RADIAN_TO_DEGREE;
              if (cos_angle_Right <= 75){
                parallel_with_beside_kickboard = false;
              }
              else {
                parallel_with_beside_kickboard = true;
              }
            }
          }

          // Assume that there are kickboards on your both side.
          else{
            if (distance_LeftFront >= distance_LeftBehind){
              double distance_Leftdiff = distance_LeftFront - distance_LeftBehind;
              double cos_angle_Left = acos(distance_Leftdiff / kickboard_length_cm);
              cos_angle_Left = cos_angle_Left * RADIAN_TO_DEGREE;
              if (cos_angle_Left <= 75) {
                Left_parallel = false;
              }
              else {
                Left_parallel = true;
              }
            }

            else{
              double distance_Leftdiff = distance_LeftBehind - distance_LeftFront;
              double cos_angle_Left = acos(distance_Leftdiff / kickboard_length_cm);

              cos_angle_Left = cos_angle_Left * RADIAN_TO_DEGREE;
              if (cos_angle_Left <= 75){
                Left_parallel = false;
              }
              else {
                Left_parallel = true;
              }
            }

            if (distance_RightFront >= distance_RightBehind){
              double distance_Rightdiff = distance_RightFront - distance_RightBehind;
              double cos_angle_Right = acos(distance_Rightdiff / kickboard_length_cm);
              cos_angle_Right = cos_angle_Right * RADIAN_TO_DEGREE;
              if (cos_angle_Right <= 75) {
                Right_parallel = false;
              }
              else {
                Right_parallel = true;
              }
            }

            else{
              double distance_Rightdiff = distance_RightBehind - distance_RightFront;
              double cos_angle_Right = acos(distance_Rightdiff / kickboard_length_cm);
              cos_angle_Right = cos_angle_Right * RADIAN_TO_DEGREE;
              if (cos_angle_Right <= 75){
                Right_parallel = false;
              }
              else {
                Right_parallel = true;
              }
            }

            if(Right_parallel && Left_parallel){
              parallel_with_beside_kickboard = true;
            }
            else {
              parallel_with_beside_kickboard = false;
            }
          }
        }
      }
    }

    // Initiate communication between imu sensor and arduino
    void initSensor() {
      Wire.begin();
      Wire.beginTransmission(MPU_ADDR); 
      Wire.write(0x6B);                  // Address of MPU6050 to write
      Wire.write(0);
      Wire.endTransmission(true);
    }

    void getData() {
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x3B);  // AcX Register Address
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_ADDR, 14, true);  // require 14byte data after AcX address
      AcX = Wire.read() << 8 | Wire.read();
      AcY = Wire.read() << 8 | Wire.read();
      AcZ = Wire.read() << 8 | Wire.read();
    }

    void getAngleY() {
      getData();
      // Pitch angle
      angleAcY = atan(-AcX / sqrt(pow(AcY, 2) + pow(AcZ, 2)));
      angleAcY *= RADIAN_TO_DEGREE;
    }

    void detect_tilted_parking(){
      if(on_the_parking_line && parallel_with_beside_kickboard){
        initSensor();
        getAngleY();
        if ((AcX != -1) && (AcY != -1) && (AcZ != -1)) {
          if ((angleAcY >= 40) || (angleAcY <= -40)) {
            untilted_parking = false;
          }
          else{
            untilted_parking = true;
          }
        }
      }
    }
};

class driving{
  private:
    int16_t AcX, AcY, AcZ;   // Acceleration & Gyro 
    double angleAcY;

    // variables that need to count the number of reckless driving on crosswalk or lane.
    uint32_t tilted_reference_time = 0.0;
    uint32_t tilted_previous_time = 0.0;
    uint32_t tilted_current_time = 0.0;
    bool tilt_start_flag = false;
    uint8_t tilted_count = 0;
    uint8_t tilted_warning_count = 0;
    //recklessness: Dangerous driving even if there are detected so many objects(person, vehicle, dog, whatever ... )
    uint8_t tilted_recklessness_count = 0;

    uint32_t current_time_for_object_detection = 0.0;
    uint32_t previous_time_for_object_detection = 0.0;
    uint32_t current_time_for_object_detection_end = 0.0;
    uint32_t previous_time_for_object_detection_end = 0.0;

    int R_Min = 5; 
    int R_Max = 25; 
    int G_Min = 4; 
    int G_Max = 42;
    int B_Min = 4; 
    int B_Max = 45; 

    /*Define int variables*/
    int Red1, Red2, Red3, Red4;
    int Green1, Green2, Green3, Green4;
    int Blue1, Blue2, Blue3, Blue4;
    int color_previous_time = 0, color_current_time = 0;

    int redValue1, redValue2, redValue3, redValue4;
    int greenValue1, greenValue2, greenValue3, greenValue4;
    int blueValue1, blueValue2, blueValue3, blueValue4;
    int Frequency;

    uint32_t white_time = 0.0, black_time = 0.0, white_start_time = 0.0, black_start_time = 0.0, crosswalk_time;
    uint8_t crosswalk_count = 0, crosswalk_warning_count=0, tmp_crosswalk_count;
    uint16_t totval1, totval2;

    bool white_flag = false;
  
  public:
    double getAngleXY() {
      getData();  
      // Pitch angle
      angleAcY = atan(-AcX / sqrt(pow(AcY, 2) + pow(AcZ, 2)));
      angleAcY *= RADIAN_TO_DEGREE;
    }

    // Initiate I2C communication between imu sensor and arduino.
    void initSensor() {
      Wire.begin();
      Wire.beginTransmission(MPU_ADDR);   // Address for I2C communication
      Wire.write(0x6B);    // Write on 0x6B for communication with MPU6050
      Wire.write(0);
      Wire.endTransmission(true);
    }

    void getData() {
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x3B);   // AcX register address
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_ADDR, 14, true); // Request 14 bytes of data after AcX address
      AcX = Wire.read() << 8 | Wire.read(); // Connect two separate bytes into one and store them in each variable
      AcY = Wire.read() << 8 | Wire.read();
      AcZ = Wire.read() << 8 | Wire.read();
    }

    void detect_dangerous_driving(){
      initSensor();
      getAngleXY();
      if((angleAcY >= 30 || angleAcY <= -30) && !tilt_start_flag){
        tilted_current_time = millis();
        tilt_start_flag = true;
      }

      if((millis()-tilted_current_time >= 3000) && (angleAcY >= 30 || angleAcY <= -30)){
        tilted_count += 1;
        Serial.println("tilt counted");
        if(tilted_count == 1){
          tilted_reference_time = millis();
        }
        tilted_current_time = millis();
        tilt_start_flag = false;
      }

      if((tilted_current_time - tilted_reference_time <= 60000) && (tilted_count >= 5)){
        if(on_the_crosswalk || many_objects_around){
          tilted_warning_count += 1;
          tilted_recklessness_count += 1;

          Serial.println("Dangerous and reckless driving detected. If caught more than three times, account ban will be imposed:");
          Serial.print("Dangerous count: ");
          Serial.print(tilted_warning_count);
          Serial.print(", Recklessness count: ");
          Serial.println(tilted_recklessness_count);
          tilted_count = 0;
        }
        else{
          tilted_warning_count += 1;
          Serial.print("Dangerous driving detected. If caught more than three times, account ban will be imposed:");
          Serial.print("Dangerous count: ");
          Serial.print(tilted_warning_count);
          Serial.print(", Recklessness count: ");
          Serial.println(tilted_recklessness_count);
          tilted_count = 0;
        }
      }

      if(tilted_warning_count >= 3){
        if(tilted_recklessness_count >= 1){
          Serial.println("Reckless driving detected: 3-day account banned");
          tilted_warning_count = 0;
          tilted_recklessness_count = 0;
        }
        else{
          Serial.println("Dangerous driving: 1-day account banned");
          tilted_warning_count = 0;
          tilted_recklessness_count = 0;
        }
      }
    }

    double measureDistanceCm(uint8_t pinTrig, uint8_t pinEcho){
      digitalWrite(pinTrig, LOW);
      delayMicroseconds(5);
      digitalWrite(pinTrig, HIGH);
      delayMicroseconds(10);
      digitalWrite(pinTrig, LOW);
      double duration = pulseIn(pinEcho, HIGH);
      double cm = 0.0343 * (duration/2);
      return cm;
    }

    // Determine whether many_objects_around_LeftFront flag sets, or clears.
    void detect_objects_around(){
      double distance_LeftFront = measureDistanceCm(LeftFrontpinTrig, LeftFrontpinEcho);
      double distance_LeftBehind = measureDistanceCm(LeftBehindpinTrig, LeftBehindpinEcho);
      double distance_RightFront = measureDistanceCm(RightFrontpinTrig, RightFrontpinEcho);
      double distance_RightBehind = measureDistanceCm(RightBehindpinTrig, RightBehindpinEcho);

      //Ensuring that there is no distance error or The area that you are in is not open land.
      if(((distance_LeftFront < 1000) && (distance_LeftFront > 0)) && ((distance_LeftBehind < 1000 && distance_LeftBehind > 0)) && ((distance_RightFront < 1000 && distance_RightFront > 0)) && ((distance_RightBehind < 1000 && distance_RightBehind > 0))){
        
        // There is an object or many objects around you!
        if((distance_LeftFront <= 30) || (distance_LeftBehind <= 30) || (distance_RightFront <= 30) || (distance_RightBehind <= 30)){
          current_time_for_object_detection = millis();
          if(current_time_for_object_detection - previous_time_for_object_detection < 5000){
            many_objects_around = true;
          }
          previous_time_for_object_detection = current_time_for_object_detection;
        }
        
        // There are no objects around you
        else if((distance_LeftFront > 30) || (distance_LeftBehind > 30) || (distance_RightFront > 30) || (distance_RightBehind > 30)){
          current_time_for_object_detection_end = millis();
          if(current_time_for_object_detection_end - previous_time_for_object_detection_end >= 10000){
            many_objects_around = false;
          }
          previous_time_for_object_detection_end = current_time_for_object_detection_end;
        }
      }
    }

    int getRed(int x) {
      if(x==1){
        digitalWrite(S2,LOW);
        digitalWrite(S3,LOW);
        Frequency = pulseIn(sensorOut1, LOW);
      }
      else if(x==2){
        digitalWrite(S22,LOW);
        digitalWrite(S32,LOW);
        Frequency = pulseIn(sensorOut2, LOW);
      }
      else if(x==3){
        digitalWrite(S23,LOW);
        digitalWrite(S33,LOW);
        Frequency = pulseIn(sensorOut3, LOW);
      }
      else if(x==4){
        digitalWrite(S24,LOW);
        digitalWrite(S34,LOW);
        Frequency = pulseIn(sensorOut4, LOW);
      }
      return Frequency;
    }

    int getGreen(int x) {
      if(x==1){
        digitalWrite(S2,HIGH);
        digitalWrite(S3,HIGH);
        Frequency = pulseIn(sensorOut1, LOW);
      }
      else if(x==2){
        digitalWrite(S22,HIGH);
        digitalWrite(S32,HIGH);
        Frequency = pulseIn(sensorOut2, LOW);
      }
      else if(x==3){
        digitalWrite(S23,HIGH);
        digitalWrite(S33,HIGH);
        Frequency = pulseIn(sensorOut3, LOW);
      }
      else if(x==4){
        digitalWrite(S24,HIGH);
        digitalWrite(S34,HIGH);
        Frequency = pulseIn(sensorOut4, LOW);
      }
      return Frequency;
    }

    int getBlue(int x) {
      if(x==1){
      digitalWrite(S2,LOW);
      digitalWrite(S3,HIGH);
      Frequency = pulseIn(sensorOut1, LOW);
      }
      else if(x==2){
      digitalWrite(S22,LOW);
      digitalWrite(S32,HIGH);
      Frequency = pulseIn(sensorOut2, LOW);
      }
      else if(x==3){
      digitalWrite(S23,LOW);
      digitalWrite(S33,HIGH);
      Frequency = pulseIn(sensorOut3, LOW);
      }
      else if(x==4){
        digitalWrite(S24,LOW);
        digitalWrite(S34,HIGH);
        Frequency = pulseIn(sensorOut4, LOW);
      }
      return Frequency;
    }

    void detect_crosswalk(){
      Red1 = getRed(1);
      Red2 = getRed(2);
      redValue1 = map(Red1, R_Min,R_Max,255,0);
      redValue2 = map(Red2, R_Min,R_Max,255,0); // all of the values need to be calibrated. This is temporay value. 
      //We should add calibrating code or We should set the min & max value by hand.
      delay(10);
      Green1 = getGreen(1);
      Green2 = getGreen(2);
      greenValue1 = map(Green1, G_Min,G_Max,255,0);
      greenValue2 = map(Green2, G_Min,G_Max,255,0);
      delay(10);
      Blue1 = getBlue(1);
      Blue2 = getBlue(2);
      blueValue1 = map(Blue1, B_Min,B_Max,255,0); 
      blueValue2 = map(Blue2, B_Min,B_Max,255,0); 
      delay(10);  

      //two color sensors are used located in the kickboard's head side
      totval1 = redValue1 + greenValue1 + blueValue1;
      totval2 = redValue2 + greenValue2 + blueValue2;
      
      if(!white_flag && ((totval1 >= 660) || (totval2 >= 660))){
        white_flag = true;
        black_time = millis() - black_start_time;
        crosswalk_count += 1;
        white_start_time = millis();
      }

      else if(white_flag && ((totval1 <= 60) || (totval2 <= 60))){
        white_flag = false;
        white_time = millis() - white_start_time;
        crosswalk_count += 1;
        black_start_time = millis();
      }

      // Average walking speed for adult males: 4.8 km/h = 1.33 m/s      
      // Crosswalk white block specification: 45 to 50 cm -> taken as a reference to the minimum value of 45 cm for safety purposes
      // Specification of the interval between white blocks of crosswalks: 1.5 * (white block length) -> 1.5 times the minimum value of 45 cm for safety purposes
      // Time taken to cross the white block of a crosswalk at the average walking speed of an adult male: (distance) / (speed) = 0.45/1.33 = 0.338 seconds
      // Time taken to cross the white block spacing of the crosswalk at the average walking speed of an adult male: 0.508 seconds
      
      // If you step on the white block as soon as you start driving, the unfair count can be unintentionally counted.
      // 30 sec for preventing unfair count
      if(((white_time>0) && (white_time<338)) && (millis() > 30000)){
        crosswalk_warning_count += 1;
        Serial.print("Detection of rapid motion within crosswalks or excessive lane changes. If caught more than three times, account ban will be imposed:");
        Serial.println(crosswalk_warning_count);
      }

      if(((black_time>0) && (black_time<508)) && (millis() > 30000)){
        crosswalk_warning_count += 1;
        Serial.print("Detection of rapid motion within crosswalks or excessive lane changes. If caught more than three times, account ban will be imposed:");
        Serial.println(crosswalk_warning_count);
      }
      
      if(crosswalk_count>=3){
        //on the crosswalk or lane change repetitively
        on_the_crosswalk = true;
        if(crosswalk_count == 3){   
          tmp_crosswalk_count = crosswalk_count;
          crosswalk_time = millis();
        }

        if(crosswalk_warning_count>=3){
          Serial.println("Reckless driving detected: 3-day account banned");
        }

        if(crosswalk_count - tmp_crosswalk_count > 0){
          tmp_crosswalk_count = crosswalk_count;
          crosswalk_time = millis();
        }

        else if((millis()-crosswalk_time > 3000) && (crosswalk_count == tmp_crosswalk_count)){
          on_the_crosswalk = false;
          crosswalk_count = 0;
          tmp_crosswalk_count = 0;
        }
      }
    }
};

void switch_mode(){
  pinMode(switch,INPUT_PULLUP);

  uint8_t button = digitalRead(switch);
  if(driving_mode==false && button == 0){
    Serial.println("Now Driving");
    driving_mode = true;
  }
  
  else if(driving_mode==true && button == 0){
    Serial.println("Now Parking");
    driving_mode = false;
  }
}

// Class declaration before start
driving driving;
parking parking;

void setup() {
  // put your setup code here, to run once:
  pinMode(parking_switch,INPUT_PULLUP);

  pinMode(LeftFrontpinTrig, OUTPUT);
  pinMode(LeftFrontpinEcho, INPUT);
  pinMode(LeftBehindpinTrig, OUTPUT);
  pinMode(LeftBehindpinEcho, INPUT);
  pinMode(RightFrontpinTrig, OUTPUT);
  pinMode(RightFrontpinEcho, INPUT);
  pinMode(RightBehindpinTrig, OUTPUT);
  pinMode(RightBehindpinEcho, INPUT);
  
  pinMode(S2, OUTPUT);    
  pinMode(S3, OUTPUT);      
  pinMode(sensorOut1, INPUT); 
  pinMode(S22, OUTPUT);    
  pinMode(S32, OUTPUT);      
  pinMode(sensorOut2, INPUT);
  pinMode(S23, OUTPUT);    
  pinMode(S33, OUTPUT);      
  pinMode(sensorOut3, INPUT); 
  pinMode(S24, OUTPUT);    
  pinMode(S34, OUTPUT);      
  pinMode(sensorOut4, INPUT);  

  Serial.begin(115200);      
  delay(1000);

  if(driving_mode){Serial.println("Initialized to driving mode");}
  else{Serial.println("Initialized to parking mode");}
}

void loop() {
  // put your main code here, to run repeatedly:
  switch_mode(); //Push the button to change the mode; 
  
  uint8_t parking_button = digitalRead(parking_switch);

  if(driving_mode){
    driving.detect_crosswalk();
    driving.detect_objects_around();
    driving.detect_dangerous_driving();
  }

  if(!driving_mode){
    parking.detect_parking_line();
    parking.measureAngle_for_parking();
    parking.detect_tilted_parking();

    if(untilted_parking && parallel_with_beside_kickboard && on_the_parking_line){
      if(parking_button == 0 ){
        Serial.println("parking complete");
        parking_complete = true;
        delay(30000);
      }
    }

    else{
      if(parking_button == 0){
        if(!on_the_parking_line){
          Serial.println("Please check the parking line.");
        }
        else if(!parallel_with_beside_kickboard){
          Serial.println("Please park parallel to the side kickboard.");
        }
        else if(!untilted_parking){
          Serial.println("Please park the kickboard parallel to the ground.");
        }
        parking_complete = false;
      }
    }
  }
}
