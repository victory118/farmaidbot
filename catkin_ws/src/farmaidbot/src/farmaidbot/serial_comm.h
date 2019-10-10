#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include "diff_steer.h"
namespace Farmaid
{
class SerialComm {
  public:
    float v_des_, w_des_;
  
    SerialComm(): v_des_(0), w_des_(0){
        prev_ros_time_ = micros();
    }
    void receiveSerialData(){
        if (Serial.available() > 0) {
            String commandString = Serial.readStringUntil('\n');  // read a line
            float command[2];
            for (int i = 0, indexPointer = 0; indexPointer != -1 ; i++ ) {
                indexPointer = commandString.indexOf(',');
                String tempString = commandString.substring(0, indexPointer);
                command[i] = tempString.toFloat();
                commandString = commandString.substring(indexPointer+1);
            }
            v_des_ = command[0];
            w_des_ = command[1];
        }
    }
    
    void send(const Odometry2D& odom) {
        unsigned long current_time = micros();
        if (current_time - prev_ros_time_ >= ros_period_micros) {
            Serial.print(odom.pos_x, 6);   Serial.print(",");  //X 
            Serial.print(odom.pos_y, 6);   Serial.print(",");  //Y 
            Serial.print(odom.theta);      Serial.print(",");    //Th
            Serial.print(odom.wheel_angle_left);    Serial.print(",");     
            Serial.println(odom.wheel_angle_right);
            prev_ros_time_ = current_time;
        }
    }
  private: 
    unsigned long prev_ros_time_;
};
};

#endif
