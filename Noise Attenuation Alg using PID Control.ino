#include <Wire.h>
#include <Servo.h>
#include <iq_module_communication.hpp>
#include <math.h>

//--- VARIABLES --------------------------------------------------//
float elapsedTime = 0, time = 0, timePrev = 0;
float rad_to_deg = 180/PI;
float PID = 0, PID1 = 0, PID2 = 0, PID3 = 0;
float updateangle = 0, desired_angle = 0;
float angle = 0;
float est_angle0 = 0, est_angle1 = 0, est_angle2 = 0, est_angle3 = 0;
float error0 = 0, error1 = 0, error2 = 0, error3 = 0;
float pid_p_0 = 0, pid_i_0 = 0, pid_d_0 = 0;
float pid_p_1 = 0, pid_i_1 = 0, pid_d_1 = 0;
float pid_p_2 = 0, pid_i_2 = 0, pid_d_2 = 0;
float pid_p_3 = 0, pid_i_3 = 0, pid_d_3 = 0;
float est_diff = 0, diff = 0, diff2 = 0;
float time0 = 0, time1 = 0, time2 = 0, time3 = 0;
float prevPID_0 = 0, prevPID_1 = 0, prevPID_2 = 0, prevPID_3 = 0;
float newvel = 0, obs_vel = 0;


//--- MOTOR SPEED SET --------------------------------------------//
float motorRads = 500; //currently set to 500rads/sec


//--- PID CONSTANT SET -------------------------------------------//
double kp=4;   //kp = 1 for 100rad/s, kp = 4 for 650rad/s 
double ki=8;   //ki = 2 for 100rad/s, ki = 8 for 650rad/s
double kd=0;   //Derivative was not used


//--- INITIALIZE SERIAL FOR EACH MOTOR FOR COMMUNICATION ---------//
IqSerial ser(Serial);
IqSerial ser1(Serial1);
IqSerial ser2(Serial2);
IqSerial ser3(Serial3);


//--- INITIALIZE IQ MODULES --------------------------------------//
PropellerMotorControlClient prop(0);
BrushlessDriveClient mot(0);


void setup() {
  ser.begin();
  ser1.begin();
  ser2.begin();
  ser3.begin();

  ser.set(prop.timeout_,1000.0f);
  ser1.set(prop.timeout_,1000.0f);
  ser2.set(prop.timeout_,1000.0f);
  ser3.set(prop.timeout_,1000.0f);

  time = millis(); // Start counting time in milliseconds
  
  // Set the inital velocity below
  ser.set(prop.ctrl_velocity_,motorRads);
  ser1.set(prop.ctrl_velocity_,motorRads);
  ser2.set(prop.ctrl_velocity_,motorRads);
  ser3.set(prop.ctrl_velocity_,motorRads);
}

void loop() {
    // PID call for motor 0
    PID_Function(ser, motorRads, &pid_p_0, &pid_i_0, &pid_d_0, &prevPID_0, &error0, &time0, &est_angle0, 0);
    timePrev = time;  
    time = millis();  
    elapsedTime = (time - timePrev) / 1000; 
    time0 = elapsedTime;
    time1 += elapsedTime;
    time2 += elapsedTime;
    time3 += elapsedTime;
    
    // PID call for motor 1
    PID_Function(ser1, motorRads, &pid_p_1, &pid_i_1, &pid_d_1, &prevPID_1, &error1, &time1, &est_angle1, 0);
    timePrev = time;  
    time = millis();  
    elapsedTime = (time - timePrev) / 1000; 
    time0 += elapsedTime;
    time1 = elapsedTime;
    time2 += elapsedTime;
    time3 += elapsedTime;
    
    // PID call for motor 2
    PID_Function(ser2, motorRads, &pid_p_2, &pid_i_2, &pid_d_2, &prevPID_2, &error2, &time2, &est_angle2, 0);
    timePrev = time;  
    time = millis();  
    elapsedTime = (time - timePrev) / 1000; 
    time0 += elapsedTime;
    time1 += elapsedTime;
    time2 = elapsedTime;
    time3 += elapsedTime;
    
    // PID call for motor 3
    PID_Function(ser3, motorRads, &pid_p_3, &pid_i_3, &pid_d_3, &prevPID_3, &error3, &time3, &est_angle3, 0);  
    timePrev = time;  
    time = millis();  
    elapsedTime = (time - timePrev) / 1000; 
    time0 += elapsedTime;
    time1 += elapsedTime;
    time2 += elapsedTime;
    time3 = elapsedTime;
 
}

void PID_Function(IqSerial &serialPort, float motorRads, float *pid_p, float *pid_i, float *pid_d, float *prevPID, float *error, float *motortime, float *est_angle, float offset){
    // get the angle of the virtual reference
    desired_angle = hmodRadf(motorRads*(time/1000));
    
    // get the angle of actual motor
    serialPort.get(mot.obs_angle_,angle);
    
    // find the difference between motor and "ref"
    // offset 0: motor is parallel to the virtual motor
    // offset PI/2: motor is 90 degrees off with respect to virtual motor
    diff = hmodRadf(desired_angle - angle + offset); 
    
    // new estimate angle
    // 1. propogate 
    //      angle += (measured rpm)*elapsedTime
    // 2. update  
    //      estimated diff = estimated angle - desired angle
    // update = ((diff-est_diff)->[-PI to PI])*((small_gain)->[0-1]))
    // est_diff += update
    // angle -= update 
    serialPort.get(mot.obs_velocity_, obs_vel);
    *est_angle += obs_vel*(*motortime);                  // estimate the angle of the motor using time since last PID call
    *est_angle = hmodRadf(*est_angle);                   // set est_angle between [-Pi, Pi]
  
    est_diff = desired_angle - (*est_angle) + offset;    // estimate the difference between the desired and estimated angle
    est_diff = hmodRadf(est_diff);                       // set est_diff between [-Pi, Pi]
  
    updateangle = hmodRadf(diff-est_diff)*0.1;
    est_diff += updateangle;
    *est_angle -= updateangle;
    *est_angle = hmodRadf(*est_angle);

    // Calculating P, I, D
    *pid_p = est_diff*kp;
    *pid_i += est_diff*(*motortime);
    //*pid_d = (motorRads - obs_vel)*kd; //not in use

    // Final PID 
    PID = *pid_p + ki*(*pid_i) + *pid_d;
    
    // Calculate the new velocity
    newvel = motorRads*1.04 + PID;  // gain value 1.04 puts real motor speed closest to the virtual motor
    
    // Command new velocity to motor
    serialPort.set(prop.ctrl_velocity_,newvel);
    
    // Save current PID value for future PID
    *prevPID = PID;
    
    // Save current error for future PID
    *error = est_diff;
}



// Get a value between -PI and PI (PI-wrapping)
float hmodRadf(float h){
  float dh;
  int i;

  if (h>0){
    i = (int)(h/(2*PI)+0.5);
  }
  else{
    i = (int)(h/(2*PI)-0.5);
  }
  dh = h - PI*2*i;

  return dh;
}
