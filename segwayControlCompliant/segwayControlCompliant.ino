#include <Encoder.h>
#include <Wire.h>
#include <NAxisMotion.h>
#include <DualVNH5019MotorShield.h>

#define FORWARDS false

#if FORWARDS
DualVNH5019MotorShield motorDriver(11, 5, 13, A0, 9, 7, 8, 12, A1, 10);
Encoder motorEncL(2, 3);
Encoder motorEncR(18, 19);
float dir = 1.0;
#else // backwards
DualVNH5019MotorShield motorDriver(8, 7, 12, A1, 10, 5, 11, 13, A0, 9);
Encoder motorEncL(19, 18);
Encoder motorEncR(3, 2);
float dir = -1.0;
#endif

NAxisMotion IMU;

//loop params
const int loopFrequency = 100; //hz
float loopTime = 1/(float)loopFrequency; //sec
unsigned long lastStreamTime = 0;
                
//car parameters
float wheel_base = 10.0 * 0.0254;
float margins = 4.0 * 0.0254;
float track_width = 10.5 * 0.0254; 
float wheel_radius = 0.0254 * 6.0 / 2.0;

//sensor readings
float leftWheelRevsPrevious = 0; float rightWheelRevsPrevious = 0;
float leftWheelVel = 0; float rightWheelVel = 0;
float yaw; float roll; float previousTheta; float previousX;

//state
float x = 0;
float dx = 0;
float theta = 0;
float dtheta = 0;

//balance control
float motorGain = 0.3*582.7652; //torque to motor command (linear relationship assumption)
float des_x = 0.0;
float des_dx = 0.0;
float des_theta = 0; //rad (not completely symmetric mass distribution)
float des_dtheta = 0.0;
float motorCommand = 0.0;
int deadZone = 0;

//lqr controller

//These gains are for teleoperation
//float K1 = 28.35; //x 
//float K2 = 26.92; //dx
//float K3 = 124.08; //theta
//float K4 = 14.02; //dtheta

//These gains are just for badass stability
float K1 = 0.9112; //x
float K2 = 3.4515; //dx
float K3 = 125.2892; //theta
float K4 = 14.8; //dtheta

//high level control
int counter = 0;
bool standbyFlag = 0;
enum gait {
  BALANCE,
  GO_FORWARD,
  STANDBY,
  JUMP
};
gait currGait = BALANCE;
bool debug = false;

void setup() 
{
    //start serial
    Serial.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }
    
    //init stuff
    I2C.begin();
    motorDriver.init();
    IMU.initSensor(); // I2C Address can be changed here
    IMU.setOperationMode(OPERATION_MODE_NDOF);
    IMU.setUpdateMode(MANUAL);  //The default is AUTO. MANUAL requires calling
                                //update functions prior to read
    
    delay(10);
}

void loop() 
{  
    //keep loop at set rate
    if ((millis() - lastStreamTime) >= (loopTime*1000)) {    
      lastStreamTime = millis(); 
      
      updateSensorReadings();
      
      gaitControl();
  
      if(debug) {
        Serial.print(" X: "); //m
        Serial.print(x);
        Serial.print(" dX: "); //m/s
        Serial.print(dx);
        Serial.print(" Theta: ");
        Serial.print(theta); //deg
        Serial.print(" dTheta: "); 
        Serial.print(dtheta); //deg/s
        Serial.print(" Desired X: ");
        Serial.print(des_x);
        Serial.print(" Motor Command: ");
        Serial.print(motorCommand); //-400 to 400
        Serial.print(" Gait: ");
        Serial.print(currGait);
        Serial.println();
      }
    }
}

void serialEvent()
{
  int incomingByte = Serial.read();
//  Serial.println(incomingByte, DEC);
  switch(incomingByte) 
  {
    case 56: 
      des_x = des_x + 0.1;

      break;
    case 50:
      des_x = des_x - 0.1;

      break;

    case 32:
      standbyFlag = 1;

    default:
    
      break;
  }
  
  
}

void balanceController()
{
    //lqr state feedback control
    float x_error = x-des_x;
    float dx_error = dx-des_dx;
    float theta_error = theta-des_theta;
    float dtheta_error = dtheta-des_dtheta;

    motorCommand = motorGain*((K1*x_error)+(K2*dx_error)+(K3*theta_error)+(K4*dtheta_error))*(3.0*0.5*0.0254);
    if (motorCommand >= 0.0) {
      motorCommand = map(motorCommand, 0, 400, deadZone, 400);
    }
    else if (motorCommand < 0.0) {
      motorCommand = map(motorCommand, -400, 0, -400, -deadZone);
    }
    motorCommand = constrain(motorCommand, -400, 400);
    if (abs(theta) > 1.3) {
         currGait = STANDBY;
         standbyFlag = 0;
    }
    motorDriver.setM2Speed(motorCommand);
    motorDriver.setM1Speed(motorCommand); 
    stopIfFault();
}

void standby() {
    motorDriver.setM2Speed(0);
    motorDriver.setM1Speed(0); 
}

void updateSensorReadings()
{        
    //update IMU information
    IMU.updateEuler();     
    IMU.updateCalibStatus(); // Update the Calibration Status
    yaw = degToRad( IMU.readEulerHeading() );
    roll = dir * IMU.readEulerPitch();
    
    //update wheel positions
    float leftWheelRevs = motorEncL.read() / 240.0;
    float rightWheelRevs = motorEncR.read() / 240.0;
    leftWheelVel = (leftWheelRevs - leftWheelRevsPrevious)/loopTime; 
    rightWheelVel = (rightWheelRevs - rightWheelRevsPrevious)/loopTime;

    //update state variables
    x = ((float)motorEncL.read() * 1.5 / 360.0)* (6.0*0.0254*M_PI);
    dx = (x - previousX)/loopTime;
    theta = dir * IMU.readEulerRoll() * 0.0174533; // IMU is rotated 90deg on robot
    dtheta = (theta - previousTheta)/loopTime;
    
    //keep track of variables for finite difference
    previousX = x;
    previousTheta = theta;
    leftWheelRevsPrevious = leftWheelRevs;
    rightWheelRevsPrevious = rightWheelRevs;
}

void gaitControl()
{   
    //finite state machine
    switch(currGait) {
      case BALANCE:
          balanceController();
//          if (millis()>10000 && counter < 50) {
//            currGait = GO_FORWARD;
//          }
          
          break;
  
      case GO_FORWARD:
          balanceController();
          des_x = des_x + 0.005;
          
          counter++;
          if (counter > 600) {
            currGait = BALANCE;
          }
            
          break;
      case STANDBY:
          standby();
          if (standbyFlag == 1)
            currGait = BALANCE;
            
          break;
      case JUMP:
          
          break;
    }
}

bool jump()
{     
    
}

//make sure motors connected
void stopIfFault()
{
    if (motorDriver.getM1Fault()) {
        Serial.println("M1 motor connection fault");
        while(1);
    }
    if (motorDriver.getM2Fault()) {
        Serial.println("M2 motor connection fault");
        while(1);
    }
}

//wrap to pi
float degToRad(float angle)
{
    angle = fmod(angle, 360.0);
    if(angle > 180.0) {
        angle = -(angle - 360.0) * M_PI / 180.0;
    } else {
        angle = -angle * M_PI / 180.0;   
    }
    return angle;
}
