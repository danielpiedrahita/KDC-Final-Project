#include <Encoder.h>
#include <Wire.h>
#include <NAxisMotion.h>
#include <DualVNH5019MotorShield.h>
#include <PID_v1.h>


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

//ground control
float nominalForwardSpeed = 3.5;
double motorVelL = 0.0, motorCmdL = 0.0, motorVelR = 0.0, motorCmdR = 0.0;
double desiredVelL = nominalForwardSpeed;
double desiredVelR = nominalForwardSpeed;
PID pidL(&motorVelL, &motorCmdL, &desiredVelL, 145.0, 80.0, 2.0, DIRECT);
PID pidR(&motorVelR, &motorCmdR, &desiredVelR, 150.0, 80.0, 2.0, DIRECT); 
        // args: input, output, setpoint, Kp, Ki, Kd, mode


//flight control
                
//car parameters
float wheel_base = 10.0 * 0.0254;
float margins = 4.0 * 0.0254;
float track_width = 10.5 * 0.0254; 
float wheel_radius = 0.0254 * 6.0 / 2.0;

//sensor readings
float leftWheelRevsPrevious = 0; float rightWheelRevsPrevious = 0;
float leftWheelVel = 0; float rightWheelVel = 0;
float roll;
float xAccel = 0.0, yAccel = 0.0, zAccel = 0.0;

//state
float x = 0;
float dx = 0;
float theta = 0;
float dtheta = 0;
float yaw = 0;
float dyaw = 0;

float previousTheta; float previousX; float previousYaw;

//balance control
float motorGain = 0.2*582.7652; //torque to motor command (linear relationship assumption) 0.3
float force2torque = 3.0*0.5*0.0254;
float motorCommandLeft = 0.0;
float motorCommandRight = 0.0;
float balanceCommand = 0.0;
float turnCommand = 0.0;
int deadZone = 0;

float des_x = 0.0;
float des_dx = 0.0;
float des_theta = 0; //rad
float des_dtheta = 0.0;
float des_yaw = 0.0;
float des_dyaw = 0.0;

//lqr controller
//These gains are for teleoperation 100 hz
float K1 = 28.3; //x 20.125
float K2 = 26.9; //dx 31.23
float K3 = 173.14; //theta 125.28
float K4 = 21.4; //dtheta 16.02
float K1turn = 2.0; //0.678
float K2turn = 0.998; //0.397

////teleop 50 hz
//float K1 = 25.42; //x 20.125
//float K2 = 24.29; //dx 31.23
//float K3 = 162.9; //theta 125.28
//float K4 = 20.06; //dtheta 16.02
//float K1turn = 1.8; 
//float K2turn = 0.9; 

////These gains are just for badass stability
//float K1 = 0.9112; //x
//float K2 = 3.4515; //dx
//float K3 = 125.2892; //theta
//float K4 = 16.0; //dtheta

//high level control
int counter = 0;
bool standbyFlag = 0;
enum gait {
  BALANCE,
  STRAIGHT,
  GO_FORWARD,
  STANDBY,
  FALL
};
gait currGait = STRAIGHT;
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
    IMU.setUpdateMode(MANUAL); 
    
    //PID init
    float K = 1000.0 / (240 / (0.1524 * 3.14159));
                // (ms/s) / (ticksPerMeter)
                // ticksPerMeter = ticksPerRev / MetersPerRev
    pidL.SetOutputLimits(0, 400);  // -400 for reverse motion
    pidL.SetSampleTime(25); // 40 hz
    pidL.SetWheelParam(K);
    pidL.SetMode(AUTOMATIC);
    pidR.SetOutputLimits(0, 400);  // -400 for reverse motion
    pidR.SetSampleTime(25);
    pidR.SetWheelParam(K);
    pidR.SetMode(AUTOMATIC);
    
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
        Serial.print(theta); //rad
        Serial.print(" dTheta: "); 
        Serial.print(dtheta); //rad/s
        Serial.print(" Yaw: "); //rad
        Serial.print(yaw);
        Serial.print(" dYaw: ");
        Serial.print(dyaw);
        Serial.print(" Desired Yaw: ");
        Serial.print(des_yaw);
        Serial.print(" Desired X: ");
        Serial.print(des_x);
        Serial.print(" Left Motor Command: ");
        Serial.print(motorCommandLeft); //-400 to 400
        Serial.print(" Right Motor Command: ");
        Serial.print(motorCommandRight);
        Serial.print(" Gait: ");
        Serial.print(currGait);
        Serial.println();
      }
    //Serial.println(millis() - lastStreamTime);
    }
}

void wheelSpeedFeedback()
{
    pidL.ComputeVelocity(motorEncL.read());
    motorDriver.setM2Speed(motorCmdL);
    pidR.ComputeVelocity(motorEncR.read());
    motorDriver.setM1Speed(motorCmdR); 
    stopIfFault();
}

void serialEvent()
{
  int incomingByte = Serial.read();
  Serial.println(incomingByte, DEC);
  switch(incomingByte) 
  {
    case 56: 
      des_x = des_x + 0.1;

      break;
    case 50:
      des_x = des_x - 0.1;

      break;

    case 52: //left
      des_yaw = des_yaw + 0.1;

      break;

    case 54: //right
      des_yaw = des_yaw - 0.1;

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
    float yaw_error = yaw - des_yaw;
    float dyaw_error = dyaw - des_dyaw;

    balanceCommand = force2torque*motorGain*((K1*x_error)+(K2*dx_error)+(K3*theta_error)+(K4*dtheta_error));
    turnCommand = force2torque*motorGain*((K1turn*yaw_error)+(K2turn*dyaw_error));

    motorCommandLeft = balanceCommand + turnCommand;
    motorCommandRight = balanceCommand - turnCommand;
    
    //limit motor commands
    motorCommandLeft = constrain(motorCommandLeft, -400, 400);
    motorCommandRight = constrain(motorCommandRight, -400, 400);

    //kill motors if it falls over
    if (abs(theta) > 1.3) {
         currGait = STANDBY;
         standbyFlag = 0;
    }

    //set motor commands
    motorDriver.setM2Speed(motorCommandLeft);
    motorDriver.setM1Speed(motorCommandRight); 
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
    IMU.updateAccel();
    IMU.updateCalibStatus(); // Update the Calibration Status
    yaw = degToRad( IMU.readEulerHeading() );
    roll = dir * IMU.readEulerPitch();
    zAccel = IMU.readAccelX(); //IMU is on its side
    Serial.println(zAccel);

    
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
    yaw = degToRad( IMU.readEulerHeading() );
    dyaw = (yaw - previousYaw)/loopTime;
    
    //keep track of variables for finite difference
    previousYaw = yaw;
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
          
          break;
      case GO_FORWARD:
          balanceController();
          des_x = des_x + 0.005;
          
          counter++;
          if (counter > 600) {
            currGait = BALANCE;
          }
            
          break;
      case STRAIGHT:
          wheelSpeedFeedback();

          if (zAccel > -1) {
            currGait = FALL;
            Serial.println("FALLING!");
          }
          
          break;
      case STANDBY:
          standby();
          if (standbyFlag == 1)
            currGait = BALANCE;
            
          break;
      case FALL:
          orientationControl();
          
          break;
    }
}

void orientationControl()
{     
  motorDriver.setM1Speed(0);
  motorDriver.setM2Speed(0);
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
