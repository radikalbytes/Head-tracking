// Robot on two wheels
// Change 'bias' to make the robot turn faster or slower (zero for stationary)
//  k1, k2, k3, k4 feedback gains are specified below

const tSensors GyroSensor          = (tSensors) S1;   //gyro sensor//

#define GyroScale 4
#define half_h 2     // Increment used in Runge-Kutta integration
#define t_scale 500
#define minDist 30  // distance before robot turns
#define v 0.5      // This is the desired velocity of the robot

task main ()
{
  float a=0.2, aa=0.001;   // aa is the filter time-constant to take care of the gyro drift
  float theta_bias = 0;
  float a_pwr=1, f_pwr=0;
  // This is the feedback loop gain
  float k1 = 0.0;     // position feedback gain
  float k2 = 90;      // velocity feedback gain
  float k3 = 8;      // tilt feedback gain
  float k4 = 10.0;    // angular velocity feedback gain
  float k_d = 16;         // how fast react to obstacles
  float k_e = 6.5;    // feedback to keeps the two wheels in-sync
  long f1=0, f2=0, Gyro_value = 0;
  int pwr=0, pwr_th, pwr_w, e=0, batt=0;
  float GyroBias = 600;  // This is the bias that I had to apply to my gyro sensor.  You may need to find your own gyro bias
  float k_pwr;  // This is the gain that is computed adaptively based on the battery voltage (as the battery is drained, the gain is increased)


  float theta=0, theta_old=0, t_old=0;
  float x=0, x_old=0, x_dot=0;
  float dist=0;
  int in_avoidance_state=0, delta_dist=0, turn_offset=0;

  // This causes the motors to stop when they are set to zero
   bFloatDuringInactiveMotorPWM = false;
    theta_old = 0;
    t_old= nSysTime;
    f1=0;
    x_old=0;


   ClearTimer(T1);
   // Find the gyro bias associated w/ the balanced position
   // Hold the robot in the balanced position for 3 sec to find the gyro bias
   GyroBias = 0;
   while (time1[T1] < 3000) {
      // filter the sensor output
      Gyro_value = SensorValue(GyroSensor);
      wait1Msec(100);
       GyroBias = (1-a)*GyroBias + a*Gyro_value;
   }
   // play a sound when the training is over
   PlaySound(soundBlip);

  // I ended up hard-coding the bias after measuring it a few times.
  // comment out the following line if you want the Gyro bias to be measured adaptively
   GyroBias = 600.5;

   // Measure the battery voltage and compensate for it by adjusting the gain (k_pwr)
  batt=nAvgBatteryLevel;
  k_pwr = 0.7 + (0.7-1.1)/(8816-8196)*(batt-8816);

  nMotorEncoder[motorC] = 0;
  nMotorEncoder[motorA] = 0;

  // Use the sonar sensor for collisoin avoidance
   SetSensorType(S3, sensorSONAR);
 while(true) {
      dist = SensorRaw[S3];
      delta_dist = dist - minDist;
     if ((delta_dist < 0) && (in_avoidance_state==0)) {  // are we close to an obstacle?
        in_avoidance_state=1;
        ClearTimer(T3);   // If yes, turn for 2.5 second
      }
      if ((in_avoidance_state==1) && (time1(T3) > 2500)) {
         in_avoidance_state=0;
         // find out how much more motorA has turned so that we can compensate for it later (when the motors are sync'ed up)
        turn_offset = nMotorEncoder[motorA] - nMotorEncoder[motorC];
      }


      // Runge-Kutta integration (http://en.wikipedia.org/wiki/Runge-kutta)
     wait1Msec(half_h);
     f2 = (SensorValue(GyroSensor)-GyroBias)/GyroScale; // f(tn+h/2)
      wait1Msec(half_h);
       theta = theta_old + (f1+2*f2)*(nSysTime-t_old)/t_scale;
       theta_old = theta;
     x = nMotorEncoder[motorC];
     // compute the linear velocity
     x_dot = x-x_old;
       f1 = (SensorValue(GyroSensor)-GyroBias)/GyroScale; // f(tn)
       t_old = nSysTime;
     x_old = x;

     // Compute the long-term average of tilt
    theta_bias = theta_bias*(1-aa) + theta*aa;


       pwr_th = k3*(theta-theta_bias);
       pwr_w = k4*f1;
       pwr = pwr_th + pwr_w + k1*x + k2*(x_dot-(v));
       f_pwr = (1-a_pwr)*f_pwr + a_pwr*pwr;
       e = nMotorEncoder[motorA] - nMotorEncoder[motorC] - turn_offset;
       // k_e is to keep the robot going straight
      motor[motorA] = k_pwr*f_pwr - k_d*in_avoidance_state - k_e*e*(1-in_avoidance_state);
      motor[motorC] = k_pwr*f_pwr + k_d*in_avoidance_state;
