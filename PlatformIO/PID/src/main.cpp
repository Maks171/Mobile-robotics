#include <Arduino.h>
// standard libraries
#include <stdio.h>
#include <time.h>
#include <math.h>
// external libraries
#include <PID_v1.h>
#include <pio_rotary_encoder.h>
// Motor encoder output pulses per 360 degree revolution
// 16 pulses, 120:1 reduction, doubled impulse number
#define ENC_COUNT_REV 3840
// initialise static encoder variables for both motors
int RotaryEncoder::rotation_motor_a = 0;
int RotaryEncoder::rotation_motor_b = 0;
double tick = 0.00163625;
int enc_left_beg;
int enc_left_end;
int enc_right_beg;
int enc_right_end;
double left_vel;
double right_vel;
double time_sample=250;

#include <PID_v1.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3

//Define Variables we'll be connecting to
double setpoint, output_left, output_right;

//Specify the links and initial tuning parameters
double Kp=2, Ki=0.0, Kd=0.0;
PID myPID_left(&left_vel, &output_left, &setpoint, Kp, Ki, Kd, DIRECT);
PID myPID_right(&right_vel, &output_right, &setpoint, Kp, Ki, Kd, DIRECT);


// create motor object
RotaryEncoder encoder_left(2, 3, MOTOR_A_SM);
RotaryEncoder encoder_right(26, 27, MOTOR_B_SM);
void setup() {
// initialise encoders
encoder_left.set_rotation(0);
encoder_right.set_rotation(0);
SerialUSB.begin(115200);
setpoint = 2*3.14;

  //turn the PID on
myPID_left.SetMode(AUTOMATIC);
myPID_right.SetMode(AUTOMATIC);
myPID_left.SetSampleTime(250);
myPID_right.SetSampleTime(250);

delay(1000);
}
void loop() {
//print encode readings in ticks

//SerialUSB.print("enc_left_ticks:= ");
//SerialUSB.println(encoder_left.get_rotation());
enc_left_beg = encoder_left.get_rotation();
//SerialUSB.print("enc_right_ticks:= ");
//SerialUSB.println(encoder_right.get_rotation());
enc_right_beg = encoder_right.get_rotation();

delay(time_sample);
enc_left_end = encoder_left.get_rotation();
enc_right_end = encoder_right.get_rotation();

left_vel = ((enc_left_end-enc_left_beg)/time_sample)*tick*1000;
right_vel = (-1*(enc_right_end-enc_right_beg)/time_sample)*tick*1000;
SerialUSB.print("left_vel:= ");
SerialUSB.println(left_vel);
SerialUSB.print("right_vel:= ");
SerialUSB.println(right_vel);

myPID_left.Compute();
myPID_right.Compute();

if (output_left>0) {
analogWrite(10,int(output_left/16.755*255));
analogWrite(11,0);
} else {

analogWrite(11,int(output_left/16.755*255));
analogWrite(10,0);
}

if (output_right<0) {
analogWrite(8,int(output_right/16.755*255));
analogWrite(9,0);
} else {

analogWrite(9,int(output_right/16.755*255));
analogWrite(8,0);
}



}