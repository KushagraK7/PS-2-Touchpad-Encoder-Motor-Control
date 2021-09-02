/*
*   Arduino code for controlling a position of a DC motor with a rotary encoder with a PS/2 touchpad.
*   
*   Made by KushagraK7: https://www.instructables.com/member/KushagraK7/
*
*
*/
#include <PIDController.h>
#include<ps2.h>         //Library for interfacing the PS/2 touchpad with the Arduino MCU.

/* ENCODER_A and ENCODER_B pins are used to read the encoder
   data from the microcontroller, the data from the encoder
   comes very fast so these two pins have to be interrupt enabled
   pins
*/
#define ENCODER_A 2
#define ENCODER_B 3
/* the MOTOR_CW and MOTOR_CCW pins are used to drive the H-bridge
   the H-bridge then drives the motors, This two pins must have to
   be PWM enabled, otherwise the code will not work.
*/
#define MOTOR_CW 9
#define MOTOR_CCW 10
/*In this section we have defined the gain values for the
   proportional,integral, and derivative controller i have set
   the gain values with the help of trial and error methods.
*/
#define __Kp 100 // Proportional constant
#define __Ki 2.5 // Integral Constant
#define __Kd 8000 // Derivative Constant

volatile long int encoder_count = 0; // stores the current encoder count
int motor_pwm_value = 255; // after PID computation data is stored in this variable.

char mstate;
char mx, my;
int x, m;

long p = 0;

PIDController pidcontroller;

PS2 mouse(6, 5);//(Clock, Data)

    void mouse_init()
    {
      mouse.write(0xff);  // reset
      mouse.read();  // ack byte
      mouse.read();  // blank */
      mouse.read();  // blank */
      mouse.write(0xf0);  // remote mode
      mouse.read();  // ack
      delayMicroseconds(100);
    }

    void encoder() {
      if (digitalRead(ENCODER_B) == HIGH) // if ENCODER_B is high increase the count
        encoder_count++; // increment the count
      else // else decrease the count
        encoder_count--;  // decrement the count
    }
    void motor_cw(int power) {
      if (power > 70) {
        analogWrite(MOTOR_CW, power); //rotate the motor if the value is grater than 100
        digitalWrite(MOTOR_CCW, LOW); // make the other pin LOW
      }
      else {
        // both of the pins are set to low
        digitalWrite(MOTOR_CW, LOW);
        digitalWrite(MOTOR_CCW, LOW);
      }
    }
    void motor_ccw(int power) {
      if (power > 70) {
        analogWrite(MOTOR_CCW, power);
        digitalWrite(MOTOR_CW, LOW);
      }
      else {
        digitalWrite(MOTOR_CW, LOW);
        digitalWrite(MOTOR_CCW, LOW);
      }
    }

void setup() {
  Serial.begin(115200); // Serial for Debugging
  pinMode(ENCODER_A, INPUT); // ENCODER_A as Input
  pinMode(ENCODER_B, INPUT); // ENCODER_B as Input
  pinMode(MOTOR_CW, OUTPUT); // MOTOR_CW as Output
  pinMode(MOTOR_CCW, OUTPUT); // MOTOR_CW as Output
  /* attach an interrupt to pin ENCODER_A of the Arduino, and when the
      pulse is in the RISING edge called the function encoder().
  */
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder, RISING);

  mouse_init();
  
  pidcontroller.begin(); // initialize the PID instance
  pidcontroller.tune(__Kp, __Ki, __Kd); // Tune the PID, arguments: kP, kI, kD
  pidcontroller.limit(-255, 255); // Limit the PID output between -255 to 255, this is important to get rid of integral windup!
  

}

void loop() {
  // put your main code here, to run repeatedly:

  /* get a reading from the mouse */
  mouse.write(0xeb);  // give me data!
  mouse.read();      // ignore ack
  mstate = mouse.read();
  mx = mouse.read();
  my = mouse.read();

   m = (int)mstate;
   x = (int)mx;

   Serial.println(p);

   p += x;

   if(m == 9) p = 0;
   
   pidcontroller.setpoint(p); // The "goal" the PID controller tries to "reach"

   motor_pwm_value = pidcontroller.compute(encoder_count);  //Let the PID compute the value, returns the calculated optimal output

   if (motor_pwm_value > 0) // if the motor_pwm_value is greater than zero we rotate the  motor in clockwise direction
    motor_ccw(motor_pwm_value);
   else // else we move it in a counter clockwise direction
    motor_cw(abs(motor_pwm_value));
     
}
