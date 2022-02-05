#include <Arduino.h>
#include <pins.h>

#include <Relay.h>
#include <MOSFET.h>

#include <Servo.h>

#include <L298N.h>

#include <STM32RTC.h>

#include <AccelStepper.h>

#include "SdFat.h"
#include "sdios.h"

#define ACTIVE_LED BLUE_LED


/* Get the rtc object */
STM32RTC &rtc = STM32RTC::getInstance();

/* Change these values to set the current initial time */
const byte seconds = 0;
const byte minutes = 0;
const byte hours = 16;

/* Change these values to set the current initial date */
/* Monday 15th June 2015 */
const byte weekDay = 1;
const byte day = 15;
const byte month = 6;
const byte year = 15;


Servo servo1;
Servo servo2;


Relay relay1(RELAY_1, false); // constructor receives (pin, isNormallyOpen) true = Normally Open, false = Normally Closed
Relay relay2(RELAY_2, false); // constructor receives (pin, isNormallyOpen) true = Normally Open, false = Normally Closed


Relay mosfet1(MOSFET_1, false); // constructor receives (pin, isNormallyOpen) true = Normally Open, false = Normally Closed
Relay mosfet2(MOSFET_2, false); // constructor receives (pin, isNormallyOpen) true = Normally Open, false = Normally Closed


// Create one motor instance
L298N dc_pump_1(DC_PUMP_1_PWM, DC_PUMP_1_EN_1, DC_PUMP_1_EN_2);
L298N dc_pump_2(DC_PUMP_2_PWM, DC_PUMP_2_EN_1, DC_PUMP_2_EN_2);
//L298N dc_pump_3(DC_PUMP_3_PWM, DC_PUMP_3_EN_1, DC_PUMP_3_EN_2);
L298N dc_pump_4(DC_PUMP_4_PWM, DC_PUMP_4_EN_1, DC_PUMP_4_EN_2);


AccelStepper stepper1(AccelStepper::DRIVER, STEPPER_1_STEP, STEPPER_1_DIR);
AccelStepper stepper3(AccelStepper::DRIVER, STEPPER_3_STEP, STEPPER_3_DIR);


void button_ISR()
{
  Serial.println("Reset");
  HAL_NVIC_SystemReset();
}


void setup()
{
  Serial.begin(115200);
  Serial.println("Water minilab");

  servo1.attach(SERVO_1_PWM);
  servo2.attach(SERVO_2_PWM);

  relay1.begin();
  relay2.begin();
  mosfet1.begin();
  mosfet2.begin();

  Serial.println("- configure RTC");
  // Select RTC clock source: LSI_CLOCK, LSE_CLOCK or HSE_CLOCK.
  // By default the LSI is selected as source.
  rtc.setClockSource(STM32RTC::LSE_CLOCK);

  rtc.begin(); // initialize RTC 24H format

  // Set the time
  rtc.setHours(hours);
  rtc.setMinutes(minutes);
  rtc.setSeconds(seconds);

  // Set the date
  rtc.setWeekDay(weekDay);
  rtc.setDay(day);
  rtc.setMonth(month);
  rtc.setYear(year);

  // you can use also
  rtc.setTime(hours, minutes, seconds);
  rtc.setDate(weekDay, day, month, year);


  pinMode(USER_BUTTON, INPUT);
  attachInterrupt(digitalPinToInterrupt(USER_BUTTON), button_ISR, FALLING);

  
  Serial.println("- testing DC pumps");
  dc_pump_2.setSpeed(255);
  dc_pump_2.forward();
  delay(3000);

  dc_pump_2.stop();
  delay(3000);

  dc_pump_2.setSpeed(127);
  dc_pump_2.backward();
  delay(3000);
  dc_pump_2.stop();
 


  Serial.println("- testing relays");
	relay1.turnOff(); //turns relay off
  relay2.turnOff(); //turns relay off
  delay(1000);
	relay1.turnOn();  //turns relay on
  relay2.turnOn();  //turns relay on
  delay(1000);
  relay1.turnOff(); //turns relay off
  relay2.turnOff(); //turns relay off
  delay(1000);
	relay1.turnOn();  //turns relay on
  relay2.turnOn();  //turns relay on

  Serial.println("- testing MOSFETS");
	mosfet1.turnOff(); //turns relay off
  mosfet2.turnOff(); //turns relay off
  delay(1500);
	mosfet1.turnOn();  //turns relay on
  mosfet2.turnOn();  //turns relay on
  delay(1500);
  mosfet1.turnOff(); //turns relay off
  mosfet2.turnOff(); //turns relay off

  Serial.println("- testing servos");
  for (int i = 0; i < 180; i++) {
    servo1.write(i);
    servo2.write(i);
    delay(3);
  }
  for (int i = 180; i > 0; i--) {
    servo1.write(i);
    servo2.write(i);
    delay(3);
  }

  Serial.println("- testing steppers");
  stepper1.setMaxSpeed(3000.0);
  stepper1.setAcceleration(3000.0);
  stepper1.moveTo(100);

  // stepper3.setMaxSpeed(3000.0);
  // stepper3.setAcceleration(3000.0);
  // stepper3.moveTo(100);
}


void loop()
{
  // // Print date...
  // Serial.printf("%02d/%02d/%02d ", rtc.getDay(), rtc.getMonth(), rtc.getYear());
  // // ...and time
  // Serial.printf("%02d:%02d:%02d.%03d\n", rtc.getHours(), rtc.getMinutes(), rtc.getSeconds(), rtc.getSubSeconds());
  // delay(1000);

  // Serial.println("- testing DC pumps");
  // dc_pump_2.setSpeed(255);
  // dc_pump_2.forward();
  // delay(3000);

  // dc_pump_2.stop();
  // delay(3000);

  // dc_pump_2.setSpeed(127);
  // dc_pump_2.backward();
  // delay(3000);
  // dc_pump_2.stop();

  // Change direction at the limits
  if (stepper1.distanceToGo() == 0)
  {
    stepper1.moveTo(-stepper1.currentPosition());
  }
  stepper1.run();

  // if (stepper3.distanceToGo() == 0)
  // {
  //   stepper3.moveTo(-stepper3.currentPosition());
  // }
  // stepper3.run();
}