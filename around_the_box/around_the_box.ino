/* This demo shows how the Zumo can use its gyroscope to detect
when it is being rotated, and use the motors to resist that
rotation.

This code was tested on a Zumo with 4 NiMH batteries and two 75:1
HP micro metal gearmotors.  If you have different batteries or
motors, you might need to adjust the PID constants.

Be careful to not move the robot for a few seconds after starting
it while the gyro is being calibrated.  During the gyro
calibration, the yellow LED is on and the words "Gyro cal" are
displayed on the LCD.

After the gyro calibration is done, press button A to start the
demo.  If you try to turn the Zumo, or put it on a surface that
is turning, it will drive its motors to counteract the turning.

This demo only uses the Z axis of the gyro, so it is possible to
pick up the Zumo, rotate it about its X and Y axes, and then put
it down facing in a new position. */

#include <Wire.h>
#include <Zumo32U4.h>
#include "TurnSensor.h"

// This is the maximum speed the motors will be allowed to turn.
// A maxSpeed of 400 lets the motors go at top speed.  Decrease
// this value to impose a speed limit.
const int16_t max_speed = 100;

Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonC buttonC;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4ProximitySensors proximity_sensors;

L3G gyro;

int start_millis;
void setup()
{
  turnSensorSetup();
  delay(500);
  turnSensorReset();
  proximity_sensors.initThreeSensors();
  lcd.print(readBatteryMillivolts());
  start_millis = millis();
}



// returns true if loop time passes through n ms boundary
bool every_n_ms(unsigned long last_loop_ms, unsigned long loop_ms, unsigned long ms) {
  return (last_loop_ms % ms) + (loop_ms - last_loop_ms) >= ms;
}

// returns theta in [-180,180)
double standardized_degrees(double theta) {
  return fmod((theta + 180 + 360), 360.) - 180.;
}

void show_proximity_on_lcd() {
    lcd.clear();
    lcd.print(standardized_degrees(turn_degrees));
    lcd.gotoXY(0,1);
    //lcd.print(turn_degrees_per_second);
    lcd.print(proximity_sensors.countsRightWithLeftLeds());
    lcd.print(",");
    lcd.print(proximity_sensors.countsRightWithRightLeds());
}


void go_forward() {
  motors.setSpeeds(max_speed , max_speed);
}

// turns toward desired angle, returns true when done
bool turn_to_angle(float desired_degrees) {
  const int max_speed = 200;

  double turn_error = standardized_degrees(desired_degrees - turn_degrees);
  lcd.clear();
  lcd.print(turn_error);
  int32_t motor_speed = turn_error * 30 - turn_degrees_per_second*0.5;

  // Constrain our motor speeds to be between
  // -maxSpeed and maxSpeed.
  motor_speed= constrain(motor_speed, -max_speed, max_speed);

  motors.setSpeeds(-motor_speed, motor_speed);
  if(abs(turn_error)<0.3 && turnRate < 5) {
    return true;
  }
  return false;
  
}

void loop()
{
  static int16_t goal_encoder_count = 0;
  static unsigned int last_loop_ms = 0;
  static int current_step = 0;
  static bool box_to_right = false;
  static int go_angle = 0;

  // update loop timers
  unsigned int loop_ms = millis();
  bool every_1_s = every_n_ms(last_loop_ms, loop_ms, 1000);
  bool every_100_ms = every_n_ms(last_loop_ms, loop_ms, 100);

  // update all sensors
  turnSensorUpdate();
  int16_t counts_left = encoders.getCountsLeft();
  int16_t counts_right = encoders.getCountsRight();
  bool proximity_left_active = proximity_sensors.readBasicLeft();
  bool proximity_front_active = proximity_sensors.readBasicFront();
  bool proximity_right_active = proximity_sensors.readBasicRight();
  proximity_sensors.read();

  
  if(every_100_ms)  {
    show_proximity_on_lcd();
  }

  if(every_1_s)  {
    goal_encoder_count += 300;
  }


  int last_box_to_right = box_to_right;
  if (proximity_sensors.countsRightWithRightLeds() >= 3)
    box_to_right++;
  else
    box_to_right = 0;

  // main state logic
  switch(current_step) {
    case 0:
      lcd.clear();
      lcd.print("step 1");
      ++current_step;
      lcd.clear();
      lcd.print((String)"step "+current_step);
      break;
    case 1:
    case 3:
    case 5:
    case 7:
    case 9:
      go_forward();
      if (last_box_to_right && !box_to_right) {
        go_angle -= 90;
        go_angle = standardized_degrees(go_angle);
        ++current_step;
        lcd.clear();
        lcd.print((String)"step "+current_step);
      }
      break;
    case 2:
    case 4:
    case 6:
    case 8:
      if (turn_to_angle(go_angle)) {
        ++current_step;
        lcd.clear();
        lcd.print((String)"step "+current_step);
      }
      break;
    case 10:
      motors.setSpeeds(0,0);
      lcd.clear();
      lcd.print("Done");
      current_step++;
      break;
    default: // final state, do nothing
      break;
   
  }


  // remember last loop time
  last_loop_ms = loop_ms;
}
