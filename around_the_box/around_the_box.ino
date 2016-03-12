/*
Simple around the box program

- Brian Erickson
*/

#include <Wire.h>
#include <Zumo32U4.h>
#include "TurnSensor.h"

// This is the maximum speed the motors will be allowed to turn.
// A maxSpeed of 400 lets the motors go at top speed.  Decrease
// this value to impose a speed limit.
const int16_t max_speed = 400;

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

void show_stats_on_lcd() {
    lcd.clear();
    lcd.print(standardized_degrees(turn_degrees));
    lcd.gotoXY(0,1);
    lcd.print(proximity_sensors.countsRightWithLeftLeds());
    lcd.print(",");
    lcd.print(proximity_sensors.countsRightWithRightLeds());
}


void go_forward(double desired_degrees) {
  double heading_error = standardized_degrees(desired_degrees - turn_degrees);

  int left_speed = constrain(max_speed - 20 * heading_error,-max_speed,max_speed);
  int right_speed = constrain(max_speed + 20 * heading_error,-max_speed,max_speed);
  
  
  motors.setSpeeds(left_speed, right_speed);
}

// turns toward desired angle, returns true when done
bool turn_to_angle(float desired_degrees) {

  double turn_error = standardized_degrees(desired_degrees - turn_degrees);
  lcd.clear();
  lcd.print(turn_error);
  int32_t motor_speed = turn_error * 20 - turn_degrees_per_second*0.5;

  // Constrain our motor speeds to be between
  // -maxSpeed and maxSpeed.
  motor_speed= constrain(motor_speed, -max_speed, max_speed);

  motors.setSpeeds(-motor_speed, motor_speed);
  if(abs(turn_error)<0.5 && turnRate < 10) {
    return true;
  }
  return false;
  
}

void loop()
{
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
  proximity_sensors.read();

  
  if(every_100_ms)  {
    show_stats_on_lcd();
  }



  bool last_box_to_right = box_to_right;
  box_to_right = (proximity_sensors.countsRightWithRightLeds() > 3);

  // main state logic
  switch(current_step) {
    case 0:
      turnSensorReset();
      go_angle = 0;
      lcd.clear();
      ++current_step;
      lcd.clear();
      lcd.print((String)"step "+current_step);
      break;
    case 1:
    case 3:
    case 5:
    case 7:
    case 9:
      go_forward(go_angle);
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
      lcd.gotoXY(0,1);
      lcd.print("a replay");
      current_step++;
      break;
    case 11:
      if(buttonA.getSingleDebouncedRelease()) {
        current_step = 0;
      }
        
      break;
    default: // final state, do nothing
      break;
   
  }


  // remember last loop time
  last_loop_ms = loop_ms;
}
