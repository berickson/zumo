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

struct Location {double x; double y;} location = {0.,0.};

L3G gyro;

int start_millis;

void setup() {
  turnSensorSetup();
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
  return fmod((theta + 180 + 3600), 360.) - 180.;
}

void show_stats_on_lcd() {
    lcd.clear();
    lcd.print(standardized_degrees(turn_degrees));
    lcd.gotoXY(0,1);
    //lcd.print(proximity_sensors.countsRightWithLeftLeds());
    //lcd.print(",");
    //lcd.print(proximity_sensors.countsRightWithRightLeds());
    lcd.print((int)location.x);
    lcd.print(",");
    lcd.print((int)location.y);
}


void go_forward(double desired_degrees, double speed = max_speed) {
  double heading_error = standardized_degrees(desired_degrees - turn_degrees);

  int left_speed = constrain(speed - 20 * heading_error,-max_speed,max_speed);
  int right_speed = constrain(speed + 20 * heading_error,-max_speed,max_speed);
  
  
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

  if(abs(turn_error)<1.0 && abs(turnRate) < 10) {
    motors.setSpeeds(-0,0);
    return true;
  }

  // enforce minimum motor speed
  if(abs(motor_speed) < 50 && abs(turnRate) < 10) {
    if(motor_speed < 0) 
      motor_speed = -50;
    else
      motor_speed = 50;
  }
  motors.setSpeeds(-motor_speed, motor_speed);
  return false;
  
}

double heading_degrees_to(double x, double y) {
  double dx = x - location.x;
  double dy = y - location.y;
  return atan2(dy, dx) * 180./PI;
}

bool move_to(double x, double y) {
  double dx = x - location.x;
  double dy = y - location.y;
  double heading_sp = atan2(dy, dx) * 180./PI;
  double remaining = sqrt(dx*dx+dy*dy);
  if(remaining < 100) {
    motors.setSpeeds(0,0);
    return true;
  }
 
  double speed = remaining*0.5;
  go_forward(heading_sp, speed);
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
  proximity_sensors.read();


  // update robot location 
  {
    double forward_distance = encoders.getCountsAndResetLeft() + encoders.getCountsAndResetRight();
    location.x += forward_distance * cos(turn_degrees * PI/180.);
    location.y += forward_distance * sin(turn_degrees * PI/180.);
  }

  
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
 //   case 9:
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
      }
      break;
    case 9:
      current_step++;
      break;
    case 10:
      if(turn_to_angle(heading_degrees_to(0,0))) {
        current_step++;
      }
      break;

    case 11:
      if (move_to(0,0)) {
        current_step++;
      }
      break;
    case 12:
      if (turn_to_angle(0.0)) {
        current_step++;
      }
      break;
    case 13:
      motors.setSpeeds(0,0);
      lcd.clear();
      lcd.print("Done");
      lcd.gotoXY(0,1);
      lcd.print("a replay");
      current_step++;
      break;
    case 14:
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
