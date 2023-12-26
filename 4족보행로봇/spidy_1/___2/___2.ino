#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <FlexiTimer2.h>
#include <SoftwareSerial.h>

SoftwareSerial blue(3, 4);

/*   front
   _________
  |0       2|
  |         |
  |1       3|
  |_________| 
  다리 길이 순서: c -> a -> b
*/

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
const int servo_pin[4][3] = { { 3, 4, 5 }, { 6, 7, 8 }, { 9, 10, 11 }, { 12, 13, 14 } };

const float length_a = 55;
const float length_b = 77.5;
const float length_c = 27.5;
const float z_absolute = -28;
const float half_body_wide = 68 / 2; // 넓이 / 2
const float half_body_length = 75 / 2; // 길이 / 2

/* Constants for movement ----------------------------------------------------*/
const float z_default = -50, z_default_up = -30, z_boot = z_absolute, z_boot_up = -12;
volatile float z_now = z_default;
volatile float z_up = z_default_up;
const float x_default = 62, x_step = 40;
const float y_start = 0, y_step = 40;
const float y_default = x_default;

/* variables for movement ----------------------------------------------------*/
volatile float site_now[4][3];
volatile float site_expect[4][3];
float temp_speed[4][3];
float move_speed;
float speed_multiple = 1;

const float spot_turn_speed = 4;
const float leg_move_speed = 4;
const float body_move_speed = 3;
const float stand_seat_speed = 1;
volatile int rest_counter;

const float KEEP = 255;

//define PI for calculation
const float pi = 3.1415926;
const float to_angle = 180 / pi;

/* variables for turn ----------------------------------------------------*/
const float turn_angle = 15 * pi / 180;
const float turn_x = half_body_wide * (cos(turn_angle) - 1) - half_body_length * sin(turn_angle);
const float turn_y = half_body_length * (cos(turn_angle) - 1) + half_body_wide * sin(turn_angle);

void setup() {
  Wire.begin();
  pwm.begin();
  Serial.begin(9600);
  blue.begin(9600);

  pwm.setPWMFreq(50);
  // Serial.println("Robot starts initialization");
  set_site(0, x_default, y_start + y_step, z_now);
  set_site(1, x_default, y_start + y_step, z_now);
  set_site(2, x_default, y_start + y_step, z_now);
  set_site(3, x_default, y_start + y_step, z_now);

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      site_now[i][j] = site_expect[i][j];
    }
  }

  delay(100);

  //start servo service
  FlexiTimer2::set(50, servo_service);
  FlexiTimer2::start();
  Serial.println("Servo service started");
  //initialize servos
  // servo_attach();
  Serial.println("Servos initialized");
}

void servo_attach(void) {
  int pwm_90 = map(90, 0, 180, 122, 500);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      pwm.setPWM(servo_pin[i][j], 1, pwm_90);
      delay(100);
    }
  }

}

void loop() {
  // put your main code here, to run repeatedly:

  if (blue.available()) {
    
    char data = blue.read();
    Serial.print(data);
    Serial.println();

    if (data == 'F') {
      Serial.println("Step forward");
      step_forward();
    } 
    else if (data == 'B') {
      Serial.println("Step back");
      step_back();
    } 
    else if (data == 'l') {
      Serial.println("Turn left");
      turn_left();
    } 
    else if (data == 'r') {
      Serial.println("Turn right");
      turn_right();
    } 
    else if (data == 'S') {
      Serial.println("Stand");
      stand();
    } 
    else if (data == 's') {
      Serial.println("Sit");
      sit();
    } 
    else if (data == 'L') {
      Serial.println("Step Left");
      step_left();
    } 
    else if (data == 'R') {
      Serial.println("Step Right");
      step_right();
    } 
    else if (data == 'Z') {
      Serial.println("Reposition");
      reposition();
    }
    else if (data == '0') {
      set_site(0, x_default, y_start + y_step, z_up-100);
      wait_all_reach();
      delay(1000);
      set_site(0, x_default, y_start + y_step, z_now); 
      wait_all_reach();
    }
    else if (data == '1') {
      set_site(1, x_default, y_start + y_step, z_up);
      wait_all_reach();
      delay(1000);
      set_site(1, x_default, y_start + y_step, z_now); 
      wait_all_reach();
    }
    else if (data == '2') {
      set_site(2, x_default, y_start + y_step, z_up);
      wait_all_reach();
      delay(1000);
      set_site(2, x_default, y_start + y_step, z_now); 
      wait_all_reach();
    }
    else if (data == '3') {
      set_site(3, x_default, y_start + y_step, z_up);
      wait_all_reach();
      delay(1000);
      set_site(3, x_default, y_start + y_step, z_now); 
      wait_all_reach();
    }
  }
}
void sit() {
  z_now = z_boot;
  z_up = z_boot_up;
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++) {
    set_site(leg, KEEP, KEEP, z_now);
  }
  wait_all_reach();
}

void stand() {
  z_now = z_default;
  z_up = z_default_up;
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++) {
    set_site(leg, KEEP, KEEP, z_now);
  }
  wait_all_reach();
}


void step_forward() {

  if (site_now[0][1] == y_start) {

    leg_move(0, x_default, y_start + 2 * y_step);
    leg_move(1, x_default, y_start);

    move_speed = body_move_speed;

    set_site(0, x_default, y_start + y_step, z_now);
    set_site(1, x_default, y_start + y_step, z_now);
    set_site(2, x_default, y_start, z_now);
    set_site(3, x_default, y_start + 2 * y_step, z_now);
    wait_all_reach();

  } else {

    leg_move(2, x_default, y_start + 2 * y_step);
    leg_move(3, x_default, y_start);

    move_speed = body_move_speed;

    set_site(0, x_default, y_start, z_now);
    set_site(1, x_default, y_start + 2 * y_step, z_now);
    set_site(2, x_default, y_start + y_step, z_now);
    set_site(3, x_default, y_start + y_step, z_now);
    wait_all_reach();
  }
}

void step_back() {

  if (site_now[3][1] == y_start) {

    leg_move(3, x_default, y_start + 2 * y_step);
    leg_move(2, x_default, y_start);

    move_speed = body_move_speed;

    set_site(0, x_default, y_start + 2 * y_step, z_now);
    set_site(1, x_default, y_start, z_now);
    set_site(2, x_default, y_start + y_step, z_now);
    set_site(3, x_default, y_start + y_step, z_now);



    wait_all_reach();

  } else {

    leg_move(1, x_default, y_start + 2 * y_step);
    leg_move(0, x_default, y_start);

    move_speed = body_move_speed;

    set_site(0, x_default, y_start + y_step, z_now);
    set_site(1, x_default, y_start + y_step, z_now);
    set_site(2, x_default, y_start + 2 * y_step, z_now);
    set_site(3, x_default, y_start, z_now);

    wait_all_reach();
  }
}

void step_left() {

  if (site_now[1][1] == x_default - x_step) {

    leg_move(1, x_default + x_step, y_start + y_step);
    leg_move(3, x_default - x_step, y_start + y_step);

    move_speed = body_move_speed;

    set_site(0, x_default - x_step, y_start + y_step, z_now);
    set_site(1, x_default, y_start + y_step, z_now);
    set_site(2, x_default + x_step, y_start + y_step, z_now);
    set_site(3, x_default, y_start + y_step, z_now);
    wait_all_reach();

  } else {

    leg_move(0, x_default + x_step, y_start + y_step);
    leg_move(2, x_default - x_step, y_start + y_step);

    move_speed = body_move_speed;

    set_site(0, x_default, y_start + y_step, z_now);
    set_site(1, x_default - x_step, y_start + y_step, z_now);
    set_site(2, x_default, y_start + y_step, z_now);
    set_site(3, x_default + x_step, y_start + y_step, z_now);
    wait_all_reach();
  }
}

void step_right() {

  if (site_now[2][1] == x_default - x_step) {

    leg_move(2, x_default + x_step, y_start + y_step);
    leg_move(0, x_default - x_step, y_start + y_step);

    move_speed = body_move_speed;

    set_site(0, x_default, y_start + y_step, z_now);
    set_site(1, x_default + x_step, y_start + y_step, z_now);
    set_site(2, x_default, y_start + y_step, z_now);
    set_site(3, x_default - x_step, y_start + y_step, z_now);

    wait_all_reach();

  } else {

    leg_move(3, x_default + x_step, y_start + y_step);
    leg_move(1, x_default - x_step, y_start + y_step);

    move_speed = body_move_speed;

    set_site(0, x_default + x_step, y_start + y_step, z_now);
    set_site(1, x_default, y_start + y_step, z_now);
    set_site(2, x_default - x_step, y_start + y_step, z_now);
    set_site(3, x_default, y_start + y_step, z_now);

    wait_all_reach();
  }
}

void turn_left() {
  reposition();

  set_site(0, x_default - turn_x, y_start + y_step + turn_y, z_now);
  set_site(1, x_default + turn_x, y_start + y_step - turn_y, z_now);
  set_site(2, x_default + turn_x, y_start + y_step - turn_y, z_now);
  set_site(3, x_default - turn_x, y_start + y_step + turn_y, z_now);

  reposition();
}

void turn_right() {
  reposition();

  set_site(0, x_default + turn_x, y_start + y_step - turn_y, z_now);
  set_site(1, x_default - turn_x, y_start + y_step + turn_y, z_now);
  set_site(2, x_default - turn_x, y_start + y_step + turn_y, z_now);
  set_site(3, x_default + turn_x, y_start + y_step - turn_y, z_now);

  reposition();
}


void servo_service(void) {
  sei();
  static float alpha, beta, gamma;  // gamma => body servo, alpha => coxa servo, beta => tibia servo

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
        site_now[i][j] += temp_speed[i][j];
      else
        site_now[i][j] = site_expect[i][j];
    }

    cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
    polar_to_servo(i, alpha, beta, gamma);
  }

  rest_counter++; //0.02초 지연(딜레이)
}

void set_site(int leg, float x, float y, float z) {
  float length_x = 0, length_y = 0, length_z = 0;

  if (x != KEEP)
    length_x = x - site_now[leg][0];
  if (y != KEEP)
    length_y = y - site_now[leg][1];
  if (z != KEEP)
    length_z = z - site_now[leg][2];

  float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

  temp_speed[leg][0] = length_x / length * move_speed;
  temp_speed[leg][1] = length_y / length * move_speed;
  temp_speed[leg][2] = length_z / length * move_speed;

  if (x != KEEP)
    site_expect[leg][0] = x;
  if (y != KEEP)
    site_expect[leg][1] = y;
  if (z != KEEP)
    site_expect[leg][2] = z;
}

void leg_move(int leg, float x, float y) {
  move_speed = leg_move_speed;
  set_site(leg, site_now[leg][0], site_now[leg][1], z_up);
  wait_all_reach();
  set_site(leg, x, y, z_up);
  wait_all_reach();
  set_site(leg, x, y, z_now);
  wait_all_reach();
}

void reposition() {
  for (int leg = 0; leg < 4; leg++) {
    if (site_now[leg][0] != x_default || site_now[leg][1] != y_start + y_step) {
      leg_move(leg, x_default, y_start + y_step);
    }
  }
}

void wait_reach(int leg) {
  while (1)
    if (site_now[leg][0] == site_expect[leg][0])
      if (site_now[leg][1] == site_expect[leg][1])
        if (site_now[leg][2] == site_expect[leg][2])
          break;
}

/*
  - wait all of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void wait_all_reach(void) {
  for (int i = 0; i < 4; i++)
    wait_reach(i);
}

void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z) {
  //calculate w-z degree
  float v, w;
  w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
  v = w - length_c;
  alpha = atan2(z, v) + acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
  beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_a / length_b);
  //calculate x-y-z degree
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);

  //trans degree pi->180
  alpha = alpha * to_angle;
  beta = beta * to_angle;
  gamma = gamma * to_angle;

  
}

void polar_to_servo(int leg, float alpha, float beta, float gamma) {
  if (leg == 0) {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  } else if (leg == 1) {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  } else if (leg == 2) {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  } else if (leg == 3) {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  }
 int pwm_alpha = map(alpha, 0, 180, 122, 500);
 if( leg == 0 ) {
   pwm_alpha -= 60;
 }
 int pwm_beta = map(beta, 0, 180, 122, 500);
 int pwm_gamma = map(gamma, 0, 180, 122, 500);
  pwm.setPWM(servo_pin[leg][0], 0, pwm_alpha);
  pwm.setPWM(servo_pin[leg][1], 0, pwm_beta);
  pwm.setPWM(servo_pin[leg][2], 0, pwm_gamma);

}
