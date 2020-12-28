#include <Servo.h>
#include <Wire.h>
#include <IRCam.h>
#include <NewPing.h>

IRCam irCam;
Servo esc;
Servo steering;



const unsigned long KILL_TIMER = 30000;

//US sensor assignment
#define SONAR_NUM     3 // Number of sensors.
#define MAX_DISTANCE 250 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 40 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define US_PIN1 2
#define US_PIN2 3
#define US_PIN3 4

//IR sensor assignment
//These sensors are active LOW and should be read with !digitalRead()
#define IR_PIN1 8     // ir sensor 1 (left most) on pin 5
#define IR_PIN2 9     // ir sensor 2 on pin 6
#define IR_PIN3 10    // ir sensor 3 (center) on pin 8
#define IR_PIN4 11    // ir sensor 4 on pin 7
#define IR_PIN5 12    // ir sensor 5 (right most) on pin 10

//servo pin assignments
#define STEERING_PIN 5  // steering servo pin
#define ESC_PIN 6       // esc pin

int center = 72;
int ir_ang = center;
int steer_ang = center;
int max_left = 140;
int left_max = max_left;
int max_right = 20;
int right_max = max_right;

int ir_ypos = 0;
int cur_speed = 1550;

unsigned long timer = 0;
unsigned long speed_change_time = 0;
unsigned long speed_delay = 250;

int ir_rec_array[6] = {0, 0, 0, 0, 0, 0};                //holds 0 or 1 based off of digital read of ir recievers
int last_rec_ir_ang = center;

unsigned long last_camera_time = 0;

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

unsigned long brake_timer = 0;
bool brake_complete = true;
unsigned brake_delay = 500;

int obstacle1 = 20;
int obstacle2 = 20;
int obstacle3 = 20;

//data smoothing
const int numReadings = 5;       //number of readings held for smoothing
int us1_readIndex = 0;
int us1_readings[numReadings];
int us1_total = 0;
int us2_readIndex = 0;
int us2_readings[numReadings];
int us2_total = 0;
int us3_readIndex = 0;
int us3_readings[numReadings];
int us3_total = 0;

int range_us1 = 100;                     // stores SMOOTHED range detected by us1 - should be target!
int last_range_us1 = 100;  
int range_us2 = 100;                     // stores SMOOTHED range detected by us1 - should be target!
int last_range_us2 = 100;  
int range_us3 = 100;                     // stores SMOOTHED range detected by us1 - should be target!
int last_range_us3 = 100;  

//ultrasonic sensors
NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(US_PIN1, US_PIN1, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(US_PIN2, US_PIN2, MAX_DISTANCE),
  NewPing(US_PIN3, US_PIN3, MAX_DISTANCE)
};

void setup() {
  // put your setup code here, to run once:

  esc.attach(ESC_PIN);
  steering.attach(STEERING_PIN);
  esc.writeMicroseconds(1500);
  steering.write(center);
  irCam.begin();                  // begin reading IR camera
  Serial.begin(9600);
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;


  for (int thisReading = 0; thisReading < numReadings; thisReading ++) { // start smoothing at zero
    us1_readings[thisReading] = 0;
    us2_readings[thisReading] = 0;
    us3_readings[thisReading] = 0;
  }
  Serial.println("Starting program...");
  esc.writeMicroseconds(cur_speed);
}

void loop() {
for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }


  //IR recievers read
  ir_rec_array[1] = !digitalRead(IR_PIN1);     // read ir 1
  ir_rec_array[2] = !digitalRead(IR_PIN2);     // read ir 2
  ir_rec_array[3] = !digitalRead(IR_PIN3);     // read ir 3
  ir_rec_array[4] = !digitalRead(IR_PIN4);     // read ir 4
  ir_rec_array[5] = !digitalRead(IR_PIN5);     // read ir 5

  //IR camera read
  irCam.update();  //required to update camera readigs

  //calculates ir angle from camera
  if (irCam.p1.x > 0 && irCam.p1.x <= 1022) {
    last_camera_time = millis();
    for (int i = 0; i <= 36; i++) {
      if (irCam.p1.x > i * (1000 / 36) && irCam.p1.x <= (i + 1) * (1000 / 36)) {
        //last_cam_ir_ang = ((center + (36 / 2)) - i);
        ir_ang = ((center + (36 / 2)) - i);
      }
    }
  }

  //ir angle from ir recievers
  if (ir_rec_array[1] == 1) {
    if (ir_rec_array[2] == 1) {
      if (ir_rec_array[3] == 1) {
        last_rec_ir_ang = center + 15;          //1,2,3
      }
      else {
        last_rec_ir_ang = left_max - 10;  //1,2
      }
    }
    else {
      last_rec_ir_ang = left_max;  //1
    }
  }
  else if (ir_rec_array[5] == 1) {
    if (ir_rec_array[4] == 1) {
      if (ir_rec_array[3] == 1) {
        last_rec_ir_ang = center - 15;          //5,4,3
      }
      else {
        last_rec_ir_ang = right_max + 10;  //5,4
      }
    }
    else {
      last_rec_ir_ang = right_max;  //5
    }
  }
  else if (ir_rec_array[3] == 1) {
    if (ir_rec_array[2] == 1) {
      if (ir_rec_array[4] == 1) {
        last_rec_ir_ang = center;               //3,2,4
      }
      else {
        last_rec_ir_ang = center + 15;  //3,2
      }
    }
    else if (ir_rec_array[4] == 1) {
      last_rec_ir_ang = center - 20;            //3,4
    }
    else {
      last_rec_ir_ang = center;  //3
    }
  }
  else if (ir_rec_array[2] == 1) {
    last_rec_ir_ang = center + 30;              //2
  }
  else if (ir_rec_array[4] == 1) {
    last_rec_ir_ang = center - 30;              //4
  }



  // put your main code here, to run repeatedly:
  if (brake_complete == true) {

    if (millis() - last_camera_time > 250) {    //using ir camera
      //this code turns to the left
      //the further the ir angle is from the currently set steering angle, the larger the turn increments are
      if (ir_ang > steer_ang) {
        if (ir_ang - steer_ang >= 30 && steer_ang < (max_left - 10)) {
          steer_ang = steer_ang + 10;
          steering.write(steer_ang);
        }
        else if (ir_ang - steer_ang >= 15 && steer_ang < (max_left - 5)) {
          steer_ang = steer_ang + 5;
          steering.write(steer_ang);
        }
        else if (ir_ang - steer_ang > 0 && steer_ang < (max_left - 1)) {
          steer_ang = steer_ang + 1;
          steering.write(steer_ang);
        }
      }

      //this code turns to the right
      //the further the ir angle is from the currently set steering angle, the larger the turn increments are
      if (ir_ang < steer_ang) {
        if (steer_ang - ir_ang >= 30 && steer_ang < (max_right + 10)) {
          steer_ang = steer_ang - 10;
          steering.write(steer_ang);
        }
        else if (steer_ang - ir_ang >= 15 && steer_ang < (max_right + 5)) {
          steer_ang = steer_ang - 5;
          steering.write(steer_ang);
        }
        else if (steer_ang - ir_ang > 0 && steer_ang < (max_right + 1)) {
          steer_ang = steer_ang - 1;
          steering.write(steer_ang);

        }
      }
    }
    else {  //if using ir recievers
      //this code turns to the left
      //the further the ir angle is from the currently set steering angle, the larger the turn increments are
      if (last_rec_ir_ang > steer_ang) {
        if (last_rec_ir_ang - steer_ang >= 30 && steer_ang < (max_left - 10 )) {
          steer_ang = steer_ang + 10;
          steering.write(steer_ang);
        }
        else if (last_rec_ir_ang - steer_ang >= 15 && steer_ang < (max_left - 5)) {
          steer_ang = steer_ang + 5;
          steering.write(steer_ang);
        }
        else if (last_rec_ir_ang - steer_ang > 0 && steer_ang < (max_left - 1)) {
          steer_ang = steer_ang + 1;
          steering.write(steer_ang);
        }
      }

      //this code turns to the right
      //the further the ir angle is from the currently set steering angle, the larger the turn increments are
      if (last_rec_ir_ang < steer_ang) {
        if (steer_ang - last_rec_ir_ang >= 30 && steer_ang > (max_right + 10)) {
          steer_ang = steer_ang - 10;
          steering.write(steer_ang);
        }
        else if (steer_ang - last_rec_ir_ang >= 15 && steer_ang > (max_right + 5)) {
          steer_ang = steer_ang - 5;
          steering.write(steer_ang);
        }
        else if (steer_ang - last_rec_ir_ang > 0 && steer_ang > (max_right + 1)) {
          steer_ang = steer_ang - 1;
          steering.write(steer_ang);

        }
      }
    }

  //speed + range control begin here...
  ir_ypos = irCam.p1.y;
  if (ir_ypos  > 1022) {
    ir_ypos = 0;
  }

    if (ir_ypos > 0 && millis() - speed_change_time >= speed_delay && brake_complete == true) {
      Serial.println("Speed loop started!");
      if (ir_ypos > 400) {                    //y > means trailbot is closer to target than desired
        if (cur_speed >= 1530) {
          cur_speed = cur_speed - 5;
          speed_change_time = millis();
        }
        esc.writeMicroseconds(cur_speed);
        Serial.print("IR Y coordinate is: ");
        Serial.print(ir_ypos);
        Serial.print(". Decreasing speed to ");
        Serial.print(cur_speed);
        Serial.println(" to open distance.");
      }
      else if (ir_ypos < 350  ) {               //y < means trailbot is farther from target than desired
        if (cur_speed <= 1600) {
          cur_speed = cur_speed + 1;
          speed_change_time = millis();
        }
        esc.writeMicroseconds(cur_speed);
        Serial.print("IR Y coordinate is: ");
        Serial.print(ir_ypos);
        Serial.print(". Increasing speed to ");
        Serial.print(cur_speed);
        Serial.println(" to close distance.");
      }
      else if (brake_complete == true) {
        esc.writeMicroseconds(cur_speed);
        Serial.println("IR Y coordinate within accepable bounds");
      }
    }
    else {
      esc.writeMicroseconds(cur_speed);
    }
    
  }
  else {
      esc.writeMicroseconds(1500);
      steering.write(center);
      delay(10);
    }
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  // The following code would be replaced with your code that does something with the ping results.
  //  for (uint8_t i = 0; i < SONAR_NUM; i++) {
  //    Serial.print(i);
  //    Serial.print("=");
  //    Serial.print(cm[i]);
  //    Serial.print("cm ");
  //  }
  //  Serial.println();

  last_range_us1 = cm[0];
  last_range_us2 = cm[1];
  last_range_us3 = cm[2];

  //US1 Smoothing
  if (last_range_us1 != 0) {
    us1_total = us1_total - us1_readings[us1_readIndex];
    us1_readings[us1_readIndex] = last_range_us1;
    us1_total = us1_total + us1_readings[us1_readIndex];
    us1_readIndex = us1_readIndex + 1;
    if (us1_readIndex >= numReadings) {
      us1_readIndex = 0;
    }
    range_us1 = us1_total / numReadings;
    //Serial.print("Smoothed range us1: ");
    //Serial.println(range_us1);
  }
  //US2 Smoothing
  if (last_range_us2 != 0) {
    us2_total = us2_total - us2_readings[us1_readIndex];
    us2_readings[us2_readIndex] = last_range_us2;
    us2_total = us2_total + us2_readings[us2_readIndex];
    us2_readIndex = us2_readIndex + 1;
    if (us2_readIndex >= numReadings) {
      us2_readIndex = 0;
    }
    range_us2 = us2_total / numReadings;
    //Serial.print("Smoothed range us2: ");
    //Serial.println(range_us2);
  }
  //US3 Smoothing
  if (last_range_us3 != 0) {
    us3_total = us3_total - us3_readings[us3_readIndex];
    us3_readings[us3_readIndex] = last_range_us3;
    us3_total = us3_total + us3_readings[us3_readIndex];
    us3_readIndex = us3_readIndex + 1;
    if (us3_readIndex >= numReadings) {
      us3_readIndex = 0;
    }
    range_us3 = us3_total / numReadings;
    //Serial.print("Smoothed rangevus3: ");
    //Serial.println(range_us3);
  }

  //Serial.print("US1: ");
  //Serial.print(range_us1);
  //Serial.print(", US2: ");
  //Serial.print(range_us2);
  //Serial.print(", US3: ");
  //Serial.println(range_us3);

  if (millis() - brake_timer > brake_delay) {
    brake_complete = true;
  }

  if (range_us1 > 0 && range_us1 < obstacle1) {
    Serial.print("Obstacle center at ");
    Serial.println(range_us1);
    esc.writeMicroseconds(1501);
    esc.writeMicroseconds(1000);
    brake_complete = false;
  }
  else if (range_us2 > 0 && range_us2 < obstacle2) {
    Serial.print("Obstacle left at ");
    Serial.println(range_us2);
    esc.writeMicroseconds(1501);
    esc.writeMicroseconds(1000);
    brake_complete = false;
  }
  else if (range_us3 > 0 && range_us3 < obstacle3) {
    Serial.print("Obstacle on right at ");
    Serial.println(range_us3);
    esc.writeMicroseconds(1501);
    esc.writeMicroseconds(1000);
    brake_complete - false;
  }


}
