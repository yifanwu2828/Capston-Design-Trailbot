//Trailbot steering and speed code
/*
    November 7: Begin recording code changes; see hitchcock_2 for prior code
    8:  Successfully implement and test multiple US reads
        Added data smoothing for US and IR
    9:  Rewrote US range / speed control
        Wrote second option for calculating IR reciever angle
        Added KILL_TIMER for testing safety
        Cleaned up / organized current code
    15: added speed control via ir camera y position

    Future?
      Implement camera ranging
*/

//Note on millis() function
/*
   The millis() function is used to take the number of ms passed since arduino started up
   Using this function allows for time to be kept without relying on delay()
   avoid delay() in normal code! (*may* work in special code outside of normal loop..but don't expect good us data)
*/

//Note on setting speed
/*
   At any time, setting speed above 1500 will cause forward motion.
   If moving forward, setting speed below 1500 will cause braking.
   To travel in reverse direction from forward, double action is required:
   The speed must be moved to brake (speed <= 1500), then to neutral, and then again to reverse (speed <= 1500)
   Brake time must be sufficient to allow vehicle to come to a stop before proceding to reverse.
      -100ms at 1550 forward speed without resistance
   Neutral must be held a minimum of 50ms before switching to reverse.
*/

#include <Servo.h>
#include <NewPing.h>
//NewPing library can be found here: https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
//help with using newping event here:
//https://bitbucket.org/teckel12/arduino-new-ping/wiki/Help%20with%2015%20Sensors%20Example%20Sketch
#include <Wire.h>
#include <IRCam.h>
//IRCam library can be found here: https://github.com/kurakuradave/IRCam

//US sensor assignment
#define SONAR_NUM     3 // Number of sensors.
#define MAX_DISTANCE 250 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 40 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define US_PIN1 2
#define US_PIN2 3
#define US_PIN3 4

//IR sensor assignment
//These sensors are active LOW and should be read with !digitalRead()
#define IR_PIN1 8     //ir sensor 1 (left most) on pin 5
#define IR_PIN2 9     // ir sensor 2 on pin 6
#define IR_PIN3 10    // ir sensor 3 (center) on pin 8
#define IR_PIN4 11    // ir sensor 4 on pin 7
#define IR_PIN5 12    // ir sensor 5 (right most) on pin 10

//servo pin assignments
#define STEERING_PIN 5  // steering servo pin
#define ESC_PIN 6       // esc pin

//time variables
unsigned long KILL_TIMER = 90000;   //timer KILLS all action in program !!ESSENTIAL TESTING SAFETY FEATURE!!
unsigned long steer_delay = 20;    // allows times to pass after steering change
unsigned long speed_delay = 20;    // allows times to pass after speed change
unsigned long steer_time = 0;       // records time of last steering change
unsigned long speed_time = 0;       // records time of last speed change
unsigned long no_detect_message_time = 0; //used to avoid excessive messages when camera doesn't detect
unsigned long no_detect_message_delay = 5000; //time between no detect messages
unsigned long lost_signal_delay = 5000;   //time after which IR signal is considered lost
unsigned long last_ir_time = 0;     //records time at which ir was last detected
unsigned long lost_signal_shutdown_delay = 60000;   //length of time to search for a lost ir signal
unsigned long startup_delay = 10000;      //prevents ir search to allow time for user to turn on beacon
unsigned long last_cam_time = 0;          //records time ir camera last picked up signal
unsigned long cam_delay = 200;            //time after last ir camera signal to switch to recievers

//variables used for three US sensors - all in cm
unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.
int us_avoidance_distance = 50;     // range below which obstacle is considered detected
int target_range = 150;             // range that trailbot tries to optimize for
int range_range = 20;               // how far above or below target_range range must go for acceleration
int us2_range = 50;                 // stores range detected by us2
int us3_range = 50;                 // stores range detected by us2
int range = 50;                     // stores SMOOTHED range detected by us1 - should be target!
int last_range = 50;                // most recent range from US1

//data smoothing
const int numReadings = 10;       //number of readings held for smoothing
int ir_readIndex = 0;             //tracks location within readings array
int us1_readIndex = 0;
int ir_readings[numReadings];     //stores readings
int us1_readings[numReadings];
int ir_total = 0;                 //stores running total of past numReadings
int us1_total = 0;

//steering variables
int left_max = 115;  //maximum left turn wheel position
int left = 87;      //standard left turn
int right_max = 40;  //maximum right turn wheel position
int right = 68;     //standard right turn
int center = 78;    //approximate center wheel position
int steer_ang = center;   //holds current angle

//speed variables
int max_speed = 1700.;  //max forward speed setting (technical limit is 2000, but limited for safety / testing)
int forward = 1600;     //starting speed for forward
int neutral = 1500;     //neutral
int reverse = 1400;     //starting speed for reverse
int max_reverse = 1300; //maximum set reverse speed (technical limit is 1000, but limited to 1300 for safety)
int brake = 1400;       //brake speed (can be set from 1450-1000, but using one speed for concistency)
int cur_speed = 1500;   //records current speed

//state tracking booleans
bool obs[SONAR_NUM + 1] = {false, false, false, false};                 //tracks obstacle in front of us sensors
bool obstacle = false;              //tracks if any obstacle is detected
bool steer_complete = false;        //used to control steering loops
bool speed_complete = false;        //used to control speed loops
bool lost_signal = false;           //tracks if ir signal is lost
bool lost_signal_shutdown = false;  //tracks if lost signal search has exceeded maximum time
bool obstacle_brake = false;        //tracks if braking has happened after obstacle was detected

//ir angle storage
int ir_ang = center;
int last_cam_ir_ang = center;
int last_rec_ir_ang = center;
int ir_ypos = 0;                //ir camera y coordinate

//define IR angle of each IR reciever based on range of steering variables
int ir1_ang = left_max;
int ir2_ang = left_max - 10;
int ir3_ang = center;
int ir4_ang = right_max + 10;
int ir5_ang = right_max;
int ir_rec_array[6] = {0, 0, 0, 0, 0, 0};                //holds 0 or 1 based off of digital read of ir recievers
int ir_ang_array[6] = {0, ir1_ang, ir2_ang, ir3_ang, ir4_ang, ir4_ang}; //array holding IR angle of each reciever

//servo
Servo steering;        // create servo object to control steering
Servo esc;             //create servo object to control esc - marked with black electrical tape

//ultrasonic sensors
NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(US_PIN1, US_PIN1, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(US_PIN2, US_PIN2, MAX_DISTANCE),
  NewPing(US_PIN3, US_PIN3, MAX_DISTANCE)
};

//IR Positioning Camera
//NOTE: y-axis is inverted!
//readings of 1023 or higher indicate no detection
IRCam irCam;
//irCam.p1.x to get x result
//irCam.p1.y to get y result

void setup() {
  Serial.begin(9600);               //attach serial
  Serial.println("Booting up...");
  steering.attach(STEERING_PIN);     // attaches steering to pin 9
  esc.attach(ESC_PIN);            // attaches esc to pin 11
  irCam.begin();                  // begin reading IR camera
  esc.writeMicroseconds(neutral); // starts motor at neutral
  pingTimer[0] = millis() + 200;  // First ping starts at 200ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++)             // Set the starting time for each US sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

  for (int thisReading = 0; thisReading < numReadings; thisReading ++) { // start smoothing at zero
    ir_readings[thisReading] = 0;
    us1_readings[thisReading] = 0;
  }
  Serial.println("Setup complete...");
  Serial.println("Sweep test...");

/*
  //sweep steering as test  --  only useful with wheels off
  for (steer_ang = center; steer_ang <= left_max; steer_ang += 1) { // goes from center to left_max
    // in steps of 1 degree
    steering.write(steer_ang);              // tell servo to go to position in variable 'pos'
    delay(15);                              // waits 15ms for the servo to reach the position
  }
  for (steer_ang = left_max; steer_ang >= center; steer_ang -= 1) { // goes from left_max to center
    steering.write(steer_ang);              // tell servo to go to position in variable 'pos'
    delay(15);                              // waits 15ms for the servo to reach the position
  }
  steer_ang = center;
  steering.write(steer_ang);
  */
  Serial.println("Begining program...");
}

void loop() {

if (millis() <= KILL_TIMER) {       //progam only runs if time passed is less than KILL_TIMER
  
  //US update
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete.
      sonar[currentSensor].timer_stop();          // Cancel old ping before new one.
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Ping! (processing continues, interrupt calls echoCheck)
    }
  }

  //US1 Smoothing
  if (last_range != 0) {
    us1_total = us1_total - us1_readings[us1_readIndex];
    us1_readings[us1_readIndex] = last_range;
    us1_total = us1_total + us1_readings[us1_readIndex];
    us1_readIndex = us1_readIndex + 1;
    if (us1_readIndex >= numReadings) {
      us1_readIndex = 0;
    }
    range = us1_total / numReadings;
    //Serial.print("Smoothed range: ");
    //Serial.println(range);
  }

  //check each us for obstacle
  if (millis() > startup_delay) {
    //set obstacle flag based on US results...
    if (range > 0 && range < us_avoidance_distance) {
      obs[1] = true;
      Serial.print("Obstacle 1: ");
      Serial.println(range);
    }
    else {
      obs[1] = false;
    }
    if (us2_range > 0 && us2_range < us_avoidance_distance) {
      obs[2] = true;
      Serial.print("Obstacle 2: ");
      Serial.println(us2_range);
    }
    else {
      obs[2] = false;
    }
    if (us3_range > 0 && us3_range < us_avoidance_distance) {
      obs[3] = true;
      Serial.print("Obstacle 3: ");
      Serial.print(us3_range);
    }
    else {
      obs[3] = false;
    }

    //checks overall status of obstacles
    if (obs[1] != true && obs[2] != true && obs[3] != true) {
      if (obstacle == true) {
        Serial.println("Obstacle cleared!");
        steering.write(center);
        esc.write(1525);
      }
      obstacle = false;
      obstacle_brake = false;
    }
    else if (obs[1] == true || obs[2] == true || obs[3] == true ) {
      if (obstacle == false) {
        obstacle = true;
        Serial.println("Obstacle detected...");
      }
    }
  }

  //manuevers to avoid obstacle if found
  if (obstacle == true) {
    if (obstacle_brake == false) {    //only brake if vehicle hasn't already stopped for this obstacle
      cur_speed = brake;
      esc.writeMicroseconds(cur_speed); //immediately brake
      delay(100);
      cur_speed = neutral;
      esc.writeMicroseconds(cur_speed); //switch to neutral
      delay(100);
      steering.write(center);
      obstacle_brake = true;
    }

    //without any further instructions below, the vehicle will stay idle until the obstacle is cleared
    if (obs[1] == true) {
      cur_speed = brake;
      if (obs[2] == true) {
        if (obs[3] == true) {
          //procedure for when all three US sensors detect an obstacle
        }
        else {
          //procedure for when us1 and us2 detect an obstacle
        }
      }
      //procedure for when only us1 detects an obstacle
    }

    else if (obs[2] == true) {
      if (obs[3] == true) {
        //procedure for when us2 and us 3 detect an obstacle
      }
      else {
        //procedure for when only us2 detects an obstacle
      }
    }

    else if (obs[3] == true) {
      //procedure when only us3 detects an obstacle
      /*
        steer_ang = right_max;
        steering.write(steer_ang);
        cur_speed = reverse;
        esc.writeMicroseconds(cur_speed);
        steer_ang = center;
        steering.write(steer_ang);
        cur_speed = 1550;
        esc.writeMicroseconds(cur_speed);
        for (int i = 0; i < center - right_max; i++) {
        steer_ang = steer_ang - i;
        steering.write(steer_ang);
        }
      */
    }
  }


  //if no obstacle was detected, read IR and manuever normally
  else if (obstacle == false) {

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
      for (int i = 0; i <= 36; i++) {
        if (irCam.p1.x > i * (1000 / 36) && irCam.p1.x <= (i + 1) * (1000 / 36)) {
          last_cam_ir_ang = ((center + (36 / 2)) - i);
          lost_signal = false;
          lost_signal_shutdown = false;
          Serial.print("Current angle detected by the camera is: ");
          Serial.println(last_cam_ir_ang);
          last_ir_time = millis();
          last_cam_time = millis();
        }
      }
    }

    //two ways to calculate angle from recievers

    //checks which ir recievers are high and averages their respective angles; simple
    /*
      else if (ir_rec_array[1] > 0 || ir_rec_array[2]  > 0 || ir_rec_array[3]  > 0 
      || ir_rec_array[4]  > 0 || ir_rec_array[5]  > 0) {
      int sum = 0;
      int count = 0;
      for (int i = 1; i <= 5; i++) {
        if (ir_rec_array[i] == 1) {
          sum = sum + ir_ang_array[i];
          count++;
        }
      }
      last_rec_ir_ang = sum / count;
      lost_signal = false;
      lost_signal_shutdown = false;
      last_ir_time = millis();
      Serial.print("Current angle detected by recievers: ");
      Serial.println(last_rec_ir_ang);
      }
    */
    //if statements to calculate ir angle from recievers;  more customization and detailed control of behavior
    if (ir_rec_array[1] == 1) {
      if (ir_rec_array[2] == 1) {
        if (ir_rec_array[3] == 1) {
          last_rec_ir_ang = center + 20;          //1,2,3
        }
        else {
          last_rec_ir_ang = center + 30;  //1,2
        }
      }
      else {
        last_rec_ir_ang = left_max;  //1
      }
    }
    else if (ir_rec_array[5] == 1) {
      if (ir_rec_array[4] == 1) {
        if (ir_rec_array[3] == 1) {
          last_rec_ir_ang = center - 20;          //5,4,3
        }
        else {
          last_rec_ir_ang = center - 30;  //5,4
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
          last_rec_ir_ang = center + 10;  //3,2
        }
      }
      else if (ir_rec_array[4] == 1) {
        last_rec_ir_ang = center - 10;            //3,4
      }
      else {
        last_rec_ir_ang = center;  //3
      }
    }
    else if (ir_rec_array[2] == 1) {
      last_rec_ir_ang = center + 15;              //2
    }
    else if (ir_rec_array[4] == 1) {
      last_rec_ir_ang = center - 15;              //4
    }

    //records ir detection from ir recievers
    if (ir_rec_array[1] == 1 || ir_rec_array[2] == 1 || ir_rec_array[3] == 2 
    || ir_rec_array[4] == 1 || ir_rec_array[5] == 1) {
      lost_signal = false;
      lost_signal_shutdown = false;
      last_ir_time = millis();
    }

    //if last_ir_time is greater than lost_signal_delay in the past, the lost_signal flag is set to begin search
    else if (millis() - last_ir_time > lost_signal_delay && millis() > startup_delay ) {
      lost_signal = true;
    }

    //smooth ir_ang
    if (millis() - last_cam_time <= cam_delay) {        //use data from IR cam if it is recent
      ir_total = ir_total - ir_readings[ir_readIndex];
      ir_readings[ir_readIndex] = last_cam_ir_ang;
      ir_total = ir_total + ir_readings[ir_readIndex];
      ir_readIndex = ir_readIndex + 1;
      if (ir_readIndex >= numReadings) {
        ir_readIndex = 0;
      }
      ir_ang = ir_total / numReadings;
    }
    else if (millis() - last_cam_time > cam_delay) {    //use data from IR receviers otherwise
      ir_total = ir_total - ir_readings[ir_readIndex];
      ir_readings[ir_readIndex] = last_rec_ir_ang;
      ir_total = ir_total + ir_readings[ir_readIndex];
      ir_readIndex = ir_readIndex + 1;
      if (ir_readIndex >= numReadings) {
        ir_readIndex = 0;
      }
      ir_ang = ir_total / numReadings;
    }

    //steer vehicle toward IR signal

    //steer and speed delay passed?
    //steer_complete provides a minimum time between steering actions to avoid eratic or excessively fast turns
    if (millis() - steer_time > steer_delay) {
      steer_complete = true;
    }
    //spped_complete allows for minimum time between speed changes
    if (millis() - speed_time > speed_delay) {
      speed_complete = true;
    }

    //actual code for turning the vehicle! should be replace with PID?
    //this code turns to the left
    //the further the ir angle is from the currently set steering angle, the larger the turn increments are
    if (ir_ang > steer_ang && lost_signal == false) {
      if (ir_ang - steer_ang > 20 && steer_complete == true) {
        steer_ang = steer_ang + 5;
        steering.write(steer_ang);
        steer_time = millis();
        steer_complete = false;
      }
      else if (ir_ang - steer_ang > 10 && steer_complete == true) {
        steer_ang = steer_ang + 2;
        steering.write(steer_ang);
        steer_time = millis();
        steer_complete = false;
      }
      else if (ir_ang - steer_ang > 0 && steer_complete == true) {
        steer_ang = steer_ang + 1;
        steering.write(steer_ang);
        steer_time = millis();
        steer_complete = false;
      }
    }

    //this code turns to the right
    //the further the ir angle is from the currently set steering angle, the larger the turn increments are
    if (ir_ang < steer_ang && lost_signal == false) {
      if (steer_ang - ir_ang > 30 && steer_complete == true) {
        steer_ang = steer_ang - 5;
        steering.write(steer_ang);
        steer_time = millis();
        steer_complete = false;
      }
      else if (steer_ang - ir_ang > 15 && steer_complete == true) {
        steer_ang = steer_ang - 2;
        steering.write(steer_ang);
        steer_time = millis();
        steer_complete = false;
      }
      else if (steer_ang - ir_ang > 0 && steer_complete == true) {
        steer_ang = steer_ang - 1;
        steering.write(steer_ang);
        steer_time = millis();
        steer_complete = false;
      }
    }

    //if the current steering angle and ir angle are identical, no turn is made
    //but the steer_compelete flag is reset
    else if (ir_ang == steer_ang && steer_complete == true ) {
      steer_time = millis();
      steer_complete = false;
    }

    //this is the current procedure for a loss of ir signal
    //by setting the lost_signal_delay time you can control how long the program waits to begin acting on lost signal
    //this is important to avoid entering lost signal mode for a flickering signal (such as an ir remote)
    //when this is entered, turn is increased to maximum left and the robot continues that circle until IR is reaquired
    else if (lost_signal == true && lost_signal_shutdown == false) {

      if (millis() - no_detect_message_time >= no_detect_message_delay) {   //prevents excessive printing
        Serial.print("No IR signal detected in ");
        Serial.print(millis() - last_ir_time);
        Serial.println(" milliseconds.");
        Serial.println("Circling left to reaquire IR...");
        no_detect_message_time = millis();
      }
      for (int i = 0; i <= left_max - steer_ang; i++) {   //steers left to loop for IR signal
        if (steer_ang < left_max) {                       //left is arbitary, could be changed to right
          steer_ang = steer_ang + 1;
          steering.write(steer_ang);
          cur_speed = 1550;
          esc.writeMicroseconds(cur_speed);
        }
        else if (steer_ang == left_max) {
          //Serial.println("Max left turn reached...");
          break;                                          //exits  loop once max turn is reached
        }
      }

      if (millis() - last_ir_time > lost_signal_shutdown_delay) {   //after delay, ends IR search
        Serial.print("No IR signal detected in ");
        Serial.print(millis() - last_ir_time);
        Serial.println(" milliseconds.");
        Serial.println("Ending reaquisition maneuver...");
        Serial.println("Idling...");
        cur_speed = neutral;
        esc.writeMicroseconds(cur_speed);
        steering.write(center);
        lost_signal_shutdown = true;
      }
    }

    //speed + range control begin here...
    if (irCam.p1.y > 0 && irCam.p1.y <= 1022) {     //checks to see if ir camera has valid data
      ir_ypos = irCam.p1.y;
    }
    else {
      ir_ypos = 0;
    }
    //remember ir camera y positions are inverted
    if (ir_ypos > 0) {
      if (ir_ypos > 550) {                    //y > 550 means trailbot is farther from target than centered
        cur_speed = cur_speed + 1;
        esc.writeMicroseconds(cur_speed);
        Serial.print("IR Y coordinate is: ");
        Serial.print(ir_ypos);
        Serial.println(". Increasing speed to close distance.");
        speed_time = millis();
        speed_complete = false;
      }
      else if (ir_ypos < 450) {               //y > 550 means trailbot is closer to target than centered
        cur_speed = cur_speed - 1;
        esc.writeMicroseconds(cur_speed);
        Serial.print("IR Y coordinate is: ");
        Serial.print(ir_ypos);
        Serial.println(". Decreasing speed to close distance.");
        speed_time = millis();
        speed_complete = false;
      }
      else {
        esc.writeMicroseconds(cur_speed);
        Serial.println("IR Y coordinate within accepable bounds");
        speed_time = millis();
        speed_complete = false;
      }
    }
    else if (ir_ypos = 0) {
    if (ir_ang > 85 && ir_ang < 71 && speed_complete == true) {       //maintains speed during turns
      cur_speed = cur_speed;
      esc.writeMicroseconds(forward);
      Serial.print("Range: ");
      Serial.print(range);
      Serial.println("Maintain current speed during turn...");
      speed_time = millis();
      speed_complete = false;
    }
    else if (range >= target_range + range_range && speed_complete == true) {  
      //decrease speed to reach target_range area                     //incrementally decreases speed when target
      if (cur_speed > forward) {                                      //is futher than desired range
        cur_speed = cur_speed - 1;
        esc.writeMicroseconds(cur_speed);
        Serial.print("Range: ");
        Serial.print(range);
        Serial.println(". Decrease speed to match target...");
      }
      else { Serial.println("Minimum speed reached..."); }
      speed_time = millis();
      speed_complete = false;
    }
    else if (range <= target_range - range_range && speed_complete == true) {  
      //increase speed to reach target_range area                     //incrementally increases speed when target
      if (cur_speed < max_speed) {                                    //is closer than desire range
        cur_speed = cur_speed + 1;
        esc.writeMicroseconds(cur_speed);
        Serial.print("Range: ");
        Serial.print(range);
        Serial.println(". Increase speed to match target...");
      }
      else { Serial.println("Maximum speed reached..."); }
      speed_time = millis();
      speed_complete = false;
    }
    }

  }   // ends else if (obstacle == false)

}
else {
  esc.writeMicroseconds(neutral);
  steering.write(center);
  Serial.println("Program ended due to KILL_TIMER for safety");
  delay(5000);
}
  
}   // ends main loop


void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    last_range = cm[0];
    us2_range = cm[1];
    us3_range = cm[2];
  }
}
