//IR sensor assignment
//These sensors are active LOW and should be read with !digitalRead()
#define ir_pin1 8     //ir sensor 1 (left most) on pin 5
#define ir_pin2 9     // ir sensor 2 on pin 6
#define ir_pin3 10    // ir sensor 3 (center) on pin 8
#define ir_pin4 11    // ir sensor 4 on pin 7
#define ir_pin5 12    // ir sensor 5 (right most) on pin 10

//steering variables
int left_max = 115;  //maximum left turn wheel position
int left = 87;      //standard left turn
int right_max = 40;  //maximum right turn wheel position
int right = 68;     //standard right turn
int center = 78;    //approximate center wheel position
int steer_ang = 78;   //holds current angle

//define IR angle of each IR reciever based on range of steering variables
int ir1_ang = left_max;
int ir2_ang = ((left_max - center) / 2) + center;
int ir3_ang = center;
int ir4_ang = center - ((center - right_max) / 2);
int ir5_ang = right_max;
int ir_rec_array[6] = {0, 0, 0, 0, 0, 0};                               //holds 0 or 1 based off of digital read of ir recievers
int ir_ang_array[6] = {0, ir1_ang, ir2_ang, ir3_ang, ir4_ang, ir4_ang}; //array holding IR angle of each reciever


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Booting up...");
}

void loop() {
  // put your main code here, to run repeatedly:
    //IR recievers read
    ir_rec_array[1] = 0;
    ir_rec_array[2] = 0;
    ir_rec_array[3] = 0;
    ir_rec_array[4] = 0;
    ir_rec_array[5] = 0;
    ir_rec_array[1] = !digitalRead(ir_pin1);     // read ir 1
    ir_rec_array[2] = !digitalRead(ir_pin2);     // read ir 2
    ir_rec_array[3] = !digitalRead(ir_pin3);     // read ir 3
    ir_rec_array[4] = !digitalRead(ir_pin4);     // read ir 4
    ir_rec_array[5] = !digitalRead(ir_pin5);     // read ir 5
    if (ir_rec_array[1] == 1 || ir_rec_array[2] == 1 || ir_rec_array[3] == 1 || ir_rec_array[4] == 1 || ir_rec_array[5] == 1) {
    for (int i = 1; i <= 5; i++) {
      Serial.print(ir_rec_array[i]);
      Serial.print(", ");
    }
    Serial.println();
    delay(200);
    }
}
