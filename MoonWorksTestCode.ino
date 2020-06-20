#include <SoftwareSerial.h>

 #include <Servo.h>

//Pin allocation
#define battery     A0
#define BTstat      2
#define buzzer      3

//Motors
#define LMspeed     5
#define RMspeed     6
#define LMrev       9
#define LMfwd       10
#define RMrev       11
#define RMfwd       12

//Servos
#define arm1        A1
#define arm2        A2
#define arm3        A3
#define arm4        A4
 
Servo control1;
Servo control2;
Servo control3;
Servo control4;

//Servo position memory
int control1_mem = 1350;
int control2_mem = 1830;
int control3_mem = 1350;
int control4_mem = 0;

//Extreme values to prevent servo crushing itself
int servomax = 2000;
int servomin = 700;

//Stores speed information of the wheels
int LM_speed_mem; 
int RM_speed_mem;
int LM_steer_mem;
int RM_steer_mem;
int LM_total_mem;
int RM_total_mem;

int battery_voltage[60];    //Stores the voltage measurements to obtain a nice average
int battery_counter = 0;    //Counts how many measurements have been taken

//Generic Timers
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
unsigned long previousMillis1 = 0;

char data_byte[6]; //Stores recieved data

//MODE0 [ MODE | MOTORSPEED | MOTORDIRECTION | STEERINGPOSITION | STEERINGDIRECTION | SPEEDSELECT ]
//MODE1 [ MODE |  CONTROL1  |    CONTROL2    |     CONTROL3     |      CONTROL4     |    BLANK    ]

SoftwareSerial BLUETOOTH(3,4);

//Initialisation
void setup(){ 
  control1.attach(arm1);  //Lower arm servo
  control1.writeMicroseconds(control1_mem);
  
  control2.attach(arm2);  //Upper arm servo
  control2.writeMicroseconds(control2_mem);
  
  control3.attach(arm3);  //Bucket servo
  control3.writeMicroseconds(control3_mem);
  
  control4.attach(arm4);  //Door servo
  control4.write(control4_mem);

  //HBridge control
  pinMode(LMfwd,OUTPUT); //IN1
  pinMode(LMrev,OUTPUT); //IN2
  pinMode(RMfwd,OUTPUT); //IN3
  pinMode(RMrev,OUTPUT); //IN4
  pinMode(LMspeed,OUTPUT); //EN1
  pinMode(RMspeed,OUTPUT); //EN2

  //Motors defaults to forward direction
  LMdir(true); 
  RMdir(true);

  pinMode(battery,INPUT); //Battery voltage
  pinMode(buzzer,OUTPUT); //Buzzer
  pinMode(BTstat,INPUT);  //Connection Status
  Serial.begin(9600);
  BLUETOOTH.begin(9600);
  BLUETOOTH.listen();
  BLUETOOTH.println("CONNECT DEVICE");}

//Connected notification
void con(){
  Serial.println("CONNECTED"); 
  char disc_tune[3] = {8,6,10}; //Connected tune
  for(int i=0 ; i<3 ; i++){
    tone(buzzer,disc_tune[i]*100); 
    delay(100);
    noTone(buzzer);}}

//Disconnected notification
void disc(){ 
  Serial.println("DISCONNECTED"); 
  char disc_tune[5] = {10,8,6,4,2}; //Disconnected tune
  for(int i=0 ; i<5 ; i++){
    tone(buzzer,disc_tune[i]*100); 
    delay(100);
    noTone(buzzer);}}

//Siren sound effect
void siren(){
  for(int t=0; t<4 ; t++){
    for(int i=1000 ; i<2000 ; i++){
      tone(buzzer,i);}
    for(int i=1000 ; i>1000 ; i--){
      tone(buzzer,i);}}
    noTone(buzzer);}

//Lower arm servo
void control1_move(){
  if(data_byte[1]-'0' >= 5 && control1_mem<=servomax+80)
  {
   control1_mem += map(data_byte[1]-'0',5,8,1,10);
   control1.writeMicroseconds(control1_mem);
   if(control2_mem < servomax+80){
    control2_mem += map(data_byte[1]-'0',5,8,1,10);
    control2.writeMicroseconds(control2_mem);
   }
  }
  if(data_byte[1]-'0' <= 3 && control1_mem>=servomin)
  {
   control1_mem -= map(data_byte[1]-'0',3,0,1,10);
   control1.writeMicroseconds(control1_mem);
   if(control2_mem > servomin-80){
    control2_mem -= map(data_byte[1]-'0',3,0,1,10);
    control2.writeMicroseconds(control2_mem);
   }
  }
}

//Upper arm servo
void control2_move(){
  if(data_byte[2]-'0' >= 5 && control2_mem>=servomin-80)
  {
   control2_mem -= map(data_byte[2]-'0',5,8,1,10);
   control2.writeMicroseconds(control2_mem);
  }
  if(data_byte[2]-'0' <= 3 && control2_mem<=servomax)
  {
   control2_mem += map(data_byte[2]-'0',3,0,1,10);
   control2.writeMicroseconds(control2_mem);
  }
}

//Bucket servo
void control3_move(){
  if(data_byte[3]-'0' >= 5 && control3_mem<=servomax)
  {
   control3_mem += map(data_byte[3]-'0',5,8,1,10);
   control3.writeMicroseconds(control3_mem);
  }
  if(data_byte[3]-'0' <= 3 && control3_mem>=servomin)
  {
   control3_mem -= map(data_byte[3]-'0',3,0,1,10);
   control3.writeMicroseconds(control3_mem);
  }
}

//Door servo
void control4_move(){
  if(data_byte[4]-'0' == 1 && control4_mem == 0)
  {
   control4_mem = 180;
   control4.write(control4_mem);
  }
  if(data_byte[4]-'0' == 0 && control4_mem == 180)
  {
   control4_mem = 0;
   control4.write(control4_mem);
  }
}
 
//Collects and displays revceived data
void data_recieved(){
      if(BLUETOOTH.available()>0) 
        {
          BLUETOOTH.readBytes(data_byte,6); //Reads 6-bit data chunks from HC-05
          for(int i=0 ; i<6 ; i++){ //Prints incoming data
            Serial.print(data_byte[i]);}
          Serial.println("");
          }}

//Selects left wheel direction
void LMdir(boolean direction){
  if(direction) //Forward
  {
    digitalWrite(LMfwd,HIGH);
    digitalWrite(LMrev,LOW);
  }
  else if(!direction) //Reverse
  {
    digitalWrite(LMfwd,LOW);
    digitalWrite(LMrev,HIGH);
  }
}

//Selects right wheel direction
void RMdir(boolean direction){
  if(direction) //Forward
  {
    digitalWrite(RMfwd,HIGH);
    digitalWrite(RMrev,LOW);
  }
  else if(!direction) //Reverse
  {
    digitalWrite(RMfwd,LOW);
    digitalWrite(RMrev,HIGH);
  }
}

//Converts throttle for motor speed
void getSpeed(){
  if(data_byte[5] == '1')
    {
      LM_speed_mem = map(data_byte[1]-'0',0,9,50,255);
      RM_speed_mem = map(data_byte[1]-'0',0,9,50,255);
    }
  else
    {
      LM_speed_mem = map(data_byte[1]-'0',0,9,40,130);
      RM_speed_mem = map(data_byte[1]-'0',0,9,40,130);
    }}

//Converts steering to motor speed
void getSteer(char dir){
  if(dir == 'L')
  {
    //High speed mode
    if(data_byte[5] == '1') {LM_steer_mem = map(data_byte[3]-'0',0,9,0,220);}
    //Low speed mode  
    else {LM_steer_mem = map(data_byte[3]-'0',0,9,0,150);}  
    RM_steer_mem = 0;
  }
  else if(dir == 'R')
  {
    //High speed mode
    if(data_byte[5] == '1') {RM_steer_mem = map(data_byte[3]-'0',0,9,0,220);}
    //Low speed mode    
    else {RM_steer_mem = map(data_byte[3]-'0',0,9,0,150);}        
    LM_steer_mem = 0;
  }}

//Car control
void carmode(){

  //Motor speed & direction
  switch(data_byte[2])
  {
    case '2': //Forward
    {
      getSpeed();
      LMdir(true);
      RMdir(true);
      break;
    }
    case '0': //Reverse
    {
      getSpeed();
      LMdir(false);
      RMdir(false);
      break;
    }
    default: //Off
    {
      LM_speed_mem = 0;
      RM_speed_mem = 0;
      break;
    }
  }
  
  //Steering direction   
  switch(data_byte[4])
  {
    case '2': //Right
    {
      getSteer('R');
      break;
    }
    case '0': //Left
    {
      getSteer('L');
      break;
    }
    default: //Centre
    {
      LM_steer_mem = 0;
      RM_steer_mem = 0;
      break;
    }
  }

  //Finds final speed for each side taking steering into account
  LM_total_mem = LM_speed_mem - LM_steer_mem + RM_steer_mem;
  RM_total_mem = RM_speed_mem - RM_steer_mem + LM_steer_mem;

  //Limits lowest speed
  if(LM_speed_mem + RM_steer_mem < LM_steer_mem) LM_total_mem = 0;  
  if(RM_speed_mem + LM_steer_mem < RM_steer_mem) RM_total_mem = 0;  
  
  //Limits highest speed
  if(LM_total_mem >= 255) LM_total_mem = 255;
  if(RM_total_mem >= 255) RM_total_mem = 255;

  //Spins on axis if no throttle input
  if(data_byte[1]-'0' == 0){
    if(LM_steer_mem > 0)
    {
      LMdir(false);
      RMdir(true);
      LM_total_mem = LM_steer_mem;
      RM_total_mem = LM_steer_mem;  
    }
    if(RM_steer_mem > 0)
    {
      LMdir(true);
      RMdir(false);
      RM_total_mem = RM_steer_mem;
      LM_total_mem = RM_steer_mem; 
    }}
  
  //Sets motor speed
  analogWrite(LMspeed,LM_total_mem);
  analogWrite(RMspeed,RM_total_mem);
}

//Arm control
void armmode(){
  control1_move();
  control2_move();
  control3_move();
  control4_move();
}

//Main program loop
void loop(){ 
  if(PIND & B00000100) //When bluetooth connection is successful
  {
    con();
    while(PIND & B00000100) //While connected
    { 
      currentMillis = millis();
      
      data_recieved(); //Collects data from android control app

      //Checks battery voltage each second
      if (currentMillis - previousMillis1 >= 1000) 
          {
            previousMillis1 = currentMillis;
            battery_voltage[battery_counter] = analogRead(A0);
            battery_counter++;
            
            //Checks average voltage after each minute
            if(battery_counter == 60)
            {
              battery_counter = 0;
              long int ave_voltage = 0;
              for(int i=0 ; i<60 ; i++) 
              {
                ave_voltage += battery_voltage[i];
              }
              
              ave_voltage = ave_voltage/60;

              BLUETOOTH.println();
              BLUETOOTH.print("Battery Voltage = ");
              BLUETOOTH.print(12.6*ave_voltage/1023);
              BLUETOOTH.print("V");
              BLUETOOTH.println();
              
              if(ave_voltage < 770) siren(); //Sounds alarm if battery is under 9.5V
            }
          }

      //Sets control mode
      switch(data_byte[0])
      {
        case '0': //App in car control mode
        {
          carmode();
          break;
        }
        case '1': //App in arm control mode
        { 
          if (currentMillis - previousMillis >= 10) //Updates servo position every 10ms
          {
            previousMillis = currentMillis;
            armmode();
          }
          break;
        }
        default:
        {
          break;
        }
      } 
    }
  }
  else if(!(PIND & B00000100)) //When not connected
    {
      disc(); 
      while(!(PIND & B00000100)){} //Waits till connection is established
    }
}




