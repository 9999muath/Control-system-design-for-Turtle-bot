//--+-+-+-+-+-+-+motor_1 pins OUTPUT & INPUT-+-+-+-+-+-+-+-+
int EncoderMotor_1 = 3;
int ClockW_motor1 = 11, AntiClockW_motor1 = 10;
int I_speed_motor1 = A4,O_speed_motor1, EN1 =12;
long int C1 =0; //COUNTER Number of motor_1 turns
int Dir_MOT1;

//-+-+-+-+-+-+-+-motor_2 pins OUTPUT & INPUT-+-+-+-+-+-+-+
int EncoderMotor_2 = 2;
int I_speed_motor2 = A5, O_speed_motor2 , EN2 =7;
int ClockW_motor2 = 9, AntiClockW_motor2 = 8;
long int c2 =0;//COUNTER Number of motor_2 turns
int Dir_MOT2;

//-+-+-+-+-+-+-+-+-+-Direction INPUT-+-+-+-+-+-+-+-+-+-+
int FORWARD_Robot =A0, BACKWARD_Robot =A1, RightRobot = A2, LeftRobot =A3 ; 

//-+-+-+-+-+-+-+-+-+-UltraSonic Sensor INPUT & OUTPUT-+-+-+-+-+-+-+-+-+-+
int TrigPin=6, EchoPin = 5;
long duration; int distance;
int LED =13;

//***********************************************************VOID Setup()************************
void setup()
{Serial.begin(9600);
//--+-+-+-+-+-+-+motor_1 pins OUTPUT & INPUT-+-+-+-+-+-+-+-+
pinMode( ClockW_motor1, OUTPUT);
pinMode( AntiClockW_motor1, OUTPUT);
pinMode( EN1, OUTPUT);
attachInterrupt(digitalPinToInterrupt(2), counter1, CHANGE);

//--+-+-+-+-+-+-+motor_2 pins OUTPUT & INPUT-+-+-+-+-+-+-+-+
pinMode( ClockW_motor2, OUTPUT);
pinMode( AntiClockW_motor2, OUTPUT);
pinMode( EN2, OUTPUT);
attachInterrupt(digitalPinToInterrupt(3), counter2, CHANGE);

//-+-+-+-+-+-+-+-+-+-UltraSonic Sensor INPUT & OUTPUT-+-+-+-+-+-+-+-+-+-+
pinMode(TrigPin, OUTPUT);
pinMode(EchoPin, INPUT);

//-+-+-+-+-+-+-+-+-+-Direction INPUT-+-+-+-+-+-+-+-+-+-+
pinMode(FORWARD_Robot, INPUT);
pinMode(BACKWARD_Robot, INPUT);
pinMode(RightRobot, INPUT);
pinMode(LeftRobot, INPUT);
}
//***********************************************************VOID Loop()************************
void loop(){
O_speed_motor1= map(analogRead(I_speed_motor1 ),0,1023,0,255);
O_speed_motor2= map(analogRead(I_speed_motor2),0,1023,0,255);

//__________-_-_ Robot in normly not move _____-_-_-_-__-_
if (digitalRead(FORWARD_Robot)==LOW && digitalRead(BACKWARD_Robot)==LOW && digitalRead(LeftRobot)==LOW && digitalRead(RightRobot)==LOW)
{
digitalWrite (ClockW_motor1, LOW);
digitalWrite (AntiClockW_motor1, LOW);
digitalWrite (ClockW_motor2, LOW);
digitalWrite (AntiClockW_motor2, LOW);
analogWrite(EN1,0);
analogWrite(EN2,0);
Serial.println("the Robot not move ");
delay(1000);
}

//__________-_-_ Robot in forward case _____-_-_-_-__-_
else if (digitalRead(FORWARD_Robot)==HIGH && digitalRead(BACKWARD_Robot)==LOW && digitalRead(LeftRobot)==LOW && digitalRead(RightRobot)==LOW)
{
digitalWrite (ClockW_motor1, HIGH);
digitalWrite (AntiClockW_motor1, LOW);
Dir_MOT1 = 1;
digitalWrite (ClockW_motor2, HIGH);
digitalWrite (AntiClockW_motor2, LOW);
Dir_MOT2 = 1;
analogWrite(EN1,O_speed_motor1);
analogWrite(EN2,O_speed_motor2);
Serial.println("the Robot move FORWARD ");
delay(1000);
}

//__________-_-_ Robot in backward case _____-_-_-_-__-_
else if (digitalRead(FORWARD_Robot)==LOW && digitalRead(BACKWARD_Robot)==HIGH && digitalRead(LeftRobot)==LOW && digitalRead(RightRobot)==LOW)
{
digitalWrite (ClockW_motor1, LOW);
digitalWrite (AntiClockW_motor1, HIGH);
Dir_MOT1 = 0;
digitalWrite (ClockW_motor2, LOW);
digitalWrite (AntiClockW_motor2, HIGH);
Dir_MOT2 = 0;
analogWrite(EN1,O_speed_motor1);
analogWrite(EN2,O_speed_motor2);
Serial.println("the Robot move BACKWARD ");
delay(1000);
}

//__________-_-_ Robot in LEFT case _____-_-_-_-__-_
else if (digitalRead(FORWARD_Robot)==LOW && digitalRead(BACKWARD_Robot)==LOW && digitalRead(LeftRobot)==HIGH && digitalRead(RightRobot)==LOW)
{
digitalWrite (ClockW_motor1, HIGH);
digitalWrite (AntiClockW_motor1, LOW);
Dir_MOT1 = 1;
digitalWrite (ClockW_motor2, LOW);
digitalWrite (AntiClockW_motor2, LOW);
Dir_MOT2 = 0;
analogWrite(EN1,O_speed_motor1);
analogWrite(EN2,0);
Serial.println("the Robot move LEFT ");
delay(1000);
}

//__________-_-_ Robot in RIGHT case _____-_-_-_-__-_
else if (digitalRead(FORWARD_Robot)==LOW && digitalRead(BACKWARD_Robot)==LOW && digitalRead(LeftRobot)==LOW && digitalRead(RightRobot)==HIGH)
{
digitalWrite (ClockW_motor1, LOW);
digitalWrite (AntiClockW_motor1, LOW);
Dir_MOT1 = 0;
digitalWrite (ClockW_motor2, HIGH);
digitalWrite (AntiClockW_motor2, LOW);
Dir_MOT2 = 1;
analogWrite(EN1,0);
analogWrite(EN2,O_speed_motor2);
Serial.println("the Robot move RIGHT ");
delay(1000);
}
// Example If button forward & backward are pressed in the same Time
else {digitalWrite (ClockW_motor1, LOW);
digitalWrite (AntiClockW_motor1, LOW);
digitalWrite (ClockW_motor2, LOW);
digitalWrite (AntiClockW_motor2, LOW);
Serial.println("EROR!!!");
delay(1000);
}

//-+-+-+-+-+-+-+-+-+-UltraSonic Prpgraming -+-+-+-+-+-+-+-+-+-+
digitalWrite(TrigPin, LOW);
delayMicroseconds(2);
digitalWrite(TrigPin, HIGH);
delayMicroseconds(10);
digitalWrite(TrigPin, LOW);
duration = pulseIn(EchoPin, HIGH);
distance = duration*0.034/2;
Serial.print("distance in cm = ");
Serial.println(distance);
  if (distance<=30){digitalWrite(LED, HIGH); Serial.println("WARNING!! The body is very close");}
  else if(distance>30){digitalWrite(LED, LOW);} 

//-+-+-+-+-+-+-+-+-+-Print data -+-+-+-+-+-+-+-+-+-+ 
Serial.print("Number of MOTOR_1 turns= ");
Serial.println(C1/47);
Serial.print("Number of MOTOR_2 turns=");
Serial.println(c2/47);
delay(200);
}

//-+-+-+-+-+-+-+-+-+-COUNTER MOTOR 1 -+-+-+-+-+-+-+-+-+-+ 
void counter1 (){
if(Dir_MOT1 ==1)
C1++;
else if (Dir_MOT1==0)
C1--; 
}

//-+-+-+-+-+-+-+-+-+-COUNTER MOTOR 2 -+-+-+-+-+-+-+-+-+-+ 
void counter2 (){
if(Dir_MOT2 ==1)
c2++;
else if (Dir_MOT2==0)
c2--; 
}
