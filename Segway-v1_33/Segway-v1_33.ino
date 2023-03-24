
// Martini V1.1

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
// Interface ###########################
int led1 = 13;
int Hazard, last_Hazard;
int i=1;
int botao = 11;
int timer = 1;

// MPU-6050 ###########################
MPU6050 accelgyro;
int16_t ax, ay, az, gx, gy, gz;
double CALIBRATE_ary, CALIBRATE_gsy, CALIBRATE2_ary, CALIBRATE2_gsy;
double timeStep, time, timePrev;
double ary, gry, gsy, ry;
double ary_accumulate;
double gyroScale = 131;
double correction = 0.03;
double decay = 0.0025;

// L298N ###########################
int INPWM1 = 5;
int IN1 = 7;
int IN2 = 6;
int IN3 = 9;
int IN4 = 8;
int INPWM2 = 10;
double MotorLevel = 60;
double MotorLimit;  //Limites do motor para que os dois motores
                    //funcionem direitinho pq sao diferentes
double charge;  //Mostra o valor do PID que está sendo aplicado

// Control-PID phi & theta ###########################
double setPoint; //Phi em que deve se fixar (é impreciso)
double bestSetPoint; //Phi de posição calibrado de referencia
double erro; //Phi atual - Phi referencia
double erro2; //Theta atual - theta referencia
double alfa;



double bateria = 1; // 0.9 bateria cheia, 1.0 bateria metade, 1.2 bateria acabando
// high battery = 25.2/2.5/5.5 legacy
// high battery = 50.2/3.5/30.5 (6.56 V)
// low battery =  50/1.4/60

double KP_phi = 25.2,
       KI_phi = 2.5,
       KD_phi = 8.5;//5.5

//double KP_phi = 50.2, estava usando esse
//       KI_phi = 1.5,
//       KD_phi = 30.5;

//double KP_phi = 50,
//       KI_phi = 1.4,
//       KD_phi = 60;

//high battery = 0.2/0.1/0.005 (6.56V)
//low battery = 0.002/0.05/0.1


double bateria2 = 1;
double KP_pos =0.18,// 0.2, //  *0.002,
       KI_pos =0.10,// 0.10,//   *0.05,
       KD_pos =0.105;// 0.085;//    *0.1;

double refPosicao = 0;
double ligaPosicao = 0.5;
double P_phi,I_phi,D_phi, PID_phi;
double P_pos,I_pos,D_pos, PID_pos;
double PIDs, PID_accumulate, PID_accumulate2, PID_accumulate_alt, PID_accumulate_last, PID_accumulate_last2;
double Filter_ary, last_ary, last_gsy, Delta, Filter_Delta, last_Delta;

double LastPIDs; //Armazena sinal PID anterior
double veloci; //Integra PIDs para achar velocidade
double Lastveloci; //Armazena velocidade anterior
double positi; //Integra VELOCI para achar posição aproximada



void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
Wire.begin();
accelgyro.initialize();
pinMode(botao, INPUT);
pinMode(led1, OUTPUT);
pinMode(IN1, OUTPUT);
pinMode(IN2, OUTPUT);
pinMode(IN3, OUTPUT);
pinMode(IN4, OUTPUT);

for (i=1;i<11;i++) {
accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
gsy = gy/gyroScale;
CALIBRATE_ary = -(180/3.141592) * atan(ax / sqrt(square(ay) + square(az))); 
CALIBRATE2_ary = CALIBRATE2_ary + CALIBRATE_ary;
CALIBRATE_gsy = gsy;
CALIBRATE2_gsy = CALIBRATE2_gsy + CALIBRATE_gsy;
}
CALIBRATE2_ary = CALIBRATE2_ary/10;
CALIBRATE2_gsy = CALIBRATE2_gsy/10;

while (digitalRead(botao) == LOW) {
  digitalWrite(led1, HIGH);
  delay(100);
}
digitalWrite(led1, LOW);
delay(1500);
}

void loop() {
  // put your main code here, to run repeatedly:

// set up time for integration
timePrev = time;
time = millis();
timeStep = (time - timePrev) / 1000; // time-step in s
accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

ary = -(180/3.141592) * atan(ax / sqrt(square(ay) + square(az))) - CALIBRATE2_ary;
gsy = gy/gyroScale - CALIBRATE2_gsy;

if (abs(ary)>0.27){
  gry = gry + (timeStep) * (gsy) + ((ary-gry)/100);
} else {
  gry = gry * 0.9;
}

Filter_ary = gry;

Delta = Filter_ary - last_ary;
Filter_Delta = Delta;

// PID Phi ###########################
erro = Filter_ary - setPoint;

P_phi = erro * KP_phi;
I_phi += erro * KI_phi;
D_phi = (Filter_Delta) * KD_phi;

PID_phi = P_phi + I_phi + D_phi;

// PID Posicao ###########################
erro2 = positi - refPosicao;

P_pos = erro2 * KP_pos; 
I_pos += erro2 * KI_pos;
if(I_pos>400){I_pos = 395;} else if (I_pos<-400){I_pos = -395;}

D_pos = (PID_accumulate - PID_accumulate_last2) * KD_pos;

PID_pos = P_pos + I_pos + D_pos;
//PID = PID_phi + PID_theta;

//if (PID_pos<0){
//  PID_pos = 1.2*PID_pos;
//}

PIDs = 1.0*((1.2)*PID_phi + (0.6)*PID_pos);

MotorLimit = 254-MotorLevel;
if (PIDs > 1 && Hazard <60){
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  if (PIDs > MotorLimit){PIDs = MotorLimit;}
  charge = PIDs + MotorLevel;
  analogWrite(INPWM1, charge);
  analogWrite(INPWM2, charge);
  
} else if (PIDs < -1 && Hazard <60){
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  if (-PIDs > MotorLimit){PIDs = -MotorLimit;}
  charge = -PIDs + MotorLevel;
  analogWrite(INPWM1, charge);
  analogWrite(INPWM2, charge);
  charge = -charge;

  } else {
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(INPWM1, 0);
  analogWrite(INPWM2, 0);
  }

if (abs(Filter_ary)>=last_Hazard){
  Hazard = abs(Filter_ary);
}
last_Hazard = Hazard;

if (I_phi>400){
  I_phi=395;
} else if (I_phi<-400){
  I_phi=-395;
}
if (I_pos>200){
  I_pos=195;
} else if (I_pos<-200){
  I_pos=-195;
}

//if (P_pos>400){
//  P_pos=395;
//} else if (P_pos<-400){
//  P_pos=-395;
//}

  // LIMITES DE PID EVITAM OVERSHOOT


if (PID_accumulate>4000){
  PID_accumulate=3950;  
} else if (PID_accumulate<-4000){
  PID_accumulate=-3950;
}

if (PID_accumulate2>8000){
  PID_accumulate2=0;
  setPoint += correction;  
  correction += -decay;  
} else if (PID_accumulate2<-8000){
  PID_accumulate2=0;
  setPoint += -correction;
  correction += -decay;
}

last_ary = Filter_ary;
last_gsy = gsy;
last_Delta = Filter_Delta;

timer += 1;


veloci = (PIDs+LastPIDs)*0.5*timeStep;
positi = (veloci+Lastveloci)*0.5*timeStep *550;
PID_accumulate_last2 = PID_accumulate;

PID_accumulate += PIDs;



PID_accumulate2 += PIDs;

//PID_accumulate_alt += PID;
PID_accumulate_alt += abs(PIDs);

LastPIDs = PIDs;
Lastveloci = veloci;

//Serial.println(val);
//Serial.print(P);
//Serial.print(",");


//Serial.print(P_phi);
//Serial.print(",");
//Serial.print(I_phi);
//Serial.print(",");
//Serial.print(D_phi);
//Serial.print(",");
//Serial.print(PID_phi);
//Serial.print(",");
//Serial.println(PIDs);
//
//Serial.println(P_pos);
//Serial.print(",");
//Serial.print(I_pos);
//Serial.print(",");
//Serial.print(D_pos);
//Serial.print(",");
//Serial.print(PID_pos);
//Serial.print(",");
//Serial.println(PID_pos);

//Serial.println(PID_pos);
//Serial.print(",");
//Serial.println(PID_phi);

//Serial.print(PID_pos);
//Serial.print(",");
//Serial.print(PID_phi);
//Serial.print(",");
//Serial.println(PID_accumulate_alt);

//Serial.print(ax);
//Serial.print(",");
//Serial.print(ay);

//Serial.print(",");
//Serial.println(az);

//Serial.print(Filter_ary);
//Serial.print(",");
//Serial.println(ary);


//Serial.print(CALIBRATE2_ary);
//Serial.print(",");
//Serial.print(PIDs);
//Serial.print(",");
//Serial.println(timeStep);

//Serial.print(bestSetPoint);
//Serial.print(",");
//Serial.println(setPoint);

//Serial.println(charge);
}
