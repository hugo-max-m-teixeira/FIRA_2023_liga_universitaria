// Bibliotecas :
#include <DC_motor_controller.h>
#include <TwoMotors.h>
#include <My_ultrassonic.h>

/********** Pinos **********/

// Motores:
// Direito (right):
#define IN1_R 8 
#define IN2_R 9
#define EN_R 11
#define A_motor_R 2
#define B_motor_R 4

// Esquerdo (left):
#define IN1_L 6
#define IN2_L 7
#define EN_L 10

#define A_motor_L 3
#define B_motor_L 5


// Sensores de distância:
// Sensor direito (right):
#define trig_R A2
#define echo_R A3

// Seansor esquerdo(left):
#define trig_L A0
#define echo_L A1

#define show_logs true
#define test_all
#define test_ultra 

/********** Objetos **********/
DC_motor_controller motorL;
DC_motor_controller motorR;

TwoMotors both(&motorR, &motorL);

My_ultrassonic ultraR(trig_R, echo_R);
My_ultrassonic ultraL(trig_L, echo_L);


/********** protótipo de funções **********/
void print(String text);

/********** Constantes e variáveis **********/
float distanceL=0, distanceR=0;


void setup() {
  // put your setup code here, to run once:
  // Motor direito (right):
  if(show_logs) Serial.begin(9600);
  print("iniciando programa...\n");
  
  motorR.hBridge(IN1_R, IN2_R, EN_R);
  motorR.setEncoderPin(A_motor_R, B_motor_R);
  motorR.setRR(21.3);
  motorR.setPins();
  //motorR.debugMaxVel();
  attachInterrupt(0 , isrR, FALLING);

  // Motor esquerdo (left):
  motorL.hBridge(IN1_L, IN2_L, EN_L);
  motorL.setEncoderPin(A_motor_L, B_motor_L);
  motorL.setRR(21.3);
  motorL.setPins();
  //motorL.debugMaxVel();
  attachInterrupt(1 , isrL, FALLING);

  both.setGyreDegreesRatio(3.7, 180);

  /*#if defined test
    testCode();
  #endif*/
}

void loop() {
  // put your main code here, to run repeatedly:
  testCode();
  return;
  
}
