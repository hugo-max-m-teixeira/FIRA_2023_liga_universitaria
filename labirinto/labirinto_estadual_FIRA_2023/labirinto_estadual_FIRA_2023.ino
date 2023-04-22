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

// Botão push-button:
#define but_pin 12

// Definições de macros para testes (descomente/comente conforme necessário):
#define show_logs false

//#define print_PID_values
//#define ultra_test
//#define motors_test

/********** Objetos **********/
DC_motor_controller motorL;
DC_motor_controller motorR;

TwoMotors both(&motorR, &motorL);

My_ultrassonic ultraR(trig_R, echo_R);
My_ultrassonic ultraL(trig_L, echo_L);


/********** Controle PID **********/

float error, last_error, P, I, D, PID;
const float  Kp = 13,     //7.5
             Ki = 0.05,    //0.02
             Kd = 0.012;    //0.04

unsigned int  last_compute;

const uint16_t refresh_time = 90; //80
float set_point = 4;

const int PID_limit = 150;

#define using_integral_limit
const float integral_limit = 7;

/********** protótipo de funções **********/
void print(String text);

/********** Constantes e variáveis **********/
float distanceL = 0, distanceR = 0;

int vel_R = 0, vel_L = 0;


void setup() {
  // put your setup code here, to run once:
  // Motor direito (right):
  if (show_logs) Serial.begin(9600);
  print("iniciando programa...\n");

  //Sensor de distância ultrassônico:
  ultraL.setPins();
  ultraR.setPins();

  // Botão:
  pinMode(but_pin, INPUT_PULLUP);

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
  attachInterrupt(1 , isrL, FALLING);

  both.setGyreDegreesRatio(3.7, 180);

  testCode(); // Programa de auto-teste, caso as macros de teste estejam ativadas.

  waitForButton();
  
  both.together(50, 0.6);

  last_compute = millis();
}

void loop() {
  // put your main code here, to run repeatedly:

  const int base_vel = 100; //50

  readDistances();

  int middle = (distanceR + distanceL) / 2.0;

  //Set_point = 6;
  set_point = middle;

  compute_PID(distanceR);

  if (PID > 0) {
    PID = PID * 1.2;
    vel_R = base_vel + PID; // Gira mais devagar
    vel_L = base_vel - (PID); // Gira mais rápido
  } else {
    vel_R = base_vel + PID;
    vel_L = base_vel - PID;
  }
  /*
  vel_R = base_vel + PID;
  vel_L = base_vel - PID;
*/
  // Momento em que a esquerda vai para frente, maior dificuldade;
  //-> Roda esquerda mais rápida -> PID negativo



  print("\n\nVelcidade direito: ");
  print(String(vel_R));
  print('\t' + String(PID));
  print("\nVelcidade esquerdo: ");
  print(String(vel_L));



  motorR.walk(vel_R);
  motorL.walk(vel_L);
  print("\nDistancia ultra direito: " + (String)distanceR);

}

void compute_PID(float input) {
  float delta_time = millis() - last_compute;

  if (delta_time >= refresh_time) {
    delta_time = delta_time / 1000.0;

    error = set_point - input;

    P = error *  Kp;
#ifdef using_integral_limit
    I += (abs(I) >= integral_limit) ? 0 : (error * Ki * delta_time);
#else
    I += error * Ki * delta_time
#endif

    D = (error - last_error) * Kd / delta_time;


    PID = P + I + D;

#ifdef print_PID_values

    print("PID values:\nP = ");
    print(String(P));
    print("\nI = ");
    print(String(I));
    print("\nD = ");
    print(String(D));
    print("\n\n");

#endif


    if (PID > PID_limit) PID = PID_limit;

    last_compute = millis();
    last_error = error;

  }
}
