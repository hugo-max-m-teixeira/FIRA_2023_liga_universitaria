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
#define show_logs true

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
const float	Kp = 1,
			Ki = 0.02,
			Kd = 0;
			
uint64_t last_compute;

const uint16_t refresh_time = 250;
const int set_point = 4;

const int PID_limit = 50;

#define using_integral_limit
const float integral_limit = 80;

/********** protótipo de funções **********/
void print(String text);

/********** Constantes e variáveis **********/
float distanceL=0, distanceR=0;


void setup() {
  // put your setup code here, to run once:
  // Motor direito (right):
  if(show_logs) Serial.begin(9600);
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
  
  last_compute = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  
  const int base_vel = 50;
  
  readDistances();
  
  int middle = (distanceR + distanceL) / 2;

  compute_PID(middle);
  
  
  both.together(base_vel - PID, base_vel + PID);
  

}

void compute_PID(float input){
	uint16_t delta_time = millis() - last_compute;
	
	if(delta_time >= refresh_time){
		delta_time = delta_time / 1000.0;
		
		error = set_point - input;
	
		P = error *  Kp;
		#ifdef using_integral_limit
			I += (abs(I) >= integral_limit) ? 0 : (error * Ki * delta_time); 
		#else 
			I += error * Ki * delta_time
		#endif
	
		D = error * Kd / delta_time;
		
		PID = P + I + D;
		
		if(PID > PID_limit) PID = PID_limit;
		
		last_compute = millis();
		last_error = error;
		
	}
}
