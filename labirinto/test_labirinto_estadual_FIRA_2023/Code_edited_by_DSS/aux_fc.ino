/********** motores **********/

void isrL (){
  motorL.isr();
}

void isrR (){
  motorR.isr();
}



/********** sensores de distância **********/

void readDistances(){
  distanceR = ultraR.getDistance_cm();
  distanceL = ultraL.getDistance_cm();
}

void print(String text){
   Serial.print(text);
}

/********** botão **********/

void waitForButton (){
  while(digitalRead(but_pin));
}
