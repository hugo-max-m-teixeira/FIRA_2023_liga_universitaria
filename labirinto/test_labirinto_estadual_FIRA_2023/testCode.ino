void testCode() {
  #if defined test_ultra
    readDistances();
    print("\n\nDistancia Esquerdo: ");
    print(String(distanceL));
    print("\nDistancia Direito: ");
    print(String(distanceR));
  #endif
  //testMotors();


}

void testMotors() {
  const uint8_t vel = 50;
  const uint8_t rotations = 1;

  motorR.walk(vel, rotations);
  motorR.walk(-vel, -rotations);

  motorL.walk(vel, rotations);
  motorL.walk(-vel, -rotations);

  both.together(vel, rotations);
  both.together(-vel, -rotations);

  both.turnDegree(80, 180);
}
