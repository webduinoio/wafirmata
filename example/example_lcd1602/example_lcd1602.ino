uint8_t BUZZER = 14; //Mark1 Buzzer

void waSetup() {
  startMusic();
}

void waLoop() {
}


void onMessage(String data) {

}

void startMusic() {
  pinMode(BUZZER, OUTPUT);
  tone(BUZZER, 2000, 100);
  delay(150);
  tone(BUZZER, 3000, 100);
}
