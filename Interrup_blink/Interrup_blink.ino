byte state = LOW;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  TCCR2A = 0x00;   //Configuração do TIMER2: NORMAL
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // 0x07 Prescaler 1:1024
  TCNT2  = 100;    //10 ms de overflow
  TIMSK2 |= (1 << TOIE2); // 0x01 Habilita interrupção do Timer2

  // Estouro = Timer2_cont x prescaler x ciclo de máquina
  // Ciclo de máquina = 1/Fosc = 1/16E6 = 62,5ns = 62,5E-9s
  // Estouro = (256 - 100) x 1024 x 62,5E-9 = 9,98ms
}

void loop() {}

ISR(TIMER2_OVF_vect) {
  TCNT2 = 100;
  toggleLed();
}

void toggleLed() {
  static int aux = 0;
  if (aux >= 50) {
    digitalWrite(LED_BUILTIN, state);
    state = !state;
    aux = 0;
  }
  aux++;

}
