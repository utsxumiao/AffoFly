#define CLOCK_MULTIPLIER 1
#define THROTTLE_PIN  A0
#define YAW_PIN       A1
#define PITCH_PIN     A2
#define ROLL_PIN      A3
#define PPM_PIN       2
#define BUZZER_PIN    9

const uint16_t PPM_FRAME_LENGTH = 24000;
const uint16_t PPM_PULSE_LENGTH = 400;
const uint8_t CHANNEL_COUNT = 4;
uint16_t PPM[CHANNEL_COUNT];


struct MyData {
  uint16_t Throttle;
  uint16_t Yaw;
  uint16_t Pitch;
  uint16_t Roll;
};
MyData data;

void setup() {
  pinMode(PPM_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  setupPPM();
  resetData();
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
}

void loop() {
  unsigned long currentMillis = millis();
  setPPMValuesFromData();
}

void getPayloadData() {
  data.Throttle = mapJoystickValues(analogRead(THROTTLE_PIN), 0, 511, 1023, false );
  data.Yaw      = mapJoystickValues(analogRead(YAW_PIN),  0, 515, 1023, true );
  data.Pitch    = mapJoystickValues(analogRead(PITCH_PIN), 0, 513, 1023, false );
  data.Roll     = mapJoystickValues(analogRead(ROLL_PIN), 0, 504, 1023, true );
}

void setPPMValuesFromData(){
  PPM[0] = data.Throttle;
  PPM[1] = data.Yaw;
  PPM[2] = data.Pitch;
  PPM[3] = data.Roll;
}

void resetData() {
  data.Throttle = 1000;
  data.Yaw = 1500;
  data.Pitch = 1500;
  data.Roll = 1500;
}

uint16_t mapJoystickValues(uint16_t val, uint16_t lower, uint16_t middle, uint16_t upper, bool reverse) {
  val = constrain(val, lower, upper);
  if (val < middle) val = map(val, lower, middle, 1000, 1500);
  else val = map(val, middle, upper, 1500, 2000);
  return reverse ? 3000 - val : val;
}

void setupPPM() {
  //digitalWrite(PPM_PIN, 0);  //set the PPM signal pin to the default state (off)
  PORTD = PORTD & ~B00000100;
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;

  OCR1A = 100;  // compare match register (not very important, sets the timeout for the first interrupt)
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}

ISR(TIMER1_COMPA_vect) {
  static boolean state = true;
  TCNT1 = 0;
#ifdef DEBUG_MODE
    //Serial.print("PPM-Throttle: ");     Serial.println(PPM[0]);
#endif
  if ( state ) {
    //end pulse
    PORTD = PORTD & ~B00000100; // turn pin 2 off.
    OCR1A = PPM_PULSE_LENGTH * CLOCK_MULTIPLIER;
    state = false;
  }else {
    //start pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;

    PORTD = PORTD | B00000100; // turn pin 2 on.
    state = true;

    if (cur_chan_numb >= CHANNEL_COUNT) {
      cur_chan_numb = 0;
      calc_rest += PPM_PULSE_LENGTH;
      OCR1A = (PPM_FRAME_LENGTH - calc_rest) * CLOCK_MULTIPLIER;
      calc_rest = 0;
    }
    else {
      OCR1A = (PPM[cur_chan_numb] - PPM_PULSE_LENGTH) * CLOCK_MULTIPLIER;
      calc_rest += PPM[cur_chan_numb];
      cur_chan_numb++;
    }
  }
}
