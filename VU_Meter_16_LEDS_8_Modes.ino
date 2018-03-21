/*  Compiled with Arduino IDE ver.1.6.12, ver.1.8.5 by Wilson Carvajal
    Vu-Meter Code by Wilson Carvajal
    Adafruit Electret Microphone Amplifier - MAX4466 with Adjustable Gain connected to Analogo Pin A5.
    Push button conneted  pin 1  to ground with a 10k resistor.
    All leds cathodes to ground and anode connected through a 120 ohms resistor to Arduino pins.
    Resistor values change if different led colors are used.
    Code adapted from Adafruit's amplietie code and Toni Birka VU-Meter: https://www.youtube.com/watch?v=UDnqc9HgAX8
    I added the VU With PEAK HOLD.
    Enjoy it!
*/

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define N_LEDS    16     // Number of LEDs
#define MIC_PIN   A5     // Microphone is attached to this analog pin
#define DC_OFFSET  0     // DC offset in mic signal - if unusure, leave 0
#define NOISE     10     // Noise/hum/interference in mic signal
#define SAMPLES   60     // Length of buffer for dynamic level adjustment
#define TOP       N_LEDS // Use to calculate bar height

unsigned long recByteCount = 0L;
unsigned long recByteSaved = 0L;
unsigned long oldTime = 0L;
unsigned long newTime = 0L;
byte buf00[512]; // buffer array 1
byte buf01[512]; // buffer array 2
byte byte1, byte2, byte3, byte4;
unsigned int bufByteCount;
byte bufWrite;

byte
dotCount  = 0,           // Frame counter for delaying dot-falling speed
volCount  = 0;           // Frame counter for storing past volume data
int
vol[SAMPLES],            // Collection of prior volume samples
    lvl       = 10,      // Current "dampened" audio level
    minLvlAvg = 0,       // For dynamic adjustment of graph low & high
    maxLvlAvg = 512;

int counter = 0;
const int buttonPin = 1;
long samplePeriod = 2;
long peakFallPeriod = 2000;
long peakLeft = 0;
long fallSpeed = 0;
float fallAcc = 0.4;
long lastPeakMs = 0;
signed int peak = 0;            // Used for falling dot
float maxLevel = 1.1;
float minLevel = 0.09;

const int numReadings = 20;     // Number of samples to keep track of (for smoothing)
int total = 0;                  // the running total
int readings[numReadings];      // the readings from the analog input
int index = 0;                  // the index of the current reading
int ledLevel;                   // Value to map to the output LEDs
int audioValue;                 // Analog value read from audio channel
int average = 0;                // the average
int maxAudioValue = 0;          // Maximum analog value read from audio channel

// Variables will change:

int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;

void setup() {
  delay(3000);               //Power-Up Safety delay
  Setup_timer2();
  Setup_ADC();
  analogReference(EXTERNAL);
  memset(vol, 0, sizeof(vol));
}

void loop() {
  buttonState = digitalRead(buttonPin);   // read the pushbutton input pin:
  if (buttonState != lastButtonState) {   // compare the buttonState to its previous state
    if (buttonState == HIGH) {            // if the state has changed, increment the counter

      buttonPushCounter++;                // if the current state is HIGH then the button went from off to on:
      if (buttonPushCounter == 10) {
        buttonPushCounter = 1;
      }
    }
    else {                                // if the current state is LOW then the button went from on to off:

    }
  }

  lastButtonState = buttonState;         // save the current state as the last state, for next time through the loop

  switch (buttonPushCounter) {
    case 1:
      buttonPushCounter == 1; {
        Vu1(); //
        break;
      }
    case 2:
      buttonPushCounter == 2; {
        Vu2(); //
        break;
      }
    case 3:
      buttonPushCounter == 3; {
        Vu3(); //
        break;
      }
    case 4:
      buttonPushCounter == 4; {
        Vu4(); //
        break;
      }
    case 5:
      buttonPushCounter == 5; {
        Vu5(); //
        break;
      }
    case 6:
      buttonPushCounter == 6; {
        Vu6(); //
        break;
      }
    case 7:
      buttonPushCounter == 7; {
        Vu7(); //
        break;
      }

    case 8:
      buttonPushCounter == 8; {
        Vu8(); //
        break;
      }

    case 9:
      buttonPushCounter == 9; {
        Vu9(); //
        break;
      }
  }
}
//======================== MODE 1 ("VU-UP") =============================//
void Vu1() {
  const int led[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
  //const int led[] = {2, 10, 3, 11, 4, 12, 5, 13, 6, 14, 7, 15, 8, 16, 9, 17};
  for (int i = 0; i < N_LEDS; i++)  {
    pinMode(led[i], OUTPUT);
  }

  sbi (TIMSK2, OCIE2A); // enable timer interrupt, start grabbing audio

  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int n, height;
  n   = analogRead(MIC_PIN); // Raw reading from mic
  n   = abs(n - 512 - DC_OFFSET); // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);     // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  for (i = 0; i < N_LEDS; i++)
  {
    if (i < height)
    {
      digitalWrite(led[i], HIGH);
    }
    else
    {
      digitalWrite(led[i], LOW);
    }
    delayMicroseconds(250);
  }

  vol[volCount] = n;   // Save sample for dynamic leveling
  if (++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter
  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++) {
    if (vol[i] < minLvl) minLvl = vol[i];
    else if (vol[i] > maxLvl) maxLvl = vol[i];
  }
  if ((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}

//======================== MODE 2 ("VU-DOWN") =============================//
void Vu2() {
  const int led[] = {17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2};
  for (int i = 0; i < N_LEDS; i++)  {
    pinMode(led[i], OUTPUT);
  }

  sbi (TIMSK2, OCIE2A); // enable timer interrupt, start grabbing audio

  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int n, height;
  n   = analogRead(MIC_PIN); // Raw reading from mic
  n   = abs(n - 512 - DC_OFFSET); // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);     // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  for (i = 0; i < N_LEDS; i++)
  {
    if (i < height)
    {
      digitalWrite(led[i], HIGH);
    }
    else
    {
      digitalWrite(led[i], LOW);
    }
    delayMicroseconds(250);
  }

  vol[volCount] = n;                                  // Save sample for dynamic leveling
  if (++volCount >= SAMPLES) volCount = 0;            // Advance/rollover sample counter
  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++) {
    if (vol[i] < minLvl) minLvl = vol[i];
    else if (vol[i] > maxLvl) maxLvl = vol[i];
  }
  if ((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6;         // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6;         // (fake rolling average)
}

//======================== MODE 3 ("VU-UP PEAK HOLD") =============================//
void Vu3() {
  sbi (TIMSK2, OCIE2A); // enable timer interrupt, start grabbing audio
  float level = sampleAudio();
  updatePeak();
  displayLevel(level);
}

//======================== MODE 4 ("VU-UP PEAK HOLD") =============================//
void Vu4() {
  sbi (TIMSK2, OCIE2A); // enable timer interrupt, start grabbing audio
  float level = sampleAudio();
  updatePeak();
  displayLevel1(level);
}

//======================== MODE 5 ("VU-SIDES TO MIDDLE") =============================//
void Vu5() {
  const int led[] = {9, 10, 8, 11, 7, 12, 6, 13, 5, 14, 4, 15, 3, 16, 2, 17};
  for (int i = 0; i < N_LEDS; i++)  {
    pinMode(led[i], OUTPUT);
  }

  sbi (TIMSK2, OCIE2A); // enable timer interrupt, start grabbing audio

  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int n, height;
  n   = analogRead(MIC_PIN); // Raw reading from mic
  n   = abs(n - 512 - DC_OFFSET); // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);     // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  for (i = 0; i < N_LEDS; i++)
  {
    if (i < height)
    {
      digitalWrite(led[i], HIGH);
    }
    else {
      digitalWrite(led[i], LOW);
    }
    delayMicroseconds(250);
  }

  vol[volCount] = n;   // Save sample for dynamic leveling
  if (++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter
  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++) {
    if (vol[i] < minLvl) minLvl = vol[i];
    else if (vol[i] > maxLvl) maxLvl = vol[i];
  }
  if ((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}

//======================== MODE 6 ("VU - MIDDLE TO SIDES") =============================//
void Vu6() {
  const int led[] = {17, 2, 16, 3, 15, 4, 14, 5, 13, 6, 12, 7, 11, 8, 10, 9};
  for (int i = 0; i < N_LEDS; i++)  {
    pinMode(led[i], OUTPUT);
  }

  sbi (TIMSK2, OCIE2A); // enable timer interrupt, start grabbing audio

  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int n, height;
  n   = analogRead(MIC_PIN); // Raw reading from mic
  n   = abs(n - 512 - DC_OFFSET); // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);     // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  for (i = 0; i < N_LEDS; i++)
  {
    if (i < height)
    {
      digitalWrite(led[i], HIGH);
    }
    else
    {
      digitalWrite(led[i], LOW);
    }
    delayMicroseconds(250);
  }

  vol[volCount] = n;   // Save sample for dynamic leveling
  if (++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter
  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++) {
    if (vol[i] < minLvl) minLvl = vol[i];
    else if (vol[i] > maxLvl) maxLvl = vol[i];
  }
  if ((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}

//======================== MODE 7 ("VU - DOT-MODE") =============================//
void Vu7() {
  const int led[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
  const int  sensitivityValue = 1200;
  for (int i = 0; i < N_LEDS; i++) {
    pinMode(led[i], OUTPUT);
  }

  sbi (TIMSK2, OCIE2A); // enable timer interrupt, start grabbing audio

  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int n, height;
  n   = analogRead(MIC_PIN); // Raw reading from mic
  n   = abs(n - 512 - DC_OFFSET); // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);     // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);
  if (height > N_LEDS) {
    height = N_LEDS - 1;
  }

  for (i = 0; i < N_LEDS; i++)
  {
    if (i == height)
    {
      digitalWrite(led[i], HIGH);
    }
    else
    {
      digitalWrite(led[i], LOW);
    }
    delayMicroseconds(600);
  }

  vol[volCount] = n;   // Save sample for dynamic leveling
  if (++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter
  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++) {
    if (vol[i] < minLvl) minLvl = vol[i];
    else if (vol[i] > maxLvl) maxLvl = vol[i];
  }
  if ((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)

}

//======================== MODE 8 ("VU - DOT-MODE-2") =============================//
void Vu8() {
  const int led[] = {2, 17, 3, 16, 4, 15, 5, 14, 6, 13, 7, 12, 8, 11, 9, 10};
  const int  sensitivityValue = 1200;
  for (int i = 0; i < N_LEDS; i++) {
    pinMode(led[i], OUTPUT);
  }

  sbi (TIMSK2, OCIE2A); // enable timer interrupt, start grabbing audio

  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int n, height;
  n   = analogRead(MIC_PIN); // Raw reading from mic
  n   = abs(n - 512 - DC_OFFSET); // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);     // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);
  if (height > N_LEDS) {
    height = N_LEDS - 1;
  }

  for (i = 0; i < N_LEDS; i++)
  {
    if (i == height)
    {
      digitalWrite(led[i], HIGH);
    }
    else
    {
      digitalWrite(led[i], LOW);
    }
    delayMicroseconds(600);
  }

  vol[volCount] = n;   // Save sample for dynamic leveling
  if (++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter
  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++) {
    if (vol[i] < minLvl) minLvl = vol[i];
    else if (vol[i] > maxLvl) maxLvl = vol[i];
  }
  if ((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)

}

//========================MODE 9 ("Stand by mode") =============================//
void Vu9() {
  counter = 0;
  const int led[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
  cbi (TIMSK2, OCIE2A); // disable timer interrupt
  for (int i = 0; i < N_LEDS; i++)
    digitalWrite(led[i], LOW);
}

float sampleAudio () {
  long start = millis();
  long sample;
  long minSample = 1023;
  long maxSample = 0;

  while (millis() - start < samplePeriod) {
    sample = analogRead(MIC_PIN);

    if (sample < minSample) {
      minSample = sample;
    }
    if (sample > maxSample) {
      maxSample = sample;
    }
  }
  return (maxSample - minSample) * 3.3 / 1023.0;
}

void displayLevel(float level) {
  const int led[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};

  // Calculate number of segments to display
  float Delta = maxLevel - minLevel;
  float delta = level - minLevel;
  int on;
  if (delta < 0) {
    on = 0;
  }
  else {
    on = min(16, int(9.0 * delta / Delta));
  }
  // Update the peak if necessary
  if (on > peak) {
    peak = on;
    peakLeft = peakFallPeriod;
    fallSpeed = 0;
  }

  // Write segments top to bottom
  int off = 16 - on;

  for (int i = 0; i < off; i++) {
    digitalWrite(led[i], LOW);
    if (i == -1 + peak) {
      digitalWrite(led[i], HIGH);
    }
  }
  for (int i = 0; i < on; i ++) {
    digitalWrite(led[i], HIGH);
  }
}

void displayLevel1(float level) {
  int led[] = {17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2};

  // Calculate number of segments to display
  float Delta = maxLevel - minLevel;
  float delta = level - minLevel;
  int on;
  if (delta < 0) {
    on = 0;
  }
  else {
    on = min(16, int(9.0 * delta / Delta));
  }
  // Update the peak if necessary
  if (on > peak) {
    peak = on;
    peakLeft = peakFallPeriod;
    fallSpeed = 0;
  }

  // Write segments top to bottom
  int off = 16 - on;

  for (int i = 0; i < off; i++) {
    digitalWrite(led[i], LOW);
    if (i == -1 + peak) {
      digitalWrite(led[i], HIGH);
    }
  }
  for (int i = 0; i < on; i ++) {
    digitalWrite(led[i], HIGH);
  }
}

void updatePeak () {
  long ms = millis();
  // Drop the peak if enough time has passed
  if (fallSpeed * (ms - lastPeakMs) > peakLeft) {
    peakLeft = peakFallPeriod - fallSpeed * (ms - lastPeakMs);
    peak = max(-1, peak - 1);
  } else {
    peakLeft = peakLeft - fallSpeed * (ms - lastPeakMs);
  }
  fallSpeed += fallAcc * (ms - lastPeakMs);
  lastPeakMs = ms;
}

void Setup_timer2() {

  TCCR2B = _BV(CS21);  // Timer2 Clock Prescaler to : 8
  TCCR2A = _BV(WGM21); // Interupt frequency  = 16MHz / (8 x 90 + 1) = 22191Hz
  OCR2A = 90; // Compare Match register set to 90

}

void Setup_ADC() {

  ADMUX = 0x65; // set ADC to read pin A5, ADLAR to 1 (left adjust)
  cbi(ADCSRA, ADPS2); // set prescaler to 8 / ADC clock = 2MHz
  sbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);
}

ISR(TIMER2_COMPA_vect) {

  sbi(ADCSRA, ADSC); // start ADC sample
  while (bit_is_set(ADCSRA, ADSC)); // wait until ADSC bit goes low = new sample ready
  recByteCount++; // increment sample counter
  bufByteCount++;
  if (bufByteCount == 512 && bufWrite == 0) {
    bufByteCount = 0;
    bufWrite = 1;
  } else if (bufByteCount == 512 & bufWrite == 1) {
    bufByteCount = 0;
    bufWrite = 0;
  }

  if (bufWrite == 0) {
    buf00[bufByteCount] = ADCH;
  }
  if (bufWrite == 1) {
    buf01[bufByteCount] = ADCH;
  }

}