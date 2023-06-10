///////////////////////////////////////////
/////////// Fetal Monitoring System ///////
///////////// by Marco Andrade ////////////
///////////////////////////////////////////

#include <LCDWIKI_GUI.h> // Core graphics library
#include <LCDWIKI_KBV.h> // Hardware-specific library

// LCD initialization
LCDWIKI_KBV mylcd(ILI9486, A3, A2, A1, A0, A4); // Model, cs, cd, wr, rd, reset

// Color definitions
const uint16_t BLACK = 0x0000;
const uint16_t BLUE = 0x001F;
const uint16_t RED = 0xF800;
const uint16_t GREEN = 0x07E0;
const uint16_t CYAN = 0x07FF;
const uint16_t MAGENTA = 0xF81F;
const uint16_t YELLOW = 0xFFE0;
const uint16_t WHITE = 0xFFFF;

// Threshold values
const int UpperThreshold = 520;

// Variables
int reading = 0;
float BPM = 0.0;
bool IgnoreReading = false;
bool FirstPulseDetected = false;
unsigned long FirstPulseTime = 0;
unsigned long SecondPulseTime = 0;
unsigned long PulseInterval = 0;

// Pin assignments
const int LoPOS = 22;     // Connect the Lo+ digital pin 22
const int LoNEG = 23;     // Connect the Lo- digital pin 23
const int V_AD8232 = 24;  // Output to energize AD8232
const int Vin_Pot = 27;   // Output to energize potentiometer
const int buzzerPin = 49; // Connect the buzzer to digital pin 49

void setup() {
  Serial.begin(9600);

  // Initialize LCD
  mylcd.Init_LCD();
  mylcd.Fill_Screen(BLACK);
  mylcd.Set_Text_Back_colour(BLACK);
  mylcd.Set_Text_Mode(0);

  // Initialize pin modes of AD8232
  pinMode(LoPOS, INPUT);
  pinMode(LoNEG, INPUT);
  pinMode(V_AD8232, OUTPUT);
  digitalWrite(V_AD8232, HIGH);
  // Set the pin to Vin of pot
  pinMode(Vin_Pot, OUTPUT);
  digitalWrite(Vin_Pot, HIGH);

  // Set the buzzer pin as an output
  pinMode(buzzerPin, OUTPUT);
}

// Function to calculate BPM
float calculateBPM(float sampleRate) {
  float bpm = 0.0;

  // Calculate BPM
  if (FirstPulseDetected) {
    SecondPulseTime = millis();
    PulseInterval = SecondPulseTime - FirstPulseTime;
    FirstPulseTime = SecondPulseTime;

    bpm = (1.0 / PulseInterval) * 60.0 * 1000;

    // Filter out low BPM readings when sampleRate is above 180 Hz
    filterBPM(bpm, sampleRate);
  }

  return bpm;
}

// Function to play sound on heartbeat
void playHeartbeatSound(int bpm) {
  int beepDuration = 60000 / bpm; // Calculate the duration of each beep
  int pauseDuration = beepDuration; // Set the pause duration equal to the beep duration for simplicity
  int maxIterations = 10; // Max number of iterations to attempt matching the BPM (prevent infinite loop)
  int iteration = 0; // Counter for the number of iterations

  while (iteration < maxIterations) {
    digitalWrite(buzzerPin, HIGH); // Activate the buzzer
    delay(beepDuration); // Keep the buzzer activated for the beep duration

    digitalWrite(buzzerPin, LOW); // Deactivate the buzzer
    delay(pauseDuration); // Pause for the pause duration

    // Calculate the number of beeps required to match the desired BPM
    int beepsPerMinute = 60000 / (beepDuration + pauseDuration);

    // Adjust the beep duration and pause duration to match the desired BPM
    beepDuration = 60000 / beepsPerMinute;
    pauseDuration = beepDuration;

    // Check if the current BPM matches the desired BPM
    if (beepsPerMinute == bpm) {
      break; // If yes, exit the loop
    }

    iteration++; // Increment the iteration counter
  }
}

void updateLCD(float bpm, float sampleRate, float voutAD8232) {
  mylcd.Fill_Screen(BLACK);

  if (bpm > 0) {
    // Display BPM
    mylcd.Set_Text_colour(WHITE);
    mylcd.Set_Text_Size(3);
    mylcd.Print_String("Heartbeat: ", 20, 80);

    mylcd.Set_Text_colour(GREEN);
    mylcd.Set_Text_Size(5);
    mylcd.Print_Number_Float(bpm, 1, 20, 150, '.', 4, ' ');

    mylcd.Set_Text_colour(WHITE);
    mylcd.Set_Text_Size(4);
    mylcd.Print_String("BPM", 200, 150);

    // Display sample rate
    mylcd.Set_Text_colour(BLUE);
    mylcd.Set_Text_Size(3);
    mylcd.Print_String("Sample Rate: ", 20, 300);

    mylcd.Set_Text_colour(WHITE);
    mylcd.Set_Text_Size(3);
    mylcd.Print_Number_Float(sampleRate, 1, 20, 350, '.', 4, ' ');

    mylcd.Set_Text_colour(WHITE);
    mylcd.Set_Text_Size(3);
    mylcd.Print_String("Hz", 120, 350);

    // Display voltage
    mylcd.Set_Text_colour(BLUE);
    mylcd.Set_Text_Size(3);
    mylcd.Print_String("Vout AD8232 ", 20, 400);

    mylcd.Set_Text_colour(WHITE);
    mylcd.Set_Text_Size(3);
    mylcd.Print_Number_Float(voutAD8232, 1, 20, 450, '.', 4, ' ');

    mylcd.Set_Text_colour(WHITE);
    mylcd.Set_Text_Size(3);
    mylcd.Print_String("V", 120, 450);
  } else {
    // No Signal Detected
    mylcd.Set_Text_colour(RED);
    mylcd.Set_Text_Size(3);
    mylcd.Print_String("No Signal Detected!", 20, 300);
  }
}

void filterBPM(float& bpm, float sampleRate) {
  if (sampleRate > 180 && bpm < 90) {
    IgnoreReading = true;
  }
}

void loop() {
  // Calculate the time difference between readings
  static unsigned long last_sample_time = 0;
  unsigned long current_time = millis();
  unsigned long time_diff = current_time - last_sample_time;

  // Read values from analog pins
  int v = analogRead(A15);
  float Vout_AD8232 = v * (5.0 / 1023.0); // Convert the analog reading to a voltage value

  int potValue = analogRead(A14);
  const int minValue = 120; // Pot minimum value in Hz
  const int maxValue = 300; // Pot maximum value in Hz
  float sampleRate = map(potValue, 0, 1023, minValue, maxValue); // Map the pot value to the desired sample rate range

  // Check if the time difference exceeds the desired sample rate
  if (time_diff >= (1000.0 / sampleRate)) {
    // Read the pulse value
    reading = digitalRead(LoPOS);

    // Check if the reading is above the upper threshold
    if (reading > UpperThreshold) {
      // Ignore the first pulse and set the first pulse time
      if (!FirstPulseDetected) {
        FirstPulseDetected = true;
        FirstPulseTime = millis();
      } else {
        // Calculate BPM and update LCD
        BPM = calculateBPM(sampleRate);
        updateLCD(BPM, sampleRate, Vout_AD8232);

        // Play heartbeat sound
        playHeartbeatSound(static_cast<int>(BPM));
      }

      IgnoreReading = false;
    } else {
      // Reset variables when the reading is below the upper threshold
      FirstPulseDetected = false;
      IgnoreReading = false;
    }

    last_sample_time = current_time;
  }
}
