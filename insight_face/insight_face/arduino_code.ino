#include <Arduino.h>

const int ledPin1 = 9; // Connect LED 1 to pin 9
const int ledPin2 = 10; // Connect LED 2 to pin 10
const unsigned long blinkInterval = 500; // 500ms for LED blinking

bool ledState = LOW;
unsigned long previousMillis = 0;
bool isFlashing = false;

void setup() {
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);

  Serial.begin(9600); // Initialize serial communication
  while (!Serial) {
    ; // Wait for serial port to connect (for Leonardo and Micro)
  }
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove any extra spaces or newlines

    if (command == "ON" && !isFlashing) {
      isFlashing = true;
      Serial.println("LED Flashing Started");
    } else if (command == "OFF") {
      stopFlashing();
      Serial.println("LED Flashing Stopped");
    }
  }

  if (isFlashing) {
    unsigned long currentMillis = millis();

    // Blink LEDs
    if (currentMillis - previousMillis >= blinkInterval) {
      previousMillis = currentMillis;
      ledState = !ledState;
      digitalWrite(ledPin1, ledState);
      digitalWrite(ledPin2, ledState);
    }
  }
}

void stopFlashing() {
  isFlashing = false;
  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);
}







// #include <Arduino.h>

// const int ledPin1 = 9; // Connect LED 1 to pin 9
// const int ledPin2 = 10; // Connect LED 2 to pin 10
// const unsigned long blinkInterval = 500; // 500ms for LED blinking

// bool ledState = LOW;
// unsigned long previousMillis = 0;
// bool isFlashing = false;
// unsigned long flashStartTime = 0;
// const unsigned long flashDuration = 10000; // Flash duration: 10 seconds

// void setup() {
//   pinMode(ledPin1, OUTPUT);
//   pinMode(ledPin2, OUTPUT);
//   digitalWrite(ledPin1, LOW);
//   digitalWrite(ledPin2, LOW);

//   Serial.begin(9600); // Initialize serial communication
//   while (!Serial) {
//     ; // Wait for serial port to connect (for Leonardo and Micro)
//   }
// }

// void loop() {
//   if (Serial.available() > 0) {
//     String command = Serial.readStringUntil('\n');
//     command.trim(); // Remove any extra spaces or newlines

//     if (command == "ON" && !isFlashing) {
//       isFlashing = true;
//       flashStartTime = millis();
//       Serial.println("LED Flashing Started");
//     } else if (command == "OFF") {
//       stopFlashing();
//       Serial.println("LED Flashing Stopped");
//     }
//   }

//   if (isFlashing) {
//     unsigned long currentMillis = millis();

//     // Check if 10 seconds have passed
//     if (currentMillis - flashStartTime >= flashDuration) {
//       stopFlashing();
//       Serial.println("LED Flashing Automatically Stopped After 10 Seconds");
//     } else {
//       // Blink LEDs
//       if (currentMillis - previousMillis >= blinkInterval) {
//         previousMillis = currentMillis;
//         ledState = !ledState;
//         digitalWrite(ledPin1, ledState);
//         digitalWrite(ledPin2, ledState);
//       }
//     }
//   }
// }

// void stopFlashing() {
//   isFlashing = false;
//   digitalWrite(ledPin1, LOW);
//   digitalWrite(ledPin2, LOW);
// }
