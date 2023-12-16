#include <DHT.h>
#include <SoftwareSerial.h>

#define DHTPIN 9      // Output pin for DHT-11
#define DHTTYPE DHT11 // DHT sensor type is DHT11

DHT dht(DHTPIN, DHTTYPE);
const int TRIG_PIN_1 = 6; // Arduino pin connected to the TRIG of the first ultrasonic sensor
const int ECHO_PIN_1 = 7; // Arduino pin connected to the ECHO of the first ultrasonic sensor
const int TRIG_PIN_2 = 2; // Arduino pin connected to the TRIG of the second ultrasonic sensor
const int ECHO_PIN_2 = 4; // Arduino pin connected to the ECHO of the second ultrasonic sensor
const int TRIG_PIN_3 = 8; // Arduino pin connected to the TRIG of the third ultrasonic sensor
const int ECHO_PIN_3 = 10; // Arduino pin connected to the ECHO of the third ultrasonic sensor
const int LED_PIN = 3;    // Arduino pin connected to the LED
const int BUZZER_PIN = 11; // Arduino pin connected to the buzzer
const int DISTANCE_THRESHOLD = 30; // Distance threshold in centimeters
SoftwareSerial bluetoothSerial(0, 1); // RX, TX

long soundspeed;

// Variables that will change
float duration_us_1, distance_cm_1;
float duration_us_2, distance_cm_2;
float duration_us_3, distance_cm_3;
unsigned long previousMillis = 0;  // Variable to store the time of the last message transmission
const long interval = 10000;  // Time interval (1 minute)

void setup() {
    Serial.begin(9600); // Initialize the serial port for communication with the PC
    bluetoothSerial.begin(9600); // Initialize the serial port for communication with the Bluetooth module
    bluetoothSerial.println("AT+CHARSET=UTF-8");

    dht.begin();
    pinMode(TRIG_PIN_1, OUTPUT); // Set Arduino pin to output mode for the first sensor
    pinMode(ECHO_PIN_1, INPUT);  // Set Arduino pin to input mode for the first sensor
    pinMode(TRIG_PIN_2, OUTPUT); // Set Arduino pin to output mode for the second sensor
    pinMode(ECHO_PIN_2, INPUT);  // Set Arduino pin to input mode for the second sensor
    pinMode(TRIG_PIN_3, OUTPUT); // Set Arduino pin to output mode for the third sensor
    pinMode(ECHO_PIN_3, INPUT);  // Set Arduino pin to input mode for the third sensor
    pinMode(LED_PIN, OUTPUT);    // Set Arduino pin to output mode for LED
    pinMode(BUZZER_PIN, OUTPUT); // Set Arduino pin to output mode for the buzzer
}

void loop() {
    unsigned long currentMillis = millis();  // Get the current time
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;  // Update the time of the last message transmission

        // Send a greeting message through the Bluetooth module
        bluetoothSerial.println("Hello!");

        // Display the message in the Serial Monitor
        Serial.println("Hello!");
    }

    delay(500); // Pause between readings

    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    // Check if readings failed and exit the program (to try again).
    if (isnan(temperature) || isnan(humidity)) {
        Serial.println("Failed to read from DHT sensor!");
        return;
    }

    // Generate a 10-microsecond pulse for the TRIG pin of the first sensor
    digitalWrite(TRIG_PIN_1, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN_1, LOW);

    // Measure the pulse duration on the ECHO pin of the first sensor
    duration_us_1 = pulseIn(ECHO_PIN_1, HIGH);

    // Generate a 10-microsecond pulse for the TRIG pin of the second sensor
    digitalWrite(TRIG_PIN_2, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN_2, LOW);

    // Measure the pulse duration on the ECHO pin of the second sensor
    duration_us_2 = pulseIn(ECHO_PIN_2, HIGH);

    // Generate a 10-microsecond pulse for the TRIG pin of the third sensor
    digitalWrite(TRIG_PIN_3, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN_3, LOW);

    // Measure the pulse duration on the ECHO pin of the third sensor
    duration_us_3 = pulseIn(ECHO_PIN_3, HIGH);

    // Calculate the distance for the first sensor
    distance_cm_1 = 0.017 * duration_us_1;

    // Calculate the distance for the second sensor
    distance_cm_2 = 0.017 * duration_us_2;

    // Calculate the distance for the third sensor
    distance_cm_3 = 0.017 * duration_us_3;

    if (distance_cm_1 < DISTANCE_THRESHOLD || distance_cm_2 < DISTANCE_THRESHOLD || distance_cm_3 < DISTANCE_THRESHOLD) {
        digitalWrite(LED_PIN, HIGH);    // Turn on the LED
        digitalWrite(BUZZER_PIN, HIGH); // Turn on the buzzer
    } else {
        digitalWrite(LED_PIN, LOW);     // Turn off the LED
        digitalWrite(BUZZER_PIN, LOW);  // Turn off the buzzer
    }

    // Display the values in the Serial Monitor
    Serial.print("Distance 1: ");
    Serial.print(distance_cm_1);
    Serial.print(" cm\t");

    Serial.print("Distance 2: ");
    Serial.print(distance_cm_2);
    Serial.print(" cm\t");
    
    Serial.print("Distance 3: ");
    Serial.print(distance_cm_3);
    Serial.print(" cm\t");

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" °C\t");

    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");

    // Send data via Bluetooth
    bluetoothSerial.print("D1:");
    bluetoothSerial.print(distance_cm_1);
    bluetoothSerial.print(", D2:");
    bluetoothSerial.print(distance_cm_2);
    bluetoothSerial.print(", D3:");
    bluetoothSerial.print(distance_cm_3);
    bluetoothSerial.print(", Temp:");
    bluetoothSerial.print(temperature);
    bluetoothSerial.print(", Hum:");
    bluetoothSerial.println(humidity);
}
