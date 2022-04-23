#include <Data.h>
#include <Wire.h>
#include <RF24.h>

#define CTBOX 1

#define ROLL_Y A0
#define PITCH_Y A1
#define YAW_Y A2

#define LASER 5
#define SAT_OP 4

#define LED_D 3
#define LED_P 2

#define NRF_CS 9
#define NRF_CE  10

#define NUM_ANGLES 3
#define SIZE_FLOAT 4
#define MIN_TX_TIME 150

//Union for I2C Communication to LabView
union uAngleBytes
{
    byte b[NUM_ANGLES * SIZE_FLOAT];
    angRPYData_t angles;
} uSatAngTx;

RF24 radio(NRF_CE, NRF_CS);
angRPYData_t satAngles;
uint32_t roll;
uint32_t pitch;
uint32_t yaw;

messageData_t messageOut;
uint8_t rxAddr[][6] = {"00001", "00002"};

uint64_t prevTxMillis = 0ul;
uint64_t currentMillis = 0ul;
uint64_t txDt = 0ul;

void requestEvent()
{
    uSatAngTx.angles = satAngles;
    Wire.write(uSatAngTx.b, NUM_ANGLES * SIZE_FLOAT);
}

void setup()
{
    Serial.begin(9600);

    Wire.begin(CTBOX);
    Wire.onRequest(requestEvent);

    pinMode(NRF_CE, OUTPUT);
    pinMode(NRF_CS, OUTPUT);
    radio.begin();
    radio.setRetries(15, 15);
    radio.openWritingPipe(rxAddr[0]);
    radio.openReadingPipe(0, rxAddr[1]);
    radio.stopListening();

    pinMode(ROLL_Y,  INPUT);
    pinMode(PITCH_Y, INPUT);
    pinMode(YAW_Y,   INPUT);
    pinMode(LASER,   INPUT_PULLUP);
    pinMode(SAT_OP,   INPUT_PULLUP);
    pinMode(LED_D,  OUTPUT);
    pinMode(LED_P,  OUTPUT);

    satAngles = {0.0, 0.0, 0.0};
    messageOut = {1, 1, {10, 20, 30}};
}

void loop()
{
    currentMillis = millis();
    txDt = currentMillis - prevTxMillis;

    // Get all inputs from physical devices here and store it in messageOut
    messageOut.laserEnable = digitalRead(LASER);
    messageOut.opMode = digitalRead(SAT_OP);

    Serial.println(String("Laser: ") + String(messageOut.laserEnable ? "Off" : "On"));
    Serial.println(String("Satellite Operation: ") + String(messageOut.opMode) + String(messageOut.opMode ? " Detumbling" : " Pointing"));
    digitalWrite(LED_D, !messageOut.opMode);
    digitalWrite(LED_P, messageOut.opMode);

    roll = analogRead(ROLL_Y);
    pitch = analogRead(PITCH_Y);
    yaw = analogRead(YAW_Y);

    Serial.println("Roll: " + String(roll) + "\tPitch: " + String(pitch) + "\tYaw: " + String(yaw));

    if (roll > 1000) satAngles.x += 0.2;
    else if (roll < 100) satAngles.x -= 0.2;
    if (pitch > 1000) satAngles.y += 0.2;
    else if (pitch < 100) satAngles.y -= 0.2;
    if (yaw > 1000) satAngles.z += 0.2;
    else if (yaw < 100) satAngles.z -= 0.2;
    messageOut.targetAngles = satAngles;

    radio.stopListening();
    if (txDt >= MIN_TX_TIME)
    {
        prevTxMillis = currentMillis;
        radio.write(&messageOut, sizeof(messageData_t));
    }

    radio.startListening();
    while(!radio.available());
    radio.read(&satAngles, sizeof(angRPYData_t));
    Serial.println("Sat Roll: " + String(satAngles.x) + "\tSat Pitch: " + String(satAngles.y) + "\tSat Yaw: " + String(satAngles.z));
}
