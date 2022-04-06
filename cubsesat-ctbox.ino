#include <Data.h>
#include <Wire.h>
#include <RF24.h>

#define CTBOX 1
#define ROLL_Y 2
#define PITCH_Y 3
#define YAW_Y 4
#define LASER 5
#define SATOP 6
#define LED_D 7
#define LED_P 8

#define NRF_CE 14
#define NRF_CS 15

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

uint64_t prevTxMillis = 0;
uint64_t currentMillis = 0;
uint64_t txDt = 0;

void requestEvent()
{
    uSatAngTx.angles = satAngles;
    Wire.write(uSatAngTx.b, NUM_ANGLES * SIZE_FLOAT);
}

void setup()
{
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
    pinMode(LASER,   INPUT);
    pinMode(SATOP,   INPUT);
    pinMode(LED_D,  OUTPUT);
    pinMode(LED_P,  OUTPUT);

    satAngles = {0.0, 0.0, 0.0};
}

void loop()
{
    currentMillis = millis();
    txDt = currentMillis - prevTxMillis;

    /* Get all inputs from physical devices here and store it in messageOut*/
    messageOut.laserEnable = digitalRead(LASER);
    messageOut.opMode = digitalRead(SATOP);

    roll = analogRead(ROLL_Y);
    pitch = analogRead(PITCH_Y);
    yaw = analogRead(YAW_Y);
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
}
