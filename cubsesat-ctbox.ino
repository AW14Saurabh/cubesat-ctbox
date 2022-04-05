#include <Data.h>
#include <Wire.h>
#include <RF24.h>

#define CTBOX 1

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

messageData_t messageOut;
uint8_t rxAddr[][6] = {"00001", "00002"};

uint64_t prevTxMillis = 0;
uint64_t currentMillis = 0;
uint64_t txDt = 0;

void requestEvent()
{
    uSatAngTx.angles = satAngles;
    for (int i = 0; i < NUM_ANGLES * SIZE_FLOAT; i ++)
        Wire.write(uSatAngTx.b[i]);
}

void setup()
{
    Wire.begin(CTBOX);
    Wire.onRequest(requestEvent);
    radio.begin();
    radio.setRetries(15, 15);
    radio.openWritingPipe(rxAddr[0]);
    radio.openReadingPipe(0, rxAddr[1]);
    radio.stopListening();
}

void loop()
{
    currentMillis = millis();
    txDt = currentMillis - prevTxMillis;

    /* Get all inputs from physical devices here and store it in messageOut*/

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
