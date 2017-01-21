#include <Arduino.h>
#include <avr/sleep.h>
#include <RF24.h>
#include <SPI.h>
#include <SpritzCipher.h>
#include <stdint.h>

enum DoorState
{
    DoorStateClosed,
    DoorStateOpen
};

struct radioPacket
{
    uint16_t magicNumber;
    uint8_t doorState;
    uint8_t sensorID;
    uint16_t vcc;
    uint8_t padding[26];
};

void goToSleepAndWakeOnEdge(uint8_t edgeToWake);
void sendStateChange(DoorState state);
uint16_t bandgap ();

const uint16_t kMagicNumber = 21212;
static const char encKey[] = "kRfxZE9WRMzsX5ns";
uint8_t writePipe[] = "2Pipa";

const uint8_t LED = 13;
const uint8_t PIN_SENSOR = 2;
const uint8_t PIN_RF24_CHIP_ENABLE = 7;
const uint8_t PIN_RF24_CHIP_SELECT = 8;

const uint8_t kSenderID = 1;

RF24 radio = RF24(PIN_RF24_CHIP_ENABLE, PIN_RF24_CHIP_SELECT);

void wake ()
{
    // cancel sleep as a precaution
    sleep_disable();
    // precautionary while we do other stuff
    detachInterrupt (0);
}   // end of wake

void setup ()
{
    Serial.begin(115200);
    digitalWrite (PIN_SENSOR, HIGH);  // enable pull-up

    radio.begin();
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_250KBPS);

    radio.openWritingPipe(writePipe);
    radio.powerDown();

    randomSeed(analogRead(A0));
}

inline uint8_t interruptEdgeForDoorState(DoorState &doorState)
{
    return doorState == DoorStateOpen ? FALLING : RISING;
}

inline DoorState currentDoorState()
{
    return digitalRead(PIN_SENSOR) == HIGH ? DoorStateOpen : DoorStateClosed;
}

void loop ()
{
    DoorState isDoorOpen = currentDoorState();

    Serial.println(isDoorOpen == DoorStateOpen ? "DOOR OPEN" : "DOOR CLOSED");

    sendStateChange(isDoorOpen);

    goToSleepAndWakeOnEdge(interruptEdgeForDoorState(isDoorOpen));

} // end of loop

void applyPadding(radioPacket *packet)
{
    size_t paddingLen = sizeof(packet->padding);
    for (uint8_t i = 0; i < paddingLen; i++)
    {
        packet->padding[i] = random(UINT8_MAX);
    }
}

void encryptPacket(radioPacket *packet, const uint8_t packetLen, const uint8_t *key, uint16_t keyLen, radioPacket *packetOut)
{
    spritz_ctx s_ctx;
    spritz_setup(&s_ctx, key, keyLen);
    spritz_crypt(&s_ctx, (const uint8_t*)packet, packetLen, (uint8_t*)packetOut);
}

void printPacket(radioPacket *packet)
{
    Serial.print(F("Magic number: "));
    Serial.println(packet->magicNumber);
    Serial.print(F("Door state: "));
    Serial.println(packet->doorState);
    Serial.print(F("SensorID: "));
    Serial.println(packet->sensorID);
    Serial.print(F("VCC: "));
    Serial.println(packet->vcc);
    Serial.print(F("packetSize: "));
    Serial.println(sizeof(radioPacket));
    Serial.println();
}

void sendStateChange(DoorState state)
{
    Serial.println("Sending state change");

    radioPacket packet;
    packet.magicNumber = kMagicNumber;
    packet.doorState = state;
    packet.sensorID = kSenderID;
    packet.vcc = bandgap();

    applyPadding(&packet);

    Serial.println("Decrypt:");
    printPacket(&packet);

    radioPacket encryptedPacket;
    encryptPacket(&packet, sizeof(radioPacket), (const uint8_t*)encKey, strlen(encKey), &encryptedPacket);

    radio.powerUp();

    if (!radio.write(&encryptedPacket, sizeof(radioPacket)))
    {
        Serial.println("Sending failed");
    }
    else
    {
        Serial.println("Sending complete");
    }

    radio.powerDown();

    delay(1000);
}

void lowBatteryWarning ()
{
    digitalWrite (LED, HIGH);
    delay (1);       // mS
    digitalWrite (LED, LOW);
    delay (999);
}

void goToSleepAndWakeOnEdge(uint8_t edgeToWake)
{
    Serial.println("Going to sleep...");

    // disable ADC
    ADCSRA = 0;

    set_sleep_mode (SLEEP_MODE_PWR_DOWN);
    sleep_enable();

    // Do not interrupt before we go to sleep, or the
    // ISR will detach interrupts and we won't wake.
    noInterrupts ();

    // will be called when pin D2 goes low
    attachInterrupt (digitalPinToInterrupt(PIN_SENSOR), wake, edgeToWake);
    EIFR = bit (INTF0);  // clear flag for interrupt 0

    // turn off brown-out enable in software
    // BODS must be set to one and BODSE must be set to zero within four clock cycles
    MCUCR = bit (BODS) | bit (BODSE);
    // The BODS bit is automatically cleared after three clock cycles
    MCUCR = bit (BODS);

    // We are guaranteed that the sleep_cpu call will be done
    // as the processor executes the next instruction after
    // interrupts are turned on.
    interrupts ();  // one cycle
    sleep_cpu ();   // one cycle
}

const long InternalReferenceVoltage = 1093;  // Adjust this value to your board's specific internal BG voltage

// Code courtesy of "Coding Badly" and "Retrolefty" from the Arduino forum
// results are Vcc * 100
// So for example, 5V would be 500.
uint16_t bandgap ()
{
    // REFS0 : Selects AVcc external reference
    // MUX3 MUX2 MUX1 : Selects 1.1V (VBG)
    ADMUX = bit (REFS0) | bit (MUX3) | bit (MUX2) | bit (MUX1);
    ADCSRA |= bit( ADSC );  // start conversion
    while (ADCSRA & bit (ADSC)){}

    uint16_t result = (((InternalReferenceVoltage * 1024) / ADC) + 5) / 10;
    ADCSRA = 0;
    return result;

}
