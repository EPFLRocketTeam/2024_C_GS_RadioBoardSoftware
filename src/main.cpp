#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <capsule.h>
#include <LoRa.h>
#include <LoopbackStream.h>
#include <SPI.h>

#include "ERT_RF_Protocol_Interface/Protocol.h"
#include "config.h"

#define LED_COLOR_TIME 100 // Color of the led will be changed for x ms each time a packet is received
static unsigned long lastPacketReceived = 0;
static uint8_t currentError = 0;

#if DEBUG
static unsigned long testtime = 0;
#endif

uint32_t colors[] = {
	0xFF0000, // Red
	0x00FF00, // Green
	0x0000FF, // Blue
	0x32A8A0, // Cyan
	0xFFEA00, // Yellow
	0xCF067C, // Purple
	0xFF0800  // Orange
};

void sendPacket();
void handlePacketLoRa(int packetSize);
void handleLoRaCapsule(uint8_t packetId, uint8_t *dataIn, uint32_t len);

Adafruit_NeoPixel led(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800); // 1 led
LoopbackStream LoRaRxBuffer(2048);
CapsuleStatic LoRaCapsule(handleLoRaCapsule);

void setup() {
	// Debug channel setup
	SERIAL_TO_PC.begin(SERIAL_TO_PC_BAUD);

	sleep(4);
	// SERIAL_TO_PC.println("Startup Started");
	// SERIAL_TO_PC.setTxTimeoutMs(0);

	// In standalone mode, UART_PORT = SERIAL_TO_PC. In motherboard mode, UART_PORT = Serial1
	// We thus only need to initialize UART_PORT in motherboard mode
	#if !MODE_STANDALONE
	UART_PORT.begin(UART_BAUD, 134217756U, 6, 5); // This for radioboard
	// UART_PORT.begin(UART_BAUD, 134217756U, 9, 46); // This for cmdIn
	UART_PORT.setTimeout(0);
	#endif

	// Light up LED with the color corresponding to the board's config
	led.begin();
	led.fill(colors[INITIAL_LED_COLOR]);
	led.show();

	// pinMode(ETHERNET_RW_PIN, OUTPUT); 
	// digitalWrite(ETHERNET_RW_PIN, RW_CTRL); // false for receiver mode, true for sender mode

	// LoRa setup
	SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
	LoRa.setPins(LORA_CS, LORA_RST, LORA_INT0);
	LoRa.setSPI(SPI);

	if (!LoRa.begin(RELAY_FREQ)) SERIAL_TO_PC.println("Starting LoRa failed!");

	// Set LoRa parameters
	LoRa.setTxPower(RELAY_POWER);
	LoRa.setSpreadingFactor(RELAY_SF);
	LoRa.setSignalBandwidth(RELAY_BW);
	LoRa.setCodingRate4(RELAY_CR);
	LoRa.setPreambleLength(RELAY_PREAMBLE_LEN);
	LoRa.setSyncWord(RELAY_SYNC_WORD);
	
	#if (LORA_CRC)
	LoRa.enableCrc(); // not necessary to work with miaou, even if miaou enbale it...:-|
	#else
	LoRa.disableCrc();
	#endif


	// ! \\ Ne fonctionne que en disableInvertIQ
	#if (LORA_INVERSE_IQ)	
		LoRa.enableInvertIQ();
	#else
		LoRa.disableInvertIQ();
	#endif
	

	LoRa.onReceive(handlePacketLoRa);
	LoRa.receive();
	LoRa.receive();
 
	SERIAL_TO_PC.println("Startup Finished");
}

void loop() {


	// Incoming data from the LoRa is processed.
	// While this is config-independant, it won't do much if the LoRa is not supposed to act as Rx.
	while (LoRaRxBuffer.available()) {
		LoRaCapsule.decode(LoRaRxBuffer.read());
  	}


	#if DEBUG
		if ((millis() - testtime) > 10000) {
			sendTestPacket();
			testtime = millis();		
		}
	#endif

	// UART_PORT.println("Here we send");


}

// Generic packet sending function
void sendTestPacket() {
  uint8_t packetData[9] = {0x1,0x2,0x49,0x1,0x2,0x3,0x4,0x5,0x6};

  LoRa.beginPacket();
  LoRa.write(packetData, sizeof(packetData));
  LoRa.endPacket();
}


// Handler for raw LoRa Rx data
void handlePacketLoRa(int packetSize) {

	if(packetSize != LoRaCapsule.getCodedLen(av_downlink_size) | LoRaCapsule.getCodedLen(gse_downlink_size))
		currentError |= ERROR_RX_NOT_A_DOWNLINK_PACKET;

	if(LoRaRxBuffer.availableForWrite() < 0)
		currentError |= ERROR_RX_BUFFER_OVERFLOW;

	// Debug message
	// SERIAL_TO_PC.println("Packet received");
	// SERIAL_TO_PC.println(packetSize);

	// Incoming data is stored in the LoRaRxBuffer for decoding by Capsule
	for (int i = 0; i < packetSize; i++) {
		LoRaRxBuffer.write(LoRa.read());
	}

}

void handleLoRaCapsule(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
	// Nice visual clue to show that the board has received a new packet.
	lastPacketReceived = millis();
	uint32_t ledColor = colors[millis()%5];
	led.fill(ledColor);
	led.show();
	
	LoRa.setFrequency(SEND_FREQ);
	LoRa.setTxPower(SEND_POWER);
	LoRa.setSpreadingFactor(SEND_SF);
	LoRa.setSignalBandwidth(SEND_BW);
	LoRa.setCodingRate4(SEND_CR);
	LoRa.setPreambleLength(SEND_PREAMBLE_LEN);

	uint8_t* new_pack;
	new_pack = LoRaCapsule.encode(packetId,dataIn,len);
	LoRa.write(new_pack,LoRaCapsule.getCodedLen(len));

	LoRa.setFrequency(RELAY_FREQ);
	LoRa.setTxPower(RELAY_POWER);
	LoRa.setSpreadingFactor(RELAY_SF);
	LoRa.setSignalBandwidth(RELAY_BW);
	LoRa.setCodingRate4(RELAY_CR);
	LoRa.setPreambleLength(RELAY_PREAMBLE_LEN);

	return;
}
