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

uint32_t colors[] = {
	0xFF0000, // Red
	0x00FF00, // Green
	0x0000FF, // Blue
	0x32A8A0, // Cyan
	0xFFEA00, // Yellow
	0xCF067C, // Purple
	0xFF0800  // Orange
};

void handlePacketLoRa(int packetSize);
void handleLoRaCapsule(uint8_t packetId, uint8_t *dataIn, uint32_t len);
void handleUartCapsule(uint8_t packetId, uint8_t *dataIn, uint32_t len);

Adafruit_NeoPixel led(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800); // 1 led
LoopbackStream LoRaRxBuffer(1024);
CapsuleStatic LoRaCapsule(handleLoRaCapsule);
CapsuleStatic UartCapsule(handleUartCapsule);

void setup() {
	// Debug channel setup
	SERIAL_TO_PC.begin(SERIAL_TO_PC_BAUD);

	sleep(4);
	SERIAL_TO_PC.println("Startup Started");
	SERIAL_TO_PC.setTxTimeoutMs(0);

	// In standalone mode, UART_PORT = SERIAL_TO_PC. In motherboard mode, UART_PORT = Serial1
	// We thus only need to initialize UART_PORT in motherboard mode
	#if !MODE_STANDALONE
	UART_PORT.begin(UART_BAUD, 134217756U, 6, 5); // This for radioboard
	// UART_PORT.begin(UART_BAUD, 134217756U, 9, 46); // This for cmdIn
	#endif

	// Light up LED with the color corresponding to the board's config
	led.begin();
	led.fill(colors[INITIAL_LED_COLOR]);
	led.show();

	// LoRa setup
	SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
	LoRa.setPins(LORA_CS, LORA_RST, LORA_INT0);
	LoRa.setSPI(SPI);

	if (!LoRa.begin(LORA_FREQ)) SERIAL_TO_PC.println("Starting LoRa failed!");

	// Set LoRa parameters
	LoRa.setTxPower(LORA_POWER);
	LoRa.setSpreadingFactor(LORA_SF);
	LoRa.setSignalBandwidth(LORA_BW);
	LoRa.setCodingRate4(LORA_CR);
	LoRa.setPreambleLength(LORA_PREAMBLE_LEN);

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

/*
const uint16_t packetSizes[] = {8, 16, 32, 64, 128, 256}; // Packet sizes in bytes
const int numPacketSizes = sizeof(packetSizes) / sizeof(packetSizes[0]);
*/

const uint8_t packetSize = 0xFF; // Packet size in bytes
const unsigned long targetDataRate = 367000; // Target data rate in bits per second
const unsigned long totalBitsToSend = 10000;
int packet_number = 0;

void loop() {


	// Incoming data from the LoRa is processed.
	// While this is config-independant, it won't do much if the LoRa is not supposed to act as Rx.
	while (LoRaRxBuffer.available()) {
		LoRaCapsule.decode(LoRaRxBuffer.read());
  	}

	// Incoming data from the UART_PORT is processed.
	// In standalone mode, this data is sent by the computer. In motherboard mode, this data has been routed by the motherboard.
	while (UART_PORT.available()) {
		UartCapsule.decode(UART_PORT.read());
  	}

	// The visual clue is reset to the "config color".
	if ((millis() - lastPacketReceived) > LED_COLOR_TIME) {
		led.fill(colors[INITIAL_LED_COLOR]);
		led.show();
	}

	/*#ifdef AV_DOWNLINK
	int packetSize = LoRa.parsePacket();
    if (packetSize > 0) {
        SERIAL_TO_PC.print("Packet received (polling)! Size: ");
        SERIAL_TO_PC.println(packetSize);
        handlePacketLoRa(packetSize);
    }
	#endif*/
}


// Handler for raw LoRa Rx data
void handlePacketLoRa(int packetSize) {
	// Debug message
	SERIAL_TO_PC.println("Packet received");
	SERIAL_TO_PC.println(packetSize);
	

	// Incoming data is stored in the LoRaRxBuffer for decoding by Capsule
	for (int i = 0; i < packetSize; i++) {
		LoRaRxBuffer.write(LoRa.read());
	}
}

void handleLoRaCapsule(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
	// Nice visual clue to show that the board has received a new packet.
	lastPacketReceived = millis();
	uint32_t ledColor = colors[INITIAL_LED_COLOR+1];
	led.fill(ledColor);
	led.show();
	
	// The radio boards do not do any processing besides LoRa/Capsule encoding/decoding.
	// The packet thusly is routed through the UART_PORT.
	uint8_t* packetToSend = UartCapsule.encode(packetId,dataIn,len);
	
	av_downlink_t pkt;
  	memcpy(&pkt, dataIn, av_downlink_size);
	SERIAL_TO_PC.print("Packet number ="); SERIAL_TO_PC.println(pkt.packet_nbr);
	UART_PORT.write(packetToSend,UartCapsule.getCodedLen(len));

	delete[] packetToSend;
}

void handleUartCapsule(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
	lastPacketReceived = millis();
	uint32_t ledColor = colors[INITIAL_LED_COLOR+1];
	led.fill(ledColor);
	led.show();

	if(packetId == GSC_INTERNAL) {
		gsc_internal_t internal_packet{
			.rssi = LoRa.packetRssi(),
			.snr = LoRa.packetSnr(),
		};
		
		uint8_t* packetToSend = UartCapsule.encode(INTERNAL_CAPSULE_ID, (uint8_t*) &internal_packet, gsc_internal_size);
		UART_PORT.write(packetToSend, UartCapsule.getCodedLen(gsc_internal_size));
		delete[] packetToSend;
	}

	uint8_t* packetToSend = LoRaCapsule.encode(packetId,dataIn,len);
	LoRa.beginPacket();
	LoRa.write(packetToSend,LoRaCapsule.getCodedLen(len));
	LoRa.endPacket();
	LoRa.receive();

	delete[] packetToSend;
}