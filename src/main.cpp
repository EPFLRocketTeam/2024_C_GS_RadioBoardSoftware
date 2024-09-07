#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <capsule.h>
#include <LoRa.h>
#include <LoopbackStream.h>
#include <SPI.h>

#include "esp_event_loop.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"


#include "AsyncUDP.h"

#include "ERT_RF_Protocol_Interface/Protocol.h"
#include "config.h"

// When SEND_TO_DB is set to true in config.h, the radioboard will attempt to connect to the GSC's wifi.
// Once connected, it will store data in the GSC's influxDB instance. 
#if SEND_TO_DB
#include <WiFiMulti.h>
#include <InfluxDbClient.h>

WiFiMulti wifiMulti;
#define WIFI_SSID "ERT_GS_WIFI"
#define WIFI_PASSWORD "ERTGSRFBG"
AsyncUDP udp;

// InfluxDB server url. Don't use localhost, always server name or ip address.
// E.g. http://192.168.1.48:8086 (In InfluxDB 2 UI -> Load Data -> Client Libraries)

// #define INFLUXDB_URL "http://172.31.112.228:8086"
#define INFLUXDB_URL "http://gs.local:8086"

// InfluxDB 2 server or cloud API authentication token (Use: InfluxDB UI -> Load Data -> Tokens -> <select token>)
#define INFLUXDB_TOKEN "PJj8u6PZN1QVggN1lkhb1bkoX9rtegXEsdh8Mk9VeWw_mvqTobYfZJpXRM2T5Z_EDWziw1zN-MdUIEo6aGB5pQ==" // NUC token

// InfluxDB 2 organization id (Use: InfluxDB UI -> Settings -> Profile -> <name under tile> )
// #define INFLUXDB_ORG "Xstrato"
#define INFLUXDB_ORG "29306b5a85a43289"
#define INFLUXDB_BUCKET "Nordend"

// InfluxDB client instance
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN);


#ifdef GSE_DOWNLINK
Point GSETelemetry("GSETelemetry");

void setupInfluxDb();
void plotPoints(PacketGSE_downlink packet);
#endif

#ifdef AV_DOWNLINK
Point AVTelemetry("AVTelemetry");

void setupInfluxDb();
double compute_downrange(double rocket_lat, double rocket_lon);
void plotPoints(av_downlink_t packet);
#endif
#endif

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

void setup()
{
	// Debug channel setup
	SERIAL_TO_PC.begin(SERIAL_TO_PC_BAUD);

	sleep(5);
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

	if (DEBUG && !LoRa.begin(LORA_FREQ)) SERIAL_TO_PC.println("Starting LoRa failed!");

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

	#if (LORA_INVERSE_IQ)
	LoRa.enableInvertIQ();
	#else
	LoRa.disableInvertIQ();
	#endif

	LoRa.onReceive(handlePacketLoRa);
	LoRa.receive();
	LoRa.receive();
	  
	// WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
	#if SEND_TO_DB
	setupInfluxDb();
	#endif
 
	SERIAL_TO_PC.println("Startup Finished");
}

/*
const uint16_t packetSizes[] = {8, 16, 32, 64, 128, 256}; // Packet sizes in bytes
const int numPacketSizes = sizeof(packetSizes) / sizeof(packetSizes[0]);
*/

const uint8_t packetSize = 0xFF; // Packet size in bytes
const unsigned long targetDataRate = 367000; // Target data rate in bits per second
const unsigned long totalBitsToSend = 10000;

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
}

// Handler for raw LoRa Rx data
void handlePacketLoRa(int packetSize) {
	// Debug message
	SERIAL_TO_PC.println("Packet received");
	SERIAL_TO_PC.println(packetSize);

	// Incoming data is stored in the LoRaRxBuffer for decoding by Cpasule
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
	UART_PORT.write(packetToSend,UartCapsule.getCodedLen(len));
	delete[] packetToSend;
	
	// Should the database be enabled, any received packet is also broadcasted on wifi.
	#if SEND_TO_DB
	#ifdef AV_DOWNLINK
	av_downlink_t packet_debug;
	// uint8_t p[80];
	//av_downlink_t packet_debug;
	memcpy(&packet_debug, dataIn, 65);

	//SERIAL_TO_PC.println(packet_debug.gnss_alt);
	// memcpy(p, &dataIn, 80);
	SERIAL_TO_PC.println(packet_debug.gnss_alt);
	udp.broadcastTo(dataIn, len, 1235);
	#endif
	#endif
	
	// Should the database be enabled, current readings are added to the timeseries.
	#if SEND_TO_DB
		// InfluxDB stream
		#if GSE_DOWNLINK
			PacketGSE_downlink packetInflux;
			memcpy(&packetInflux, dataIn, packetGSE_downlink_size);
			plotPoints(packetInflux);
		#endif
		#if AV_DOWNLINK
			av_downlink_t packetInflux;
			memcpy(&packetInflux, dataIn, 65);
			plotPoints(packetInflux);
		#endif
	#endif
}

void handleUartCapsule(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
	lastPacketReceived = millis();
	uint32_t ledColor = colors[INITIAL_LED_COLOR+1];
	led.fill(ledColor);
	led.show();

	uint8_t* packetToSend = LoRaCapsule.encode(packetId,dataIn,len);
	LoRa.beginPacket();
	LoRa.write(packetToSend,LoRaCapsule.getCodedLen(len));
	LoRa.endPacket();
	LoRa.receive();
	LoRa.receive();
	delete[] packetToSend;
}

#if SEND_TO_DB
void setupInfluxDb() {
	// Connect WiFi
   
	WiFi.mode(WIFI_STA);
	wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);
	while (wifiMulti.run() != WL_CONNECTED) {
		SERIAL_TO_PC.print(".");
		delay(500);
	}

	SERIAL_TO_PC.println();
	SERIAL_TO_PC.println("Wifi connected :-)");

	// Set InfluxDB 1 authentication params
	// client.setConnectionParamsV1(INFLUXDB_URL, INFLUXDB_DB_NAME, INFLUXDB_USER, INFLUXDB_PASSWORD);

	// Add constant tags - only once
	// GSETelemetry.addTag("device", "GSEReceiver");
	
	// Check server connection
	if (client.validateConnection()) {
		SERIAL_TO_PC.print("Connected to InfluxDB: ");
		SERIAL_TO_PC.println(client.getServerUrl());
	} else {
		SERIAL_TO_PC.print("InfluxDB connection failed: ");
		SERIAL_TO_PC.println(client.getLastErrorMessage());
	}
}

#ifdef GSE_DOWNLINK
void plotPoints(PacketGSE_downlink packet) {
	// Point data
	GSETelemetry.addField("Tank Pressure", packet.tankPressure);
	GSETelemetry.addField("Tank Temperature", packet.tankTemperature);
	GSETelemetry.addField("Filling Pressure", packet.fillingPressure);
	GSETelemetry.addField("loadcellRaw", packet.loadcellRaw);
	GSETelemetry.addField("Fill N2O", packet.status.fillingN2O == ACTIVE ? 1.0:0.0);
	GSETelemetry.addField("Disconnect Active", (packet.disconnectActive == ACTIVE ? 1.0:0.0));
	GSETelemetry.addField("GSE Purge", (packet.status.vent == ACTIVE ? 1.0:0.0));
	if (!client.writePoint(GSETelemetry)) {
		Serial.print("InfluxDB write failed: ");
		Serial.println(client.getLastErrorMessage());
	}
	
}
#endif

#ifdef AV_DOWNLINK
void plotPoints(av_downlink_t packet) {
	// Point data
	AVTelemetry.addField("Lat", packet.gnss_lat);
	AVTelemetry.addField("Lon", packet.gnss_lon);
	AVTelemetry.addField("Downrange", compute_downrange(packet.gnss_lat, packet.gnss_lon));
	// SERIAL_TO_PC.println("here we are");
	AVTelemetry.addField("Alt", packet.gnss_alt);
	AVTelemetry.addField("Vertical Speed", packet.gnss_vertical_speed);
	AVTelemetry.addField("Tank Temperature", packet.tank_temp);
	AVTelemetry.addField("Chamber Pressure", packet.chamber_pressure);
	AVTelemetry.addField("N2O Pressure", packet.N2O_pressure);
	AVTelemetry.addField("AV State", packet.av_state);
	// SERIAL_TO_PC.println("here we are 2");
	AVTelemetry.addField("Valve Pressurize", packet.engine_state.pressurize== 1 ? 1.0 : 0.0);
	AVTelemetry.addField("Valve Purge", packet.engine_state.purge == 1 ? 1.0 : 0.0);
	AVTelemetry.addField("Valve Reserve", packet.engine_state.reserve== 1 ? 1.0 : 0.0);
	AVTelemetry.addField("Servo Fuel", packet.engine_state.servo_fuel== 1 ? 1.0 : 0.0);
	AVTelemetry.addField("Servo N2O", packet.engine_state.servo_N2O== 1 ? 1.0 : 0.0);
	AVTelemetry.addField("Vent Fuel", packet.engine_state.vent_fuel== 1 ? 1.0 : 0.0);
	AVTelemetry.addField("Vent N2O", packet.engine_state.vent_N2O == 1 ? 1.0 : 0.0);
	// SERIAL_TO_PC.println("here we are 3");
	if (!client.writePoint(AVTelemetry)) {
		Serial.print("InfluxDB write failed: ");
		Serial.println(client.getLastErrorMessage());
	}
	// SERIAL_TO_PC.print("InfluxDB write success");
} 

double compute_downrange(double rocket_lat, double rocket_lon) {
	float gs_lat = GS_LAT;
	float gs_lon = GS_LON;

	// haversineDistance
	double earthRadius = 6371000.0;
	gs_lat = gs_lat * M_PI / 180.0;
	gs_lon = gs_lon * M_PI / 180.0;

	rocket_lat = rocket_lat * M_PI / 180.0;
	rocket_lon = rocket_lon * M_PI / 180.0;

	double dlat = rocket_lat - gs_lat;
	double dlon = rocket_lon - gs_lon;

	double a = sin(dlat/2) * sin(dlat/2) + cos(gs_lat) * cos(rocket_lat) * sin(dlon/2) * sin(dlon/2);
	double c = 2 * atan2(sqrt(a), sqrt(1-a));
	double distance = earthRadius * c;
		
	return distance;
}
#endif
#endif