#define DEBUG 				false

#define MODE_STANDALONE   true  // otherwise, connected to motherboard

#define SEND_TO_DB      false



#if MODE_STANDALONE
#define UART_PORT   		USBSerial
#else
#define UART_PORT   		Serial1
#endif
#define UART_BAUD   		115200

#define SERIAL_TO_PC        USBSerial
#define SERIAL_TO_PC_BAUD   115200		// baudrate not considered as USB speed

// PIN/GPIO Definition on Radio Module ERT

#define LORA_SCK                42
#define LORA_MOSI               44
#define LORA_MISO               43
#define LORA_CS                 41
#define LORA_INT0               21
#define LORA_INT5               39
#define LORA_RST                -1  

#define NEOPIXEL_PIN            18

#ifdef UPLINK
  #define LORA_FREQ         UPLINK_FREQUENCY
  #define LORA_POWER        UPLINK_POWER
  #define LORA_BW           UPLINK_BW
  #define LORA_SF           UPLINK_SF
  #define LORA_CR           UPLINK_CR
  #define LORA_PREAMBLE_LEN UPLINK_PREAMBLE_LEN
  #define LORA_SYNC_WORD    UPLINK_SYNC_WORD
  #define LORA_CRC          UPLINK_CRC
  #define LORA_INVERSE_IQ   UPLINK_INVERSE_IQ

  #define INITIAL_LED_COLOR 0

  #define RADIOMODULE_NAME "UPLINK"
#elif AV_DOWNLINK
  #define LORA_FREQ         AV_DOWNLINK_FREQUENCY
  #define LORA_POWER        AV_DOWNLINK_POWER
  #define LORA_BW           AV_DOWNLINK_BW
  #define LORA_SF           AV_DOWNLINK_SF
  #define LORA_CR           5
  #define LORA_PREAMBLE_LEN AV_DOWNLINK_PREAMBLE_LEN
  #define LORA_SYNC_WORD    AV_DOWNLINK_SYNC_WORD
  #define LORA_CRC          false //AV_DOWNLINK_CRC
  #define LORA_INVERSE_IQ   AV_DOWNLINK_INVERSE_IQ
  #define INITIAL_LED_COLOR 1

  #define RADIMODULE_NAME "AV_DOWNLINK_1"
#elif GSE_DOWNLINK
  #define LORA_FREQ         GSE_DOWNLINK_FREQUENCY
  #define LORA_POWER        GSE_DOWNLINK_POWER
  #define LORA_BW           GSE_DOWNLINK_BW
  #define LORA_SF           GSE_DOWNLINK_SF
  #define LORA_CR           GSE_DOWNLINK_CR
  #define LORA_PREAMBLE_LEN GSE_DOWNLINK_PREAMBLE_LEN
  #define LORA_SYNC_WORD    GSE_DOWNLINK_SYNC_WORD
  #define LORA_CRC          GSE_DOWNLINK_CRC
  #define LORA_INVERSE_IQ   GSE_DOWNLINK_INVERSE_IQ

  #define INITIAL_LED_COLOR 2

  #define RADIOMODULE_NAME "GSE_DOWNLINK"
#endif

#define LORA_CURRENT_LIMIT 120