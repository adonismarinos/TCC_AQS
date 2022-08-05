#include <Thermistor.h>
#include <arduino_lmic.h>
#include <arduino_lmic_hal_boards.h>
#include <arduino_lmic_hal_configuration.h>
#include <arduino_lmic_lorawan_compliance.h>
#include <arduino_lmic_user_configuration.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

/************************MQ5sensor************************************/
/************************Hardware Related Macros************************************/
#define         MQ5PIN                       (0)      //define which analog input channel you are going to use
#define         RL_VALUE_MQ5                 (1)      //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR_MQ5      (6.455)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                      //which is derived from the chart in datasheet
                                                      
/************************mq2sensor************************************/
/************************Hardware Related Macros************************************/
#define         MQ2PIN                       (1)     //define which analog input channel you are going to use
#define         RL_VALUE_MQ2                 (1)     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR_MQ2      (9.577)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                     //which is derived from the chart in datasheet

/************************MQ7sensor************************************/
/************************Hardware Related Macros************************************/
#define         MQ7PIN                       (2)      //define which analog input channel you are going to use
#define         RL_VALUE_MQ7                 (1)      //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR_MQ7      (26.09)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                      //which is derived from the chart in datasheet

/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
                                                     //normal operation

/**********************Application Related Macros**********************************/
#define         GAS_HYDROGEN                  (0)
#define         GAS_LPG                       (1)
#define         GAS_METHANE                   (2)
#define         GAS_CARBON_MONOXIDE           (3)
#define         GAS_ALCOHOL                   (4)
#define         GAS_SMOKE                     (5)
#define         GAS_PROPANE                   (6)
#define         accuracy                      (0)   //for linearcurves

/*****************************Globals************************************************/
float           RoMQ5 = 0;                            //Ro is initialized to 10 kilo ohms
float           RoMQ2 = 0;                            //Ro is initialized to 10 kilo ohms
float           RoMQ7 = 0;                            //Ro is initialized to 10 kilo ohms
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0x5E, 0x17, 0x05, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0xD8, 0xE9, 0x27, 0xCB, 0x1A, 0x2A, 0x3F, 0x56, 0xC5, 0x5C, 0x3C, 0x14, 0xD0, 0x51, 0xE9, 0xB0 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);
}

static osjob_t sendjob;


// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, LMIC_UNUSED_PIN},
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("artKey: ");
              for (int i=0; i<sizeof(artKey); ++i) {
                Serial.print(artKey[i], HEX);
              }
              Serial.println("");
              Serial.print("nwkKey: ");
              for (int i=0; i<sizeof(nwkKey); ++i) {
                Serial.print(nwkKey[i], HEX);
              }
              Serial.println("");
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

struct leituras_t
{
  int MQ_5;
  int MQ_2;
  int MQ_7;
  int MICS;
};

leituras_t leitura;

void do_send(osjob_t* j){
  
  Serial.println("Measuring pollutants...");
  leitura.MQ_5 = MQ5GetGasPercentage(MQRead(MQ5PIN)/RoMQ5,GAS_CARBON_MONOXIDE);
  leitura.MQ_2 = MQ2GetGasPercentage(MQRead(MQ2PIN)/RoMQ2,GAS_SMOKE);
  leitura.MQ_7 = MQ7GetGasPercentage(MQRead(MQ7PIN)/RoMQ7,GAS_METHANE);
  leitura.MICS = analogRead(A3);
    
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
      // Prepare upstream data transmission at the next possible time.
      LMIC_setTxData2(1, (char*)&leitura, sizeof(leituras_t), 0);
      Serial.println(F("Packet queued"));
  }
  Serial.print("CO Value = "); 
  Serial.println(leitura.MQ_5); 
  Serial.print("Smoke Value = ");
  Serial.println(leitura.MQ_2);
  Serial.print("Methane = ");
  Serial.println(leitura.MQ_7);
  Serial.print("NO2 Value = ");
  Serial.println(leitura.MICS);
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {    
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  Serial.print("MQ5: Calibrating...\n");                
  RoMQ5 = MQCalibration(MQ5PIN);                       //Calibrating the sensor. Please make sure the sensor is in clean air 
                                                    //when you perform the calibration                    
  Serial.print("MQ5: Calibration is done...\n"); 
  
  Serial.print("MQ2: Calibrating...\n");                
  RoMQ2 = MQCalibration(MQ2PIN);                       //Calibrating the sensor. Please make sure the sensor is in clean air 
                                                    //when you perform the calibration                    
  Serial.print("MQ2: Calibration is done...\n"); 
  
  Serial.print("MQ7: Calibrating...\n");                
  RoMQ7 = MQCalibration(MQ7PIN);                       //Calibrating the sensor. Please make sure the sensor is in clean air 
                                                    //when you perform the calibration                    
  Serial.print("MQ7: Calibration is done...\n"); 
  Serial.print("RoMQ5=");
  Serial.print(RoMQ5);
  Serial.print("kohm");
  Serial.print("\n");
  Serial.print("RoMQ2=");
  Serial.print(RoMQ2);
  Serial.print("kohm");
  Serial.print("\n");
  Serial.print("RoMQ7=");
  Serial.print(RoMQ7);
  Serial.print("kohm");
  Serial.print("\n");
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  LMIC_setAdrMode(false);
  LMIC_setLinkCheckMode(0);
  LMIC_setDrTxpow(DR_SF9,14);
  LMIC_selectSubBand(1);

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

// the loop routine runs over and over again forever:
void loop() {
  os_runloop_once();
}

float MQCalibration(int mq_pin)
{
  int i;
  float RS_AIR_val=0,r0;

  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {                     //take multiple samples
    RS_AIR_val += MQResistanceCalculation(analogRead(mq_pin), mq_pin);
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  RS_AIR_val = RS_AIR_val/CALIBARAION_SAMPLE_TIMES;              //calculate the average value

  if (mq_pin == 0){
    r0 = RS_AIR_val/RO_CLEAN_AIR_FACTOR_MQ5;                      //RS_AIR_val divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                                 //according to the chart in the datasheet 
    }
  else if (mq_pin == 1){
    r0 = RS_AIR_val/RO_CLEAN_AIR_FACTOR_MQ2;                      //RS_AIR_val divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                                 //according to the chart in the datasheet 
    }
  else if (mq_pin == 2){
    r0 = RS_AIR_val/RO_CLEAN_AIR_FACTOR_MQ7;                      //RS_AIR_val divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                                 //according to the chart in the datasheet 
    }   
  
  return r0; 
}

float MQRead(int mq_pin)
{
  int i;
  float rs=0;

  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin),mq_pin);
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs/READ_SAMPLE_TIMES;

  return rs;  
}

int MQ5GetGasPercentage(float rs_ro_ratio, int gas_id)
{ 
  if ( accuracy == 0 ) {
  if ( gas_id == GAS_HYDROGEN ) {
    return (pow(10,((-3.986*(log10(rs_ro_ratio))) + 3.075)));
  } else if ( gas_id == GAS_LPG ) {
    return (pow(10,((-2.513*(log10(rs_ro_ratio))) + 1.878)));
  } else if ( gas_id == GAS_METHANE ) {
    return (pow(10,((-2.554*(log10(rs_ro_ratio))) + 2.265 )));
  } else if ( gas_id == GAS_CARBON_MONOXIDE ) {
    return (pow(10,((-6.900*(log10(rs_ro_ratio))) + 6.241)));
  } else if ( gas_id == GAS_ALCOHOL ) {
    return (pow(10,((-4.590*(log10(rs_ro_ratio))) + 4.851)));
  }   
  }
} 

int MQ2GetGasPercentage(float rs_ro_ratio, int gas_id)
{ 
  if ( accuracy == 0 ) {
  if ( gas_id == GAS_HYDROGEN ) {
    return (pow(10,((-2.109*(log10(rs_ro_ratio))) + 2.983)));
  } else if ( gas_id == GAS_LPG ) {
    return (pow(10,((-2.123*(log10(rs_ro_ratio))) + 2.758)));
  } else if ( gas_id == GAS_METHANE ) {
    return (pow(10,((-2.622*(log10(rs_ro_ratio))) + 3.635)));
  } else if ( gas_id == GAS_CARBON_MONOXIDE ) {
    return (pow(10,((-2.955*(log10(rs_ro_ratio))) + 4.457)));
  } else if ( gas_id == GAS_ALCOHOL ) {
    return (pow(10,((-2.692*(log10(rs_ro_ratio))) + 3.545)));
  } else if ( gas_id == GAS_SMOKE ) {
    return (pow(10,((-2.331*(log10(rs_ro_ratio))) + 3.596)));
  } else if ( gas_id == GAS_PROPANE ) {
    return (pow(10,((-2.174*(log10(rs_ro_ratio))) + 2.799)));
  }    
  }
} 

int MQ7GetGasPercentage(float rs_ro_ratio, int gas_id)
{ 
  if ( accuracy == 0 ) {
  if ( gas_id == GAS_CARBON_MONOXIDE ) {
    return (pow(10,((-1.525*(log10(rs_ro_ratio))) + 1.994)));
  } else if ( gas_id == GAS_HYDROGEN ) {
    return (pow(10,((-1.355*(log10(rs_ro_ratio))) + 1.847)));
  } else if ( gas_id == GAS_LPG ) {
    return (pow(10,((-7.622*(log10(rs_ro_ratio))) + 8.919 )));
  } else if ( gas_id == GAS_METHANE ) {
    return (pow(10,((-11.01*(log10(rs_ro_ratio))) + 14.32)));
  } else if ( gas_id == GAS_ALCOHOL ) {
    return (pow(10,((-14.72*(log10(rs_ro_ratio))) + 19.31)));
  }   
  } 
}

float MQResistanceCalculation(int raw_adc, int mq_pin)
{
  if (mq_pin == 0){
    return ( ((float)RL_VALUE_MQ5*(1023-raw_adc)/raw_adc));
    }  
  else if (mq_pin == 1){
    return ( ((float)RL_VALUE_MQ2*(1023-raw_adc)/raw_adc));
    }
  else if (mq_pin == 2){
    return ( ((float)RL_VALUE_MQ7*(1023-raw_adc)/raw_adc));
    }  
}
