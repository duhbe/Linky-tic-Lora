/*
   Send meteo data and liny TIC data to the Lora 433Mhz gateway 
   as JSON messages.
   Examples: 
   mto1{"vbat": 4.15,"t1": 27.6,"t2": 28.0,"h": 78.1,"p": 99083.1,"a":  188.3}
   linky{"HCHC": 6225400,"HCHP": 8545391,"PTEC": 1,"IINST1": 1,"IINST2": 0,"IINST3": 1}

   Select board CubeCell board HTCC AB01
*/


#include <string.h>


#define DEBUG 1

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <SoftwareSerial.h>
#include <SparkFunCCS811.h>     //Click here to get the library: http://librarymanager/All#SparkFun_CCS811
#include <SparkFunBME280.h>     //Click here to get the library: http://librarymanager/All#SparkFun_BME280
#include <ClosedCube_HDC1080.h> //Click here to get the library: http://librarymanager/All#ClosedCube_HDC1080

// LED pin
#define kLEDPIN GPIO1
// Pin used to power up the sensors when required
#define kWAKPIN GPIO0


#define RF_FREQUENCY                                433000000 // Hz

#define TX_OUTPUT_POWER                             22        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
//  1: 250 kHz,
//  2: 500 kHz,
//  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
//  2: 4/6,
//  3: 4/7,
//  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

static RadioEvents_t RadioEvents;

// Wakeup every 20s
#define timetillwakeup 20000
static TimerEvent_t wakeUp;
uint8_t lowpower = 1;

// Create the software serial object
// used to receive the TIC frames
SoftwareSerial mySS(GPIO2, GPIO3);

// #define CCS811_ADDR 0x5B  //Default I2C Address
#define CCS811_ADDR 0x5A    //Alternate I2C Address

CCS811 myCCS811(CCS811_ADDR);
ClosedCube_HDC1080 myHDC1080;
BME280 myBMP280;

// ------------------------------------------------------------
// Blink the LED
// ------------------------------------------------------------
void isAlive(void)
{
  for (uint32_t i = 0; i < 5; i++) {
    digitalWrite(kLEDPIN, HIGH);
    delay(50);
    digitalWrite(kLEDPIN, LOW);
    delay(50);
  }
}


// ------------------------------------------------------------
// Initialize the meteo sensors
// ------------------------------------------------------------
void initSensors() {
  // Initialize the BMP280.
  // Note that the BMP has no humidity sensor (the BME has...).
  myBMP280.setI2CAddress(0x76);
  if ( ! myBMP280.beginI2C() ) {
#ifdef DEBUG
    Serial.println("Error initializing BMP280... Stopping.");
#endif
    while (1);      //No reason to go any further
  }

  myBMP280.setFilter(1); //0 to 4 is valid. Filter coefficient. See 3.4.4
  myBMP280.setStandbyTime(3); // Read every 250ms. 0 to 7 valid. Time between readings. See table 27.

  myBMP280.setTempOverSample(1); //0 to 16 are valid. 0 disables temp sensing. See table 24.
  myBMP280.setPressureOverSample(1); //0 to 16 are valid. 0 disables pressure sensing. See table 23.
  myBMP280.setMode(MODE_NORMAL); //MODE_SLEEP, MODE_FORCED, MODE_NORMAL is valid.

  // Init HDC1080
  myHDC1080.begin(0x40);

  /*
  //  CCS811 is ihnibited ( consumes too much power...)
  // and must stay on for a long time to provide correct data

    // Check connection to CCS811
    if ( ! myCCS811.begin())
    {
    Serial.println("Error initializing CCS811... Stopping.");
    while (1);      //No reason to go further
    }
  */

#ifdef DEBUG
  Serial.println("Sensors initialized!");
#endif
}

// ------------------------------------------------------------
// Prepare to go to sleep
// The actual action to switch to the sleep mode 
// is done in the main loop.
// ------------------------------------------------------------
void toSleep()
{
  // Power off sensors.
  digitalWrite(kWAKPIN, LOW);
  
  // Request switch to deep sleep...
  lowpower = 1;
  
  // And program next wake-up
  TimerSetValue( &wakeUp, timetillwakeup );
  TimerStart( &wakeUp );

  // Detach interrupt
  detachInterrupt(GPIO2);

  // Request switch to low power mode
  lowpower = 1;
}


char msg[128];

// ------------------------------------------------------------
// This function is called at wake-up time
// ------------------------------------------------------------
void onWakeUp()
{
  uint8_t cksum = 0;
  uint8_t i = 0;

  // Blink the LED
  isAlive();

  // Power on sensors
  digitalWrite(kWAKPIN, HIGH);
  // Initialize sensors
  initSensors();
  // Wait 10ms before reading values
  delay (10);

  /* CCS811 is inhibited (too much power)
    //Check if data is ready
    if (myCCS811.dataAvailable())
    {
    //If so, have the sensor read and calculate the results.
    //Get them later
    myCCS811.readAlgorithmResults();

    //compensating the CCS811 with humidity and temperature readings from the HDC1080
    myCCS811.setEnvironmentalData(myHDC1080.readHumidity(), myHDC1080.readTemperature());
    }
  */

  // Build the JSON meteo message
  char str[10];

  sprintf(msg, "mto1{");
  strcat(msg, "\"vbat\": ");
  dtostrf(getBatteryVoltage() / 1000.0, 4, 2, str);
  strcat(msg, str);
  strcat(msg, ",\"t1\": ");
  dtostrf(myHDC1080.readTemperature(), 4, 1, str);
  strcat(msg, str);
  strcat(msg, ",\"t2\": ");
  dtostrf(myBMP280.readTempC(), 4, 1, str);
  strcat(msg, str);
  strcat(msg, ",\"h\": ");
  dtostrf(myHDC1080.readHumidity(), 4, 1, str);
  strcat(msg, str);
  strcat(msg, ",\"p\": ");
  dtostrf(myBMP280.readFloatPressure(), 6, 1, str);
  strcat(msg, str);
  strcat(msg, ",\"a\": ");
  dtostrf(myBMP280.readFloatAltitudeMeters(), 6, 1, str);
  strcat(msg, str);
  strcat(msg, "}");

  cksum = 0;
  i = 0;
  while (msg[i])
    cksum += msg[i++];
  cksum = -cksum;
  msg[i] = cksum;
  msg[i + 1] = 0;

  // To read CO2 and TVOC (inhibited) 
  // myCCS811.getCO2()*100.0, myCCS811.getTVOC()*100.0

  #ifdef DEBUG
  Serial.println(msg);
  #endif
  
  // Send the meteo data
  Radio.Send( (uint8_t *)&msg, strlen(msg) );

  // Stop lowpower mode.
  lowpower = 0;

  // Setup soft serial
  mySS.begin(1200);

}


// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------

void setup() {
  boardInitMcu( );

  // Setup serial line
#ifdef DEBUG
  Serial.begin(115200);
#endif

  // Setup LED
  pinMode(kLEDPIN, OUTPUT);
  pinMode(kWAKPIN, OUTPUT);

  // Setup I2C
  Wire.begin();
  Wire.setClock(10000); // Increase to fast I2C speed!

  // Power sensors
  digitalWrite(kWAKPIN, HIGH);

  // Init sensors
  initSensors();

  // Init LoRa radio
  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );
  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                     LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                     LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                     true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

  // Blink the LED
  isAlive();

  // Initialize the timer
  TimerInit( &wakeUp, onWakeUp );

  // Go to sleep
  toSleep();

#ifdef DEBUG
  Serial.println("Go to main loop...");
#endif


}


// States of the frame decoding FSM
typedef enum { WAIT_SOF, FILL_FRAME } state_t;

// Tags to be decoded from last to first.
char *tags[] = {"IINST3", "IINST2", "IINST1", "PTEC", "HCHP", "HCHC"};

// Buffer used to decode received frames
char frame [32];

// ------------------------------------------------------------
//
// ------------------------------------------------------------

void loop()
{

  static uint8_t idx = 0;
  static state_t state = WAIT_SOF;
  char c;
  static uint8_t datctr = 5;

  if (lowpower) {
    // note that lowPowerHandler() runs six times before the mcu goes into lowpower mode;
    lowPowerHandler();
  }
  else
    while ( mySS.available() )
    {
      c = mySS.read() & 0x7F;
      #ifdef DEBUG
      Serial.print(c);
      Serial.print(" = ");
      Serial.println(c, HEX);
      #endif
      
      switch ( state ) {
        case WAIT_SOF: // We are waiting for a SOF
          if ( c == 0x0A ) // Start of frame
          {
            idx = 0;
            state = FILL_FRAME;
            #ifdef DEBUG
            Serial.println("In fill\n");
            #endif
          }
          break;
        case FILL_FRAME:
          if ( c == 0x0D ) // End of frame
          {
            #ifdef DEBUG
            Serial.println("Checking: ");
            #endif
            // Close the string
            frame[idx - 2] = 0;

            // Check checksum
            uint8_t cksum = 0x0;
            for (uint8_t i = 0; i < idx - 2; i++)
            { cksum += frame[i];

            }

            cksum = (cksum & 0x3F) + 0x20;

            // Is the frame OK?
            if ( cksum == frame[idx - 1])
            {
              // YES: the frame is OK.
              #ifdef DEBUG
              Serial.println("Valid frame");
              Serial.println(frame);
              #endif
              
              idx = 0;
              while (frame[idx++] != 0x20);
              
              // Extract the tag
              frame[idx - 1] = 0;

              if ( !strcmp(tags[datctr], frame) )
              {
                if ( datctr == 5 )
                  sprintf(msg, "linky{");
                else
                  strcat(msg, ",");

                strcat(msg, "\"");
                strcat(msg, tags[datctr]);
                strcat(msg, "\": ");
                if (datctr == 3)
                {
                  // Data from frame 3 (PTAC) is changed from 
                  // a char to an int
                  strcat(msg, (frame[idx + 1] == 'P') ? "1" : "0"  );
                }
                else
                {
                 // Remove the leading '0' if any (they are not supported by the JSON format)
                  uint8_t i = idx;
                  while( frame[i+1]) {
                    if (frame[i] == '0') {
                      frame[i] = 0x20; // Replace the '0' by a white space
                      i++;
                    }
                    else  
                      break;
                  }
                  strcat(msg, &frame[idx]);
                }

                #ifdef DEBUG
                Serial.println("<== ");
                Serial.println(tags[datctr]);
                #endif

                // Check if we have received all data
                if ( !datctr )
                {
                  // Yes, close the message and send it...
                  
                  strcat(msg, "}");
                  datctr = 5;

                  // Compute and append checksum
                  cksum = 0;
                  idx = 0;
                  while (msg[idx])
                    cksum += msg[idx++];
                  cksum = -cksum;
                  msg[idx] = cksum;
                  msg[idx + 1] = 0;


                #ifdef DEBUG
                  Serial.println(msg);
                #endif

                  // Send message
                  Radio.Send( (uint8_t *)&msg, strlen(msg) );

                  // Go to sleep mode
                  toSleep();

                }
                else
                  datctr--;

              }

            }
            else
            {
              Serial.println("Invalid frame ");
              Serial.println(frame);
            }

            state = WAIT_SOF;
            #ifdef DEBUG
            Serial.println("In wait\n");
            #endif
          }
          else if ( idx < 20 )
            frame[idx++] = c;
          else
          {
            state = WAIT_SOF;
            #ifdef DEBUG
            Serial.println("In wait\n");
            #endif
          }

          break;
        default:
          break;
      }
    }




}
