/* 
 * Geiger counter with OLED display and RF transmission of counter data
 * 
 * To generate 400 V from 5V supply for a DOB-50 Geiger-Muller tube the circuit 
 * Uses an chinese made transformer coil. 
 *   https://es.aliexpress.com/item/32925211741.html?spm=a2g0s.9042311.0.0.274263c0CGA7vR
 *   https://www.ebay.es/itm/10KV-High-Voltage-Transformer-High-frequency-Booster-Coil-Inverter-20x17x15mm/172438919607?ssPageName=STRK%3AMEBIDX%3AIT&_trksid=p2057872.m2749.l2648
 *   
 * The coil is driven with an MOSFET PWM driver module. A 3 Ohm - 4.7 Ohm resistor
 * is used in series with the coil (outside this range is difficult or impossible
 * to get 400V stable output by using only the PWM duty cycle.
 * 
 * A full bridge rectifier and a peak detector (diode + capacitor is used to rectify AC 
 * and reduce ripple. Diodes used: CL77 * fast-switching high voltage and high voltage 
 * 22nF capacitor. 
 * 
 * The PWM is configured at about 31 kHz (see the setup function). A lower one, maybe at 
 * 20 kHz gives better performance, but 31 kHz is easy to configure for Uno and Nano. 20k 
 * or more is neededto get 400V with a reasonable power dissipation.
 * 
 * Warning, the output voltage depends on several parameters (input voltage, PWM fequency 
 * and duty cycle, series resistor value, MOSFET transistor and driver circuit used, so a
 * tuning mechanism is needed. In this circuit, a SETUP button activates the tuning cycle
 * and the analog input from a potentiomenter is used to set the PWM duty cycle. A tunable
 * voltage regulator might be needed to set the output voltage with more precision.
 * 
 * MOSGET PWM Driver.
 * Supplier 1, with schematics:
 *  https://protosupplies.com/product/high-power-dual-mosfet-switch-module/
 *  (The 10K R1 should be checked, I measured 1K)
 * AliExpress:
 *  https://es.aliexpress.com/item/32803005422.html?spm=a2g0s.9042311.0.0.274263c0CLIwze
 *    
 * Can bre replaced with a suitable MOSFET equivalent, but it has to be a low on-resistance
 * one. An 8 mOhm IFR3205 MOSFET has been tested succesfully (a 100-ohm resistor between
 * PWM output and gate might help to enhance stability).
 * 
 * The Geiger-Muller tube has almost infinite resistance that drops to very low 
 * when a particle is detected. It is connected with two 10M and 100K resistors
 * creating avoltage divider with an about 1:100 ratio. Voltage is measured in the 
 * divider:
 * 
 * 400V ----[10M]---(DOB-50GM)--+--[100K]---GND
 *                              |
 *                           INPUT PIN
 *                           
 * 
 * Standard background to be expected is about 1-3 counts per minute, with very high
 * variability (0-0-2-0-8-0-2-0-1-1-2 for example).
 * 
 */
/////////////////////////////////////////////////////////////////////////
//////////////////////   CONFIGURATION   ////////////////////////////////
/////////////////////////////////////////////////////////////////////////

#define SENSOR_TYPE SS_GEIGER_CPM
#define SENSOR_NAME "GM1"
#define SENSOR_ID      1

#define GEIGER_HARDWARE_VERSION 3   // 1-FIRST PROTOTYPE, 2-SECOND PROTOTYPE FIRST PCB PRODUCED), 3-3RD VERSION 

#if (GEIGER_HARDWARE_VERSION == 1)   // Fist prototype pinout
 const int BUTTON_PIN        =   3;  // Mode change interrupr button (must be 3 or 2)
 const int DETECT_PIN        =   2;  // Geiger counter interrupts (MUST be 2 or 3)
#warning "WARNING\n** USING *OLD* PINOUT"
#else
 const int BUTTON_PIN        =   2;  // Mode change interrupr button (must be 3 or 2)
 const int DETECT_PIN        =   3;  // Geiger counter interrupts (MUST be 2 or 3)
#warning "*************************\n** USING *NEW* PINOUT  ***************************"
#endif

 const int PWMOUT_PIN        =   9;  // PWM For inverter driver
 const int LED_PIN           =  13;  // Led Pin
 const int DUTY_POT_PIN      =  A0;  // PWM configuration potentiomenter input 
 //
 const int INITIAL_PWM_DUTY  =  38;  // Initial duty cycle of HV coil pwm
 const int MAX_PWM_DUTY      =  90;  // Maxium value
 const int MIN_PWM_DUTY      =   1;  // Minimum
 const int DUTY_EEPROM_ADDR  =   2;  // EEPROM address where the last PWM seting is stored
 //
 const int OLED_RESET       =   4;  // Adafruit SSD1306 OLED display reset pin
 const int OLED_ROTATION    =   0;  // 0=0, 1=90, 2=180, 3=270 degrees
 //
 const int NODE_EEPROM_ADDR =   1;  // EEPROM address where the RF node id is taken from (See EEPROM_programmer.ino)
 const int RF_OUTPUT_PIN   =   11;   // Interface to de RF transmitter 
 //

// 20 secs between extra resends
#define ADDITIONAL_RESEND_INTERVAL 20000

// UNCOMENT TO ENABLE DEBUGGING THROUGH SERIAL 
#define SERIAL_DEBUG
#define _GEIGER_VERSION 3
 
/////////////////////////////////////////////////////////////////////////
/////////////////////////////// DISPLAY /////////////////////////////////
/////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 display(OLED_RESET);
#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

/////////////////////////////// BITMAPS ////////////////////////////////
#include "bitmaps.h"


/////////////////////////////////////////////////////////////////////////
////////////////////////  433 MHZ BROADCAST /////////////////////////////
/////////////////////////////////////////////////////////////////////////

// Sources: 
//   mchr3k (Manchester encoding)
//      https://github.com/mchr3k/arduino-libs-manchester
//   Andreas Rohner:
//      https://github.com/zeitgeist87/RFTransmitter
//      https://github.com/zeitgeist87/RFReceiver  (used in receiver)
//
#define RF433_ZEITGEIST     1   // Andreas Rohner (zeitgeist87) library
#define RF433_MANCHESTER    2   // mchr3k Library 

#define RF433_BROADCAST RF433_MANCHESTER

#if RF433_BROADCAST == RF433_ZEITGEIST
//-------------------------------------------------------- zeitgeist87's
#include <RFTransmitter.h>
unsigned int NODE_ID = EEPROM[NODE_EEPROM_ADDR];
unsigned int BACKOFF_DELAY = 350; 
unsigned int RESEND_COUNT = 1; // RESENTS  
unsigned int PULSE_LENGTH = 333;  // Pulse length in microseconds must agree with receiver

RFTransmitter transmitter(RF_OUTPUT_PIN, NODE_ID, PULSE_LENGTH, BACKOFF_DELAY, RESEND_COUNT);

#elif RF433_BROADCAST == RF433_MANCHESTER
//--------------------------------------------------------- mchr3k's 
#include <Manchester.h>
#define MANCHESTER_SPEED MAN_600

#else
#warning "*** RADIO BROADCAST NOT CONFIGURED ***"
#endif


/////////////////////////////////////////////////////////////////////////
////////////////////////  SENSOR PACKET LIBRARY /////////////////////////
/////////////////////////////////////////////////////////////////////////
#ifdef RF433_BROADCAST
#include <sensed.h>

sensed sensor_data(SENSOR_TYPE, SENSOR_NAME, SENSOR_ID);

byte *dataframe = NULL;  // not null means at least one frame sent, may resend

// Packet resend interval
unsigned long lastMillisResend = millis();    
unsigned const int MaxMillisResend = ADDITIONAL_RESEND_INTERVAL; // resend every 20 seconds
#endif

/////////////////////////////////////////////////////////////////////////
////////////////////////  EEPROM (CONFGIG)  /////////////////////////////
/////////////////////////////////////////////////////////////////////////

#include <EEPROM.h>

/////////////////////////////////////////////////////////////////////////
///////////////////////// GEIGER COUNTER GLOBALS ////////////////////////
/////////////////////////////////////////////////////////////////////////

// Geiger tube counter interrupt handling
unsigned long counterStart = 0;               // microseconds counter started
volatile unsigned int  counterCount = 0;      // Number of interrupts received
volatile unsigned long counterTime  = 0;      // Debounce timer

// Mode button interrupt handling
volatile bool configMode               = false;    // Enter configuration mode
volatile unsigned long inactiveMillis  = 0;        // ms config pot not changed
const unsigned long maxInactivePot     = 15000;    // exit config if no pot change in 15s

#if (GEIGER_HARDWARE_VERSION == 3)   // VERSION 3 USES POLLING
//-- 13/01/2021 CHANGED INTERRUPT TO POLLING W/DEBOUNCING FOR MODE BUTTON
int buttonState;              // the current reading from the input pin
int lastButtonState = HIGH;   // the previous reading from the input pin (input_pullup)
unsigned long lastDebounceTime =   0;  // the last time the output pin was toggled
unsigned long debounceDelay    = 120;  // the debounce time
#else
//-- PREVIOS VERSIONS USED INTERRUPTS
volatile unsigned long buttonTime      = micros(); // Debounce timer
#endif

// PWM duty cycle
byte currentDuty = EEPROM[DUTY_EEPROM_ADDR];

// Report control 
volatile unsigned long counterLastMilis = 0;  // Miliseconds of last report
volatile bool detected = false;               // Particle Detected
const unsigned long counterRepInterval = 200; // Miliseconds between report;

// Counts per minute calculation
const float avg_period = 60.0;                // In seconds, 60 = 1 min
float elapsed = 0.0;                          // Elapsed since avg_period reset
float cpm = 0.0;                              // counts per minute            
float previous_cpm = 0.0;
unsigned int previous_count = 0;

// LED Control
unsigned long lastMillisLED = millis();    
unsigned const int MaxMillisLED = 300;

//

/////////////////////////////////////////////////////////////////////////
/////////////////////////////// PROTOTYPES //////////////////////////////
/////////////////////////////////////////////////////////////////////////

void handle_counter_int() { detected=true; if ((micros()-counterTime)>300) counterCount++; counterTime=micros(); }  // Handlde interrupt

#if (GEIGER_HARDWARE_VERSION < 3)   
//-- 13/01/2021 CHANGED INTERRUPT TO POLLING FOR MODE BUTTON IN VERSION 3 (REQUIRES PULLUP RESISTOR IN BUTTON)
void handle_button_int()  { if ((micros()-buttonTime)>300) configMode = !configMode; buttonTime=micros(); inactiveMillis = millis(); }  // Handlde interrupt
#endif

void configure_pwm();                          // change PWM duty cycle with pot
void init_counter(); //                        // (re-)initialize countermeter         
void compute_counts(bool force);               // Compute and report counts 
void report_counts(void);                      // Update display
void report_PWM_duty();                        // Update dusplay with current duty cycle
void broadcast_data(bool resend);              // RF433 broadcast

/////////////////////////////////////////////////////////////////////////
////////////////////  *** GEIGER COUNTER SETUP ***  /////////////////////
/////////////////////////////////////////////////////////////////////////
void setup() {

  // Serial
#ifdef SERIAL_DEBUG
  Serial.begin(9600);
  Serial.print("GEIGER COUNTER VERSION ");Serial.println(_GEIGER_VERSION);
#endif

  // OLED Display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display.setRotation(OLED_ROTATION);
  display.display();
  display.clearDisplay();
  display.drawBitmap(0, 0, logo, 128, 32, WHITE);
  display.display();
  delay(2000);
  display.clearDisplay();
  
  // Flash pin
  pinMode(LED_PIN, OUTPUT); //

  // Initialize mode button
  //-- 13/01/2021 CHANGED INTERRUPT TO POLLING FOR MODE BUTTON
  #if (GEIGER_HARDWARE_VERSION < 3)   // Fist prototype pinout
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handle_button_int, LOW);
  #endif
  
  // Set 9-BIT FAST PWM MODE  
  TCCR1A = TCCR1A & 0xe0 | 2;
  TCCR1B = TCCR1B & 0xe0 | 0x09; 
  // SET DUTY CYCLE
  if ((currentDuty<=MIN_PWM_DUTY)||(currentDuty>=MAX_PWM_DUTY))
    currentDuty = INITIAL_PWM_DUTY;
  EEPROM[DUTY_EEPROM_ADDR] = currentDuty;
  analogWrite(PWMOUT_PIN, currentDuty); 
  
  // Geiger counter
  init_counter();
  pinMode(DETECT_PIN, OUTPUT);
  digitalWrite(DETECT_PIN, LOW);
  pinMode(DETECT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(DETECT_PIN), handle_counter_int, FALLING);
  delay(300);
  configMode = false;

  // Manchester lib initialization
  // (zeitgeist3k's inituializes on constructor)
  // man.workAround1MhzTinyCore(); //add this in order for transmitter to work with 1Mhz Attiny85/84
#if RF433_BROADCAST == RF433_MANCHESTER  
  man.setupTransmit(RF_OUTPUT_PIN, MANCHESTER_SPEED);
#endif

   // Begin timing
   sensor_data.begin();

}

/////////////////////////////////////////////////////////////////////////
////////////////////  *** GEIGER COUNTER LOOP ***  //////////////////////
/////////////////////////////////////////////////////////////////////////
void loop() {

  //--- RECOMPUTE CURRENT COUNTS PER MINUTE (CPM)
  compute_counts(false);

#if (GEIGER_HARDWARE_VERSION > 2)   
  //--- POLL MODE BUTTON AND DEBOUNCE
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState) {
  // reset the debouncing timer
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;
      // only toggle the LED if the new button state is LOW 
      if (buttonState == HIGH) {
        configMode = !configMode;
        inactiveMillis = millis();
      }
    }
  }
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
#endif

  //--- Configuring PWM -- Handle PWM pot
  if (configMode){
    int inputval = analogRead(DUTY_POT_PIN);
    int newval = map (inputval, 0, 1023, MIN_PWM_DUTY, MAX_PWM_DUTY);
    if (newval != currentDuty){
      inactiveMillis = millis();
      currentDuty = newval; 
      analogWrite(PWMOUT_PIN, currentDuty); // 
      EEPROM[DUTY_EEPROM_ADDR] = currentDuty;
    } else {
      if ((millis()-inactiveMillis) > maxInactivePot)
        configMode = false;
    }
    report_PWM_duty();
  }

  //-- Not configuring -- update standard display
  else 
  {
    display.display();
  }

  // LED 
  if (detected)   {
      digitalWrite(LED_PIN, HIGH);
      if ((millis() - lastMillisLED) >= MaxMillisLED)  {
          detected = false;
          digitalWrite(LED_PIN, LOW);
      }
  } else {
    lastMillisLED = millis();    
  }  

}

/////////////////////////////////////////////////////////////////////////
///////// *** COMPUTE COUNTS PER MINUTE, DISPLAY IF NEEDED *** //////////
/////////////////////////////////////////////////////////////////////////

void compute_counts(bool force) {
   elapsed = millis() - counterStart;
   if (elapsed < 10.0) return;
   cpm = (float)counterCount * (avg_period/(elapsed/1000.0));
    if ( force || ((millis() - counterLastMilis) >= counterRepInterval)) {
       report_counts(); 
       counterLastMilis = millis();
    }
    if (elapsed >= (1000.0*avg_period)) {  // Average time expired
       broadcast_data(false);
       lastMillisResend = millis();
       noInterrupts();
       init_counter(); 
       interrupts();
   } 
#ifdef RF433_BROADCAST      
   // Resend frame every 10 seconds (in case first frame lost)
   else if ((millis() - lastMillisResend) >= MaxMillisResend)  {
       if (dataframe) {
          broadcast_data(true);
          // sensor_data.dump(&Serial);
       }
       lastMillisResend = millis();    
    }  
#endif
}

/////////////////////////////////////////////////////////////////////////
////////////////////////  *** UPDATE DISPLAY *** ////////////////////////
/////////////////////////////////////////////////////////////////////////

void report_counts() {
  display.clearDisplay();
  // RADIATION ICON 
   if (detected)
  {
      display.drawBitmap(-10, 0, lcd_bmp, 128, 32, WHITE);
  }
  else {
      display.setTextSize(1);
      display.setCursor(70,10);
      display.print(counterCount); display.print(" ");display.print((int)(elapsed/1000.0));display.println("s");
      display.setCursor(70,24);
      display.setTextSize(1);
      display.println("(Prev)");  
  }
  // TEXT ON DISPLAY
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(4,3);
  if (cpm  <= 9.99) display.println(cpm,  1); else display.println((int)cpm);
  display.setCursor(4,19);
  display.println(previous_count);
  //if (previous_cpm <= 9.99) display.println(previous_cpm,1 ); else display.println((int)previous_cpm);
}

/////////////////////////////////////////////////////////////////////////
////////////////////////  *** BROADCAST DATA *** ////////////////////////
/////////////////////////////////////////////////////////////////////////


void broadcast_data(bool resend) {
#ifdef RF433_BROADCAST   
  if (!resend) {
   // Create and send dataframe
   sensor_data.set(cpm);
   sensor_data.set_checksum();
  }
#if RF433_BROADCAST == RF433_MANCHESTER
   dataframe = (byte *)sensor_data.x_bufptr();
   man.transmitArray(sensor_data.buflen(), dataframe);
#elif RF433_BROADCAST == RF433_ZEITGEIST
   dataframe = (byte *)sensor_data.bufptr();
   transmitter.send(dataframe, sensor_data.msglen());
#else
#error "** INVALID RF433_BROADCAST OPTION" RF433_BROADCAST
#endif //-- RF433_BROADCAST == x
#ifdef SERIAL_DEBUG 
   Serial.print(resend? "** RE": "**** ");
   Serial.print("SENT  ");Serial.print((int)dataframe, HEX);Serial.print(" ");
   Serial.print(sensor_data.msglen()); 
   Serial.println(" Bytes --------------------------------------------");
   sensor_data.dump(&Serial);
#endif //-- SERIAL_DEBUG
#endif //-- RF433_BROADCAST
}
/////////////////////////////////////////////////////////////////////////
////////////// Initialize CPM and save previous count ///////////////////
/////////////////////////////////////////////////////////////////////////

void init_counter() {
     //Serial.print("F_Count="); Serial.print(counterCount); Serial.print(" CPM=");Serial.println(cpm);
     counterStart  = millis();
     previous_cpm = cpm;
     previous_count = counterCount;
     counterCount = 0;
}

/////////////////////////////////////////////////////////////////////////
////////////// REPORT PWM %DUTY IN CALIRATION MODE  /////////////////////
/////////////////////////////////////////////////////////////////////////
void report_PWM_duty() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(50,9);
  display.println("CONFIG");
  
  // TEXT ON DISPLAY
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(4,3);
  display.println(currentDuty);
  display.setCursor(4,20);
  display.setTextSize(1);
  display.println("DUTY%");  
  display.display();
}
