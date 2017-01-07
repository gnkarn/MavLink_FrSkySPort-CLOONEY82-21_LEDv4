/*
 * led_control.ino part of MavLink_FrSkySPort
 *
 * Copyright (C) 2015 Jochen Kielkopf
 * https://github.com/Clooney82/MavLink_FrSkySPort
 *
 * modified by Alf Pettersson
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY, without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses>.
 *
 * Additional permission under GNU GPL version 3 section 7
 *
 * If you modify this Program, or any covered work, by linking or
 * combining it with FrSkySportTelemetry library (or a modified
 * version of that library), containing parts covered by the terms
 * of FrSkySportTelemetry library, the licensors of this Program
 * grant you additional permission to convey the resulting work.
 * {Corresponding Source for a non-source form of such a combination
 * shall include the source code for the parts of FrSkySportTelemetry
 * library used as well as that of the covered work.}
 *
 * ====================================================================================================
 *
 * Lighting Controller for ArduCopter based on Teensy 3.1 and
 * APM2.5 Mavlink to FrSky X8R SPort interface using Teensy 3.1
 *
 * Teensy 3.1 PIN 6 is used for controlling WS2812B LEDs
 * Keep in mind teensy only provide 3.3V Output, to control WS2812B LEDs you need to shift this to 5V.
 * possible Solution:  *1: https://www.pjrc.com/teensy/td_libs_OctoWS2811.html
 *                     *2: http://forums.adafruit.com/viewtopic.php?f=47&t=48908&p=246902#p246902
 *
 * To Use this script in Simulation Mode (#define SIMULATION_MODE)
 * In SIMULATION_MODE script can run stand alone on your development environment, but no input is possible.
 */
#ifdef USE_TEENSY_LED_SUPPORT
//#####################################################################################################
//### MAIN CONFIG PART                                                                              ###
//#####################################################################################################
//### INCLUDES                                                                                      ###
//#####################################################################################################
#include <FastLED.h>


//#define SIMULATION_MODE // uncomment to enable simulation mode ( habilitar para probar los diferentes modos)
//#define standalone      // DO NOT UNCOMMENT, just for testing purpose as standalone script without mavlink part.

//#####################################################################################################
//### DEFAULT VARIABLES                                                                             ###
//#####################################################################################################
#define RIGHT 0
#define LEFT  1
#define OFF   0
#define ON    1
#define AUTO  2
#define BLINK 3
#define PWM_MIN    982  // minimum pwm value of your RC-System
#define PWM_MID   1496  // middle  pwm value of your RC-System
#define PWM_MAX   2006  // maximum pwm value of your RC-System

#ifdef SIMULATION_MODE
int LED_MODE_SWITCH;
int LED_DIMM_CHAN;
#else
  #define LED_MODE_SWITCH 15-1   // Channel Number of RC Channel used for manual Lightmode
  #define LED_DIMM_CHAN   16-1   // Channel Number of RC Channel used for dimming LEDs
  #define LED_BPM_CHAN   14-1  // Channel Number of RC Channel used for BPM LEDs
#endif



//#####################################################################################################
//### FASTLED CONFIG                                                                                ###
//#####################################################################################################
#define RGBORDER  GRB // BRG para 2811, GRB para 2812
#define LEDTYPE   WS2812B

#define NUM_ARMS 3
#define NUM_LEDS_PER_STRIP 50
#define BRIGHTNESS          96
#define FRAMES_PER_SECOND   48
#define NUM_LEDS_PER_ARM    36
#define NUM_LEDS NUM_LEDS_PER_ARM*NUM_ARMS      // Number of LED's
CRGB Vir_led[NUM_LEDS]     ;                      // simula una tira continua con todos los leds de cada brazo conectados en serie
#define BRIGHTNESS          255
#define DATA_PIN    6

CRGB leds[NUM_ARMS][NUM_LEDS_PER_STRIP]; // 

// CRGBArray<NUM_LEDS_PER_STRIP> leds[NUM_ARMS] ; // 

unsigned long targetmillis_LEFT;
unsigned long targetmillis_RIGHT;
unsigned long targetmillis_ARMED;

int state_LEFT = 0;
int state_RIGHT = 0;
int state_ARMED = 0;

float dim = 100; // variable global para dim
int gbpm = 64 ; // variable global para efectos de led que quiero variar desde el SD del remoto

//###################################################################################################
//### EFECTOS                                                                              ###
//###################################################################################################

// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void(*SimplePatternList[])();
SimplePatternList gPatterns = { applause, discostrobe, setBandera, brazoSenoidal,circling,sinelon, rainbow, rainbowWithGlitter, confetti, juggle, bpm,  sinelon2,paletteWaves };


uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns
uint8_t g_bpm=20; // global bpm usado por ejemplo en circling
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void addGlitter(fract8 chanceOfGlitter); // incorporada esta declaracion pues al tener un tipo dentro de la funcion el compilador de arduino se marea , en realidada todas las funciones deberian estar declaradas en un .h aparte

void setArms(uint8_t h, uint8_t s, uint8_t v, int pos ) ;
void setSegment (int linit,int  lend, int br ,int pos, CRGB color);
void HeliLed (int bpm , int hue,float dim  );





//#####################################################################################################
//### LED VARIABLES                                                                                 ###
//#####################################################################################################

#define GPS  1
#define ARMED  2
#define FRONT  3
#define BACK  4
#define SIDE  5
#define FLASH  6
int  LED_DEF[NUM_ARMS][NUM_LEDS_PER_STRIP] = {{ARMED,ARMED,ARMED,ARMED,ARMED,ARMED,ARMED,ARMED,ARMED,ARMED,ARMED,ARMED,GPS,ARMED,ARMED,ARMED,     GPS,   GPS,   GPS,   GPS,   GPS,       ARMED,ARMED,  ARMED, GPS, ARMED, ARMED, ARMED, ARMED, ARMED, ARMED, ARMED, ARMED, ARMED, ARMED, ARMED,ARMED,  SIDE,SIDE,SIDE,SIDE,SIDE,SIDE,SIDE,SIDE,SIDE,SIDE,SIDE,SIDE, GPS },
                                              { BACK, BACK, BACK, BACK, BACK, BACK, BACK, BACK, BACK, BACK, BACK, BACK, BACK, BACK, BACK, BACK,     FLASH, FLASH, FLASH, FLASH, FLASH,     FRONT, FRONT, FRONT, FRONT, FRONT, FRONT, FRONT, FRONT, FRONT, FRONT, FRONT, FRONT, FRONT, FRONT, FRONT,FRONT,  GPS,GPS,GPS,GPS,GPS,GPS,GPS,GPS,GPS,GPS,GPS,GPS,GPS },
                                              { BACK, BACK, BACK, BACK, BACK, BACK, BACK, BACK, BACK, BACK, BACK, BACK, BACK, BACK, BACK, BACK,     FLASH, FLASH, FLASH, FLASH, FLASH,     FRONT, FRONT, FRONT, FRONT, FRONT, FRONT, FRONT, FRONT, FRONT, FRONT, FRONT, FRONT, FRONT, FRONT, FRONT,FRONT,  GPS,GPS,GPS,GPS,GPS,GPS,GPS,GPS,GPS,GPS,GPS,GPS,GPS }};


// organiza los leds para que luego la secuencia sea siempre ascendente independientemente de la forma de cableado de la tira

int  LED_Layout[3][16] = {{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15, },
                              { 36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21 }     ,
                              { 37,38,39,40,41,42,43,44,45,46,47,48,49,49,49,49}};


int LED_MODE = 999;
int FL_RUN = 0;

uint8_t ap_base_mode_last = 0;
int32_t ground_level = 0;

#ifdef SIMULATION_MODE
int SIM_LOOP;
unsigned long targetmillisfc_sim_mode = 0;
#ifdef standalone
int ap_base_mode;
int ap_bar_altitude;
int ap_sat_visible;
int ap_fixtype;
int ap_custom_mode;
int ap_throttle;
int ap_chancount;
int ap_chan_raw[18];
int ap_rssi;
#endif
#endif

unsigned long currentmillis;      // current milliseconds
unsigned long lastmillis;         // milliseconds of last loop
unsigned long targetmillis_FLASH; // target milliseconds
unsigned long targetmillis_GPS;
unsigned long targetmillis_LS;

int state_GPS;
int state_FLASH;

int pos;
int dir = RIGHT;

/*
* LED_COLORS
* CRGB::BLACK   <=> CHSV(0,0,0)
* CRGB::WHITE   <=> CHSV(0,0,255)
* CRGB::RED   <=> CHSV(0,255,255)
* CRGB::GREEN   <=> CHSV(120,255,255)
* CRGB::YELLOW  <=> CHSV(60,255,255)
* CRGB::ORANGE  <=> CHSV(39,255,255)
*/
/*
* Currently used variables from MavLink_FrSkySPort.ino:
* ap_base_mode  => ARMED=1, DISARMED=0
* ap_bar_altitude => ALT from barometer, 100 = 1m
* ap_sat_visible  => numbers of visible satelites
* ap_fixtype    => 0 = No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
* ap_custom_mode  => see http://copter.ardupilot.com/wiki/configuration/arducopter-parameters/#flight_mode_1_arducopterfltmode1
* ap_throttle   => 0..100
* ap_chancount  => Number of RC_Channels used
* ap_chan_raw[i]  => RC channel 1-18 value, in microseconds. A value of UINT16_MAX (65535U)
* ap_rssi     => Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
*/

//#####################################################################################################
//### PROGAM BEGIN                                                                                  ###
//#####################################################################################################
//### PROGRAM SETUP                                                                                 ###
//#####################################################################################################
#ifdef standalone
void setup() {        // uncomment in SIMULATION_MODE
#else
void Teensy_LED_Init() {   // comment in SIMULATION_MODE
#endif
 //FastLED.addLeds<WS2811, 6,RGBORDER >(leds[0], NUM_LEDS_PER_STRIP);  //Right front arm
  FastLED.addLeds<WS2812B, DATA_PIN+2,RGBORDER>(leds[0], NUM_LEDS_PER_STRIP);  //Left front arm
  FastLED.addLeds<WS2812B,DATA_PIN+1,RGBORDER>(leds[1], NUM_LEDS_PER_STRIP);  //Left front arm
  FastLED.addLeds<WS2812B, DATA_PIN,RGBORDER>(leds[2], NUM_LEDS_PER_STRIP);  //Left front arm
   //FastLED.addLeds<APA102, 6, 7, RGB,DATA_RATE_MHZ(12)>(leds[0], NUM_LEDS_PER_STRIP);  // HABILITAR PARA APA
  //FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, RGB, DATA_RATE_MHZ(12)>(leds, NUM_LEDS);  // DOCUMENTADO PARA apa
  FastLED.setBrightness(BRIGHTNESS);
#ifdef standalone
  Serial.begin(9600);
#endif

// test de leds habilitar

  for (int i = 0; i < 50; i++) {
        leds[0][i] = CHSV(0, 255, 5*i);
        FastLED.show();
        delay(20);
          }
  for (int i = 0; i < 50; i++) {
        leds[1][i] = CHSV(85, 255, 5*i);
            FastLED.show();
        delay(20);
          }
  for (int i = 0; i < 50; i++) {
        leds[2][i] = CHSV(170, 255, 5*i);
            FastLED.show();
        delay(20);
          }

 // fin test de leds
  FastLED.delay(3000);

 // ***********test de efecto ( habilitar)
 // ***************void HeliLed (int bpm , int hue,float dim  )
  //   comentar bloque para  -   ANULAR TEST DE EFECTOS


  for (int i = 0; i < 3000; i++) {
 //        HeliLed ( 100 , 0 , 200 );
  // Call the current pattern function once, updating the 'leds' array
   gPatterns[gCurrentPatternNumber](); // habilitar esto
  
  FastLED.show(); // display this frame
  FastLED.delay(1000 / FRAMES_PER_SECOND );// 60 FRAMESPERSECOND

   // do some periodic updates
  EVERY_N_MILLISECONDS(20) { gHue++; } // slowly cycle the "base color" through the rainbow
  EVERY_N_SECONDS(10) { nextPattern(); } // change patterns periodically
  //EVERY_N_MILLISECONDS(60000/g_bpm){g_bpm = (g_bpm*2) % 80 ; } // 60000/bpm es el tiempo  que se tarda en dar una vuelta 
  }

      
}

//#####################################################################################################
//### PROGRAM LOOP                                                                                  ###
//#####################################################################################################
#ifdef standalone
void loop() {         // uncomment in SIMULATION_MODE
#else
  
 void Teensy_LED_process() {   // comment in SIMULATION_MODE ( DEJAR NORMAL)
#endif
        

    
  // float dim = 100; // para probar lo paso a global 
  currentmillis = millis();
  ap_base_mode_last = ap_base_mode;

#ifdef SIMULATION_MODE
  fc_simulation_mode();
#endif
  if (LED_MODE_SWITCH == 0) {
    get_mode();
    if (LED_DIMM_CHAN != 0) {
      dim = rc_dimmer_proz();
    }
  }
  else {
#ifdef SIMULATION_MODE
#else
    /*
    * from MavLink_FrSkySPort.ino
    * ap_chan_raw[i]  => RC channel i input value, in microseconds. A value of UINT16_MAX (65535U)
    */
    //Serial.print("led mode sw : "); //debug
    //Serial.print(ap_chan_raw[LED_MODE_SWITCH]); // debug
    gbpm = map(ap_chan_raw[LED_BPM_CHAN],1000,2000,2,200);          // varia el gbpm en mas o en menos en funcion del valor del canal 17 
	//Serial.print(" gbpm "); //debug
	//Serial.println(gbpm);	//debug
    
    if (ap_chan_raw[LED_MODE_SWITCH] <= 1000) {
      get_mode();   // DEFAULT, LED Mode - auto set by FC
    }
    else if (ap_chan_raw[LED_MODE_SWITCH] <= 1080) {
      LED_MODE = 101; // USER_DEFINED LED Mode 1
    }
    else if (ap_chan_raw[LED_MODE_SWITCH] <= 1165) {
      LED_MODE = 102; // USER_DEFINED LED Mode 2
    }
    else if (ap_chan_raw[LED_MODE_SWITCH] <= 1250) {
      LED_MODE = 103; // USER_DEFINED LED Mode 3
    }
    else if (ap_chan_raw[LED_MODE_SWITCH] <= 1320) {
      LED_MODE = 104; // USER_DEFINED LED Mode 4
    }
    else if (ap_chan_raw[LED_MODE_SWITCH] <=  1400) {
      LED_MODE = 105; // USER_DEFINED LED Mode 5   
    }
    else if (ap_chan_raw[LED_MODE_SWITCH] <=  1480) {
      LED_MODE = 106; // USER_DEFINED LED Mode 6 
    }
    else if (ap_chan_raw[LED_MODE_SWITCH] <=  1560) {
      LED_MODE = 107; // USER_DEFINED LED Mode 7 
    }
    else if (ap_chan_raw[LED_MODE_SWITCH] <=  1640) {
      LED_MODE = 108; // USER_DEFINED LED Mode 8 
    }
    else if (ap_chan_raw[LED_MODE_SWITCH] <=  1730) {
      LED_MODE = 109; // USER_DEFINED LED Mode 9 
    }
    else if (ap_chan_raw[LED_MODE_SWITCH] <=  1810) {
      LED_MODE = 110; // USER_DEFINED LED Mode 10 
    }
    else if (ap_chan_raw[LED_MODE_SWITCH] <=  1890) {
      LED_MODE = 111; // USER_DEFINED LED Mode 11 
    }
    else if (ap_chan_raw[LED_MODE_SWITCH] <=  1966) {
      LED_MODE = 112; // USER_DEFINED LED Mode 12
    }
    else {
      get_mode();   // DEFAULT, LED Mode - auto set by FC
    }
    if (LED_DIMM_CHAN != 0) {
      dim = rc_dimmer_proz();
    }
#endif
  }
  dim = dim / 100;  // ver si esta linea hace falta -- probar.

  switch (LED_MODE) {
	 

  case 1:   // NO_GPS Flight modes
    default_mode(ON, dim);
    break;
  case 2:   // GPS    Flight modes
    front_arms(ON, dim);
    rear_arms(ON, dim);
    flash_pos_light(ON, dim);
    get_armed_status(ON, dim);
    get_gps_status(ON, dim);
    break;
  case 3:   // MANUAL Flight modes
    default_mode(OFF, dim);
    break;
  case 4:   // AUTO   Flight modes
    LarsonScanner(0, 0, NUM_LEDS_PER_STRIP, dim);
    get_armed_status(ON, dim);
    get_gps_status(ON, dim);
    front_arms(ON, dim);
    rear_arms(ON, dim);
    break;
  case 5:   // LAND   Flight mode
    LarsonScanner(0, 0, NUM_LEDS_PER_STRIP, dim);
          flash_pos_light(ON, dim);
    get_armed_status(ON, dim);
    get_gps_status(ON, dim);
    break;
  case 6:   // BRAKE Flight mode
    flash_pos_light(ON, dim);
    get_armed_status(ON, dim);
    get_gps_status(ON, dim);
    break;
  case 7:   // FLIP  Flight mode
    LarsonScanner(0, 0, NUM_LEDS_PER_STRIP, dim);
    break;
  case 8:   // AUTO_TUNE, ...
                LarsonScanner(0, 0, NUM_LEDS_PER_STRIP, dim);
                front_arms(ON, dim);
    rear_arms(ON, dim);

    break;
  case 101:   // USER Mode 1
    //LarsonScanner(0, 0, NUM_LEDS_PER_STRIP, dim);
   discostrobe();
    
    break;
  case 102:   // USER Mode 2
    LarsonScanner(0, 0, NUM_LEDS_PER_STRIP, dim);
    get_armed_status(ON, dim);
    get_gps_status(ON, dim);
    break;
  case 103:   // USER Mode 3
    front_arms(ON, dim);
    rear_arms(ON, dim);
    get_armed_status(ON, dim);
    get_gps_status(ON, dim);
    flash_pos_light(ON, dim);
    longRoll();             
    //setBandera();
    break;
  case 104:   // USER Mode 4
    //front_arms(OFF, dim);
    //rear_arms(OFF, dim);
    //get_armed_status(OFF, dim);
    //get_gps_status(OFF, dim);
    flash_pos_light(ON, dim);
    //brazoSenoidal();          //  prueba de efecto
    setBandera();
    break;
  case 105:   // USER Mode 5 - 
    rainbowWithGlitter();
    break;
  case 106:   // USER Mode 6 -
	  paletteWaves();
    flash_pos_light(ON, dim);
    break;
  case 107:   // USER Mode 7 -
    applause();
    break;
  case 108:   // USER Mode 8 -
    // void HeliLed (int bpm , int hue,float dim  )
     HeliLed(30 ,  gHue, dim  );
    break;
  case 109:   // USER Mode 9 -
    bpm();
    break;
  case 110:   // USER Mode 10 -
    confetti();
    break;
  case 111:   // USER Mode 11 -
    paletteWaves();
    break;
  case 112:   // USER Mode 12 -
    juggle();
    addGlitter(80);
    break;
    
  case 999:   // disarmed
    front_arms(OFF, dim);
    rear_arms(OFF, dim);
    flash_pos_light(ON, dim);
    get_armed_status(ON, dim);
    get_gps_status(ON, dim);
    break;
  }
   //ThrotleLed (int brazo , int posicion, int LedsNum, int hue, float dim  )
    ThrotleLed (0, 2, 13, 200,  dim  );
	// Serial.print(" dim "); //debug
	//Serial.println(dim);	//debug toma valores float de 0 a 1

  FastLED.show();
  lastmillis = currentmillis;
  //Serial.print(" LedMode ");  //debug
  //Serial.println(LED_MODE);		//debug
}

//#####################################################################################################
//###                             DIMMER FUNCTIONs                                                  ###
//#####################################################################################################
byte rc_dimmer() {          // ppm to byte (min 0 / max 255)
  double factor = (PWM_MAX - PWM_MIN) / 255;
  /*
  * double ppm_val = ap_chan_raw[LED_DIMM_CHAN] - PWM_MIN;
  * float dim = ppm_val / factor;
  */
  byte TARGET_DIM;
  if ((ap_chan_raw[LED_DIMM_CHAN] - PWM_MIN) / factor > 255) {
    TARGET_DIM = 255;
  }
  else if ((ap_chan_raw[LED_DIMM_CHAN] - PWM_MIN) / factor < 0) {
    TARGET_DIM = 0;
  }
  else {
    TARGET_DIM = (ap_chan_raw[LED_DIMM_CHAN] - PWM_MIN) / factor;
  }
  return TARGET_DIM;
}

byte rc_dimmer_proz() {         // ppm to % ( min 0 % / max 100 % )
  double factor = (PWM_MAX - PWM_MIN) / 100;
  byte TARGET_DIM;
  if (ap_chan_raw[LED_DIMM_CHAN] > 900) {
    TARGET_DIM = (ap_chan_raw[LED_DIMM_CHAN] - PWM_MIN) / factor;
    if (TARGET_DIM > 100) {
      TARGET_DIM = 100;
    }
    else if (TARGET_DIM < 0) {
      TARGET_DIM = 0;
    }
  }
  else {
    TARGET_DIM = 15;
  }
  return TARGET_DIM;
}

//#####################################################################################################
//### SIMULATION FUNCTION                                                                           ###
//#####################################################################################################
#ifdef SIMULATION_MODE
void fc_simulation_mode() {
  int max_loops = 60;
  
  if (lastmillis >= targetmillisfc_sim_mode) {
    //LED_MODE_SWITCH = 0;// GNK
    if (SIM_LOOP <= 3) {
      ap_fixtype = 0;
      ap_sat_visible = random(0, SIM_LOOP);
      //SIM_LOOP = 20;
    }
    else if (SIM_LOOP <= 5) {
      ap_fixtype = 1;
      ap_sat_visible = random(SIM_LOOP, 3);
    }
    else if (SIM_LOOP > 5) {
      ap_fixtype = 2;
      ap_sat_visible = random(SIM_LOOP, 6);
    }

    if (SIM_LOOP > 8) {
      ap_fixtype = 3;
      ap_sat_visible = random(6, 20);
    }

    if (SIM_LOOP > 10) {
      ap_base_mode = 1;
    }
    else {
      ap_base_mode = 0;
      ap_bar_altitude = 0;
    }

    if (SIM_LOOP > 13) {
      ap_custom_mode = random(0, 17);
      ap_throttle = random(50, 100);
      ap_bar_altitude = random(0, 1000);
    }
    else {
      ap_custom_mode = 0;
      ap_throttle = 0;
    }

    if (SIM_LOOP > 20 && SIM_LOOP < 40) {
      LED_MODE = random(101, 105);
      LED_MODE_SWITCH = 13;
    }
    else {
      LED_MODE_SWITCH = 0;
    }

    if (SIM_LOOP > max_loops - 10) {
      ap_throttle = 0;
    }

    if (SIM_LOOP > max_loops - 5) {
      ap_throttle = 0;
      ap_base_mode = 0;
      ap_custom_mode = 0;
      LED_MODE = 0;
    }

    if (SIM_LOOP > max_loops) {
      SIM_LOOP = 0;
    }
    else {
      SIM_LOOP++;
    }
    
    if (SIM_LOOP < 9) {
    LED_MODE = SIM_LOOP;
    LED_MODE_SWITCH = 13;
    }
    
    /*
    LED_MODE = 102;
    LED_MODE = 4;
    LED_MODE_SWITCH = 13;
    */
    
    Serial.println();
    Serial.print("LED_MODE: ");
    Serial.print(LED_MODE);
    Serial.print(" LastMillis: ");
    Serial.print(lastmillis);
    Serial.print(" CurMillis: ");
    Serial.print(currentmillis);
    Serial.print(", targetMillis: ");
    Serial.print(targetmillisfc_sim_mode);
    Serial.print(", Loop: ");
    Serial.print(SIM_LOOP);
    Serial.print(":");
    Serial.println();
    Serial.print("fixtype: ");
    Serial.print(ap_fixtype);
    Serial.print(" #Sats: ");
    Serial.print(ap_sat_visible);
    Serial.print(" Armed?: ");
    Serial.print(ap_base_mode);
    Serial.print(" FlightModes: ");
    Serial.print(ap_custom_mode);
    Serial.print(" Throttle: ");
    Serial.print(ap_throttle);
    Serial.print(" ALT: ");
    Serial.print(ap_bar_altitude);
    Serial.println();
    
    targetmillisfc_sim_mode = currentmillis + 3000;
  }
}
#endif

//#####################################################################################################
//### GET ArduCopter Flightmode                                                                     ###
//#####################################################################################################
void get_mode() {
  /* from MavLink_FrSkySPort.ino
  *  ap_custom_mode  => see http://copter.ardupilot.com/wiki/configuration/arducopter-parameters/#flight_mode_1_arducopterfltmode1
  *  LED_MODE  TYPE
  *  1         NO_GPS Flight modes
  *  2         GPS    Flight modes
  *  3         MANUAL Flight modes
  *  4         AUTO   Flight modes
  *  5         LAND
  *  6         BRAKE
  *  7         FLIP
  *  8         AUTO_TUNE, ...
  *  999       DISARMED
  *
  *  VALUE  MEANING    LED_MODE
  *  0      Stabilize  1
  *  1      Acro       3
  *  2      AltHold    1
  *  3      Auto       4
  *  4      Guided     4
  *  5      Loiter     2
  *  6      RTL        4
  *  7      Circle     4
  *  9      Land       5
  *  11     Drift      3
  *  13     Sport      1
  *  14     Flip       7
  *  15     AutoTune   8
  *  16     PosHold    2
  *  17     Brake      6
  *  ...    ...        ...
  *
  *  Mapping of AC Flight modes to LED lighting modes.
  */
  if (ap_base_mode == 1) {
    switch (ap_custom_mode) {
    case 0:  LED_MODE = 1; break;
    case 1:  LED_MODE = 3; break;
    case 2:  LED_MODE = 1; break;
    case 3:  LED_MODE = 4; break;
    case 4:  LED_MODE = 5; break;
    case 5:  LED_MODE = 2; break;
    case 6:  LED_MODE = 4; break;
    case 7:  LED_MODE = 4; break;
    case 9:  LED_MODE = 5; break;
    case 11: LED_MODE = 3; break;
    case 13: LED_MODE = 1; break;
    case 14: LED_MODE = 7; break;
    case 15: LED_MODE = 8; break;
    case 16: LED_MODE = 2; break;
    case 17: LED_MODE = 6; break;
    default: LED_MODE = 1; break;
    }
  }
  else {
    LED_MODE = 999;
  }
}

//#####################################################################################################
//### SETUP DEFAULT MODE FOR LED LIGHTNING - SIDEARMS, FRONT-, BREAK-, LANDING-LIGHT always off     ###
//#####################################################################################################
void default_mode(int STATUS, float dim) {
  front_arms(STATUS, dim);
  rear_arms(STATUS, dim);
  flash_pos_light(STATUS, dim);
  get_armed_status(ON, dim);
  get_gps_status(ON, dim);
}





//#####################################################################################################
//### GET ARMED STATUS FROM ARDUPILOT AND PRINT CORESPONDING LED(s)                                 ###
//#####################################################################################################
void get_armed_status(int STATUS, float dim) {
  if (STATUS == 1) {
    switch (ap_base_mode) {
    case 0:
      for (int i = 0; i < NUM_ARMS; i++) {
        for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
          if (LED_DEF[i][j] == ARMED) {
            leds[i][j] = CHSV(39, 255, 255 * dim);
          }
        }
      }
      break;
    case 1:
      for (int i = 0; i < NUM_ARMS; i++) {
        for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
          if (LED_DEF[i][j] == ARMED) {
            leds[i][j] = CHSV(96, 255, 255 * dim);
          }
        }
      }
      break;
    default:
      for (int i = 0; i < NUM_ARMS; i++) {
        for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
          if (LED_DEF[i][j] == ARMED) {
            leds[i][j] = CHSV(0, 0, 0);
          }
        }
      }
      break;
    }
  }
  else {
      for (int i = 0; i < NUM_ARMS; i++) {
        for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
          if (LED_DEF[i][j] == ARMED) {
            leds[i][j] = CHSV(0, 0, 0);
          }
        }
      }
  }
}

//#####################################################################################################
//### GET GPS STATUS FROM ARDUPILOT AND PRINT CORESPONDING LED(s)                                 ###
//#####################################################################################################
void get_gps_status(int STATUS, float dim) {
  /* from MavLink_FrSkySPort.ino
  *  ap_sat_visible  => numbers of visible satellites
  *  ap_fixtype    => 0 = No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
  */
  CRGB COLOR;
  int FREQ;
  if (STATUS == 1) {
    //Serial.print(ap_fixtype);
    switch (ap_fixtype) {
    case 0:  //blink red
      COLOR = CHSV(0, 255, 255 * dim);
      FREQ = 750;
      break;
    case 1: //blink orange
      COLOR = CHSV(39, 255, 255 * dim);
      FREQ = 600;
      break;
    case 2: // blue
      COLOR = CHSV(160, 255, 128 * dim);
      FREQ = 500;
      break;
    case 3: // const blue
      COLOR = CHSV(160, 255, 180 * dim);
      FREQ = 0;
      state_GPS = 0;
      break;
    case 4: // const blue
      COLOR = CHSV(160, 255, 255 * dim);
      FREQ = 0;
      state_GPS = 0;
      break;
    default: // off
      COLOR = CHSV(0, 0, 0);
      FREQ = 0;
    }
    if (currentmillis >= targetmillis_GPS) {
      if (state_GPS == 0) {
        for (int i = 0; i < NUM_ARMS; i++) {
          for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
            if (LED_DEF[i][j] == GPS) {
              leds[i][j] = COLOR;
            }
          }
        }
        state_GPS = 1;
      }
      else {
        for (int i = 0; i < NUM_ARMS; i++) {
          for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
            if (LED_DEF[i][j] == GPS) {
              leds[i][j] = CHSV(0, 0, 0);
            }
          }
        }
        state_GPS = 0;
      }
      targetmillis_GPS = lastmillis + FREQ;
    }
  }
  else {
    for (int i = 0; i < NUM_ARMS; i++) {
      for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
        if (LED_DEF[i][j] == GPS) {
          leds[i][j] = CHSV(0, 0, 0);
        }
      }
    }
  }
}


//#####################################################################################################
//### DEFAULT FRONT ARM LED MODE (CONST-Green)                                                        ###
//#####################################################################################################
void front_arms(int STATUS, float dim) {
    if (STATUS == 1) {
        for (int i = 0; i < NUM_ARMS; i++) {
            for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
                if (LED_DEF[i][j] == FRONT) {
                    leds[i][j] = CHSV(96, 255, 255 * dim);
                }
            }
        }
    } else {
        for (int i = 0; i < NUM_ARMS; i++) {
            for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
                if (LED_DEF[i][j] == FRONT) {
                    leds[i][j] = CHSV(0, 0, 0);
                }
            }
        }
    }
}

//#####################################################################################################
//### DEFAULT SIDE ARM LED MODE (CONST-YELLOW)                                                      ###
//#####################################################################################################
void side_arms(int STATUS, float dim) {
    if (STATUS == 1) {
        for (int i = 0; i < NUM_ARMS; i++) {
            for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
                if (LED_DEF[i][j] == SIDE) {
                    leds[i][j] = CHSV(60, 255, 255 * dim);
                }
            }
        }
    } else {
        for (int i = 0; i < NUM_ARMS; i++) {
            for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
                if (LED_DEF[i][j] == SIDE) {
                    leds[i][j] = CHSV(0, 0, 0);
                }
            }
        }
    }
}

//#####################################################################################################
//### DEFAULT REAR ARM LED MODE (CONST-RED)                                                       ###
//#####################################################################################################
void rear_arms(int STATUS, float dim) {
    if (STATUS == 1) {
        for (int i = 0; i < NUM_ARMS; i++) {
            for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
                if (LED_DEF[i][j] == BACK) {
                    leds[i][j] = CHSV(0, 255, 255 * dim);
                }
            }
        }
    } else {
        for (int i = 0; i < NUM_ARMS; i++) {
            for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
                if (LED_DEF[i][j] == BACK) {
                    leds[i][j] = CHSV(0, 0, 0);
                }
            }
        }
    }
}

//#####################################################################################################
//### FLASH POSITION LIGHTS (around motors)                                                         ###
//#####################################################################################################
/*  con esta funcion solo enciende y apaga 
 *   
 *   
void flash_pos_light(int STATUS, float dim) {
  int DELAY = 1000;
  if (dim < 0.3) dim = 0.3;
    if (STATUS == ON) {
  if (currentmillis >= targetmillis_FLASH) {    // turn leds on
      if (state_FLASH == 0) {
      for (int i = 0; i < NUM_ARMS; i++) {
        for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
          if (LED_DEF[i][j] == FLASH) {
            leds[i][j] = CHSV(255, 0, 255 * dim);
          }
        }
      }
      state_FLASH = 1;
                        DELAY = DELAY / 7;
    }
    else {
        
        for (int i = 0; i < NUM_ARMS; i++) {  // turn leds off
        for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
          if (LED_DEF[i][j] == FLASH) {
            leds[i][j] = CHSV(255, 0, 0);
          }
        }
      }
            state_FLASH = 0;
            if (FL_RUN == 3) {
              DELAY = DELAY / 9;
            }
            else if (FL_RUN == 4) {
              FL_RUN = 0;
            }
            FL_RUN++;
    }
    targetmillis_FLASH = lastmillis + 750;
  }
    }
    else {
    
        for (int i = 0; i < NUM_ARMS; i++) {
      for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
          if (LED_DEF[i][j] == FLASH) {
        leds[i][j] = CHSV(0, 0, 0);
          }
            }
  }
    }
}
*/
// **** con esta funcion hace un pulso senoidal  *****

void flash_pos_light(int STATUS, float dim) {
  int BPM = 60;
  // uint8_t bright = beatsin8( BPM /*BPM*/, 10 /*dimmest*/, 255 /*brightest*/ ); 
  if (STATUS == ON) {
  uint8_t bright = cubicwave8(beat8(BPM));// GENERA * 60*1000/bpm MILISECS
      for (int i = 0; i < NUM_ARMS; i++) {
        for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
          if (LED_DEF[i][j] == FLASH) {
            leds[i][j] = CHSV(255, 0, bright * dim);
          }
        }
      }
  }
      else {
    
        for (int i = 0; i < NUM_ARMS; i++) {
          for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
            if (LED_DEF[i][j] == FLASH) {
            leds[i][j] = CHSV(0, 0, 0);
            }
          }
        }
    }
}


//#####################################################################################################
//### LARSON SCANNER (CIRCLING ON ALL LEDs)                                                         ###
//#####################################################################################################
void LarsonScanner(byte LS_hue, int START_POS, int END_POS, float dim) {
  int DELAY = 50;
  if (pos == 0) {
    pos = START_POS;
  }
  else if (pos < START_POS) {
    pos = START_POS;
  }
  else if (pos > END_POS) {
    pos = START_POS;
  }

  if (currentmillis >= targetmillis_LS) {
    switch (dir) {
    case RIGHT:
      pos++;
      if (pos == END_POS - 1)
        dir = LEFT;
      break;
    case LEFT:
      pos--;
      if (pos == START_POS)
        dir = RIGHT;
      break;
    }
    for (int i = 0; i < NUM_ARMS; i++) {
      for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
        leds[i][j] = CHSV(0, 0, 0);
      }
    }
      if (dir == RIGHT) {
      if (pos <= END_POS - 1 && pos >= START_POS)
                                for (int i = 0; i < NUM_ARMS; i++) {
                                  leds[i][pos] = CHSV(LS_hue, 255, 255 * dim);
                                }
      if (pos <= END_POS + 1 && pos >= START_POS + 1)
                                for (int i = 0; i < NUM_ARMS; i++) {
                                  leds[i][pos - 1] = CHSV(LS_hue, 255, 180 * dim);
                                }
      if (pos <= END_POS + 2 && pos >= START_POS + 2)
                                for (int i = 0; i < NUM_ARMS; i++) {
                                  leds[i][pos - 2] = CHSV(LS_hue, 255, 100 * dim);
                                }
      if (pos == -2)
                                for (int i = 0; i < NUM_ARMS; i++) {
                                  leds[i][pos + 2] = CHSV(LS_hue, 255, 100 * dim);
                                }
    }
    if (dir == LEFT) {
      if (pos <= END_POS - 1 && pos >= START_POS)
                                for (int i = 0; i < NUM_ARMS; i++) {
                                  leds[i][pos] = CHSV(LS_hue, 255, 255 * dim);
                                }
      if (pos <= END_POS - 2 && pos >= START_POS - 1)
                                for (int i = 0; i < NUM_ARMS; i++) {
                                  leds[i][pos + 1] = CHSV(LS_hue, 255, 180 * dim);
                                }
      if (pos <= END_POS - 3 && pos >= START_POS - 2)
                                for (int i = 0; i < NUM_ARMS; i++) {
                                  leds[i][pos + 2] = CHSV(LS_hue, 255, 100 * dim);
                                }
      if (pos == END_POS + 1)
                                for (int i = 0; i < NUM_ARMS; i++) {
                                  leds[i][pos - 2] = CHSV(LS_hue, 255, 100 * dim);
                                }
    }
    targetmillis_LS = lastmillis + DELAY;
  }
}


// funciona para trasladar el valor del acelerador que es de 0-100 a una tira de leds de un Brazo, y una posicion, y una longitud de LedsNum
// ************************************************************
void ThrotleLed (int brazo , int posicion, int LedsNum, int hue, float dim  ){
  uint16_t  ThrotleLedNum = (ap_throttle * LedsNum)/100 ;
  //int ThrotleLedNum = map(ap_throttle,0,100,0,LedsNum);
  for ( int i=0 ; i< LedsNum;i++){
    if (i <= ThrotleLedNum)  { leds[brazo][(LED_Layout[posicion][i])]= CHSV(hue, 255, 255*dim);
    } 
      else { leds[brazo][LED_Layout[posicion][i]]= CHSV(0, 255, 0);
      }
  }
}

// ************************************************************
// funciona  led Helicoptero
// ************************************************************

void HeliLed (int bpm , int hue,float _dim  ){
 uint8_t Brazo0= beatsin8(bpm , 0, _dim ,0,0);
  uint8_t Brazo1= beatsin8(bpm , 0, _dim ,0,83);
   uint8_t Brazo2= beatsin8(bpm , 0, _dim ,0,166);
   SetBrazo(0,hue,Brazo0*(Brazo0>120));
    SetBrazo(1,hue,Brazo1*(Brazo1>120));
     SetBrazo(2,hue,Brazo2*(Brazo2>120));
}


// ************************************************************
// funciona  led Helicoptero
// ************************************************************

void SetBrazo (int brazo, int hue, int v ){
      fill_solid( leds[brazo], NUM_LEDS_PER_STRIP, CHSV(hue,255,v));
     
    }

//################################################################################
//###                                   efectos                                ###
//################################################################################
  void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE(gPatterns);
}

  // An animation to play while the crowd goes wild after the big performance
void applause()
{
  static uint16_t lastPixel = 0;
  fadeToBlackBy( Vir_led, NUM_LEDS, 32);
  Vir_led[lastPixel] = CHSV(random8(HUE_BLUE,HUE_PURPLE),255,255);
  lastPixel = random16(NUM_LEDS);
  Vir_led[lastPixel] = CRGB::White;
  vled_draw(); // draw from virtual to fisical leds
}


void rainbow()
{
  // FastLED's built-in rainbow generator
  fill_rainbow(Vir_led, NUM_LEDS, gHue, 7);
  vled_draw(); // draw from virtual to fisical leds
}

void addGlitter(fract8 chanceOfGlitter)
{
  if (random8() < chanceOfGlitter) {
    Vir_led[random16(NUM_LEDS)] += CRGB::White;
  }
  vled_draw(); // draw from virtual to fisical leds
}

void rainbowWithGlitter()
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
  vled_draw(); // draw from virtual to fisical leds
}



void confetti()
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy(Vir_led, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  Vir_led[pos] += CHSV(gHue + random8(64), 200, 255);
  vled_draw(); // draw from virtual to fisical leds
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy(Vir_led, NUM_LEDS, 60);
  int pos = beatsin16(13, 0, NUM_LEDS);
  Vir_led[pos] += CHSV(gHue, 255, 192);
  vled_draw(); // draw from virtual to fisical leds
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = gbpm ; //62
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8(BeatsPerMinute, 64, 255);
  for (int i = 0; i < NUM_LEDS; i++) { //9948
    Vir_led[i] = ColorFromPalette(palette, gHue + (i * 2), beat - gHue + (i * 10));
  }
  vled_draw(); // draw from virtual to fisical leds
  //Serial.print(" bpm : ");
  //Serial.print(gbpm);
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy(Vir_led, NUM_LEDS, 20);
  byte dothue = 0;
  for (int i = 0; i < 8; i++) {
    Vir_led[beatsin16(i + 7, 0, NUM_LEDS)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
  vled_draw(); // draw from virtual to fisical leds
}



// Updated sinelon (no visual gaps)
void sinelon2()
{
  // a colored dot sweeping 
  // back and forth, with 
  // fading trails
  fadeToBlackBy(Vir_led, NUM_LEDS, 20);
  int pos = beatsin16(13, 0, NUM_LEDS);
  static int prevpos = 0;
  if (pos < prevpos) {
    fill_solid(Vir_led + pos, (prevpos - pos) + 1, CHSV(gHue, 220, 255));
  }
  else {
    fill_solid(Vir_led + prevpos, (pos - prevpos) + 1, CHSV(gHue, 220, 255));
  }
  prevpos = pos;
  vled_draw(); // draw from virtual to fisical leds
}

void circling()
{
  // circulo completo
  
  fadeToBlackBy(Vir_led, NUM_LEDS, 10);
  int pos = scale8(beat8(g_bpm),NUM_LEDS);
  
  Vir_led[pos] += CHSV(gHue, 255, 192);
  vled_draw(); // draw from virtual to fisical leds
}


void brazoSenoidal()
{
  // a colored dot sweeping back and forth, with fading trails en cada brazo
  
  fadeToBlackBy(leds[0], NUM_LEDS_PER_STRIP, 30);
  int pos = beatsin16(20, 0, 16);
  setArms(gHue, 250, 192, pos );
  
}

// hace el render de la tira virtual a las tiras fisicas en cada brazo
void vled_draw() {
  Vled_invert(2); 
  memcpy(&leds[1],&Vir_led[NUM_LEDS_PER_ARM], NUM_LEDS_PER_ARM*3);
  Vled_invert(0); 

}

// para los brazos que estan conectados en sentido inverso
void Vled_invert(int brazo) {
  for (int i = 0; i < NUM_LEDS_PER_ARM; i++)
  {
    leds[brazo][i] = Vir_led[NUM_LEDS_PER_ARM*(brazo+1) - i];

  }
}
    

void setArms(uint8_t h, uint8_t s, uint8_t v, int pos  ) { //        setea todo el brazo con las tiras en el mismo patron
  leds[0][LED_Layout[0][pos]] = CHSV (h, s, v );
  leds[0][LED_Layout[1][pos]] = CHSV (h, s, v );
  leds[0][LED_Layout[2][pos]] = CHSV (h, s, v);
  memcpy(&leds[1], &leds[0][0], NUM_LEDS_PER_STRIP * 3);
  memcpy(&leds[2], &leds[0][0], NUM_LEDS_PER_STRIP * 3);
}

void setBandera() { //        setea cada brazo con los colores de la bandera
 
  // a colored dot sweeping back and forth, with fading trails en cada brazo
  
  fadeToBlackBy(leds[0], NUM_LEDS_PER_STRIP, 4);
  int pos = beatsin16(16, 0, 15);
  setArms(143, 175, 255, pos );  // setea todas las tiras celestes
  leds[0][LED_Layout[2][pos]] = CHSV (0,0,255); // setea la tira blanca
  memcpy(&leds[1], &leds[0][0], NUM_LEDS_PER_STRIP * 3);
  memcpy(&leds[2], &leds[0][0], NUM_LEDS_PER_STRIP * 3);
  
}

//****************************************************************
// en los brazos 1 y 2 establece una linea blanca proporcional al angulo de Roll 
// *******************************************************************

void longRoll(){
  uint8_t alfa= ((ap_roll_angle)) ; // *255/360
  fract8 longitud = sin8(alfa);
  
  uint8_t longi = map( longitud,0,255,0 ,(NUM_LEDS_PER_ARM/2));
  //setSegment (uint8_t linit,uint8_t lend, uint8_t br ,uint8_t pos, CRGB color)
  setSegment(0,longi,2,0,CRGB::White);
  setSegment(0,longi,2,1,CRGB::White);
  leds[2][LED_Layout[0][longi]]=CRGB::Red;
  leds[2][LED_Layout[1][longi]]=CRGB::Red;

  setSegment(0,NUM_LEDS_PER_ARM/2-longi,1,0,CRGB::White);
  setSegment(0,NUM_LEDS_PER_ARM/2-longi,1,1,CRGB::White);
  leds[1][LED_Layout[0][NUM_LEDS_PER_ARM/2-longi]]=CRGB::Red;
  leds[1][LED_Layout[1][NUM_LEDS_PER_ARM/2-longi]]=CRGB::Red;
  
   //Serial.print("roll : ");
    //Serial.print(ap_roll_angle);
    //Serial.print("longitud : ");
    //Serial.print(longitud);
    //Serial.print("longi : ");
    //Serial.print(longi);
  
  
  fadeToBlackBy(leds[1], NUM_LEDS_PER_STRIP, 20);
  fadeToBlackBy(leds[2], NUM_LEDS_PER_STRIP, 20);
  
}

// ****  setea un segmento desde el led de inicio al led del final con una barra de color fijo 

void setSegment (int linit,int  lend, int br ,int pos, CRGB color){
  for (int i =linit; i<lend ;i++){ 
    leds[br][LED_Layout[pos][i]]= color;
  }
}









#endif


