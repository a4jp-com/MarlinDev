/************************************************
 * Rambo pin assignments: From MarlinDev folder
 ************************************************/

// Servo support
#define SERVO0_PIN 22 // Motor header MX1
#define SERVO1_PIN 23 // Motor header MX2
#define SERVO2_PIN 24 // Motor header MX3
#define SERVO3_PIN  5 // PWM header pin 5

#if ENABLED(Z_PROBE_SLED)
  #define SLED_PIN -1
#endif

#undef X_MS1_PIN
#undef X_MS2_PIN
#undef Y_MS1_PIN
#undef Y_MS2_PIN
#undef Z_MS1_PIN
#undef Z_MS2_PIN
#undef E0_MS1_PIN
#undef E0_MS2_PIN
#undef E1_MS1_PIN
#undef E1_MS2_PIN

#undef DIGIPOTSS_PIN
//Fan_2 2

/*****************
#if ENABLED(ULTRA_LCD)

  #define KILL_PIN -1 //Glen//was 80 maybe a mistake

#endif // ULTRA_LCD 

#if ENABLED(VIKI2) || ENABLED(miniVIKI)
  #define BEEPER_PIN 44
  // Pins for DOGM SPI LCD Support
  #define DOGLCD_A0  70
  #define DOGLCD_CS  71
  #define LCD_SCREEN_ROT_180

  #define SD_DETECT_PIN -1 // Pin 72 if using easy adapter board

  #if ENABLED(TEMP_STAT_LEDS)
    #define STAT_LED_RED      22
    #define STAT_LED_BLUE     32
  #endif
#endif // VIKI2/miniVIKI

#if ENABLED(FILAMENT_SENSOR)
  //Filip added pin for Filament sensor analog input
  #define FILWIDTH_PIN        3
#endif

*/

/************************************************
 * Rambo pin assignments: From Marlin 1.0.0 with updates for terms
 ************************************************/
 
#ifndef __AVR_ATmega2560__
#error Oops!  Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH true

#define X_STEP_PIN 37
#define X_DIR_PIN 48
#define X_MIN_PIN 12
#define X_MAX_PIN 24
#define X_ENABLE_PIN 29
#define X_MS1_PIN 40
#define X_MS2_PIN 41

#define Y_STEP_PIN 36
#define Y_DIR_PIN 49
#define Y_MIN_PIN 11
#define Y_MAX_PIN 23
#define Y_ENABLE_PIN 28
#define Y_MS1_PIN 69
#define Y_MS2_PIN 39

#define Z_STEP_PIN 35
#define Z_DIR_PIN 47
#define Z_MIN_PIN 10
#define Z_MAX_PIN 30
#define Z_ENABLE_PIN 27
#define Z_MS1_PIN 68
#define Z_MS2_PIN 67

#define HEATER_BED_PIN 3
#define TEMP_BED_PIN 7         //2014/02/04  0:T0 / 1:T1 / 2:T2 / 7:T3

#define HEATER_0_PIN  9
#define TEMP_0_PIN 0

#define HEATER_1_PIN 7
#define TEMP_1_PIN -1

#define HEATER_2_PIN -1
#define TEMP_2_PIN -1

#define E0_STEP_PIN         34
#define E0_DIR_PIN          43
#define E0_ENABLE_PIN       26
#define E0_MS1_PIN 65
#define E0_MS2_PIN 66

#define E1_STEP_PIN         33
#define E1_DIR_PIN          42
#define E1_ENABLE_PIN       25
#define E1_MS1_PIN 63
#define E1_MS2_PIN 64

#define DIGIPOTSS_PIN 38
#define DIGIPOT_CHANNELS {4,5,3,0,1} // X Y Z E0 E1 digipot channels to stepper driver mapping

#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            13
#define FAN_PIN            8
#define PS_ON_PIN          4
#define KILL_PIN           80  //Glen//was -1
#define SUICIDE_PIN        -1  //PIN that has to be turned on right after start, to keep power flowing.

//2014/01/20 Add pin definitions
#define FAN_1_PIN   6
#define FAN_2_PIN   2
//

//2014/04/16
#ifdef ULTRA_LCD
  #ifdef NEWPANEL
  //arduino pin which triggers an piezzo beeper
    #define BEEPER_PIN -1    //Glen//was 79// Beeper on AUX-4
    #define LCD_PINS_RS 70 
    #define LCD_PINS_ENABLE 71
    #define LCD_PINS_D4 72
    #define LCD_PINS_D5 73 
    #define LCD_PINS_D6 74
    #define LCD_PINS_D7 75
   
    //buttons are directly attached using AUX-2
    //#define BTN_EN1 76     //Glen//might work 85
    //#define BTN_EN2 77     //Glen//might work 84
    //#define BTN_ENC 78     //Glen//might work 81 //the click
   
    #define BLEN_C 2
    #define BLEN_B 1
    #define BLEN_A 0
   
    #define SD_DETECT_PIN 46 //Ramps does not use this port
   
    //encoder rotation values
    #define encrot0 0
    #define encrot1 2
    #define encrot2 3
    #define encrot3 1
  #else //old style panel with shift register
    //arduino pin witch triggers an piezzo beeper
    #define BEEPER_PIN -1    //Glen was 33 // No Beeper added
    //buttons are attached to a shift register
    // Not wired this yet
    // #define SHIFT_CLK 38
    // #define SHIFT_LD 42
    // #define SHIFT_OUT 40
    // #define SHIFT_EN 17
   
    #define LCD_PINS_RS 70 
    #define LCD_PINS_ENABLE 17
    #define LCD_PINS_D4 23
    #define LCD_PINS_D5 25 
    #define LCD_PINS_D6 27
    #define LCD_PINS_D7 29
   
    //encoder rotation values
    #define encrot0 0
    #define encrot1 2
    #define encrot2 3
    #define encrot3 1
   
    //bits in the shift register that carry the buttons for:
    // left up center down right red
    #define BL_LE 7
    #define BL_UP 6
    #define BL_MI 5
    #define BL_DW 4
    #define BL_RI 3
    #define BL_ST 2
    #define BLEN_B 1
    #define BLEN_A 0
  #endif 
#endif //ULTRA_LCD

// 2014/04/16
#define LCD_PINS_RS 70      //ext2_5 //??CS chip select /??SS chip slave select
#define LCD_PINS_ENABLE 71  //ext2_7
#define LCD_PINS_D4 72      //??SCK (CLK) clock
#define LCD_PINS_D5 73 
#define LCD_PINS_D6 74      //ext2_13
#define LCD_PINS_D7 75
#define BEEPER_PIN -1

//Click encoder pins
//#define SW_DOWN  85       //ext2_6
#define BTN_EN1  85         //ext2_6//Glen//76?

//#define SW_UP 84          //ext2_8
#define BTN_EN2 84          //ext2_8//Glen//77? 

//#define SW_ENTER 81       //ext_14
#define BTN_ENC 81          //ext_14//Glen//Center button//78 doesn't work here

#define BTN_LEFT 83         //ext_10
#define BTN_RIGHT 82        //ext_12
//#define BTN_HOME 80       //ext_16//Glen//80 is used above for now as a kill switch
//