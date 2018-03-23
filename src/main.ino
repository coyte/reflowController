
// ***** INCLUDES *****
#include <Arduino.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <SPI.h>
#include <Ucglib.h>  //https://github.com/olikraus/ucglib
#include <max6675.h> //https://github.com/adafruit/MAX6675-library.git

// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE
{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_PREHEAT,
  REFLOW_STATE_SOAK,
  REFLOW_STATE_REFLOW,
  REFLOW_STATE_COOL,
  REFLOW_STATE_COMPLETE,
  REFLOW_STATE_TOO_HOT,
  REFLOW_STATE_ERROR
} reflowState_t;

typedef enum REFLOW_STATUS
{
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
} reflowStatus_t;

typedef  enum SWITCH
{
  SWITCH_NONE,
  SWITCH_1,
  SWITCH_2
} switch_t;

typedef enum DEBOUNCE_STATE
{
  DEBOUNCE_STATE_IDLE,
  DEBOUNCE_STATE_CHECK,
  DEBOUNCE_STATE_RELEASE
} debounceState_t;

typedef enum REFLOW_PROFILE
{
  REFLOW_PROFILE_LEADFREE,
  REFLOW_PROFILE_LEADED
} reflowProfile_t;

// ***** CONSTANTS *****
// ***** GENERAL PROFILE CONSTANTS *****
#define PROFILE_TYPE_ADDRESS 0
#define TEMPERATURE_ROOM 50
#define TEMPERATURE_SOAK_MIN 150
#define TEMPERATURE_COOL_MIN 100
#define SENSOR_SAMPLING_TIME 1000
#define SOAK_TEMPERATURE_STEP 5

// ***** LEAD FREE PROFILE CONSTANTS *****
#define TEMPERATURE_SOAK_MAX_LF 200
#define TEMPERATURE_REFLOW_MAX_LF 250
#define SOAK_MICRO_PERIOD_LF 9000

// ***** LEADED PROFILE CONSTANTS *****
#define TEMPERATURE_SOAK_MAX_PB 180
#define TEMPERATURE_REFLOW_MAX_PB 224
#define SOAK_MICRO_PERIOD_PB 10000

// ***** SWITCH SPECIFIC CONSTANTS *****
#define DEBOUNCE_PERIOD_MIN 100

// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 100
#define PID_KI_PREHEAT 0.025
#define PID_KD_PREHEAT 20
// ***** SOAKING STAGE *****
#define PID_KP_SOAK 300
#define PID_KI_SOAK 0.05
#define PID_KD_SOAK 250
// ***** REFLOW STAGE *****
#define PID_KP_REFLOW 300
#define PID_KI_REFLOW 0.05
#define PID_KD_REFLOW 350
#define PID_SAMPLE_TIME 1000

// ***** LCD MESSAGES *****
const char* lcdMessagesReflowStatus[] = {
  "Ready      ",
  "Preheat ",
  "Soak    ",
  "Reflow  ",
  "Cooldown",
  "Complete",
  "Hot!       ",
  "Error   "
};

// ***** DEGREE SYMBOL FOR LCD *****
unsigned char degree[8]  = {
  140, 146, 146, 140, 128, 128, 128, 128
};

// ***** PIN ASSIGNMENTS *****
//Display ST7735
#define TFT_CS      10
#define TFT_RST     8  
#define TFT_DC      9

//MAX6675 module (does not like multiple device on bus, so swSPI)
#define ktcSO       7
#define ktcCS       6
#define ktcCLK      5

//Others
const int ssrPin=2;
const int ledPin=3;
const int buzzerPin=4;
const int switchPin=A1;
const int ventPin=A5;



//#define ssrPin      2
//#define buzzerPin   3
//#define switchPin  A1
//#define ledPin      4

// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
int windowSize;
unsigned long windowStartTime;
unsigned long nextCheck;
unsigned long nextRead;
unsigned long updateLcd;
unsigned long timerSoak;
unsigned long buzzerPeriod;
unsigned char soakTemperatureMax;
unsigned char reflowTemperatureMax;
unsigned long soakMicroPeriod;
// Reflow oven controller state machine state variable
reflowState_t reflowState;
// Reflow oven controller status
reflowStatus_t reflowStatus;
// Reflow profile type
reflowProfile_t reflowProfile;
// Switch debounce state machine state variable
debounceState_t debounceState;
// Switch debounce timer
long lastDebounceTime;
// Switch press status
switch_t switchStatus;
switch_t switchValue;
switch_t switchMask;
// Seconds timer
int timerSeconds;
// Thermocouple fault status
unsigned char fault;

// Instanciate PID, thermocouple and display
PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
MAX6675 thermocouple(ktcCLK, ktcCS, ktcSO);
Ucglib_ST7735_18x128x160_HWSPI ucg(TFT_DC, TFT_CS,  TFT_RST);

// Display string variable for temperature
char inputstr[15];

void setup()
{
  // Serial communication at 9600 bps
  Serial.begin(9600);
  Serial.println("Initializing Tiny Reflow Controller");

  // Check current selected reflow profile
  unsigned char value = EEPROM.read(PROFILE_TYPE_ADDRESS);
  if ((value == 0) || (value == 1))
  {
    // Valid reflow profile value
    reflowProfile = value;
  }
  else
  {
    // Default to lead-free profile
    EEPROM.write(PROFILE_TYPE_ADDRESS, 0);
    reflowProfile = REFLOW_PROFILE_LEADFREE;
  }


  // SSR pin initialization to ensure reflow oven is off
  pinMode(ssrPin, OUTPUT);
  digitalWrite(ssrPin, LOW);


  // Buzzer pin initialization to ensure annoying buzzer is off
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  

  // LED pins initialization and turn on upon start-up (active high)
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  // Ventilation pin initialization and turn off
  pinMode(ventPin, OUTPUT);
  digitalWrite(ventPin, LOW);

  Serial.println("Initializing Display + 8 sec wait"); 
  // Init TFT
  ucg.begin(UCG_FONT_MODE_TRANSPARENT);
  ucg.clearScreen();
  ucg.setColor(1,255,255,255);		// use white as background for SOLID mode
  ucg.setColor(0,255,255,255);    // set active color white for box
  ucg.drawBox(0, 0, 128,160);     // draw white box 
  ucg.setColor(0,0,0,0);          // set active Black for text
  ucg.setPrintPos(5,45);
  ucg.setPrintDir(0);
  ucg.setFont(ucg_font_fub20_tf);
  ucg.print("Tiny");
  ucg.setPrintPos(20,80);
  ucg.print("Reflow");
  ucg.setFont(ucg_font_helvR10_hf);
  ucg.setPrintPos(22,100);
  ucg.print("V0.01");
  ucg.setPrintPos(6,155);
  ucg.print("niels@teekens.info");
  delay(5000);
  ucg.setColor(0, 255, 255, 255); //set active color to White
  ucg.drawBox(0, 0, 128,160);     //clear screen
  ucg.setColor(0,0,0,0);          //set active color to black
  ucg.drawHLine(0, 130, 128);
  ucg.drawVLine(60, 130, 38);
  ucg.setPrintPos(7,152);
  ucg.print("Profile");
  ucg.setPrintPos(70,152);
  ucg.print("Go/Stop");
  ucg.setColor(1,255,255,255);		// use white as background for SOLID mode
  ucg.setFontMode(UCG_FONT_MODE_SOLID);
  delay(3000);


  // Turn off LED (active high)
  digitalWrite(ledPin, LOW);
  // Set window size
  windowSize = 2000;
  // Initialize time keeping variable
  nextCheck = millis();
  // Initialize thermocouple reading variable
  nextRead = millis();
  // Initialize LCD update timer
  updateLcd = millis();
  Serial.println("End of setup, entering loop");
}

void loop()
{
  // Current time
  unsigned long now;

  // Time to read thermocouple?
  if (millis() > nextRead)
  {
    // Read thermocouple next sampling period
    nextRead += SENSOR_SAMPLING_TIME;
    // Read current temperature
    input = thermocouple.readCelsius();
  }

  if (millis() > nextCheck)
  {
    // Check input in the next seconds
    nextCheck += 1000;
    // If reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Toggle red LED as system heart beat
      digitalWrite(ledPin, !(digitalRead(ledPin)));
      digitalWrite(ventPin, HIGH);
      // Increase seconds timer for reflow curve analysis
      timerSeconds++;
      // Send temperature and time stamp to serial
      Serial.print(timerSeconds);
      Serial.print(",");
      Serial.print(setpoint);
      Serial.print(",");
      Serial.print(input);
      Serial.print(",");
      Serial.println(output);
    }
    else
    {
      // Turn off red LED
      digitalWrite(ledPin, LOW);
      digitalWrite(ventPin, LOW);
    }
  }

  if (millis() > updateLcd)
  {
    // Update LCD in the next 250 ms
    updateLcd += 250;
    // Print current system state
    ucg.setPrintPos(6,57);
    ucg.print(lcdMessagesReflowStatus[reflowState]);
  
    if (reflowProfile == REFLOW_PROFILE_LEADFREE)
    {
      ucg.setPrintPos(7,152);
      ucg.print("   LF   ");
    }
    else
    {
      ucg.setPrintPos(7,152);
      ucg.print("   PB   ");
    }
    // Move the cursor to the 2 line
    //lcd.setCursor(0, 1);

    // If currently in error state
    if (reflowState == REFLOW_STATE_ERROR)
    {
      // Thermocouple error (open, shorted)
      ucg.setPrintPos(6,97);
      ucg.print("TC Error");

    }
    else
    {
      // Display current temperature
      ucg.setPrintPos(6,97);
      ucg.print(input);
      dtostrf(input,8,2,inputstr);
      ucg.drawGlyph(ucg.getStrWidth(inputstr),97,0,176);       // Display degree Celsius symbol
      ucg.setPrintPos(50,97);
      ucg.print("C");
    }
  }

  // Reflow oven controller state machine
  switch (reflowState)
  {
    case REFLOW_STATE_IDLE:
      // If oven temperature is still above room temperature
      if (input >= TEMPERATURE_ROOM)
      {
        reflowState = REFLOW_STATE_TOO_HOT;
      }
      else
      {
        // If switch is pressed to start reflow process
        if (switchStatus == SWITCH_1)
        {
          // Send header for CSV file
          Serial.println("Time,Setpoint,Input,Output");
          // Intialize seconds timer for serial debug information
          timerSeconds = 0;
          // Initialize PID control window starting time
          windowStartTime = millis();
          // Ramp up to minimum soaking temperature
          setpoint = TEMPERATURE_SOAK_MIN;
          // Load profile specific constant
          if (reflowProfile == REFLOW_PROFILE_LEADFREE)
          {
            soakTemperatureMax = TEMPERATURE_SOAK_MAX_LF;
            reflowTemperatureMax = TEMPERATURE_REFLOW_MAX_LF;
            soakMicroPeriod = SOAK_MICRO_PERIOD_LF;
          }
          else
          {
            soakTemperatureMax = TEMPERATURE_SOAK_MAX_PB;
            reflowTemperatureMax = TEMPERATURE_REFLOW_MAX_PB;
            soakMicroPeriod = SOAK_MICRO_PERIOD_PB;
          }
          // Tell the PID to range between 0 and the full window size
          reflowOvenPID.SetOutputLimits(0, windowSize);
          reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
          // Turn the PID on
          reflowOvenPID.SetMode(AUTOMATIC);
          // Proceed to preheat stage
          reflowState = REFLOW_STATE_PREHEAT;
        }
      }
      break;

    case REFLOW_STATE_PREHEAT:
      reflowStatus = REFLOW_STATUS_ON;
      // If minimum soak temperature is achieve
      if (input >= TEMPERATURE_SOAK_MIN)
      {
        // Chop soaking period into smaller sub-period
        timerSoak = millis() + soakMicroPeriod;
        // Set less agressive PID parameters for soaking ramp
        reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
        // Ramp up to first section of soaking temperature
        setpoint = TEMPERATURE_SOAK_MIN + SOAK_TEMPERATURE_STEP;
        // Proceed to soaking state
        reflowState = REFLOW_STATE_SOAK;
      }
      break;

    case REFLOW_STATE_SOAK:
      // If micro soak temperature is achieved
      if (millis() > timerSoak)
      {
        timerSoak = millis() + soakMicroPeriod;
        // Increment micro setpoint
        setpoint += SOAK_TEMPERATURE_STEP;
        if (setpoint > soakTemperatureMax)
        {
          // Set agressive PID parameters for reflow ramp
          reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
          // Ramp up to first section of soaking temperature
          setpoint = reflowTemperatureMax;
          // Proceed to reflowing state
          reflowState = REFLOW_STATE_REFLOW;
        }
      }
      break;

    case REFLOW_STATE_REFLOW:
      // We need to avoid hovering at peak temperature for too long
      // Crude method that works like a charm and safe for the components
      if (input >= (reflowTemperatureMax - 5))
      {
        // Set PID parameters for cooling ramp
        reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
        // Ramp down to minimum cooling temperature
        setpoint = TEMPERATURE_COOL_MIN;
        // Proceed to cooling state
        reflowState = REFLOW_STATE_COOL;
      }
      break;

    case REFLOW_STATE_COOL:
      // If minimum cool temperature is achieve
      if (input <= TEMPERATURE_COOL_MIN)
      {
        // Retrieve current time for buzzer usage
        buzzerPeriod = millis() + 1000;
        // Turn on buzzer to indicate completion
        digitalWrite(buzzerPin, HIGH);
        // Turn off reflow process
        reflowStatus = REFLOW_STATUS_OFF;
        // Proceed to reflow Completion state
        reflowState = REFLOW_STATE_COMPLETE;
      }
      break;

    case REFLOW_STATE_COMPLETE:
      if (millis() > buzzerPeriod)
      {
        // Turn off buzzer
        digitalWrite(buzzerPin, LOW);
        // Reflow process ended
        reflowState = REFLOW_STATE_IDLE;
      }
      break;

    case REFLOW_STATE_TOO_HOT:
      // If oven temperature drops below room temperature
      if (input < TEMPERATURE_ROOM)
      {
        // Ready to reflow
        reflowState = REFLOW_STATE_IDLE;
      }
      break;

    
  }

  // If switch 1 is pressed
  if (switchStatus == SWITCH_1)
  {
    // If currently reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Button press is for cancelling
      // Turn off reflow process
      reflowStatus = REFLOW_STATUS_OFF;
      // Reinitialize state machine
      reflowState = REFLOW_STATE_IDLE;
    }
  }
  // Switch 2 is pressed
  else if (switchStatus == SWITCH_2)
  {
    // Only can switch reflow profile during idle
    if (reflowState == REFLOW_STATE_IDLE)
    {
      // Currently using lead-free reflow profile
      if (reflowProfile == REFLOW_PROFILE_LEADFREE)
      {
        // Switch to leaded reflow profile
        reflowProfile = REFLOW_PROFILE_LEADED;
        EEPROM.write(PROFILE_TYPE_ADDRESS, 1);
      }
      // Currently using leaded reflow profile
      else
      {
        // Switch to lead-free profile
        reflowProfile = REFLOW_PROFILE_LEADFREE;
        EEPROM.write(PROFILE_TYPE_ADDRESS, 0);
      }
    }
  }
  // Switch status has been read
  switchStatus = SWITCH_NONE;

  // Simple switch debounce state machine (analog switch)
  switch (debounceState)
  {
    case DEBOUNCE_STATE_IDLE:
      // No valid switch press
      switchStatus = SWITCH_NONE;
      switchValue = readSwitch();
      //Serial.print("case: IDLE, switchvalue: "); Serial.println(switchValue); //print value of switch
      // If either switch is pressed
      if (switchValue != SWITCH_NONE)
      {
        // Keep track of the pressed switch
        //Serial.println("Button pressed, setting debouncestate to DEBOUNCE_STATE_CHECK"); 
        switchMask = switchValue;
        // Intialize debounce counter
        lastDebounceTime = millis();
        // Proceed to check validity of button press
        debounceState = DEBOUNCE_STATE_CHECK;
      }
      break;

    case DEBOUNCE_STATE_CHECK:
      switchValue = readSwitch();
      //Serial.print("case: CHECK, switchvalue: "); Serial.println(switchValue); //print value of switch
      if (switchValue == switchMask)
      {
        // If minimum debounce period is completed
        if ((millis() - lastDebounceTime) > DEBOUNCE_PERIOD_MIN)
        {
          // Valid switch press
          switchStatus = switchMask;
          // Proceed to wait for button release
          debounceState = DEBOUNCE_STATE_RELEASE;
        }
      }
      // False trigger
      else
      {
        // Reinitialize button debounce state machine
        debounceState = DEBOUNCE_STATE_IDLE;
      }
      break;

    case DEBOUNCE_STATE_RELEASE:
      switchValue = readSwitch();
      if (switchValue == SWITCH_NONE)
      {
        // Reinitialize button debounce state machine
        debounceState = DEBOUNCE_STATE_IDLE;
      }
      break;
  }

  // PID computation and SSR control
  if (reflowStatus == REFLOW_STATUS_ON)
  {
    now = millis();

    reflowOvenPID.Compute();

    if ((now - windowStartTime) > windowSize)
    {
      // Time to shift the Relay Window
      windowStartTime += windowSize;
    }
    if (output > (now - windowStartTime)) digitalWrite(ssrPin, HIGH);
    else digitalWrite(ssrPin, LOW);
  }
  // Reflow oven process is off, ensure oven is off
  else
  {
    digitalWrite(ssrPin, LOW);
  }
}

switch_t readSwitch(void)
{
  int switchAdcValue = 0;

  switchAdcValue = analogRead(switchPin);
  //Serial.print("Value of switchAdcValue: "); Serial.println(switchAdcValue); //print value of switch
  // Add some allowance (+10 ADC step) as ADC reading might be off a little 
  // due to 3V3 deviation and also resistor value tolerance
  if (switchAdcValue >= 870) return SWITCH_NONE;
  if (switchAdcValue <= 10) return SWITCH_1;
  if (switchAdcValue <= 550) return SWITCH_2;

  return SWITCH_NONE;
}