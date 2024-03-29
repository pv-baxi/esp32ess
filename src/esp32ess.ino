/*
          Victron Multiplus 2 ESS using ESP32 controlling VE.Bus

Implements an ESS system with Victron Multiplus II and Pylontech batteries.
Instead by a Cerbo GX, the Multiplus is directly controlled by the ESP32 via
the VE.Bus.

Documentation and latest source code can be found here:
https://github.com/pv-baxi/esp32ess

All ASCII art is generated using the following generator with font "Standard":
https://patorjk.com/software/taag/#p=display&f=Standard&t=

Disclaimer:
Use this code at your own risk. I don't take any responsibility for damage
this code or the associated hardware might cause. Note that electrical
installations are only allowed to be done (or modified) by persons having the
skills and permission to do that.

SPDX-FileCopyrightText: © 2023 PV Baxi <pv-baxi@gmx.de>
SPDX-License-Identifier: GPL-3.0-or-later
*/

#include <LiquidCrystal.h>
#include "driver/twai.h"
#include <WebServer.h>
#include <ESPmDNS.h>
#include "CRC16.h"
#include <HTTPClient.h>

// ################################################
//    ____                _              _       
//   / ___|___  _ __  ___| |_ __ _ _ __ | |_ ___ 
//  | |   / _ \| '_ \/ __| __/ _` | '_ \| __/ __|
//  | |__| (_) | | | \__ \ || (_| | | | | |_\__ \
//   \____\___/|_| |_|___/\__\__,_|_| |_|\__|___/
//
// ################################################

//Wifi password
#include "my_wifi_pw.h"   // <-- create a header file in same directory, only containing the following two lines with your WiFi password
//const char* SSID = "your_SSID";
//const char* PASSWORD = "your_WiFi_password";

//Other
const int BUTTON_GPIO = 0;                            //on-devkit-board button is connected to GPIO 0
const int RS=16, EN=13, D4=4, D5=5, D6=14, D7=15;     //HD44780 text display gpio pins
const int CAN_RX_PIN = 22, CAN_TX_PIN = 21;           //Pylontech battery CAN = TWAI bus gpio pins
const int VEBUS_RXD1=26, VEBUS_TXD1=25, VEBUS_DE=27;  //Victron Multiplus VE.bus RS485 gpio pins
const int METER_IMPULSE_INPUT_PIN = 35;               //input pin from optical 1/10000kWh impulse output of power meter
const int METER_SML_INPUT_PIN = 34;                   //input pin from optical SML data output of power meter
const int SML_TXD_UNUSED = 17;                        //some GPIO we need to wire the unused UART TX to
const int RED_LED = 2;                                //output gpio pin with external red LED (active low)
const int CPS = 10000;                  //CyclesPerSecond = How often per second our timer ISR is excuted
const int WAITING_TIME_ACK = 0.9*CPS;   //How long we wait for the ESS command to be acknowledged by Multiplus. Comes normally between 25..43ms, so 1000 = 100ms makes sense
const int CHARGE_LIMIT = 3300;          //Maximum Multiplus sink/charging power (70A * min. battery charging voltage), maximum is 3300W by experiment
const int CHARGE_LIMIT_STEP = 200;      //step away from charge-limit (-Watt)
const int DISCHARGE_LIMIT = 4300;       //Maximum Multiplus power feeding into the grid, maximum is 4300W by experiment
const int DISCHARGE_LIMIT_STEP = 200;   //step away from discharge-limit (+Watt)
const int8_t SOC_LIMIT_UPPER = 96;      //turn off extra charging when battery is >= 96%
const int8_t SOC_LIMIT_LOWER = 6;       //turn off extra discharge at SOC <= 6%, but still allow extra charging (except no-charge-mode)
const int8_t SOC_LIMIT_LOWEST = 3;      //stop ESS state machine and sending ESS commands if SOC <= 3% and also stop logging soon
const int8_t SOC_CHARGE_ONLY_VS_0W = 25;   //25% switch to charge-only if below, or back to Inverter+Charger if above this battery level
const int8_t SOC_0W_VS_MIN_STRATEGY = 0;  //35% start using minimum strategy above this SOC
const int8_t SOC_MIN_VS_IMM_STRATEGY = 90; //90% start using immediate strategy above this SOC (disabled if same as maximum strategy)
const int8_t SOC_IMM_VS_MAX_STRATEGY = 90; //90% start using maximum strategy above this SOC (When it's sure that battery will be full again next day, lower threshold can be used.)
const int SOC_LIMIT_UPPER_STEP = 100;   //discharge step away from 0W in case battery full (Watt)
const int SOC_LIMIT_LOWER_STEP = 100;   //charge step away from 0W in case battery empty (Watt)
const int NO_CHARGE_MODE_STEP = 100;    //discharge step away from 0W if in no-charge-mode (Watt)
const int MAX_METER_DIFFERENCE = 200;   //if difference between current and last measurent > 100W, measure again to ignore short spikes (Watt)
const int ESS_POWER_OFFSET = +5;        //after each ESS power calculation, add this offset to feed a little bit more into the grid (e.g. +2W)
const int ESS_TIMEOUT_PREVENT = 10*CPS; //Multiplus ESS is automatically disabled about 15 sec after last command written (FW v508), so every 10s we should re-send ESS power
const int BUTTON_DEBOUNCE_TIME = 0.1*CPS;//0.1 seconds gives good results
const int BUTTON_LONGPRESS_TIME = 3.0*CPS;//3.0 seconds for a longpress
const int LOGFILE_SIZE = 65510;         //Maximum text buffer that can be sent via HTTP is 65519 byte (2^16 -1 -16)
const int SOC_WATCHDOG_CYCLES = 2.5*CPS;//time after which Multiplus is turned off in case there was no SOC received from battery (2.5 seconds)
const char SWITCH_MODE_DEFAULT = 'i';   //Default switch mode to start with. For mode characters see function switchSpecialModes(); Use '0' in winter;
const int SWITCH_MODE_DURATION = 60*60*CPS;//automatically exit specific the switch modes after 60 minutes
const float NOM_VOLT = 48.0;            //Battery nominal voltage (for Watt/Wh estimation)
const int ELECTRIC_METER_TIME_OFFSET = -17*60*60 -4*60 -13;  //To get time from meter runtime value; 21.2.2024:-17:04:13



// #############################################################################
//    ____ _       _           _                    _       _     _           
//   / ___| | ___ | |__   __ _| |  __   ____ _ _ __(_) __ _| |__ | | ___  ___ 
//  | |  _| |/ _ \| '_ \ / _` | |  \ \ / / _` | '__| |/ _` | '_ \| |/ _ \/ __|
//  | |_| | | (_) | |_) | (_| | |   \ V / (_| | |  | | (_| | |_) | |  __/\__ \
//   \____|_|\___/|_.__/ \__,_|_|    \_/ \__,_|_|  |_|\__,_|_.__/|_|\___||___/
//
// #############################################################################

//mode options:
boolean extraCharge = false;          //allow/disallow negative ESS power (extra charging) to avoid sourcing additional current from the grid
//other variables:
int displayCounter = 0;             //counts how often LC display was updated
char frbuf0[256];                   //assembles one complete frame received by Multiplus
char frbuf1[256];                   //frame without replacement
char txbuf1[64];                    //buffer for assembling bare command towards Multiplus (without replacements or checksum)
char txbuf2[64];                    //Multiplus output buffer containing the final command towards Multiplus, including replacements and checksum
//char str[3*128+1];                //Buffer to temporarily store commands from/to Multiplus in HEX format and send it to SerialMonitor for debugging. +1 for being able to terminate our string with zero
uint16_t frp = 0;                   //Pointer into Multiplus framebuffer frbuf[] to store received frames.
byte frameNr = 0;                   //Last frame number received from Multiplus. Own command has be be sent with frameNr+1, otherwise it will be ignored by Multiplus.
int synccnt = 0;                    //Counts every 5 received sync frames in veBusHandling(), sending CommandReadRAMVar 10 times per second
int veCmdCounter = 0;               //Counts the number of ESS commands sent to Multiplus, including the failed ones.
short essPowerDesired = 0;          //This value will be written into Multiplus when setting veCmdSendState = 2; negative = charging battery; positive = feed into grid
short essPowerTmp = 0;              //temporary value during ESS power calcuation
int meterPower = 0;                 //result of newest power measurement
int meterPowerPrevious = 0;         //result of previous power measurement for spike detection
int meterPowerOld = 0;              //for comparison in usingAbsoluteMeter() calculation if we got better or worse
float essR = 0.998;                 //Multiplus should not apply more power than meter shows, to keep regulation stable. In my case, Multiplus power is about 3.7% to 7.5% less than meter power. So setting to 1.00 is fine.
                                    //Baxi=0.998 ; newMultiplusESSpower = essR * meterPower;
byte veCmdSendState = 0;            //2 ready to send; 0x1X has been send but no ACK yet; 0 acknowledged=done; 7 set Charger+Discharger mode; 5 set Charge-Only mode
int veTxCmdFailCnt = 0;             //Counts the number of commands that were not acknowledged by Multiplus and had to be resent.
int veRxCmdFailCnt = 0;             //amount of received VE.Bus commands with wrong checksum
char switchMode = SWITCH_MODE_DEFAULT;  //Switches between different regulation modes (special modes), see SWITCH_MODE_DEFAULT
char logfile[LOGFILE_SIZE];         //logfile buffer, used as ringbuffer with pointer below
int p_logfile = 0;                  //pointer to logfile ringbuffer
int logfileCounter = 0;             //counts every byte written into logfile
int logfileCounterStop = 0;         //in case logging has to stop soon (to keep failure log), this symbol contains the maximum logfile size when logging has to stop.
boolean batHealthWritten = false;   //Flag for writing battery health once into logfile
boolean stopWritingLogfile = false; //writing to logfile will be stopped in X bytes in order to not overwrite important events
int debugmin = 999999;              //used to log minimum battery level
int debugmax = -999999;             //used to log maximum battery level
int8_t soc = -1;                    //State Of Charge = received battery charge level via CAN bus (0..100%), -1 = invalid / not available
boolean newMeterValue = false;      //indicating to various functions that we got a new power meter measurement result
char printFrame[16];                //part of a frame, to be printed on LCD
char sBuf[1024];                    //Circular buffer, stores received SML stream from power meter; Size must be power of 2 and bigger (or equal) than SML_LENGTH_MAX
uint16_t smlp = 0;                  //Pointer into SML buffer, used as circular pointer
const int SML_LENGTH_MAX = 512;     //This is the maximum length we allow for the SML stream
char sBuf2[SML_LENGTH_MAX];         //linear buffer with SML stream
uint16_t smlCnt = 0;                //counter to count length of SML stream
uint16_t smlLength = 0;             //detected length of SML stream
int smlLengthDisplay = 0;           //Show SML length on LCD if > 0 
//Classes
TaskHandle_t Task0;
hw_timer_t *My_timer = NULL;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
WebServer server(80);
CRC16 crc(0x1021, 0xFFFF, 0xFFFF, true, true);
//Multiplus variables
short   multiplusESSpower = -1;                  //ESS power value currently applied in Multiplus, init with -1, so that it will be very likely updated on start
float   multiplusTemp = 11.1;
float   multiplusDcCurrent = -22.2;
float   dcCurrentMin = 0.0;
float   dcCurrentMax = 0.0;
int16_t multiplusBatteryAh = -12345;
byte    multiplusBattery_byte05;
byte    multiplusBattery_byte06;
byte    multiplusBattery_byte07;
float   dcVoltageMin = 999;
float   dcVoltageMax = -999;
float    acFrequencyMin = 200;
uint32_t timeAcFrequencyMin = -ELECTRIC_METER_TIME_OFFSET;
float    acFrequencyMax = 0;
uint32_t timeAcFrequencyMax = -ELECTRIC_METER_TIME_OFFSET;
float    acVoltageMin = 999;
uint32_t timeAcVoltageMin = -ELECTRIC_METER_TIME_OFFSET;
float    acVoltageMax = -999;
uint32_t timeAcVoltageMax = -ELECTRIC_METER_TIME_OFFSET;
float   multiplusTempMin = 999;
float   multiplusTempMax = -999;
byte    multiplusStatus80 = 23;                 //00=ok, 02=battery low
byte    multiplusVoltageStatus;                 //11=ACin_disconnected, 12=inverter_off, 13=Battery_good_inverter_on, 16=will not invert, 17=Moment_of_ACin_interruption, 19=Moment_of_ACin_back_on
                                                //If Multiplus starts with "mains on" blinking, it will not allow inverting, even if it is in Charge+Invert mode. Then multiplusVoltageStatus = 16/17 depending on the mode.
                                                //If Multiplus starts correctly with "inverter on" and then synchronizing to mains, then multiplusVoltageStatus = 12/13 depending on the mode.
byte    multiplusEmergencyPowerStatus;          //00=mains, 02=in_emergency_power_mode (=state of ACin relay)
byte    masterMultiLED_LEDon = 123;             //Bits 0..7 = mains on, absorption, bulk, float, inverter on, overload, low battery, temperature
byte    masterMultiLED_LEDblink = 234;          //(LEDon=1 && LEDblink=1) = blinking; (LEDon=0 && LEDblink=1) = blinking_inverted 
byte    masterMultiLED_Status = 12;             //0=ok, 2=battery low
byte    masterMultiLED_AcInputConfiguration;    //0x10=1st_AC_input_no_mode_change, 0x90=1st_AC_input_after_mode_change
float   masterMultiLED_MinimumInputCurrentLimit;
float   masterMultiLED_MaximumInputCurrentLimit;//for details on MasterMultiLED frame see Victron MK2 protocol PDF
float   masterMultiLED_ActualInputCurrentLimit;
byte    masterMultiLED_SwitchRegister;          //See Victron MK2 manual chapter 6.2; bits: 0x01=Charger_on_received, 0x02=Inverter_on_received, 0x10=Charger_finally_on, 0x20=Inverter_finally_on
float    multiplusAcFrequency;
uint32_t multiplusE4_Timestamp;                 //unit = 400ns; 49980 between two E4 frames coming every 20ms; 2498999 for 50 frames = 1 second
byte     multiplusE4_byte11;                    //B0=Mode0x55@0.0A, B5=Mode0x55@Charging, B8=Mode0x77@Standby/Timeout, B9=Mode0x77@Discharging, BE=Mode0x77@Charging, 49=EmergencyPower
byte     multiplusE4_byte12;
float    multiplusPowerFactor;
int8_t   multiplusAcPhase;                      //AC phase angle, circles around the full 8bit range
float    multiplusDcVoltage;
byte     multiplusE4_byte17;
byte     multiplusE4_byte18;
float    multiplusUMainsRMS;
float    multiplusIMainsRMS;
int16_t  multiplusPmainsFiltered;

//Power meter variables
char     electricMeter_DeviceID[10];                         //electric meter device ID (we assume 10 bytes)
char     electricMeter_Status180[3];                         //Status sent with 1.8.0 (consumption)
char     electricMeter_Status280[3];                         //Status sent with 2.8.0 (feed-in)
double   electricMeter_Consumption = 0;                      //kWh value 1.8.0
double   electricMeter_FeedIn = 0;                           //kWh value 2.8.0
uint32_t electricMeter_Runtime = -ELECTRIC_METER_TIME_OFFSET;  //meter runtime in seconds; initialize to 00:00:00 real time
double    electricMeter_Power = 0;
double    electricMeter_PowerL1 = 0;
double    electricMeter_PowerL2 = 0;
double    electricMeter_PowerL3 = 0;
int      electricMeterStatusDifferent = 0;                   //to find out if there is ever any difference between Status 1.8.0 and 2.8.0
int      electricMeterCRCwrong = 0;                          //count every SML stream that was discarded due to wrong CRC
int      electricMeterSignPositive = 0;                      //how often within the last power measurement interval we got a positive sign from status
int      electricMeterSignNegative = 0;                      //how often within the last power measurement interval we got a negative sign from status
int      electricMeterCurrentSign = +1;                      //by default we assume consumption
short    estTargetPower = 0;                                 //current target from electric meter and powerACin for ESS power 
int      essPowerStrategy = 0;                               //7=maximum, 5=immediate, 3=minimum, 0=0W
const int SIZE_PWR_RINGBUF = 30;                             //30 seconds
short estTargetPowerRingBuf[SIZE_PWR_RINGBUF] = {0};         //ringbuffer of estTargetPower of the last SIZE_PWR_RINGBUF seconds
int ptrPowerRingBuf = 0;                                     //start with 1st element in ringBuffer
const int SIZE_P_ACIN_RINGBUF = 32;                          //How many last multiplusPmainsFiltered values we want to store. Must be power of 2 !
int16_t pACinRingBuf[SIZE_P_ACIN_RINGBUF] = {0};             //ring buffer of last multiplusPmainsFiltered values, coming due to CommandReadRAMVar every 100ms
int ptr_pACinRingBuf = 0;                                    //start with 1st element in ringBuffer
double powerACin = 0;                                        //power on ACin, synchronized from pACinRingBuf[], with value from electric meter

int8_t shellyStatus = -1;                                    //-1=unknown, 0=off, 1=on
const int SIZE_P_METER_DC_RINGBUF = 60;                         //Must be definitely more than SIZE_PWR_RINGBUF !! to also work with maximum strategy
int16_t pMeterRingBuf[SIZE_P_METER_DC_RINGBUF] = {0};
int16_t pDcRingBuf[SIZE_P_METER_DC_RINGBUF] = {0};
int ptr_pMeterDcRingBuf = 0;





// ##########################################################################################
//   ___       _                             _      _                     _ _ _             
//  |_ _|_ __ | |_ ___ _ __ _ __ _   _ _ __ | |_   | |__   __ _ _ __   __| | (_)_ __   __ _ 
//   | || '_ \| __/ _ \ '__| '__| | | | '_ \| __|  | '_ \ / _` | '_ \ / _` | | | '_ \ / _` |
//   | || | | | ||  __/ |  | |  | |_| | |_) | |_   | | | | (_| | | | | (_| | | | | | | (_| |
//  |___|_| |_|\__\___|_|  |_|   \__,_| .__/ \__|  |_| |_|\__,_|_| |_|\__,_|_|_|_| |_|\__, |
//                                    |_|                                             |___/ 
// ##########################################################################################

//Symbols used in ISR:
volatile int cmdAckCnt = 0;                 //counter down-counting the time it takes for Multiplus acknowledging an ESS command
volatile int essTimeoutCounter = ESS_TIMEOUT_PREVENT; //Multiplus disables ESS if no value is written within 15 seconds
volatile int socWatchdog = 20000;           //timer to turn off Multiplus in case there was no SOC received from battery; start with 2 seconds, so that no CAN bus error is displayed on reboot
volatile int specialModeCnt = SWITCH_MODE_DURATION;   //timer to automatically turn off special mode after XX minutes
volatile int buttonPressCnt = -1;           //Initialize button-press counter in locked (paused) state
volatile int cylTime = CPS/10;              //counts down within 1/10 of a second and increases relTime when 0 is reached
volatile int relTime = 0;                   //relative time, unit = 100ms, on 1000 resets to 0
volatile int oneSecondTimer = 0;            //used to generate flag oneSecondOver every second
volatile bool oneSecondOver = false;        //set to true every second (so save DC and meter power into ringbuffer for Shelly)
volatile bool oneMinuteOver = false;        //set to true every minute (to write the time into logfile)
//for digital meter pulse measurement:
volatile boolean IRpinDisplay = LOW;        //remember state of IR input pin to let LED blink once in main loop
volatile int highCnt = 1000;                //how long the high-pulse took. Normal is 20 = 2ms. Initialize with higher value, meaning no valid measurement
volatile int totalCnt = 4000000;            //how long the complete measurement (high-pulse plus low time) took. Initialize with some very high value meaning very low power
volatile int highCycles = 0;                //length of the concluding high pulse
volatile int highCyclesPrevious = 0;        //to validate if a power measurement also include the length of the preceding high pulse
volatile int totalCycles = 0;               //length of complete power measurent (high+low pulse)
volatile boolean meterHighPulse = false;    //current state: true = within high-pulse; false = within low pulse
volatile boolean isrNewMeterPulse = false;  //indicating that new meter impulse came in

// ===========================================================================
//                Timer ISR - executed 10000 times per second
// ---------------------------------------------------------------------------
// This ISR has two tasks:
// 1) It runs several 10kHz timers for different purposes
// 2) It measures current absolute meter power using optical impulse output
// ===========================================================================
void IRAM_ATTR onTimer()
{
  if ((buttonPressCnt >= 0) && (buttonPressCnt < (30.0*CPS))) buttonPressCnt++;  //nobody will press the button longer than 30 seconds
  if (essTimeoutCounter > 0) essTimeoutCounter--;
  if (cmdAckCnt > 0) cmdAckCnt--;
  if (socWatchdog > 0) socWatchdog--;     //watchdog timer ensuring that SOC value is received from battery
  if (specialModeCnt > 0) specialModeCnt--;   //Comment out to keep Special Mode infintitely until next button press
  cylTime--;
  if (cylTime <= 0)
  {
    cylTime = (CPS/10);  //  reset to 1/10 of a second
    relTime++;           //  this is a 1/10 second counter
    if (relTime >= 600) {
      relTime = 0;   //overflow at 600 units = 60 seconds
      oneMinuteOver = true;
    }
  }
  oneSecondTimer++;
  if (oneSecondTimer == CPS) {
    oneSecondOver = true;
    oneSecondTimer = 0;
  }
  //============================================================================================================
  //The following code is for reading the optical 1/10000kWh impulses from the power meter.
  // Advantage: Works with every digital power meter. No pin-code required.
  // Disadvantage: Power value is always positive, there is no information if currently consumption or feed-in.
      totalCnt++;
      highCnt++;
      boolean IRpin = digitalRead(METER_IMPULSE_INPUT_PIN); //read IR input at this moment
      if (IRpin==HIGH) IRpinDisplay=HIGH;   //if it was high, remember to display later (but don't reset to LOW here)
      if ((!meterHighPulse) && (IRpin == HIGH))    //if we are within the low-period and detect a high-pulse
      {
        //on the beginning of a 2ms high-pulse we do:
        totalCycles=totalCnt;
        highCnt = 0;
        totalCnt = 0;
        meterHighPulse = true;
      }
      if ((meterHighPulse) && (IRpin == LOW))    //if we are within high-pulse and detect low...
      {
        //on the beginning of the low-period we do:
        highCyclesPrevious = highCycles;
        highCycles = highCnt;
        meterHighPulse = false;
        isrNewMeterPulse = true;   //indicating that new meter impulse came in and thus new impulse power value might be available
      }
  // End of optical 1/10000kWh measurement code
  //============================================================================================================
}



// ###################################################
//     _____                 _   _                 
//    |  ___|   _ _ __   ___| |_(_) ___  _ __  ___ 
//    | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
//    |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
//    |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
//                                                 
// ###################################################


// ===========================================================================
//                      Adds string to logfile buffer
// ---------------------------------------------------------------------------
// After string is added to ringbuffer, pointer is incremented and wrapped if
// needed. Function does NOT add \n, this needs to be added manually.
// Number of bytes written to logfile are counted to be shown on main webpage.
// ===========================================================================
void addToLogfile(String s)
{
  if (!stopWritingLogfile || (logfileCounter < logfileCounterStop))
  {
    for (int i=0; i<s.length(); i++)
    {
      logfile[p_logfile++]=s[i];
      if (p_logfile==LOGFILE_SIZE) p_logfile=0;   //wrap pointer around at end of buffer
      logfileCounter++;   //increase logfile size counter
    }
  }
}

void addFrameToLogfile(char *frame, int startbyte, int endbyte)
{
  if (!stopWritingLogfile || (logfileCounter < logfileCounterStop))
  {
    logfile[p_logfile++]='\n';
    if (p_logfile==LOGFILE_SIZE) p_logfile=0;   //wrap pointer around at end of buffer
    logfileCounter++;   //increase logfile size counter
    for (int i=startbyte; i<=endbyte; i++)
    {
      char c = frame[i];
      char n = (c >> 4) | 48;
      if (n > 57) n = n+7;
      logfile[p_logfile++]=n;
      if (p_logfile==LOGFILE_SIZE) p_logfile=0;   //wrap pointer around at end of buffer
      logfileCounter++;   //increase logfile size counter
      n = (c & 0x0F) | 48;
      if (n > 57) n = n+7;
      logfile[p_logfile++]=n;
      if (p_logfile==LOGFILE_SIZE) p_logfile=0;   //wrap pointer around at end of buffer
      logfileCounter++;   //increase logfile size counter
      logfile[p_logfile++]=' ';
      if (p_logfile==LOGFILE_SIZE) p_logfile=0;   //wrap pointer around at end of buffer
      logfileCounter++;   //increase logfile size counter
    }
  }
}



// ===========================================================================
//                   Assemble write-ESS-power VE.Bus command
// ---------------------------------------------------------------------------
// This function assembles the basic command to write the desired power towards
// the Multiplus ESS assistant. Documentation is here:
// https://github.com/pv-baxi/esp32ess/blob/main/docs/README.md#vebus-send-frames
// ===========================================================================
int prepareESScommand(char *outbuf, short power, byte desiredFrameNr)
{
  byte j=0;
  outbuf[j++] = 0x98;           //MK3 interface to Multiplus
  outbuf[j++] = 0xf7;           //MK3 interface to Multiplus
  outbuf[j++] = 0xfe;           //data frame
  outbuf[j++] = desiredFrameNr;
  outbuf[j++] = 0x00;           //our own ID
  outbuf[j++] = 0xe6;           //our own ID
  outbuf[j++] = 0x37;           //CommandWriteViaID
  outbuf[j++] = 0x02;           //Flags, 0x02=RAMvar and no EEPROM
  outbuf[j++] = 0x83;           //ID = address of ESS power in assistand memory
  outbuf[j++] = (power & 0xFF); //Lo value of power (positive = into grid, negative = from grid)
  outbuf[j++] = (power >> 8);   //Hi value of power (positive = into grid, negative = from grid)
  return j;
}

// ===========================================================================
//              Assemble switch-Multiplus-mode VE.Bus command
// ---------------------------------------------------------------------------
// switchState:
// 0x04 = Sleep
// 0x05 = Charger-Only mode
// 0x06 = Inverter-Only mode (turn AC-in off!)
// 0x07 = Charger+Inverter mode (normal ON mode)
// Note: From Inverter-Only mode, which is like emergency-power mode, you need
//       to switch to Charger+Inverter mode first. Then wait ca. 3 minutes
//       until Multiplus has synchronized to AC-in again.
// ===========================================================================
int prepareSwitchCommand(char *outbuf, byte switchState, byte desiredFrameNr)
{
  byte j=0;
  outbuf[j++] = 0x98;           //MK3 interface to Multiplus
  outbuf[j++] = 0xf7;           //MK3 interface to Multiplus
  outbuf[j++] = 0xfe;           //data frame
  outbuf[j++] = desiredFrameNr;
  outbuf[j++] = 0x3F;           //VE.Bus switch Multiplus command
  outbuf[j++] = switchState;
  outbuf[j++] = 0x00;
  outbuf[j++] = 0x00;
  outbuf[j++] = 0x00;
  return j;
}

int prepareCommandReadRAMVar(char *outbuf, byte desiredFrameNr)
{
  byte j=0;
  outbuf[j++] = 0x98;           //MK3 interface to Multiplus
  outbuf[j++] = 0xf7;           //MK3 interface to Multiplus
  outbuf[j++] = 0xfe;           //data frame
  outbuf[j++] = desiredFrameNr;
  outbuf[j++] = 0x00;           //our own ID
  outbuf[j++] = 0xe6;           //our own ID
  outbuf[j++] = 0x30;           //CommandReadRAMVar
  outbuf[j++] = 0;              //1st RAM ID (up to 6 possible)
  outbuf[j++] = 15;             //2nd RAM ID
  return j;
}

int prepareCommandGetRAMVarInfo(char *outbuf, byte desiredFrameNr)
{
  byte j=0;
  outbuf[j++] = 0x98;           //MK3 interface to Multiplus
  outbuf[j++] = 0xf7;           //MK3 interface to Multiplus
  outbuf[j++] = 0xfe;           //data frame
  outbuf[j++] = desiredFrameNr;
  outbuf[j++] = 0x00;           //our own ID
  outbuf[j++] = 0xe6;           //our own ID
  outbuf[j++] = 0x36;           //CommandReadRAMVar
  outbuf[j++] = 6;              //<Lo(RAM ID)>
  outbuf[j++] = 0;              //<Hi(RAM ID)>
  return j;
}



// ===========================================================================
//                        VE.Bus command replace FA..FF
// ---------------------------------------------------------------------------
// After the basic VE.Bus commmand has been assembled, this function searches
// through the command bytes following the frame number and replaces bytes
// between 0xFA and 0xFF with the following two-bytes sequence:
// 0xFA -> 0xFA 0x7A
// 0xFB -> 0xFA 0x7B
// 0xFC -> 0xFA 0x7C
// 0xFD -> 0xFA 0x7D
// 0xFE -> 0xFA 0x7E
// 0xFF -> 0xFA 0x7F
// ===========================================================================
int commandReplaceFAtoFF(char *outbuf, char *inbuf, int inlength)
{
  int j=0;
  //copy over the first 4 bytes of command, as there is no replacement
  for (int i = 0; i < 4; i++)
  {
    outbuf[j++] = inbuf[i];
  } 
  //starting from 5th byte, replace 0xFA..FF with double-byte character
  for (int i = 4; i < inlength; i++)
  {
    byte c = inbuf[i];
    if (c >= 0xFA)
    {
      outbuf[j++] = 0xFA;
      outbuf[j++] = 0x70 | (c & 0x0F);
    }
    else
    {
      outbuf[j++] = c;    //no replacement
    }
  }
  return j;   //new length of output frame
}


int destuffFAtoFF(char *outbuf, char *inbuf, int inlength)
{
  int j=0;
  for (int i = 0; i < 4; i++) outbuf[j++] = inbuf[i];
  for (int i = 4; i < inlength; i++)
  {
    byte c = inbuf[i];
    if (c == 0xFA)
    {
      i++;
      c = inbuf[i];
      if (c == 0xFF)    //if 0xFA is the checksum, leave the FA and following FF (end of frame) in as it was.
      {                 //The rule comes from the fact that for checking the checksum just the sum of all bytes must be zero. That's why the 0x00 behind the 0xFA is not appended as an exception.
        outbuf[j++] = 0xFA;
        outbuf[j++] = c;
      }
      else outbuf[j++] = c + 0x80;
    }
    else outbuf[j++] = c;    //no replacement
  }  
  return j;   //new length of output frame
}

bool verifyChecksum(char *inbuf, int inlength)
{
  byte cs = 0;
  for (int i = 2; i < inlength; i++) cs += inbuf[i];  //sum over all bytes excluding the first two (address)
  if (cs == 0) return true; else return false;
}

// ===========================================================================
//                   Append checksum and 0xFF to command
// ---------------------------------------------------------------------------
// After the command has been assembled and 0xFA..FF has been replaced, a
// checksum needs to be added. Documentation is here:
// https://github.com/pv-baxi/esp32ess/blob/main/docs/README.md#checksum-calculation
// ===========================================================================
int appendChecksum(char *buf, int inlength)
{
  int j=0;
  //calculate checksum starting from 3rd byte
  byte cs=1;
  for (int i = 2; i < inlength; i++)
  {
    cs -= buf[i];
  }
  j = inlength;
  if (cs >= 0xFB)   //EXCEPTION: Only replace starting from 0xFB, as inserting {0xFA 0x00} wouldn't make sense as 0x00 doesn't affect the checksum!
  {
    buf[j++] = 0xFA;
    buf[j++] = (cs-0xFA);
  }
  else
  {
    buf[j++] = cs;
  }
  buf[j++] = 0xFF;  //append End Of Frame symbol
  return j;   //new length of output frame
}

// ===========================================================================
//                  Convert Multiplus bytes frame to HEX string
// ---------------------------------------------------------------------------
// Only needed for sending a received data or sync frame from Multiplus to the
// debug serial link for debugging purposes.
// Currently not used in the code.
// ===========================================================================
void convertFrameToHEXstring(char *inbuf, char *outstr, int length)
{
        int j = 0;  //begin with first character in outstr
        for (int i = 0; i < length; i++)
        {
          char c = inbuf[i];
          byte n = (c >> 4) | 48;
          if (n > 57) n = n+39;
          outstr[j++] = n;
          n = (c & 0x0F) | 48;
          if (n > 57) n = n+39;
          outstr[j++] = n;
          outstr[j++] = 32;
        }
        outstr[j] = 0;
}


//true if frame was known, false if unknown
bool decodeVEbusFrame(char *frame, int len)
{
  bool result = false;
  if ((frame[0]==0x83) && (frame[1]==0x83)) {     //Only accept frames from single Multiplus
    if ((frame[2] == 0xFD) && (len=10) && (frame[4] == 0x55))    //If sync frame
    {
      result = true;  //We simply ignore known sync frames
    }
    if (frame[2] == 0xFE)    //If data frame:
    {
      switch (frame[4]) {
        case 0x80:    //80 = Condition of Charger/Inverter (Temp+Current)
        {
          if ((len==19) && (frame[5]==0x80) && (frame[8]==0x80) && ((frame[11]&0x10)==0x10))
          {
            result = true; //known frame
            multiplusStatus80 = frame[7];
            multiplusVoltageStatus = frame[6];
            int16_t t = (frame[10]<<8) + frame[9];
            multiplusDcCurrent = t/10.0;
            if (multiplusDcCurrent < dcCurrentMin) dcCurrentMin = multiplusDcCurrent;
            if (multiplusDcCurrent > dcCurrentMax) dcCurrentMax = multiplusDcCurrent;
            multiplusEmergencyPowerStatus = frame[12];
            if ((frame[11] & 0xF0) == 0x30) {
              multiplusTemp = frame[15]/10.0*1.25;  //x1.25 as otherwise temperature is too low
              if (multiplusTemp < multiplusTempMin) multiplusTempMin = multiplusTemp;
              if (multiplusTemp > multiplusTempMax) multiplusTempMax = multiplusTemp;
            }
          }
          break;
        }
        case 0xE4:    //E4 = AC phase information (comes with 50Hz)
        {
          if ( (len==21) ) {// &&
            //  ((frame[11]==0xB0) || (frame[11]==0xB5) || (frame[11]==0xB8) || (frame[11]==0xB9) || (frame[11]==0xBE) || (frame[11]==0x49)) &&
            //  ((frame[12]==0x8F) || (frame[12]==0x0F) || (frame[12]==0x80)) &&
            //  ((frame[17]==0x79) || (frame[17]==0x19) || (frame[17]==0x59)) &&
            //  ((frame[18]==0x00) || (frame[18]==0x01) || (frame[18]==0x08) || (frame[18]==0x09)) ) {
            uint16_t ut = (frame[7]<<8) + frame[6];
            multiplusAcFrequency = ut / 1000.0;
            if (multiplusAcFrequency < acFrequencyMin) {
              acFrequencyMin = multiplusAcFrequency;
              timeAcFrequencyMin = electricMeter_Runtime;
            }
            if (multiplusAcFrequency > acFrequencyMax) {
              acFrequencyMax = multiplusAcFrequency;
              timeAcFrequencyMax = electricMeter_Runtime;
            }
            multiplusE4_Timestamp = (frame[10]<<16) + (frame[9]<<8) + frame[8];
            multiplusE4_byte11 = frame[11];
            multiplusE4_byte12 = frame[12];
            int16_t it = (frame[14]<<8) + frame[13];
            multiplusPowerFactor = it / 32768.0;
            multiplusAcPhase = frame[15];
            ut = ((frame[17] & 0x0F)<<8) + frame[16];
            multiplusDcVoltage = ut / 50.0;
            if (multiplusDcVoltage < dcVoltageMin) dcVoltageMin = multiplusDcVoltage;
            if (multiplusDcVoltage > dcVoltageMax) dcVoltageMax = multiplusDcVoltage;
            multiplusE4_byte17 = frame[17];
            multiplusE4_byte18 = frame[18];
            result = true; //known frame
          }
          break;
        }
        case 0x70:    //70 = DC capacity counter
        {
          if ((len==15) && (frame[8]==0xBC) && (frame[9]==0x02) && (frame[12]==0x00))
          {
            multiplusBatteryAh = (frame[11]<<8) + frame[10];  //maybe value is also 24bit including frame[12]?
            multiplusBattery_byte05 = frame[5];
            multiplusBattery_byte06 = frame[6];
            multiplusBattery_byte07 = frame[7];
            result = true; //known frame
          }
          break;
        }
        case 0x00:    //Acknowledgement to ESS power command
        {
          if ((len == 9) && (frame[5] == 0xE6) && (frame[6] == 0x87)) {
            if ((veCmdSendState==0x12) && (cmdAckCnt > 0))    //if ACK was received in time
            {
              multiplusESSpower = essPowerDesired;              //Update ESS power value successfully applied in Multiplus
              addToLogfile(" "+String(multiplusESSpower)+"W "+String(int(round(multiplusDcCurrent*multiplusDcVoltage))));  //Write applied ESS power to logfile
              essTimeoutCounter = ESS_TIMEOUT_PREVENT;          //restart ESS timeout counter
              veCmdSendState = 0;                               //sending command is done
            }
            result = true; //mark as known frame
          }
          if ((len == 13) && (frame[5] == 0xE6) && (frame[6] == 0x85))    //Reply to CommandReadRAMVar (automatically polled 10 times per second, no re-send)
          {
            int16_t t = (frame[8]<<8) + frame[7];
            multiplusUMainsRMS = t/100.0;
            if (multiplusUMainsRMS < acVoltageMin) {
              acVoltageMin = multiplusUMainsRMS;
              timeAcVoltageMin = electricMeter_Runtime;
            }
            if (multiplusUMainsRMS > acVoltageMax) {
              acVoltageMax = multiplusUMainsRMS;
              timeAcVoltageMax = electricMeter_Runtime;
            }
            multiplusPmainsFiltered = (frame[10]<<8) + frame[9];
            ptr_pACinRingBuf = (ptr_pACinRingBuf + 1) & (SIZE_P_ACIN_RINGBUF-1);         //increase and wrap pointer first
            pACinRingBuf[ptr_pACinRingBuf] = multiplusPmainsFiltered;   //now write newest value into ringbuffer
          //if (multiplusPmainsFiltered>=0) addToLogfile("\n+"+String(multiplusPmainsFiltered)); else addToLogfile("\n"+String(multiplusPmainsFiltered)); //uncomment to log every ACin value
            result = true; //mark as known frame
          }
          if ((frame[5] == 0xE6) && (frame[6] == 0x8E)) {
            if ((veCmdSendState==0x19) && (cmdAckCnt > 0))    //if reply was received in time
            {
              veCmdSendState = 0;          //sending command is done
            }
            //result = true;              //don't mark as know frame as we want to see reply in logfile
          }
          break;
        }
        case 0x41:    //41 = MasterMultiLED frame
        {
          if ((len==19) && (frame[5]==0x10))    //frame[5] is the only unknown byte in this frame, rest is documented in Victron MK2 manual
          {
            masterMultiLED_LEDon = frame[6];
            masterMultiLED_LEDblink = frame[7];
            masterMultiLED_Status = frame[8];
            masterMultiLED_AcInputConfiguration = frame[9];
            int16_t t = (frame[11]<<8) + frame[10];
            masterMultiLED_MinimumInputCurrentLimit = t/10.0;
            t = (frame[13]<<8) + frame[12];
            masterMultiLED_MaximumInputCurrentLimit = t/10.0;
            t = (frame[15]<<8) + frame[14];
            masterMultiLED_ActualInputCurrentLimit = t/10.0;
            masterMultiLED_SwitchRegister = frame[16];
            //Check if mode was changed in time
            if (cmdAckCnt > 0)
            {
              if ((veCmdSendState==0x17) && ((masterMultiLED_SwitchRegister&0x03)==0x03)) {
                addToLogfile("\nSwitched to Charger+Inverter mode");
                veCmdSendState = 0;                             //sending mode=Charger+Inverter mode is done
              }
              if ((veCmdSendState==0x15) && ((masterMultiLED_SwitchRegister&0x03)==0x01)) {
                addToLogfile("\nSwitched to Charger-Only mode");
                veCmdSendState = 0;                             //sending mode=ChargerOnly mode is done
              }
            }
            result = true; //mark as known frame
          }
          break;
        }
      }
    }
  }
  return result;
}

// ===========================================================================
//                     VE.Bus to Multiplus command handling
// ---------------------------------------------------------------------------
// This function exclusively "deals" with the VE.Bus serial buffer. It's
// written as a state machine acting on variable veCmdSendState:
//   2 ready to send
//   1 has been send but no ACK yet
//   0 acknowledged = done
// Documentation of this function is here:
// https://github.com/pv-baxi/esp32ess/blob/main/docs/README.md#vebus-receive-frames
// ===========================================================================
void veCmdReSendHandling()
{
  //Check if command acknowledge time is over. If yes, trigger to resend command
  if (((veCmdSendState & 0x10)==0x10) && (cmdAckCnt <= 0))
  {
    veCmdSendState = veCmdSendState & 0x0F;       //Re-send command
    veTxCmdFailCnt++;     //increase failed commands counter
    addToLogfile(" FAILcmd"+String(veCmdSendState,HEX));
  }
}


// ===========================================================================
//                 Apply new ESS power and start setting
// ---------------------------------------------------------------------------
// This function is a comfortable way of sending a new ESS power value into
// the Multiplus. Negative ESS power values mean charging the battery and
// positive ESS power values mean feeding power into the grid. If the
// Multiplus is wired in ACin configuration, with nothing connected to ACout2,
// this is exactly the power it will then consume or feed-in on ACin. For
// ACout2 configuration, see description and examples below.
//
// In case the Multiplus is wired in ACout2 configuration, to my experience
// the ESS power value seems to work a bit different, defining the Minimum
// that the Multiplus has to charge or discharge. Examples:
// 0W ESS power; 400W PV power on ACout2; 300W consumption on ACout2
//  -> Multiplus will charge the battery from ACout2 with 100W
// 0W ESS power; 400W PV power on ACout2; 600W consumption on ACout2
//  -> Multiplus will discharge the battery into ACout2 with 200W
// 100W ESS power; 400W PV power on ACout2; 600W consumption on ACout2
//  -> Multiplus will discharge the battery into ACout2 with 200W
//  -> power on ACin is still 0W
// 250W ESS power; 400W PV power on ACout2; 600W consumption on ACout2
//  -> Multiplus will discharge the battery into ACout2 with 250W
//  -> thus the resulting feed into the grid is 50W on ACin
// -50W ESS power; 400W PV power on ACout2; 300W consumption on ACout2
//  -> Multiplus will charge the battery from ACout2 with 100W
//  -> power on ACin is still 0W
// -200W ESS power; 400W PV power on ACout2; 300W consumption on ACout2
//  -> Multiplus will charge the battery from ACout2 with 200W
//  -> thus the resulting consumption on ACin is 100W
// -100W ESS power; 400W PV power on ACout2; 600W consumption on ACout2
//  -> Multiplus will charge the battery from ACout2 with 100W
//  -> thus the resulting consumption on ACin is 300W
// ===========================================================================
bool applyNewEssPower(short power)
{
  if (veCmdSendState == 0) {  //Last check that we're good to send new value
    essPowerDesired = power;
    veCmdSendState = 2;            //force sending ESS command
    return true;
  }
  else return false;
}


// ===========================================================================
//            Limit new ESS power based on SOC and mode options
// ---------------------------------------------------------------------------
// This function limits the desired ESS power in variable essPowerTmp based
// on the current battery level and the mode option switches.
// So if the battery is full, we don't allow (extra-)charging anymore. If the
// battery is empty, we don't allow (extra-)discharge. Also (extra-)charging
// and/or (extra-)discharging can be omitted independent of the battery level
// by the mode switches.
// In ACin-only wiring of the Multiplus, charging = extraCharging and
// discharging = extraDischarging. So there is no sense in forbidding one of
// them in a real setup.
// However, in ACout2 wiring, and with an ESS power value of for example 0W,
// the Multiplus decides itself if he needs to currently charge or discharge
// the battery to keep power on ACin at 0W. In that configuration ESS power
// values unequal to 0W are referred as "extraCharging" and "extraDischarging"
// as they control the remaining power difference on ACin and thus towards the
// power meter.
// ===========================================================================
short limitNewEsspower(short power)
{
  //check if battery is completely full or empty and limit ESS power accordingly:
  if (((power < 0) && (soc >= SOC_LIMIT_UPPER)) || ((power > 0) && (soc <= SOC_LIMIT_LOWER))) power = 0;
  //disallow negative ESS power in case extraCharge=false
  if (!extraCharge && (power < 0)) power = 0;
  return power;
}

// ===========================================================================
//                    Calculate new ESS power subfunction
// ---------------------------------------------------------------------------
// This subfunction calculates a new ESS power value for the Multiplus based
// on the current ESS power and the result from an absolute power meter. It
// is specialized for meters that only give absolute power values without a
// positive/negative sign.
// The direction of the ESS power update is given by variable essR, which is
// altered before calling this subfunction depending if the meter value got
// better or worse from the last update.
// Limitation to (DISCHARGE_LIMIT) and (-CHARGE_LIMIT) avoids that our code
// tries to get more and more power out/in our Multiplus that it actually
// cannot deliver.
// Having this in a subfunction allows to first try a preferred direction and
// applying all limits. In case this would result in no change of ESS power,
// we can easily try the opposite direction instead using the same calculation
// subfunction.
// ===========================================================================
inline void calculateNewESSpower()
{
  essPowerTmp = multiplusESSpower + round(essR * meterPower);   //calculate new EES power based on meter value and new direction R
  essPowerTmp += ESS_POWER_OFFSET;   //add small offset (to always feed little bit more into the grid)
  //limit to maximum positive and negative power
  if (essPowerTmp > DISCHARGE_LIMIT) essPowerTmp = DISCHARGE_LIMIT;
  if (essPowerTmp < -CHARGE_LIMIT) essPowerTmp = -CHARGE_LIMIT;
  essPowerTmp = limitNewEsspower(essPowerTmp);  //Limit new ESS power based on SOC and mode options
}

// ===========================================================================
//    Calculate and apply new ESS power value to Multiplus (absolute meter)
// ---------------------------------------------------------------------------
// This function is specifically implemented if meter only delivers absolute
// power values, meaning that the direction (consumption or feed-in) is
// unknown.
// After settling time is over, and we assume that the desired ESS power has
// been applied by the Multiplus, we should NOT use the next value coming in
// from the power meter. Because this value was still measured during the
// settling period, when the Multiplus power was still adjusting.
// That's why we should discard this first meter value and wait for the next
// one, here called "second measurement".
// With the absolute power meter the information which direction to update the
// ESS power value normally comes from "playing around" with the power of the
// Multiplus. But as soon as the Multiplus gets to a limit, maybe because we
// consume more power than it can deliver, or if the battery is empty or full,
// we don't have the ability to "play around" in both directions anymore. In
// that case, even so the power meter indicates a difference, we have to trust
// that we currently do the best we can. We can only try from time to time to
// step away from the limit and evaluate if the absolute meter value got less
// or more.
// As this meter change could also come from a general fluctuation in power,
// there is no use in trying small steps away from the limit. For that reason
// we have the _STEP constants applied at the end of this code, which ensure
// minimum power steps for every limit situation.
// ===========================================================================
void updateMultiplusPower_usingAbsoluteMeter()
{
  if (newMeterValue) {
      //Here we had a new meter value
        //First determine if we keep direction or better go into opposite direction:
        if (meterPower >= meterPowerOld) essR = -essR; //if electric meter difference got worse or stayed equal, swap direction for the following ESS power update
        //Calculate new ESS power:
        //addToLogfile(String(multiplusESSpower));
        //here difference was small enough, so let's continue
        meterPowerOld = meterPower; //remember current meter value for later comparison if we got better or worse
        //we can now calculate the new ESS power.
        calculateNewESSpower();
        //if limitation would result in zero change of multiplusESSpower, try other direction
        if (essPowerTmp == multiplusESSpower)
        {
          essR = -essR;           //change direction
          calculateNewESSpower(); //calculate ESS power again
        }
        //in case we are at an outer limit, only step by small amount of Watt first
        if ((multiplusESSpower==0) && (soc >= SOC_LIMIT_UPPER) && (essPowerTmp > SOC_LIMIT_UPPER_STEP)) essPowerTmp = SOC_LIMIT_UPPER_STEP;                    //discharge step in case battery full
        if ((multiplusESSpower==0) && (soc <= SOC_LIMIT_LOWER) && (essPowerTmp < -SOC_LIMIT_LOWER_STEP)) essPowerTmp = -SOC_LIMIT_LOWER_STEP;                  //charge step in case battery empty
        if ((multiplusESSpower==0) && (!extraCharge) && (essPowerTmp > NO_CHARGE_MODE_STEP)) essPowerTmp = NO_CHARGE_MODE_STEP;                              //discharge step in case No-Charge-Mode
        if ((multiplusESSpower==DISCHARGE_LIMIT) && (essPowerTmp < DISCHARGE_LIMIT-DISCHARGE_LIMIT_STEP)) essPowerTmp = DISCHARGE_LIMIT-DISCHARGE_LIMIT_STEP; //step away from discharge-limit (+)
        if ((multiplusESSpower==-CHARGE_LIMIT) && (essPowerTmp > -CHARGE_LIMIT+CHARGE_LIMIT_STEP)) essPowerTmp = -CHARGE_LIMIT+CHARGE_LIMIT_STEP;             //step away from charge-limit (-)
        applyNewEssPower(essPowerTmp);  //apply essPowerTmp to Multiplus
        //applyNewEssPower(300);  //DEBUG: dirty hack do to try some different power
  }
}

void updateMultiplusPower_usingSignedMeter()
{
  if (newMeterValue) {
    //calculate power to compensate
    estTargetPower = round(powerACin + abs(essR)*electricMeter_Power);
    //write latest value into ringbuffer
    ptrPowerRingBuf++;
    if (ptrPowerRingBuf > (SIZE_PWR_RINGBUF-1)) ptrPowerRingBuf = 0;  //wrap pointer if needed
    estTargetPowerRingBuf[ptrPowerRingBuf] = estTargetPower;
    //Now evaluate all values in the ringbuffer based on the update strategy -> Find max and min value of current ringbuffer
    short min = +30000;
    short max = -30000;
    //addToLogfile("\n");   //DEBUG print current Ringbuffer into Logfile
    for (int i=0; i<SIZE_PWR_RINGBUF; i++) {
      short pwr = estTargetPowerRingBuf[i];
      if (pwr < min) min = pwr;
      if (pwr > max) max = pwr;
      //addToLogfile(" "+String(pwr));   //DEBUG print current Ringbuffer into Logfile
    }
    //Now update depending on strategy
    if      (essPowerStrategy == 5) essPowerTmp = estTargetPower; //Immediate strategy
    else if (essPowerStrategy == 7) essPowerTmp = max;            //Maximum strategy
    else if (essPowerStrategy == 3) essPowerTmp = min;            //Minimum strategy
    else                            essPowerTmp = 0;              //if (essPowerStrategy == 0) : 0W strategy (compensate ACout2 only)
    essPowerTmp = limitNewEsspower(essPowerTmp);       //Limit ESS power based on SOC and mode options
    applyNewEssPower(essPowerTmp);
  }
}




// ===========================================================================
//                   Avoid Multiplus ESS assistant timeout
// ---------------------------------------------------------------------------
// If the ESS assistant installed on the Multiplus doesn't receive a new ESS
// power value for about 15 seconds (evaluated with firmware v508), the ESS
// assistant assumes a failure and turns off completely. This means it stops
// any charging or discharging of the battery and just passes power between
// ACin and ACout2.
//
// This is required in case electical meter turns off in case of emergerncy
// as well as if optical-impulse-based power measurement is done. Because then
// it can easily take longer than 15 second until new impulse comes in.
// ===========================================================================
void avoidMultiplusTimeout()
{
  //Check if ESS power value was not re-written into Multiplus for more than 10 seconds
  if ((essTimeoutCounter <= 0) && (veCmdSendState==0))
  {
    addToLogfile("\nT");
    //Also in this case react on reached SOC limits:
    essPowerTmp = limitNewEsspower(multiplusESSpower);                  //Limit current ESS power based on SOC and mode options
    if (essPowerTmp != multiplusESSpower) addToLogfile(" + SOC limit"); //In case this happens, write into the same logfile line
    applyNewEssPower(essPowerTmp);                                      //apply essPowerTmp to Multiplus
  }
}

void shellyControl()
{
  //save meter power into ringbuffer
  if (oneSecondOver) {
    oneSecondOver = false;
    ptr_pMeterDcRingBuf++;
    if (ptr_pMeterDcRingBuf >= SIZE_P_METER_DC_RINGBUF) ptr_pMeterDcRingBuf = 0;  //wrap pointer if needed
    pMeterRingBuf[ptr_pMeterDcRingBuf] = round(electricMeter_Power);
    pDcRingBuf[ptr_pMeterDcRingBuf] = round(multiplusDcCurrent*multiplusDcVoltage);
    if (WiFi.status() == WL_CONNECTED) {
      //Find max and min values of current ringbuffer
      short minDC = +30000;
      short maxMeter = -30000;
      for (int i=0; i<SIZE_P_METER_DC_RINGBUF; i++) {
        short pwrMeter = pMeterRingBuf[i];
        short pwrDc = pDcRingBuf[i];
        if (pwrDc < minDC) minDC = pwrDc;
        if (pwrMeter > maxMeter) maxMeter = pwrMeter;
      }
      //Decide if Shelly needs to be turned on or off
      if (((shellyStatus==0) || (shellyStatus==-1)) && (minDC >= -0.2) && (maxMeter < -900)) {   // -(ShellyPower + 100W)
        //Turn Shelly on
        if (switchShelly("192.168.178.10", true) == true) {
          shellyStatus = 1;
          addToLogfile("\nShelly 800W on");
        }
      }
      else if (((shellyStatus==1) || (shellyStatus==-1)) && ((minDC < -0.2) || (maxMeter > 10))) {
        //Turn Shelly off
        if (switchShelly("192.168.178.10", false) == true) {
          shellyStatus = 0;
          addToLogfile("\nShelly 800W off");
        }
      }
    }
  }
}

bool switchShelly(String shellyIP, bool turnOn)
{
  String command = "http://" + shellyIP + "/relay/0?turn=";
  if (turnOn) command += "on"; else command += "off";
  HTTPClient http;
  http.setTimeout(1000);
  http.begin(command.c_str());
  int httpResponseCode = http.GET();     //HTTP response code != 200 means error
  http.end();
  //addToLogfile("\nShelly: "+httpResponseCode);
  if (httpResponseCode == 200) return true; else return false;
}

// ===========================================================================
//                     Monitor battery minimum/maximum
// ---------------------------------------------------------------------------
// Although called "debug", this function is simply used to log the minimum
// and maximum of the battery level (SOC) during a runtime period (until a
// reboot happens).
// Additionally this function checks against a minimum and a maximum the
// battery level is not allowed to reach. If that happens, the logfile is
// stopped in about half of its size. This means that in case you're on
// holiday and during that period the battery got charged or discharged too
// much, you can read through the logfile and see in the middle of the logfile
// what made it happen and how it continued.
// ===========================================================================
void debugEvaluate(int debugval, int stopLoggingMin, int stopLoggingMax)
{
  if (debugval > debugmax) debugmax = debugval;
  if (debugval < debugmin) debugmin = debugval;
  if (!stopWritingLogfile && ((debugval <= stopLoggingMin) || (debugval >= stopLoggingMax)))
  {
    stopWritingLogfile = true;               //the end of logging is near..
    logfileCounterStop = logfileCounter + LOGFILE_SIZE/2;  //still allow half the buffer to be overwritten with log entries
    addToLogfile("\n"+String(relTime)+" LOGGING END due to "+String(debugval));
  }
}


// ===========================================================================
//                      CAN bus battery message handling
// ---------------------------------------------------------------------------
// This function reads the CAN (TWAI) bus messages from the battery. There is
// no need to send any specific message twards the battery. The battery status
// messages are automatically sent-out as long as the CAN bus transceiver is
// automatically acknowleding the messages. (Otherwise the battery will just
// continiously re-send a useless status message until it gets acknowledged.)
// A message with identifier 0x355 comes about every second. As soon as it is
// received, this function decodes the battery level to global variable soc,
// and once writes the battery health into the logfile.
// Additionally this function resets a watchdog on every message 0x355, so
// that as soon as there is no SOC coming from the battery anymore, our global
// soc variable is set to -1 and our program can react accordingly e.g. by not
// sending ESS power values to the Multiplus anymore, which will then stop
// charging/discharging the battery within a minute.
// ===========================================================================
void batteryHandling()
{
  //Check if CAN bus message from battery is received containing SOC (state of charge) value
  if (TWAI_ALERT_RX_DATA) {
    // One or more messages received. Handle all.
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK){
      //SOC value is in 1st byte of 0x355 message, battery health = 3rd byte
      if (message.identifier == 0x355)
      {
        if (!batHealthWritten) {
          addToLogfile("\nBattery health: "+String(message.data[2])+"%\n");
          batHealthWritten = true;
        }
        socWatchdog = SOC_WATCHDOG_CYCLES;  //reset watchdog timer
        soc = message.data[0];         //get battery SOC from CAN message
        debugEvaluate(soc, SOC_LIMIT_LOWEST, 101 );  //log min/max and stop writing logfile soon if battery reached critical level (min,max)
      }
    }
  }
  //Check if there was no SOC received from battery anymore and watchdog timer is expired
  if ((soc >= 0) && (socWatchdog <= 0))
  {
    soc = -1;                 //set SOC negative, to indicate that it is invalid
    addToLogfile("\n"+String(relTime)+" Battery CAN bus FAIL");
  }
}


// ===========================================================================
//                  Check and evaluate impulse from power meter
// ---------------------------------------------------------------------------
// This function simply checks if there was a new optical 1/10000kWh impulse
// measured by the ISR from the official power meter. If yes, it checks if the
// 2ms high-pulses defining the period are within valid range. If that's the
// case, the power in Watt is calculated from the measured period by division.
// ===========================================================================
void checkImpulsePowerMeter()
{
  newMeterValue = false; //Here the meter value is not new anymore
  if (isrNewMeterPulse)  //if we got new power meter impulse from ISR
  {
    isrNewMeterPulse = false;  //reset flag
    if ((highCyclesPrevious >= 18) && (highCyclesPrevious <= 21)
    && (totalCycles >= 360) && (totalCycles <= 3600000)
    && (highCycles >= 18) && (highCycles <= 21)) //if within senseful range
    {
      meterPowerPrevious = meterPower;       //remember previous value to detect sudden and large changes
      meterPower = 3600000 / totalCycles;   //calculate power in Watt from measured cycles
      //Evaluate the sign information from Info-DSS SML data
      if (electricMeterSignPositive > electricMeterSignNegative) electricMeterCurrentSign = +1;
      else if (electricMeterSignNegative > electricMeterSignPositive) electricMeterCurrentSign = -1;
      else if ((electricMeterSignNegative == electricMeterSignPositive) && (electricMeterSignPositive > 0)) electricMeterCurrentSign = 0;   //if sign is fluctuating we better assume 0W on average
      //else we just keep the sign from last time
      electricMeterSignPositive = 0;  //clear the counters to start counting again in next measurement period
      electricMeterSignNegative = 0;
      //include the sign into power value
      meterPower = electricMeterCurrentSign * meterPower;
      newMeterValue = true;               //indicate that we got a new meter value
    }
  }
}


// ===========================================================================
//            Add new meter value to logfile and spike detection
// ---------------------------------------------------------------------------
// This function adds the measured power from the meter including a relative
// time stamp to the logfile in a nicely formatted way, so that all power
// values are below each other.
// Additionally this function checks if there was an unusual high difference
// compared to the last measured meter value. If that's the case, and if the
// setting time of the Multiplus was already (or nearly) over, the settling
// time is extended again to SETTLING_IGNORE_PWR_SPIKE.
// In case the power spike was correct, and generated by a large sink turning
// on, this will just delay the ESS power adaptation by a neglectable amount
// of time. In case it was a short spike due to e.g. a motor turning on, this
// spike detection will avoid unnecessary adaptation to the spike.
// ===========================================================================
void onNewMeterValue()
{
  //If one minute is over, add current time + SOC + bat. voltage into logfile
  if (oneMinuteOver) {
    addToLogfile('\n'+secondsToTimeStr(electricMeter_Runtime+ELECTRIC_METER_TIME_OFFSET,false)+" "+String(soc)+"% "+String(multiplusDcVoltage, 2)+"V");
    oneMinuteOver = false;
  }
  //If new power meter value came in, write it to logfile first:
  if (newMeterValue)
  {
    //calculate Multiplus ACin power for the same time period
    powerACin = 0;
    powerACin += pACinRingBuf[(ptr_pACinRingBuf -5 )&(SIZE_P_ACIN_RINGBUF-1)];
    powerACin += pACinRingBuf[(ptr_pACinRingBuf -6 )&(SIZE_P_ACIN_RINGBUF-1)];
    powerACin += pACinRingBuf[(ptr_pACinRingBuf -7 )&(SIZE_P_ACIN_RINGBUF-1)];
    powerACin += pACinRingBuf[(ptr_pACinRingBuf -8 )&(SIZE_P_ACIN_RINGBUF-1)];
    powerACin += pACinRingBuf[(ptr_pACinRingBuf -9 )&(SIZE_P_ACIN_RINGBUF-1)];
    powerACin += pACinRingBuf[(ptr_pACinRingBuf -10 )&(SIZE_P_ACIN_RINGBUF-1)];
    powerACin += pACinRingBuf[(ptr_pACinRingBuf -11 )&(SIZE_P_ACIN_RINGBUF-1)];
    powerACin = powerACin / 7;
    //write power to logfile
    String tSpaces = "";                    //determine required spaces to achieve fixed digits for relTime
    if (relTime < 10) tSpaces = "  ";
    else if (relTime < 100) tSpaces = " ";
    addToLogfile("\n"+tSpaces+String(relTime));
    if (meterPower>=0) addToLogfile(" +"+String(meterPower)); else addToLogfile(" "+String(meterPower));
    int p = round(powerACin);
    if (p>=0) addToLogfile(" +"+String(p)); else addToLogfile(" "+String(p));
    // If difference to previous meter value is very large, it just might just be a sudden power spike (e.g. from a device turning on).
    // In case we're well in the settling, high power differences are normal and can be ignored. In case settling was already (or nearly)
    // over, settling time is extended again:
    // if ((abs(meterPower-meterPowerPrevious) >= MAX_METER_DIFFERENCE) && (multiplusSettlingCnt < SETTLING_IGNORE_PWR_SPIKE))
    // {
    //   multiplusSettlingCnt = SETTLING_IGNORE_PWR_SPIKE;
    //   addToLogfile(" spike "+String((relTime+multiplusSettlingCnt/(CPS/10))%600));
    // }
  }
}


int searchSMLentry(char* smlBuffer, char* searchString, int searchStringLen, int searchStart, int searchStop)
{
  int searchEndIndex = searchStop - searchStringLen;
  bool found = false;
  int i = searchStart;
  while ((i < searchEndIndex+1) && (found == false)) {  //This search loop can probably be improved...
    found = true;
    for (int j=0; j<searchStringLen; j++) found = found && (smlBuffer[i+j]==searchString[j]);
    if (found == false) i++;
  }
  if (i < searchEndIndex+1) {   //if search string was found
    return i;
  }
  else return -1;
}

int readSMLentry(char* smlBuffer, int smlLength, int pos, int skipBytes, int skiprows, char* resultDest, int maxBytesRead)
{
  int len = 0;
  if (pos < smlLength) {   //if start position is inside SML stream
    pos += skipBytes;     //skip for example the list definition (0x77), if we included that in the search
    //skip (skiprows) rows in that list, INCLUDING the row contained in the searchString
    for (int row=0; row<skiprows; row++) pos += (smlBuffer[pos] & 0x0F); //low-nibble in first byte of row contains how many bytes to skip
    //now we point to the desired row
    len = (smlBuffer[pos++] & 0x0F) - 1;    //get data length of row
    if (len > maxBytesRead) len = maxBytesRead;
    if (pos+len <= smlLength) {  //check that we are not accessing bytes outside the SML message...
      for (int j=0; j<len; j++) resultDest[j]=smlBuffer[pos++];  //read following len bytes
    }
    else len = 0;
  }
  return len;
}


void checkSMLpowerMeter()
{
  newMeterValue = false; //Here the meter value is not new anymore
  //Check for new bytes on UART
  while (Serial2.available())
  {
    const uint16_t CIR = sizeof(sBuf)-1;  //get the value once
    char c = Serial2.read();    //read one byte
    smlp = CIR&(smlp+1);        //increase and wrap pointer to make circular buffer
    sBuf[smlp] = c;             //write the new byte into circular SML buffer
    smlCnt += 1;                //increase SML length counter
    //search the end of the SML stream
    //-8 -7 -6 -5 -4 -3 -2 -1 -0
    //00 1B 1B 1B 1B 1A xx xx xx
    if ((sBuf[CIR&(smlp-8)]==0x00) && (sBuf[CIR&(smlp-7)]==0x1B) && (sBuf[CIR&(smlp-6)]==0x1B) && (sBuf[CIR&(smlp-5)]==0x1B) &&
        (sBuf[CIR&(smlp-4)]==0x1B) && (sBuf[CIR&(smlp-3)]==0x1A)) {   //if we successfully found the end of SML stream
      //End of SML stream detected
      smlLength = smlCnt;                                         //detected length of our SML stream
      if (smlLength > SML_LENGTH_MAX) smlLength = SML_LENGTH_MAX; //limit to the maximum length we allow
      smlCnt = 0;                                                 //reset SML length counter, to count next SML stream
      //Pointer smlp points exactly to the last byte of the stream
      //Copy smlLength bytes of the message into a new buffer and calculate CRC
      crc.restart();
      for (uint16_t i=0; i<(smlLength-2); i++)  //exclude the two CRC bytes at the end
      {
        c = sBuf[CIR&(smlp-(smlLength-1)+i)];
        crc.add(c);
        sBuf2[i] = c;
      }
      //now copy the last two bytes being the CRC
      sBuf2[smlLength-2] = sBuf[CIR&(smlp-1)];
      sBuf2[smlLength-1] = sBuf[CIR&(smlp-0)];
      //verify CRC
      int16_t smlCRC = (sBuf2[smlLength-1]<<8) + sBuf2[smlLength-2];    //get CRC from the received stream
      int16_t calculatedCRC = crc.calc();
      if (smlCRC == calculatedCRC) {     //If CRC is matching
        smlLengthDisplay = 15;  //display SML length 15 times on display, then vanish again
        //decode SML stream
        //addFrameToLogfile(sBuf2,0,smlLength-1); //DEBUG
        char temp[8];       //8 bytes to fit Wh counters with full precision
        int8_t scaler = 0;  //required for correct power/counters readout
        int pos = -1;       //position where we found the search string inside the SML stream
        //Device ID
        char searchString0[] = {0x77,0x07,0x01,0x00,0x00,0x00,0x09,0xFF};
        pos = searchSMLentry(sBuf2, searchString0, sizeof(searchString0), 0, smlLength-1);
        if (pos >= 0) readSMLentry(sBuf2, smlLength, pos, 1, 5, electricMeter_DeviceID, 10);
        //1.8.0 (consumption) status + kWh counter
        char searchString1[] = {0x77,0x07,0x01,0x00,0x01,0x08,0x00,0xFF};
        pos = searchSMLentry(sBuf2, searchString1, sizeof(searchString1), 0, smlLength-1);
        if (pos >= 0) {
          readSMLentry(sBuf2, smlLength, pos, 1, 1, electricMeter_Status180, 3);
          if (readSMLentry(sBuf2, smlLength, pos, 1, 4, temp, 1) == 1) {          //Scaling factor 1.8.0
            scaler = temp[0];
            int len = readSMLentry(sBuf2, smlLength, pos, 1, 5, temp, 8);            //Consumption 1.8.0
            if (len > 0) {
              uint64_t p64 = 0;
              uint64_t d64 = 0;
              int bitshift = 0;
              for (int i = len-1; i>=0 ; i--) {
                d64 = temp[i];
                d64 = d64 << bitshift;
                p64 += d64;
                bitshift += 8;
              }
              electricMeter_Consumption = p64 * pow(10,scaler) * 0.001;   // *0.001 as we want kWh, not Wh
            }
          }
        }
        //2.8.0 (feed-in) status + kWh counter
        char searchString2[] = {0x77,0x07,0x01,0x00,0x02,0x08,0x00,0xFF};
        pos = searchSMLentry(sBuf2, searchString2, sizeof(searchString2), 0, smlLength-1);
        if (pos >= 0) {
          readSMLentry(sBuf2, smlLength, pos, 1, 1, electricMeter_Status280, 3);
          if (readSMLentry(sBuf2, smlLength, pos, 1, 4, temp, 1) == 1) {          //Scaling factor 2.8.0
            scaler = temp[0];
            int len = readSMLentry(sBuf2, smlLength, pos, 1, 5, temp, 8);            //Consumption 2.8.0
            if (len > 0) {
              uint64_t p64 = 0;
              uint64_t d64 = 0;
              int bitshift = 0;
              for (int i = len-1; i>=0 ; i--) {
                d64 = temp[i];
                d64 = d64 << bitshift;
                p64 += d64;
                bitshift += 8;
              }
              electricMeter_FeedIn = p64 * pow(10,scaler) * 0.001;   // *0.001 as we want kWh, not Wh
            }
          }
        }
        //Runtime
        char searchString3[] = {0x72,0x62,0x01,0x65};
        pos = searchSMLentry(sBuf2, searchString3, sizeof(searchString3), 0, smlLength-1);
        if (pos >= 0) {
          int len = readSMLentry(sBuf2, smlLength, pos, 1, 1, temp, 4);
          if (len == 4) {
            electricMeter_Runtime = (temp[0]<<24) + (temp[1]<<16) + (temp[2]<<8) + temp[3];
            //Check if time is exactly 5:00. If yes, reset min/max values
            int h = 0;
            int m = 0;
            int s = 0;
            secondsToTime(h, m, s, electricMeter_Runtime+ELECTRIC_METER_TIME_OFFSET);
            if ((h==5) && (m==0)) {
              debugmin = soc;
              debugmax = soc;
              dcCurrentMin = multiplusDcCurrent;
              dcCurrentMax = multiplusDcCurrent;
              acVoltageMin = multiplusUMainsRMS;
              timeAcVoltageMin = -ELECTRIC_METER_TIME_OFFSET;
              acVoltageMax = multiplusUMainsRMS;
              timeAcVoltageMax = -ELECTRIC_METER_TIME_OFFSET;
              acFrequencyMin = multiplusAcFrequency;
              timeAcFrequencyMin = -ELECTRIC_METER_TIME_OFFSET;
              acFrequencyMax = multiplusAcFrequency;
              timeAcFrequencyMax = -ELECTRIC_METER_TIME_OFFSET;
              dcVoltageMin = multiplusDcVoltage;
              dcVoltageMax = multiplusDcVoltage;
              multiplusTempMin = multiplusTemp;
              multiplusTempMax = multiplusTemp;
            }
          }
        }
        //Continue with values only available in "complete" SML info
        //Summed power
        char searchString4[] = {0x77,0x07,0x01,0x00,0x10,0x07,0x00,0xFF};
        pos = searchSMLentry(sBuf2, searchString4, sizeof(searchString4), 0, smlLength-1);
        if (pos >= 0) {
          if (readSMLentry(sBuf2, smlLength, pos, 1, 4, temp, 1) == 1) {          //Scaling factor summed power
            scaler = temp[0];
            int len = readSMLentry(sBuf2, smlLength, pos, 1, 5, temp, 4);         //Summed power ín W
            if (len == 4) {
              int32_t p32 = (temp[0]<<24) + (temp[1]<<16) + (temp[2]<<8) + temp[3];
              electricMeter_Power = p32 * pow(10,scaler);
              //if ((electricMeter_Status180[2] & 0xF0) == 0xA0) electricMeter_Power = -electricMeter_Power;  //not needed as my meter already gives a signed value
              //also update the integer values
              meterPowerPrevious = meterPower;
              meterPower = round(electricMeter_Power);
              newMeterValue = true;
            }
          }
        }
        //Power L1
        char searchString5[] = {0x77,0x07,0x01,0x00,0x24,0x07,0x00,0xFF};
        pos = searchSMLentry(sBuf2, searchString5, sizeof(searchString5), 0, smlLength-1);
        if (pos >= 0) {
          if (readSMLentry(sBuf2, smlLength, pos, 1, 4, temp, 1) == 1) {          //Scaling factor power L1
            scaler = temp[0];
            int len = readSMLentry(sBuf2, smlLength, pos, 1, 5, temp, 4);         //power L1 ín W
            if (len == 4) {
              int32_t p32 = (temp[0]<<24) + (temp[1]<<16) + (temp[2]<<8) + temp[3];
              electricMeter_PowerL1 = p32 * pow(10,scaler);
            }
          }
        }
        //Power L2
        char searchString6[] = {0x77,0x07,0x01,0x00,0x38,0x07,0x00,0xFF};
        pos = searchSMLentry(sBuf2, searchString6, sizeof(searchString6), 0, smlLength-1);
        if (pos >= 0) {
          if (readSMLentry(sBuf2, smlLength, pos, 1, 4, temp, 1) == 1) {          //Scaling factor power L2
            scaler = temp[0];
            int len = readSMLentry(sBuf2, smlLength, pos, 1, 5, temp, 4);         //power L2 ín W
            if (len == 4) {
              int32_t p32 = (temp[0]<<24) + (temp[1]<<16) + (temp[2]<<8) + temp[3];
              electricMeter_PowerL2 = p32 * pow(10,scaler);
            }
          }
        }
        //Power L3
        char searchString7[] = {0x77,0x07,0x01,0x00,0x4C,0x07,0x00,0xFF};
        pos = searchSMLentry(sBuf2, searchString7, sizeof(searchString7), 0, smlLength-1);
        if (pos >= 0) {
          if (readSMLentry(sBuf2, smlLength, pos, 1, 4, temp, 1) == 1) {          //Scaling factor power L3
            scaler = temp[0];
            int len = readSMLentry(sBuf2, smlLength, pos, 1, 5, temp, 4);         //power L3 ín W
            if (len == 4) {
              int32_t p32 = (temp[0]<<24) + (temp[1]<<16) + (temp[2]<<8) + temp[3];
              electricMeter_PowerL3 = p32 * pow(10,scaler);
            }
          }
        }
        //Count power signs from status words for combination with impulse power measurement
        //0x8x=consumption; 0xAx=feed-in
        if ((electricMeter_Status180[2] & 0xF0) == 0x80) electricMeterSignPositive++;
        else if ((electricMeter_Status180[2] & 0xF0) == 0xA0) electricMeterSignNegative++;
        //Currently, we evaluate both status words and count for both
        if ((electricMeter_Status280[2] & 0xF0) == 0x80) electricMeterSignPositive++;
        else if ((electricMeter_Status280[2] & 0xF0) == 0xA0) electricMeterSignNegative++;
        //Compare both status words
        if ((electricMeter_Status180[0]!=electricMeter_Status280[0]) ||
            (electricMeter_Status180[1]!=electricMeter_Status280[1]) ||
            (electricMeter_Status180[2]!=electricMeter_Status280[2])) {
          electricMeterStatusDifferent++;   //Increase counter
          addToLogfile("\nElectric meter status not matching");
          addFrameToLogfile(electricMeter_Status180,0,2);
          addFrameToLogfile(electricMeter_Status280,0,2);
        }
      }
      else {  //If CRC was wrong:
        electricMeterCRCwrong++;
        //addToLogfile("\n"+String(smlLength)+" "+String(calculatedCRC,HEX));
        //addFrameToLogfile(sBuf2,0,smlLength-1);
      }
    }
  }
}

void automaticChargerOnlySwitching()
{
  //First check if we currently supply emergency power, but are not in 'E'-mode yet. If yes, switch to 'E'-Mode.
  if ((multiplusEmergencyPowerStatus==0x02) && (switchMode!='E')) switchSpecialModes('E');
  //Now decide on the ESS power update strategy
  if (switchMode == 'm') essPowerStrategy = 7;  //force maximum strategy
  else if (switchMode == 'i') essPowerStrategy = 5;  //force immediate strategy
  else if ((switchMode == '0') || (switchMode == 'C') || (switchMode == 'E')) essPowerStrategy = 0;  //force 0W ESS power
  else {
    //if we are not forced to specific mode, then decide based on battery level:
    if (soc > SOC_IMM_VS_MAX_STRATEGY) essPowerStrategy = 7;      //maximum
    else if (soc > SOC_MIN_VS_IMM_STRATEGY) essPowerStrategy = 5; //immediate
    else if (soc > SOC_0W_VS_MIN_STRATEGY) essPowerStrategy = 3;  //minimum
    else essPowerStrategy = 0;           //if below, the default is 0W strategy
  }
  //Finally check if the working mode of the Multiplus needs to be changed for some reason.
  if (veCmdSendState == 0) {
    if (switchMode=='C') {              //if ChargeOnly mode is forced
      if ((masterMultiLED_SwitchRegister & 0x30) != 0x10) veCmdSendState = 0x05;   //If not in charger-only mode, force switching to it
    }
    else if (switchMode=='E') {         //if Charger+Inverter mode is forced
      if ((masterMultiLED_SwitchRegister & 0x30) != 0x30) veCmdSendState = 0x07;   //If not in Inverter+Charger mode, force switching to it
    }
    else {  //if we are not in any special force-mode:
      if ((soc>=0) && (soc < SOC_CHARGE_ONLY_VS_0W) && ((masterMultiLED_SwitchRegister&0x30)==0x30)) veCmdSendState = 0x05; //switch to charger-only-mode
      if ((soc > SOC_CHARGE_ONLY_VS_0W) && ((masterMultiLED_SwitchRegister&0x30)==0x10)) veCmdSendState = 0x07;            //switch to charger+inverter mode
    }
  }
}



// ===========================================================================
//              Switch between special modes using Boot-Button
// ---------------------------------------------------------------------------
// Call this function with desiredMode=0 in main loop to handle button press.
// Call this function with desiredMode=SWITCH_MODE_DEFAULT in setup() to set
// mode option variables for desired default mode at program start.
// ===========================================================================
void switchSpecialModes(char desiredMode)
{
  boolean switchModeChanged = false;    //create temporary flag
  if (desiredMode==0) {
    //Here, function was called from main loop.
    //Time-limited modes: Check, if we are in such mode and duration is over:
    if ( (switchMode=='E')    && (specialModeCnt<=0))
    {
      switchMode = SWITCH_MODE_DEFAULT;
      switchModeChanged = true;           //flag to apply mode below
      addToLogfile("\n-> back in default Mode by timeout:");
    }
    //React on button-press
    if (buttonPressCnt >= BUTTON_LONGPRESS_TIME) {            //Long push detected
      //Give pending VE.Bus command time to finish
      digitalWrite(RED_LED, HIGH);   //turn red LED on, so that also without display we can see to be in "programming mode"
      lcd.clear();
      lcd.print("Finish VE.Bus...");
      delay(2000);
      //set output GPIOs to defined state
      digitalWrite(VEBUS_DE, LOW);   //make sure VE.Bus direction is Read
      lcd.clear();
      lcd.print("# PROGRAMMING MODE #");
      //officially deadlock our program
      while (true) delay(100);
    }
    boolean bootButton = digitalRead(BUTTON_GPIO);
    if (bootButton == HIGH) {                               //If button is released
      if (buttonPressCnt >= BUTTON_DEBOUNCE_TIME) {         //Short push detected
        if (switchMode == 'E') switchMode = 'N';        //Normal Mode (Also allow charging from power "before" the Multiplus, ACin, L2 or L3)
        else if (switchMode == 'N') switchMode = 'n';   //no-extra-Charge-Mode (Only allows charging from power on ACout2)
        else if (switchMode == 'n') switchMode = 'i';   //immediate strategy Mode (like 'n'-mode, but always use immediate strategy)
        else if (switchMode == 'i') switchMode = 'm';   //maximum strategy Mode (like 'n'-mode, but always use maximum strategy)
        else if (switchMode == 'm') switchMode = '0';   //0W Charge+Discharge Mode (Only compensation for power on ACout2)
        else if (switchMode == '0') switchMode = 'C';   //0W Charger-only Mode (No compensation at all, only charging from ACout2)
        else if (switchMode == 'C') switchMode = 'E';   //0W Emergency-Power mode (Compensation also below SOC_CHARGE_ONLY_VS_0W)
        specialModeCnt = SWITCH_MODE_DURATION;  //always start timer. If mode automatically ends or not, depends on code above.
        switchModeChanged = true;               //flag to apply mode below
      }
      buttonPressCnt = -1;    //-1 = lock counter. Don't start it yet if button is not pressed.
    }
    else {    //Here button is pressed
      if (buttonPressCnt == -1) buttonPressCnt=0;   //allow to start counter (but don't reset it!)
    }
  }
  else {
    //If this function was called with desired mode from setup():
    switchMode = desiredMode;               //apply desired mode
    specialModeCnt = SWITCH_MODE_DURATION;  //always start timer. If mode automatically ends or not, depends on code above.
    switchModeChanged = true;               //flag to apply mode below
  }
  //If there was a mode-change, apply mode:
  if (switchModeChanged) {
    switchModeChanged = false;  //reset flag
    switch (switchMode) {
      case 'N':    // Normal mode: Useful for Multiplus in ACin-only wiring
      {
        extraCharge = true;           //allow (extra) charging
        addToLogfile("\n-> enter Normal Mode");
        break;
      }
      case '0':    // 0W ON Mode: Useful for testing. And useful in ACout-wiring during months of limited PV power.
      {
        extraCharge = false;          //don't allow extra charging
        addToLogfile("\n-> enter 0W ON Mode");
        break;
      }
      case 'C':    // 0W ChargerOnly Mode
      {
        extraCharge = false;          //don't allow extra charging
        addToLogfile("\n-> enter 0W ChargerOnly Mode");
        break;
      }
      case 'E':    // Emergency-Power mode (force Charger+Inverter mode)
      {
        extraCharge = false;          //don't allow extra charging
        addToLogfile("\n-> enter Emergency-Power mode");
        break;
      }
      case 'n':    // no-extra-Charge-Mode: Useful with ACout-wiring if PV is completely connected to ACout.
      {           //                Also useful with problematic devices on absolute power meter (not providing consumption / feed-in direction)
        extraCharge = false;          //no extra charging
        addToLogfile("\n-> enter no-extra-Charge-Mode");
        break;
      }
      case 'i':    // immediate strategy + no-extra-Charge-Mode
      {
        extraCharge = false;          //no extra charging
        addToLogfile("\n-> enter immediate strategy Mode");
        break;
      }
      case 'm':    // maximum strategy + no-extra-charge mode
      {  
        extraCharge = false;          //no extra charging
        addToLogfile("\n-> enter maximum strategy Mode");
        break;
      }

    }
  }
}


// ===========================================================================
//                              Update displays
// ---------------------------------------------------------------------------
// This function updates all the information on the HD44780 compatible
// display, which is in my case 4x20 characters. Additionally, the optical
// impulse from the power meter is forwarded to an LED.
// ===========================================================================
void updateDisplays()
{
    //Forward power meter pulse to red LED
    digitalWrite(RED_LED, IRpinDisplay);    //Write the remembered state of the IR input pin to red LED
    IRpinDisplay = LOW;               //make that LED will turn off next Displaying-Cycle
  // --- Line 1 ---
    //Print DC voltage
    lcd.setCursor(0, 0);
    lcd.print(multiplusDcVoltage,2);
    lcd.print("V ");
    //Print DC current
    lcd.setCursor(7, 0);
    lcd.print(multiplusDcCurrent,1);
    lcd.print("A  ");
    //Print temperature
    lcd.setCursor(14, 0);
    lcd.print(multiplusTemp,1);
    lcd.print(char(0xDF));
    lcd.print("C ");
  // --- Line 2 ---
    // //Print failed + total ESS commands
    // lcd.setCursor(0, 1);
    // lcd.print(veTxCmdFailCnt);
    // lcd.print("/");
    // lcd.print(veCmdCounter);
    // lcd.print("/");
    // lcd.print(veRxCmdFailCnt);
    // lcd.print(" ");
    //Print applied ESS power value
    lcd.setCursor(0, 1);
    lcd.print(multiplusESSpower);
    lcd.print("W    ");
    //Print current ESS power on ACin
    lcd.setCursor(7, 1);
    lcd.print(int(round(powerACin)));
    lcd.print("W    ");
    //Print Multiplus Voltage Status
    lcd.setCursor(14, 1);
    lcd.print(multiplusVoltageStatus,HEX);
    lcd.print(" ");
    //Print battery charge level (SOC)
    lcd.setCursor(17, 1);
    if ((soc >= 0) && (soc <= 100)) lcd.print(soc); else lcd.print("--"); //only 2 digits as we hope to never reach 100% due to failure
    lcd.print("% ");
  // --- Line 3 ---
    //Print if logged into WiFi or not
    lcd.setCursor(0, 2);
    if (WiFi.status() == WL_CONNECTED) lcd.print("W"); else lcd.print("-");
    //print ESS power strategy, which is always a number below 10:
    lcd.print(essPowerStrategy);
    //Print Special Mode indication (1-byte ASCII):
    lcd.print(switchMode);        //print switchMode number as ASCII character
    //Print Multiplus SwitchMode
    lcd.setCursor(4, 2);
    lcd.print(masterMultiLED_SwitchRegister,HEX);
    lcd.print(" ");
    //Print Shelly status
    lcd.setCursor(7, 2);
    if (shellyStatus==1) lcd.print(char(0xFF));
    else if (shellyStatus==0) lcd.print("-");
    else lcd.print("?");
    //Print current meter value
    lcd.setCursor(10, 2);
    if (meterPower==0) lcd.print(0);
    else if (meterPower<0) lcd.print(meterPower);
    else {
      lcd.print('+');
      lcd.print(meterPower);
    }
    lcd.print("W  ");
    //Print blinking SML stream length
    lcd.setCursor(17, 2);
    if (smlLengthDisplay > 0) {
      lcd.print(smlLength);
      smlLengthDisplay -= 1;
    }
    else lcd.print("   ");
  // --- Line 4 ---
    //print other debug information
    lcd.setCursor(0, 3);
    lcd.print(multiplusBattery_byte07,HEX);
    lcd.print(" ");
    lcd.print(multiplusBattery_byte06,HEX);
    lcd.print(" ");
    lcd.print(multiplusBattery_byte05,HEX);
    lcd.print("  ");
    lcd.print(multiplusE4_byte18,HEX);
    lcd.print(" ");
    lcd.print((multiplusE4_byte17 >> 4),HEX);
    lcd.print(" ");
    lcd.print(multiplusE4_byte12,HEX);
    lcd.print(" ");
    lcd.print(multiplusE4_byte11,HEX);
    lcd.print(" ");
    // lcd.print(multiplusAcFrequency, 3);
    // lcd.print(" ");
//    if (multiplusPowerFactor >= 0) lcd.print(" ");
//    lcd.print(multiplusPowerFactor, 3);
//    lcd.print(" ");
    //Print first 7 HEX bytes of desired buffer
    // lcd.setCursor(0, 3);
    // char temp[4];
    // for (int i=0;i<7;i++)
    // {
    //   sprintf(temp,"%02X ",electricMeter_DeviceID[i]);
    //   lcd.print(temp);
    // }
}

void secondsToTime(int &hours, int &minutes, int &seconds, uint32_t largeSeconds)
{
  uint32_t t = largeSeconds;
  int s = t % 60;       //get the seconds
  t = (t - s)/60;
  int m = t % 60;       //get the minutes
  t = (t - m)/60;
  int h = t % 24;       //get the hours
  seconds = s;
  minutes = m;
  hours = h;
}

String secondsToTimeStr(uint32_t seconds, bool includeSeconds)
{
  uint32_t t = seconds;
  int s = t % 60;       //get the seconds
  t = (t - s)/60;
  int m = t % 60;       //get the minutes
  t = (t - m)/60;
  int h = t % 24;       //get the hours
  char time[9];
  if (includeSeconds) sprintf(time,"%02d:%02d:%02d",h,m,s); else sprintf(time,"%02d:%02d",h,m);
  return time;
}

String meterDeviceIdToStr(char *deviceID)
{
  char str[20];
  sprintf(str,"%d %c%c%c%02X %08d",deviceID[1],deviceID[2],deviceID[3],deviceID[4],deviceID[5],(deviceID[6]<<24)+(deviceID[7]<<16)+(deviceID[8]<<8)+deviceID[9]);
  return str;
}


// ===========================================================================
//                 Sends main webpage with basic information
// ---------------------------------------------------------------------------
// This function handles displaying the web page as soon as one calls the IP
// adress of the ESP32 in a webbrowser within the local network.
// ===========================================================================
void handleRoot() {
  //mains on:
  String strMainsOn = "";
  bool on = (masterMultiLED_LEDon & 0x01);
  bool blink = (masterMultiLED_LEDblink & 0x01);
  if (on && !blink) strMainsOn = "mains on";
  if (on && blink) strMainsOn = "<i>mains on</i>";
  if (!on && blink) strMainsOn = "<i>mains on</i>";
  //absorption:
  String strAbsorption = "";
  on = (masterMultiLED_LEDon & 0x02);
  blink = (masterMultiLED_LEDblink & 0x02);
  if (on && !blink) strAbsorption = "absorption";
  if (on && blink) strAbsorption = "<i>absorption</i>";
  if (!on && blink) strAbsorption = "<i>absorption</i>";
  //bulk:
  String strBulk = "";
  on = (masterMultiLED_LEDon & 0x04);
  blink = (masterMultiLED_LEDblink & 0x04);
  if (on && !blink) strBulk = "bulk";
  if (on && blink) strBulk = "<i>bulk</i>";
  if (!on && blink) strBulk = "<i>bulk</i>";
  //float:
  String strFloat = "";
  on = (masterMultiLED_LEDon & 0x08);
  blink = (masterMultiLED_LEDblink & 0x08);
  if (on && !blink) strFloat = "float";
  if (on && blink) strFloat = "<i>float</i>";
  if (!on && blink) strFloat = "<i>float</i>";
  //inverter on:
  String strInverterOn = "";
  on = (masterMultiLED_LEDon & 0x10);
  blink = (masterMultiLED_LEDblink & 0x10);
  if (on && !blink) strInverterOn = "inverter on";
  if (on && blink) strInverterOn = "<i>inverter on</i>";
  if (!on && blink) strInverterOn = "<i>inverter on</i>";
  //overload:
  String strOverload = "";
  on = (masterMultiLED_LEDon & 0x20);
  blink = (masterMultiLED_LEDblink & 0x20);
  if (on && !blink) strOverload = "overload";
  if (on && blink) strOverload = "<i>overload</i>";
  if (!on && blink) strOverload = "<i>overload</i>";
  //low battery:
  String strLowBattery = "";
  on = (masterMultiLED_LEDon & 0x40);
  blink = (masterMultiLED_LEDblink & 0x40);
  if (on && !blink) strLowBattery = "low battery";
  if (on && blink) strLowBattery = "<i>low battery</i>";
  if (!on && blink) strLowBattery = "<i>low battery</i>";
  //temperature:
  String strTemperature = "";
  on = (masterMultiLED_LEDon & 0x80);
  blink = (masterMultiLED_LEDblink & 0x80);
  if (on && !blink) strTemperature = "temperature";
  if (on && blink) strTemperature = "<i>temperature</i>";
  if (!on && blink) strTemperature = "<i>temperature</i>";
  //build webpage
  String webpage = "<!DOCTYPE html><html><head><title>ESP32 Multiplus ESS</title></head><body><font size=\"7\">";  //font size 7 is largest allowed
  webpage += "<p><b>ESP32 Multiplus ESS</b></p>";
  if ((soc >= 0) && (soc <= 100)) {   //if SOC wihin valid range, show it as number on webpage
    webpage += "Battery:    "+String(soc)+"%<br>";
  } else {
    webpage += "Battery:    CAN bus connection failure! Multiplus disabled.<br>";
  }
  if (meterPower >= 0) webpage += "PowerMeter: +"+String(meterPower)+"W<br>";
  else webpage += "PowerMeter: "+String(meterPower)+"W<br>";
  webpage += "ESS power:  "+String(multiplusESSpower)+"W ("+String(int(round(powerACin)))+"W)<br>";
  webpage += "ESP32 Switch-Mode: '"+String(switchMode)+"'<br>";
  webpage += "ESS power update strategy: "+String(essPowerStrategy)+"<br>";
  webpage += "Shelly 800W: "+String(shellyStatus)+"<br>";
  webpage += "<br>";
  webpage += "<table>";
  webpage += "<tr><th>charger</th><th>inverter</th></tr>";
  webpage += "<tr><td>&nbsp;&nbsp;&nbsp;"+strMainsOn+"</td><td>&nbsp;&nbsp;&nbsp;"+strInverterOn+"</td></tr>";
  webpage += "<tr><td>&nbsp;&nbsp;&nbsp;"+strBulk+"</td><td>&nbsp;&nbsp;&nbsp;"+strOverload+"</td></tr>";
  webpage += "<tr><td>&nbsp;&nbsp;&nbsp;"+strAbsorption+"</td><td>&nbsp;&nbsp;&nbsp;"+strLowBattery+"</td></tr>";
  webpage += "<tr><td>&nbsp;&nbsp;&nbsp;"+strFloat+"</td><td>&nbsp;&nbsp;&nbsp;"+strTemperature+"</td></tr>";
  webpage += "</table>";
  webpage += "<br>";
  webpage += "Multiplus DC current: "+String(multiplusDcCurrent,1)+"A ("+String(int(round(multiplusDcCurrent*multiplusDcVoltage)))+"W)<br>";
  webpage += "Multiplus DC voltage: "+String(multiplusDcVoltage, 2)+"V<br>";
  webpage += "Multiplus ACin voltage: "+String(multiplusUMainsRMS, 2)+"V<br>";
  webpage += "Multiplus AC frequency: "+String(multiplusAcFrequency, 3)+"Hz<br>";
  webpage += "Multiplus power factor: "+String(multiplusPowerFactor, 4)+"<br>";
  webpage += "Multiplus capacity in-out: "+String(multiplusBatteryAh)+"Ah ("+String(multiplusBatteryAh*NOM_VOLT/1000.0,1)+"kWh)<br>";
  webpage += "Multiplus temperature: "+String(multiplusTemp,1)+"&deg;C<br>";
  webpage += "<br>";
  webpage += "MasterMultiLED Status: "+String(masterMultiLED_Status)+"<br>";
  webpage += "Charger/Inverter Status: "+String(multiplusStatus80)+"<br>";
  webpage += "Voltage status: 0x"+String(multiplusVoltageStatus,HEX)+"<br>";
  webpage += "Emergency power status: "+String(multiplusEmergencyPowerStatus)+"<br>";
  webpage += "AC input current limit: "+String(masterMultiLED_ActualInputCurrentLimit,1)+"A<br>";
  webpage += "AC input configuration: 0x"+String(masterMultiLED_AcInputConfiguration,HEX)+"<br>";
  webpage += "MasterMultiLED switch value: 0x"+String(masterMultiLED_SwitchRegister,HEX)+"<br>";
  webpage += "Multiplus battery frame byte 07: 0x"+String(multiplusBattery_byte07,HEX)+"<br>";
  webpage += "Multiplus battery frame byte 06: 0x"+String(multiplusBattery_byte06,HEX)+"<br>";
  webpage += "Multiplus battery frame byte 05: 0x"+String(multiplusBattery_byte05,HEX)+"<br>";
  webpage += "Multiplus E4 frame byte 18: 0x"+String(multiplusE4_byte18,HEX)+"<br>";
  webpage += "Multiplus E4 frame byte 17: 0x"+String((multiplusE4_byte17 >> 4),HEX)+"<br>";
  webpage += "Multiplus E4 frame byte 12: 0x"+String(multiplusE4_byte12,HEX)+"<br>";
  webpage += "Multiplus E4 frame byte 11: 0x"+String(multiplusE4_byte11,HEX)+"<br>";
  webpage += "<br>";
  webpage += "VE.Bus TX frames failed: "+String(veTxCmdFailCnt)+"/"+String(veCmdCounter)+"<br>";
  webpage += "VE.Bus RX frames wrong checksum: "+String(veRxCmdFailCnt)+"<br>";
  webpage += "<br>";
  webpage += "<b>Electric meter</b><br>";
  webpage += "<table>";
  webpage += "<tr><td>ID:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+meterDeviceIdToStr(electricMeter_DeviceID)+"</td></tr>";
  webpage += "<tr><td>Consumption:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(electricMeter_Consumption)+" kWh</td></tr>";
  webpage += "<tr><td>Feed-in:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(electricMeter_FeedIn)+" kWh</td></tr>";
  webpage += "<tr><td>Status 1.8.0:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(electricMeter_Status180[0],HEX)+" "+String(electricMeter_Status180[1],HEX)+" "+String(electricMeter_Status180[2],HEX)+"</td></tr>";
  webpage += "<tr><td>Status 2.8.0:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(electricMeter_Status280[0],HEX)+" "+String(electricMeter_Status280[1],HEX)+" "+String(electricMeter_Status280[2],HEX)+"</td></tr>";
  webpage += "<tr><td>Runtime:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(electricMeter_Runtime)+" sec = "+String(electricMeter_Runtime/(60*60*24))+" days</td></tr>";
  webpage += "<tr><td>Power L1:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(electricMeter_PowerL1)+"W</td></tr>";
  webpage += "<tr><td>Power L2:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(electricMeter_PowerL2)+"W</td></tr>";
  webpage += "<tr><td>Power L3:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(electricMeter_PowerL3)+"W</td></tr>";
  webpage += "<tr><td>Power (balanced):</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(electricMeter_Power)+"W</td></tr>";
  webpage += "</table>";
  webpage += "<br>";
  webpage += "Time from meter runtime: "+secondsToTimeStr(electricMeter_Runtime+ELECTRIC_METER_TIME_OFFSET,true)+"<br>";
  webpage += "Status 1.8.0 and 2.8.0 differences: "+String(electricMeterStatusDifferent)+"<br>";
  webpage += "Wrong CRC in Info-DSS messages: "+String(electricMeterCRCwrong)+"<br>";
  webpage += "<br>";
  webpage += "Battery min: "+String(debugmin)+"%<br>";
  webpage += "Battery max: "+String(debugmax)+"%<br>";
  webpage += "ACin Umin: "+String(acVoltageMin, 2)+"V ("+secondsToTimeStr(timeAcVoltageMin+ELECTRIC_METER_TIME_OFFSET,true)+")<br>";
  webpage += "ACin Umax: "+String(acVoltageMax, 2)+"V ("+secondsToTimeStr(timeAcVoltageMax+ELECTRIC_METER_TIME_OFFSET,true)+")<br>";
  webpage += "AC frequency min: "+String(acFrequencyMin, 3)+"Hz ("+secondsToTimeStr(timeAcFrequencyMin+ELECTRIC_METER_TIME_OFFSET,true)+")<br>";
  webpage += "AC frequency max: "+String(acFrequencyMax, 3)+"Hz ("+secondsToTimeStr(timeAcFrequencyMax+ELECTRIC_METER_TIME_OFFSET,true)+")<br>";
  webpage += "DC Umin: "+String(dcVoltageMin, 2)+"V<br>";
  webpage += "DC Umax: "+String(dcVoltageMax, 2)+"V<br>";
  webpage += "DC discharge current max: "+String(dcCurrentMin,1)+"A ("+String(int(round(dcCurrentMin*NOM_VOLT)))+"W)<br>";
  webpage += "DC charge current max: "+String(dcCurrentMax,1)+"A ("+String(int(round(dcCurrentMax*NOM_VOLT)))+"W)<br>";
  webpage += "Multiplus temp. min: "+String(multiplusTempMin,1)+"&deg;C<br>";
  webpage += "Multiplus temp. max: "+String(multiplusTempMax,1)+"&deg;C<br>";
  webpage += "<br>";
  webpage += "<a href=\"logfile.txt\">logfile.txt of last actions</a><br>";
  webpage += String(logfileCounter)+" bytes written since last logfile view";
  webpage += "</font></body></html>";
  server.send(200, "text/html", webpage);
}

// ===========================================================================
//                Sends logfile buffer as logfile.txt via HTTP
// ---------------------------------------------------------------------------
// This function handles displaying/sending a file called logfile.txt as soon
// as one clicks on the link on the ESP32 webpage.
// The logfile has a size of (LOGFILE_SIZE) and is overwritten like a
// ringbuffer. So when it's full, oldest information is overwritten first.
// Improvement:
// Normally I would've displayed the logfile always from oldest to newest
// information. But with my limited programming skills I didn't find a way to
// do this without wasting an additional copy buffer in RAM. That's why I'm
// currently sending the buffer as it is. And before sending it, I simply add
// a "bar" into it, showing that above this bar it ends with the newest
// information and below this bar it starts with the oldest information.
// ===========================================================================
void handleLogfile() {
  int tmp = p_logfile;    //save logfile position pointer
  addToLogfile("\n######## newest entry above ########\n\n\n\n\n\n\n\n\n\n\n######## oldest entry below ########\n");  //temporary text marker
  server.send(200, "text/plain", logfile);
  p_logfile = tmp;    //restore original pointer so that temporary text will be overwritten again
  if (!stopWritingLogfile) logfileCounter = 0;     //viewing the logfile resets the logfile size counter (but only if logging will NOT end soon)
}




// ################################
//    ____       _               
//   / ___|  ___| |_ _   _ _ __  
//   \___ \ / _ \ __| | | | '_ \ 
//    ___) |  __/ |_| |_| | |_) |
//   |____/ \___|\__|\__,_| .__/ 
//                        |_|    
// ################################
void setup() {
  //settings CAN/TWAI bus to battery
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  //Look in the api-reference for other speed sets.
  twai_filter_config_t f_config = {.acceptance_code = (0x355 << 21),        //only filter 0x355 messages containing SOC (battery level)
                                                 .acceptance_mask = ~(0x7FF << 21),
                                                 .single_filter = true};
  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("Driver installed");
  } else {
    Serial.println("Failed to install driver");
    return;
  }
  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("Driver started");
  } else {
    Serial.println("Failed to start driver");
    return;
  }
  //Set up the LCD's number of columns and rows:
  lcd.begin(20, 4);
  //initialize charge/discharge mode options based on default mode
  switchSpecialModes(SWITCH_MODE_DEFAULT);
  //Setup serial ports
  Serial.begin(115200);  //SerialMonitor for debug information
  Serial1.begin(256000, SERIAL_8N1, VEBUS_RXD1, VEBUS_TXD1);  //VE.Bus RS485 to Multiplus
  Serial2.begin(9600, SERIAL_8N1, METER_SML_INPUT_PIN, SML_TXD_UNUSED); //, true);  //SML data from power meter //invert=false/true
  //IO pins
  pinMode(BUTTON_GPIO, INPUT_PULLUP);  //Initialize BOOT button GPIO as input with pull-up resistor
  pinMode(VEBUS_DE, OUTPUT);           //RS485 RE/DE direction pin for UART1
  digitalWrite(VEBUS_DE, LOW);         //set RS485 direction to read
  pinMode(METER_IMPULSE_INPUT_PIN, INPUT);//Power meter optical 1/10000kWh impulse input, currently evaluated in ISR
  pinMode(RED_LED, OUTPUT);            //Red LED currently showing the optical impulse from the power meter
  digitalWrite(RED_LED, LOW);          //turn red LED off
  //Init webserver
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  MDNS.begin("esp32");
  server.on("/", handleRoot);
  server.on("/logfile.txt", handleLogfile);
  server.begin();
  //set up interrupt timer for ISR
  My_timer = timerBegin(0, 2, true);  //2nd value is prescaler, which must be minimum 2 = 40MHz
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 4000, true);  //value at which ISR is executed, cannot be 0, 1220 means 32760Hz, 4000 means 10kHz
  timerAlarmEnable(My_timer);
  //Now execute the VE.Bus handling code on core0, where it's executed by FreeRTOS in 1ms slices:
  xTaskCreatePinnedToCore(veBusHandling, "VEBus", 10000, NULL, 0, &Task0, 0);   //Priority 0, CPU 0
  //Wait a second before starting the main loop
  delay(1000);
}

void veBusHandling(void * parameter)
{
  //Before starting, clear the VE.Bus RX buffer
  Serial1.flush(false);        //TXonly=false -> including RX buffer
  //Now run the following code continiously
  while(true)
  {
    //Check for new bytes on UART
    while (Serial1.available())
    {
      char c = Serial1.read();  //read one byte
      frbuf0[frp++] = c;        //store into framebuffer
      if (c==0xFF)              //in case current byte was EndOfFrame, interprete frame
      {
        if ((frbuf0[2] == 0xFD) && (frbuf0[4] == 0x55))  //if it was a sync frame:
        {
          frameNr = frbuf0[3];
          synccnt++;
          if (synccnt == 5) {   //every 5th sync frame, meaning 10 times per second, CommandGetRAMVarInfo is sent, NO re-send
            synccnt = 0;    //reset counter
            //build desired command
            int len = 0;
            len = prepareCommandReadRAMVar(txbuf1, (frameNr+1) & 0x7F);
            //postprocess command
            len = commandReplaceFAtoFF(txbuf2, txbuf1, len);
            len = appendChecksum(txbuf2, len);
            //write command into Multiplus :-)
            digitalWrite(VEBUS_DE,HIGH);          //set RS485 direction to write
            Serial1.write(txbuf2, len);           //write command bytes to UART
            Serial1.flush();                      //simply wait until the command is sent out
            digitalWrite(VEBUS_DE,LOW);           //set RS485 direction to read
            veCmdCounter++;                       //amount of VE.Bus commands sent (successful + unsuccessful)
          }
          //avoid sending command in (synccnt == 1 or 4) to avoid conflict of command or reply with ReadRAMVar command or reply
          else if ((synccnt>=2) && (synccnt<=2) && (veCmdSendState > 0) && !(veCmdSendState & 0x10)) {    //if a new VE.Bus command is waiting to be sent (excluding acknowledgements)
            //build desired command
            int len = 0;
            if (veCmdSendState == 2)                                 len = prepareESScommand(txbuf1, essPowerDesired, (frameNr+1) & 0x7F); //write new ESS power
            else if ((veCmdSendState >= 4) && (veCmdSendState <= 7)) len = prepareSwitchCommand(txbuf1, veCmdSendState, (frameNr+1) & 0x7F);//7=Normal 6=InvertOnly 5=ChargeOnly 4=Standby
            else if (veCmdSendState == 9)                            len = prepareCommandGetRAMVarInfo(txbuf1, (frameNr+1) & 0x7F);
            //postprocess command
            len = commandReplaceFAtoFF(txbuf2, txbuf1, len);
            len = appendChecksum(txbuf2, len);
            //write command into Multiplus :-)
            digitalWrite(VEBUS_DE,HIGH);          //set RS485 direction to write
            Serial1.write(txbuf2, len);           //write command bytes to UART
            Serial1.flush();                      //simply wait until the command is sent out
            digitalWrite(VEBUS_DE,LOW);           //set RS485 direction to read
            cmdAckCnt = WAITING_TIME_ACK;         //set counter with max. waiting time for ACK
            veCmdCounter++;                       //amount of VE.Bus commands sent (successful + unsuccessful)
            veCmdSendState = veCmdSendState | 0x10;   //wait for confirmation if command was successful
          }
        }
        else if ((frp==5) && (frbuf0[0] == 0x00) && (frbuf0[1] == 0xFB) && (frbuf0[2]+frbuf0[3] == 0x7F) && (frbuf0[4] == 0xFF))
        { //5-bytes frames coming when Multiplus boots and when ACin disconnects
          //do nothing for now and ignore those frames
        }
        else
        {    //If frame was not sync, and not the bootup frame above, let's decode it:
          if (verifyChecksum(frbuf0, frp)) {
            int frlen = destuffFAtoFF(frbuf1, frbuf0, frp);
            if (not decodeVEbusFrame(frbuf1, frlen)) addFrameToLogfile(frbuf1,0,frlen-1);
          }
          else {
            //only add wrong frame to logfile, don't decode
            addToLogfile("\nwrong CS:");
            addFrameToLogfile(frbuf0,0,frp-1);  //add complete frame into logfile
            veRxCmdFailCnt++;                   //amount of received commands with wrong checksum
          }
        }
        //now framebuffer can be cleared
        frp = 0;
      }
    }
  }
}

// #################################################
//   __  __       _         _                      
//  |  \/  | __ _(_)_ __   | |    ___   ___  _ __  
//  | |\/| |/ _` | | '_ \  | |   / _ \ / _ \| '_ \ 
//  | |  | | (_| | | | | | | |__| (_) | (_) | |_) |
//  |_|  |_|\__,_|_|_| |_| |_____\___/ \___/| .__/ 
//                                          |_|    
// #################################################
void loop() {
  automaticChargerOnlySwitching();            //Switch Multiplus into ChargerOnly-Mode if battery is below specific level
  veCmdReSendHandling();                      //Multiplus VEbus: Process VEbus information and execute command-sending state machine
  batteryHandling();                          //Battery: Get new battery level (SOC value) from CAN bus, if available
  checkSMLpowerMeter();
  //checkImpulsePowerMeter();                   //Impulse power meter: Check if new 1/10000kWh impulse is available and calculate meter value
  onNewMeterValue();                          //Power meter: If available, add new meter value to logfile and detect power spikes
  //updateMultiplusPower_usingAbsoluteMeter();  //Evaluate absolute meter power, and if applicable, apply new ESS power value
  updateMultiplusPower_usingSignedMeter();    //Based on electrical meter power, calculates and sends new ESS power into Multiplus.
  avoidMultiplusTimeout();                    //Ensures that at least every 10 seconds Multiplus gets an ESS command. Required if ImpulsePowerMeter and in case of Emergency power.
  shellyControl();
  switchSpecialModes(0);                      //handle BOOT button press activities (GPIO0)
  updateDisplays();                           //update Display and LEDs
  server.handleClient();                      //handle webserver HTTP request
}
