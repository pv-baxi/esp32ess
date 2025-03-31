/*
          Victron Multiplus 2 ESS using ESP32 controlling VE.Bus

Implements an ESS system with Victron Multiplus II and Pylontech batteries.
Instead by a Cerbo GX, the Multiplus is directly controlled by the ESP32 via
the VE.Bus.

NOTE:
Code was not working with Expressif esp32 board library 3.0.4. Version 3.1.1
and later was verified to work again. 

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
#include <WiFi.h>
#include <WebServer.h>
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
const int SHELLY_CF = 23;                             //Pin with frequency pulses resulting in desired "power" on external Shelly 1PM
const int VEBUS_RXD1=26, VEBUS_TXD1=25, VEBUS_DE=27;  //Victron Multiplus VE.bus RS485 gpio pins
const int METER_IMPULSE_INPUT_PIN = 35;               //input pin from optical 1/10000kWh impulse output of power meter
const int METER_SML_INPUT_PIN = 34;                   //input pin from optical SML data output of power meter
const int SML_TXD_UNUSED = 17;                        //some GPIO we need to wire the unused UART TX to
const int RED_LED = 2;                                //output gpio pin with external red LED (active low)
const int CPS = 10000;                  //CyclesPerSecond = How often per second our timer ISR is excuted
const int WAITING_TIME_ACK = 0.9*CPS;   //How long we wait for the ESS command to be acknowledged by Multiplus. Comes normally between 25..43ms, so 1000 = 100ms makes sense
const int8_t SOC_ADAPT_LOWEST = 10;     //Reset socInverterOnOff to this value as soon as battery.soc reached 100% and is thus balanced again. Default = 10%
const int8_t SOC_ADAPT_HIGHEST = 95;    //socInverterOnOff cannot become higher than this. Default = 95%
const int8_t SOC_ADAPT_STEP = 3;        //increase socInverterOnOff by this amount of percent every day, until SOC_ADAPT_HIGHEST is reached or battery.soc reaches 100%, default=3%, off=0
const int8_t SOC_TH_STRATEGY_OFFSET = 40; //Below (socInverterOnOff+SOC_TH_STRATEGY_OFFSET) with 2x Pylontech batteries, minimum power compensation strategy is used
const int ESS_TIMEOUT_PREVENT = 5*CPS;  //Multiplus ESS is automatically disabled about 15 sec after last command written (FW v508), so every 5s we re-send ESS power, also to be safe in case of http timeouts
const int BUTTON_DEBOUNCE_TIME = 0.1*CPS;//0.1 seconds gives good results
const int BUTTON_LONGPRESS_TIME = 1.0*CPS;//1.0 seconds for a longpress
const int SOC_WATCHDOG_CYCLES = 2.5*CPS;//time after which Multiplus is turned off in case there was no SOC received from battery (2.5 seconds)
const char SWITCH_MODE_DEFAULT = 'A';   //Default switch mode to start with. For mode characters see function switchSpecialModes()
const float NOM_VOLT = 48.0;            //Battery nominal voltage (for kWh estimation)
const float POWER_CAL = 0.998;   //Baxi=0.998: Calibration factor from powerMeter to Multiplus power. Multiplus should not apply more power than meter shows to keep regulation stable.
const int SIZE_POWER_TREND_RINGBUF = 15*60;     //How many last electric meter values are analysed for automatic mode switching (15 minutes)
const int AUTOMATIC_MAXIMUM_MODE_DURATION = 30*60*CPS;  //if poor compensation is detected, switch to Min/Max strategy for 30 minutes
const int POWER_TREND_TH = 10;            //10Wh (Turning on 2200W once costs ~2.6Wh) (7Wh seemed too low, Wasching machine is abbout 1Wh/min)
const int CONSUMER_SHELLY_SWITCHING_INTERVAL = 3*60;  //IN SECONDS, use minimum 3 minutes to avoid problems with starting compressor motors
const int ELECTRIC_METER_TIME_OFFSET = 9*60*60 +57*60 +7;  //To get time from meter runtime value (meter was installed in early summer at ~8am)
const int16_t TRICKLE_CHARGE_POWER = 45;  //AC Power which is at minimum charged into battery in ChargeOnly case (includes Multiplus standby power)
                                          //35W was not enough during night with 2 batteries, but seems to be exactly 0W BMS power with 1 battery
                                          //40W still not enough during day while boiler turns on/off
const int16_t CONSTANT_CHARGE_POWER = 540;  //AC Power in Watt which is used to charge the battery in 'C' mode (minus Multiplus standby power, minus 93% AC-DC eff.) 385W -> 350W BMS, 540W -> 500W BMS
const int16_t POWER_TH_SHELLY_PLUS = 2100;  //If minimum feed-in is more than this, an additional consumer Shelly is turned on
const int16_t POWER_TH_SHELLY_MINUS = 50;   //If minimum feed-in is less than this, the last consumer Shelly is turned off again

const size_t N_CONSUMER_SHELLYS = 5;
const char * CONSUMER_IP[N_CONSUMER_SHELLYS]={    //If there is excess power, 1st IP in list is enabled first, last IP is enabled last
    "192.168.178.6",    //0 = warm water boiler 2000W
    "192.168.178.15",   //1 = small heater 500W
    "192.168.178.14",   //2 = Shelly Plug S white 800W
    "192.168.178.12",   //3 = switched socket 2000W
    "192.168.178.16"    //4 = Shelly Plug S black 1000W
    // "192.168.178.11",   //5 = triple socket outlet
    // "192.168.178.13",   //6 = dehumidifier 3L 210W
 };

struct SWITCH_CONDITION
{
    int8_t  shellyNr;   //ref to list of network IPs
    int8_t  hourOn;     //0..23
    int8_t  minOn;      //0..59
    int8_t  hourOff;    //0..23
    int8_t  minOff;     //0..59
    int8_t  batteryThr; //0..100%
    bool    startedChg; //true = battery must've charged a few percent this day; false = don't care, also allow rule during night
};
const SWITCH_CONDITION CONSUMER_RULES[] = {//};         //Note: Rule batteryThr goes on-top of adaptive inverter-On/Off-threshold ("0" means same as inverter on/off, "10" means 10% higher)
// shelly hourOn:minOn hourOff:minOff batteryThr startedChg
     {0,     6,     0,    10,    30,      15,      true},   //warm water boiler - morning
     {0,    10,    30,    12,     0,      20,      true},   //warm water boiler - noon
     {0,    12,     0,    14,     0,      35,      true},   //warm water boiler - afternoon
     {0,    14,     0,    22,     0,      65,      true}    //warm water boiler - evening
//   {5,     9,     0,    18,     0,      70,      true},   //dehumidifier 210W 3L
//   {1,     7,     0,    22,    50,      70,      true},   //small heater 500W
//   {2,     7,     0,    17,     0,      71,      true},   //Shelly Plug S 800W
//   {3,     6,    15,    22,     0,      72,      true},   //switched socket 2000W
//   {4,    12,     0,    24,     0,      73,      true},   //triple socket outlet 1000W
//   {6,     8,     0,    18,     0,      73,      true}    //dehumidifier 7L 250W
// shelly hourOn:minOn hourOff:minOff batteryThr startedChg
 };



// #############################################################################
//    ____ _       _           _                    _       _     _           
//   / ___| | ___ | |__   __ _| |  __   ____ _ _ __(_) __ _| |__ | | ___  ___ 
//  | |  _| |/ _ \| '_ \ / _` | |  \ \ / / _` | '__| |/ _` | '_ \| |/ _ \/ __|
//  | |_| | | (_) | |_) | (_| | |   \ V / (_| | |  | | (_| | |_) | |  __/\__ \
//   \____|_|\___/|_.__/ \__,_|_|    \_/ \__,_|_|  |_|\__,_|_.__/|_|\___||___/
//
// #############################################################################

bool chargeFromACin = true;      //allow negative ESS power (ACin-based charging from power "before" the Multiplus = ACin, L2 or L3). Set to false if PV is only connected to ACout1
bool timeIsValid = false;        //Set to true once we got a valid time, e.g. (electricMeter_Runtime + ELECTRIC_METER_TIME_OFFSET) from power meter
bool chargeOnly = false;
const int8_t BATTERY_STARTED_CHARING_OFFSET = 5;
bool batteryStartedCharging = true; //To monitor once battery charged again (BATTERY_STARTED_CHARING_OFFSET percent) on a new day
char frbuf0[256];                   //assembles one complete frame received by Multiplus
char frbuf1[256];                   //frame without replacement
char txbuf1[64];                    //buffer for assembling bare command towards Multiplus (without replacements or checksum)
char txbuf2[64];                    //Multiplus output buffer containing the final command towards Multiplus, including replacements and checksum
//char str[3*128+1];                //Buffer to temporarily store commands from/to Multiplus in HEX format and send it to SerialMonitor for debugging. +1 for being able to terminate our string with zero
uint16_t frp = 0;                   //Pointer into Multiplus framebuffer frbuf[] to store received frames.
byte frameNr = 0;                   //Last frame number received from Multiplus. Own command has be be sent with frameNr+1, otherwise it will be ignored by Multiplus.
int synccnt = 0;                    //Counts every 5 received sync frames in veBusHandling(), sending CommandReadRAMVar 10 times per second
int veCmdCounter = 0;               //Counts the number of ESS commands sent to Multiplus, including the failed ones.
int16_t essPowerTmp = 0;            //continiously updated ESS power value coming out ouf calculation
int16_t essPowerTmp2 = 0;           //ESS power value after calculation and after discharge/charge power limitation
int16_t essPowerDesired = 0;        //This value will be written into Multiplus when setting veCmdSendState = 2; negative = charging battery; positive = feed into grid
int impulseMeterPower = 0;          //electric meter value measured from 1/10kWh impulses
int decisiveMeterPower = 0;         //electric meter value used in calculations and decisions
byte veCmdSendState = 0;            //2 ready to send; 0x1X has been send but no ACK yet; 0 acknowledged=done; 7 set Charger+Discharger mode; 5 set Charge-Only mode
int veTxCmdFailCnt = 0;             //Counts the number of commands that were not acknowledged by Multiplus and had to be resent.
int veRxCmdFailCnt = 0;             //amount of received VE.Bus commands with wrong checksum
char switchMode = SWITCH_MODE_DEFAULT;//Switches the program between different user modes, see function switchSpecialModes().
const int LOGFILE_ALL_SIZE = 60000; //Maximum text buffer that can be sent via HTTP is 65519 byte (2^16 -1 -16)
char logfileAll[LOGFILE_ALL_SIZE];  //logfile buffer, used as ringbuffer with pointer below
int p_logfileAll = 0;               //pointer to logfileAll ringbuffer
int logfileAllCounter = 0;          //counts every byte written into logfileAll
const int LOGFILE_FEW_SIZE = 5510;  //Maximum text buffer that can be sent via HTTP is 65519 byte (2^16 -1 -16)
char logfileFew[LOGFILE_FEW_SIZE];  //logfile buffer, used as ringbuffer with pointer below
int p_logfileFew = 0;               //pointer to logfileFew ringbuffer
int logfileFewCounter = 0;          //counts every byte written into logfileFew
String webpage;
struct {                            //Struct of Pylontech BMS values:
  int16_t soc = -1;                   //Battery level 0-100% (negative = invalid)
  int16_t socMin = 999999;              //used to log minimum battery level
  int16_t socMax = -999999;             //used to log maximum battery level
  uint32_t socMinTime = -ELECTRIC_METER_TIME_OFFSET;
  uint32_t socMaxTime = -ELECTRIC_METER_TIME_OFFSET;
  int16_t soh = -1;                   //Battery health 0-100% (negative = invalid)
  float chargeVoltage = -1;
  float chargeCurrentLimit = -1;
  float dischargeCurrentLimit = -1;
  float dischargeVoltage = -1;
  float voltage = -1;
  float current = -1;
  int power = -1;
  float temperature = -1;
  char manufacturer[9] = {0};
  int8_t nrPacksInParallel = -1;
  uint8_t protectionFlags1 = 0;
  uint8_t protectionFlags2 = 0;
  uint8_t warningFlags1 = 0;
  uint8_t warningFlags2 = 0;
  uint8_t requestFlags = 0;
} battery;
int16_t socInverterOnOff = 50;  //Adaptive SOC-low threshold. Switch to charge-only if below, or back to Inverter+Charger if above this battery level; Increased every day to ensure that battery reaches 100% for balancing from time to time; start with 50% after reboot
double bmsPowerAverage = 0;
float alpha = 0.03;                                //averaging factor for bmsPowerAverage (0.03 is ~ 5 minutes)
float batteryTempMin = 999;
float batteryTempMax = -999;
float batteryCurrentMin = 999;
float batteryCurrentMax = -999;
int batteryPowerMin = 99999;
int batteryPowerMax = -99999;
bool newImpulseMeterPower = false;   //new power value is ready from impulse + resticted-digital meter output (less often/accurate)
bool newDigitalMeterPower = false;   //new power value is ready from full-digital meter output (full precision)
int infoDssCntSinceLastMeterPower = 0;  //counts all valid Info-DSS messages since last meter power value. Used to detect if meter currently delivers full or restricted data
bool newMeterValue = false;      //indicating to various functions that we got a new power meter measurement result
char sBuf[1024];                    //Circular buffer, stores received SML stream from power meter; Size must be power of 2 and bigger (or equal) than SML_LENGTH_MAX
uint16_t smlp = 0;                  //Pointer into SML buffer, used as circular pointer
const int SML_LENGTH_MAX = 512;     //This is the maximum length we allow for the SML stream (eBZ DD3 meter: 352 byte full / 256 byte reduced)
char sBuf2[SML_LENGTH_MAX];         //linear buffer with SML stream
uint16_t smlCnt = 0;                //counter to count length of SML stream
uint16_t smlLength = 0;             //detected length of SML stream
int gridSetpoint = 0;               //Power compensation offset: negative always feed a bit into the grid, positive always consumes a bit from the grid (-10W = 0.25kWh feed-in per day); NOTE: High positive values with (chargeFromACin==true) might result in charging battery from grid!

//Classes
TaskHandle_t Task0;
hw_timer_t *My_timer = NULL;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
WebServer server(80);

CRC16 crc(0x1021, 0xFFFF, 0xFFFF, true, true);
//Multiplus variables
int16_t   multiplusESSpower = -1;                  //ESS power value currently applied in Multiplus, init with -1, so that it will be very likely updated on start
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
byte    multiplusEmergencyPowerStatus = 0;      //00=mains, 02=in_emergency_power_mode (=state of ACin relay)
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
int16_t  multiplusPmainsFiltered;               //multiplusPmainsFiltered is always exactly -(multiplusPinverterFiltered+multiplusPoutputFiltered)
int16_t  multiplusPinverterFiltered;            //multiplusPinverterFiltered is always exactly -(multiplusPmainsFiltered+multiplusPoutputFiltered)
//int16_t  multiplusPoutputFiltered;              //multiplusPoutputFiltered is always exactly -(multiplusPmainsFiltered+multiplusPinverterFiltered)

//Power meter variables
char     electricMeter_DeviceID[10];                         //electric meter device ID (we assume 10 bytes)
const int MAX_METER_STATUS_BYTES = 4;
char     electricMeter_Status180[MAX_METER_STATUS_BYTES];    //Status sent with 1.8.0 (Only consumption status is sent by every meter.)
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
int      electricMeterCurrentSign = 0;                       //by default we are undecided and set power = 0W
int16_t  estTargetPower = 0;                                 //current ESS power target, calculated from electric meter and powerACin
int      essPowerStrategy = 5;                               //Strategy currently used: 7=maximum, 5=immediate, 3=minimum, 0=0W
const int SIZE_PWR_RINGBUF = 45;                             //45 seconds
int16_t estTargetPowerRingBuf[SIZE_PWR_RINGBUF] = {0};       //ringbuffer of estTargetPower of the last SIZE_PWR_RINGBUF seconds for ESS power strategies
int ptrPowerRingBuf = 0;                                     //start with 1st element in ringBuffer
const int SIZE_P_ACIN_RINGBUF = 32;                          //How many last multiplusPmainsFiltered values we want to store. Must be power of 2 !
int16_t pACinRingBuf[SIZE_P_ACIN_RINGBUF] = {0};             //ring buffer of last multiplusPmainsFiltered values, coming due to CommandReadRAMVar every 100ms
int ptr_pACinRingBuf = 0;                                    //start with 1st element in ringBuffer
int16_t powerACin = 0;                                       //power on ACin, synchronized from pACinRingBuf[], with value from electric meter
const int SIZE_P_INVERTER_RINGBUF = 32;                      //How many last multiplusPinverterFiltered values we want to store. Must be power of 2 !
int16_t pInverterRingBuf[SIZE_P_INVERTER_RINGBUF] = {0};     //ring buffer of last multiplusPinverterFiltered values, coming due to CommandReadRAMVar every 100ms
int ptr_pInverterRingBuf = 0;                                //start with 1st element in ringBuffer
int16_t powerInverter = 0;                                   //power out of inverter, synchronized from pInverterRingBuf[], with value from electric meter

int     shellyState = 0;                                     //Init with 0 = all consumers OFF (1=1st consumer ON, 2=1st+2nd consumer ON etc.)
int     shellyActuations = 0;                                //counts how often the set of Shellys have been controlled
int     shellyFails = 0;                                     //counts how often a single Shelly did not acknowledge a switch http command
int8_t  shellyConsumer[N_CONSUMER_SHELLYS] = {0};            //per Shelly: 0 = unknown state, -1 = off, +1 = on
int     nrShellysEnabledByRule = 0;                          //How many Shellys are currently turned ON by CONSUMER_RULES[]

const int SIZE_SHELLY_POWER_RINGBUFFERS = CONSUMER_SHELLY_SWITCHING_INTERVAL - 30;     //use 30 seconds less to give consumers some time to start before DC current averagaing begins
int16_t powerChargerRingBuf[SIZE_SHELLY_POWER_RINGBUFFERS] = {0};                      //ringbuffer of AC power: pos=charging, neg=inverting
int     ptr_powerChargerRingBuf = 0;
int32_t averageChargingPower = 0;
int16_t powerControlDeviationRingBuf[SIZE_SHELLY_POWER_RINGBUFFERS] = {0};           //positive values mean feed-in; negative values mean consumption
int     ptr_powerControlDeviationRingBuf = 0;
int32_t averageControlDeviationFeedIn = 0;
int16_t powerMeterRingBuf[SIZE_SHELLY_POWER_RINGBUFFERS] = {0};           //positive values mean feed-in; negative values mean consumption
int     ptr_powerMeterRingBuf = 0;
int32_t minimumFeedIn = 0;

double  electricMeterHourlyConsumption[24] = {0};           //consumptions for each expired hour [0]=23:00..0:00 ; [9]=8:00..9:00 ; [23]=22:00..23:00
double  electricMeterConsumptionOneHourAgo = -100;          //negative means value not initialized yet
double  electricMeter24hConsumption = 0;                    //sum of consumption over all 24 hourly values
double  electricMeterHourlyFeedIn[24] = {0};                //grid feed-in for each expired hour [0]=23:00..0:00 ; [9]=8:00..9:00 ; [23]=22:00..23:00
double  electricMeterFeedInOneHourAgo = -100;               //negative = value not initialized yet
double  electricMeter24hFeedIn = 0;                         //sum of feed-in over all 24 hourly values
int     timeNewHourDone = -100;                             //negative = value not initialized yet

double multiplusDcVoltageCalibration = 0;                   //averaging calibration offset, updated whenever DC current < 1.5A
uint32_t multiplusDcVoltageCalibrationCnt = 0;              //How often DC voltage calibration was updated
double chargeCableResistance = 0;                           //averaged from Multiplus-BMbetaS voltage difference divided by BMS current (in mOhm)
uint32_t chargeCableResistanceCnt = 0;                      //How often chargeCableResistance was updated
double dischargeCableResistance = 0;                        //averaged from Multiplus-BMS voltage difference divided by BMS current (in mOhm)
uint32_t dischargeCableResistanceCnt = 0;                   //How often dischargeCableResistance was updated
float beta = 0.001;                                         //averaging factor for cable resistances

int16_t powerTrendRingBuf[SIZE_POWER_TREND_RINGBUF] = {0};          //to calculate 15min power trend, used by automatic strategy switching
int ptr_powerTrendRingBuf = 0;                                  //start with 1st element in ringBuffer
float powerTrendConsumption = 0;                            //summed positive (consumption) power over complete ringbuffer (in Ws if meter value comes every second)
float powerTrendFeedIn = 0;                                 //summed negative (feed-in) power over complete ringbuffer (in Ws if meter value comes every second)
uint32_t secondsInMinStrategy = 0;                       //counts how many seconds we used Min ESS power strategy
uint32_t secondsInMaxStrategy = 0;                       //counts how many seconds we used Max ESS power strategy

bool oneSecondOver = false;        //set to true every second (to write the time into logfile)
bool oneMinuteOver = false;        //set to true every minute (to write the time into logfile)




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
volatile int automaticMinMaxModeCnt = 0;    //Counter turning off Maximum mode in 'A' and 'S' automatic programs
volatile int buttonPressCnt = -1;           //Initialize button-press counter in locked (paused) state
volatile int cylTime = 0;                   //counts ISR calls up to one second (= CPS) and is than reset to 0 again
volatile int shellyWaitCnt = CONSUMER_SHELLY_SWITCHING_INTERVAL*CPS;  //Shelly-control waiting timer. Start not before 3 minutes.
volatile bool isrOneSecondOver = false;     //set to true every second
volatile int minuteTimer = 0;               //relative time, unit = 100ms, on 1000 resets to 0
volatile bool isrOneMinuteOver = false;     //set to true every minute
//for digital meter pulse measurement:
volatile bool IRpinDisplay = LOW;        //remember state of IR input pin to let LED blink once in main loop
volatile int highCnt = 1000;                //how long the high-pulse took. Normal is 20 = 2ms. Initialize with higher value, meaning no valid measurement
volatile int totalCnt = 4000000;            //how long the complete measurement (high-pulse plus low time) took. Initialize with some very high value meaning very low power
volatile int highCycles = 0;                //length of the concluding high pulse
volatile int highCyclesPrevious = 0;        //to validate if a power measurement also include the length of the preceding high pulse
volatile int totalCycles = 0;               //length of complete power measurent (high+low pulse)
volatile bool meterHighPulse = false;    //current state: true = within high-pulse; false = within low pulse
volatile bool isrNewMeterPulse = false;  //indicating that new meter impulse came in
volatile int shelly1PMcnt = 0;
volatile int shelly1PMpulsewidth = 100;

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
  if (automaticMinMaxModeCnt > 0) automaticMinMaxModeCnt--;
  if (shellyWaitCnt > 0) shellyWaitCnt--;
  cylTime++;
  if (cylTime >= CPS) {
    cylTime = 0;              //reset subsecond-timer
    isrOneSecondOver = true;  //set one-second-flag from ISR
    minuteTimer++;            //this is a 1 second counter
    if (minuteTimer >= 60) {
      minuteTimer = 0;          //overflow at 60 seconds
      isrOneMinuteOver = true;  //set one-minute-flag from ISR
    }
  }
  //Code for outputting SOC to external Shelly 1PM
  shelly1PMcnt++;
  if (shelly1PMcnt == shelly1PMpulsewidth-1) digitalWrite(SHELLY_CF, HIGH);
  else if (shelly1PMcnt >= shelly1PMpulsewidth) {
    digitalWrite(SHELLY_CF, LOW);
    shelly1PMcnt = 0;
  }
  //============================================================================================================
  //The following code is for reading the optical 1/10000kWh impulses from the power meter.
  // Advantage: Works with every digital power meter. No pin-code required.
  // Disadvantage: Power value is always positive, there is no information if currently consumption or feed-in.
      totalCnt++;
      highCnt++;
      bool IRpin = digitalRead(METER_IMPULSE_INPUT_PIN); //read IR input at this moment
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
  int h = 0;
  int m = 0;
  int s = 0;
  secondsToTime(h, m, s, seconds);
  char time[9];
  if (includeSeconds) sprintf(time,"%02d:%02d:%02d",h,m,s); else sprintf(time,"%02d:%02d",h,m);
  return time;
}


void relativeTimers()
//This function ensures that the time flags are true for exactly one execution of the loop() function and then false again.
{
  //at first, reset old relative time flags
  oneMinuteOver = false;
  oneSecondOver = false;
  //Generate new flags
  if (isrOneSecondOver)
  {
    isrOneSecondOver = false;
    oneSecondOver = true;
  }
  if (isrOneMinuteOver)
  {
    isrOneMinuteOver = false;
    oneMinuteOver = true;
  }
}


// ===========================================================================
//                      Adds string to logfile buffer
// ---------------------------------------------------------------------------
// After string is added to ringbuffer, pointer is incremented and wrapped if
// needed. Function does NOT add \n, this needs to be added manually.
// Number of bytes written to logfile are counted to be shown on main webpage.
// ===========================================================================
void addToLogfile(String s, int destination)
{
  //Add to ALL-Logfile if desired
  if (destination & 1) {
    for (int i=0; i<s.length(); i++)
    {
      logfileAll[p_logfileAll++]=s[i];
      if (p_logfileAll==LOGFILE_ALL_SIZE) p_logfileAll=0;   //wrap pointer around at end of buffer
      logfileAllCounter++;   //increase logfile size counter
    }
  }
  //Add to FEW-Logfile if desired
  if (destination & 2) {
    if (s[0]=='\n') {
      String t = "\n"+secondsToTimeStr(electricMeter_Runtime+ELECTRIC_METER_TIME_OFFSET,false);
      for (int i=0; i<t.length(); i++)
      {
        logfileFew[p_logfileFew++]=t[i];
        if (p_logfileFew==LOGFILE_FEW_SIZE) p_logfileFew=0;   //wrap pointer around at end of buffer
        logfileFewCounter++;   //increase logfile size counter
      }
    }
    for (int i=0; i<s.length(); i++)
    {
      logfileFew[p_logfileFew++]=s[i];
      if (p_logfileFew==LOGFILE_FEW_SIZE) p_logfileFew=0;   //wrap pointer around at end of buffer
      logfileFewCounter++;   //increase logfile size counter
    }
  }
}

void addFrameToLogfile(char *frame, int startbyte, int endbyte)
{
  logfileAll[p_logfileAll++]='\n';
  if (p_logfileAll==LOGFILE_ALL_SIZE) p_logfileAll=0;   //wrap pointer around at end of buffer
  logfileAllCounter++;   //increase logfile size counter
  for (int i=startbyte; i<=endbyte; i++)
  {
    char c = frame[i];
    char n = (c >> 4) | 48;
    if (n > 57) n = n+7;
    logfileAll[p_logfileAll++]=n;
    if (p_logfileAll==LOGFILE_ALL_SIZE) p_logfileAll=0;   //wrap pointer around at end of buffer
    logfileAllCounter++;   //increase logfile size counter
    n = (c & 0x0F) | 48;
    if (n > 57) n = n+7;
    logfileAll[p_logfileAll++]=n;
    if (p_logfileAll==LOGFILE_ALL_SIZE) p_logfileAll=0;   //wrap pointer around at end of buffer
    logfileAllCounter++;   //increase logfile size counter
    logfileAll[p_logfileAll++]=' ';
    if (p_logfileAll==LOGFILE_ALL_SIZE) p_logfileAll=0;   //wrap pointer around at end of buffer
    logfileAllCounter++;   //increase logfile size counter
  }
}



// ===========================================================================
//                   Assemble write-ESS-power VE.Bus command
// ---------------------------------------------------------------------------
// This function assembles the basic command to write the desired power towards
// the Multiplus ESS assistant. Documentation is here:
// https://github.com/pv-baxi/esp32ess/blob/main/docs/README.md#vebus-send-frames
// ===========================================================================
int prepareESScommand(char *outbuf, int16_t power, byte desiredFrameNr)
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
  outbuf[j++] = 0;              //0=UmainsRMS             1st RAM ID (up to 6 possible)
  outbuf[j++] = 15;             //15=PmainsFiltered       2nd RAM ID
  outbuf[j++] = 14;             //14=PinverterFiltered    3rd RAM ID
  //outbuf[j++] = 16;             //16=PoutputFiltered      4th RAM ID
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
            if (frame[12] != multiplusEmergencyPowerStatus) {   //If emergency power status has changed:
              multiplusEmergencyPowerStatus = frame[12];
              addToLogfile("\nmultiplusEmergencyPowerStatus = "+String(multiplusEmergencyPowerStatus),3);
            }
            if ((frame[11] & 0xF0) == 0x30) {
              multiplusTemp = frame[15]/10.0 *2.0 -8.0;  //my guess: (*2.0 - 8°C) as otherwise temperature is too low
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
        case 0x00:    //Acknowledgement to ESS power command and reply to CommandReadRAMVar
        {
          if ((len == 9) && (frame[5] == 0xE6) && (frame[6] == 0x87)) {
            if ((veCmdSendState==0x12) && (cmdAckCnt > 0))    //if ACK was received in time
            {
              multiplusESSpower = essPowerDesired;              //Update ESS power value successfully applied in Multiplus
              // if (minuteTimer < 10) addToLogfile("\n "+String(minuteTimer)); else addToLogfile("\n"+String(minuteTimer));
              // addToLogfile(" "+String(multiplusESSpower));     //Write applied ESS power to logfile
              addToLogfile(".",1);     //Indicate successful application to Multiplus with a dot only, to save Logfile space
              essTimeoutCounter = ESS_TIMEOUT_PREVENT;          //restart ESS timeout counter
              veCmdSendState = 0;                               //sending command is done
            }
            result = true; //mark as known frame
          }
          if ((len == 15) && (frame[5] == 0xE6) && (frame[6] == 0x85))    //Reply to CommandReadRAMVar (automatically polled 10 times per second, no re-send), len==15 with 3 values
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
            multiplusPinverterFiltered = (frame[12]<<8) + frame[11];
            ptr_pInverterRingBuf = (ptr_pInverterRingBuf + 1) & (SIZE_P_INVERTER_RINGBUF-1);         //increase and wrap pointer first
            pInverterRingBuf[ptr_pInverterRingBuf] = multiplusPinverterFiltered;   //now write newest value into ringbuffer
            //multiplusPoutputFiltered = (frame[14]<<8) + frame[13];      //note that multiplusPoutputFiltered is negated compared to multiplusPmainsFiltered
            //addToLogfile("\n"+String(multiplusPmainsFiltered)+" "+String(multiplusPinverterFiltered)+" "+String(multiplusPoutputFiltered)); //uncomment to log every value
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
                addToLogfile("\n-> Switched Multiplus to Charger+Inverter mode",3);
                veCmdSendState = 0;                             //sending mode=Charger+Inverter mode is done
              }
              if ((veCmdSendState==0x15) && ((masterMultiLED_SwitchRegister&0x03)==0x01)) {
                addToLogfile("\n-> Switched Multiplus to Charger-Only mode",3);
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
    addToLogfile(" FAILcmd"+String(veCmdSendState,HEX),3);
  }
}


// ===========================================================================
//                 Apply new ESS power into Multiplus
// ---------------------------------------------------------------------------
// This function is a comfortable way of sending a new ESS power value into
// the Multiplus. The ESS power value is the desired power on ACin.
// Negative ESS power means the Multiplus will consume x Watt from ACin.
// Positive ESS power means the Multiplus will feed/deliver x Watt into ACin.
// Note:
// The ESS power is only a "target"-value. The actual power on ACin will not
// change immediately. In my code, the actual power on ACin is available in
// these two variables:
// multiplusPmainsFiltered    latest power on ACin (requested 10x per second)
// powerACin                  power on ACin synchronized with decisiveMeterPower
// ===========================================================================
bool applyNewEssPower(int16_t power)
{
  if (veCmdSendState == 0) {  //Last check that we're good to send new value
    essPowerDesired = power;
    veCmdSendState = 2;            //force sending ESS command
    return true;
  }
  else return false;
}



// ===========================================================================
//                React on new electric power meter value
// ---------------------------------------------------------------------------
// This function adds the measured power from the meter including a relative
// time stamp to the logfile in a nicely formatted way, so that all power
// values are below each other. Additionally it does the following:
// - write the Time into the logfile every minute
// - calculate powerACin & powerInverter to be in sync with meter power
// - calculate the power which was drained from the battery into a Shelly
//   device since last Shelly switch change.
// - also write ACin-power ESS-power and battery/inverter-power
//   into the same logfile line
// - do everything that depends on the current time, like
//   - hourly consumption / feed-in
//   - reset min/max values at 5am
// ===========================================================================
void onNewMeterValue()
{
  //If one minute is over, add current time + SOC + bat. voltage into logfile
  if (oneMinuteOver) {
    addToLogfile("\n\n"+String(battery.soc)+"% "+String(switchMode)+String(essPowerStrategy),1);
    addToLogfile("\n"+String(multiplusDcVoltage, 2)+"V",1);
    addToLogfile("\nC/D "+String(int(round(battery.chargeCurrentLimit)))+"/"+String(int(round(battery.dischargeCurrentLimit))),1);
    addToLogfile("\nW1/W2/R "+String(battery.warningFlags1,HEX)+"/"+String(battery.warningFlags2,HEX)+"/"+String(battery.requestFlags,HEX),1);
    addToLogfile("\n"+String(powerTrendConsumption, 1)+"/"+String(powerTrendFeedIn, 1)+"Wh",1);
    addToLogfile("\n"+secondsToTimeStr(electricMeter_Runtime+ELECTRIC_METER_TIME_OFFSET,false),1);
  }
  //If new power meter value came in, write it to logfile first:
  if (newMeterValue)  //It might take up to one minute or longer for a new meter value to come in!
  {
    //calculate Multiplus ACin power for the same time period
    double p = 0;
    p += pACinRingBuf[(ptr_pACinRingBuf -5 )&(SIZE_P_ACIN_RINGBUF-1)];
    p += pACinRingBuf[(ptr_pACinRingBuf -6 )&(SIZE_P_ACIN_RINGBUF-1)];
    p += pACinRingBuf[(ptr_pACinRingBuf -7 )&(SIZE_P_ACIN_RINGBUF-1)];
    p += pACinRingBuf[(ptr_pACinRingBuf -8 )&(SIZE_P_ACIN_RINGBUF-1)];
    p += pACinRingBuf[(ptr_pACinRingBuf -9 )&(SIZE_P_ACIN_RINGBUF-1)];
    p += pACinRingBuf[(ptr_pACinRingBuf -10)&(SIZE_P_ACIN_RINGBUF-1)];
    p += pACinRingBuf[(ptr_pACinRingBuf -11)&(SIZE_P_ACIN_RINGBUF-1)];
    powerACin = round(p / 7);         //pos = feed-into meter, neg = consume from meter
    //calculate Multiplus inverter power for the same time period
    p = 0;
    p += pInverterRingBuf[(ptr_pInverterRingBuf -5 )&(SIZE_P_INVERTER_RINGBUF-1)];
    p += pInverterRingBuf[(ptr_pInverterRingBuf -6 )&(SIZE_P_INVERTER_RINGBUF-1)];
    p += pInverterRingBuf[(ptr_pInverterRingBuf -7 )&(SIZE_P_INVERTER_RINGBUF-1)];
    p += pInverterRingBuf[(ptr_pInverterRingBuf -8 )&(SIZE_P_INVERTER_RINGBUF-1)];
    p += pInverterRingBuf[(ptr_pInverterRingBuf -9 )&(SIZE_P_INVERTER_RINGBUF-1)];
    p += pInverterRingBuf[(ptr_pInverterRingBuf -10)&(SIZE_P_INVERTER_RINGBUF-1)];
    p += pInverterRingBuf[(ptr_pInverterRingBuf -11)&(SIZE_P_INVERTER_RINGBUF-1)];
    powerInverter = round (p / 7);    //pos = battery is charged, neg = battery is discharged/drained
    //Calculate the power to compensate from summed meter power plus Multiplus feed-in-power on ACin
    estTargetPower = round(powerACin + POWER_CAL*decisiveMeterPower -gridSetpoint);
    //Write into logfile:
    if (decisiveMeterPower < 0) addToLogfile("\n"+String(decisiveMeterPower),1);    //pos = consumption from grid,  neg = feed into grid
     else addToLogfile("\n+"+String(decisiveMeterPower),1);
    if (powerACin < 0) addToLogfile(String(powerACin),1);                           //pos = feed into meter,        neg = consume from meter
     else addToLogfile("+"+String(powerACin),1);
    if (estTargetPower < 0) addToLogfile(String(estTargetPower),1);                 //pos = feed into meter,        neg = consume from meter
     else addToLogfile("+"+String(estTargetPower),1);
    //If we can update the ESS power immediately, we do that:
    if      (essPowerStrategy == 5) essPowerTmp = estTargetPower;                       //Immediate strategy
    else if (essPowerStrategy == 7) essPowerTmp = max(estTargetPower, essPowerTmp);     //Maximum strategy
    else if (essPowerStrategy == 3) essPowerTmp = min(estTargetPower, essPowerTmp);     //Minimum strategy
  }
  //Using the values generated on each new meter power, now do every second:
  if (oneSecondOver)
  {
    //write latest value into ringbuffer
    estTargetPowerRingBuf[ptrPowerRingBuf] = estTargetPower;
    ptrPowerRingBuf++;
    if (ptrPowerRingBuf >= SIZE_PWR_RINGBUF) ptrPowerRingBuf = 0;  //wrap pointer if needed
    //Now evaluate all values in the ringbuffer based on the update strategy -> Find max and min value of current ringbuffer
    int16_t min = +30000;
    int16_t max = -30000;
    //addToLogfile("\n");   //DEBUG print current Ringbuffer into Logfile
    for (int i=0; i<SIZE_PWR_RINGBUF; i++) {
      int16_t pwr = estTargetPowerRingBuf[i];
      if (pwr < min) min = pwr;
      if (pwr > max) max = pwr;
      //addToLogfile(" "+String(pwr));   //DEBUG print current Ringbuffer into Logfile
    }
    //Every second, update depending on strategy
    if      (essPowerStrategy == 7) essPowerTmp = max;            //Maximum strategy
    else if (essPowerStrategy == 3) essPowerTmp = min;            //Minimum strategy
    else if (essPowerStrategy == 0) essPowerTmp = 0;              //0W strategy (compensate ACout1 only)
    //collect charger/inverter AC power into ringbuffer
    powerChargerRingBuf[ptr_powerChargerRingBuf] = multiplusPinverterFiltered;
    ptr_powerChargerRingBuf++;
    if (ptr_powerChargerRingBuf >= SIZE_SHELLY_POWER_RINGBUFFERS) ptr_powerChargerRingBuf = 0;  //wrap pointer if needed
    //calculate average charger/inverter AC power over complete ringbuffer
    averageChargingPower = 0;
    for (int i=0; i<SIZE_SHELLY_POWER_RINGBUFFERS; i++) {
      averageChargingPower += powerChargerRingBuf[i];    //equally average charger+inverter power
    }
    averageChargingPower = averageChargingPower / SIZE_SHELLY_POWER_RINGBUFFERS;   //integer division is precise enough
    //collect difference between Pmains(ACin) and desired ESS power into ringbuffer
    powerControlDeviationRingBuf[ptr_powerControlDeviationRingBuf] = multiplusPmainsFiltered - multiplusESSpower;
    ptr_powerControlDeviationRingBuf++;
    if (ptr_powerControlDeviationRingBuf >= SIZE_SHELLY_POWER_RINGBUFFERS) ptr_powerControlDeviationRingBuf = 0;  //wrap pointer if needed
    //calculate average control deviation power over complete ringbuffer
    averageControlDeviationFeedIn = 0;
    for (int i=0; i<SIZE_SHELLY_POWER_RINGBUFFERS; i++) {
      if (powerControlDeviationRingBuf[i]>0) averageControlDeviationFeedIn += powerControlDeviationRingBuf[i]; //only accumulate positive (feed-in) deviations
    }
    averageControlDeviationFeedIn = averageControlDeviationFeedIn / SIZE_SHELLY_POWER_RINGBUFFERS;   //integer division is precise enough
    //Ring buffer of power meter values, to obtain minimum feed-in over last Shelly switching period
    powerMeterRingBuf[ptr_powerMeterRingBuf] = decisiveMeterPower;
    ptr_powerMeterRingBuf++;
    if (ptr_powerMeterRingBuf >= SIZE_SHELLY_POWER_RINGBUFFERS) ptr_powerMeterRingBuf = 0;  //wrap pointer if needed
    //obtain minimum feed-in over complete ringbuffer
    minimumFeedIn = 9999999;  //initialize with unreachable high power
    for (int i=0; i<SIZE_SHELLY_POWER_RINGBUFFERS; i++) {
      if ((-powerMeterRingBuf[i]) < minimumFeedIn) minimumFeedIn = (-powerMeterRingBuf[i]); //search minimum of inverted power meter values (feed-in)
    }
    //Store latest meter power into ringbuffer for automatic mode switching (15min powerTrend calculation)
    //we use a seperate ringbuffer for this, as powerTrend ringbuffer needs to be cleared after every mode switch
    powerTrendRingBuf[ptr_powerTrendRingBuf] = decisiveMeterPower;
    ptr_powerTrendRingBuf++;
    if (ptr_powerTrendRingBuf >= SIZE_POWER_TREND_RINGBUF) ptr_powerTrendRingBuf = 0;  //wrap pointer if needed
    //Calculate consumption trend by summing over the complete ringbuffer
    int32_t cons = 0;
    int32_t feed = 0;
    for (int i=0; i<SIZE_POWER_TREND_RINGBUF; i++) {
      if (powerTrendRingBuf[i] > 0) cons += powerTrendRingBuf[i];  //only include power consumed from grid
      if (powerTrendRingBuf[i] < 0) feed -= powerTrendRingBuf[i];  //only include (negative) power fed into grid
    }
    powerTrendConsumption = cons / (60.0*60.0);
    powerTrendFeedIn      = feed / (60.0*60.0);
  }
  //In case ESS power would be negative, check if charging from ACin is allowed
  if (!chargeFromACin && (essPowerTmp < 0)) essPowerTmp = 0;
  //Estimate expected battery charge power with current essPowerTmp
  int chargePower = multiplusPmainsFiltered + multiplusPinverterFiltered - essPowerTmp;           //Note: Calculating with battery.power instead of multiplusPinverterFiltered is unstable.
  //Constant charge mode
  if (switchMode=='C') {
    if (chargePower < CONSTANT_CHARGE_POWER) {
      essPowerTmp2 = multiplusPmainsFiltered + multiplusPinverterFiltered - CONSTANT_CHARGE_POWER;   //Charge at least with CONSTANT_CHARGE_POWER (minus Multiplus standby power) or more if PV is available
    }
  }
  else if (chargeOnly && (chargePower < TRICKLE_CHARGE_POWER)) { //Normal mode during charge-only due to low battery
    essPowerTmp2 = multiplusPmainsFiltered + multiplusPinverterFiltered - TRICKLE_CHARGE_POWER;   //Charge at least with low compensation power (minus Multiplus standby power) or more if PV is available
  }
  else essPowerTmp2 = essPowerTmp;  //Normal compensation; Or we are charging >TRICKLE_CHARGE_POWER anyway
  //Check if ESS power value needs to be updated or was not re-written into Multiplus for more than 10 seconds (avoid timeout)
  if ((veCmdSendState==0) && ((essPowerTmp2 != essPowerDesired) || (essTimeoutCounter <= 0))) applyNewEssPower(essPowerTmp2);
}


bool switchShelly(String shellyIP, bool turnOn)
{
  String command = "http://" + shellyIP + "/relay/0?turn=";
  if (turnOn) command += "on"; else command += "off";
  HTTPClient http;
  http.setConnectTimeout(600);        //in ms, influences if command reaches Shelly at all (must be at least 400ms)
  http.setTimeout(200);               //in ms, influences if we receive successful response from Shelly (must be at least 150ms)
  http.begin(command.c_str());
  int httpResponseCode = http.GET();  //HTTP response code != 200 means error
  http.end();
  if (minuteTimer < 10) addToLogfile("\n "+String(minuteTimer),1); else addToLogfile("\n"+String(minuteTimer),1);
  if (httpResponseCode == 200) {
    if  (turnOn) addToLogfile(" "+shellyIP+"=on",1); else addToLogfile(" "+shellyIP+"=off",1);
    return true;
  } else {
    if  (turnOn) addToLogfile(" "+shellyIP+"=FAIL(on)",1); else addToLogfile(" "+shellyIP+"=FAIL(off)",1);
    return false;
  }
}


void controlConsumerShellys()
{
  if ((shellyWaitCnt <= 0) && (WiFi.status() == WL_CONNECTED)) {   //Only try switching if waiting time is over and if WIFI is available
    //Already set new default waiting time
    shellyWaitCnt = CONSUMER_SHELLY_SWITCHING_INTERVAL*CPS;  //Set the minimum waiting time
    //write average current into logfile
    addToLogfile("\nChg"+String(averageChargingPower)+" avgFd"+String(averageControlDeviationFeedIn)+" minFd"+String(minimumFeedIn),1);
    if ((chargeOnly) || (multiplusEmergencyPowerStatus==0x02)) {   //If multiplus is in charge-only mode due to low battery, in emergency power mode:
      shellyState = 0;     //fix to: all PV ON, all consumers OFF
    }
    else {
      //check if shellyState should be increased (one more consumer Shelly ON)
      if (minimumFeedIn > POWER_TH_SHELLY_PLUS) shellyState++;
      //check if shellyState should be decreased (one more Consumer Shelly OFF)
      else if (minimumFeedIn < POWER_TH_SHELLY_MINUS) shellyState--;
    }
    //saturate state at min and max
    if (shellyState < 0) shellyState = 0;
    if (shellyState > N_CONSUMER_SHELLYS) shellyState = N_CONSUMER_SHELLYS;
    //write latest Shelly-state into logfile
    addToLogfile(" Shelly:"+String(shellyState),1);
    //switch consumer Shellys
    int8_t  shellyStateConsumerTemp[N_CONSUMER_SHELLYS];
    for (int i=0; i<N_CONSUMER_SHELLYS; i++) {      //For each consumer Shelly do:
      bool turnOn = (i < shellyState);  //state = 0 = all consumers OFF ; 1 = 1st consumer ON, rest OFF ; 2 = 1st+2nd consumer ON, rest OFF ...
      if (turnOn) shellyStateConsumerTemp[i] = 1; else shellyStateConsumerTemp[i] = -1;
    }
    //Additionally apply consumer rules defined in CONSUMER_RULES[]:
    if (timeIsValid) {
      int h = 0;
      int m = 0;
      int s = 0;
      secondsToTime(h, m, s, electricMeter_Runtime+ELECTRIC_METER_TIME_OFFSET);
      //Go through all of the rules and only turn ON if rule is met
      nrShellysEnabledByRule = 0;
      if (switchMode == 'A') {     //Only apply rules in full automatic mode, don't apply in 'S' or other modes
        for(auto rule: CONSUMER_RULES) {   //Check all the rules
          if ( ((h > rule.hourOn) || ((h == rule.hourOn) && (m >= rule.minOn))) &&
              ((h < rule.hourOff) || ((h == rule.hourOff) && (m <= rule.minOff))) &&
              (battery.soc >= rule.batteryThr + socInverterOnOff) &&                //rule threshold goes on-top of adaptive inverter-On/Off-threshold
              ((rule.startedChg==false) || (rule.startedChg & batteryStartedCharging)) )
          {
            shellyStateConsumerTemp[rule.shellyNr] = 1;   //Conditions met, the Shelly belonging to this rule can be turned on
            nrShellysEnabledByRule++;
          }
        }
      }
    }
    //Try to apply consumer Shelly states in case they changed
    for(int shelly=0; shelly<N_CONSUMER_SHELLYS; shelly++) {
      if (shellyStateConsumerTemp[shelly] != shellyConsumer[shelly]) {   //if Shelly state is different:
        if (shellyStateConsumerTemp[shelly] == 1) {
          shellyActuations++;
          if (switchShelly(CONSUMER_IP[shelly], true)) shellyConsumer[shelly] = 1; else shellyFails++;   //only mark Shelly as turned on when we got successful response
        }
        else if (shellyStateConsumerTemp[shelly] == -1) {
          shellyActuations++;
          if (switchShelly(CONSUMER_IP[shelly], false)) shellyConsumer[shelly] = -1; else shellyFails++; //only mark Shelly as turned off when we got successful response
        }
      }
    }
  }
}



//true if frame was known, false if unknown
bool decodeBatteryCanMessage(twai_message_t message)
{
  bool result = false;
  int16_t temp;
  //interprete message
  switch (message.identifier) {
    //0x70, 0x371 or 0x373 would deliver detailed information, but for unknown reason is not received from my Pylontech US5000 batteries
    case 0x35C: //requests
    {
      if (message.data_length_code == 2) {
        battery.requestFlags = message.data[0];
        result = true;
      }
      break;
    }
    case 0x359: //Protection & warning flags
    {
      if (message.data_length_code == 7) {
        battery.protectionFlags1 = message.data[0];
        battery.protectionFlags2 = message.data[1];
        battery.warningFlags1 = message.data[2];
        battery.warningFlags2 = message.data[3];
        battery.nrPacksInParallel = message.data[4];
        result = true;
      }
      break;
    }
    case 0x35E: //Manufacturer string
    {
      for (int i = 0; i < message.data_length_code; i++) battery.manufacturer[i] = message.data[i];
      battery.manufacturer[message.data_length_code] = 0;    //terminate string correctly with zero
      result = true;
      break;
    }
    case 0x356: //current voltage, current & temperature
    {
      if (message.data_length_code == 6) {
        temp = (message.data[1]<<8) + message.data[0];
        battery.voltage = temp*0.01;
        temp = (message.data[3]<<8) + message.data[2];
        battery.current = temp*0.1;
        battery.power = round(battery.voltage * battery.current);
        if (battery.current < batteryCurrentMin) batteryCurrentMin = battery.current;
        if (battery.current > batteryCurrentMax) batteryCurrentMax = battery.current;
        if (battery.power < batteryPowerMin) batteryPowerMin = battery.power;
        if (battery.power > batteryPowerMax) batteryPowerMax = battery.power;
        bmsPowerAverage = (1-alpha)*bmsPowerAverage + alpha*battery.power;
        temp = (message.data[5]<<8) + message.data[4];
        battery.temperature = temp*0.1;
        if (battery.temperature < batteryTempMin) batteryTempMin = battery.temperature;
        if (battery.temperature > batteryTempMax) batteryTempMax = battery.temperature;
        //update DC voltage calibration offset whenever DC current is < 1.5A
        if ((abs(battery.current)<1.5) && (abs(multiplusDcCurrent)<1.5)) {
          multiplusDcVoltageCalibration = (1-beta)*multiplusDcVoltageCalibration + beta*(battery.voltage-multiplusDcVoltage);
          multiplusDcVoltageCalibrationCnt++;
        }
        //calculate battery cable resistances whenever current is > 20A
        if ((battery.current > 20) && (multiplusDcCurrent > 20)) {
          double r = 1000 * ((multiplusDcVoltage+multiplusDcVoltageCalibration) - battery.voltage) / battery.current;   //in mOhm
          chargeCableResistance = (1-beta)*chargeCableResistance + beta*r;
          chargeCableResistanceCnt++;
        }
        else if ((battery.current < -20) && (multiplusDcCurrent < -20)) {
          double r = 1000 * ((multiplusDcVoltage+multiplusDcVoltageCalibration) - battery.voltage) / battery.current;   //in mOhm
          dischargeCableResistance = (1-beta)*dischargeCableResistance + beta*r;
          dischargeCableResistanceCnt++;
        }
        result = true;
      }
      break;
    }
    case 0x351: //maximum voltages & currents
    {
      if (message.data_length_code == 8) {
        temp = (message.data[1]<<8) + message.data[0];
        battery.chargeVoltage = temp*0.1;
        temp = (message.data[3]<<8) + message.data[2];
        battery.chargeCurrentLimit = temp*0.1;
        temp = (message.data[5]<<8) + message.data[4];
        battery.dischargeCurrentLimit = temp*0.1;
        temp = (message.data[7]<<8) + message.data[6];
        battery.dischargeVoltage = temp*0.1;
        result = true;
      }
      break;
    }
    case 0x355: //SOC + SOH
    {
      if (message.data_length_code == 4) {
        battery.soc = (message.data[1]<<8) + message.data[0];         //State Of Charge
        if (battery.soc < battery.socMin) {
          battery.socMin = battery.soc;
          battery.socMinTime = electricMeter_Runtime;
        }
        if (battery.soc >= battery.socMin + BATTERY_STARTED_CHARING_OFFSET) batteryStartedCharging = true;
        if ((battery.soc >= 95) && (battery.soc <= 100)) batteryStartedCharging = true;                     //If battery reached 95%, also set batteryStartedCharging flag, to avoid that (socMin + BATTERY_STARTED_CHARING_OFFSET) is unreachable
        if (battery.soc > battery.socMax) {
          battery.socMax = battery.soc;
          battery.socMaxTime = electricMeter_Runtime;
        }
        socWatchdog = SOC_WATCHDOG_CYCLES;  //reset watchdog timer
        battery.soh =  (message.data[3]<<8) + message.data[2];        //State Of Health
        result = true;
      }
      break;
    }
  }
  return result;
}


// void sendCanMessage()
// {
//   twai_message_t message;
//   message.identifier = 0x305;
//   message.extd = 0;
//   message.data_length_code = 8;
//   for (int i = 0; i < message.data_length_code; i++) message.data[i] = 0;
//   //Queue message for transmission
//   if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
//     addToLogfile("\nMessage queued for transmission");
//   } else {
//     addToLogfile("\nFailed to queue message for transmission");
//   }
// }

// ===========================================================================
//                      CAN bus battery message handling
// ---------------------------------------------------------------------------
// This function reads the CAN (TWAI) bus messages from the battery. There is
// no need to send any specific message twards the battery. The battery status
// messages are automatically sent-out as long as the CAN bus transceiver is
// automatically acknowleding the messages. (Otherwise the battery will just
// continiously re-send a useless status message until it gets acknowledged.)
// On the Pylontech battery, messages from the BMS come about every second.
// ===========================================================================
void batteryHandling()
{
  //Check if CAN bus message from battery is received containing SOC (state of charge) value
  if (TWAI_ALERT_RX_DATA) {
    // One or more messages received. Handle all.
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
      if (!decodeBatteryCanMessage(message)) {
        //If message is unknown, write it to logfile
        addToLogfile("\n0x"+String(message.identifier,HEX),3);
        for (int i = 0; i < message.data_length_code; i++) addToLogfile(" "+String(message.data[i],HEX),3);
      }
    }
  }
  //Check if there was no SOC received from battery anymore and watchdog timer is expired
  if ((battery.soc >= 0) && (socWatchdog <= 0))
  {
    battery.soc = -1;                 //set SOC negative, to indicate that it is invalid
    addToLogfile("\n"+String(minuteTimer)+" Battery CAN bus FAIL",3);
  }
  //Calculate pulse rate for external Shelly 1PM from battery SOC
  shelly1PMpulsewidth = round(1.749*CPS/battery.soc);             //17490 = 1%/W with CPS=10000Hz
  if (shelly1PMpulsewidth < 173) shelly1PMpulsewidth = 173;       //limit to max. 101 %/W
  if (shelly1PMpulsewidth > 10*CPS) shelly1PMpulsewidth = 10*CPS; //limit to min. 0.17 %/W = 10 seconds max pulse width
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
  if (isrNewMeterPulse)  //if we got new power meter impulse from ISR
  {
    isrNewMeterPulse = false;  //reset flag
    if ((highCyclesPrevious >= 18) && (highCyclesPrevious <= 21)
    && (totalCycles >= 360) && (totalCycles <= 3600000)
    && (highCycles >= 18) && (highCycles <= 21)) //if within senseful range
    {
      if ((electricMeterSignPositive+electricMeterSignNegative) >= 1)   //AND if we have at least one sign (comes every second) from digital meter output
      {
        impulseMeterPower = 3600000 / totalCycles;   //calculate power in Watt from measured cycles
        //Evaluate the sign information from Info-DSS SML data
        if (electricMeterSignPositive >= (2*electricMeterSignNegative)) electricMeterCurrentSign = +1;      //if at least double as many positive signs than negative, then assume positive
        else if (electricMeterSignNegative >= (2*electricMeterSignPositive)) electricMeterCurrentSign = -1; //if at least double as many negative signs than positive, then assume negative
        else electricMeterCurrentSign = 0;            //if sign is ambiguous/fluctuating we better assume 0W power (which makes sense on average)
        //clear counters to start counting signs again
        electricMeterSignPositive = 0;
        electricMeterSignNegative = 0;
        //include the sign into power value
        impulseMeterPower = electricMeterCurrentSign * impulseMeterPower;
        newImpulseMeterPower = true;  //indicate that we got a new impulse meter value
      }
    }
  }
}

void decideElectricMeterValueToUse()
{
  newMeterValue = false;   //when we execute this function, the previous meter value has already been processed and is not valid anymode
  if (newDigitalMeterPower)  //if we got a new digital meter power with sign, we should use that
  {
    decisiveMeterPower = round(electricMeter_Power);
    //reset sub-flags
    newDigitalMeterPower = false;
    newImpulseMeterPower = false;       //To avoid that subsequent impulse meter value triggers new meter power again
    infoDssCntSinceLastMeterPower = 0;  //reset Info-DSS message counter since last meter power
    //set main flag
    newMeterValue = true;               //indicate that we got a new decisive meter power
  }
  else if (newImpulseMeterPower && (infoDssCntSinceLastMeterPower > 0))  //if we did not get a digital meter power, but an impulse-based one, use that one instead
  {
    decisiveMeterPower = impulseMeterPower;
    //reset sub-flags
    newImpulseMeterPower = false;
    infoDssCntSinceLastMeterPower = 0;  //reset Info-DSS message counter since last meter power
    //set main flag
    newMeterValue = true;               //indicate that we got a new decisive meter power
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
      if (smlCRC == calculatedCRC) {    //If CRC is matching
        infoDssCntSinceLastMeterPower++;  //increase counter
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
          int nrStatusBytes = readSMLentry(sBuf2, smlLength, pos, 1, 1, electricMeter_Status180, MAX_METER_STATUS_BYTES);
          //Move every character to right (if needed)
          for (int i=(MAX_METER_STATUS_BYTES-1); i>=(MAX_METER_STATUS_BYTES-nrStatusBytes); i--) {
            electricMeter_Status180[i] = electricMeter_Status180[i-(MAX_METER_STATUS_BYTES-nrStatusBytes)];
          }
          //Fill unused status characters on the left with zeros (if needed)
          for (int i=0; i<(MAX_METER_STATUS_BYTES-nrStatusBytes); i++) electricMeter_Status180[i] = 0;
          //Count power signs for combination with impulse power measurement (0x8x = consumption; 0xAx = feed-in)
          if ((electricMeter_Status180[MAX_METER_STATUS_BYTES-1] & 0xF0) == 0x80) electricMeterSignPositive++;
          else if ((electricMeter_Status180[MAX_METER_STATUS_BYTES-1] & 0xF0) == 0xA0) electricMeterSignNegative++;
          //Continue decoding with consumption counter
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
            timeIsValid = true;   //indicate that we got a valid time from power meter and time-specific operations can start
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
              //if ((electricMeter_Status180[MAX_METER_STATUS_BYTES-1] & 0xF0) == 0xA0) electricMeter_Power = -electricMeter_Power;  //not needed as my meter already gives a signed value
              //also update the integer values
              newDigitalMeterPower = true;  //indicate that we got a new digital meter power from Info-DSS output
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
      }
      else {  //If CRC was wrong:
        electricMeterCRCwrong++;
        addToLogfile("\nWrong SML CRC: "+String(calculatedCRC,HEX)+" ("+String(smlLength)+"byte)",1);
        //addFrameToLogfile(sBuf2,0,smlLength-1);
      }
    }
  }
}


void automaticSwitching()
{
  //Decide on the ESS power update strategy
  if ((multiplusEmergencyPowerStatus==0x02) || (switchMode=='Z')) {  //If we are off-grid or in 'Z' = Zero-switchMode:
    essPowerStrategy = 0;         //use 0W strategy (as there is nothing to compensate on AC-in)
    automaticMinMaxModeCnt = 0;   //exit Min/Max strategy mode in case we were in
    IRpinDisplay = HIGH;          //Turn red LED on to indicate abnormal situation
  }
  // else if (chargeOnly) {  //If we are in charge-only mode due to low battery:
  //   essPowerStrategy = 3;         //Use minimum strategy, because we don't want to waste energy for feeding-in. And if PV is dropping due to shadows, charging a few more seconds from grid does not harm.
  //   automaticMinMaxModeCnt = 0;   //exit Min/Max strategy mode in case we are in
  // }
  else
  { //we are on-grid, then use automatic strategy:
    if ((automaticMinMaxModeCnt <= 0) && (essPowerStrategy != 5)) {  //if Max/Min duration is over, but we didn't switch back to immedate strategy yet:
      //reset power Trend and calculation buffer
      for (int i=0; i<SIZE_POWER_TREND_RINGBUF; i++) powerTrendRingBuf[i] = 0;  //clear Buffer
      powerTrendConsumption = 0;
      powerTrendFeedIn = 0;
      //switch back to immediate strategy
      essPowerStrategy = 5;
    }
    else if (automaticMinMaxModeCnt <= 0) {   //if we are currently in normal immediate compensation mode (NOT in Min/Max)
      if ((powerTrendConsumption >= POWER_TREND_TH) && (powerTrendFeedIn >= POWER_TREND_TH)) {   //if both 15min grid-consumption and feed-in are above threshold (poor compensation detected)
        automaticMinMaxModeCnt = AUTOMATIC_MAXIMUM_MODE_DURATION;   //set Timer; use different strategy for some time
        essPowerStrategy = 7;                                       //switch to default alternative strategy, which is Maximum
        addToLogfile("\nMin/Max mode start: cons="+String(powerTrendConsumption,1)+" feed="+String(powerTrendFeedIn,1),3);
      }
    }
  }
  //Check if Minimum compensation strategy should be selected instead of Maximum
  if (automaticMinMaxModeCnt > 0) {   //if we are in Min/Max strategy duration
    if (battery.soc < (socInverterOnOff+SOC_TH_STRATEGY_OFFSET)) {
      essPowerStrategy = 3;  //Minimum Strategy saves battery, but also results in the battery slightly charged from the grid during sunlight fluctuations
    }
    else essPowerStrategy = 7;  //use Maximum Strategy to minimize grid consumption
  }
  //Adaptive SOC inverter ON/OFF threshold. Check if battery max threshold reached 99..100%. If yes, battery is successfully balanced again and we can allow lowest discharge again:
  if ((battery.socMax == 99) || (battery.socMax == 100)) socInverterOnOff = SOC_ADAPT_LOWEST;
  //Check if battery gives Low_voltage (0x04) or BMS_internal (0x08) warnings or alarms, or if max. discharge current is below 150A
  if ((battery.soc>=0) && (!chargeOnly) && ((battery.warningFlags1 & 0x04) || (battery.protectionFlags1 & 0x04) || (battery.warningFlags2 & 0x08) || (battery.protectionFlags2 & 0x08) || (battery.dischargeCurrentLimit < 79))) {
    socInverterOnOff = battery.soc + 2; //SOC-limit has to be set to at least +1%, to stop discharging immediately, use +2% to avoid repeated warning after little recharge of battery
    addToLogfile("\nBatt warn1="+String(battery.warningFlags1,HEX)+" prot1="+String(battery.protectionFlags1,HEX)+" warn2="+String(battery.warningFlags2,HEX)+" prot2="+String(battery.protectionFlags2,HEX)+" Idchg="+String(battery.dischargeCurrentLimit,1)+"A",3);
    addToLogfile("\nBatt Ubms="+String(battery.voltage,2)+" Ibms="+String(battery.current,1)+" Ump="+String(multiplusDcVoltage,2)+" Imp="+String(multiplusDcCurrent,1),3);
    addToLogfile("\nBatt th="+String(socInverterOnOff)+"%",3);
  }
  //Allow battery discharging or not:
  if (multiplusEmergencyPowerStatus==0x02) {  //If we are off-grid (mains=off), force Charger+Inverter mode
    chargeOnly = false;
  }
  else {  //if we are not in any special force-mode:
    if ((battery.soc>=0) && (battery.soc < socInverterOnOff)) chargeOnly = true;   //switch to charger-only-mode
    if (battery.soc > socInverterOnOff)                       chargeOnly = false;  //switch to charger+inverter mode
  }
  //Do all the time-based stuff
  //This is using the time (seconds) from electric meter. Note that this stops if there is no input from meter or on blackout.
  if (timeIsValid) {
    int h = 0;
    int m = 0;
    int s = 0;
    secondsToTime(h, m, s, electricMeter_Runtime+ELECTRIC_METER_TIME_OFFSET);
    //Check if some variables were not initialized yet
    if (timeNewHourDone < 0) timeNewHourDone = h;   //initialize with current hour after ESP32 reboot
    if (electricMeterConsumptionOneHourAgo<0) electricMeterConsumptionOneHourAgo = electricMeter_Consumption; //initialize after ESP32 reboot
    if (electricMeterFeedInOneHourAgo<0) electricMeterFeedInOneHourAgo = electricMeter_FeedIn; //initialize after ESP32 reboot
    //Only do once when a full hour is over and minute = 0:
    if ((m==0) && (h != timeNewHourDone) && (h>=0) && (h<=23)) {
      timeNewHourDone = h;    //save new hour to only execute following code once
      //Generate the hourly consumption/feed-in overview
      electricMeterHourlyConsumption[h] = electricMeter_Consumption - electricMeterConsumptionOneHourAgo;
      electricMeterConsumptionOneHourAgo = electricMeter_Consumption;
      electricMeterHourlyFeedIn[h] = electricMeter_FeedIn - electricMeterFeedInOneHourAgo;
      electricMeterFeedInOneHourAgo = electricMeter_FeedIn;
      //Update Wh sums over last 24h
      electricMeter24hConsumption = 0;
      electricMeter24hFeedIn = 0;
      for (uint16_t i=0; i<24; i++)
      {
        electricMeter24hConsumption += electricMeterHourlyConsumption[i];
        electricMeter24hFeedIn += electricMeterHourlyFeedIn[i];
      }      
      //Check if time is exactly 4:00am. If yes, reset min/max values
      if (h==4) {
        battery.socMin = battery.soc;
        battery.socMax = battery.soc;
        battery.socMinTime = electricMeter_Runtime;
        battery.socMaxTime = electricMeter_Runtime;
        dcCurrentMin = multiplusDcCurrent;
        dcCurrentMax = multiplusDcCurrent;
        acVoltageMin = multiplusUMainsRMS;
        timeAcVoltageMin = electricMeter_Runtime;
        acVoltageMax = multiplusUMainsRMS;
        timeAcVoltageMax = electricMeter_Runtime;
        acFrequencyMin = multiplusAcFrequency;
        timeAcFrequencyMin = electricMeter_Runtime;
        acFrequencyMax = multiplusAcFrequency;
        timeAcFrequencyMax = electricMeter_Runtime;
        dcVoltageMin = multiplusDcVoltage;
        dcVoltageMax = multiplusDcVoltage;
        multiplusTempMin = multiplusTemp;
        multiplusTempMax = multiplusTemp;
        secondsInMinStrategy = 0;
        secondsInMaxStrategy = 0;
        batteryTempMin = battery.temperature;
        batteryTempMax = battery.temperature;
        batteryCurrentMin = battery.current;
        batteryCurrentMax = battery.current;
        batteryPowerMin = battery.power;
        batteryPowerMax = battery.power;
        shellyActuations = 0;
        shellyFails = 0;
        batteryStartedCharging = false;
        addToLogfile("\n",3);
      }
      //At exactly 13:00, increase adaptive battery-low threshold
      if (h==13) {
        socInverterOnOff += SOC_ADAPT_STEP;         //increase inverter-ON/OFF threshold
        if (socInverterOnOff > SOC_ADAPT_HIGHEST) socInverterOnOff = SOC_ADAPT_HIGHEST; //don't increase inverter-ON/OFF threshold higher than SOC_ADAPT_HIGHEST
      }
    }
  }
  //Count seconds we were in Min or Max ESS power update strategy
  if ((oneSecondOver) && (essPowerStrategy==3)) secondsInMinStrategy++;
  if ((oneSecondOver) && (essPowerStrategy==7)) secondsInMaxStrategy++;
}



// ===========================================================================
//              Switch between modes and strategies using Boot-Button
// ---------------------------------------------------------------------------
// Long push and Short push are supported.
// ===========================================================================
void switchSpecialModes()
{
  //React on button-press
  if (buttonPressCnt >= BUTTON_LONGPRESS_TIME) {
    //LONG PUSH detected, code follows here:
    if (switchMode == 'Z') {
      switchMode = 'C';                                     //Constant-charge-mode, charges battery with fixed power and float voltage
      socInverterOnOff = SOC_ADAPT_LOWEST;                  //If we come from 'Z' mode, also reset SOC-low-threshold to lowest
    }
    else if (switchMode == 'C') switchMode = 'Z';           //Zero: Same as Save mode, but always use ESSpower = 0W (no charging/compensating power before Multiplus). This mode is required if no meter is available.
    else if (switchMode != 'C') switchMode = 'C';           //Constant-charge-mode, charges battery with fixed power and float voltage
    addToLogfile("\n->UserMode="+String(switchMode),3);
    //end of LONG PUSH code
    buttonPressCnt = -2;                    //reset & lock counter. Don't start it yet if button is not released AND then pressed again
  }
  bool bootButton = digitalRead(BUTTON_GPIO);
  if (bootButton == HIGH) {   //If button is not pressed
    if (buttonPressCnt >= BUTTON_DEBOUNCE_TIME) {
      //SHORT PUSH detected, code follows here:
      if ((switchMode == 'S') || (switchMode == 'C') || (switchMode == 'Z')) switchMode = 'A'; //Automatic: High consumption during short time period switches ESS strategy to Max(7) on Min(3) automatically
      else if (switchMode == 'A') switchMode = 'S';                                            //Save: Same as Automatic mode, but don't turn-on Shellys based on rules (only turn-on if energy is left over)
      addToLogfile("\n->UserMode="+String(switchMode),3);
    }
    buttonPressCnt = -1;    //-1 = lock counter. Don't start it yet if button is not pressed.
  }
  else {    //If button is pressed
    if (buttonPressCnt == -1) buttonPressCnt=0;   //allow to start counter (but don't reset it!)
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
    //Note: after line 0 follows line 2
    //      after line 1 follows line 3
    //      after line 2 follows line 1
    //      after line 3 follows line 0
  // --- Line 1 ---
    //Clear last three characters of this line. As adding spaces at the end of the line might overwrite into next line.
    lcd.setCursor(17, 0);
    lcd.print("   ");
    //Time
    lcd.setCursor(0, 0);
    if (timeIsValid) lcd.print(secondsToTimeStr(electricMeter_Runtime+ELECTRIC_METER_TIME_OFFSET,true));
    else {
      lcd.print("--:--");
      lcd.print(battery.soh);
      lcd.print("% ");
    }
    //Wifi + PowerStrategy + SwitchMode
    lcd.setCursor(9, 0);
    if (WiFi.status() == WL_CONNECTED) lcd.print("W"); else lcd.print("-");
    lcd.print(essPowerStrategy);
    lcd.print(switchMode);        //print switchMode number as ASCII character
    //Multiplus Switch Register & Voltage Status
    lcd.setCursor(13, 0);
    lcd.print(masterMultiLED_SwitchRegister,HEX);
    lcd.print(multiplusVoltageStatus,HEX);
    lcd.print(" ");
    //Shelly state & amount of Shellys turned ON by CONSUMER_RULES[]
    lcd.setCursor(17, 0);
    if (shellyState >= 0) lcd.print(" ");
    lcd.print(shellyState);
    lcd.print(nrShellysEnabledByRule);
  // --- Line 2 ---
    //Clear last three characters of this line. As adding spaces at the end of the line might overwrite into next line.
    lcd.setCursor(17, 1);
    lcd.print("   ");
    //Battery charge level (SOC)
    lcd.setCursor(0, 1);
    if ((battery.soc >= 0) && (battery.soc <= 100)) lcd.print(battery.soc); else lcd.print("---");
    lcd.print("% ");
    //Min and Max SOC and current inverter ON/OFF threshold
    lcd.setCursor(4, 1);
    lcd.print(socInverterOnOff);
    lcd.print(char(0xA3));
    lcd.print(battery.socMin);
    lcd.print(char(0x7E));
    lcd.print(battery.socMax);
    lcd.print("  ");
    //Battery power
    lcd.setCursor(14, 1);
    if (battery.power==0) lcd.print(" ");
    else if (battery.power>0) lcd.print("+");
    lcd.print(battery.power);
    lcd.print("W");
  // --- Line 3 ---
    //Clear last three characters of this line. As adding spaces at the end of the line might overwrite into next line.
    lcd.setCursor(17, 2);
    lcd.print("   ");
    //Multiplus battery voltage
    lcd.setCursor(0, 2);
    lcd.print(multiplusDcVoltage, 2);
    lcd.print("V");
    // //Temperature, minimum of BMS or Multiplus
    // lcd.setCursor(7, 2);
    // float t = min(multiplusTemp,battery.temperature);
    // lcd.print(t,1);
    // lcd.print(char(0xDF));
    // lcd.print("C  ");
    //AC inverter power
    lcd.setCursor(7, 2);
    if (multiplusPinverterFiltered==0) lcd.print(" ");
    else if (multiplusPinverterFiltered>0) lcd.print("+");
    lcd.print(multiplusPinverterFiltered);
    lcd.print("W  ");    
    //ESS power (ACin output power) desired
    lcd.setCursor(14, 2);
    if (multiplusESSpower==0) lcd.print(" ");
    else if (multiplusESSpower>0) lcd.print("+");
    lcd.print(multiplusESSpower);
    if (abs(multiplusESSpower) <= 9999) lcd.print("W"); //Don't print "W" if value is more than 4 digits
  // --- Line 4 ---
    //Clear last three characters of this line. As adding spaces at the end of the line might overwrite into next line.
    lcd.setCursor(17, 3);
    lcd.print("   ");
    //Sum of consumption Wh of last 24h
    lcd.setCursor(0, 3);
    lcd.print(int(round(electricMeter24hConsumption*1000)));
    lcd.print("Wh   ");
    //Electric meter power
    lcd.setCursor(7, 3);
    if (decisiveMeterPower==0) lcd.print(" ");
    else if (decisiveMeterPower>0) lcd.print("+");
    lcd.print(decisiveMeterPower);
    lcd.print("W  ");
    //Sum of feed-in Wh of last 24h
    lcd.setCursor(14, 3);
    int p = int(round(electricMeter24hFeedIn*1000));
    lcd.print(p);
    if (p <= 9999) lcd.print("Wh"); //Don't print "Wh" if value is more than 4 digits
}

String meterDeviceIdToStr(char *deviceID)
{
  char str[20];
  sprintf(str,"%d %c%c%c%02X %08d",deviceID[1],deviceID[2],deviceID[3],deviceID[4],deviceID[5],(deviceID[6]<<24)+(deviceID[7]<<16)+(deviceID[8]<<8)+deviceID[9]);
  return str;
}

// ===========================================================================
//                      Updates main webpage in memory
// ---------------------------------------------------------------------------
// ===========================================================================
void updateMainWebsite() {
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
  webpage = "<!DOCTYPE html><html><head><title>ESP32 Multiplus ESS</title></head><body><font size=\"7\">";  //font size 7 is largest allowed
  webpage += "<p><b>ESP32 Multiplus ESS</b></p>";
  if ((battery.soc >= 0) && (battery.soc <= 100)) {   //if SOC wihin valid range, show it as number on webpage
    webpage += "Battery:    "+String(battery.soc)+"% (Th="+String(socInverterOnOff)+")<br>";
  } else {
    webpage += "Battery:    CAN bus connection failure! Multiplus disabled.<br>";
  }
  webpage += "Time: ";
  if (timeIsValid) webpage += secondsToTimeStr(electricMeter_Runtime+ELECTRIC_METER_TIME_OFFSET,true)+"<br>"; else webpage += "- still unknown -<br>";
  if (decisiveMeterPower >= 0) webpage += "PowerMeter: +"+String(decisiveMeterPower)+"W<br>";
  else webpage += "PowerMeter: "+String(decisiveMeterPower)+"W<br>";
  webpage += "ESS power:  "+String(multiplusESSpower)+"W<br>";
  webpage += "ESP32 Switch-Mode: '"+String(switchMode)+"'<br>";
  webpage += "ESS power update strategy: "+String(essPowerStrategy)+"<br>";
  webpage += String(int(round(SIZE_POWER_TREND_RINGBUF/60.0)))+"min cons. power trend: "+String(powerTrendConsumption, 1)+"Wh (Th="+String(POWER_TREND_TH)+")<br>";
  webpage += String(int(round(SIZE_POWER_TREND_RINGBUF/60.0)))+"min feed-in power trend: "+String(powerTrendFeedIn, 1)+"Wh (Th="+String(POWER_TREND_TH)+")<br>";
  webpage += "ESS min strategy time: "+secondsToTimeStr(secondsInMinStrategy,true)+"<br>";
  webpage += "ESS max strategy time: "+secondsToTimeStr(secondsInMaxStrategy,true)+"<br>";
  webpage += "Shelly state: "+String(shellyState)+"<br>";
  webpage += "Shelly actuations: "+String(shellyActuations)+"<br>";
  if (shellyActuations>0) webpage += "Avg. fails per actuation: "+String(float(shellyFails)/shellyActuations, 3)+"<br>";
  webpage += "Minimum feed-in: "+String(minimumFeedIn)+"W<br>";
  webpage += "Avg. ctrl deviation feed-in: "+String(averageControlDeviationFeedIn)+"W<br>";
  webpage += "Avg. AC charging power: "+String(averageChargingPower)+"W<br>";
  webpage += "Avg. BMS charging power: "+String(int(round(bmsPowerAverage)))+"W<br>";
  webpage += "Shellys enabled by consumer rules: "+String(nrShellysEnabledByRule)+"<br>";
  webpage += "<br>";
  webpage += "<table>";
  webpage += "<tr><th>charger</th><th>inverter</th></tr>";
  webpage += "<tr><td>&nbsp;&nbsp;&nbsp;"+strMainsOn+"</td><td>&nbsp;&nbsp;&nbsp;"+strInverterOn+"</td></tr>";
  webpage += "<tr><td>&nbsp;&nbsp;&nbsp;"+strBulk+"</td><td>&nbsp;&nbsp;&nbsp;"+strOverload+"</td></tr>";
  webpage += "<tr><td>&nbsp;&nbsp;&nbsp;"+strAbsorption+"</td><td>&nbsp;&nbsp;&nbsp;"+strLowBattery+"</td></tr>";
  webpage += "<tr><td>&nbsp;&nbsp;&nbsp;"+strFloat+"</td><td>&nbsp;&nbsp;&nbsp;"+strTemperature+"</td></tr>";
  webpage += "</table>";
  webpage += "<button onclick=\"window.location.href='/shortpress'\">Short Press</button>";
  webpage += "<button onclick=\"window.location.href='/longpress'\">Long Press</button>";
  webpage += "<br>";
  webpage += "<b>Multiplus</b><br>";
  webpage += "<table>";
  webpage += "<tr><td>DC voltage:</td><td>&nbsp;&nbsp;</td><td>"+String(multiplusDcVoltage, 2)+"V</td></tr>";
  webpage += "<tr><td>DC current:</td><td>&nbsp;&nbsp;</td><td>"+String(multiplusDcCurrent,1)+"A ("+String(int(round(multiplusDcCurrent*multiplusDcVoltage)))+"W)</td></tr>";
  webpage += "<tr><td>Charger/Inverter AC power:</td><td>&nbsp;&nbsp;</td><td>"+String(multiplusPinverterFiltered)+"W</td></tr>";
  webpage += "<tr><td>ACin power:</td><td>&nbsp;&nbsp;</td><td>"+String(multiplusPmainsFiltered)+"W</td></tr>";
  webpage += "<tr><td>ACin voltage:</td><td>&nbsp;&nbsp;</td><td>"+String(multiplusUMainsRMS, 2)+"V</td></tr>";
  webpage += "<tr><td>AC frequency:</td><td>&nbsp;&nbsp;</td><td>"+String(multiplusAcFrequency, 3)+"Hz</td></tr>";
  webpage += "<tr><td>Power factor:</td><td>&nbsp;&nbsp;</td><td>"+String(multiplusPowerFactor, 4)+"</td></tr>";
  webpage += "<tr><td>Capacity in-out:</td><td>&nbsp;&nbsp;</td><td>"+String(multiplusBatteryAh)+"Ah ("+String(multiplusBatteryAh*NOM_VOLT/1000.0,1)+"kWh)</td></tr>";
  webpage += "<tr><td>Temperature:</td><td>&nbsp;&nbsp;</td><td>"+String(multiplusTemp,1)+"&deg;C</td></tr>";
  webpage += "<tr><td>MasterMultiLED Status:</td><td>&nbsp;&nbsp;</td><td>"+String(masterMultiLED_Status)+"</td></tr>";
  webpage += "<tr><td>MasterMultiLED SwitchVal:</td><td>&nbsp;&nbsp;</td><td>0x"+String(masterMultiLED_SwitchRegister,HEX)+"</td></tr>";
  webpage += "<tr><td>Charger/Inverter Status:</td><td>&nbsp;&nbsp;</td><td>"+String(multiplusStatus80)+"</td></tr>";
  webpage += "<tr><td>Voltage status:</td><td>&nbsp;&nbsp;</td><td>0x"+String(multiplusVoltageStatus,HEX)+"</td></tr>";
  webpage += "<tr><td>Emergency power status:</td><td>&nbsp;&nbsp;</td><td>"+String(multiplusEmergencyPowerStatus)+"</td></tr>";
  webpage += "<tr><td>AC input current limit:</td><td>&nbsp;&nbsp;</td><td>"+String(masterMultiLED_ActualInputCurrentLimit,1)+"A</td></tr>";
  webpage += "<tr><td>AC input configuration:</td><td>&nbsp;&nbsp;</td><td>0x"+String(masterMultiLED_AcInputConfiguration,HEX)+"</td></tr>";
  webpage += "<tr><td>Battery frame byte 07:</td><td>&nbsp;&nbsp;</td><td>0x"+String(multiplusBattery_byte07,HEX)+"</td></tr>";
  webpage += "<tr><td>Battery frame byte 06:</td><td>&nbsp;&nbsp;</td><td>0x"+String(multiplusBattery_byte06,HEX)+"</td></tr>";
  webpage += "<tr><td>Battery frame byte 05:</td><td>&nbsp;&nbsp;</td><td>0x"+String(multiplusBattery_byte05,HEX)+"</td></tr>";
  webpage += "<tr><td>E4 frame byte 18:</td><td>&nbsp;&nbsp;</td><td>0x"+String(multiplusE4_byte18,HEX)+"</td></tr>";
  webpage += "<tr><td>E4 frame byte 17:</td><td>&nbsp;&nbsp;</td><td>0x"+String((multiplusE4_byte17 >> 4),HEX)+"</td></tr>";
  webpage += "<tr><td>E4 frame byte 12:</td><td>&nbsp;&nbsp;</td><td>0x"+String(multiplusE4_byte12,HEX)+"</td></tr>";
  webpage += "<tr><td>E4 frame byte 11:</td><td>&nbsp;&nbsp;</td><td>0x"+String(multiplusE4_byte11,HEX)+"</td></tr>";
  webpage += "<tr><td>VE.Bus TX frames failed:</td><td>&nbsp;&nbsp;</td><td>"+String(veTxCmdFailCnt)+"/"+String(veCmdCounter)+"</td></tr>";
  webpage += "<tr><td>VE.Bus RX checksum fail:</td><td>&nbsp;&nbsp;</td><td>"+String(veRxCmdFailCnt)+"</td></tr>";
  webpage += "</table>";
  webpage += "<br>";
  webpage += "<b>Electric meter (Info-DSS)</b><br>";
  webpage += "<table>";
  webpage += "<tr><td>ID:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+meterDeviceIdToStr(electricMeter_DeviceID)+"</td></tr>";
  webpage += "<tr><td>Consumption:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(electricMeter_Consumption)+" kWh</td></tr>";
  webpage += "<tr><td>Feed-in:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(electricMeter_FeedIn)+" kWh</td></tr>";
  webpage += "<tr><td>Status 1.8.0:</td><td>&nbsp;&nbsp;&nbsp;</td><td>";
  for (int i=0; i<MAX_METER_STATUS_BYTES; i++) webpage += String(electricMeter_Status180[i],HEX)+" ";
  webpage += "</td></tr>";
  webpage += "<tr><td>Runtime:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(electricMeter_Runtime)+" sec = "+String(electricMeter_Runtime/(60*60*24))+" days</td></tr>";
  webpage += "<tr><td>Power L1:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(electricMeter_PowerL1)+"W</td></tr>";
  webpage += "<tr><td>Power L2:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(electricMeter_PowerL2)+"W</td></tr>";
  webpage += "<tr><td>Power L3:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(electricMeter_PowerL3)+"W</td></tr>";
  webpage += "<tr><td>Power (balanced):</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(electricMeter_Power)+"W</td></tr>";
  webpage += "<tr><td>SML length:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(smlLength)+" bytes</td></tr>";
  webpage += "<tr><td>CRC failures:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(electricMeterCRCwrong)+"</td></tr>";
  webpage += "</table>";
  webpage += "<br>";
  webpage += "<b>BMS</b><br>";
  webpage += "<table>";
  webpage += "<tr><td>State of charge:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(battery.soc)+"%</td></tr>";
  webpage += "<tr><td>State of health:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(battery.soh)+"%</td></tr>";
  webpage += "<tr><td>Charge voltage max:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(battery.chargeVoltage,1)+"V</td></tr>";
  webpage += "<tr><td>Charge current limit:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(battery.chargeCurrentLimit,1)+"A</td></tr>";
  webpage += "<tr><td>Discharge current limit:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(battery.dischargeCurrentLimit,1)+"A</td></tr>";
  webpage += "<tr><td>Discharge voltage min:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(battery.dischargeVoltage,1)+"V</td></tr>";
  webpage += "<tr><td>Voltage:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(battery.voltage,2)+"V</td></tr>";
  webpage += "<tr><td>Current:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(battery.current,1)+"A</td></tr>";
  webpage += "<tr><td>Temperature:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(battery.temperature,1)+"&deg;C</td></tr>";
  webpage += "<tr><td>Manufacturer:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(battery.manufacturer)+"</td></tr>";
  webpage += "<tr><td>Nr of packs in parallel:</td><td>&nbsp;&nbsp;&nbsp;</td><td>"+String(battery.nrPacksInParallel)+"</td></tr>";
  webpage += "<tr><td>Protection flags 1:</td><td>&nbsp;&nbsp;&nbsp;</td><td>0x"+String(battery.protectionFlags1,HEX)+"</td></tr>";
  webpage += "<tr><td>Protection flags 2:</td><td>&nbsp;&nbsp;&nbsp;</td><td>0x"+String(battery.protectionFlags2,HEX)+"</td></tr>";
  webpage += "<tr><td>Warning flags 1:</td><td>&nbsp;&nbsp;&nbsp;</td><td>0x"+String(battery.warningFlags1,HEX)+"</td></tr>";
  webpage += "<tr><td>Warning flags 2:</td><td>&nbsp;&nbsp;&nbsp;</td><td>0x"+String(battery.warningFlags2,HEX)+"</td></tr>";
  webpage += "<tr><td>Request flags:</td><td>&nbsp;&nbsp;&nbsp;</td><td>0x"+String(battery.requestFlags,HEX)+"</td></tr>";
  webpage += "</table>";
  webpage += "<br>";
  webpage += "<table>";
  webpage += "<tr><th>Time period</th><th>&nbsp;&nbsp;&nbsp;Consumption</th><th>&nbsp;&nbsp;&nbsp;Feed-in (Wh)</th></tr>";
  webpage += "<tr><td>0:00..1:00</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyConsumption[1]*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyFeedIn[1]*1000)))+"</td></tr>";
  webpage += "<tr><td>1:00..2:00</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyConsumption[2]*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyFeedIn[2]*1000)))+"</td></tr>";
  webpage += "<tr><td>2:00..3:00</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyConsumption[3]*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyFeedIn[3]*1000)))+"</td></tr>";
  webpage += "<tr><td>3:00..4:00</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyConsumption[4]*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyFeedIn[4]*1000)))+"</td></tr>";
  webpage += "<tr><td>4:00..5:00</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyConsumption[5]*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyFeedIn[5]*1000)))+"</td></tr>";
  webpage += "<tr><td>5:00..6:00</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyConsumption[6]*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyFeedIn[6]*1000)))+"</td></tr>";
  webpage += "<tr><td>6:00..7:00</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyConsumption[7]*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyFeedIn[7]*1000)))+"</td></tr>";
  webpage += "<tr><td>7:00..8:00</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyConsumption[8]*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyFeedIn[8]*1000)))+"</td></tr>";
  webpage += "<tr><td>8:00..9:00</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyConsumption[9]*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyFeedIn[9]*1000)))+"</td></tr>";
  webpage += "<tr><td>9:00..10:00</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyConsumption[10]*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyFeedIn[10]*1000)))+"</td></tr>";
  webpage += "<tr><td>10:00..11:00</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyConsumption[11]*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyFeedIn[11]*1000)))+"</td></tr>";
  webpage += "<tr><td>11:00..12:00</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyConsumption[12]*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyFeedIn[12]*1000)))+"</td></tr>";
  webpage += "<tr><td>12:00..13:00</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyConsumption[13]*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyFeedIn[13]*1000)))+"</td></tr>";
  webpage += "<tr><td>13:00..14:00</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyConsumption[14]*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyFeedIn[14]*1000)))+"</td></tr>";
  webpage += "<tr><td>14:00..15:00</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyConsumption[15]*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyFeedIn[15]*1000)))+"</td></tr>";
  webpage += "<tr><td>15:00..16:00</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyConsumption[16]*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyFeedIn[16]*1000)))+"</td></tr>";
  webpage += "<tr><td>16:00..17:00</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyConsumption[17]*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyFeedIn[17]*1000)))+"</td></tr>";
  webpage += "<tr><td>17:00..18:00</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyConsumption[18]*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyFeedIn[18]*1000)))+"</td></tr>";
  webpage += "<tr><td>18:00..19:00</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyConsumption[19]*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyFeedIn[19]*1000)))+"</td></tr>";
  webpage += "<tr><td>19:00..20:00</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyConsumption[20]*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyFeedIn[20]*1000)))+"</td></tr>";
  webpage += "<tr><td>20:00..21:00</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyConsumption[21]*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyFeedIn[21]*1000)))+"</td></tr>";
  webpage += "<tr><td>21:00..22:00</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyConsumption[22]*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyFeedIn[22]*1000)))+"</td></tr>";
  webpage += "<tr><td>22:00..23:00</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyConsumption[23]*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyFeedIn[23]*1000)))+"</td></tr>";
  webpage += "<tr><td>23:00..0:00</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyConsumption[0]*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeterHourlyFeedIn[0]*1000)))+"</td></tr>";
  webpage += "<tr><td>Total</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeter24hConsumption*1000)))+"</td><td>&nbsp;&nbsp;&nbsp;"+String(int(round(electricMeter24hFeedIn*1000)))+"</td></tr>";
  webpage += "</table>";
  webpage += "<br>";
  webpage += "Battery min: "+String(battery.socMin)+"% ("+secondsToTimeStr(battery.socMinTime+ELECTRIC_METER_TIME_OFFSET,true)+")<br>";
  webpage += "Battery max: "+String(battery.socMax)+"% ("+secondsToTimeStr(battery.socMaxTime+ELECTRIC_METER_TIME_OFFSET,true)+")<br>";
  webpage += "ACin Umin: "+String(acVoltageMin, 2)+"V ("+secondsToTimeStr(timeAcVoltageMin+ELECTRIC_METER_TIME_OFFSET,true)+")<br>";
  webpage += "ACin Umax: "+String(acVoltageMax, 2)+"V ("+secondsToTimeStr(timeAcVoltageMax+ELECTRIC_METER_TIME_OFFSET,true)+")<br>";
  webpage += "AC frequency min: "+String(acFrequencyMin, 3)+"Hz ("+secondsToTimeStr(timeAcFrequencyMin+ELECTRIC_METER_TIME_OFFSET,true)+")<br>";
  webpage += "AC frequency max: "+String(acFrequencyMax, 3)+"Hz ("+secondsToTimeStr(timeAcFrequencyMax+ELECTRIC_METER_TIME_OFFSET,true)+")<br>";
  webpage += "DC Umin: "+String(dcVoltageMin, 2)+"V<br>";
  webpage += "DC Umax: "+String(dcVoltageMax, 2)+"V<br>";
  webpage += "DC discharge current max: "+String(dcCurrentMin,1)+"A<br>";
  webpage += "DC charge current max: "+String(dcCurrentMax,1)+"A<br>";
  webpage += "Multiplus temp. min: "+String(multiplusTempMin,1)+"&deg;C<br>";
  webpage += "Multiplus temp. max: "+String(multiplusTempMax,1)+"&deg;C<br>";
  webpage += "BMS temp. min: "+String(batteryTempMin,1)+"&deg;C<br>";
  webpage += "BMS temp. max: "+String(batteryTempMax,1)+"&deg;C<br>";
  webpage += "BMS current/power min: "+String(batteryCurrentMin,1)+"A/"+String(batteryPowerMin)+"W<br>";
  webpage += "BMS current/power max: "+String(batteryCurrentMax,1)+"A/"+String(batteryPowerMax)+"W<br>";
  webpage += "Multiplus DC volt calibr.: "+String(1000*multiplusDcVoltageCalibration,1)+"mV<br>";
  webpage += "Multiplus DC volt cal cnt: "+String(multiplusDcVoltageCalibrationCnt)+"<br>";
  webpage += "Battery Rcable @charge: "+String(chargeCableResistance,1)+"m&#8486;<br>";
  webpage += "Battery Rcable charge cnt: "+String(chargeCableResistanceCnt)+"<br>";
  webpage += "Battery Rcable @discharge: "+String(dischargeCableResistance,1)+"m&#8486;<br>";
  webpage += "Battery Rcable dischg cnt: "+String(dischargeCableResistanceCnt)+"<br>";
  webpage += "<br>";
  webpage += "<a href=\"logfileAll.txt\">logfileAll.txt of last actions</a><br>";
  webpage += String(logfileAllCounter)+" bytes written since last logfile view<br>";
  webpage += "<br>";
  webpage += "<a href=\"logfileFew.txt\">logfileFew.txt of last actions</a><br>";
  webpage += String(logfileFewCounter)+" bytes written since last logfile view<br>";
  webpage += "</font></body></html>";
}


// ===========================================================================
//                          Sends main webpage
// ---------------------------------------------------------------------------
// This function handles displaying the web page as soon as one calls the IP
// adress of the ESP32 in a webbrowser within the local network.
// ===========================================================================
void handleRoot() {
  server.send(200, "text/html", webpage);
}

// ===========================================================================
//                Sends logfile buffer as logfile.txt via HTTP
// ---------------------------------------------------------------------------
// This function handles displaying/sending a file called logfile.txt as soon
// as one clicks on the link on the ESP32 webpage.
// The logfile has a size of (LOGFILE_ALL_SIZE) and is overwritten like a
// ringbuffer. So when it's full, oldest information is overwritten first.
// Improvement:
// Normally I would've displayed the logfile always from oldest to newest
// information. But with my limited programming skills I didn't find a way to
// do this without wasting an additional copy buffer in RAM. That's why I'm
// currently sending the buffer as it is. And before sending it, I simply add
// a "bar" into it, showing that above this bar it ends with the newest
// information and below this bar it starts with the oldest information.
// ===========================================================================
void handleLogfileAll() {
  int tmp = p_logfileAll;    //save logfile position pointer
  addToLogfile("\nMeter ACin ESSpower ##### newest above ######\n\n\n\n\n\n\n\n\n\n\n############# oldest entry below ############\n",1);  //temporary text marker
  server.send(200, "text/plain", logfileAll);
  p_logfileAll = tmp;    //restore original pointer so that temporary text will be overwritten again
  logfileAllCounter = 0;     //viewing the logfile resets the logfile size counter
}

void handleLogfileFew() {
  int tmp = p_logfileFew;    //save logfile position pointer
  addToLogfile("\n#### newest above ####\n",2);  //temporary text marker
  server.send(200, "text/plain", logfileFew);
  p_logfileFew = tmp;    //restore original pointer so that temporary text will be overwritten again
  logfileFewCounter = 0;     //viewing the logfile resets the logfile size counter
}

void handleButtonShortpress() {
  buttonPressCnt = BUTTON_DEBOUNCE_TIME;
  server.send(200, "text/html", "<meta http-equiv=\"refresh\" content=\"0; URL=/\">");
}

void handleButtonLongpress() {
  buttonPressCnt = BUTTON_LONGPRESS_TIME;
  server.send(200, "text/html", "<meta http-equiv=\"refresh\" content=\"0; URL=/\">");
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
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
  addToLogfile("\nESP32ESS START",3);
  //settings CAN/TWAI bus to battery
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  //Look in the api-reference for other speed sets.
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
//  twai_filter_config_t f_config = {.acceptance_code = (0x355 << 21),        //only filter 0x355 messages containing SOC (battery level)
//                                                 .acceptance_mask = ~(0x7FF << 21),
//                                                 .single_filter = true};
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
  //Setup serial ports
  Serial.begin(115200);  //SerialMonitor for debug information
  Serial1.begin(256000, SERIAL_8N1, VEBUS_RXD1, VEBUS_TXD1);  //VE.Bus RS485 to Multiplus
  Serial2.setRxBufferSize(4096); //4096 bytes is enough to store 10x full Info-DSS message of 352 bytes each
  Serial2.begin(9600, SERIAL_8N1, METER_SML_INPUT_PIN, SML_TXD_UNUSED); //, true);  //SML data from power meter //invert=false/true
  //IO pins
  pinMode(BUTTON_GPIO, INPUT_PULLUP);  //Initialize BOOT button GPIO as input with pull-up resistor
  pinMode(VEBUS_DE, OUTPUT);           //RS485 RE/DE direction pin for UART1
  digitalWrite(VEBUS_DE, LOW);         //set RS485 direction to read
  pinMode(METER_IMPULSE_INPUT_PIN, INPUT);//Power meter optical 1/10000kWh impulse input, currently evaluated in ISR
  pinMode(RED_LED, OUTPUT);            //Red LED currently showing the optical impulse from the power meter
  digitalWrite(RED_LED, LOW);          //turn red LED off
  pinMode(SHELLY_CF, OUTPUT);          //Pin with frequency pulses resulting in desired "power" on external Shelly 1PM
  digitalWrite(SHELLY_CF, LOW);        //Low is default state, pulses should be 38us long
  //Init webserver
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  server.on("/", handleRoot);
  // server.on("/", []() {
  //   server.send(200, "text/plain", "this works as well");
  // });
  server.on("/logfileAll.txt", handleLogfileAll);
  server.on("/logfileFew.txt", handleLogfileFew);
  server.on("/shortpress", handleButtonShortpress);
  server.on("/longpress", handleButtonLongpress);
  server.onNotFound(handleNotFound);
  server.begin();
  //set up interrupt timer for ISR
  My_timer = timerBegin(40000000);  //40MHz (or prescaler = 2)
  timerAttachInterrupt(My_timer, &onTimer);
  timerAlarm(My_timer, 4000, true, 0);  //value at which ISR is executed, cannot be 0, 1220 means 32760Hz, 4000 means 10kHz
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
            addToLogfile("\nwrong CS:",1);
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
  relativeTimers();                           //generate oneSecondOver and oneMinuteOver flags
  automaticSwitching();                       //Switch Multiplus into ChargerOnly-Mode if battery is below specific level
  veCmdReSendHandling();                      //Multiplus VEbus: Process VEbus information and execute command-sending state machine
  batteryHandling();                          //Battery: Get BMS values (e.g. SOC battery level) from CAN bus, if available
  checkSMLpowerMeter();                       //Power meter with Info-DSS: Check if new values are received and decode them
  checkImpulsePowerMeter();                   //Check if we also got new absolute meter value from 1/10000kWh impulses
  decideElectricMeterValueToUse();
  onNewMeterValue();                          //If new power value is available, add to logfile. Also calculates and sends new ESS power into Multiplus.
  controlConsumerShellys();
  switchSpecialModes();                       //handle BOOT button press activities (GPIO0)
  updateDisplays();                           //update Display and LEDs
  updateMainWebsite();
  server.handleClient();                      //handle webserver HTTP requests
}
