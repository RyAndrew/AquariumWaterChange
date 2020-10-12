#include <OneWire.h>
#include <DallasTemperature.h>

//comment out for production. disables serial messages output via usb
//#define DEV_ENABLE_SERIAL_DEBUG true

#define ONE_WIRE_BUS 10

OneWire dallasWire(ONE_WIRE_BUS);

DallasTemperature dallas(&dallasWire);

#define TEMPERATURE_PRECISION 12 // Highest resolution

//hard coded DS18B20 addresses, not ideal
uint8_t tempProbeRodiAddr[] ={ 0x28,0xFF,0x3D,0x20,0x71,0x17,0x03,0x74 };
uint8_t tempProbeTankAddr[] ={ 0x28,0xFF,0x1A,0xE4,0x70,0x17,0x03,0x41 };

float tempProbeRodiReading = 0;
float tempProbeRodiReadingLast = 0;

float tempProbeTankReading = 0;
float tempProbeTankReadingLast = 0;

#define COMMAND_PING 'a'

#define COMMAND_DRAIN_TANK_VALVE_OPEN 'd'
#define COMMAND_DRAIN_TANK_VALVE_CLOSE 'e'

#define COMMAND_FILL_TANK_PUMP_ON 'f'
#define COMMAND_FILL_TANK_PUMP_OFF 'g'

#define COMMAND_RODI_AIR_HEAT_ON 'h'
#define COMMAND_RODI_AIR_HEAT_OFF 'i'

#define COMMAND_TANK_FILTER_ON 'j'
#define COMMAND_TANK_FILTER_OFF 'k'

#define COMMAND_IDLE 'l' //resets everything to default state - filter on and water changer items off
#define COMMAND_HEAT_AERATE_RODI  'm' //heat the water in RODI storage until it reachest the tank temperature
#define COMMAND_DRAIN_TANK  'n' //opens solenoid valve until tank low sensor triggers
#define COMMAND_FILL_TANK 'o' //checks tank low is triggered and barrel full is triggered, runs pump until tank high is triggered or barrel low is triggered
#define COMMAND_WATER_CHANGE 'p' //run the Heat/Aerate, Drain, Fill commands in that order

#define COMMAND_RESCAN_TEMP_PROBES 'z'

char lastCommand = COMMAND_IDLE;
char runningCommand = '0';

#define NO_WATER 1
#define YES_WATER 0

enum commandResults {
  COMMAND_SUCCESS,
  COMMAND_RUNNING,
  COMMAND_FAIL
};
uint8_t lastCommandResult = COMMAND_SUCCESS;

uint8_t runningCommandStarted = false;
uint8_t runningWaterChangeCommand = false;

unsigned long rodiHeatStartTime = 0;
unsigned long rodiHeatTimeout = 21600000; // 6 * 60 * 60 * 1000 = 6 hours for max heat time

unsigned long tankDrainStartTime = 0;
unsigned long tankDrainTimeout = 900000; // 15 * 60 * 1000 = 15 minutes for max drain time

unsigned long tankFillStartTime = 0;
unsigned long tankFillTimeout = 600000; // 10 * 60 * 1000 = 10 minutes for max fill time

char serialCommand = 0;

//Tank High
uint8_t levelSensor1StateLast = 0;
uint8_t levelSensor1State = 0;
int levelSensor1 = 14;

//Tank Low
uint8_t levelSensor2StateLast = 0;
uint8_t levelSensor2State = 0;
int levelSensor2 = 16;

//RODI High
uint8_t levelSensor3StateLast = 0;
uint8_t levelSensor3State = 0;
int levelSensor3 = 15;

//RODI Low
uint8_t levelSensor4StateLast = 0;
uint8_t levelSensor4State = 0;
int levelSensor4 = 18;

int pinTankFilter = 19;
int pinTankFillPump = 9;
int pinRodiAirHeat = 8;

byte tankFilterState = 1;
byte tankFillPumpState = 0;
byte rodiAirHeatState = 0;

unsigned long lastReadTempsInterval = 750; //min 750 for 12 bit accuracy
unsigned long lastReadTempsTime = 0;

unsigned long minOutputStateInterval = 50;

unsigned long lastFullOutputStateInterval = 2000;
unsigned long lastFullOutputStateTime = 0;

//digital pin
int pinTankDrainSolenoidValveBkp = 21; //drain valve backup pin is always digital 0 or 1 in case PWM fails
int pinTankDrainSolenoidValve = 6;
byte tankDrainSolenoidValveState = 0;
unsigned long tankDrainSolenoidValveStateDelay = 1000;
unsigned long tankDrainSolenoidValveStateOpenTime = 0;
byte tankDrainSolenoidValveStatePwm = 0;

bool triggerSerialOutput = true;

// the setup routine runs once when you press reset:
void setup() {

  #ifdef DEV_ENABLE_SERIAL_DEBUG
    Serial.begin(115200); //usb port
  #endif
  
  Serial1.begin(115200); //rx tx pins
  Serial1.setTimeout(50); //50ms timeout for serial commands // Serial1.readStringUntil()

  pinMode(levelSensor1, INPUT);
  pinMode(levelSensor2, INPUT);
  pinMode(levelSensor3, INPUT);
  pinMode(levelSensor4, INPUT);

  pinMode(pinTankDrainSolenoidValve, OUTPUT);
  digitalWrite(pinTankDrainSolenoidValve, LOW);
  pinMode(pinTankDrainSolenoidValveBkp, OUTPUT);
  digitalWrite(pinTankDrainSolenoidValveBkp, LOW);

  //relay configurable high or low triggered, default high triggered
  pinMode(pinTankFilter, OUTPUT);
  digitalWrite(pinTankFilter, LOW);

  pinMode(pinTankFillPump, OUTPUT);
  digitalWrite(pinTankFillPump, HIGH);

  pinMode(pinRodiAirHeat, OUTPUT);
  digitalWrite(pinRodiAirHeat, HIGH);

  findAllTempSensors();
  
  dallas.setWaitForConversion(false);
  dallas.requestTemperatures();
}
// the loop routine runs over and over again forever:
void loop() {

  readTemps();

  readLevelSensors();
  
  readSerialCommand();
  
  processActiveCommand();

  handleDrainSolenoidState();
  
  outputState();

}
void readTemps() {

  if (millis() - lastReadTempsTime < lastReadTempsInterval) {
    return;
  }
  lastReadTempsTime = millis();

  //read rodi
  tempProbeRodiReading = dallas.getTempF(tempProbeRodiAddr);
  if(tempProbeRodiReading == DEVICE_DISCONNECTED_F ) 
  {
    #ifdef DEV_ENABLE_SERIAL_DEBUG
      Serial.println("Error reading temp from Rodi");
    #endif
    tempProbeRodiReading = 0;
  }
  if(tempProbeRodiReadingLast != tempProbeRodiReading){
    triggerSerialOutput = true;
  }
  tempProbeRodiReadingLast = tempProbeRodiReading;


  //read tank
  tempProbeTankReading = dallas.getTempF(tempProbeTankAddr);
  if(tempProbeTankReading == DEVICE_DISCONNECTED_F ) 
  {
    #ifdef DEV_ENABLE_SERIAL_DEBUG
      Serial.println("Error reading temp from Tank");
    #endif
    tempProbeTankReading = 0;
  }
  if(tempProbeTankReadingLast != tempProbeTankReading){
    triggerSerialOutput = true;
  }
  tempProbeTankReadingLast = tempProbeTankReading;
  
  dallas.requestTemperatures();
  
  #ifdef DEV_ENABLE_SERIAL_DEBUG
    Serial.print("reading temps ");
    Serial.print(millis() - lastReadTempsTime);
    Serial.println("ms elapsed");
  #endif
}

void readLevelSensors() {

  levelSensor1State = digitalRead(levelSensor1);
  if(levelSensor1State != levelSensor1StateLast){
    triggerSerialOutput = true;
  }
  levelSensor1StateLast = levelSensor1State;
  
  levelSensor2State = digitalRead(levelSensor2);
  if(levelSensor2State != levelSensor2StateLast){
    triggerSerialOutput = true;
  }
  levelSensor2StateLast = levelSensor2State;
  
  levelSensor3State = digitalRead(levelSensor3);
  if(levelSensor3State != levelSensor3StateLast){
    triggerSerialOutput = true;
  }
  levelSensor3StateLast = levelSensor3State;
  
  levelSensor4State = digitalRead(levelSensor4);
  if(levelSensor4State != levelSensor4StateLast){
    triggerSerialOutput = true;
  }
  levelSensor4StateLast = levelSensor4State;

}

void readSerialCommand(){

  if (Serial1.available() > 0){
    String cmdRecieved;
    int cmdRecievedLen;

    cmdRecieved = Serial1.readStringUntil("\r\n");
    //    Serial.print("cmdRecieved ");
    //    Serial.println(cmdRecieved);

    cmdRecievedLen = cmdRecieved.length();

    //    Serial.print("cmdRecieved length ");
    //    Serial.println(cmdRecievedLen);

    //    Serial.print("cmdRecieved DEC ");
    //    for(int i = 0; i < cmdRecievedLen; i++){
    //      Serial.print(cmdRecieved.charAt(i),DEC);
    //      Serial.print(" ");
    //    }
    //    Serial.println();


    if (cmdRecieved.length() == 2) {
      serialCommand = cmdRecieved.charAt(0);
    }
  }

  if (serialCommand != 0 && serialCommand > 96 && serialCommand < 123 ) {
    #ifdef DEV_ENABLE_SERIAL_DEBUG
      Serial.print("Recieved Command ");
      Serial.println(serialCommand);
    #endif
    handleSerialCommand();
    triggerSerialOutput = true;
  }

}

void handleSerialCommand() {

  switch (serialCommand) {
    case COMMAND_PING:
      #ifdef DEV_ENABLE_SERIAL_DEBUG
        Serial.println("pong");
      #endif
      Serial1.println("pong");
      break;
    case COMMAND_RESCAN_TEMP_PROBES:
      findAllTempSensors();
      break;
    case COMMAND_DRAIN_TANK_VALVE_OPEN:
      drainOpen();
      break;
    case COMMAND_DRAIN_TANK_VALVE_CLOSE:
      drainClose();
      break;
    case COMMAND_FILL_TANK_PUMP_ON:
      pumpOn();
      break;
    case COMMAND_FILL_TANK_PUMP_OFF:
      pumpOff();
      break;
    case COMMAND_RODI_AIR_HEAT_ON:
      rodiAirHeatOn();
      break;
    case COMMAND_RODI_AIR_HEAT_OFF:
      rodiAirHeatOff();
      break;
    case COMMAND_TANK_FILTER_ON:
      filterOn();
      break;
    case COMMAND_TANK_FILTER_OFF:
      filterOff();
      break;
    case COMMAND_HEAT_AERATE_RODI:
    
      resetCommandOutputs();
      runningWaterChangeCommand = false;
      runningCommand = lastCommand = COMMAND_HEAT_AERATE_RODI;
      lastCommandResult = COMMAND_RUNNING;
      break;
    case COMMAND_DRAIN_TANK:
    
      resetCommandOutputs();
      runningWaterChangeCommand = false;
      runningCommand = lastCommand = COMMAND_DRAIN_TANK;
      lastCommandResult = COMMAND_RUNNING;
      break;
    case COMMAND_FILL_TANK:
    
      resetCommandOutputs();
      runningWaterChangeCommand = false;
      runningCommand = lastCommand = COMMAND_FILL_TANK;
      lastCommandResult = COMMAND_RUNNING;
      break;
    case COMMAND_WATER_CHANGE:

      resetCommandOutputs();
      runningWaterChangeCommand = true;
      runningCommand = COMMAND_HEAT_AERATE_RODI;
      lastCommand = COMMAND_WATER_CHANGE;
      lastCommandResult = COMMAND_RUNNING;
      break;
    case COMMAND_IDLE:
      
      resetCommandOutputs();
      runningWaterChangeCommand = false;
      lastCommand = COMMAND_IDLE;
      runningCommand = COMMAND_IDLE;
      lastCommandResult = COMMAND_SUCCESS;
      break;
  }
  serialCommand = 0;

}
void processActiveCommand(){
  switch(runningCommand){
    //case COMMAND_IDLE:
    //    nothing to do
    //  break;
    
    case COMMAND_HEAT_AERATE_RODI:
    
      //if "rodi low" does not detect water = error, if "rodi high" does not detect water = error
      if(runningCommandStarted == false && (levelSensor3State == NO_WATER || levelSensor4State == NO_WATER)){
       #ifdef DEV_ENABLE_SERIAL_DEBUG
        if(levelSensor3State == NO_WATER){
            Serial.println("air heat failed - rodi not full");
        }
        if(levelSensor4State == NO_WATER){
            Serial.println("air heat failed - rodi low water detected");
        }
       #endif
       
        finishCommand(COMMAND_FAIL);
        return;  
      }
      
      //start timer if it hasn't, also useful for first time command has started, reset everything just in case
      if(runningCommandStarted == false){
        runningCommandStarted = true;
        rodiHeatStartTime = millis();
        rodiAirHeatOn();
        #ifdef DEV_ENABLE_SERIAL_DEBUG
          Serial.println("rodi heat air started");
        #endif
      }

      if (millis() - rodiHeatStartTime >= rodiHeatTimeout) {
        #ifdef DEV_ENABLE_SERIAL_DEBUG
          Serial.println("rodi heat air timeout failed");
        #endif
        finishCommand(COMMAND_FAIL);
        return;
      }
      
      //when rodi temp sensor matches tank temp sensor = we are done
      if(tempProbeRodiReading >= tempProbeTankReading){
        #ifdef DEV_ENABLE_SERIAL_DEBUG
          Serial.println("rodi heat air success");
        #endif
        finishCommand(COMMAND_SUCCESS);
      }
      break;
    case COMMAND_DRAIN_TANK:
    
      //if water sensor for "tank low" does not detect water = tank already drained = error
      if(runningCommandStarted == false && levelSensor2State == NO_WATER){
        #ifdef DEV_ENABLE_SERIAL_DEBUG
          Serial.println("drain failed - tank low = no water");
        #endif
        
        finishCommand(COMMAND_FAIL);
        return;
      }
      
      //start timer if it hasn't, also useful for first time command has started, reset everything just in case
      if(runningCommandStarted == false){
        runningCommandStarted = true;
        
        tankDrainStartTime = millis();
        
        drainOpen();
        #ifdef DEV_ENABLE_SERIAL_DEBUG
          Serial.println("draining tank");
        #endif
      }

      if (millis() - tankDrainStartTime >= tankDrainTimeout) {
        #ifdef DEV_ENABLE_SERIAL_DEBUG
          Serial.println("draining tank timeout failed");
        #endif
        finishCommand(COMMAND_FAIL);
        return;
      }


      //when water sensor for "tank low" does not detect water = we are done
      if(levelSensor2State == NO_WATER){
        #ifdef DEV_ENABLE_SERIAL_DEBUG
          Serial.println("draining tank success");
        #endif
        finishCommand(COMMAND_SUCCESS);
      }
      
      break;
      
    case COMMAND_FILL_TANK:

      //if "tank high" detects water = error, if "rodi low" detects water = error, if "rodi high" does not detect water = error
      if(runningCommandStarted == false && (levelSensor1State == YES_WATER || levelSensor3State == NO_WATER || levelSensor4State == NO_WATER)){
       #ifdef DEV_ENABLE_SERIAL_DEBUG
        if(levelSensor1State == YES_WATER){
            Serial.println("fill failed - tank full");
        }
        if(levelSensor3State == NO_WATER){
          Serial.println("fill failed - rodi not full");
        }
        if(levelSensor4State == NO_WATER){
          Serial.println("fill failed - rodi low water");
        }
       #endif
       
        finishCommand(COMMAND_FAIL);
        return;  
      }
      
      //start timer if it hasn't, also useful for first time command has started, reset everything just in case
      if(runningCommandStarted == false){
        runningCommandStarted = true;
        tankFillStartTime = millis();
        pumpOn();
        #ifdef DEV_ENABLE_SERIAL_DEBUG
          Serial.println("filling tank");
        #endif
      }

      if (millis() - tankFillStartTime >= tankFillTimeout) {
        #ifdef DEV_ENABLE_SERIAL_DEBUG
          Serial.println("filling tank timeout failed");
        #endif
        finishCommand(COMMAND_FAIL);
        return;
      }
      
      //water sensor for "tank high" detects water = command done
      if(levelSensor1State == YES_WATER){
        #ifdef DEV_ENABLE_SERIAL_DEBUG
          Serial.println("filling tank success");
        #endif
        finishCommand(COMMAND_SUCCESS);
        return;
      }
      
      //water sensor for "rodi low" does not detect water = command done
      if(levelSensor4State == NO_WATER){
        #ifdef DEV_ENABLE_SERIAL_DEBUG
          Serial.println("filling tank low rodi fail");
        #endif
        finishCommand(COMMAND_FAIL);
      }
          
      break;
  }
  
}
void finishCommand(uint8_t result){
  
  //if a command is done - if we are changing the water - proceed to the next command
  
  if(result == COMMAND_FAIL){
    runningWaterChangeCommand = false;
  }
  
  resetCommandOutputs();
  
  if(runningWaterChangeCommand == true){
    switch(runningCommand){
      case COMMAND_HEAT_AERATE_RODI:
        filterOff();
        runningCommand = COMMAND_DRAIN_TANK;
        break;
      case COMMAND_DRAIN_TANK:
        //filter stays off for next 2 commands in sequence
        runningCommand = COMMAND_FILL_TANK;
        break;
      case COMMAND_FILL_TANK:
      default:
        runningWaterChangeCommand = false;
        runningCommand = 0;
        lastCommandResult = result;
        triggerSerialOutput = true;

        //must turn filter back on because it's exlcuded from resetCommandOutputs when runningWaterChangeCommand = true
        filterOn();
        break;
    }
  }else{
    runningCommand = 0;
    lastCommandResult = result;
    triggerSerialOutput = true;
  }
}

void resetCommandOutputs(){
  if(tankDrainSolenoidValveState == 1){
    drainClose();
  }
  if(runningWaterChangeCommand == false){
    filterOn();
  }
  pumpOff();
  rodiAirHeatOff();
  
  runningCommandStarted = false;
}
void drainOpen(){
  tankDrainSolenoidValveState = 1;
  tankDrainSolenoidValveStatePwm = 0;
  
  digitalWrite(pinTankDrainSolenoidValveBkp, HIGH);
}
void drainClose(){
  tankDrainSolenoidValveState = 0;
  tankDrainSolenoidValveStatePwm = 1;
  tankDrainSolenoidValveStateOpenTime = 0;
  
  digitalWrite(pinTankDrainSolenoidValveBkp, LOW);
}
void rodiAirHeatOn(){
  rodiAirHeatState = 1;
  digitalWrite(pinRodiAirHeat, LOW);
}
void rodiAirHeatOff(){
  rodiAirHeatState = 0;
  digitalWrite(pinRodiAirHeat, HIGH);
}
void filterOn(){
  tankFilterState = 1;
  digitalWrite(pinTankFilter, LOW);
}
void filterOff(){
  tankFilterState = 0;
  digitalWrite(pinTankFilter, HIGH);
}
void pumpOn(){
  tankFillPumpState = 1;
  digitalWrite(pinTankFillPump, LOW);
}
void pumpOff(){
  tankFillPumpState = 0;
  digitalWrite(pinTankFillPump, HIGH);
}
void findAllTempSensors() {

  dallas.begin();

  dallas.setResolution(tempProbeRodiAddr, TEMPERATURE_PRECISION);
  dallas.setResolution(tempProbeTankAddr, TEMPERATURE_PRECISION);

/*
  TempDeviceAddress deviceAddress;

  tempSensorCount = 0;

  oneWire.reset_search();

  while (oneWire.search(deviceAddress)) {
    if (dallas.validAddress(deviceAddress) && dallas.validFamily(deviceAddress)) {
      memcpy(deviceAddress, tempSensors[tempSensorCount], 8);


      Serial.print("found #");
      Serial.print(tempSensorCount + 1);
      Serial.print(" - ");
      for (int i = 0; i < 8; i++) {
        Serial.print(deviceAddress[i], HEX);
        Serial.print(",");
      }
      Serial.println("");

      tempSensorCount++;
    }
  }

  Serial.print("found temp sensors: ");
  Serial.println(tempSensorCount);
*/
}
void handleDrainSolenoidState() {

  if(tankDrainSolenoidValveState == 1){
      if(tankDrainSolenoidValveStatePwm == 0){
        if(tankDrainSolenoidValveStateOpenTime == 0){
          tankDrainSolenoidValveStateOpenTime = millis();
          analogWrite(pinTankDrainSolenoidValve, 255); // open valve 100%
          #ifdef DEV_ENABLE_SERIAL_DEBUG
            Serial.println("solenoid open at 100%");
          #endif
        }else{
            if (millis() - tankDrainSolenoidValveStateOpenTime >= tankDrainSolenoidValveStateDelay) {
              tankDrainSolenoidValveStateOpenTime = 0;
              tankDrainSolenoidValveStatePwm = 1;
              analogWrite(pinTankDrainSolenoidValve, 128); // open valve 50%
              #ifdef DEV_ENABLE_SERIAL_DEBUG
                Serial.println("solenoid open at 30%");
              #endif
            }
        }
      }
  }else{
    //if we just closed the valve, then give solenoid full voltage to "slam" it shut
      if(tankDrainSolenoidValveStatePwm == 1){
        if(tankDrainSolenoidValveStateOpenTime == 0){
          tankDrainSolenoidValveStateOpenTime = millis();
          analogWrite(pinTankDrainSolenoidValve, 255); // open valve 100%
          #ifdef DEV_ENABLE_SERIAL_DEBUG
            Serial.println("solenoid closing at 100%");
          #endif
          
        }else{
          if (millis() - tankDrainSolenoidValveStateOpenTime >= tankDrainSolenoidValveStateDelay) {
            tankDrainSolenoidValveStatePwm = 0;
            tankDrainSolenoidValveStateOpenTime = 0;
            analogWrite(pinTankDrainSolenoidValve, 0); // open valve 0%
            #ifdef DEV_ENABLE_SERIAL_DEBUG
              Serial.println("solenoid closing at 0%");
            #endif
            
          }
        }
      }
  }
}
void outputState(){
  unsigned long lastSerial = millis() - lastFullOutputStateTime;
  
  if( (triggerSerialOutput == 0 || lastSerial < minOutputStateInterval) && lastSerial < lastFullOutputStateInterval ){
      return;
  }
  triggerSerialOutput = 0;
  lastFullOutputStateTime = millis();
  
  
  #ifdef DEV_ENABLE_SERIAL_DEBUG
    Serial.print("c=");Serial.print(lastCommand);
  #endif
  Serial1.print("c=");Serial1.print(lastCommand);


  #ifdef DEV_ENABLE_SERIAL_DEBUG
    Serial.print(",cr=");Serial.print(lastCommandResult);
  #endif
  Serial1.print(",cr=");Serial1.print(lastCommandResult);

  
  #ifdef DEV_ENABLE_SERIAL_DEBUG
    Serial.print(",w1=");Serial.print(levelSensor1State == YES_WATER ? "1" : "0");
  #endif
  Serial1.print(",w1=");Serial1.print(levelSensor1State == YES_WATER ? "1" : "0");
  
  #ifdef DEV_ENABLE_SERIAL_DEBUG
    Serial.print(",w2=");Serial.print(levelSensor2State == YES_WATER ? "1" : "0");
  #endif
  Serial1.print(",w2=");Serial1.print(levelSensor2State == YES_WATER ? "1" : "0");

  #ifdef DEV_ENABLE_SERIAL_DEBUG
    Serial.print(",w3=");Serial.print(levelSensor3State == YES_WATER ? "1" : "0");
  #endif
  Serial1.print(",w3=");Serial1.print(levelSensor3State == YES_WATER ? "1" : "0");

  #ifdef DEV_ENABLE_SERIAL_DEBUG
    Serial.print(",w4=");Serial.print(levelSensor4State == YES_WATER ? "1" : "0");
  #endif
  Serial1.print(",w4=");Serial1.print(levelSensor4State == YES_WATER ? "1" : "0");


  #ifdef DEV_ENABLE_SERIAL_DEBUG
    Serial.print(",t1=");Serial.print(tempProbeRodiReading);
  #endif
  Serial1.print(",t1=");Serial1.print(tempProbeRodiReading);
  
  
  #ifdef DEV_ENABLE_SERIAL_DEBUG
    Serial.print(",t2=");Serial.print(tempProbeTankReading);
  #endif
  Serial1.print(",t2=");Serial1.print(tempProbeTankReading);
  
  
  #ifdef DEV_ENABLE_SERIAL_DEBUG
    Serial.print(",m1=");Serial.print(tankDrainSolenoidValveState);
  #endif
  Serial1.print(",m1=");Serial1.print(tankDrainSolenoidValveState);
  
  
  #ifdef DEV_ENABLE_SERIAL_DEBUG
    Serial.print(",r1=");Serial.print(tankFillPumpState);
  #endif
  Serial1.print(",r1=");Serial1.print(tankFillPumpState);
  
  
  #ifdef DEV_ENABLE_SERIAL_DEBUG
    Serial.print(",r2=");Serial.print(rodiAirHeatState);
  #endif
  Serial1.print(",r2=");Serial1.print(rodiAirHeatState);

  
  #ifdef DEV_ENABLE_SERIAL_DEBUG
    Serial.print(",r3=");Serial.print(tankFilterState);
  #endif
  Serial1.print(",r3=");Serial1.print(tankFilterState);
  
  #ifdef DEV_ENABLE_SERIAL_DEBUG
    Serial.println();
  #endif
  Serial1.println();

}
