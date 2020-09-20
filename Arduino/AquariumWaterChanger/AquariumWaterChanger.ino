#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 10

OneWire dallasWire(ONE_WIRE_BUS);

DallasTemperature dallas(&dallasWire);

#define TEMPERATURE_PRECISION 9 // Lower resolution

//hard coded DS18B20 addresses, not ideal
uint8_t tempProbeRodiAddr[] ={ 0x28,0xFF,0x3D,0x20,0x71,0x17,0x03,0x74 };
uint8_t tempProbeTankAddr[] ={ 0x28,0xFF,0x1A,0xE4,0x70,0x17,0x03,0x41 };

float tempProbeRodiReading = 0;
float tempProbeRodiReadingLast = 0;

float tempProbeTankReading = 0;
float tempProbeTankReadingLast = 0;

#define COMMAND_PING 'a'
//opens solenoid valve until tank low sensor triggers
#define COMMAND_DRAIN_TANK 'b'
//checks tank low is triggered and barrel full is triggered, runs pump until tank high is triggered or barrel low is triggered
#define COMMAND_FILL_TANK 'c'

#define COMMAND_DRAIN_TANK_VALVE_OPEN 'd'
#define COMMAND_DRAIN_TANK_VALVE_CLOSE 'e'
#define COMMAND_FILL_TANK_PUMP_ON 'f'
#define COMMAND_FILL_TANK_PUMP_OFF 'g'

#define COMMAND_RODI_AIR_HEAT_ON 'h'
#define COMMAND_RODI_AIR_HEAT_OFF 'i'

#define COMMAND_TANK_FILTER_ON 'j'
#define COMMAND_TANK_FILTER_OFF 'k'

#define COMMAND_IDLE 'l'
#define COMMAND_DRAIN_TANK 'm'
#define COMMAND_FILL_TANK 'n'
#define COMMAND_WATER_CHANGE 'o'

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

unsigned long lastReadTempsInterval = 1000;
unsigned long lastReadTempsTime = 0;

unsigned long minOutputStateInterval = 100;

unsigned long lastFullOutputStateInterval = 2000;
unsigned long lastFullOutputStateTime = 0;

//digital pin
int pinTankDrainSolenoidValveBkp = 21;
int pinTankDrainSolenoidValve = 6;
byte tankDrainSolenoidValveState = 0;
unsigned long tankDrainSolenoidValveStateInterval = 1000;
unsigned long tankDrainSolenoidValveStateOpenTime = 0;
byte tankDrainSolenoidValveStatePwm = 0;

unsigned long tankDrainStartTime = 0;
unsigned long tankDrainTimeout = 10 * 60 * 1000;// 10 minutes for max drain time

unsigned long tankFillStartTime = 0;
unsigned long tankFillTimeout = 5 * 60 * 1000;// 5 minutes for max fill time

bool triggerSerialOutput = true;

// the setup routine runs once when you press reset:
void setup() {

  Serial.begin(115200); //usb port
  Serial1.begin(115200); //rx tx pins
  Serial1.setTimeout(50); //50ms timeout for serial commands // Serial.readStringUntil()

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
void processActiveCommand(){
  switch(runningCommand){
    //case COMMAND_IDLE:
    //    nothing to do
    //  break;
    
    case COMMAND_DRAIN_TANK:
    
      //if water sensor for "tank low" does not detect water = tank already drained = error
      if(tankDrainStartTime == 0 && levelSensor2State == NO_WATER){
        Serial.println("drain failed - tank low = no water");
        resetAllOutputs();
        runningCommand = 0;
        lastCommandResult = COMMAND_FAIL;
        triggerSerialOutput = true;
        return;  
      }
      
      //start timer if it hasn't, also useful for first time command has started, reset everything just in case
      if(tankDrainStartTime == 0){
        resetAllOutputs();
        tankDrainStartTime = millis();
      }

      if (millis() - tankDrainStartTime >= tankDrainTimeout) {
        resetAllOutputs();
        runningCommand = 0;
        tankDrainStartTime = 0;
        lastCommandResult = COMMAND_FAIL;
        triggerSerialOutput = true;
        return;
      }
      
      //if drain is closed open it!
      if(tankDrainSolenoidValveState == 0){
        drainOpen();
        Serial.println("draining tank");
      }

      //when water sensor for "tank low" does not detect water = we are done
      if(levelSensor2State == NO_WATER){
        resetAllOutputs();
        tankDrainStartTime = 0;
        runningCommand = 0;
        lastCommandResult = COMMAND_SUCCESS;
        triggerSerialOutput = 1;
      }
      
      break;
      
    case COMMAND_FILL_TANK:

      //if "tank high" detects water = error, if "rodi low" detects water = error, if "rodi high" does not detect water = error
      if(tankFillStartTime == 0 && (levelSensor1State == YES_WATER || levelSensor3State == NO_WATER || levelSensor4State == NO_WATER)){
        if(levelSensor1State == YES_WATER){
          Serial.println("fill failed - tank full");
        }
        if(levelSensor3State == NO_WATER){
          Serial.println("fill failed - rodi not full");
        }
        if(levelSensor4State == NO_WATER){
          Serial.println("fill failed - rodi low water");
        }
        resetAllOutputs();
        runningCommand = 0;
        lastCommandResult = COMMAND_FAIL;
        triggerSerialOutput = true;
        return;  
      }
      
      //start timer if it hasn't, also useful for first time command has started, reset everything just in case
      if(tankFillStartTime == 0){
        resetAllOutputs();
        tankFillStartTime = millis();
      }

      if (millis() - tankFillStartTime >= tankFillTimeout) {
        resetAllOutputs();
        runningCommand = 0;
        tankFillStartTime = 0;
        lastCommandResult = COMMAND_FAIL;
        triggerSerialOutput = true;
        return;
      }
      
      //if pump is off turn it on
      if(tankFillPumpState == 0){
        pumpOn();
        Serial.println("filling tank");
      }
      
      //water sensor for "tank high" detects water = command done
      if(levelSensor1State == YES_WATER){
        resetAllOutputs();
        tankFillStartTime = 0;
        runningCommand = 0;
        lastCommandResult = COMMAND_SUCCESS;
        triggerSerialOutput = 1;
      }
      
      //water sensor for "tank high" detects water = command done
      if(levelSensor1State == YES_WATER){
        resetAllOutputs();
        tankFillStartTime = 0;
        runningCommand = 0;
        lastCommandResult = COMMAND_SUCCESS;
        triggerSerialOutput = 1;
      }
          
      break;
      
    case COMMAND_WATER_CHANGE:

      //wait until rodi is full
      //wait until water temp matches tank
      //drain tank
      //fill tank
      
      break;
  }
  
}
void handleDrainSolenoidState() {

  if(tankDrainSolenoidValveState == 1){
      if(tankDrainSolenoidValveStatePwm == 0){
        if(tankDrainSolenoidValveStateOpenTime == 0){
          tankDrainSolenoidValveStateOpenTime = millis();
          analogWrite(pinTankDrainSolenoidValve, 255); // open valve 100%
          Serial.println("solenoid open at 100%");
        }else{
            if (millis() - tankDrainSolenoidValveStateOpenTime >= tankDrainSolenoidValveStateInterval) {
              tankDrainSolenoidValveStateOpenTime = 0;
              tankDrainSolenoidValveStatePwm = 1;
              analogWrite(pinTankDrainSolenoidValve, 77); // open valve 30%
              Serial.println("solenoid open at 30%");
            }
        }
      }
  }else{
    //if we just closed the valve, then give solenoid full voltage for 250ms to "hammer" it shut
      if(tankDrainSolenoidValveStatePwm == 1){
        if(tankDrainSolenoidValveStateOpenTime == 0){
          tankDrainSolenoidValveStateOpenTime = millis();
          analogWrite(pinTankDrainSolenoidValve, 255); // open valve 100%
          Serial.println("solenoid closing at 100%");
        }else{
          if (millis() - tankDrainSolenoidValveStateOpenTime >= tankDrainSolenoidValveStateInterval) {
            tankDrainSolenoidValveStatePwm = 0;
            tankDrainSolenoidValveStateOpenTime = 0;
            analogWrite(pinTankDrainSolenoidValve, 0); // open valve 0%
            Serial.println("solenoid closing at 0%");
          }
        }
      }
  }
}

void handleSerialCommand() {

  switch (serialCommand) {
    case COMMAND_PING:
      Serial.println("pong");
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
    case COMMAND_DRAIN_TANK:
      runningCommand = lastCommand = COMMAND_DRAIN_TANK;
      lastCommandResult = COMMAND_RUNNING;
      break;
    case COMMAND_FILL_TANK:
      runningCommand = lastCommand = COMMAND_FILL_TANK;
      lastCommandResult = COMMAND_RUNNING;
      break;
    case COMMAND_WATER_CHANGE:
      runningCommand = lastCommand = COMMAND_WATER_CHANGE;
      lastCommandResult = COMMAND_RUNNING;
      break;
    case COMMAND_IDLE:
      resetAllOutputs();
      
      lastCommand = COMMAND_IDLE;
      runningCommand = COMMAND_IDLE;
      lastCommandResult = COMMAND_SUCCESS;
  
      break;
  }
  serialCommand = 0;

}
void resetAllOutputs(){
  if(tankDrainSolenoidValveState == 1){
    drainClose();
  }
  filterOn();
  pumpOff();
  rodiAirHeatOff();
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
void readTemps() {

  if (millis() - lastReadTempsTime < lastReadTempsInterval) {
    return;
  }
  lastReadTempsTime = millis();

  dallas.requestTemperatures();

  //read rodi
  tempProbeRodiReading = dallas.getTempF(tempProbeRodiAddr);
  if(tempProbeRodiReading == DEVICE_DISCONNECTED_F ) 
  {
    Serial.println("Error reading temp from Rodi");
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
    Serial.println("Error reading temp from Tank");
    tempProbeTankReading = 0;
  }
  if(tempProbeTankReadingLast != tempProbeTankReading){
    triggerSerialOutput = true;
  }
  tempProbeTankReadingLast = tempProbeTankReading;
  
}
void outputState(){
  unsigned long lastSerial = millis() - lastFullOutputStateTime;
  
  if( (triggerSerialOutput == 0 || lastSerial < minOutputStateInterval) && lastSerial < lastFullOutputStateInterval ){
      return;
  }
  triggerSerialOutput = 0;
  lastFullOutputStateTime = millis();
  
  Serial.print("c=");
  Serial1.print("c=");
  Serial.print(lastCommand);
  Serial1.print(lastCommand);
  
  Serial.print(",cr=");
  Serial1.print(",cr=");
  Serial.print(lastCommandResult);
  Serial1.print(lastCommandResult);
  
  Serial.print(",w1=");
  Serial1.print(",w1=");
  if (levelSensor1State == YES_WATER) {
    Serial.print("1");
    Serial1.print("1");
  } else {
    Serial.print("0");
    Serial1.print("0");
  }

  Serial.print(",w2=");
  Serial1.print(",w2=");
  if (levelSensor2State == YES_WATER) {
    Serial.print("1");
    Serial1.print("1");
  } else {
    Serial.print("0");
    Serial1.print("0");
  }

  Serial.print(",w3=");
  Serial1.print(",w3=");
  if (levelSensor3State == YES_WATER) {
    Serial.print("1");
    Serial1.print("1");
  } else {
    Serial.print("0");
    Serial1.print("0");
  }

  Serial.print(",w4=");
  Serial1.print(",w4=");
  if (levelSensor4State == YES_WATER) {
    Serial.print("1");
    Serial1.print("1");
  } else {
    Serial.print("0");
    Serial1.print("0");
  }
  
  Serial.print(",t1=");
  Serial1.print(",t1=");
  Serial.print(tempProbeRodiReading);
  Serial1.print(tempProbeRodiReading);
  
  Serial.print(",t2=");
  Serial1.print(",t2=");
  Serial.print(tempProbeTankReading);
  Serial1.print(tempProbeTankReading);


  Serial.print(",m1=");
  Serial1.print(",m1=");
  Serial.print(tankDrainSolenoidValveState);
  Serial1.print(tankDrainSolenoidValveState);

  Serial.print(",r1=");
  Serial1.print(",r1=");
  Serial.print(tankFillPumpState);
  Serial1.print(tankFillPumpState);
  
  Serial.print(",r2=");
  Serial1.print(",r2=");
  Serial.print(rodiAirHeatState);
  Serial1.print(rodiAirHeatState);

  Serial.print(",r3=");
  Serial1.print(",r3=");
  Serial.print(tankFilterState);
  Serial1.print(tankFilterState);
  
  Serial.println();
  Serial1.println();

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
    Serial.print("Recieved Command ");
    Serial.println(serialCommand);
    handleSerialCommand();
    triggerSerialOutput = true;
  }

}
