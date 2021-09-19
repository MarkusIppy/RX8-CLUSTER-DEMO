/*RX8 Cluster Demo with Rx8 Throttle pedal 
This Demo simmulates driving on a 5 Speed RX8 .
The REV counter will move based on the Trhottle input on A0 
Initially the Simmulation will be in Neutral gear , when reving first time to Max RPM , the car will shift into 1st gear.
Everytime you hit max rpm the car will shift up one gear until 5th gear. when releasing the throttle the rpm will go down and the car will down shift until neutral 

The Arduino will output a Square wave of 30 Hz per RPM with 50 % Duty cycle to drive other  mechanical or digital clusters (Verified with Apexi RSM)


Todo's : 
1. Code cleanup 
2. Implement 2nd frequency for speed output (Frequency * 1.4) 
3. Hold Max RPM a little while to make the shifts slower 

Credits : 
The Frequency code is adapted from :  https://github.com/shadjiiski/uno-square-wavegen
The Rx8 Cluster code is adapted from : https://github.com/DaveBlackH/MazdaRX8Arduino/
*/
#include <Arduino.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#define CANint 2

int throttlesig = A0;    // select the input pin for the potentiometer
int throttleval = 0;       // variable to store the value coming from the sensor
int throttletempval = 0; // Temporariy measurement
double test;
int maxrpmdelaycounter = 5;    //
int currentcounter = 0; //

const int numReadings = 20;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average



//////////// FREQUENCY GENERATOR ////////////////////
const long NORMALIZATION_FACTOR = 1000;
const long MIN_FREQUENCY = (int) (0.2 * NORMALIZATION_FACTOR);
const long MAX_FREQUENCY = 8000 * NORMALIZATION_FACTOR;
const long MIN_DUTY = 0 * NORMALIZATION_FACTOR;
const long MAX_DUTY = 1 * NORMALIZATION_FACTOR;

//const int prescalers[] = {1, 8, 64, 256, 1024};
long mFrequency = (long) (90.42 * NORMALIZATION_FACTOR); // 90.43 Hz
long mFrequencyB = (long) (90.42 * NORMALIZATION_FACTOR); // 90.43 Hz
long mDutyCycleA = (long) (0.1 * NORMALIZATION_FACTOR); // 10%, pin 9
long mDutyCycleB = (long) (0.1 * NORMALIZATION_FACTOR); // 10%, pin 10
//////////// FREQUENCY GENERATOR END  ////////////////////


MCP_CAN CAN0(10); // Set CS to pin 10


int requestedrpm = 1050;
int rpmudaterate = 40;
int currentgear = 0; //Gears 1-5
int acceldecel = 1; //1 car accelerates 0 car decelerates
int maxrpm = 8400;
int lastrpm;
//FD RX7 5 Speed gear ratios 
int ratio1  =  120;
int ratio2  =  74;
int ratio3  =  54;
int ratio4  =  37;
int ratio5  =  28;///25.8 ;//28;
int gearratio = ratio1; //Set initial gear ratio for 1st gear
long lastRefreshTime = 0;
long lastRefreshTime2 = 0;
long lastRefreshTime3 = 0;
// Variables for PCM, Only overrideable if PCM removed from CAN
bool checkEngineMIL;
bool checkEngineBL;
byte engTemp;
byte odo;
bool oilPressure;
bool lowWaterMIL;
bool batChargeMIL;
bool oilPressureMIL;

// Variables for PCM, Only overrideable if PCM removed from CAN
int engineRPM;
int vehicleSpeed;
byte throttlePedal;

// Variables for ABS/DSC, Only overrideable if ABS/DSC removed from CAN
bool dscOff;
bool absMIL;
bool brakeFailMIL;
bool etcActiveBL;
bool etcDisabled;

// Variables for Wheel Speed
int frontLeft;
int frontRight;
int rearLeft;
int rearRight;

//Variables for reading in from the CANBUS
unsigned char len = 0;
unsigned char buf[8];
unsigned long ID = 0;

//Setup Array's to store bytes to send to CAN on Various ID's
byte send201[8]  = {0, 0, 255, 255, 0, 0, 0, 255};
byte send420[7]  = {0, 0, 0, 0, 0, 0, 0};
byte send212[7]  = {0, 0, 0, 0, 0, 0, 0};
byte send300[1]  = {0};
//Setup PCM Status's required to fool all other CAN devices that everything is OK, just send these out continuously
byte send203[7]  = {19, 19, 19, 19, 175, 3, 19}; //data to do with traction control
byte send215[8]  = {2, 45, 2, 45, 2, 42, 6, 129};
byte send231[5]  = {15, 0, 255, 255, 0};
byte send240[8]  = {4, 0, 40, 0, 2, 55, 6, 129};
byte send620[7]  = {0, 0, 0, 0, 16, 0, 4}; //needed for abs light to go off, byte 7 is different on different cars, sometimes 2,3 or 4
byte send630[8]  = {8, 0, 0, 0, 0, 0, 106, 106}; //needed for abs light to go off, AT/MT and Wheel Size
byte send650[1]  = {0};

//KCM / Immobiliser chat replies
byte send41a[8] = {7, 12, 48, 242, 23, 0, 0, 0}; // Reply to 47 first
byte send41b[8] = {129, 127, 0, 0, 0, 0, 0, 0}; // Reply to 47 second

void setup() {
  Serial.begin(115200);
  setPWM(mFrequency, mDutyCycleA, mDutyCycleB); // Freqency Generator
  pinMode(9, OUTPUT);// Freqency Generator (RPM Square wave )
  pinMode(8, OUTPUT); // Freqency Generator (SPEED Square wave )
  pinMode(23, OUTPUT);
  digitalWrite(23, HIGH);
  Serial.println("Start Setup");

  pinMode(CANint, INPUT);

  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  if (CAN0.begin(CAN_500KBPS) == CAN_OK) {
    Serial.println("Found High Speed CAN");
  } else {
    Serial.println("Failed to find High Speed CAN");
    while (1) {
      Serial.print("Loop Forever");
      delay(1000);
    }
  }


  setDefaults(); 
}


void loop() {
  
  throttleinput();
  
//Every 200 ms we update the Cluster for temperature Oil Pressure and Stati
  if (millis() - lastRefreshTime >= 200) {
    lastRefreshTime += 200;  
    sendOnTenth();
    if(currentgear !=5)
    {
    maxrpm = 8600;
    requestedrpm = (throttleval * 77 + 1000);}
    else {
      requestedrpm = (throttleval * 74 + 1000);} // Ensure we only rev to 8400 RPM in 5th gear to avoid the annoing shift beep 
    if (requestedrpm < engineRPM)
    {
      acceldecel = 0;
    }
    else
    {
      acceldecel = 1;
    }
  }


  //Update CANBUS Live Data every 10 ms update RPM and Speed via Canbus

  if (millis() - lastRefreshTime2 >= 10) {
    lastRefreshTime2 += 10;
    fastsend();
  }
  //This update rate is variable to change the update speed of the rpm increase decrease per gear
  if (millis() - lastRefreshTime3 >= 50) {
    lastRefreshTime3 += 50;
    rpmupdate();
    drivingsim();
  }
}








void throttleinput() {
  //we will use a safe range as there are always fluctuations
  //the range we evaluate is from 340 to 800 on the analog readout , then we convert it as a percentage

  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = analogRead(throttlesig);
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;

  delay(1);        // delay in between reads for stability

///
  
  throttletempval = average;

  if (throttletempval > 340 & throttletempval <= 780)
  {
    throttleval = (throttletempval - 340) * 0.217L;
 
  }
  if (throttletempval > 780)
  {
    throttleval = 100;
  }
  if (throttletempval < 340)
  {
    throttleval = 0;
  }
 //  Serial.println("");
 // Serial.print("Throttle % : ");
//  Serial.print(throttleval);
}

void setDefaults() {
  Serial.println("Setup Started");
  // StatusMIL
  engTemp         = 145; //Roughly in the middle
  odo             = 0;
  oilPressure     = 1;
  checkEngineMIL  = 0;
  checkEngineBL   = 0;
  lowWaterMIL     = 0;
  batChargeMIL    = 0;
  oilPressureMIL  = 0;

  // StatusPCM
  engineRPM       = 900;   // RPM
  vehicleSpeed    = 1;      // KPH
  throttlePedal   = 0;      // %

  // StatusDSC
  dscOff          = 0;
  absMIL          = 0;
  etcActiveBL     = 0;
  etcDisabled     = 0;
  brakeFailMIL    = 0;
}

void updateMIL() {
  send420[0] = engTemp;
  send420[1] = odo;
  send420[4] = oilPressure;

  if (checkEngineMIL == 1) {
    send420[5] = send420[5] | 0b01000000;
  } else {
    send420[5] = send420[5] & 0b10111111;
  }

  if (checkEngineBL == 1) {
    send420[5] = send420[5] | 0b10000000;
  } else {
    send420[5] = send420[5] & 0b01111111;
  }

  if (lowWaterMIL == 1) {
    send420[6] = send420[6] | 0b00000010;
  } else {
    send420[6] = send420[6] & 0b11111101;
  }

  if (batChargeMIL == 1) {
    send420[6] = send420[6] | 0b01000000;
  } else {
    send420[6] = send420[6] & 0b10111111;
  }

  if (oilPressureMIL == 1) {
    send420[6] = send420[6] | 0b10000000;
  } else {
    send420[6] = send420[6] & 0b01111111;
  }
}

void updatePCM() {
  int tempEngineRPM = engineRPM * 3.85;
  int tempVehicleSpeed = (vehicleSpeed * 100) + 10000;

  send201[0] = highByte(tempEngineRPM);
  send201[1] = lowByte(tempEngineRPM);

  send201[4] = highByte(tempVehicleSpeed);
  send201[5] = lowByte(tempVehicleSpeed);

  send201[6] = (200 / 100) * throttlePedal;   //Pedal information is in 0.5% increments
}

void updateDSC() {
  if (dscOff == 1) {
    send212[3] = send212[3] | 0b00000100;
  } else {
    send212[3] = send212[3] & 0b01111011;
  }

  if (absMIL == 1) {
    send212[4] = send212[4] | 0b00001000;
  } else {
    send212[4] = send212[4] & 0b11110111;
  }

  if (brakeFailMIL == 1) {
    send212[4] = send212[4] | 0b01000000;
  } else {
    send212[4] = send212[4] & 0b10111111;
  }

  if (etcActiveBL == 1) {
    send212[5] = send212[5] | 0b00100000;
  } else {
    send212[5] = send212[5] & 0b11011111;
  }

  if (etcDisabled == 1) {
    send212[5] = send212[5] | 0b00010000;
  } else {
    send212[5] = send212[5] & 0b11101111;
  }
}

void sendOnTenth() {
  //PCM Status's to mimic the PCM being there, these may be different for different cars, and not all are always required, better safe and include them all.
  CAN0.sendMsgBuf(0x203, 0, 7, send203);
  CAN0.sendMsgBuf(0x215, 0, 8, send215);
  CAN0.sendMsgBuf(0x231, 0, 8, send231);
  CAN0.sendMsgBuf(0x240, 0, 8, send240);
  CAN0.sendMsgBuf(0x620, 0, 7, send620);
  CAN0.sendMsgBuf(0x630, 0, 8, send630);
  CAN0.sendMsgBuf(0x650, 0, 1, send650);
  CAN0.sendMsgBuf(0x300, 0, 1, send300);
  
  updateMIL();
  CAN0.sendMsgBuf(0x420, 0, 7, send420);



  // Add this section back in if you want to take control of ABS / DSC Lights.
  updateDSC();
  CAN0.sendMsgBuf(0x212, 0, 7, send212);

}

void fastsend() {
  //PCM Status's to mimic the PCM being there, these may be different for different cars, and not all are always required, better safe and include them all.

  updatePCM();
  // Serial.print("CAN PCM");
  CAN0.sendMsgBuf(0x201, 0, 8, send201);
}

void drivingsim()
{

  //Shift through gears :

  //Upshift

  if (acceldecel == 1)
  {
    if (currentcounter < maxrpmdelaycounter && engineRPM >= maxrpm)
    {
      currentcounter++;

      }

    switch (currentgear) {
       case 0:
        vehicleSpeed = 0;
        if (currentcounter < maxrpmdelaycounter && engineRPM >= maxrpm)
        {
        rpmudaterate = 15; //
        }
        else
        {
        rpmudaterate = 400;  
          
        }
        if(engineRPM >= maxrpm & currentcounter == maxrpmdelaycounter)
          {
          currentcounter = 0;
          currentgear++;
          vehicleSpeed = 5;
          engineRPM = vehicleSpeed * gearratio;
          }
        break;
      case 1:
        gearratio = ratio1 ; // get car in gear
        rpmudaterate = 100; 
        if(engineRPM >= maxrpm)
          {
          currentgear++;
          engineRPM = vehicleSpeed * ratio2;
          }
        break;
      case 2:
        // We just switched to second gear so the rpm must fall accordingly
        gearratio = ratio2 ; // now we set the gear ratio for second gear
        rpmudaterate = 90; 
        if(engineRPM >= maxrpm)
          {
          currentgear++;
          engineRPM = vehicleSpeed * ratio3;
          }
        break;
      case 3:
        // Statement(s)
        gearratio = ratio3 ; // now we set the gear ratio for third gear
        rpmudaterate = 40; 
        if(engineRPM >= maxrpm)
          {
          currentgear++;
          engineRPM = vehicleSpeed * ratio4;
          }
        break;

      case 4:
        // Statement(s)
        gearratio = ratio4 ; // now we set the gear ratio for fouth gear
        rpmudaterate = 30;
        if(engineRPM >= maxrpm)
          {
          currentgear++;
          engineRPM = vehicleSpeed * ratio5;
          }
        break;

      case 5:
        // Statement(s)
        gearratio = ratio5; // now we set the gear ratio for fifth gear
        //engineRPM = vehicleSpeed * gearratio; // we drop the rpm accordingly
        rpmudaterate = 20; //
        //requestedrpm = 8400;
        // maxrpm = 8100;
        break;

      default:

        // Statement(s)

        break; //

    }
  }

  // Downshift

  if (acceldecel == 0)
  {



    switch (currentgear) {

      case 1:
        // We just switched to second gear so the rpm must fall accordingly
        if (engineRPM < 1500) // Downshift only if we dont't overrev
        {
          currentgear--;
          gearratio = 0;
        }
        rpmudaterate = 20; //( we update rpm rise every 20 ms)
        break;
      case 2:

        if ((vehicleSpeed * ratio1) < 8100) // Downshift only if we dont't overrev
        {
          currentgear--;
          engineRPM = vehicleSpeed * ratio1; // we drop the rpm accordingly
          gearratio = ratio1;
        }
        rpmudaterate = 20; //( we update rpm rise every 20 ms)
        //requestedrpm = 3000;
        break;
      case 3:
        if ((vehicleSpeed * ratio2) < 8100) // Downshift only if we dont't overrev
        {
          currentgear--;
          engineRPM = vehicleSpeed * ratio2; // we drop the rpm accordingly
          gearratio = ratio2;
        }
        rpmudaterate = 20; //( we update rpm rise every 25 ms)
        //requestedrpm = 3000;
        break;

      case 4:
        if ((vehicleSpeed * ratio3) < 8100) // Downshift only if we dont't overrev
        {
          currentgear--;
          engineRPM = vehicleSpeed * ratio3; // we drop the rpm accordingly
          gearratio = ratio3;
        }
        rpmudaterate = 20; //( we update rpm rise every 40 ms)
        //requestedrpm = 3000;
        break;

      case 5:

        if ((vehicleSpeed * ratio4) < 8100) // Downshift only if we dont't overrev
        {
          currentgear--;
          engineRPM = vehicleSpeed * ratio4; // we drop the rpm accordingly
          gearratio = ratio4;
        }
        rpmudaterate = 20; //( we update rpm rise every 45 ms)

        break;

      default:

        // Statement(s)

        break; //

    }
  }


}

void rpmupdate()
{

  // Serial.println(acceldecel);
  if ((acceldecel == 1) && (engineRPM < requestedrpm))
  {
    //Serial.println("Accelerate");

    test = engineRPM / 0.03L;
    setPWM(test, 500);
    if (currentgear != 0)
    {
      vehicleSpeed = engineRPM / gearratio;
      engineRPM = engineRPM + rpmudaterate; // Increase RPM by 50 RPM
    }
    else
    {
      if (engineRPM < 1050 ) //prevent bounce at idle
      {
      engineRPM = engineRPM + 10; // Increase RPM by 200 RPM
      }
      else{
        engineRPM = engineRPM + 200; // Increase RPM by 200 RPM
        }
    }
  }

  if ((acceldecel == 0) && (engineRPM > requestedrpm))
  {
    //Serial.println("Braking");
    //Serial.println(engineRPM);
    engineRPM = engineRPM - rpmudaterate;
    test = engineRPM / 0.03L;
    setPWM(test, 500);
    if (currentgear != 0)
    {
      vehicleSpeed = engineRPM / gearratio;
    }
  }
  lastrpm = engineRPM;
}
//////////FREQUENCY GENERATION ////////////////////////////////////////////////////////
/**
   Chooses the smallest prescaler for a given frequency. This way maximum duty
   cycle resolution is achieved.
*/
int getPrescalerForFrequency(long frequency) {
  //  tc = base frequency / target frequency / prescaler / 2
  // 65535 = 8M / (prescaler * frequency)
  // prescaler = 8M / (65535 * frequency)
  float prescaler = (8000000.0 / 65535.0) /
                    (1.0 * frequency / NORMALIZATION_FACTOR);
  if (prescaler <= 1)
    return 1;
  else if (prescaler <= 8)
    return 8;
  else if (prescaler <= 64)
    return 64;
  else if (prescaler <= 256)
    return 256;
  else if (prescaler <= 1024)
    return 1024;
  else // effectively stop the timer
    return 0;
}

/**
   Returns an integer that holds the proper bits set for the given prescaler.
   This value should be set to the TCCR1B register.

   Atmel Atmega 328 Datasheet:
   Table 16-5
*/
int preparePrescaler(int prescaler) {
  switch (prescaler) {
    case 1: return _BV(CS10);
    case 8: return _BV(CS11);
    case 64: return _BV(CS10) | _BV(CS11);
    case 256: return _BV(CS12);
    case 1024: return _BV(CS12) | _BV(CS10);
    // effectively stops the timer. This should show the user that wrong input
    // was provided.
    default: return 0;
  }
}

/**
   This sets a phase and frequency correct PWM mode.
   Counting up from 0 to ICR1 (inclusive).

   Atmel Atmega 328 Datasheet:
   Table 16-4
*/
inline int prepareWaveGenMode() {
  return _BV(WGM13);
}

/**
   Clear OC1A/OC1B on Compare Match when upcounting. Set OC1A/OC1B on Compare
   Match when downcounting.

   Atmel Atmega 328 Datasheet:
   Table 16-3
*/
inline int prepareNormalCompareOutputMode() {
  return _BV(COM1A1) | _BV(COM1B1);
}

/**
   Set OC1A/OC1B on Compare Match when upcounting. Clear OC1A/OC1B on Compare
   Match when downcounting.

   Atmel Atmega 328 Datasheet:
   Table 16-3
*/
inline int prepareInvertedCompareOutputMode() {
  return _BV(COM1A1) | _BV(COM1B1) | _BV(COM1A0) | _BV(COM1B0);
}

/**
   Set OC1A and clear OC1B on Compare Match when upcounting. Clear OC1A and set
   OC1B on Compare Match when downcounting.

   Atmel Atmega 328 Datasheet:
   Table 16-3
*/
inline int prepareInvertedANormalBCompareOutputMode() {
  return _BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1);
}

/**
   Sets the Timer/Counter1 Control Register A

   Atmel Atmega 328 Datasheet:
   Section 16.11.1
*/
void setTCCR1A() {
  TCCR1A = prepareNormalCompareOutputMode();
}

/**
   Sets the Timer/Counter1 Control Register B

   Atmel Atmega 328 Datasheet:
   Section 16.11.2
*/
void setTCCR1B(int prescaler) {
  TCCR1B = prepareWaveGenMode() | preparePrescaler(prescaler);
}

/**
   Sets PWM with the given frequency and duty cycle on both pins 9 and 10
*/
void setPWM(long frequency, long duty) {
  setPWM(frequency, duty, duty);
}

void setPWM(long frequency, long dutyA, long dutyB) {
  if (frequency < MIN_FREQUENCY ||
      frequency > MAX_FREQUENCY ||
      dutyA < MIN_DUTY ||
      dutyA > MAX_DUTY ||
      dutyB < MIN_DUTY ||
      dutyB > MAX_DUTY)
    return;
  int prescaler = getPrescalerForFrequency(frequency);
  // so we can't do this frequency. Interesting...
  if (prescaler == 0)
    return;

  // update the saved state
  mFrequency = frequency;
  mDutyCycleA = dutyA;
  mDutyCycleB = dutyB;

  setTCCR1A();
  setTCCR1B(prescaler);
  long top = (long)(8000000.0 /
                    (1.0 * prescaler * frequency / NORMALIZATION_FACTOR) + 0.5);
                    
  ICR1 = top;
  OCR1A = (long)(1.0 * top * dutyA / NORMALIZATION_FACTOR + 0.5);
  OCR1B = (long)(1.0 * top * dutyB / NORMALIZATION_FACTOR + 0.5);
}
