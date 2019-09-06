/*
  Arlo-Speed-Maneuvers
  
  Examples that use ArloRobot library to make the arlo travel in certain
  speeds for certain amounts of time.
*/
#include <QTRSensors.h>
#include <ArloRobot.h>                        // Include Arlo library
#include <SoftwareSerial.h>

         // Arlo object
SoftwareSerial ArloSerial(12, 13);            // Serial in I/O 12, out I/O 13

int countsLeft, countsRight;                  // Encoder counting variables



QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


void setup()                                  // Setup function
{
//  tone(4, 3000, 2000);                        // Piezospeaker beep
  //QTR Sensors setup
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A4,A5,A6,A7}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
  
//////////////////////////////////////////////////  
  
  Serial.println("Sketch running...");        // Display starting message
  
  ArloSerial.begin(19200);                    // Start DHB-10 serial com
  Arlo.begin(ArloSerial);                     // Pass to Arlo object

  Arlo.clearCounts();                         // Clear encoder counts

                // Go forward 144 counts/sec
  delay(1000);                                // for three seconds
  
                             // ...for one second

void loop() {
  
   uint16_t position = qtr.readLineBlack(sensorValues);

  if (sensorValues[3] > 750 && sensorValues[4] > 750 )
  {
      Arlo.writeSpeeds(30, 30);               // Go backward 144 counts/sec
    
    }//|| sensorValues[6] > 750 || sensorValues[5] > 750

      if (sensorValues[4] > 750 && sensorValues[5] > 750 )
  {
    //  Arlo.writeSpeeds(30, 5); 
      Arlo.writeSpeeds(5,20);// Go backward 144 counts/sec
    
    }
    
         if (sensorValues[7] > 750)
  {
    /
      Arlo.writeSpeeds(5,20);// Go backward 144 counts/sec
    
    }//|| sensorValues[1] > 750 || sensorValues[2] > 750 

         if (sensorValues[2] > 750 && sensorValues[3] > 750 )
  {
       Arlo.writeSpeeds(20, 5);
    }
            if (sensorValues[0] > 750 && sensorValues[1] > 750 )
  {
       Arlo.writeSpeeds(20, 5);
    }
    
    else
    {
      Arlo.writeSpeeds(0, 0);
      }  
   for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position); 
 
  delay(200);

 
 
 
