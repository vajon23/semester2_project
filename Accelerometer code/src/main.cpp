#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

#define g_value 9.80665

int counter = 0; // counts breaths
float count_threshold = 0.08; // thereshold for counter
float motion_threshold = g_value*0.3; // thereshold for motion (if above or below data is rejected)
float average, middle_point;
int flag = 0;

void Initiliazation_AM(void);
float averaged_value(void);
float middle(void);

Adafruit_MMA8451 mma = Adafruit_MMA8451();
sensors_event_t event;
unsigned int time=0;

void setup(void) {
  Serial.begin(9600);
  Initiliazation_AM();
  middle_point= middle();
}

void loop() {

  average = averaged_value();
  if((average > (middle_point+motion_threshold)) || (average < (middle_point-motion_threshold))) flag = 1;
  else flag = 0;

  if(flag==0){
    //float g = average/g_value;
   Serial.print("middle = ");Serial.print(middle_point); Serial.print("\t");
   
    Serial.print("V = ");Serial.print(average); Serial.print("\t");
    Serial.println();
    _delay_ms(100);

    if((average > (middle_point+count_threshold)) || (average < (middle_point-count_threshold))){
      counter++;
      Serial.print("Breath counter = ");Serial.print(counter); Serial.print("\t");
      _delay_ms(500);
    }


  }
  else{
    Serial.print("TOO MUCH MOTION!!! "); Serial.println();
    _delay_ms(1000);
  }

}

void Initiliazation_AM(void){
  Serial.println("Adafruit MMA8451 test!");
  if (! mma.begin()) {
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("MMA8451 found!");
  mma.setRange(MMA8451_RANGE_2_G);
  Serial.print("Range = "); Serial.print(2 << mma.getRange());  
  Serial.println("G");
}

float averaged_value(void){
int count = 5;
float sum = 0;
 for(int i=1;i<=count;i++){
    mma.read();
    mma.getEvent(&event);
    float V = sqrt(pow(event.acceleration.x,2)+pow(event.acceleration.y,2)+pow(event.acceleration.z,2));
    sum+=V;
  }
  average = sum / float(count);
  return average;
}

float middle(void){
  float sum2 = 0;
  Serial.println("Configuring Accelerometer...");
  for(int i=1; i<=20; i++){
    average = averaged_value();
    sum2+=average;
    _delay_ms(200);
  }
  float middle_point = sum2/float(20);
  Serial.println("Done!");
  return middle_point;
}