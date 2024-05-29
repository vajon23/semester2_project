#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <DFRobot_Heartrate.h>


void Data_line(unsigned long int time, int bpm_breathing, int bpm_PPG, float celsius1, float celsius2, float delta1, float delta2); // Puts together the data line that is saved to the SD card or showed in the monitor
void Initial_values(float celsius1, float celsius2, float& Initial_AM_value, float& Person_initial_1, float& Person_initial_2); // calculates Initial values for temperature sensors and accelerometer
unsigned long int time=0; // used for time stamp 
/***************PPG Sensor ****************/
DFRobot_Heartrate Heartrate(ANALOG_MODE);
/***************Temperature Sensors ****************/
float Person_initial_1, Person_initial_2; // Person's initial temperature values from each sensor
float celsius1, celsius2; // Person's current temperature values from each sensor
void Temperatures(const int Pin1, const int Pin2, float& celsius1, float& celsius2); // Calculates the current temperature values (which are averaged)
/***************Accelerometer ****************/
Adafruit_MMA8451 mma = Adafruit_MMA8451();
int bpm_breathing =0;
sensors_event_t event; // gives out values in m/s^2
#define g_value 9.80665 // definition of g constant value
long unsigned int t0_breath, t1_breath; // breathing in time of first and second breath
float Initial_AM_value; // Initial accelerometer vector V value
int breath_counter = 0; // counts breaths
int trigger = 1; // makes sures only one BPM value is calculated
int period_flag = 0; // needed for breath period determination
static int timer_flag = 0; // if  set timerflag=1, activate timer
static int timer_flag3 = 0; // timer flag for accelerometer offset reset
void Initiliazation_AM(void); // says if the Accelerometer is working properly
float Acceleration(void); // gives out an avareged value of 5 values from Accelerometer
void set_timer(unsigned long int millisec); // set a timer in the background while not disturbing other functions
void set_timer_AC_reset(unsigned long int millisec);
float breathing_period(void); // calculates the period of the breathing 
bool reading=false;
/*********************SD_card***************************/
static int timer_flag2 = 0; // timer for data output rate
void set_timer_data(unsigned long int millisec);
void Initialization_SD(void); // says if the SD card is working properly
void Write_to_file(char* file_name, unsigned long int time, int bpm_breathing, int bpm_PPG, float celsius1, float celsius2, float delta1, float delta2); // writes data_string into file with file_name

float max, min;

void setup(void) {
  Serial.begin(9600);
  Initiliazation_AM();
  Initialization_SD();
  Initial_values(celsius1, celsius2, Initial_AM_value, Person_initial_1, Person_initial_2);
  Serial.print("offset ");Serial.print(Initial_AM_value);Serial.print("\n");
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  reading=true;
  max=Initial_AM_value;
  min=Initial_AM_value;
}

void loop() {
  /***************************RECORDING SETUP*************************/
  if (!reading) {
    char c = 0;
    Serial.readBytes(&c, 1);
    if (c=='S') {
      reading=true;
      Serial.println("Starting...");
    }
  } 
  else {
    if (Serial.available()) {
      char c = 0;
      Serial.readBytes(&c, 1);
      if (c=='S') {
        reading=false;
        Serial.println("Stopping...");
      }
    }
    
    /************************Accelerometer ************************/
    float breath_period;
    int motion_flag = 0; // used for filtering out high motion
    float reset_threshold = 0.02;
    float count_threshold = 0.006; // thereshold for breath counter
    float motion_threshold = g_value*0.3; // thereshold for motion (if above, data is rejected)
    float sum2;
    float average = Acceleration();
    set_timer(1000);

    if((average > (Initial_AM_value+motion_threshold)) || (average < (Initial_AM_value-motion_threshold))) motion_flag = 1; // sets flag to 1 if there is too much motion
    else motion_flag = 0; // sets flag to 0 if the motion value is okay

    if(motion_flag==0){
      if((average > (Initial_AM_value+reset_threshold)) || (average < (Initial_AM_value-reset_threshold) || bpm_breathing>26)){
          sum2 = 0;
          int idx=0;
          while(idx<20){
            if(timer_flag3==0){
              timer_flag3 = 1;
              average = Acceleration();
              sum2+=average;
              idx++;
            }
            set_timer_AC_reset(10);
          }
          Initial_AM_value = sum2/float(20);
          bpm_breathing = 16;
      }
      if(timer_flag==0){
        if((average > (Initial_AM_value+count_threshold)) || (average < (Initial_AM_value-count_threshold))){ // checks if the acceleration is beyond breathing threshold to register a breath
          breath_counter++;
          breath_period = breathing_period();
          timer_flag = 1; // activates a 1 second timer after registering a breathing motion to reduce multiple captures of one breath count
        }
        if(period_flag == 0 && trigger!=1){
        bpm_breathing = 60000 / breath_period; // dividing 60 seconds by breath period to get breathing frequency per minute
        trigger=1;
        }
      }
    }

  /**************************TEMPERATURE****************************************/
    const int Pin1 = A6, Pin2 = A7; // Temp sensor pin wire connected to analog pin 6
    Temperatures(Pin1, Pin2, celsius1, celsius2);
    float delta1 = celsius1 - Person_initial_1;//the change between the current temperature and the person's initial tempeperature values for both sensors
    float delta2 = celsius2 - Person_initial_2;

  /**************************PPG**************************************/
   static int bpm_PPG;
   Heartrate.getValue(A1); // reads analog (raw) value from the sensor
   int hr = Heartrate.getRate();// compiles a value in BPM for hear rate
   bpm_PPG = hr?hr:bpm_PPG; // keeps the last BPM value
   
  /********************************PRINTING*******************************************/
    time = millis(); // time in ,mili seconds since start of measurement
    if(timer_flag2 == 0){
      timer_flag2 = 1;
      Data_line(time, bpm_breathing, bpm_PPG, celsius1, celsius2, delta1, delta2);
      Write_to_file((char*)"/d.csv", time, bpm_breathing, bpm_PPG, celsius1, celsius2, delta1, delta2);
    }
    set_timer_data(300);
  }
}
void Temperatures(const int Pin1, const int Pin2, float& celsius1, float& celsius2){
  float mv1, mv2;  // milivolts and final temperature      
  float TotTemp1 = 0;  // for oversampling
  float RawTemp1 = 0;   
  float TotTemp2 = 0;  // for oversampling
  float RawTemp2 = 0; 

  for (int x=0; x<64; x++){  // oversampling for more acurate reading
    TotTemp1 += analogRead(Pin1);
    TotTemp2 += analogRead(Pin2);
  }

  RawTemp1 = TotTemp1/64.0; //getting the mean temp (in bits)
  mv1 = ((RawTemp1/12.25)/1023.0)*5000.0;  // calculating the nr of milivolts -> 12.48 because that is the amplification
  celsius1 = mv1/10.0;  // the voltage rises 10mv when temp rises 1degreeC 

  RawTemp2 = TotTemp2/64.0; //getting the mean temp (in bits)
  mv2 = ((RawTemp2/12.25)/1023.0)*5000.0;  // calculating the nr of milivolts
  celsius2 = mv2/10.0;  // the voltage rises 10mv when temp rises 1degreeC
}

void Data_line(unsigned long int time, int bpm_breathing, int bpm_PPG, float celsius1, float celsius2, float delta1, float delta2){
  Serial.print(time);Serial.print(';');
  Serial.print(bpm_breathing);Serial.print(';');
  Serial.print(bpm_PPG);Serial.print(';');
  Serial.print(celsius1);Serial.print(';');
  Serial.print(celsius2);Serial.print(';');
  Serial.print(delta1);Serial.print(';');
  Serial.print(delta2);Serial.print('\n');
}

void Initiliazation_AM(void){
  if (! mma.begin()) {
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("MMA8451 found!");
  mma.setRange(MMA8451_RANGE_2_G); // set range to +-2g
  mma.writeRegister8(MMA8451_REG_CTRL_REG1, 0x04); // active low noise mode, max range +-4g
  mma.setDataRate(MMA8451_DATARATE_1_56_HZ); // set data output rate, (640ms) lowest possible = less noise
  mma.writeRegister8(MMA8451_REG_CTRL_REG2, 0x02); // sets oversampling to high resolution mode, 1024 output sample ratio (averaging filter)
  Serial.print("Range = "); Serial.print(2 << mma.getRange());  
  Serial.println("G");
}

float Acceleration(void){
int count = 5;
float sum = 0;
float average;

 for(int i=1;i<=count;i++){
    mma.read();
    mma.getEvent(&event);
    float V = sqrt(pow(event.acceleration.x,2)+pow(event.acceleration.y,2)+pow(event.acceleration.z,2));
    sum+=V;
  }
  average = sum / float(count);
  return average;
}

void Initial_values(float celsius1, float celsius2, float& Initial_AM_value, float& Person_initial_1, float& Person_initial_2){
  float sum2 = 0;
  float temp_average_1 = 0, temp_average_2 = 0;
  Serial.println("Configuring Sensors...");
  for(int i=1; i<=30; i++){
   float average = Acceleration();
    sum2+=average;
    temp_average_1 += celsius1;
    temp_average_2 += celsius2;
    delay(100);
  }
  Initial_AM_value = sum2/float(30);
  Person_initial_1 = temp_average_1 / float(30);  //the persons individual temperature based on the average on their first 30 measured temps
  Person_initial_2 = temp_average_2 / float(30);
  Serial.println("Done!");
  
}

void Initialization_SD(void){
 Serial.print("Initializing SD card...");

  if (!SD.begin(10)) { // for arduino nano SS pin is 10
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
}

void Write_to_file(char* file_name, unsigned long int time, int bpm_breathing, int bpm_PPG, float celsius1, float celsius2, float delta1, float delta2){
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File myFile = SD.open(file_name, O_CREAT | O_APPEND | O_WRITE);

   //if the file opened okay, write to it:
  if (myFile) {
   // Serial.print("Writing to " ); Serial.print(file_name);Serial.print("...\n");
    myFile.print(time);myFile.print(';');myFile.flush();
    myFile.print(bpm_breathing);myFile.print(';');myFile.flush();
    myFile.print(bpm_PPG);myFile.print(';');myFile.flush();
    myFile.print(celsius1);myFile.print(';');myFile.flush();
    myFile.print(celsius2);myFile.print('\n');myFile.flush(); // this line writes in SD card
    // close the file:
    myFile.close();
   // Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.print("error opening ");Serial.println(file_name);
  }
}

void set_timer(unsigned long int millisec){
  static long unsigned int t_start;
  if(timer_flag==1){
    t_start=millis();
    timer_flag=2;
  }
  if(timer_flag==2){
    if(t_start+millisec<millis()) timer_flag = 0;
  }
}

void set_timer_data(unsigned long int millisec){
  static long unsigned int t_start2;
  if(timer_flag2==1){
    t_start2=millis();
    timer_flag2=2;
  }
  if(timer_flag2==2){
    if(t_start2+millisec<millis()) timer_flag2 = 0;
  }
}

void set_timer_AC_reset(unsigned long int millisec){
  static long unsigned int t_start3;
  if(timer_flag3==1){
    t_start3=millis();
    timer_flag3=2;
  }
  if(timer_flag3==2){
    if(t_start3+millisec<millis()) timer_flag3 = 0;
  }
}

float breathing_period(void) {
  float breath_period; // Period between breaths
  if(period_flag == 2){
      t1_breath = millis();// takes the time value of second breath
      breath_period = t1_breath-t0_breath;  // substracts the two values = Period
      period_flag = 0;
      trigger = 0;
      return breath_period;
    }

  if(period_flag < 2) {
    if(period_flag == 0) t0_breath = millis(); // saves time value of first breath
    period_flag++;
  }
  return 0;
}