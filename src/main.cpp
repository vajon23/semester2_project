#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>


String Data_line(unsigned long int time, int bpm_breathing, int bpm_PPG, float Person_initial_1, float Person_initial_2); // Puts together the data line that is saved to the SD card or showed in the monitor
void Initial_values(float celsius1, float celsius2, float& Initial_AM_value, float& Person_initial_1, float& Person_initial_2); // calculates Initial values for temperature sensors and accelerometer
unsigned long int time=0; // used for time stamp 

/***************PPG Sensor ****************/
volatile float Tavrg = 0; //averaged period value used to calculate the freqeuncy (BPM)
void PPG_period(void); // Interrupt routine
volatile unsigned int first = 0; // flag for interrupt
volatile long int t0;
volatile float rawT; // Period between falling edges

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
int timer_flag = 0; // if  set timerflag=1, activate timer
void Initiliazation_AM(void); // says if the Accelerometer is working properly
float Acceleration(void); // gives out an avareged value of 5 values from Accelerometer
void set_timer(unsigned long int millisec); // set a timer in the background while not disturbing other functions
float breathing_period(void); // calculates the period of the breathing 
bool reading=false;

/*********************SD_card***************************/
File myFile; // used for remebering the text file that is being worked with in the SD card
File root; // needed to get SD card directory() function
void Initialization_SD(void); // says if the SD card is working properly
void printDirectory(File dir, int numTabs); // prints all the text file name that are in SD card
void Check_existance(String file_name); // chech if that name file is in SD card
void Read_whole_file(String file_name); // reads and prints out everything that is in a file
void Write_to_file(String file_name, String data_string); // writes data_string into file with file_name

void setup(void) {
  Serial.begin(9600);
  Initiliazation_AM();
  Initialization_SD();
  Initial_values(celsius1, celsius2, Initial_AM_value, Person_initial_1, Person_initial_2);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  root = SD.open("/");
  //printDirectory(root, 0);
  //Check_existance("example.txt");
  //Read_whole_file("example.txt");
  //SD.remove("example.txt");// delete this file
  pinMode(3, INPUT); // sets pin D3 as input for the PPG sensor
  attachInterrupt(digitalPinToInterrupt(3), PPG_period, FALLING); // adds an interrupt that happens when there is a falling edge in the PPG sensor's signal
}

void loop() {
  if (!reading) {
    auto a = Serial.readStringUntil('\n');
    if (a==String("START")) {
      reading=true;
      Serial.println("Starting...");
    }
  } 
  else {
    if (Serial.available()) {
      if (Serial.readStringUntil('\n')==String("STOP")) {
        reading=false;
        Serial.println("Stopping...");
      }
    }
    
    /************************Accelerometer ************************/
    float breath_period;
    int motion_flag = 0; // used for filtering out high motion
    float count_threshold = 0.005; // thereshold for breath counter
    float motion_threshold = g_value; // thereshold for motion (if above, data is rejected)
    float average = Acceleration();
    set_timer(1250);

    if((average > (Initial_AM_value+motion_threshold)) || (average < (Initial_AM_value-motion_threshold))) motion_flag = 1; // sets flag to 1 if there is too much motion
    else motion_flag = 0; // sets flag to 0 if the motion value is okay

    if(motion_flag==0){
      if(timer_flag==0){
        if((average > (Initial_AM_value+count_threshold)) || (average < (Initial_AM_value-count_threshold))){ // checks if the acceleration is beyond breathing threshold to register a breath
          breath_counter++;
          breath_period = breathing_period();
          timer_flag = 1; // activates a 1.5 second timer after registering a breathing motion to reduce multiple captures of one breath count
        }
        if(period_flag == 0 && trigger!=1){
        bpm_breathing = 60000 / breath_period; // dividing 60 seconds by breath period to get breathing frequency per minute
        trigger=1;
        //bpm_breathing+=saved;
        //bpm_breathing/=2;
        //Serial.print("BPM = ");Serial.print(bpm_breathing); Serial.print(" period = ");Serial.print(breath_period);Serial.print("\n");
        //saved=bpm_breathing;
        }
      }
    }

  /**************************TEMPERATURE****************************************/
    const int Pin1 = A6, Pin2 = A7; // Temp sensor pin wire connected to analog pin 6
    Temperatures(Pin1, Pin2, celsius1, celsius2);
  // float delta1 = celsius1 - Person_initial_1;//the change between the current temperature and the person's initial tempeperature values for both sensors
  // float delta2 = celsius2 - Person_initial_2;

    /*if (delta1 > x){   and also for delta2 just need to find the tresholds from that one research paper
      stressed_lvl=1;
    }
    if (delta1 > y){
      stressed_lvl=2;
    }
    if (delta1 > z){
      stressed_lvl=3;
    }*/

  /**************************PPG**************************************/
    int bpm_PPG = 60000.0 / Tavrg; 
  /********************************PRINTING*******************************************/
    time = millis(); // time in ,mili seconds since start of measurement
    String datastring = Data_line(time, bpm_breathing, bpm_PPG, celsius1, celsius2);
    Serial.println(datastring);
    Write_to_file("datalog_new.csv",datastring);
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

String Data_line(unsigned long int time, int bpm_breathing, int bpm_PPG, float celsius1, float celsius2){
  String dataString = "";
  dataString +=String(time);dataString += ";";
  dataString +=String(bpm_breathing);dataString += ";";
  dataString += String(bpm_PPG);dataString += ";";
  dataString +=String(celsius1);dataString += ";";
  dataString +=String(celsius2);
  return(dataString);
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

void printDirectory(File dir, int numTabs) {
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

void Check_existance(String file_name){
  if (SD.exists(file_name)) {
    Serial.print(file_name);Serial.print(" exists.\n");
  } else {
    Serial.print(file_name);Serial.print(" doesn't exist.\n");
  }
}

void Read_whole_file(String file_name){
  //open the file for reading:
  myFile = SD.open(file_name);
  if (myFile) {
    Serial.print(file_name);Serial.print(":\n");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.print("error opening ");Serial.print(file_name);
  }
}

void Write_to_file(String file_name, String data_string){
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open(file_name, FILE_WRITE);

   //if the file opened okay, write to it:
  if (myFile) {
   // Serial.print("Writing to " ); Serial.print(file_name);Serial.print("...\n");
    myFile.println(data_string); // this line writes in SD card
    // close the file:
    myFile.close();
   // Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.print("error opening ");Serial.print(file_name);
  }
}

void PPG_period() { // Interrupt routine
  if(first == 0) {
    t0 = millis(); // saves time value of first falling edge
    first = 1;
//digitalWrite(LED_BUILTIN, 1);
  }
  else {
    rawT = millis() - t0; // takes the time value of second falling edge and substracts the first = Period
   // Tavrg += rawT;
    //Tavrg /= 2;
    Tavrg=rawT;
  //digitalWrite(LED_BUILTIN, 0); 
    first = 0;
  }
}

void set_timer(unsigned long int millisec){
int t_start;
  if(timer_flag==1){
    t_start=millis();
    timer_flag=2;
  }
  if(timer_flag==2){
    if(t_start+millisec<millis()) timer_flag = 0;
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