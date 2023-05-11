#include <QTRSensors.h>

//Hello, the code below is for a robot that will follow the black line.

//These coefficients are needed for the PD controller
#define KP 0.029
#define KD 0.0005

#define set_speed 130    //minimum speed of the Motors
#define max_speed 160    //max. speed of the Motors

#define DEBUG 0 

// create motor, Lmotor, Rmotor, button, QTRSensors
uint8_t Lmotor_1 = 2;
uint8_t Lmotor_2 = 4;
uint8_t Lmotor_PWM = 5;  
uint8_t Rmotor_1 = 7;
uint8_t Rmotor_2 = 8;
uint8_t Rmotor_PWM = 9;
int runmotor = 6;
const int button = 3;

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup()
{
  Serial.begin(9600);
  Serial.println("hi");
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(10);

  delay(100);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Lmotor_1, OUTPUT);
  pinMode(Lmotor_2, OUTPUT);
  pinMode(Lmotor_PWM, OUTPUT);
  pinMode(Rmotor_1, OUTPUT);
  pinMode(Rmotor_2, OUTPUT);
  pinMode(Rmotor_PWM, OUTPUT);
  pinMode(runmotor, OUTPUT);
  pinMode(button, INPUT_PULLUP);

  delay(100);
}

int btnTimer = 0; 
bool flag_colib = false;              
bool flag_btn  = false;
bool flag_move = false;
bool flag_lap = false;

int lastError;
int stop_line;

bool is_lap;
uint8_t count_laps = 0;

void loop()
{

  int motorSpeed;        
  int leftMotorSpeed;
  int rightMotorSpeed;  

  bool btnState = !digitalRead(button); //Reading the inverse value of the button

  //start calibration
  /*When we just turned on the machine and pressed the button, 
  the calibration of the sensors will begin. You will need to move the car 
  from side to side so that all the sensors light up and 
  the car drives correctly along the line*/
  if(btnState && !flag_colib && millis() - btnTimer > 100) {      //colibration       

    btnTimer = millis();
    manual_calibration();
  
    Serial.println("colibration succsed");
    flag_colib = true;
  }
  
  /*When we click on the button, this function will run
  Thanks to this, we raise flag_move, and the car starts to go*/
  if (btnState && !flag_btn && flag_colib && millis() - btnTimer > 100)   //button pressed
    {          
      btnTimer = millis();
      delay(400);

      flag_move = !flag_move;
      flag_btn = !flag_btn;
      stop_line = 0;
      Serial.print("Button pressed\n");
    }
      else if(!btnState) flag_btn = false;
      
  //If the machine is standing and it is calibrated, the machine runs
  
  if (!flag_move && flag_colib ) {                                //Run car
    digitalWrite(runmotor,HIGH);
  
    int position = qtr.readLineBlack(sensorValues);               //We're going down the black line
    int error = position - 3500;                                  //Variable that looks up our location
  
    motorSpeed = KP * error + KD * (error - lastError);       //PD controller
    leftMotorSpeed = set_speed - motorSpeed;
    rightMotorSpeed = set_speed + motorSpeed;
  
    //Setting the speed of the motors
    leftMotorSpeed = constrain(leftMotorSpeed, 0, max_speed);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, max_speed);
    int lastError = error;

    /*If you have problems with rotation and you think 
    it's a sensor problem, you can uncomment this print_sensor_data() function, 
    and comment out set_motors(). Thanks to this, you can check the correct operation of your sensors.*/

    //print_sensor_data(sensorValues[SensorCount], position);
    set_motors(leftMotorSpeed, rightMotorSpeed); // set motor speeds using the two motor speed variables above
  
    //Next, count the number of circles
    if(sensorValues[0] > 800 && sensorValues[7] > 800){
      is_lap = true;
    }
    else is_lap = false;
    stop_line = laps(is_lap);

}

/*If the car is moving and it is calibrated OR 
it has passed the start line more than 3 times - it will stop*/
  if ((flag_move && flag_colib) || stop_line >= 3) {                              //Stop car
   
    Serial.println("stop motor");
    digitalWrite(runmotor,LOW);
  }
  
}


//The function that allows our motors to move
void set_motors(int motor1speed, int motor2speed)
{
analogWrite(Lmotor_PWM,motor1speed); //run motor
analogWrite(Rmotor_PWM,motor2speed);
digitalWrite(Lmotor_1,HIGH);
digitalWrite(Lmotor_2,LOW);
digitalWrite(Rmotor_1,LOW);
digitalWrite(Rmotor_2,HIGH);
}

int laps(bool is_lap)                              
{
  Serial.println(is_lap);

if(is_lap)
{ 
  if(!flag_lap) 
  {     // when do lap
    Serial.print("laps: ");
    Serial.println(count_laps);

   count_laps++;
    flag_lap = true;
  }
}
  else flag_lap = false;

  return count_laps;
}

void print_sensor_data(uint16_t sensorValues[SensorCount], int position){
  for (uint8_t i = 0; i < SensorCount; i++)   //print sensor data 
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);
  delay(250);
}

void manual_calibration() {
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate(start) 400 times to make calibration take about 10 seconds.
  Serial.println("start calibration");
  for (uint16_t i = 0; i < 170; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  delay(200);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  
  if (DEBUG){
    Serial.print("debug");
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
  }
}