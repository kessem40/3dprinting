
#include <LiquidCrystal_I2C.h>
#include <Wire.h> 
#include <Servo.h> 

//pins setups

#define ButtonPin 1      // define green button digital input
#define StopButton 2     // define red button digital input
#define RunOut 3         // define runout sensor digital input

const int stepPin1 = 8;  // define left stepper motor steps pin
const int dirPin1 = 9;   // define left stepper motor direction pin

const int stepPin3 = 10; // define scrolling stepper motor steps pin
const int dirPin3 = 11;  // define scrolling stepper motor direction pin

const int stepPin2 = 12; // define right stepper motor steps pin
const int dirPin2 = 13;  // define right stepper motor direction pin

#define PWM_pin  5        // make sure to connect to PWM PIN
#define ThermistorPin  A3 // define thermistor temp input
#define FanPin  4         // define fan on/off pin


//types setups

LiquidCrystal_I2C lcd(0x27, 20, 4);   // create lcd object
Servo myservo1;  // create servo object to control a servo
Servo myservo2;  // create servo object to control a servo


//BUTTON const
int buttonState = 0;         // variable for reading the pushbutton status
int RunOutState = 0;         // variable for reading the runout sensor status

// PID consts

const float Kp = 5; // 5
const float Kd = 0.01; // 0.01
float Current_temp ;// had to be read
float Desired_temp = 21; // Room temp
float pwm_signal;
float v_check=0;  // LPF first order 


void setup() 
{
Serial.begin(9600);
lcd.init();
lcd.backlight();	// Turn on the blacklight and print a message.
pinMode(stepPin1,OUTPUT); 
pinMode(dirPin1,OUTPUT);
pinMode(stepPin2,OUTPUT); 
pinMode(dirPin2,OUTPUT);
pinMode(stepPin3,OUTPUT); 
pinMode(dirPin3,OUTPUT);
pinMode(ButtonPin, INPUT_PULLUP);  // enable internal pull-up for button pin
pinMode(StopButton, INPUT_PULLUP);  // enable internal pull-up for STOP button pin
pinMode(RunOut, INPUT_PULLUP);  // enable internal pull-up for STOP button pin
attachInterrupt(0, StopButtonFunc, FALLING);
pinMode(PWM_pin,OUTPUT);// setup pwn heater pin
TCCR2B = TCCR2B & B11111000 | 0x03;    // pin 5 and 11 PWM frequency of 928.5 Hz
pinMode(FanPin,OUTPUT);// setup fan on/off pin as output

}

void loop()
{
   
  myservo1.attach(6);  // attaches the servo on pin 6 to the servo object
  myservo2.attach(7);  // attaches the servo on pin 7 to the servo object
  
  Wellcome();          // print on LCD screen "Hello, Please Insert The Filaments And Then Press The Green button."
  
  delay(500);          // pause for 500 miliseconds
  
  ServoMove1(40);      // right servo spin to an open position  
  ServoMove2(43);      // left servo spin to an open position
  
  Button();            // Indicator green button to proceed
  myservo1.detach();   // unattaches the servo on pin 6 to the servo object
  myservo2.detach();   // unattaches the servo on pin 7 to the servo object
  
  Button();
  StepperMoveLeft(3355);  //left stepper moves the filament to The beginning of the heating element
  StepperMoveRight(3440); //right stepper moves the filament to The beginning of the heating element
  
  HeatToTemp(190);     // heat-block is heating to 190 celsius
  
  lcd.clear();         // clear previous view from LCD 
  lcd.setCursor (0,0); // printing on top, left 
  lcd.print("Reached Soldering"); // first line
  lcd.setCursor (0,1); // printing on second line, left 
  lcd.print("Temp.");  // second line
  
  StepperMoveNegetiveDir(920,200); // filamets move toward each other inside the heater block - slow
  delay(3000);         // pause for 3 seconds
  
  StepperMoveNegetiveDir(30,200);  // extra push of filaments edges towards each other
  delay(5000);
  
  CoolToTemp(60); // sending pwm_signal=0 and starting the fan until temp gets to 60 celsius
  
  StepperMoveSameDirSPD(); //steppers moving the united filament,while third stepper rolling it.
  Button();           
  
}

///////////////////////////////////////////////////////////////FUNCTIONS////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Wellcome()
{
  lcd.clear();
  lcd.setCursor (0,0); //
  lcd.print("Hello, Please Insert");
  lcd.setCursor (0,1); // go to start of 2nd line
  lcd.print("The Filaments And");
  lcd.setCursor (0,2); // go to start of 2nd line
  lcd.print("Then Press The Green");
  lcd.setCursor (0,3); // go to start of 2nd line
  lcd.print("button.");
  Button();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void StopButtonFunc() // interuppet for stop all
{
  int x=0;
  if(x=0){
  Serial.println("STOP");
  }
}                                 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Button() // loop that indicate the status of the button
{
 buttonState = digitalRead(ButtonPin);
  while(buttonState == HIGH){
    buttonState = digitalRead(ButtonPin);
  }
}              
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ServoMove1(int dgr) // func that gets a degree for the servo movement 
                         //and afterwards move it to the needed position
{
    myservo1.write(dgr);  // tell servo to go to position in variable 'dgr'
    lcd.clear();
    lcd.setCursor (0,0); 
    lcd.print("Servo Stopper Is");
    lcd.setCursor (0,1); 
    lcd.print("Open.");
}    
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ServoMove2(int dgr) // func that gets a degree for the servo movement 
                         //and afterwards move it to the needed position
{
    myservo2.write(dgr);  // tell servo to go to position in variable 'dgr'
}                
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void StepperMoveNegetiveDir(int stp,float temp3) // func that gets amount of
                                                 //steps and move both steppers 
{
    Desired_temp=temp3;
    lcd.clear();
    lcd.setCursor (0,0); //
    lcd.print("Stepper Motors Is");
    lcd.setCursor (0,1); //
    lcd.print("Moving In Negetive");
    lcd.setCursor (0,2); //
    lcd.print("Directions.");
    digitalWrite(dirPin1,LOW); // Enables the motor to move in a particular direction
    digitalWrite(dirPin2,LOW); // Enables the motor to move in a particular direction
    for(int x = 0; x < stp; x++) {
    digitalWrite(stepPin1,HIGH); 
    digitalWrite(stepPin2,HIGH); 
    delayMicroseconds(5000); 
    digitalWrite(stepPin1,LOW); 
    digitalWrite(stepPin2,LOW); 
    delayMicroseconds(5000);
  }
    lcd.clear();
    lcd.setCursor (0,0); //
    lcd.print("Reached The Touching");
    lcd.setCursor (0,1); //
    lcd.print("Point.");
}        
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void StepperMoveSameDirSPD() 
{
    digitalWrite(FanPin,HIGH);
    lcd.clear();
    lcd.setCursor (0,0); //
    lcd.print("Stepper Motors Is");
    lcd.setCursor (0,1); //
    lcd.print("Moving In Same");
    lcd.setCursor (0,2); //
    lcd.print("Direction.");
    digitalWrite(dirPin1,LOW); // Enables the motor to move in a particular direction
    digitalWrite(dirPin2,HIGH); // Enables the motor to move in a particular direction
    digitalWrite(dirPin3,HIGH);
    RunOutState = digitalRead(RunOut);
  while(RunOutState == HIGH){
    RunOutState = digitalRead(RunOut);
    digitalWrite(stepPin1,HIGH); 
    digitalWrite(stepPin2,HIGH); 
    delayMicroseconds(500); 
    digitalWrite(stepPin1,LOW); 
    digitalWrite(stepPin2,LOW); 
    delayMicroseconds(500);
   }
  for(int x=0 ; x<8000 ; x++){
    digitalWrite(stepPin1,HIGH); 
    digitalWrite(stepPin2,HIGH);
    digitalWrite(stepPin3,HIGH); 
    delayMicroseconds(500); 
    digitalWrite(stepPin1,LOW); 
    digitalWrite(stepPin2,LOW); 
    digitalWrite(stepPin3,LOW);
    delayMicroseconds(500);  
  }

}
        
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void StepperMoveLeft(int stp) // func that gets a degree for the servo movement and afterwards move it to the needed position
{
    lcd.clear();
    lcd.setCursor (0,0); //
    lcd.print("Left Stepper Motor");
    lcd.setCursor (0,1); //
    lcd.print("Is Moving.");
    digitalWrite(dirPin1,LOW); // Enables the motor to move in a particular direction
  for(int x = 0; x < stp; x++) {
    digitalWrite(stepPin1,HIGH); 
    delayMicroseconds(500); 
    digitalWrite(stepPin1,LOW); 
    delayMicroseconds(500); 
  }
    

}   

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void StepperMoveRight(int stp) // func that gets a degree for the servo movement and afterwards move it to the needed position
{
   lcd.clear();
    lcd.setCursor (0,0); //
    lcd.print("Right Stepper Motor");
    lcd.setCursor (0,1); //
    lcd.print("Is Moving.");
    digitalWrite(dirPin2,LOW); // Enables the motor to move in a particular direction
  for(int x = 0; x < stp; x++) {
    digitalWrite(stepPin2,HIGH); 
    delayMicroseconds(500); 
    digitalWrite(stepPin2,LOW); 
    delayMicroseconds(500); 
  }
   

}   
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////   
void HeatToTemp(float temp1)                     
{
  Desired_temp=temp1;
  lcd.clear();
  lcd.setCursor (0,0); // print on LCD in 1st line
  lcd.print("Heating Up.");
  Read_temprature();           // get current temp
  PID(Kp, Kd);             //controlling heater temp func
  Plotter();                   // plotter for programmer 
  for(int x = 0; x < 1300; x++)
  {
    Read_temprature();     // get current temp
    PID(Kp, Kd );       //controlling heater temp func
    Plotter();  
    delay(20);
         
  } 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////   
void CoolToTemp(float temp2)                     
{
  digitalWrite(FanPin,HIGH);
  Desired_temp=temp2;
  lcd.clear();
  lcd.setCursor (0,0); //
  lcd.print("Cooling Down.");
  while(Desired_temp<abs(Current_temp)){
  analogWrite(PWM_pin,0);
  Read_temprature();                                // get current temp
  Plotter();                                        // plotter for programmer 
  delay(20);
  }

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Read_temprature()               // Function to read the temperature
{
  if(v_check==0)             //get initial voltage value from thermistor
  {
    v_check=analogRead(ThermistorPin);   
  }
  // Define contsts for Thermopil
  static const float R1 = 10000; //known resistor value
  static float logR2, R2, T;
  static const float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
  static float Vo;
  static float prev_temp = 0; 
  
  Vo = analogRead(ThermistorPin); // read analog  data
  if(abs(v_check-Vo)<50){  //Filter extreme values 
  R2 = R1 * (1023.0 / (float)Vo - 1.0); //resistance of thermistor
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2)); //Steinhart-Hart equation
  T = T - 273.15; // MOVING TO Celsius
  Current_temp = T;
  v_check=Vo;
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Plotter()
{
  Serial.print("Current_Temperature:"); 
  Serial.print(Current_temp);
  Serial.print(" ");
  Serial.print("Desired_Temperature:"); 
  Serial.print(Desired_temp);
  Serial.print(" ");
  Serial.print("PWM_signal:"); 
  Serial.println(pwm_signal);
  
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PID(float KP, float KD) // PID function
{
  /// Functions vars
  static float error=0;                //Initializing variables
  static float last_error=0;           //Initializing variables
  static float pid_p;
  static float pid_d;
  static float Kp_temp = KP ; //5
  static float Kd_temp = KD ; //0.01
  static float elapsed_time = 0;
  static float last_time = 0;           //Initializing variables
  static float sum_error = 0;           //Initializing variables
  elapsed_time = millis()-last_time;    //Calulete Delta time
  last_time = elapsed_time + last_time;
  last_error = error;
  error = Desired_temp - Current_temp;  // Get error
  pid_p = Kp_temp*error;                                   //calculete p
  pid_d = Kd_temp*(error-last_error)/(elapsed_time/1000);  // calcuelte d
  pwm_signal = pid_p+pid_d;             //PD
  pwm_signal = pwm_signal > 0 ? min(200,pwm_signal): 0;    // if pwm>0 take the minimum value , else pwm=0
  analogWrite(PWM_pin, pwm_signal);                        // send pwm signal to heater block
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////   
