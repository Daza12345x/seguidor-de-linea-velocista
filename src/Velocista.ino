#include <IngenieroMakerSeguidor16s.h>
#include <ESC.h>
barra16 Robot;
int p;
int pos;
#define MOTORLEFT_DIR_A  4  //   4      
#define MOTORLEFT_DIR_B  5  //   5      
#define MOTORLEFT_PWM    3  //   3    
#define MOTORRIGH_DIR_A  7  //   7      
#define MOTORRIGH_DIR_B  8  //   8     
#define MOTORRIGH_PWM    6  //   6      
#define BOTON            2 
#define GO               9
#define RDY              2
#define LED             13 
#define PIN_ESC 10
ESC EDF27(PIN_ESC);
int V_TURBINA = 1700;

float KP = 0.45;//0.25     0.18 2.5 la pocisiona en el centro 0.3 .25                               0.48       0.25                 0.23
float KD = 18;//5.56   2.56   16  velocidad ocn la que retorna en el medio 10  6.8              7.3           5.5                  60
float Ki = 0.002;    // acumulativo 

int Velmax = 230; // 245

int error1=0;
int error2=0;
int error3=0;
int error4=0;
int error5=0;
int error6=0;
//
int val1 ;

void setup() {
 //Serial.begin(9600);
  pinMode(MOTORRIGH_DIR_A ,OUTPUT);
  pinMode(MOTORRIGH_DIR_B ,OUTPUT);
  pinMode(MOTORRIGH_PWM   ,OUTPUT);
  pinMode(MOTORLEFT_DIR_A ,OUTPUT);
  pinMode(MOTORLEFT_DIR_B ,OUTPUT);
  pinMode(MOTORLEFT_PWM   ,OUTPUT);
  pinMode(LED             ,OUTPUT);
  pinMode(BOTON           ,INPUT);
  //digitalWrite(LED,0);
  while(true){
    int W=digitalRead(BOTON);
     digitalWrite(LED,0);
     delay(150);
     digitalWrite(LED,1);
     delay(150);
    if(W==1){
      digitalWrite(LED,0);
      delay(100);
      break;
    }
  }

 // WaitBoton();
  digitalWrite(LED_BUILTIN, HIGH);
  delay(3000);
  Robot.leer_blanco(); //PONER SENSORES EN BLANCO
  
  digitalWrite(LED_BUILTIN, LOW);
  
  while(true){
    int w=digitalRead(BOTON);
    if(w==1){
      digitalWrite(LED,0);
      delay(100);
      break;
    }
  }
  
  digitalWrite(LED_BUILTIN, HIGH);
  delay(3000);
  Robot.leer_negro();   //PONER SENSORES EN NEGRO
   digitalWrite(LED_BUILTIN, LOW);
  while(true){
    int x=digitalRead(BOTON);
    if(x==1){
      digitalWrite(LED,0);
      delay(100);
      break;
    }
  }

  
  digitalWrite(LED_BUILTIN, HIGH);
  Robot.Calcula_muestras(); //CALCULA TABLA DE MUESTRAS
 // GO_Boton();            // CONTROL DE ARRANQUE
  digitalWrite(LED_BUILTIN, LOW);
  delay (1000);
   while (true)
  {    
   int val = digitalRead(RDY); 

    int setpoint=750;
   int position = readSensor();
   int proportional = (int)position-750;
   //Serial.println(proportional);
   
   proportional = proportional/5;
   
   
 if (proportional < -20)
 {
     digitalWrite(LED, LOW);
  }
  else  if (proportional > 20 )
  {
  digitalWrite(LED, LOW);  
  }
  else  
  {
digitalWrite(LED, HIGH);  
  }
  
  float power_difference_fl = (proportional * 1.5);
  int power_difference = (int) power_difference_fl;
  
  int maximun_tolerance = (Velmax);
  
  if (power_difference > maximun_tolerance)
    power_difference = maximun_tolerance;
    
  if (power_difference < -(maximun_tolerance))
    power_difference = -(maximun_tolerance);
  
   motores(power_difference,  -power_difference);
   //Serial.println(proportional);
  // Serial.println(power_difference);
  // Serial.println(setpoint);
  val = digitalRead(RDY);
   if(val==1){digitalWrite(LED, LOW);
       //  motores(100,100); 
    delay(200);
    break;}
  }
     EDF27.init();//Inicializa el timer1
   //myservo.writeMicroseconds(2000); 
    delay(150); 
    EDF27.setSpeed(1000);//Apaga el motor
     delay(3500);
 // delay(1500);
    digitalWrite(LED,1);
    EDF27.setSpeed(1600);
    delay(1500);
    
  while(true){
    int w=digitalRead(GO);
     EDF27.setSpeed(1300);
    if(w==1){
      digitalWrite(LED,0);
      delay(100);
      break;
    }}
  EDF27.setSpeed(V_TURBINA); 
  
}

void setMotorLeft(int value)
{
  if ( value >= 0 )
  {
    digitalWrite(MOTORRIGH_DIR_A,HIGH);
    digitalWrite(MOTORRIGH_DIR_B,LOW);
  }
  else
  {
    digitalWrite(MOTORRIGH_DIR_A,LOW);
    digitalWrite(MOTORRIGH_DIR_B,HIGH);
    value *= -1;
  }
  analogWrite(MOTORRIGH_PWM,value);
}

void setMotorRigh(int value)
{  
  if ( value >= 0 )
  {
    digitalWrite(MOTORLEFT_DIR_A,HIGH);
    digitalWrite(MOTORLEFT_DIR_B,LOW);
  }
  else
  {
    digitalWrite(MOTORLEFT_DIR_A,LOW);
    digitalWrite(MOTORLEFT_DIR_B,HIGH);
    value *= -1;
  }    
  analogWrite(MOTORLEFT_PWM,value);
}

void motores(int left, int righ)
{
  //digitalWrite(STANDBY,HIGH);
  setMotorLeft(left);
  setMotorRigh(righ);
}

void freno(boolean left, boolean righ, int value)
{
//  digitalWrite(STANDBY,HIGH);

  if ( left )
  {
    digitalWrite(MOTORRIGH_DIR_A,HIGH);
    digitalWrite(MOTORRIGH_DIR_B,HIGH);
    analogWrite (MOTORRIGH_PWM, value);
  }

  if ( righ )
  {
    digitalWrite(MOTORLEFT_DIR_A,HIGH);
    digitalWrite(MOTORLEFT_DIR_B,HIGH);
    analogWrite (MOTORLEFT_PWM, value);
  }
}


unsigned int posicion = 0; 

int proporcional = 0;      
int integral = 0;          
int derivativo = 0;          
   
int diferencial = 0;   
int last_prop;         
int Target = 600;


void loop() {
 val1 = digitalRead(GO);
 if( val1 == 1){
PID();
frenos_contorno();

}
else{ 
  EDF27.setSpeed(1000);//Apaga el motor
  analogWrite (MOTORRIGH_PWM, 0);
   analogWrite (MOTORLEFT_PWM, 0);}
}
//pid

void PID(){
  posicion = readSensor();
  proporcional = ((int)posicion)-750;
  /*if ( proporcional <= -Target )
  {
    setMotorLeft(0);
    freno(false,true,255);
  }
  else if ( proporcional >= Target )
  {
    setMotorRigh(0);
    freno(true,false,255);
  }*/
  derivativo = proporcional - last_prop; 
  integral = error1+error2+error3+error4+error5+error6;
  last_prop = proporcional;

  error6=error5;
  error5=error4;  
  error4=error3;  
  error3=error2;
  error2=error1;
  error1=proporcional;

  int diferencial = ( proporcional * KP ) + ( derivativo * KD )+ (integral*Ki) ;
  
  if ( diferencial > Velmax ) diferencial = Velmax; 
  else if ( diferencial < -Velmax ) diferencial = -Velmax;

( diferencial < 0 ) ? motores(Velmax+diferencial, Velmax) : motores(Velmax, Velmax-diferencial);
}



int readSensor(void){
  long int p = 0;
   Robot.Leer_Sensores_ir(0); //1 linea blanca 
   p=Robot.proporcional();
  if(pos>=1450 && p<=-1){
    p=1600;
  }
  if(pos<=100. && p<=-1){
    p=0;
  }
  pos=p;
   return p;
}



void frenos_contorno(){
      
    if (posicion >= 1450) 
    {
        motores(253,-110); 
    }
    if (posicion <= 100)
    {
        motores(-110,253);
    }
}
void WaitBoton() {
  while (digitalRead(BOTON));
}
// BOTON DE INICIO
void GO_Boton() {
  while (!digitalRead(GO));
}
