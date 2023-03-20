#include <ESP32Servo.h>
#include <ESP32Tone.h>
#include <ESP32PWM.h>

Servo servo_rodillaR;
Servo servo_caderaR;
Servo servo_tobilloR;

Servo servo_rodillaL;
Servo servo_caderaL;
Servo servo_tobilloL;

#define rodillaR_ini 90
#define caderaR_ini 90 
#define tobilloR_ini 100 

#define rodillaL_ini 90
#define caderaL_ini 90
#define tobilloL_ini 85

double rodillaR_ = rodillaR_ini; // angulo rodilla derecha
double caderaR_ = caderaR_ini; // angulo cadera derecha
double tobilloR_ = tobilloR_ini; // angulo tobillo derecho

double rodillaL_ = rodillaL_ini; // angulo rodilla izquierda
double caderaL_ = caderaL_ini; // angulo cadera izquierda
double tobilloL_ = tobilloL_ini; // angulo tobillo izquierdo

const double Long_Pierna = 140;

#define Long_Cadera 65
#define Long_Rodilla 75
#define PASO 30
#define SwingingChange 10
#define SupportChange 5

#define DERECHA 0
#define IZQUIERDA 1
int PIERNA = IZQUIERDA;

void Inverse_kinematics(double x,double y,double l1,double l2);
void swinging(); //balanceo antes de dar un paso, para cambiar el peso de pierna
void support(); // apoyo para finalizar la zancada
void calculate_step(double dist); //basicamente teorema de Pitagoras
void error_func();
void Two_Servo_Move_Lineal(Servo &servo1, Servo &servo2, double &init1, double &init2, double fin1, double fin2 , double inc_time, double step_time);
void One_Servo_Move_Lineal(Servo &servo1, double &init1, double fin1, double inc_time, double step_time);

void setup() 
{
  servo_rodillaR.setPeriodHertz(50);// Standard 50hz servo
  servo_caderaR.setPeriodHertz(50);// Standard 50hz servo
  servo_tobilloR.setPeriodHertz(50);// Standard 50hz servo

  servo_rodillaL.setPeriodHertz(50);// Standard 50hz servo
  servo_caderaL.setPeriodHertz(50);// Standard 50hz servo
  servo_tobilloL.setPeriodHertz(50);// Standard 50hz servo
  
  servo_rodillaR.attach(25, 400, 2500);
  servo_caderaR.attach(26, 400, 2500);    
  servo_tobilloR.attach(27, 400, 2500); 
  servo_rodillaL.attach(32, 400, 2500);
  servo_caderaL.attach(33, 400, 2500);    
  servo_tobilloL.attach(12, 400, 2500);
  
  servo_rodillaR.write(rodillaR_);
  servo_caderaR.write(caderaR_); 
  servo_tobilloR.write(tobilloR_);
  servo_rodillaL.write(rodillaL_);
  servo_caderaL.write(caderaL_); 
  servo_tobilloL.write(tobilloL_);
  
  Serial.begin(115200); //Para depurar

  delay(5000);

}

void loop() 
{
  swinging(); // BALANCEO INICIAL
  calculate_step(PASO); // CALCULO LA IK DEL PASO Y LA EJECUTO
  support(); // APOYO FINAL
  Serial.println("FIN PASO");
  servo_rodillaR.write(rodillaR_ini);
  servo_caderaR.write(caderaR_ini); 
  servo_tobilloR.write(tobilloR_ini);
  servo_rodillaL.write(rodillaL_ini);
  servo_caderaL.write(caderaL_ini); 
  servo_tobilloL.write(tobilloL_ini);
  delay(20000);
  //support(); // APOYO FINAL
}
/*
x : posición X del centro del pien en dos dimensiones
y : posición X del centro del pien en dos dimensiones (cte)
l1 : longitud entre la cadera y la rodilla
l2 : longitud entre la rodilla y el pie 
*/
void Inverse_kinematics(double x,double y,double l1,double l2)
{
  double val1;
  double val2;

  // aplico ik para calcular los ángulos:
  
  val1 = (pow(x,2.0)+ pow(y,2.0)- pow(l1, 2.0)- pow(l2,2.0))/(2*l1*l2);
  if (val1 >= 1) val1 = 1; //por errores de redondeo a veces me da > 1 y falla la función acos
  
  val2 = (y*(l1+l2*cos(0))-x*l2*sin(0))/(x*(l1+l2*cos(0))+y*l2*sin(0));
  
  //PARA DEPURAR
  Serial.print("val1 ");
  Serial.println(val1);
  Serial.print("val2 ");
  Serial.println(val2);

  val1 = acos(val1); // valores que se pasen a acos deben estar entre -1 y 1
  val2 = atan(val2);
  
  //PARA DEPURAR
  Serial.print("val1 ");
  Serial.println(val1);
  Serial.print("val2 ");
  Serial.println(val2);
  
  //paso ambos valores de radianes a grados:
  /*val1 = (180/3.1416)*val1 + 90 - 5; //estos 5 grados que resto es para que flexione y sea más natural
  val2 = (180/3.1416)*val2 + 90; // el servo se inicializa a 90 grados luego su posición lo calculo respecto a este*/
  val1 = (180/3.1416)*val1 + 5; //estos 5 grados que resto es para que flexione y sea más natural
  val2 = (180/3.1416)*val2; // el servo se inicializa a 90 grados luego su posición lo calculo respecto a este
  //PARA DEPURAR
  Serial.print("val1 ");
  Serial.println(val1);
  Serial.print("val2 ");
  Serial.println(val2);
  
  if(val1 > 90 || val2 > 90)
  {
    error_func(); // en caso de mal calculo me salgo y no fuerzo los motores
  }
  else // si todo bien muevo los motores
  {
    if(PIERNA == DERECHA)
    {
      /*rodillaR_= val1;
      caderaR_ = val2;
      tobilloR_ = 90.0;
      
      rodillaL_= 90.0; //estiro la pierna apoyada*/
      
      //caderaL_ = caderaL_ini;
      tobilloR_ = tobilloR_ini;
      Two_Servo_Move_Lineal(servo_rodillaR, servo_caderaR, rodillaR_, caderaR_, (rodillaR_-val1), (caderaR_+val2) , 20000.0, 500000.0);
      //Two_Servo_Move_Lineal(servo_rodillaR, servo_caderaR, rodillaR_, caderaR_, val1, val2 , 20000.0, 500000.0); // de estirado al valor del ik
      Two_Servo_Move_Lineal(servo_caderaL, servo_rodillaL, caderaL_, rodillaL_, caderaL_ini, rodillaL_ini , 20000.0, 500000.0); //paso del valor anterior a estirado
      servo_tobilloR.write(tobilloR_);
      //servo_caderaL.write(caderaL_);   //estiro cadera
    }
    else if(PIERNA == IZQUIERDA)
    {
      /*rodillaL_= val1;
      caderaL_ = val2;
      tobilloL_ = 85.0;
      
      rodillaR_= 90.0; //estiro la pierna apoyada*/
      
      //caderaR_ = caderaR_ini;
      tobilloL_ = tobilloL_ini;
      Two_Servo_Move_Lineal(servo_rodillaL, servo_caderaL, rodillaL_, caderaL_, (rodillaL_+val1), (caderaL_-val2) , 20000.0, 200000.0);
      //Two_Servo_Move_Lineal(servo_rodillaL, servo_caderaL, rodillaL_, caderaL_, val1, val2 , 20000.0, 500000.0); // de estirado al valor del ik
      Two_Servo_Move_Lineal(servo_caderaR, servo_rodillaR, caderaR_, rodillaR_, caderaR_ini, rodillaR_ini , 20000.0, 200000.0); //paso del valor anterior a estirado
      //Two_Servo_Move_Lineal(servo_tobilloL, servo_rodillaR, tobilloL_, rodillaR_, tobilloL_ini, rodillaR_ini , 5000.0, 500000.0); //paso del valor anterior a estirado
      servo_tobilloL.write(tobilloL_);
      //servo_caderaL.write(caderaR_);   //estiro cadera
    }
    else
    {
      error_func();
    }
    delay(100);
  }
}

void swinging() //balanceo antes de dar un paso, para cambiar el peso de pierna
{
  if(PIERNA == DERECHA)
  {
    One_Servo_Move_Lineal(servo_tobilloR, tobilloR_, (tobilloR_+3*SwingingChange), 20000.0, 500000.0);
    delay(1000);
    One_Servo_Move_Lineal(servo_tobilloL, tobilloL_, (tobilloL_-SwingingChange), 20000.0, 500000.0);
  }
  else if(PIERNA == IZQUIERDA)
  {
    /*tobilloR_ -= SwingingChange;
    tobilloL_ -= SwingingChange;*/
    One_Servo_Move_Lineal(servo_tobilloL, tobilloL_, (tobilloR_-4*SwingingChange), 20000.0, 400000.0);
    //delay(1000);
    One_Servo_Move_Lineal(servo_tobilloR, tobilloR_, (tobilloR_+2*SwingingChange), 20000.0, 500000.0);
  }
  else
  {
    error_func();
  }
  
  delay(100);
}

void support() //inclino hacia adelante para apoyar la pierna que ha ejecutado el paso
{
  if(PIERNA == DERECHA)
  {
    /*caderaL_ += SupportChange;
    rodillaL_ += SupportChange;*/
    tobilloL_ = tobilloL_ini;
    //
    Two_Servo_Move_Lineal(servo_rodillaL, servo_caderaL, rodillaL_, caderaL_, (rodillaL_+SupportChange), (caderaL_+SupportChange) , 5000.0, 500000.0); // Inclino adelante
    //servo_rodillaL.write(tobilloL_);
    //
    PIERNA = IZQUIERDA; // cambio estado
  }
  else if(PIERNA == IZQUIERDA)
  {
    /*caderaR_ += SupportChange;
    rodillaR_ += SupportChange;*/
    //tobilloR_ = tobilloR_ini;
    //Two_Servo_Move_Lineal(servo_rodillaR, servo_caderaR, rodillaR_, caderaR_, (rodillaR_+SupportChange), (caderaR_+SupportChange) , 5000.0, 500000.0); // Inclino adelante
    One_Servo_Move_Lineal(servo_caderaR, caderaR_, (rodillaR_+SupportChange), 20000.0, 200000.0);
    One_Servo_Move_Lineal(servo_tobilloR, tobilloR_, (tobilloR_ini), 20000.0, 1000000.0);
    //servo_rodillaR.write(tobilloR_); 
    PIERNA = DERECHA; // cambio estado
  }
  else
  {
    error_func();
  }
  delay(100);
}

void calculate_step(double dist) //basicamente la regla de los catetos y la hipotenusa
{
  //dist = y
  double x;
  x = sqrt(pow(Long_Pierna,2.0)-pow(dist,2.0));
  Serial.print("x ");
  Serial.println(x);
  
  Inverse_kinematics(x, dist ,Long_Cadera, Long_Rodilla);
}

void error_func()
{
  // setea la posición inicial y corta el loop:
  servo_rodillaR.write(90);
  servo_caderaR.write(90); 
  servo_tobilloR.write(90);
  servo_rodillaL.write(90);
  servo_caderaL.write(90); 
  servo_tobilloL.write(90);
  
  exit(0);
}
void Two_Servo_Move_Lineal(Servo &servo1, Servo &servo2, double &init1, double &init2, double fin1, double fin2 , double inc_time, double step_time)
{
  double inc1, inc2;
  unsigned long previous;
  unsigned out = 0;
  inc1 = ((fin1 - init1)*inc_time)/step_time;
  if(inc1 == 0)
  {
    out++;
  }
  inc2 = ((fin2 - init2)*inc_time)/step_time;
  if(inc2 == 0)
  {
    out++;
  }
  
  previous = micros();
  
  while( out != 2 )
  {
    init1 += inc1;
    init2 += inc2;
    
    servo1.write(init1);
    servo2.write(init2);
    
    while( micros() < (previous + inc_time) ) // espera activa
    {
    }
    
    if((abs(fin1 - init1) < abs(inc1)))
    {
      inc1 = 0;
      out++;
    }
    if((abs(fin2 - init2) < abs(inc2)))
    {
      inc2 = 0;
      out++;
    }
    previous = micros();
  }
  delay(100); // parada entre cada movimiento de pareja de servos, para evitar sobrecarga de la batería
}
void One_Servo_Move_Lineal(Servo &servo1, double &init1, double fin1, double inc_time, double step_time)
{
  double inc1;
  unsigned long previous;
  unsigned out = 0;
  inc1 = ((fin1 - init1)*inc_time)/step_time;
  if(inc1 == 0)
  {
    out++;
  }
  
  previous = micros();
  
  while( out != 1 )
  {
    init1 += inc1;
    
    servo1.write(init1);
    
    while( micros() < (previous + inc_time) ) // espera activa
    {
    }
    
    if((abs(fin1 - init1) < abs(inc1)))
    {
      inc1 = 0;
      out++;
    }
    previous = micros();
  }
  delay(200); // parada entre cada movimiento de pareja de servos, para evitar sobrecarga de la batería
}
