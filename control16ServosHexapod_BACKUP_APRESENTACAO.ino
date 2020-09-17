#include <Wire.h>
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  150 
#define SERVOMAX  600

//Portas extras para o controle dos dois servos que nao estao conectados ao modulo I2C
int porta_servo_1 = 8;
int porta_servo_2 = 9;
Servo servo_1;
Servo servo_2;
//////////////////////////////////#########################################################################################################
//////////////////////////////////#########################################################################################################
float X_hexa = 0;
float Y_hexa = 20;
float Z_hexa = -18;

float p1_x = X_hexa;
float p1_y = Y_hexa;
float p1_z = Z_hexa;

    float p1_a1 = 0;
    float p1_a2 = 0;
    float p1_a3 = 0;

float p2_x = X_hexa;
float p2_y = Y_hexa;
float p2_z = Z_hexa;

    float p2_a1 = 0;
    float p2_a2 = 0;
    float p2_a3 = 0;

float p3_x = X_hexa;
float p3_y = Y_hexa;
float p3_z = Z_hexa;

    float p3_a1 = 0;
    float p3_a2 = 0;
    float p3_a3 = 0;

float p4_x = X_hexa;
float p4_y = Y_hexa;
float p4_z = Z_hexa;

    float p4_a1 = 0;
    float p4_a2 = 0;
    float p4_a3 = 0;

float p5_x = X_hexa;
float p5_y = Y_hexa;
float p5_z = Z_hexa;

    float p5_a1 = 0;
    float p5_a2 = 0;
    float p5_a3 = 0;

float p6_x = X_hexa;
float p6_y = Y_hexa;
float p6_z = Z_hexa;

    float p6_a1 = 0;
    float p6_a2 = 0;
    float p6_a3 = 0;
    
int r = 5;

float rad2deg(float rad){
    float pi = 3.14159265359;
    return (180 / pi) * rad;
}


float deg2rad(float deg){
    float pi = 3.14159265359;
    return pi * deg /180;
}

void cinematic(float x, float y, float z, float *a01, float *a02, float *a03){
      float L1 = 8.5;
      float L2 = 10;
      float L3 = 19.5;
      //float r=5;

      float t1 = atan2(y,x);
      
      float x_ = sqrt(x*x+y*y) - L1;
     //OK Serial.print("X: ");Serial.println(x_,5);
  
      float t3 = -acos((x_*x_+z*z - L2*L2 - L3*L3)/(2*L2*L3));
      
      float t2 = atan((z*(L2+L3*cos(t3)) - (x_*L3*sin(t3)))  /  (x_*(L2+L3*cos(t3)) + (z*L3*sin(t3))));

      //Serial.print(t1,5);Serial.print(" ");Serial.print(t2,5);Serial.print(" ");Serial.println(t3,5);
     //  Serial.println("AQUI ----=====----====----");
       //Serial.print("T1: ");Serial.print(t1,4);Serial.print(" T2: ");Serial.print(t2,4);Serial.print(" T3: ");Serial.println(t3,4);
       //Serial.print("T1: ");Serial.print(t1,4);Serial.print(" X: ");Serial.print(x,4);Serial.print(" Y: ");Serial.println(y,4);
      float hexa1 = 180 - rad2deg(t1);
      float hexa2 = 75 - rad2deg(t2);
      float hexa3 = 199 + rad2deg(t3);
      
     // Serial.print(hexa1);Serial.print(" ");Serial.print(hexa2);Serial.print(" ");Serial.println(hexa3);
      float aux01 = round(hexa1);
      float aux02 = round(hexa2);
      float aux03 = round(hexa3);
      
      *a01 = aux01;
      *a02 = aux02;
      *a03 = aux03;
}

void calcRelPos(float *xn, float *yn, float *zn, int p, int ang){
  *xn = *xn + r*sin(ang - 1.0472*(p - 1));  //1.0472 = 60 graus
  *yn = *yn + r*cos(ang - 1.0472*(p - 1));
  *zn = *zn + r; 
  //Serial.println("X: ");Serial.print(*xn);Serial.print(" Y: ");Serial.print(*yn);Serial.print(" Z: ");Serial.println(*zn);
      
}
//////////////////////////////////#########################################################################################################
//////////////////////////////////#########################################################################################################
int auxAngle = 90;

int angleServ[18];
int angleServ_pass[18];


int auxWalk = false;
int auxWalk2 = false;

int delayT = 400;

int auxCont = 180;
int auxCont2 = 0;

void setup() {
  
  angleServ[0] = 90;
  angleServ[1] = 90;
  angleServ[2] = 90;
  angleServ[3] = 90;
  angleServ[4] = 90;
  angleServ[5] = 90;
  angleServ[6] = 0;
  angleServ[7] = 0;
  angleServ[8] = 0;
  angleServ[9] = 0;
  angleServ[10] = 0;
  angleServ[11] = 0;
  angleServ[12] = 180;
  angleServ[13] = 180;
  angleServ[14] = 180;
  angleServ[15] = 180;
  angleServ[16] = 180;
  angleServ[17] = 180;
  
  angleServ_pass[0] = 90;
  angleServ_pass[1] = 90;
  angleServ_pass[2] = 90;
  angleServ_pass[3] = 90;
  angleServ_pass[4] = 90;
  angleServ_pass[5] = 90;
  angleServ_pass[6] = 0;
  angleServ_pass[7] = 0;
  angleServ_pass[8] = 0;
  angleServ_pass[9] = 0;
  angleServ_pass[10] = 0;
  angleServ_pass[11] = 0;
  angleServ_pass[12] = 180;
  angleServ_pass[13] = 180;
  angleServ_pass[14] = 180;
  angleServ_pass[15] = 180;
  angleServ_pass[16] = 180;
  angleServ_pass[17] = 180;
  
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  servo_1.attach(porta_servo_1);
  servo_2.attach(porta_servo_2);
  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  yield();
}

void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 70;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

int convert(int angle){
  if(angle > 180){
    angle = 180;
  }
  if(angle < 0){
    angle = 0;
  }

  float aux = 150 + 2.5*angle;
  
  if(aux < 150){
    aux = 150;
  }
  if(aux > 600){
    aux = 600;
  }
  
  return (int)aux;  
}

void setAngleServo(int servoNum, int newAngle){
    if(angleServ_pass[servoNum] < newAngle){
      for(int i = angleServ_pass[servoNum]; i < newAngle; i += 5){
        //delay(5);
        if(servoNum < 16){
           pwm.setPWM(servoNum, 0, convert(i));
        }else{
          if(servoNum == 16){
            servo_1.write(i); 
          }
          if(servoNum == 17){
            servo_2.write(i); 
          }
        }
      }
    }else if(angleServ_pass[servoNum] > newAngle){
      for(int i = angleServ_pass[servoNum]; i > newAngle; i -= 5){
        //delay(5);
        if(servoNum < 16){
           pwm.setPWM(servoNum, 0, convert(i));
        }else{
          if(servoNum == 16){
            servo_1.write(i); 
          }
          if(servoNum == 17){
            servo_2.write(i); 
          }
        }   
      }
    }

    angleServ_pass[servoNum] = newAngle;
}

void setAllServosAngle(int servos[], int angles[], int numAux){
    int range = 0;
    for(int i = 0; i < numAux; i++){
      if(angleServ_pass[servos[i]] - angles[i] < 0){
        if(-(angleServ_pass[servos[i]] - angles[i]) > range){
          range = - (angleServ_pass[servos[i]] - angles[i]); 
        }
      }else{
        if((angleServ_pass[servos[i]] - angles[i]) > range){
          range = (angleServ_pass[servos[i]] - angles[i]); 
        }
      }
    }
    
    range ++;
    
    for(int i = 0; i < range; i++){
      for(int j = 0; j < numAux; j++){
        if(angleServ_pass[servos[j]] < angles[j]){
          angleServ_pass[servos[j]] ++;  
        }else if(angleServ_pass[servos[j]] > angles[j]){
          angleServ_pass[servos[j]] --;
        }

        delay(5);
          if(servos[j] < 16){
             pwm.setPWM(servos[j], 0, convert(angleServ_pass[servos[j]]));
          }else{
            if(servos[j] == 16){
              servo_1.write(angleServ_pass[servos[j]]); 
            }
            if(servos[j] == 17){
              servo_2.write(angleServ_pass[servos[j]] ); 
            }
          }
      }
    }
}

void showAngles(){
  Serial.println("-------------SERVOS----------------");
  Serial.print("Servo 0: "); Serial.println(angleServ_pass[0]);
  Serial.print("Servo 1: "); Serial.println(angleServ_pass[1]);
  Serial.print("Servo 2: "); Serial.println(angleServ_pass[2]);
  Serial.print("Servo 3: "); Serial.println(angleServ_pass[3]);
  Serial.print("Servo 4: "); Serial.println(angleServ_pass[4]);
  Serial.print("Servo 5: "); Serial.println(angleServ_pass[5]);
  Serial.print("Servo 6: "); Serial.println(angleServ_pass[6]);
  Serial.print("Servo 7: "); Serial.println(angleServ_pass[7]);
  Serial.print("Servo 8: "); Serial.println(angleServ_pass[8]);
  Serial.print("Servo 9: "); Serial.println(angleServ_pass[9]);
  Serial.print("Servo 10: "); Serial.println(angleServ_pass[10]);
  Serial.print("Servo 11: "); Serial.println(angleServ_pass[11]);
  Serial.print("Servo 12: "); Serial.println(angleServ_pass[12]);
  Serial.print("Servo 13: "); Serial.println(angleServ_pass[13]);
  Serial.print("Servo 14: "); Serial.println(angleServ_pass[14]);
  Serial.print("Servo 15: "); Serial.println(angleServ_pass[15]);
  Serial.print("Servo 16: "); Serial.println(angleServ_pass[16]);
  Serial.print("Servo 17: "); Serial.println(angleServ_pass[17]);
  Serial.print("-----------------------------------");
}

  void moveCinem(){
        if((auxCont2 < 160) and (auxCont2 > -1)){
          int aux = 6;
          int auxServ[aux] = {6,7,8,9,10,11};
          int auxAngle[aux] = {auxCont2,auxCont2,auxCont2,auxCont2,auxCont2,auxCont2};
          setAllServosAngle(auxServ, auxAngle, aux);
          showAngles();
        }else{
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
        }

        if((auxCont < 181) and (auxCont > 20)){
          int aux = 6;
          int auxServ[aux] = {12,13,14,15,16,17};
          int auxAngle[aux] = {auxCont,auxCont,auxCont,auxCont,auxCont,auxCont};
          setAllServosAngle(auxServ, auxAngle, aux);
          showAngles();
        }else{
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
        }

        
}

void movimentoYmais(int m1, int m2, int m3){
    int numVals = 30;
        int cinemAngleT1[numVals] = {90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90};
        int cinemAngleT2[numVals] = {64, 62, 60, 58, 55, 53, 50, 48, 46, 43, 44, 45, 45, 46, 47, 48, 49, 51, 52, 53, 55, 58, 60, 62, 64, 66, 68, 70, 73, 75};
        int cinemAngleT3[numVals] = {103, 101, 99, 97, 95, 93, 92, 90, 88, 86, 88, 91, 93, 96, 98, 101, 103, 106, 109, 112, 114, 116, 117, 119, 121, 123, 125, 127, 129, 132};
        
        for(int i = 0; i < numVals; i++){
          int auxContT1 = cinemAngleT1[i];
          int auxContT2 = cinemAngleT2[i];
          int auxContT3 = cinemAngleT3[i];

          if(auxContT1 < 0){
            auxContT1 = 0;
          }
          if(auxContT2 < 0){
            auxContT2 = 0;
          }
          if(auxContT3 < 0){
            auxContT3 = 0;
          }

          setAngleServo(m1, auxContT1);
          setAngleServo(m2, auxContT2);
          setAngleServo(m3, auxContT3);

          delay(10);
        }
}

void movimentoRetornoYmais(int m1, int m2, int m3){
        int numVals = 10;
        int cinemAngleT1[numVals] = {90, 90, 90, 90, 90, 90, 90, 90, 90, 90};
        int cinemAngleT2[numVals] = {74, 72, 71, 70, 69, 69, 68, 68, 67, 67};
        int cinemAngleT3[numVals] = {128, 125, 122, 120, 117, 114, 112, 110, 108, 105};
        
        for(int i = 0; i < numVals; i++){
          int auxContT1 = cinemAngleT1[i];
          int auxContT2 = cinemAngleT2[i];
          int auxContT3 = cinemAngleT3[i];

          if(auxContT1 < 0){
            auxContT1 = 0;
          }
          if(auxContT2 < 0){
            auxContT2 = 0;
          }
          if(auxContT3 < 0){
            auxContT3 = 0;
          }

          setAngleServo(m1, auxContT1);
          setAngleServo(m2, auxContT2);
          setAngleServo(m3, auxContT3);

          delay(10);
        }
}


void movimentoYmenos(int m1, int m2, int m3){
  int numVals = 30;
        int cinemAngleT1[numVals] = {90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90};
        int cinemAngleT2[numVals] = {64, 62, 60, 58, 55, 53, 50, 48, 46, 43, 43, 42, 42, 42, 42, 42, 42, 42, 43, 43, 46, 49, 52, 54, 57, 60, 62, 65, 67, 69};
        int cinemAngleT3[numVals] = {103, 101, 99, 97, 95, 93, 92, 90, 88, 86, 84, 82, 80, 78, 76, 74, 73, 71, 70, 68, 71, 73, 75, 77, 79, 81, 84, 86, 88, 90};
        
        for(int i = 0; i < numVals; i++){
          int auxContT1 = cinemAngleT1[i];
          int auxContT2 = cinemAngleT2[i];
          int auxContT3 = cinemAngleT3[i];

          if(auxContT1 < 0){
            auxContT1 = 0;
          }
          if(auxContT2 < 0){
            auxContT2 = 0;
          }
          if(auxContT3 < 0){
            auxContT3 = 0;
          }

          setAngleServo(m1, auxContT1);
          setAngleServo(m2, auxContT2);
          setAngleServo(m3, auxContT3);

          delay(10);
        }
}

void movimentoRetornoYmenos(int m1, int m2, int m3){
  int numVals = 10;
        int cinemAngleT1[numVals] = {90, 90, 90, 90, 90, 90, 90, 90, 90, 90};
        int cinemAngleT2[numVals] = {69, 68, 67, 67, 67, 66, 66, 66, 67, 67};
        int cinemAngleT3[numVals] = {91, 92, 94, 95, 97, 98, 100, 102, 103, 105};
        
        for(int i = 0; i < numVals; i++){
          int auxContT1 = cinemAngleT1[i];
          int auxContT2 = cinemAngleT2[i];
          int auxContT3 = cinemAngleT3[i];

          if(auxContT1 < 0){
            auxContT1 = 0;
          }
          if(auxContT2 < 0){
            auxContT2 = 0;
          }
          if(auxContT3 < 0){
            auxContT3 = 0;
          }

          setAngleServo(m1, auxContT1);
          setAngleServo(m2, auxContT2);
          setAngleServo(m3, auxContT3);

          delay(10);
        }
}

void movimentoXmenos(int m1, int m2, int m3){
  int numVals = 30;
        int cinemAngleT1[numVals] = {90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 89, 88, 87, 86, 85, 84, 83, 82, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81};
        int cinemAngleT2[numVals] = {64, 62, 60, 58, 55, 53, 50, 48, 46, 43, 44, 44, 45, 45, 46, 46, 47, 48, 49, 50, 52, 54, 56, 58, 60, 63, 65, 67, 69, 72};
        int cinemAngleT3[numVals] = {103, 101, 99, 97, 95, 93, 92, 90, 88, 86, 88, 89, 91, 92, 94, 96, 98, 100, 102, 104, 106, 107, 109, 111, 113, 115, 117, 119, 121, 123};
        
        for(int i = 0; i < numVals; i++){
          int auxContT1 = cinemAngleT1[i];
          int auxContT2 = cinemAngleT2[i];
          int auxContT3 = cinemAngleT3[i];

          if(auxContT1 < 0){
            auxContT1 = 0;
          }
          if(auxContT2 < 0){
            auxContT2 = 0;
          }
          if(auxContT3 < 0){
            auxContT3 = 0;
          }

          setAngleServo(m1, auxContT1);
          setAngleServo(m2, auxContT2);
          setAngleServo(m3, auxContT3);

          delay(10);
        }
}


void movimentoRetornoXmenos(int m1, int m2, int m3){
  int numVals = 10;
        int cinemAngleT1[numVals] = {81, 82, 83, 84, 85, 86, 87, 88, 89, 90};
        int cinemAngleT2[numVals] = {71, 70, 69, 69, 68, 68, 68, 67, 67, 67};
        int cinemAngleT3[numVals] = {121, 119, 117, 115, 113, 111, 110, 108, 107, 105};
        
        for(int i = 0; i < numVals; i++){
          int auxContT1 = cinemAngleT1[i];
          int auxContT2 = cinemAngleT2[i];
          int auxContT3 = cinemAngleT3[i];

          if(auxContT1 < 0){
            auxContT1 = 0;
          }
          if(auxContT2 < 0){
            auxContT2 = 0;
          }
          if(auxContT3 < 0){
            auxContT3 = 0;
          }

          setAngleServo(m1, auxContT1);
          setAngleServo(m2, auxContT2);
          setAngleServo(m3, auxContT3);

          delay(10);
        }
}

void movimentoXmais(int m1, int m2, int m3){
  int numVals = 30;
        int cinemAngleT1[numVals] = {90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 91, 92, 94, 95, 96, 98, 99, 101, 102, 104, 104, 104, 104, 104, 104, 104, 104, 104, 104, 104};
        int cinemAngleT2[numVals] = {64, 62, 60, 58, 55, 53, 50, 48, 46, 43, 43, 43, 42, 42, 42, 42, 42, 42, 42, 42, 44, 47, 50, 52, 55, 57, 60, 62, 65, 67};
        int cinemAngleT3[numVals] = {103, 101, 99, 97, 95, 93, 92, 90, 88, 86, 85, 83, 82, 81, 80, 78, 77, 76, 75, 75, 77, 79, 81, 83, 85, 87, 89, 91, 93, 95};
        
        for(int i = 0; i < numVals; i++){
          int auxContT1 = cinemAngleT1[i];
          int auxContT2 = cinemAngleT2[i];
          int auxContT3 = cinemAngleT3[i];

          if(auxContT1 < 0){
            auxContT1 = 0;
          }
          if(auxContT2 < 0){
            auxContT2 = 0;
          }
          if(auxContT3 < 0){
            auxContT3 = 0;
          }

          setAngleServo(m1, auxContT1);
          setAngleServo(m2, auxContT2);
          setAngleServo(m3, auxContT3);

          delay(10);
        }
}


void movimentoRetornoXmais(int m1, int m2, int m3){
      int numVals = 10;
        int cinemAngleT1[numVals] = {102, 101, 99, 98, 96, 95, 94, 92, 91, 90};
        int cinemAngleT2[numVals] = {67, 67, 67, 66, 66, 66, 66, 66, 67, 67};
        int cinemAngleT3[numVals] = {96, 97, 98, 99, 100, 101, 102, 103, 104, 105};
        
        for(int i = 0; i < numVals; i++){
          int auxContT1 = cinemAngleT1[i];
          int auxContT2 = cinemAngleT2[i];
          int auxContT3 = cinemAngleT3[i];

          if(auxContT1 < 0){
            auxContT1 = 0;
          }
          if(auxContT2 < 0){
            auxContT2 = 0;
          }
          if(auxContT3 < 0){
            auxContT3 = 0;
          }

          setAngleServo(m1, auxContT1);
          setAngleServo(m2, auxContT2);
          setAngleServo(m3, auxContT3);

          delay(10);
        }
}
  
void loop() {
  //Posicionamento Default para as tres juntas de cada pata: 90  /  0  /  180

  while (Serial.available() > 0) {
      
      auxAngle = Serial.parseInt();

      if(auxAngle == 374){
          int aux = 6;
          int auxnewAngleM = 90;
          int auxServ[aux] = {0,1,2,3,4,5};
          int auxAngle[aux] = {auxnewAngleM,auxnewAngleM,auxnewAngleM,auxnewAngleM,auxnewAngleM,auxnewAngleM};
          setAllServosAngle(auxServ, auxAngle, aux);
          showAngles();
          Serial.print("Ajuste em 90 dos primeiros motores");
      }
      
      if(auxAngle == 98){
        auxCont += 1;
          Serial.print("AuxCont = ");Serial.println(auxCont);
      }
      
      if(auxAngle == 99){
        auxCont -= 1;
          Serial.print("AuxCont = ");Serial.println(auxCont);
      }

      if(auxAngle == 100){
        delay(100);

        if((auxCont < 181) and (auxCont > 20)){
          int aux = 6;
          int auxServ[aux] = {12,13,14,15,16,17};
          int auxAngle[aux] = {auxCont,auxCont,auxCont,auxCont,auxCont,auxCont};
          setAllServosAngle(auxServ, auxAngle, aux);
          showAngles();
        }else{
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
        }
      }

      if(auxAngle == 198){
        auxCont2 += 1;
          Serial.print("AuxCont2 = ");Serial.println(auxCont2);
      }
      
      if(auxAngle == 199){
        auxCont2 -= 1;
          Serial.print("AuxCont2 = ");Serial.println(auxCont2);
      }

      if(auxAngle == 101){
        delay(100);

        if((auxCont2 < 160) and (auxCont2 > -1)){
          int aux = 6;
          int auxServ[aux] = {6,7,8,9,10,11};
          int auxAngle[aux] = {auxCont2,auxCont2,auxCont2,auxCont2,auxCont2,auxCont2};
          setAllServosAngle(auxServ, auxAngle, aux);
          showAngles();
        }else{
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
        }
      }

      
      if(auxAngle == 1000){
        auxCont = 58;
        auxCont2 = 0;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      }

     /* if(auxAngle == 1001){
        auxCont = 61;
        auxCont2 = 3;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      }
      
      if(auxAngle == 1002){
        auxCont = 64;
        auxCont2 = 10;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      }
      
      if(auxAngle == 1003){
        auxCont = 68;
        auxCont2 = 18;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      }
      if(auxAngle == 1004){
        auxCont = 73;
        auxCont2 = 26;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      }
      
      if(auxAngle == 1005){
        auxCont = 78;
        auxCont2 = 34;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      }
      if(auxAngle == 1006){
        auxCont = 84;
        auxCont2 = 42;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      }
      
      if(auxAngle == 1007){
        auxCont = 90;
        auxCont2 = 50;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      }

      if(auxAngle == 1008){
        auxCont = 97;
        auxCont2 = 59;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      }
      
      if(auxAngle == 1009){
        auxCont = 105;
        auxCont2 = 67;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      }
  

  if(auxAngle == 1010){
        moveCinem();
        delay(500);
        auxCont = 58;
        auxCont2 = 0;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);

          
        moveCinem();
        delay(500);
        auxCont = 61;
        auxCont2 = 3;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
        
        moveCinem();
        delay(500);
        auxCont = 64;
        auxCont2 = 10;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      
        moveCinem();
        delay(500);
        auxCont = 68;
        auxCont2 = 18;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      
        moveCinem();
        delay(500);
        auxCont = 73;
        auxCont2 = 26;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      
        moveCinem();
        delay(500);
        auxCont = 78;
        auxCont2 = 34;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
     
        moveCinem();
        delay(500);
        auxCont = 84;
        auxCont2 = 42;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      
        moveCinem();
        delay(500);
        auxCont = 90;
        auxCont2 = 50;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      
        moveCinem();
        delay(500);
        auxCont = 97;
        auxCont2 = 59;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
     
        moveCinem();
        delay(500);
        auxCont = 105;
        auxCont2 = 67;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
        moveCinem();
        delay(500);      
  }*/


  if(auxAngle == 555){
        setAngleServo(6, auxCont2);
        setAngleServo(7, auxCont2);
        setAngleServo(8, auxCont2);
        setAngleServo(9, auxCont2);
        setAngleServo(10, auxCont2);
        setAngleServo(11, auxCont2);        
        setAngleServo(12, auxCont);
        setAngleServo(13, auxCont);
        setAngleServo(14, auxCont);
        setAngleServo(15, auxCont);
        setAngleServo(16, auxCont);
        setAngleServo(17, auxCont);
        auxCont = 58;
        auxCont2 = 0;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);

          
        setAngleServo(6, auxCont2);
        setAngleServo(7, auxCont2);
        setAngleServo(8, auxCont2);
        setAngleServo(9, auxCont2);
        setAngleServo(10, auxCont2);
        setAngleServo(11, auxCont2);        
        setAngleServo(12, auxCont);
        setAngleServo(13, auxCont);
        setAngleServo(14, auxCont);
        setAngleServo(15, auxCont);
        setAngleServo(16, auxCont);
        setAngleServo(17, auxCont);
        auxCont = 61;
        auxCont2 = 3;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
        
        
        setAngleServo(6, auxCont2);
        setAngleServo(7, auxCont2);
        setAngleServo(8, auxCont2);
        setAngleServo(9, auxCont2);
        setAngleServo(10, auxCont2);
        setAngleServo(11, auxCont2);        
        setAngleServo(12, auxCont);
        setAngleServo(13, auxCont);
        setAngleServo(14, auxCont);
        setAngleServo(15, auxCont);
        setAngleServo(16, auxCont);
        setAngleServo(17, auxCont);
        auxCont = 64;
        auxCont2 = 10;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      
        
        setAngleServo(6, auxCont2);
        setAngleServo(7, auxCont2);
        setAngleServo(8, auxCont2);
        setAngleServo(9, auxCont2);
        setAngleServo(10, auxCont2);
        setAngleServo(11, auxCont2);        
        setAngleServo(12, auxCont);
        setAngleServo(13, auxCont);
        setAngleServo(14, auxCont);
        setAngleServo(15, auxCont);
        setAngleServo(16, auxCont);
        setAngleServo(17, auxCont);
        auxCont = 68;
        auxCont2 = 18;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      
        
        setAngleServo(6, auxCont2);
        setAngleServo(7, auxCont2);
        setAngleServo(8, auxCont2);
        setAngleServo(9, auxCont2);
        setAngleServo(10, auxCont2);
        setAngleServo(11, auxCont2);        
        setAngleServo(12, auxCont);
        setAngleServo(13, auxCont);
        setAngleServo(14, auxCont);
        setAngleServo(15, auxCont);
        setAngleServo(16, auxCont);
        setAngleServo(17, auxCont);
        auxCont = 73;
        auxCont2 = 26;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      
        
        setAngleServo(6, auxCont2);
        setAngleServo(7, auxCont2);
        setAngleServo(8, auxCont2);
        setAngleServo(9, auxCont2);
        setAngleServo(10, auxCont2);
        setAngleServo(11, auxCont2);        
        setAngleServo(12, auxCont);
        setAngleServo(13, auxCont);
        setAngleServo(14, auxCont);
        setAngleServo(15, auxCont);
        setAngleServo(16, auxCont);
        setAngleServo(17, auxCont);
        auxCont = 78;
        auxCont2 = 34;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
     
        
        setAngleServo(6, auxCont2);
        setAngleServo(7, auxCont2);
        setAngleServo(8, auxCont2);
        setAngleServo(9, auxCont2);
        setAngleServo(10, auxCont2);
        setAngleServo(11, auxCont2);        
        setAngleServo(12, auxCont);
        setAngleServo(13, auxCont);
        setAngleServo(14, auxCont);
        setAngleServo(15, auxCont);
        setAngleServo(16, auxCont);
        setAngleServo(17, auxCont);
        auxCont = 84;
        auxCont2 = 42;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      
        
        setAngleServo(6, auxCont2);
        setAngleServo(7, auxCont2);
        setAngleServo(8, auxCont2);
        setAngleServo(9, auxCont2);
        setAngleServo(10, auxCont2);
        setAngleServo(11, auxCont2);        
        setAngleServo(12, auxCont);
        setAngleServo(13, auxCont);
        setAngleServo(14, auxCont);
        setAngleServo(15, auxCont);
        setAngleServo(16, auxCont);
        setAngleServo(17, auxCont);
        auxCont = 90;
        auxCont2 = 50;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      
        
        setAngleServo(6, auxCont2);
        setAngleServo(7, auxCont2);
        setAngleServo(8, auxCont2);
        setAngleServo(9, auxCont2);
        setAngleServo(10, auxCont2);
        setAngleServo(11, auxCont2);        
        setAngleServo(12, auxCont);
        setAngleServo(13, auxCont);
        setAngleServo(14, auxCont);
        setAngleServo(15, auxCont);
        setAngleServo(16, auxCont);
        setAngleServo(17, auxCont);
        auxCont = 97;
        auxCont2 = 59;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
     
        
        setAngleServo(6, auxCont2);
        setAngleServo(7, auxCont2);
        setAngleServo(8, auxCont2);
        setAngleServo(9, auxCont2);
        setAngleServo(10, auxCont2);
        setAngleServo(11, auxCont2);        
        setAngleServo(12, auxCont);
        setAngleServo(13, auxCont);
        setAngleServo(14, auxCont);
        setAngleServo(15, auxCont);
        setAngleServo(16, auxCont);
        setAngleServo(17, auxCont);
        auxCont = 105;
        auxCont2 = 67;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
        
        setAngleServo(6, auxCont2);
        setAngleServo(7, auxCont2);
        setAngleServo(8, auxCont2);
        setAngleServo(9, auxCont2);
        setAngleServo(10, auxCont2);
        setAngleServo(11, auxCont2);        
        setAngleServo(12, auxCont);
        setAngleServo(13, auxCont);
        setAngleServo(14, auxCont);
        setAngleServo(15, auxCont);
        setAngleServo(16, auxCont);
        setAngleServo(17, auxCont);      
      }
      /*if(auxAngle == 190){
          int tempoA = 20;
          for(int i = 0; i < 37; i ++){
            setAngleServo(6, 67 - i);
            setAngleServo(8, 67 - i);
            setAngleServo(10, 67 - i);
            delay(tempoA);
          }
          delay(1000);
          for(int i = 0; i < 30; i ++){
            setAngleServo(0, 90 + i);
            setAngleServo(2, 90 - i);
            setAngleServo(4, 90 + i);
            delay(tempoA);
          }        
          delay(1000);
          for(int i = 0; i < 37; i ++){
            setAngleServo(6, 30 + i);
            setAngleServo(8, 30 + i);
            setAngleServo(10, 30 + i);
            delay(tempoA);
          } 
          delay(1000);
          for(int i = 0; i < 37; i ++){
            setAngleServo(7, 67 - i);
            setAngleServo(9, 67 - i);
            setAngleServo(11, 67 - i);
            delay(tempoA);
          }
          delay(1000);
          for(int i = 0; i < 30; i ++){
            setAngleServo(0, 120 - i);
            setAngleServo(2, 60 + i);
            setAngleServo(4, 120 - i);
            delay(tempoA);
          }   
          delay(1000);
          for(int i = 0; i < 30; i ++){
            setAngleServo(1, 90 + i);
            setAngleServo(3, 90 - i);
            setAngleServo(5, 90 + i);
            delay(tempoA);
          } 
          delay(1000);
          for(int i = 0; i < 37; i ++){
            setAngleServo(7, 30 + i);
            setAngleServo(9, 30 + i);
            setAngleServo(11, 30 + i);
            delay(tempoA);
          } 
          delay(1000);
          for(int i = 0; i < 37; i ++){
            setAngleServo(6, 67 - i);
            setAngleServo(8, 67 - i);
            setAngleServo(10, 67 - i);
            delay(tempoA);
          }
          delay(1000);
          for(int i = 0; i < 30; i ++){
            setAngleServo(1, 120 - i);
            setAngleServo(3, 60 + i);
            setAngleServo(5, 120 - i);
            delay(tempoA);
          } 
          delay(1000);
          for(int i = 0; i < 37; i ++){
            setAngleServo(6, 30 + i);
            setAngleServo(8, 30 + i);
            setAngleServo(10, 30 + i);
            delay(tempoA); 
          }
      }

      if(auxAngle == 198){
          setAngleServo(7, 30);
          setAngleServo(9, 30);
          setAngleServo(11, 30);
          delay(1000);
          setAngleServo(1, 120); 
          setAngleServo(3, 120); 
          setAngleServo(5, 120);          
          delay(1000);
          setAngleServo(1, 90);
          setAngleServo(3, 90);
          setAngleServo(5, 90);  
          delay(1000);
          setAngleServo(7, 67);
          setAngleServo(9, 67);
          setAngleServo(11, 67);                   
      }*/



       if(auxAngle == 556){
        setAngleServo(6, auxCont2);
        setAngleServo(7, auxCont2);
        setAngleServo(8, auxCont2);
        setAngleServo(9, auxCont2);
        setAngleServo(10, auxCont2);
        setAngleServo(11, auxCont2);        
        setAngleServo(12, auxCont);
        setAngleServo(13, auxCont);
        setAngleServo(14, auxCont);
        setAngleServo(15, auxCont);
        setAngleServo(16, auxCont);
        setAngleServo(17, auxCont);
        auxCont = 105; //58; 
        auxCont2 = 67; //0;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);

          
        setAngleServo(6, auxCont2);
        setAngleServo(7, auxCont2);
        setAngleServo(8, auxCont2);
        setAngleServo(9, auxCont2);
        setAngleServo(10, auxCont2);
        setAngleServo(11, auxCont2);        
        setAngleServo(12, auxCont);
        setAngleServo(13, auxCont);
        setAngleServo(14, auxCont);
        setAngleServo(15, auxCont);
        setAngleServo(16, auxCont);
        setAngleServo(17, auxCont);
        auxCont = 97; //61;
        auxCont2 = 59; //3;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
        
        
        setAngleServo(6, auxCont2);
        setAngleServo(7, auxCont2);
        setAngleServo(8, auxCont2);
        setAngleServo(9, auxCont2);
        setAngleServo(10, auxCont2);
        setAngleServo(11, auxCont2);        
        setAngleServo(12, auxCont);
        setAngleServo(13, auxCont);
        setAngleServo(14, auxCont);
        setAngleServo(15, auxCont);
        setAngleServo(16, auxCont);
        setAngleServo(17, auxCont);
        auxCont = 90; //64;
        auxCont2 = 50; //10;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      
        
        setAngleServo(6, auxCont2);
        setAngleServo(7, auxCont2);
        setAngleServo(8, auxCont2);
        setAngleServo(9, auxCont2);
        setAngleServo(10, auxCont2);
        setAngleServo(11, auxCont2);        
        setAngleServo(12, auxCont);
        setAngleServo(13, auxCont);
        setAngleServo(14, auxCont);
        setAngleServo(15, auxCont);
        setAngleServo(16, auxCont);
        setAngleServo(17, auxCont);
        auxCont =  84; //68;
        auxCont2 = 42; //18;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      
        
        setAngleServo(6, auxCont2);
        setAngleServo(7, auxCont2);
        setAngleServo(8, auxCont2);
        setAngleServo(9, auxCont2);
        setAngleServo(10, auxCont2);
        setAngleServo(11, auxCont2);        
        setAngleServo(12, auxCont);
        setAngleServo(13, auxCont);
        setAngleServo(14, auxCont);
        setAngleServo(15, auxCont);
        setAngleServo(16, auxCont);
        setAngleServo(17, auxCont);
        auxCont = 78; //73;
        auxCont2 = 34; //26;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      
        
        setAngleServo(6, auxCont2);
        setAngleServo(7, auxCont2);
        setAngleServo(8, auxCont2);
        setAngleServo(9, auxCont2);
        setAngleServo(10, auxCont2);
        setAngleServo(11, auxCont2);        
        setAngleServo(12, auxCont);
        setAngleServo(13, auxCont);
        setAngleServo(14, auxCont);
        setAngleServo(15, auxCont);
        setAngleServo(16, auxCont);
        setAngleServo(17, auxCont);
        auxCont = 73; //78;
        auxCont2 = 26; //34;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
     
        
        setAngleServo(6, auxCont2);
        setAngleServo(7, auxCont2);
        setAngleServo(8, auxCont2);
        setAngleServo(9, auxCont2);
        setAngleServo(10, auxCont2);
        setAngleServo(11, auxCont2);        
        setAngleServo(12, auxCont);
        setAngleServo(13, auxCont);
        setAngleServo(14, auxCont);
        setAngleServo(15, auxCont);
        setAngleServo(16, auxCont);
        setAngleServo(17, auxCont);
        auxCont = 68; //84;
        auxCont2 = 18; //42;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      
        
        setAngleServo(6, auxCont2);
        setAngleServo(7, auxCont2);
        setAngleServo(8, auxCont2);
        setAngleServo(9, auxCont2);
        setAngleServo(10, auxCont2);
        setAngleServo(11, auxCont2);        
        setAngleServo(12, auxCont);
        setAngleServo(13, auxCont);
        setAngleServo(14, auxCont);
        setAngleServo(15, auxCont);
        setAngleServo(16, auxCont);
        setAngleServo(17, auxCont);
        auxCont = 64; //90;
        auxCont2 = 10; //50;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
      
        
        setAngleServo(6, auxCont2);
        setAngleServo(7, auxCont2);
        setAngleServo(8, auxCont2);
        setAngleServo(9, auxCont2);
        setAngleServo(10, auxCont2);
        setAngleServo(11, auxCont2);        
        setAngleServo(12, auxCont);
        setAngleServo(13, auxCont);
        setAngleServo(14, auxCont);
        setAngleServo(15, auxCont);
        setAngleServo(16, auxCont);
        setAngleServo(17, auxCont);
        auxCont = 61; //97;
        auxCont2 = 3; //59;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
     
        
        setAngleServo(6, auxCont2);
        setAngleServo(7, auxCont2);
        setAngleServo(8, auxCont2);
        setAngleServo(9, auxCont2);
        setAngleServo(10, auxCont2);
        setAngleServo(11, auxCont2);        
        setAngleServo(12, auxCont);
        setAngleServo(13, auxCont);
        setAngleServo(14, auxCont);
        setAngleServo(15, auxCont);
        setAngleServo(16, auxCont);
        setAngleServo(17, auxCont);
        auxCont = 58; //105;
        auxCont2 = 0; //67;
          Serial.print("Valor inadequado: AuxCont = ");Serial.println(auxCont);
          Serial.print("Valor inadequado: AuxCont2 = ");Serial.println(auxCont2);
        
        setAngleServo(6, auxCont2);
        setAngleServo(7, auxCont2);
        setAngleServo(8, auxCont2);
        setAngleServo(9, auxCont2);
        setAngleServo(10, auxCont2);
        setAngleServo(11, auxCont2);        
        setAngleServo(12, auxCont);
        setAngleServo(13, auxCont);
        setAngleServo(14, auxCont);
        setAngleServo(15, auxCont);
        setAngleServo(16, auxCont);
        setAngleServo(17, auxCont);      
      }






























      
//////////////////////////////////////////////////////#############################//////////////////////////////////////////////////////#############################//////////////////////////////////////////////////////#############################//////////////////////////////////////////////////////#############################
//////////////////////////////////////////////////////#############################//////////////////////////////////////////////////////#############################//////////////////////////////////////////////////////#############################//////////////////////////////////////////////////////#############################
//////////////////////////////////////////////////////#############################//////////////////////////////////////////////////////#############################//////////////////////////////////////////////////////#############################//////////////////////////////////////////////////////#############################

       if(auxAngle == 1338){
          int ang = 0;
          float aux_x = p1_x;
          float aux_y = p1_y;
          float aux_z = p1_z;
                Serial.println("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
          cinematic(p1_x, p1_y, p1_z, &p1_a1, &p1_a2, &p1_a3);
          cinematic(p3_x, p3_y, p3_z, &p3_a1, &p3_a2, &p3_a3);
          cinematic(p5_x, p5_y, p5_z, &p5_a1, &p5_a2, &p5_a3);
    
          calcRelPos(&p1_x, &p1_y, &p1_z, 1, ang); 
          calcRelPos(&p3_x, &p3_y, &p3_z, 3, ang); 
          calcRelPos(&p5_x, &p5_y, &p5_z, 5, ang); 
          
      int stepsNum = 10;
      Serial.println("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= COMEC");

      //PRIMEIRO
      for(int i = 0; i <= stepsNum; i++){
        float p_aux_X = aux_x - (aux_x-p1_x) * i/stepsNum;
        float p_aux_Y = aux_y - (aux_y-p1_y) * i/stepsNum;
        float p_aux_Z = aux_z - (aux_z-p1_z) * i/stepsNum;
        
        float p_aux_X_p3 = aux_x - (aux_x-p3_x) * i/stepsNum;
        float p_aux_Y_p3 = aux_y - (aux_y-p3_y) * i/stepsNum;
        float p_aux_Z_p3 = aux_z - (aux_z-p3_z) * i/stepsNum;
        
        float p_aux_X_p5 = aux_x - (aux_x-p5_x) * i/stepsNum;
        float p_aux_Y_p5 = aux_y - (aux_y-p5_y) * i/stepsNum;
        float p_aux_Z_p5 = aux_z - (aux_z-p5_z) * i/stepsNum;
        cinematic(p_aux_X, p_aux_Y, p_aux_Z, &p1_a1, &p1_a2, &p1_a3);
        cinematic(p_aux_X_p3, p_aux_Y_p3, p_aux_Z_p3, &p3_a1, &p3_a2, &p3_a3);
        cinematic(p_aux_X_p5, p_aux_Y_p5, p_aux_Z_p5, &p5_a1, &p5_a2, &p5_a3);
        
        setAngleServo(0, p1_a1);
        setAngleServo(6, p1_a2);
        setAngleServo(12, p1_a3);
        
        setAngleServo(4, p3_a1);
        setAngleServo(10, p3_a2);
        setAngleServo(16, p3_a3);
        //setAngleServo(2, p3_a1);
        //setAngleServo(8, p3_a2);
        //setAngleServo(14, p3_a3);
        
        setAngleServo(2, p5_a1);
        setAngleServo(8, p5_a2);
        setAngleServo(14, p5_a3);
        //setAngleServo(4, p5_a1);
        //setAngleServo(10, p5_a2);
        //setAngleServo(16, p5_a3);
        delay(20);
        Serial.print("01 - Angulos: ");Serial.print(p1_a1);Serial.print(" ");Serial.print(p1_a2);Serial.print(" ");Serial.println(p1_a3);
        Serial.print("01 - Posicao: ");Serial.print(p_aux_X);Serial.print(" ");Serial.print(p_aux_Y);Serial.print(" ");Serial.println(p_aux_Z);
      }
       Serial.println("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
      
//===============================================================================================================================================================================
//===============================================================================================================================================================================
      cinematic(p1_x, p1_y, aux_z, &p1_a1, &p1_a2, &p1_a3);
      cinematic(p3_x, p3_y, aux_z, &p3_a1, &p3_a2, &p3_a3);
      cinematic(p5_x, p5_y, aux_z, &p5_a1, &p5_a2, &p5_a3);
      Serial.print("01 - Angulos: ");Serial.print(p1_a1);Serial.print(" ");Serial.print(p1_a2);Serial.print(" ");Serial.println(p1_a3);
      Serial.print("01 - Posicao: ");Serial.print(p1_x);Serial.print(" ");Serial.print(p1_y);Serial.print(" ");Serial.println(p1_z);
       Serial.println("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= COMEC");
//===============================================================================================================================================================================
//===============================================================================================================================================================================
      float aux_x_interm = p1_x;
      float aux_y_interm = p1_y;
      float aux_z_interm = p1_z;

      float aux_x_interm_p3 = p3_x;
      float aux_y_interm_p3 = p3_y;
      float aux_z_interm_p3 = p3_z;

      float aux_x_interm_p5 = p5_x;
      float aux_y_interm_p5 = p5_y;
      float aux_z_interm_p5 = p5_z;

      
      //SEGUNDO
      for(int i = 0; i <= stepsNum; i++){
        float p_aux_X = aux_x_interm;
        float p_aux_Y = aux_y_interm;
        float p_aux_Z = aux_z_interm + (aux_z-p1_z) * i/stepsNum;
        cinematic(p_aux_X, p_aux_Y, p_aux_Z, &p1_a1, &p1_a2, &p1_a3);

        
        float p_aux_X_p3 = aux_x_interm_p3;
        float p_aux_Y_p3 = aux_y_interm_p3;
        float p_aux_Z_p3 = aux_z_interm_p3 + (aux_z-p3_z) * i/stepsNum;
        cinematic(p_aux_X_p3, p_aux_Y_p3, p_aux_Z_p3, &p3_a1, &p3_a2, &p3_a3);

        
        float p_aux_X_p5 = aux_x_interm_p5;
        float p_aux_Y_p5 = aux_y_interm_p5;
        float p_aux_Z_p5 = aux_z_interm_p5 + (aux_z-p5_z) * i/stepsNum;
        cinematic(p_aux_X_p5, p_aux_Y_p5, p_aux_Z_p5, &p5_a1, &p5_a2, &p5_a3);
        
        setAngleServo(0, p1_a1);
        setAngleServo(6, p1_a2);
        setAngleServo(12, p1_a3);
        
        setAngleServo(4, p3_a1);
        setAngleServo(10, p3_a2);
        setAngleServo(16, p3_a3);
        //setAngleServo(2, p3_a1);
        //setAngleServo(8, p3_a2);
        //setAngleServo(14, p3_a3);
        
        setAngleServo(2, p5_a1);
        setAngleServo(8, p5_a2);
        setAngleServo(14, p5_a3);
        //setAngleServo(4, p5_a1);
        //setAngleServo(10, p5_a2);
        //setAngleServo(16, p5_a3);
        delay(20);
        Serial.print("01 - Angulos: ");Serial.print(p1_a1);Serial.print(" ");Serial.print(p1_a2);Serial.print(" ");Serial.println(p1_a3);
        Serial.print("01 - Posicao: ");Serial.print(p1_x);Serial.print(" ");Serial.print(p_aux_Y);Serial.print(" ");Serial.println(p_aux_Z);
      }
       Serial.println("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
      
//===============================================================================================================================================================================
//===============================================================================================================================================================================

      cinematic(aux_x, aux_y, aux_z, &p1_a1, &p1_a2, &p1_a3);
      Serial.print("01 - Angulos: ");Serial.print(p1_a1);Serial.print(" ");Serial.print(p1_a2);Serial.print(" ");Serial.println(p1_a3);
      Serial.print("01 - Posicao: ");Serial.print(p1_x);Serial.print(" ");Serial.print(p1_y);Serial.print(" ");Serial.println(p1_z);
      Serial.println("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= COMEC");
      p1_z = aux_z;
      p3_z = aux_z;
      p5_z = aux_z;

      //========================================

      cinematic(p2_x, p2_y, p2_z, &p2_a1, &p2_a2, &p2_a3);
      cinematic(p4_x, p4_y, p4_z, &p4_a1, &p4_a2, &p4_a3);
      cinematic(p6_x, p6_y, p6_z, &p6_a1, &p6_a2, &p6_a3);
      calcRelPos(&p2_x, &p2_y, &p2_z, 2, ang); //Pernas de 1 ate 6
      calcRelPos(&p4_x, &p4_y, &p4_z, 4, ang); //Pernas de 1 ate 6
      calcRelPos(&p6_x, &p6_y, &p6_z, 6, ang); //Pernas de 1 ate 6
//===============================================================================================================================================================================
//===============================================================================================================================================================================
      aux_x_interm = p1_x;
      aux_y_interm = p1_y;
      aux_z_interm = p1_z;

      aux_x_interm_p3 = p3_x;
      aux_y_interm_p3 = p3_y;
      aux_z_interm_p3 = p3_z;

      aux_x_interm_p5 = p5_x;
      aux_y_interm_p5 = p5_y;
      aux_z_interm_p5 = p5_z;

      
      //TERCEIRO
      for(int i = 0; i <= stepsNum; i++){
        float p_aux_X = aux_x_interm + (aux_x-p1_x) * i/stepsNum;
        float p_aux_Y = aux_y_interm + (aux_y-p1_y) * i/stepsNum;
        float p_aux_Z = aux_z_interm;
        cinematic(p_aux_X, p_aux_Y, p_aux_Z, &p1_a1, &p1_a2, &p1_a3);

        float p_aux_X_p3 = aux_x_interm_p3 + (aux_x-p3_x) * i/stepsNum;
        float p_aux_Y_p3 = aux_y_interm_p3 + (aux_y-p3_y) * i/stepsNum;
        float p_aux_Z_p3 = aux_z_interm_p3;
        cinematic(p_aux_X_p3, p_aux_Y_p3, p_aux_Z_p3, &p3_a1, &p3_a2, &p3_a3);

        
        float p_aux_X_p5 = aux_x_interm_p5 + (aux_x-p5_x) * i/stepsNum;
        float p_aux_Y_p5 = aux_y_interm_p5 + (aux_y-p5_y) * i/stepsNum;
        float p_aux_Z_p5 = aux_z_interm_p5;
        cinematic(p_aux_X_p5, p_aux_Y_p5, p_aux_Z_p5, &p5_a1, &p5_a2, &p5_a3);
        
        setAngleServo(0, p1_a1);
        setAngleServo(6, p1_a2);
        setAngleServo(12, p1_a3);
        
        setAngleServo(4, p3_a1);
        setAngleServo(10, p3_a2);
        setAngleServo(16, p3_a3);
        //setAngleServo(2, p3_a1);
        //setAngleServo(8, p3_a2);
        //setAngleServo(14, p3_a3);
        
        setAngleServo(2, p5_a1);
        setAngleServo(8, p5_a2);
        setAngleServo(14, p5_a3);
        //setAngleServo(4, p5_a1);
        //setAngleServo(10, p5_a2);
        //setAngleServo(16, p5_a3);

        //(((((((((((((((((((((((((((((((((((((((((((((((((((

        float p_aux_X_p2 = aux_x - (aux_x-p2_x) * i/stepsNum;
        float p_aux_Y_p2 = aux_y - (aux_y-p2_y) * i/stepsNum;
        float p_aux_Z_p2 = aux_z - (aux_z-p2_z) * i/stepsNum;
        
        float p_aux_X_p4 = aux_x - (aux_x-p4_x) * i/stepsNum;
        float p_aux_Y_p4 = aux_y - (aux_y-p4_y) * i/stepsNum;
        float p_aux_Z_p4 = aux_z - (aux_z-p4_z) * i/stepsNum;
        
        float p_aux_X_p6 = aux_x - (aux_x-p6_x) * i/stepsNum;
        float p_aux_Y_p6 = aux_y - (aux_y-p6_y) * i/stepsNum;
        float p_aux_Z_p6 = aux_z - (aux_z-p6_z) * i/stepsNum;
        cinematic(p_aux_X_p2, p_aux_Y_p2, p_aux_Z_p2, &p2_a1, &p2_a2, &p2_a3);
        cinematic(p_aux_X_p4, p_aux_Y_p4, p_aux_Z_p4, &p4_a1, &p4_a2, &p4_a3);
        cinematic(p_aux_X_p6, p_aux_Y_p6, p_aux_Z_p6, &p6_a1, &p6_a2, &p6_a3);
        
        setAngleServo(1, p6_a1);
        setAngleServo(7, p6_a2);
        setAngleServo(13, p6_a3);
        
        setAngleServo(5, p2_a1);
        setAngleServo(11, p2_a2);
        setAngleServo(17, p2_a3);
        
        setAngleServo(3, p4_a1);
        setAngleServo(9, p4_a2);
        setAngleServo(15, p4_a3);
        
        /*setAngleServo(1, p2_a1);
        setAngleServo(7, p2_a2);
        setAngleServo(13, p2_a3);
        
        setAngleServo(5, p4_a1);
        setAngleServo(11, p4_a2);
        setAngleServo(17, p4_a3);
        
        setAngleServo(3, p6_a1);
        setAngleServo(9, p6_a2);
        setAngleServo(15, p6_a3);*/
        
        //))))))))))))))))))))))))))))))))))))))))))))))))))

        
        delay(20);
        Serial.print("01 - Angulos: ");Serial.print(p1_a1);Serial.print(" ");Serial.print(p1_a2);Serial.print(" ");Serial.println(p1_a3);
        Serial.print("01 - Posicao: ");Serial.print(p1_x);Serial.print(" ");Serial.print(p_aux_Y);Serial.print(" ");Serial.println(p_aux_Z);
      }
       Serial.println("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
      
      
      
      p1_x = aux_x;
      p1_y = aux_y;
      p1_z = aux_z;
      
      p3_x = aux_x;
      p3_y = aux_y;
      p3_z = aux_z;
      
      p5_x = aux_x;
      p5_y = aux_y;
      p5_z = aux_z;

//===============================================================================================================================================================================
//===============================================================================================================================================================================
     

      float aux_x_interm_p2 = p2_x;
      float aux_y_interm_p2 = p2_y;
      float aux_z_interm_p2 = p2_z;

      float aux_x_interm_p4 = p4_x;
      float aux_y_interm_p4 = p4_y;
      float aux_z_interm_p4 = p4_z;

      float aux_x_interm_p6 = p6_x;
      float aux_y_interm_p6 = p6_y;
      float aux_z_interm_p6 = p6_z;

      //QUARTO
      for(int i = 0; i <= stepsNum; i++){
        float p_aux_X_p2 = aux_x_interm_p2;
        float p_aux_Y_p2 = aux_y_interm_p2;
        float p_aux_Z_p2 = aux_z_interm_p2 + (aux_z-p2_z) * i/stepsNum;
        cinematic(p_aux_X_p2, p_aux_Y_p2, p_aux_Z_p2, &p2_a1, &p2_a2, &p2_a3);

        
        float p_aux_X_p4 = aux_x_interm_p4;
        float p_aux_Y_p4 = aux_y_interm_p4;
        float p_aux_Z_p4 = aux_z_interm_p4 + (aux_z-p4_z) * i/stepsNum;
        cinematic(p_aux_X_p4, p_aux_Y_p4, p_aux_Z_p4, &p4_a1, &p4_a2, &p4_a3);

        
        float p_aux_X_p6 = aux_x_interm_p6;
        float p_aux_Y_p6 = aux_y_interm_p6;
        float p_aux_Z_p6 = aux_z_interm_p6 + (aux_z-p6_z) * i/stepsNum;
        cinematic(p_aux_X_p6, p_aux_Y_p6, p_aux_Z_p6, &p6_a1, &p6_a2, &p6_a3);
                
        setAngleServo(1, p6_a1);
        setAngleServo(7, p6_a2);
        setAngleServo(13, p6_a3);
        
        setAngleServo(5, p2_a1);
        setAngleServo(11, p2_a2);
        setAngleServo(17, p2_a3);
        
        setAngleServo(3, p4_a1);
        setAngleServo(9, p4_a2);
        setAngleServo(15, p4_a3);
        
        /*setAngleServo(1, p2_a1);
        setAngleServo(7, p2_a2);
        setAngleServo(13, p2_a3);
        
        setAngleServo(5, p4_a1);
        setAngleServo(11, p4_a2);
        setAngleServo(17, p4_a3);
        
        setAngleServo(3, p6_a1);
        setAngleServo(9, p6_a2);
        setAngleServo(15, p6_a3);*/
        delay(20);
      }
       Serial.println("-=-=-=-=-=-=-=-=-=-=-=ULTIMO-=-=-=-=-=-=-=-=-=-=");
       p2_z = aux_z;
       p4_z = aux_z;
       p6_z = aux_z;

      aux_x_interm_p2 = p2_x;
      aux_y_interm_p2 = p2_y;
      aux_z_interm_p2 = p2_z;

      aux_x_interm_p4 = p4_x;
      aux_y_interm_p4 = p4_y;
      aux_z_interm_p4 = p4_z;

      aux_x_interm_p6 = p6_x;
      aux_y_interm_p6 = p6_y;
      aux_z_interm_p6 = p6_z;

      
      //QUINTO
      for(int i = 0; i <= stepsNum; i++){
        float p_aux_X_p2 = aux_x_interm_p2 + (aux_x-p2_x) * i/stepsNum;
        float p_aux_Y_p2 = aux_y_interm_p2 + (aux_y-p2_y) * i/stepsNum;
        float p_aux_Z_p2 = aux_z_interm_p2;
        cinematic(p_aux_X_p2, p_aux_Y_p2, p_aux_Z_p2, &p2_a1, &p2_a2, &p2_a3);

        float p_aux_X_p4 = aux_x_interm_p4 + (aux_x-p4_x) * i/stepsNum;
        float p_aux_Y_p4 = aux_y_interm_p4 + (aux_y-p4_y) * i/stepsNum;
        float p_aux_Z_p4 = aux_z_interm_p4;
        cinematic(p_aux_X_p4, p_aux_Y_p4, p_aux_Z_p4, &p4_a1, &p4_a2, &p4_a3);

        
        float p_aux_X_p6 = aux_x_interm_p6 + (aux_x-p6_x) * i/stepsNum;
        float p_aux_Y_p6 = aux_y_interm_p6 + (aux_y-p6_y) * i/stepsNum;
        float p_aux_Z_p6 = aux_z_interm_p6;
        cinematic(p_aux_X_p6, p_aux_Y_p6, p_aux_Z_p6, &p6_a1, &p6_a2, &p6_a3);
             
        setAngleServo(1, p6_a1);
        setAngleServo(7, p6_a2);
        setAngleServo(13, p6_a3);
        
        setAngleServo(5, p2_a1);
        setAngleServo(11, p2_a2);
        setAngleServo(17, p2_a3);
        
        setAngleServo(3, p4_a1);
        setAngleServo(9, p4_a2);
        setAngleServo(15, p4_a3);
        
        /*setAngleServo(1, p2_a1);
        setAngleServo(7, p2_a2);
        setAngleServo(13, p2_a3);
        
        setAngleServo(5, p4_a1);
        setAngleServo(11, p4_a2);
        setAngleServo(17, p4_a3);
        
        setAngleServo(3, p6_a1);
        setAngleServo(9, p6_a2);
        setAngleServo(15, p6_a3);*/
        
        delay(20);
      }

      
      p2_x = aux_x;
      p2_y = aux_y;
      p2_z = aux_z;
      
      p4_x = aux_x;
      p4_y = aux_y;
      p4_z = aux_z;
      
      p6_x = aux_x;
      p6_y = aux_y;
      p6_z = aux_z;

    }
//////////////////////////////////////////////////////#############################//////////////////////////////////////////////////////#############################//////////////////////////////////////////////////////#############################//////////////////////////////////////////////////////#############################
//////////////////////////////////////////////////////#############################//////////////////////////////////////////////////////#############################//////////////////////////////////////////////////////#############################//////////////////////////////////////////////////////#############################
//////////////////////////////////////////////////////#############################//////////////////////////////////////////////////////#############################//////////////////////////////////////////////////////#############################//////////////////////////////////////////////////////#############################

  }
}
