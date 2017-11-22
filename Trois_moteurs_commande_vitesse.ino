/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438

Ce  sketch créé la commande moteur sur les trois moteurs tour lidar et actions de Dubins
Connect a DC motor to M2 pour le lidar
Connect a DC motor to M3 et M4 pour la propulsion
On entre trois valeurs en retour du controleur qui correspondent à Valeur 1 == mode de rotation 0 en avant 1 en arrière 2 rotation psi positive 3 negative 4 rotation lidar 5 rotation lidar inverse.
LEs valeurs 2 et 3 sont codées sur -128 127 mini maxi (0 255) (10 bits).
On va chercher à reguler la vitesse de rotation du lidar à 1 hz soit un tour par seconde.
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h> 
unsigned long time;
int currentValue = 0;
int values[] = {0,0,0};
boolean bulturet = false ;
int turet_static = -70;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 


// Connect a DC motor to port M2 this is the turet motor for lidar sensors
Adafruit_DCMotor *turet_Motor = AFMS.getMotor(2);
// Connect a DC motor to port M3 this is the driving motor 1
Adafruit_DCMotor *left_driving_Motor = AFMS.getMotor(3);
// Connect a DC motor to port M4 this is the driving motor 2
Adafruit_DCMotor *right_driving_Motor = AFMS.getMotor(4);



void setup() {
  Serial.begin(115200);           // set up Serial library at 9600 bps
  time=millis();


   AFMS.begin();  // create with the default frequency 1.6KHz
// AFMS.begin(1000);  // OR with a different frequency, say 1KHz


}


void loop() {

   time=millis();

//  left_driving_Motor ->run(BACKWARD);
//  right_driving_Motor->run(FORWARD);
//  turet_Motor->run(FORWARD);

  if (Serial.available()){

  int incomingValue  = Serial.read();
  values[currentValue]= incomingValue;


  if(currentValue == 0){
      if(values[0] == 0){
      right_driving_Motor->run(FORWARD);
      left_driving_Motor ->run(BACKWARD);
      turet_Motor->run(FORWARD);
      bulturet = true ;
 }
      if(values[0] == 1){
      right_driving_Motor->run(BACKWARD);
      left_driving_Motor ->run(FORWARD);
      turet_Motor->run(FORWARD);
      bulturet = true ;      
 }
      if(values[0] == 2){
      right_driving_Motor->run(FORWARD);
      left_driving_Motor ->run(FORWARD);
      turet_Motor->run(FORWARD); 
      bulturet = true ;
 }
      if(values[0] == 3){
      right_driving_Motor->run(BACKWARD);
      left_driving_Motor ->run(BACKWARD);
      turet_Motor->run(FORWARD);
      bulturet = true ; 
 }
      if(values[0] == 4){
      turet_Motor->run(FORWARD);
      bulturet = true ;
      if (bulturet == true){ turet_Motor ->setSpeed(map(turet_static,-128,127,0,255));} 
 }
      if(values[0] == 5){
      turet_Motor->run(BACKWARD);
      bulturet = true ;
      if (bulturet == true){ turet_Motor ->setSpeed(map(turet_static,-128,127,0,255));}
 }
   }
  if(currentValue == 1){
         left_driving_Motor ->setSpeed(map(values[1],-128,127,0,255));
         if (bulturet == true){ turet_Motor ->setSpeed(map(turet_static,-128,127,0,255));}   
 
    }
  if(currentValue == 2){
         right_driving_Motor->setSpeed(map(values[2],-128,127,0,255));
         if (bulturet == true) {turet_Motor ->setSpeed(map(turet_static,-128,127,0,255));}  
  
    }
  currentValue++;
    
  if(currentValue > 2){
  currentValue =0;  
  }
 }
}
