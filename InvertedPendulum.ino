#define ENCODER_OPTIMIZE_INTERRUPTS

#include <AccelStepper.h>
#include <PID_v1.h>
#include <Encoder.h>

//Déclaration de variable
double stepsPerMM, setPoint, input, output, coef;
double kp, ki, kd;
long counter = 0;
double ang = 0;

//Déclaration de constante
const int stepMode = 2;
const int stepModes[6][5] = {
  {5, 1, 0, 0, 0},
  {10, 2, 1, 0, 0},
  {20, 4, 0, 1, 0},
  {40, 8, 1, 1, 0},
  {80, 16, 0, 0, 1},
  {160, 32, 1, 1, 1}
};

//Déclaration des pin
int encodeurA = 2;
int encodeurB = 3;
int directionPin = 8;
int stepPin = 9;
int stepperMode2 = 10;
int stepperMode1 = 11;
int stepperMode0 = 12;
int indicateurLED = 13;

AccelStepper stepper(AccelStepper::DRIVER, stepPin, directionPin);
PID pid(&input, &output, &setPoint, kp, ki, kd, DIRECT);
Encoder encoder(encodeurA, encodeurB);

void setup() {
  kp = 300.00;
  ki = 1500.00;
  kd = 0.00;
  coef = -1/100;
  
  pid = PID(&input, &output, &setPoint, kp, ki, kd, DIRECT);
  pid.SetOutputLimits(-10000000, 10000000);

  //Mofidication de l'objet stepper
  stepsPerMM = stepModes[stepMode][0];
  stepper.setMaxSpeed(10000000);
  stepper.setMinPulseWidth(5);

  //Initialisation des grandeur de pas (full -> 1/32)
  pinMode(stepperMode0, OUTPUT);
  pinMode(stepperMode1, OUTPUT);
  pinMode(stepperMode2, OUTPUT);
  digitalWrite(stepperMode0, stepModes[stepMode][2]);
  digitalWrite(stepperMode1, stepModes[stepMode][3]);
  digitalWrite(stepperMode2, stepModes[stepMode][4]);

  //Fermé LED indicateur
  pinMode(indicateurLED, OUTPUT);
  digitalWrite(indicateurLED, LOW);
  
  //Prendre point de référence (direcetment vers le bas
  setPoint = 0;
  pid.SetMode(AUTOMATIC);

  //Allumé LED indicateur (Prêt!)
  digitalWrite(13, HIGH);  
  encoder.write(0);

  //Boucle pour attendre d'être mis exactement à 180deg avant de commencé le balancement
  while(true){
    int count = encoder.read();
    double ang = count * (360.00 / 2000.00);
    if (ang == 180 or ang == -180){
      break;
    } //Fin if
  } //Fin while
} // Fin setup

void loop() {
  while(abs(input) < 250) {
    //Lecture du pas de l'encodeur
    int count = encoder.read();
    //Coversion en deg
    double ang = count * (360.00 / 2000.00);
    //Calcule d'ajustement
    input = coef*stepper.currentPosition()/stepsPerMM + 500*sin(ang*(PI/180));
    pid.Compute();
    stepper.setSpeed(output);
    stepper.runSpeed();
  } //Fin while
} //Fin loop
