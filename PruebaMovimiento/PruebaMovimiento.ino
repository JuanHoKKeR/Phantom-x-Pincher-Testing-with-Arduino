#include <ax12.h> //ax12 library allows to send DYNAMIXEL commands
#include <BioloidController.h>

BioloidController controller = BioloidController(1000000); //1000000 = bauds for control


const int effector = 5;
const int Servo4 = 4; 
const int Servo3 = 3;
const int Servo2 = 2;  
const int Servo1 = 1;
//--------------POSICION------------------
int getServoPosition(int servoID){
  int position;
  for(int i = 0; i < 10; i++){
    position = dxlGetPosition(servoID);
    if(position > -1){
      return position;
    }
  }
  Serial.print("Failed reading position of servo ");
  Serial.println(servoID);
  return -1;
}
//------------SERIAL ARRANQUE------------
boolean isReady(){
  delay(500); 
  if(Serial.available() == 0) {
    return false;}
  int inByte = Serial.read();
  if(inByte == 'r' || inByte == 'R'){
    return true;
  }
  return false;
}
//-----MODO POSICION Y AJUSTE SEGURO-----
void ensureServoInJointMode(int ServoID){
  if (dxlGetMode(ServoID) == 1){
      Serial.println("Servo is already in 'joint' mode.");
      return;   
  }
  Serial.println("Switching the servo to 'joint' mode."); 
  dxlSetJointMode(ServoID, 0, 1023);   
}
//-----------MOVER SERVO----------------
void moveServoToPosition(int position,int ID){
  dxlSetGoalPosition(ID, position); //Ajusta la posicion del Servo
  //waitServoPositionStablize(ID);  //Asegura que el servo no se este movimiendo actualmente
  delay(100);
}
void waitServoPositionStablize(int servoID){
  while(dxlGetMoving(servoID)){
    delay(200);
  }
}
//---------LECTURA VELOCIDAD------------
int readCurrentSpeed(int servoid){
  int speedValue;
  int retryCount = 5;
  do{
    speedValue = dxlGetSpeed(servoid);
    retryCount--;
  }while(speedValue < 0 || retryCount > 0);
  return speedValue;
}
//-----MOVIMIENTO USANDO BIOLOID-------------
void moveTo(unsigned int * pose, int motionDurationInMilliseconds){
    delay(100); // Recommanded pause
    controller.loadPose(pose);   // load the pose from FLASH, into the nextPose buffer
    controller.readPose();       // read in current servo positions to the curPose buffer
    controller.interpolateSetup(motionDurationInMilliseconds); 
    while(controller.interpolating > 0){  // do this while we have not reached our new pose
        controller.interpolateStep();     // move servos, if necessary. 
        delay(1);
    }  
}
//-----------------------------------------
void setup() {
  Serial.begin(9600);   
  Serial.println(".... COMUNICACION SERIAL ESTABLECIDA ...");
  Serial.println("#### WARNING: All servo can move with the instruction");
  Serial.println("Digita R para posicionar los servos a HOME"); 
  while(!isReady());
  Serial.println("INICIALIZANDO MOVIMIENTO!!");
  ensureServoInJointMode(2);
  ensureServoInJointMode(3);
  ensureServoInJointMode(4);
  HomePosition();
  Serial.println("... Home Position ...");
  Serial.println("...MENU DE OPCIONES...");
  Serial.println("A - Realiza Movimiento Base de Boiloid");
  Serial.println("B - Realiza Movimiento Simple");
  Serial.println("C - Seleccionar Servo y mover manualmente");
  Serial.println("X - Salir del programa");
}

// We assume we have 4 servos with consecutive IDs from 1 to 4
const int MaxServoID = 4;
const PROGMEM unsigned int rightPositions[] = {MaxServoID, 200, 300, 400, 500};
const PROGMEM unsigned int leftPositions[] = {MaxServoID, 1000, 900, 800, 700};

bool shouldMoveToTheLeft = true;

void loop() {
  
  if (Serial.available() > 0) {
    char command = Serial.read(); // Lee un carácter del puerto serial
    command = toupper(command);
    switch (command) {
      case 'A':
        Serial.println("Realizando Movimiento");
        if(shouldMoveToTheLeft){
          Serial.println("<<<< Moving all servos to the left");
          moveTo(leftPositions, 1000);
        }else{
          Serial.println(">>>> Moving all servos to the right");
          moveTo(rightPositions, 3000);
        }
        shouldMoveToTheLeft = !shouldMoveToTheLeft;
        break;
      
      case 'B':
        // Realiza la acción correspondiente al comando 'B'
        Serial.println("Realizando Movimiento Basico");
        Position1();
        break;

      case 'C':
        Serial.println("MODO MANUAL");
        ModeManual();
        break;

      case 'X':
        // Salir del programa
        Serial.println("Saliendo del programa");
        break;

      default:
        Serial.println("Comando no válido. Introduce un comando válido.");
        break;
    }
  }
}
  


void HomePosition(){
  int posicion=0;
  for(int i=1;i<=5;i++){
    posicion=getServoPosition(i);
    Serial.println(posicion);
    dxlSetGoalSpeed(i, 150);
    Serial.print("Moviendo el Servo: ");Serial.print(i);Serial.println(" A la poscicion 513");
    moveServoToPosition(512,i);
    posicion=getServoPosition(i);
    Serial.println(posicion);
    delay(500);
  }
}

void Position1(){
  HomePosition();
  dxlSetGoalSpeed(effector, 360);
  dxlSetGoalSpeed(Servo4, 150);
  dxlSetGoalSpeed(Servo3, 150);
  dxlSetGoalSpeed(Servo2, 150);
  dxlSetGoalSpeed(Servo1, 200);
  int cont=0;
  while(cont<3){
    moveServoToPosition(650,Servo2);
    delay(100);
    moveServoToPosition(375,Servo3);
    delay(100);
    moveServoToPosition(375,Servo4);
    moveServoToPosition(0,effector);
    delay(100);
    moveServoToPosition(600,Servo2);
    delay(100);
    moveServoToPosition(300,Servo3);
    delay(100);
    moveServoToPosition(250,Servo4);
    moveServoToPosition(1023,effector);
    delay(100);
    HomePosition();
    cont=cont+1;
  }
  
}

void ModeManual(){
  bool finish = false;
  while (!finish) {
    Serial.println("Seleccionar el Servo a mover...");
    int ServoID;
    int numeroLeido = LecturaNumero();
    ServoID = (numeroLeido > 0 && numeroLeido <= 5) ? numeroLeido : 1;
    Serial.print("Servo seleccionado: "); Serial.print(ServoID);
    dxlSetGoalSpeed(ServoID, 150);
    Serial.print("El servo está en la posición: ");
    Serial.println(getServoPosition(ServoID));
    Serial.println("Seleccionar Posición a Mover...");
    int posicion = LecturaNumero();
    posicion = (posicion > 0 && posicion <= 1023) ? posicion : 512;
    moveServoToPosition(posicion, ServoID);
    Serial.println("Continuar en Esta función (1), SALIR (0)");
    int opcion = LecturaNumero();
    finish = (opcion == 0); 
  }
}


int LecturaNumero() {
  while (true) {
    if (Serial.available() > 0) {
      int number = Serial.parseInt();
      if (Serial.read() == '\n') {
        return number;
      } else {
        Serial.println("Número no válido. Ingresa un número seguido de una nueva línea.");
      }
    }
  }
}

