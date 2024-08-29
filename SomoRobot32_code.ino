// Motor pin definitions
const int RightMotorForward = 3;      // Right motor forward direction
const int RightMotorReverse = 2;      // Right motor reverse direction
const int LeftMotorForward = 5;       // Left motor forward direction
const int LeftMotorReverse = 4;       // Left motor reverse direction
// Sensor pin definitions
const int LeftLineSensor = 53;        // Left line sensor
const int RightLineSensor = 10;       // Right line sensor
const int LeftDistanceSensor = 13;    // Left distance sensor
const int SecondLeftSensor = 52;      // Second left distance sensor
const int FrontDistanceSensor = 12;   // Front distance sensor
const int RightDistanceSensor = 11;   // Right distance sensor
const int SecondRightSensor = 9;      // Second right distance sensor
// H-Bridge and Key definitions
const int LeftHBridge = 21;           // Left H-bridge control
const int RightHBridge = 8;           // Right H-bridge control
const int LeftKey = 25;               // Left key connection
const int CommonKey = 27;             // Common key connection
const int RightKey = 23;              // Right key connection
// Control Variables
const int MaxMotorSpeed = 255;        // Maximum motor speed
// STRATEGY_LATERAL = 1, STRATEGY_FORWARD = 2, STRATEGY_WAIT_LATERAL = 3, STRATEGY_WAIT_FORWARD = 4, STRATEGY_MOVE_INVERSE = 5
const int Strategy = 1;               // Strategy to use
// Our delay system variable
unsigned long currentMillis;          // Variable to track time

void setup() {
  // Initialize motor pins
  pinMode(RightMotorForward, OUTPUT);
  pinMode(RightMotorReverse, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorReverse, OUTPUT);
  pinMode(LeftHBridge, OUTPUT);
  pinMode(RightHBridge, OUTPUT);
  pinMode(CommonKey, OUTPUT);
    
  // Initialize sensor pins
  pinMode(LeftLineSensor, INPUT);
  pinMode(RightLineSensor, INPUT);
  pinMode(LeftDistanceSensor, INPUT);
  pinMode(SecondLeftSensor, INPUT);
  pinMode(FrontDistanceSensor, INPUT);
  pinMode(RightDistanceSensor, INPUT);
  pinMode(SecondRightSensor, INPUT);
  
  // Initialize key pins
  pinMode(LeftKey, INPUT);
  pinMode(RightKey, INPUT);
  
  // Set initial states for H-bridge and key connections
  digitalWrite(LeftHBridge, HIGH);
  digitalWrite(RightHBridge, HIGH);
  digitalWrite(CommonKey, HIGH);
  
  // Start serial communication and delay for initial setup
  Serial.begin(9600);
  delay(4110);  // Delay to start the robot after 5 seconds
}

void loop() {
  // Execute the selected strategy
  switch (Strategy) {  
    
    case 1: //STRATEGY_LATERAL
        Serial.println("Strategy 1");
        InitialMovement();
        while (true) {
         HandleReactions(SecondRightSensor, SecondLeftSensor, RightDistanceSensor, LeftDistanceSensor, FrontDistanceSensor, 60, 60);
        }             
    break;     
    case 2: //STRATEGY_FORWARD
        Serial.println("Strategy 2");
        InitialMovement();
        while (true) {
         HandleReactions(FrontDistanceSensor, SecondRightSensor, SecondLeftSensor, RightDistanceSensor, LeftDistanceSensor, 60, 60);
        }            
    break;    
    case 3: //STRATEGY_WAIT_LATERAL
        Serial.println("Strategy 3");
        InitialMovement();
        currentMillis = millis();
        while (millis() - currentMillis <= 3000) {
         
         HandleReactions(SecondRightSensor, SecondLeftSensor, RightDistanceSensor, LeftDistanceSensor, FrontDistanceSensor, 0, 0);
        }
        while (true) {
         HandleReactions(SecondRightSensor, SecondLeftSensor, RightDistanceSensor, LeftDistanceSensor, FrontDistanceSensor, 60, 60);
        }           
    break;      
    case 4: //STRATEGY_WAIT_FORWARD 
        Serial.println("Strategy 4");
        InitialMovement();
        currentMillis = millis();
        while (millis() - currentMillis <= 3000) {
         HandleReactions(FrontDistanceSensor, SecondRightSensor, SecondLeftSensor, RightDistanceSensor, LeftDistanceSensor, 0, 0);
        }
        while (true) {
         HandleReactions(FrontDistanceSensor, SecondRightSensor, SecondLeftSensor, RightDistanceSensor, LeftDistanceSensor, 60, 60);
        }           
    break;      
    case 5: //STRATEGY_GO_OUT_THE_RING
        Serial.println("Strategy 5");
        Move(-60, -60, 0);
        delay(1000);
        Move(0, 0, 0);
        delay(5000);        
    break;      
    default: //default for any error
        Serial.print("Error"); 
        //HandleReactions(SecondRightSensor, SecondLeftSensor, RightDistanceSensor, LeftDistanceSensor, FrontDistanceSensor, 60, 60);
  }
}

void InitialMovement() {
  // Read key connection signals
  int leftSignal = digitalRead(LeftKey);
  int rightSignal = digitalRead(RightKey);
  
  // Determine initial movement based on key signals
  if (leftSignal == HIGH && rightSignal == LOW) {
    Serial.println("Left_Signal");  // Code for turning right
    Move(-60, 60, 175);  // Turn right
  } else if (leftSignal == LOW && rightSignal == HIGH) {
    Serial.println("Right_Signal");  // Code for turning left
    Move(60, -60, 175);  // Turn left
  } else {
    Serial.println("Forward_Signal");  // Code for moving forward
    Move(60, 60, 200);   // Move forward
  }
}

void HandleReactions(int s1, int s2, int s3, int s4, int s5, int L, int R) { //This code allows me to easily change priorities between sensors(-_-) 
  StopMovement(60);
  // Determine robot behavior based on sensor readings
  if (digitalRead(FrontDistanceSensor) == HIGH && digitalRead(LeftDistanceSensor) == HIGH && digitalRead(RightDistanceSensor) == HIGH) {
    Move(60, 60, 0);  // Move forward if no obstacles detected
  } else if (digitalRead(LeftDistanceSensor) == HIGH && digitalRead(RightDistanceSensor) == HIGH) {
    Move(60, 60, 0);  // Move forward if obstacles detected on both sides
  } else if (digitalRead(s1) == HIGH) {
    Type_of_move(s1);        // Move depend on type of sensor s1 
  } else if (digitalRead(s2) == HIGH) {
    Type_of_move(s2);        // Move depend on type of sensor s2  
  } else if (digitalRead(s3) == HIGH) {
    Type_of_move(s3);        // Move depend on type of sensor s3     
  } else if (digitalRead(s4) == HIGH) {
    Type_of_move(s4);        // Move depend on type of sensor s4    
  } else if (digitalRead(s5) == HIGH) {
    Type_of_move(s5);        // Move depend on type of sensor s5      
  } else {
    Move(L, R, 0);           // default Move based on the given speed
  }
}

void Move(int L, int R, int Time) {
  // Control the robot's movement and speed
  if (Time == 0) {
    SetSpeed(L, R);
  } else {
    currentMillis = millis();
    while (millis() - currentMillis <= Time) {
      StopMovement(60);  // Temporarily stop movement
      SetSpeed(L, R);
    }
  }
}

void SetSpeed(int L, int R) {
  // Adjust motor speeds based on input values
  L = constrain(L, -100, 100);
  R = constrain(R, -100, 100);
  L = map(L, 0, 100, 0, MaxMotorSpeed);
  R = map(R, 0, 100, 0, MaxMotorSpeed);
  
  // Set right motor speed
  if (L >= 0) {
    analogWrite(RightMotorForward, 0);
    analogWrite(RightMotorReverse, L);
  } else {
    L = -L;
    analogWrite(RightMotorForward, L);
    analogWrite(RightMotorReverse, 0);
  }
  
  // Set left motor speed
  if (R >= 0) {
    analogWrite(LeftMotorForward, 0);
    analogWrite(LeftMotorReverse, R);
  } else {
    R = -R;
    analogWrite(LeftMotorForward, R);
    analogWrite(LeftMotorReverse, 0);
  }
}

void StopMovement(int Strength) {
  // Temporarily stop the robot based on line sensor readings
  if (digitalRead(LeftLineSensor) == LOW) {
    Move(-Strength, -Strength, 0);
    delay(350);
    Move(Strength, -Strength, 0);
    delay(600);
  } else if (digitalRead(RightLineSensor) == LOW) {
    Move(-Strength, -Strength, 0);
    delay(350);
    Move(-Strength, Strength, 0);
    delay(600);
  }
}

void Type_of_move(int s){ 
  //to determine the type sensor and move depending on that
  switch(s){
    case 13:
      Move(-100,100,0);
    break;
    case 11:
      Move(100,-100,0);
    break;
    case 52:
      Move(-100,100,300);
    break;
    case 9:
      Move(100,-100,300);
    break;
    default:
      Move(60,60,0);
  } 
}
