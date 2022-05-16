//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 01 Test Driver L298N

int Motor_L = 11; //--> D11->IN4 (To adjust the direction of the left motor rotation)
int Motor_L_PWM = 10; //--> D10->IN3 (To set the left motor PWM value)
int Motor_R_PWM = 9; //--> D9->IN2 (To set the right motor PWM value)
int Motor_R = 8; //--> D8->IN1 (To adjust the direction of the right motor rotation)

byte PWM_Val; //--> Variable for the PWM value

void setup() {
  // put your setup code here, to run once:

  pinMode(Motor_L, OUTPUT);
  pinMode(Motor_R, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  //----------------------------------------Forward Motor Rotation Direction
  PWM_Val = 70;
  digitalWrite(Motor_L, LOW);
  analogWrite(Motor_L_PWM, PWM_Val);
//  digitalWrite(Motor_R, LOW);
//  analogWrite(Motor_R_PWM, PWM_Val);
  delay(2000);
  //----------------------------------------

  //----------------------------------------Backward Motor Rotation Direction
//  PWM_Val = 255-70;
//  digitalWrite(Motor_L, HIGH);
//  analogWrite(Motor_L_PWM, PWM_Val);
//  digitalWrite(Motor_R, HIGH);
//  analogWrite(Motor_R_PWM, PWM_Val);
//  delay(2000);
  //----------------------------------------
}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<