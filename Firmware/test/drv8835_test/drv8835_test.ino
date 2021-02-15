#define AIN1 6
#define AIN2 11
#define BIN1 10
#define BIN2 9


enum motor_mode {
  COAST,
  FORWARD_COAST,
  REVERSE_COAST,
  FORWARD_BREAK,
  REVERSE_BREAK,
  BREAK
};

enum motor_mode mode; 

void setup() {
  // put your setup code here, to run once:
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  mode = COAST; 
  moveMotor(1, mode, 125 );
  moveMotor(0, mode, 125 );

  delay (5000); 
  
  mode = FORWARD_COAST; 
  moveMotor(1, mode, 125 );
  moveMotor(0, mode, 125 );

  delay (5000); 
 
  mode = REVERSE_COAST; 
  moveMotor(1, mode, 125 );
  moveMotor(0, mode, 125 );

  delay (5000); 
  
  mode = FORWARD_BREAK; 
  moveMotor(1, mode, 125 );
  moveMotor(0, mode, 125 );

  delay (5000); 

  mode = REVERSE_BREAK; 
  moveMotor(1, mode, 125 );
  moveMotor(0, mode, 125 );

  delay (5000); 

  mode = BREAK; 
  moveMotor(1, mode, 125 );
  moveMotor(0, mode, 125 );

  delay (5000); 

  
}


//motorDirection - 1 Left - 0 - Right 
void moveMotor(bool motorDirection, uint8_t mode, uint8_t pwm){


    switch (mode) {
      
      case COAST:
        //left motor 
        if (motorDirection){
          analogWrite(AIN1, 0);
          analogWrite(AIN2, 0);
        }
        //right motor 
        else{
          analogWrite(BIN1, 0);
          analogWrite(BIN2, 0); 
        }
        break;
        
      case FORWARD_COAST:
        
        //left motor 
        if (motorDirection){
          analogWrite(AIN1, pwm);
          analogWrite(AIN2, 0);
        }
        //right motor 
        else{
          analogWrite(BIN1, pwm);
          analogWrite(BIN2, 0); 
        }
        break;
  
      case REVERSE_COAST:
        //left motor 
        if (motorDirection){
          analogWrite(AIN1, 0);
          analogWrite(AIN2, pwm);
        }
        //right motor 
        else{
          analogWrite(BIN1, 0);
          analogWrite(BIN2, pwm); 
        }
        break;
        
      case FORWARD_BREAK:
        //left motor 
        if (motorDirection){
          digitalWrite(AIN1, HIGH);
          analogWrite(AIN2, pwm);
        }
        //right motor 
        else{
          digitalWrite(BIN1, HIGH);
          analogWrite(BIN2, pwm); 
        }
        break;
        
      case REVERSE_BREAK:
        //left motor 
        if (motorDirection){
          analogWrite(AIN1, pwm);
          digitalWrite(AIN2, HIGH);
        }
        //right motor 
        else{
          analogWrite(BIN1, pwm);
          digitalWrite(BIN2, HIGH); 
        }
        break;

      case BREAK:
        //left motor 
        if (motorDirection){
          digitalWrite(AIN1, HIGH);
          digitalWrite(AIN2, HIGH);
        }
        //right motor 
        else{
          digitalWrite(BIN1, HIGH);
          digitalWrite(BIN2, HIGH); 
        }
        break;
        
    }  
}
