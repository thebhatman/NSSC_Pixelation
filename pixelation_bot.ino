int R_MOTOR_POS = 8;
int R_MOTOR_NEG = 9;
int L_MOTOR_POS = 11;
int L_MOTOR_NEG = 12;
int delay_forward = 125;
int delay_turn = 125;

void setup()
{
  pinMode(L_MOTOR_POS, OUTPUT);
  pinMode(L_MOTOR_NEG, OUTPUT);
  pinMode(R_MOTOR_POS, OUTPUT);
  pinMode(R_MOTOR_NEG, OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
  Serial.begin(9600);
}

void stop() {
    digitalWrite(R_MOTOR_POS,HIGH);
    digitalWrite(R_MOTOR_NEG,HIGH);
    
    digitalWrite(L_MOTOR_POS,HIGH);
    digitalWrite(L_MOTOR_NEG,HIGH);
}

void forward() {
    digitalWrite(R_MOTOR_POS,HIGH);
    digitalWrite(R_MOTOR_NEG,LOW);
    
    digitalWrite(L_MOTOR_POS,LOW);
    digitalWrite(L_MOTOR_NEG,HIGH);
    
    delay(delay_forward);
    stop();
}

void right() {
    digitalWrite(R_MOTOR_POS,LOW);
    digitalWrite(R_MOTOR_NEG,LOW);
    
    digitalWrite(L_MOTOR_POS,LOW);
    digitalWrite(L_MOTOR_NEG,HIGH);
    
    delay(delay_turn);
    stop();
}

void left() {
    digitalWrite(R_MOTOR_POS,HIGH);
    digitalWrite(R_MOTOR_NEG,LOW);
    
    digitalWrite(L_MOTOR_POS,LOW);
    digitalWrite(L_MOTOR_NEG,LOW);
    
    delay(delay_turn);
    stop();
}

void back() {
    digitalWrite(R_MOTOR_POS,LOW);
    digitalWrite(R_MOTOR_NEG,HIGH);
    
    digitalWrite(L_MOTOR_POS,HIGH);
    digitalWrite(L_MOTOR_NEG,LOW);
    
    delay(delay_forward);
    stop();
}

void blink() {
  digitalWrite(LED_BUILTIN,HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN,LOW);
}

void loop()
{
    int incomingByte = 0;

    if(Serial.available() > 0) {
       incomingByte = Serial.read();
       
       if(incomingByte == 'W') {
          forward();
          Serial.println("forward");
          delay(200);
          stop();

       }
       if(incomingByte == 'A') {
          left(); 
          Serial.println("left");
          delay(200);
          stop();
       }
       if(incomingByte == 'D') {
         right();
         Serial.println("right");
         delay(200);
         stop();
       }
       if(incomingByte == 'S') {
          back(); 
          Serial.println("back");
          delay(500);
          stop();
       }
       if(incomingByte == 'B') {
          blink();
          Serial.println("blink");
       }
    }
}
