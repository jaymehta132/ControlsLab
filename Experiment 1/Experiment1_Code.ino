int inputPin = A0;
int output = 0;

int pinRed = 10;
int pinBlack = 11;

void antiClockWise(int motorSpeed = 255){
  analogWrite(pinBlack, motorSpeed);
  analogWrite(pinRed, 0);
}


void clockWise(int motorSpeed = 255){
  analogWrite(pinBlack, 0);
  analogWrite(pinRed, motorSpeed);
}

void halt(int delay_value = 1000){
  analogWrite(pinBlack, 0);
  analogWrite(pinRed, 0);
  delay(delay_value);
}

void setup(){
  // put your setup code here, to run once:
  pinMode(pinBlack, OUTPUT);
  pinMode(pinRed, OUTPUT);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  //o0utput = analogRead(inputPin);
  //Serial.println(output);
  output = analogRead(inputPin);
  Serial.println(output);
  clockWise(255);
  delay(1000);
  halt(1000);
  antiClockWise(255);
  delay(1000);
  halt(1000);
  
  
}
