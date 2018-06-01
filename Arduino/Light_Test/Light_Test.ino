 
#define BLUE 3
#define RED 4
#define GREEN 5
#define OVER 6

int t = 1000;

void setup() {
  // put your setup code here, to run once:
  pinMode(BLUE, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(OVER, INPUT);
  digitalWrite(BLUE,HIGH);
  digitalWrite(RED,HIGH);
  digitalWrite(GREEN,HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(OVER)==LOW)
  {
    digitalWrite(BLUE,LOW);
    digitalWrite (RED,LOW);
    digitalWrite(GREEN,LOW);
    delay(t);
    digitalWrite(BLUE,LOW);
    digitalWrite(RED,HIGH);
    digitalWrite(GREEN,HIGH);
    delay(t);
    digitalWrite(BLUE,HIGH);
    digitalWrite(RED,LOW);
    digitalWrite(GREEN,HIGH);
    delay(t);
    digitalWrite(BLUE,HIGH);
    digitalWrite(RED,HIGH);
    digitalWrite(GREEN,LOW);
    delay(t);
  }
  digitalWrite(BLUE,HIGH);
  digitalWrite(RED,HIGH);
  digitalWrite(GREEN,HIGH);
  delay(100);
}
