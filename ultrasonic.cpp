#define SOUND_MUL 0.0343  // Sound travels at 0.0343cm/us, to move back and forth from object takes 2x time (return trip also), hence we can multiply by 0.0343 or divide by 29.1
// Multiple HC-SR04, Single Interrupt: https://web.archive.org/web/20200728232718/http://kunoichi.be/projects/multiple-hc-sr04-with-arduino-using-only-1-interrupt-pin/
// Sharing echo pins https://arduino.stackexchange.com/questions/34174/using-multiple-hc-sr04-sharing-trigger-pin-and-interference-between-hc-sr04

unsigned long duration, cm, inches;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  // The echoPin will look for a HIGH pulse of 10 or more microseconds
  // The reason why the LOW pulse is there is to ensure HIGH pulse is easily detected

  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);  // PulseIn works if HIGH (Start) --D1--> LOW --D2--> HIGH --D3--> LOW (End), then it will only measure D3 and nothing else, output to unsigned long in microseconds

  cm = (duration / 2) * SOUND_MUL ; 
  // Can also use a array buffer to sprintf to, and then println the buffer at the cost of memory
  Serial.print(cm);
  Serial.print(" cm");
  Serial.println();

  delay(250);
}
