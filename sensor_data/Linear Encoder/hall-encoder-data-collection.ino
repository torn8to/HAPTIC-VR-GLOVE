int sinPin = 25;
int cosPin = 26;

int n = 0;

void setup() {
  pinMode(sinPin, INPUT);
  pinMode(cosPin, INPUT);

  Serial.begin(115200);
  if(csv) {
    Serial.println("Reading Number,Sin Reading,Cos Reading");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(n);
  Serial.print(",");
  Serial.print(analogRead(sinPin));
  Serial.print(",");
  Serial.println(analogRead(cosPin));
  n++; 
  delay(100);

}
