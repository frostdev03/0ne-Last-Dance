const int ledPin = 8; 

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  Serial.begin(115200);
  digitalWrite(ledPin, HIGH); 
  Serial.println("LED OFF");
  delay(1000);                     
  digitalWrite(ledPin, LOW); 
  Serial.println("LED ON");  
  delay(5000);                     
}