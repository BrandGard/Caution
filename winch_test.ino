int pinInput1 = 47;
int pinInput2 = 49;

void setup() {
  pinMode(pinInput1, OUTPUT);
  pinMode(pinInput2, OUTPUT);
  
}

void loop() {
  
    digitalWrite(pinInput1, HIGH);
    digitalWrite(pinInput2, LOW);
    
    delay(5000);

    digitalWrite(pinInput1, LOW);
    digitalWrite(pinInput2, HIGH);

    delay(5000);

    digitalWrite(pinInput2, LOW);

    delay(5000);
    
    
    

}

