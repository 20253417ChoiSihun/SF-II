#define PIN_LED 13
unsigned int count, toggle;

void setup() {
  Serial.begin(115200);
  while (!Serial){
    ; // wait for serial port to connect.
  }
  Serial.println("Hello World!");
  count=toggle=0;
  digitalWrite(PIN_LED, toggle);
  // put your setup code here, to run once:
   pinMode(PIN_LED, OUTPUT);

}

void loop() {
  Serial.println(++count);
  toggle=toggle_state(toggle); 
  digitalWrite(PIN_LED, toggle);
  delay(1000);
  digitalWrite(PIN_LED,count);
  delay(1000);
  
  
  // put your main code here, to run repeatedly:

}

int toggle_state(int toggle) {
  return toggle;
}
