const int led = 13;
int usbRead = 0;
void setup() {
     Serial.begin(9600);
     while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
     }
      pinMode(led, OUTPUT);
     digitalWrite(led, LOW);
}
void loop () {
    if(Serial.available() ) {
      int c = Serial.parseInt();
      Serial.println(c);
    }
//        usbRead = Serial.read(), "0"  ; // when you send an integer over USB from Raspberry Pi to Arduino it will be in ASCI form and here will converted back to the integer
//       Serial.println(usbRead);
//       if(usbRead == 1 ) {
//           blink();
//           usbRead = 0;
       
}
void blink() {
     for(int x = 0; x <3 ; x ++ ) {
         digitalWrite(led, HIGH ) ;
         delay(1000);
         digitalWrite(led, LOW);
         delay(1000);
    }
}


