#include <Arduino.h>
void setup() {
// write your initialization code here

    Serial.begin(15200);
}

void loop() {
// write your code here
    Serial.write("YIPPEE");
}