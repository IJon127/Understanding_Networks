/**************************************************************************
Joystick to Serial
To connect on a POSIX computer:
cat <serialport> | nc <server address> <server port>

Uses Tom Igoe's SerialJoystick: https://github.com/tigoe/BallDropGame/blob/main/SerialJoystick/SerialJoystick.ino

created 07 Oct 2022
modified 07 Oct 2022
by I-Jon Hsieh
**************************************************************************/

const int exitBtn = A2;

const int leftLED = 4;        
const int rightLED = 5;      
const int upLED = 6;          
const int downLED = 7;      


const int sendInterval = 50;     // minimum time between messages to the server
const int debounceInterval = 5;  // used to smooth out pushbutton readings

int lastButtonState = HIGH;  // previous state of the pushbutton
long lastTimeSent = 0;       // timestamp of the last server message

void setup() {
  Serial.begin(9600);

  pinMode(exitBtn, INPUT_PULLUP);
  pinMode(leftLED, OUTPUT);
  pinMode(rightLED, OUTPUT);
  pinMode(upLED, OUTPUT);
  pinMode(downLED, OUTPUT);
}

void loop() {

  if (millis() - lastTimeSent > sendInterval) {
    int xSensor = analogRead(A0);
    delay(1);
    int ySensor = analogRead(A1);

    // map x and y readings to a 3-point range
    // and subtract 1 to get -1 to 1, with
    // 0 at rest:
    xSensor = map(xSensor, 0, 1023, 0, 3) - 1;
    ySensor = map(ySensor, 0, 1023, 0, 3) - 1;

    switch (xSensor) {
      case -1:  //left
        
        Serial.print("l");
        digitalWrite(leftLED, HIGH);
        break;
      case 0:  // center
        digitalWrite(rightLED, LOW);
        digitalWrite(leftLED, LOW);

        break;
      case 1:  // right
        
        Serial.print("r");
        digitalWrite(rightLED, HIGH);
        break;
    }

    switch (ySensor) {
      case -1:  //up
        
        Serial.print("u");
        digitalWrite(upLED, HIGH);
        break;
      case 0:  // center
        digitalWrite(upLED, LOW);
        digitalWrite(downLED, LOW);
        break;
      case 1:  // down
        Serial.print("d");
        digitalWrite(downLED, HIGH);
        break;
    }
    //save this moment as last time you sent a message:
    lastTimeSent = millis();
  }



  int buttonState = digitalRead(exitBtn);

  // if the button changes state:
  if (buttonState != lastButtonState) {
    delay(debounceInterval);
    if (buttonState == LOW) {
      Serial.print("x"); // disconnect:
      Serial.print(0x1F); // send ctrl-C
    }
    
    lastButtonState = buttonState;
  }

}
