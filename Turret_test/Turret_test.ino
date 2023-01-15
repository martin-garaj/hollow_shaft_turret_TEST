// LED pin
#define LED_PIN             13
// shield enable pin (active LOW)
#define SHIELD_EN_PIN       8
// step pin for X-axis
#define X_STP_PIN           2
// step pin for Y-axis
#define Y_STP_PIN           3
// direction pin for X-axis
#define X_DIR_PIN           5
// direction pin for Y-axis
#define Y_DIR_PIN           6

void setup() {
  // put your setup code here, to run once:

  // enable CNC shield
  pinMode(SHIELD_EN_PIN, OUTPUT);
  digitalWrite(SHIELD_EN_PIN, LOW);

  // set pin directions
  pinMode(X_STP_PIN, OUTPUT);
  pinMode(Y_STP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

}

int counter = 0;

void loop() {
  // put your main code here, to run repeatedly:

  if(counter == 10){
    digitalWrite(X_STP_PIN, HIGH);
    digitalWrite(Y_STP_PIN, HIGH);
    digitalWrite(LED_PIN, HIGH);
    delay(1);
    digitalWrite(X_STP_PIN, LOW);
    digitalWrite(Y_STP_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    counter = 0;
  }

  counter++;

  delay(1);
}
