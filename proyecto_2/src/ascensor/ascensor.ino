// Encoder

volatile long encoderCount = 0;

// Posiciones de pisos

const long PISO1 = 0;
const long PISO2 = 5000;
const long PISO3 = 10000;

// Setpoint

long setpoint = PISO1;

// Home

#define HOME_SWITCH 12

// L298N

#define ENA 5
#define IN1 6
#define IN2 7

// Botones

#define BTN1 8
#define BTN2 9
#define BTN3 10

// Encoder

#define ENCODER_A 2
#define ENCODER_B 3

float Kp = 0.10;
float Ki = 0.0005;
float Kd = 0.20;

float integral = 0;
float prevError = 0;

void encoderISR()
{
  if (digitalRead(ENCODER_A) ==
      digitalRead(ENCODER_B))
  {
    encoderCount++;
  }
  else
  {
    encoderCount--;
  }
}

void subirMotor(int pwm)
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  analogWrite(ENA, pwm);
}

void bajarMotor(int pwm)
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  analogWrite(ENA, pwm);
}

void detenerMotor()
{
  analogWrite(ENA, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void leerBotones()
{
  if (!digitalRead(BTN1))
    setpoint = PISO1;

  if (!digitalRead(BTN2))
    setpoint = PISO2;

  if (!digitalRead(BTN3))
    setpoint = PISO3;
}

void homing()
{
  bajarMotor(80);

  while (digitalRead(HOME_SWITCH))
  {
    delay(1);
  }

  detenerMotor();

  encoderCount = 0;

  delay(300);
}

float calcularPID()
{
  long error =
      setpoint - encoderCount;

  integral += error;

  integral =
      constrain(
          integral,
          -50000,
          50000);

  float derivada =
      error - prevError;

  float salida =
      Kp * error +
      Ki * integral +
      Kd * derivada;

  prevError = error;

  return salida;
}

void controlarAscensor()
{
  long error =
      setpoint - encoderCount;

  long distancia =
      abs(error);

  if (distancia < 50)
  {
    detenerMotor();

    integral = 0;

    return;
  }

  //--------------------------------
  // MUY LEJOS
  //--------------------------------

  if (distancia > 2000)
  {
    if (error > 0)
      subirMotor(255);
    void loop()
    {
      leerBotones();

      controlarAscensor();

      static unsigned long t = 0;

      if (millis() - t > 100)
      {
        t = millis();

        Serial.print("POS: ");

        Serial.print(
            encoderCount);

        Serial.print(" SP: ");

        Serial.print(
            setpoint);

        Serial.print(" ERR: ");

        Serial.println(
            setpoint -
            encoderCount);
      }
    }
    else bajarMotor(255);

    return;
  }

  //--------------------------------
  // DISTANCIA MEDIA
  //--------------------------------

  if (distancia > 500)
  {
    if (error > 0)
      subirMotor(180);
    else
      bajarMotor(180);

    return;
  }

  //--------------------------------
  // PID FINO
  //--------------------------------

  float control =
      calcularPID();

  int pwm =
      abs(control);

  pwm =
      constrain(
          pwm,
          70,
          200);

  if (control > 0)
    subirMotor(pwm);
  else
    bajarMotor(pwm);
}

void setup()
{
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);
  pinMode(BTN3, INPUT_PULLUP);

  pinMode(HOME_SWITCH,
          INPUT_PULLUP);

  pinMode(ENCODER_A,
          INPUT_PULLUP);

  pinMode(ENCODER_B,
          INPUT_PULLUP);

  attachInterrupt(
      digitalPinToInterrupt(
          ENCODER_A),
      encoderISR,
      CHANGE);

  Serial.begin(115200);

  homing();
}

void loop()
{
  leerBotones();

  controlarAscensor();

  static unsigned long t = 0;

  if (millis() - t > 100)
  {
    t = millis();

    Serial.print("POS: ");

    Serial.print(
        encoderCount);

    Serial.print(" SP: ");

    Serial.print(
        setpoint);

    Serial.print(" ERR: ");

    Serial.println(
        setpoint -
        encoderCount);
  }
}