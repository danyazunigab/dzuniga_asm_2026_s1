// Encoder

volatile long encoderCount = 0;

// Posiciones de pisos

const long PISO1 = 10;
const long PISO2 = 105;
const long PISO3 = 195;
const long PISO4 = 285;
const long PISO5 = 380;

// Setpoint

long setpoint = PISO1;

// Home

#define TRIG_PIN 13
#define ECHO_PIN A0

const float HOME_DISTANCE_CM = 1.0;
const float HOME_TOLERANCE_CM = 1.0;

// L298N

#define ENA 5
#define IN1 7
#define IN2 6

// Botones

#define BTN1 8
#define BTN2 9
#define BTN3 10
#define BTN4 11
#define BTN5 12

// Encoder

#define ENCODER_A 2
#define ENCODER_B 3

float Kp = 0.18;
float Ki = 0.0005;
float Kd = 0.45;

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

float leerDistancia()
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);

    digitalWrite(TRIG_PIN, LOW);

    long duracion =
        pulseIn(
            ECHO_PIN,
            HIGH,
            25000
        );

    if(duracion == 0)
        return 999;

    return duracion * 0.0343 / 2.0;
}

void actualizarHome()
{
    static uint8_t contador = 0;

    float distancia =
        leerDistancia();

    bool enHome =
        abs(
            distancia -
            HOME_DISTANCE_CM
        ) < HOME_TOLERANCE_CM;

    if(enHome)
    {
        contador++;

        if(contador >= 3)
        {
            noInterrupts();
            encoderCount = 0;
            interrupts();
            detenerMotor();

            contador = 3;
        }
    }
    else
    {
        contador = 0;
    }
}
bool estaEnHome()
{
    float distancia = leerDistancia();

    return abs(
        distancia -
        HOME_DISTANCE_CM
    ) < HOME_TOLERANCE_CM;
}

void subirMotor(int pwm)
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  analogWrite(ENA, pwm);
}

void bajarMotor(int pwm)
{
    if(estaEnHome())
    {
        detenerMotor();
        return;
    }

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

  if (!digitalRead(BTN4))
    setpoint = PISO4;

  if (!digitalRead(BTN5))
    setpoint = PISO5;
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

    if(distancia < 3)
    {
        detenerMotor();

        integral = 0;

        return;
    }

if(distancia > 80)
{
    if(error > 0)
        subirMotor(240);
    else
        bajarMotor(240);

    return;
}

if(distancia > 20)
{
    if(error > 0)
        subirMotor(140);
    else
        bajarMotor(140);

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
            140,
            255
        );

    if(control > 0)
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
  pinMode(BTN4, INPUT_PULLUP);
  pinMode(BTN5, INPUT_PULLUP);

pinMode(
    TRIG_PIN,
    OUTPUT
);

pinMode(
    ECHO_PIN,
    INPUT
);

 digitalWrite(TRIG_PIN, LOW);

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
    Serial.println("Ascensor iniciado");

    // Si al arrancar ya está en el piso 1,
    // sincronizamos inmediatamente
    float distancia = leerDistancia();

    if(abs(distancia - HOME_DISTANCE_CM) < HOME_TOLERANCE_CM)
    {
        noInterrupts();
        encoderCount = 0;
        interrupts();

        Serial.println("HOME detectado");
    }
}

void loop()
{
//    actualizarHome();
//
//    leerBotones();
//
//    controlarAscensor();
//
//  static unsigned long t = 0;
//
//  if (millis() - t > 100)
//  {
//    t = millis();
//
//    Serial.print("POS: ");
//
//    Serial.print(
//        encoderCount);
//
//    Serial.print(" SP: ");
//
//    Serial.print(
//        setpoint);
//
//    Serial.print(" ERR: ");
//
//    Serial.println(
//        setpoint -
//        encoderCount);
//  }
detenerMotor();
delay(2000);
Serial.print(encoderCount);
subirMotor(255);
}
