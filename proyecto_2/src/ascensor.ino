// -------------------------
// Pines
// -------------------------

#define POT_PIN A0

#define ENA 5
#define IN1 7
#define IN2 8

#define BTN_P1 2
#define BTN_P2 3
#define BTN_P3 4

// -------------------------
// Pisos (calibrar)
// -------------------------

const int pisoADC[3] =
    {
        120, // Piso 1
        520, // Piso 2
        920  // Piso 3
};

// -------------------------
// PID
// -------------------------

float Kp = 1.8;
float Ki = 0.02;
float Kd = 0.15;

float error = 0;
float errorAnterior = 0;
float integral = 0;

// -------------------------
// Control
// -------------------------

int setpoint = pisoADC[0];

const int TOLERANCIA = 8;

// Zona donde empieza PID
const int ZONA_PID = 120;

// PWM mínimo para vencer fricción
const int PWM_MIN = 60;

// PWM rápido
const int PWM_RAPIDO = 255;

// PWM medio
const int PWM_MEDIO = 180;

void moverMotor(int pwm)
{
  pwm = constrain(abs(pwm), 0, 255);

  analogWrite(ENA, pwm);
}

void subirMotor(int pwm)
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  moverMotor(pwm);
}

void bajarMotor(int pwm)
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  moverMotor(pwm);
}

void detenerMotor()
{
  analogWrite(ENA, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void leerBotones()
{
  if (!digitalRead(BTN_P1))
    setpoint = pisoADC[0];

  if (!digitalRead(BTN_P2))
    setpoint = pisoADC[1];

  if (!digitalRead(BTN_P3))
    setpoint = pisoADC[2];
}

float calcularPID(float posicion)
{
  error = setpoint - posicion;

  integral += error;

  integral = constrain(integral,
                       -3000,
                       3000);

  float derivada =
      error - errorAnterior;

  float salida =
      Kp * error +
      Ki * integral +
      Kd * derivada;

  errorAnterior = error;

  return salida;
}

void controlarAscensor()
{
  int posicion =
      analogRead(POT_PIN);

  int distancia =
      abs(setpoint - posicion);

  // Llegó al piso
  if (distancia <= TOLERANCIA)
  {
    detenerMotor();

    integral = 0;

    return;
  }

  // -------------------------
  // Muy lejos
  // Máxima velocidad
  // -------------------------

  if (distancia > 300)
  {
    if (posicion < setpoint)
      subirMotor(PWM_RAPIDO);
    else
      bajarMotor(PWM_RAPIDO);

    return;
  }

  // -------------------------
  // Distancia media
  // -------------------------

  if (distancia > ZONA_PID)
  {
    if (posicion < setpoint)
      subirMotor(PWM_MEDIO);
    else
      bajarMotor(PWM_MEDIO);

    return;
  }

  // -------------------------
  // PID fino
  // -------------------------

  float control =
      calcularPID(posicion);

  int pwm =
      abs(control);

  pwm += PWM_MIN;

  pwm = constrain(
      pwm,
      PWM_MIN,
      255);

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

  pinMode(BTN_P1, INPUT_PULLUP);
  pinMode(BTN_P2, INPUT_PULLUP);
  pinMode(BTN_P3, INPUT_PULLUP);

  Serial.begin(115200);
}

void loop()
{
  leerBotones();

  controlarAscensor();

  static unsigned long t = 0;

  if (millis() - t > 100)
  {
    t = millis();

    Serial.print("SP: ");
    Serial.print(setpoint);

    Serial.print(" POS: ");
    Serial.println(
        analogRead(POT_PIN));
  }
}