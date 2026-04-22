#include <Arduino.h>
#include <math.h>
#include <arduinoFFT.h>

// --- Pines ---
const int DAC_PIN = 25;
const int ADC_PIN = 34;

// --- FFT ---
const uint16_t N = 256;
const float SAMPLE_FREQ = 2000.0f;  // 1 / 500µs = 2 kHz

float vReal[N];
float vImag[N];
uint8_t sineTable[N];

ArduinoFFT<float> FFT(vReal, vImag, N, SAMPLE_FREQ);

// --- UART2: comunicacion con el receptor ---
// TX=17, RX=16 (pines por defecto de UART2 en ESP32)
const uint32_t UART2_BAUD = 921600;

// Protocolo: header 0xAA 0xBB + vReal[N]*float + vImag[N]*float
const uint8_t FRAME_HEADER[2] = {0xAA, 0xBB};

// Captura N muestras del ADC sincronizadas con el DAC
void captureSamples() {
    for (int i = 0; i < N; i++) {
        dacWrite(DAC_PIN, sineTable[i]);
        delayMicroseconds(500);
        vReal[i] = (float)analogRead(ADC_PIN);
        vImag[i] = 0.0f;
    }
}

// Envia los coeficientes complejos de la FFT por UART2
// El receptor necesita ambas partes (real e imaginaria) para hacer IFFT
void transmitFrame() {
    Serial2.write(FRAME_HEADER, sizeof(FRAME_HEADER));
    Serial2.write((uint8_t*)vReal, N * sizeof(float));
    Serial2.write((uint8_t*)vImag, N * sizeof(float));
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(UART2_BAUD, SERIAL_8N1, 16, 17);
    delay(1000);

    // Tabla de seno precomputada: valores 0-255 para el DAC de 8 bits
    for (int i = 0; i < N; i++) {
        sineTable[i] = (uint8_t)((sinf(2.0f * PI * i / N) + 1.0f) * 127.5f);
    }

    Serial.println("Transmisor listo. N=256, Fs=2kHz, UART2@921600");
}

void loop() {
    // 1. Generar senoidal por DAC y capturar con ADC
    captureSamples();

    // 2. Eliminar DC offset (el ADC devuelve 0-4095, la media ~2048 domina bin 0)
    float mean = 0.0f;
    for (int i = 0; i < N; i++) mean += vReal[i];
    mean /= N;
    for (int i = 0; i < N; i++) vReal[i] -= mean;

    // 3. Calcular FFT (sin ventana: capturamos exactamente 1 ciclo entero por bloque)
    FFT.compute(FFTDirection::Forward);

    // 4. Transmitir coeficientes complejos al receptor antes de modificarlos
    transmitFrame();

    // 5. Debug: frecuencia dominante por Serial0
    FFT.complexToMagnitude();
    Serial.printf("Freq pico: %.2f Hz\n", FFT.majorPeak());

    delay(50);
}
