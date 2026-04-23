#include <Arduino.h>
#include <math.h>
#include <arduinoFFT.h>
#include <algorithm>

const int      DAC_PIN     = 25;
const int      ADC_PIN     = 34;
const uint16_t N           = 256;
const uint16_t HALF        = N / 2;
const float    SAMPLE_FREQ = 2000.0f;
const uint32_t UART2_BAUD  = 921600;

float    vReal[N];
float    vImag[N];
uint8_t  sineTable[N];
ArduinoFFT<float> FFT(vReal, vImag, N, SAMPLE_FREQ);

// ---- Formas de onda -------------------------------------------------------
// Fs=2000 Hz, Nyquist=1000 Hz → bins útiles: 1..127
// Ciclos disponibles: 32→250Hz  64→500Hz  96→750Hz
enum WaveMode { SINE, SQUARE, HARMONICS };
WaveMode currentMode = SINE;
const char* modeNames[] = { "SENOIDAL 250Hz", "CUADRADA 250+750Hz", "ARMONICOS 250+500+750Hz" };

void buildWaveTable(WaveMode mode) {
    for (int i = 0; i < N; i++) {
        float t   = (float)i / N;
        float val = 0.0f;
        switch (mode) {
            case SINE:
                val = sinf(2.0f * PI * 32 * t);
                break;
            case SQUARE:
                // Onda cuadrada bandlimitada: solo armónicos impares < Nyquist
                // 250 Hz (k=32) + 750 Hz (k=96, amplitud 1/3)
                val  = sinf(2.0f * PI * 32 * t);
                val += (1.0f / 3.0f) * sinf(2.0f * PI * 96 * t);
                val /= (1.0f + 1.0f / 3.0f);   // normalizar a ±1
                break;
            case HARMONICS:
                // Tono rico: fundamental + 2do + 3er armónico
                // 250 Hz (k=32) + 500 Hz (k=64) + 750 Hz (k=96)
                val  =        sinf(2.0f * PI * 32 * t);
                val += 0.5f * sinf(2.0f * PI * 64 * t);
                val += 0.25f * sinf(2.0f * PI * 96 * t);
                val /= (1.0f + 0.5f + 0.25f);  // normalizar a ±1
                break;
        }
        sineTable[i] = (uint8_t)((val + 1.0f) * 127.5f);
    }
    Serial.printf("Modo: %s\n", modeNames[mode]);
}

// ---- Protocolo binario ----------------------------------------------------
#pragma pack(push,1)
struct PacketHeader { uint16_t sync, n, k, seq; float etot; };
struct BinPayload   { uint16_t index; int16_t re_q15, im_q15; };
struct PacketTail   { uint16_t checksum; };
#pragma pack(pop)

static uint16_t   seqCounter = 0;
static float      binEnergy[N];
static uint16_t   sortedIdx[HALF];
static BinPayload binBuf[HALF];

// --------------------------------------------------------------------------
void captureSamples() {
    for (int i = 0; i < N; i++) {
        dacWrite(DAC_PIN, sineTable[i]);
        delayMicroseconds(500);
        vReal[i] = (float)analogRead(ADC_PIN);
        vImag[i] = 0.0f;
    }
}

void removeDC() {
    float mean = 0.0f;
    for (int i = 0; i < N; i++) mean += vReal[i];
    mean /= N;
    for (int i = 0; i < N; i++) vReal[i] -= mean;
}

static inline int16_t toQ15(float v, float scale) {
    float n = v / scale;
    if (n >  1.0f) n =  1.0f;
    if (n < -1.0f) n = -1.0f;
    return (int16_t)(n * 32767.0f);
}

uint16_t transmitPacket() {
    float etot = 0.0f;
    for (uint16_t k = 1; k < HALF; k++) {
        binEnergy[k] = vReal[k]*vReal[k] + vImag[k]*vImag[k];
        sortedIdx[k-1] = k;
        etot += binEnergy[k];
    }
    const uint16_t validCount = HALF - 1;
    std::sort(sortedIdx, sortedIdx + validCount, [](uint16_t a, uint16_t b) {
        return binEnergy[a] > binEnergy[b];
    });
    float accumulated = 0.0f;
    uint16_t numBins = 0;
    while (numBins < validCount && accumulated < 0.95f * etot) {
        accumulated += binEnergy[sortedIdx[numBins]];
        numBins++;
    }
    float maxAbs = 1.0f;
    for (uint16_t i = 0; i < numBins; i++) {
        float r = fabsf(vReal[sortedIdx[i]]);
        float m = fabsf(vImag[sortedIdx[i]]);
        if (r > maxAbs) maxAbs = r;
        if (m > maxAbs) maxAbs = m;
    }
    PacketHeader hdr = { 0xA55A, N, numBins, seqCounter++, etot };
    uint16_t chk = 0;
    const auto xorB = [&chk](const uint8_t* p, size_t len) {
        for (size_t i = 0; i < len; i++) chk ^= p[i];
    };
    xorB((uint8_t*)&hdr, sizeof(hdr));
    for (uint16_t i = 0; i < numBins; i++) {
        binBuf[i] = { sortedIdx[i], toQ15(vReal[sortedIdx[i]], maxAbs), toQ15(vImag[sortedIdx[i]], maxAbs) };
        xorB((uint8_t*)&binBuf[i], sizeof(BinPayload));
    }
    PacketTail tail = { chk };
    Serial2.write((uint8_t*)&hdr,   sizeof(hdr));
    Serial2.write((uint8_t*)binBuf, numBins * sizeof(BinPayload));
    Serial2.write((uint8_t*)&tail,  sizeof(tail));
    return numBins;
}

// ---- Setup & Loop ---------------------------------------------------------
void setup() {
    Serial.begin(115200);
    Serial2.begin(UART2_BAUD, SERIAL_8N1, 16, 17);
    delay(1000);
    buildWaveTable(currentMode);
    Serial.println("Comandos: s=senoidal  q=cuadrada  h=armonicos");
}

void loop() {
    // Cambio de forma de onda por comando Serial
    if (Serial.available()) {
        char c = (char)Serial.read();
        if      (c == 's') { currentMode = SINE;      buildWaveTable(SINE); }
        else if (c == 'q') { currentMode = SQUARE;    buildWaveTable(SQUARE); }
        else if (c == 'h') { currentMode = HARMONICS; buildWaveTable(HARMONICS); }
    }

    captureSamples();
    removeDC();
    FFT.compute(FFTDirection::Forward);
    uint16_t k = transmitPacket();
    FFT.complexToMagnitude();
    Serial.printf("[%s] seq=%u bins=%u peak=%.1fHz\n",
                  modeNames[currentMode], seqCounter - 1, k, FFT.majorPeak());
    delay(50);
}
