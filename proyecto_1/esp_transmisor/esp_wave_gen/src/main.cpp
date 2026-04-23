#include <Arduino.h>
#include <math.h>
#include <arduinoFFT.h>
#include <algorithm>

const int      DAC_PIN     = 25;
const int      ADC_PIN     = 34;
const uint16_t N           = 256;
const uint16_t HALF        = N / 2;          // 128
const float    SAMPLE_FREQ = 2000.0f;        // 1/500µs = 2 kHz
const uint32_t UART2_BAUD  = 921600;

float    vReal[N];
float    vImag[N];
uint8_t  sineTable[N];
ArduinoFFT<float> FFT(vReal, vImag, N, SAMPLE_FREQ);

// ---- Protocolo binario ---------------------------------------------------
#pragma pack(push,1)
struct PacketHeader {
    uint16_t sync;   // 0xA55A  (transmitido como 0x5A 0xA5, little-endian)
    uint16_t n;      // 256
    uint16_t k;      // bins enviados
    uint16_t seq;    // contador de bloque
    float    etot;   // energía total
};
struct BinPayload {
    uint16_t index;
    int16_t  re_q15;
    int16_t  im_q15;
};
struct PacketTail {
    uint16_t checksum;
};
#pragma pack(pop)

static uint16_t   seqCounter = 0;
static float      binEnergy[N];    // energía por bin FFT
static uint16_t   sortedIdx[HALF]; // bins 1..HALF-1 ordenados por energía
static BinPayload binBuf[HALF];    // payload a transmitir

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

// Retorna cantidad de bins enviados
uint16_t transmitPacket() {
    // Energía por bin: sólo mitad útil (bins 1..HALF-1, sin DC ni Nyquist)
    float etot = 0.0f;
    for (uint16_t k = 1; k < HALF; k++) {
        binEnergy[k] = vReal[k]*vReal[k] + vImag[k]*vImag[k];
        sortedIdx[k-1] = k;
        etot += binEnergy[k];
    }

    // Ordenar por energía descendente
    const uint16_t validCount = HALF - 1;
    std::sort(sortedIdx, sortedIdx + validCount, [](uint16_t a, uint16_t b) {
        return binEnergy[a] > binEnergy[b];
    });

    // Seleccionar bins hasta cubrir 95% de la energía total
    float accumulated = 0.0f;
    uint16_t numBins = 0;
    while (numBins < validCount && accumulated < 0.95f * etot) {
        accumulated += binEnergy[sortedIdx[numBins]];
        numBins++;
    }

    // Escala Q15: máximo valor absoluto entre los bins seleccionados
    float maxAbs = 1.0f;
    for (uint16_t i = 0; i < numBins; i++) {
        float r = fabsf(vReal[sortedIdx[i]]);
        float m = fabsf(vImag[sortedIdx[i]]);
        if (r > maxAbs) maxAbs = r;
        if (m > maxAbs) maxAbs = m;
    }

    // Construir header
    PacketHeader hdr = { 0xA55A, N, numBins, seqCounter++, etot };

    // Checksum XOR acumulativo: header + bins
    uint16_t chk = 0;
    const auto xorBytes = [&chk](const uint8_t* p, size_t len) {
        for (size_t i = 0; i < len; i++) chk ^= p[i];
    };
    xorBytes((uint8_t*)&hdr, sizeof(hdr));

    for (uint16_t i = 0; i < numBins; i++) {
        binBuf[i] = {
            sortedIdx[i],
            toQ15(vReal[sortedIdx[i]], maxAbs),
            toQ15(vImag[sortedIdx[i]], maxAbs)
        };
        xorBytes((uint8_t*)&binBuf[i], sizeof(BinPayload));
    }

    PacketTail tail = { chk };

    // Transmitir
    Serial2.write((uint8_t*)&hdr,   sizeof(hdr));
    Serial2.write((uint8_t*)binBuf, numBins * sizeof(BinPayload));
    Serial2.write((uint8_t*)&tail,  sizeof(tail));

    return numBins;
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(UART2_BAUD, SERIAL_8N1, 16, 17);
    delay(1000);

    // 32 ciclos en 256 muestras → f = 32 * 2000 / 256 = 250 Hz
    for (int i = 0; i < N; i++) {
        sineTable[i] = (uint8_t)((sinf(2.0f * PI * 32 * i / N) + 1.0f) * 127.5f);
    }

    Serial.println("TX | N=256 Fs=2kHz f=250Hz UART2@921600");
}

void loop() {
    captureSamples();
    removeDC();
    FFT.compute(FFTDirection::Forward);

    uint16_t k = transmitPacket();

    // Debug (complexToMagnitude modifica vReal in-place, usar después de transmitir)
    FFT.complexToMagnitude();
    Serial.printf("seq=%u bins=%u freq=%.1fHz\n", seqCounter - 1, k, FFT.majorPeak());

    delay(50);
}
