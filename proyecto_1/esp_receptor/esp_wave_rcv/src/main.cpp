#include <Arduino.h>
#include <arduinoFFT.h>

const int      DAC_PIN    = 25;
const uint16_t N          = 256;
const uint16_t HALF       = N / 2;
const float    SAMPLE_FREQ = 2000.0f;
const uint32_t UART2_BAUD = 921600;

float vReal[N];
float vImag[N];
ArduinoFFT<float> FFT(vReal, vImag, N, SAMPLE_FREQ);

// ---- Protocolo binario (idéntico al transmisor) --------------------------
#pragma pack(push,1)
struct PacketHeader {
    uint16_t sync;
    uint16_t n;
    uint16_t k;
    uint16_t seq;
    float    etot;
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

static BinPayload rxBins[HALF];

// --------------------------------------------------------------------------

// Lee exactamente `len` bytes de Serial2, bloqueante con timeout
bool readExact(uint8_t* buf, size_t len, uint32_t timeoutMs = 300) {
    uint32_t start = millis();
    size_t received = 0;
    while (received < len) {
        if (millis() - start > timeoutMs) return false;
        if (Serial2.available()) buf[received++] = (uint8_t)Serial2.read();
    }
    return true;
}

// Espera el par de bytes de sincronía: 0x5A 0xA5 (0xA55A en little-endian)
bool waitSync(uint32_t timeoutMs = 1000) {
    uint8_t prev = 0, cur = 0;
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        if (!Serial2.available()) continue;
        prev = cur;
        cur  = (uint8_t)Serial2.read();
        if (prev == 0x5A && cur == 0xA5) return true;
    }
    return false;
}

uint16_t xorChecksum(const uint8_t* data, size_t len, uint16_t init = 0) {
    for (size_t i = 0; i < len; i++) init ^= data[i];
    return init;
}

// Lee y valida un paquete completo (el sync ya fue detectado por waitSync)
bool receivePacket(PacketHeader& hdr, uint16_t& numBins) {
    // sync ya consumido; completar los 10 bytes restantes del header
    hdr.sync = 0xA55A;
    if (!readExact((uint8_t*)&hdr + 2, sizeof(hdr) - 2)) return false;

    if (hdr.n != N)           return false;  // bloque de tamaño incorrecto
    numBins = hdr.k;
    if (numBins == 0 || numBins >= HALF) return false;

    if (!readExact((uint8_t*)rxBins, numBins * sizeof(BinPayload))) return false;

    PacketTail tail;
    if (!readExact((uint8_t*)&tail, sizeof(tail))) return false;

    // Verificar integridad
    uint16_t chk = xorChecksum((uint8_t*)&hdr, sizeof(hdr));
    chk = xorChecksum((uint8_t*)rxBins, numBins * sizeof(BinPayload), chk);
    return chk == tail.checksum;
}

// Construye el espectro complejo con simetría conjugada
void buildSpectrum(uint16_t numBins) {
    for (int i = 0; i < N; i++) { vReal[i] = 0.0f; vImag[i] = 0.0f; }

    for (uint16_t i = 0; i < numBins; i++) {
        uint16_t idx = rxBins[i].index;
        if (idx == 0 || idx >= HALF) continue;

        float re = rxBins[i].re_q15 / 32767.0f;
        float im = rxBins[i].im_q15 / 32767.0f;

        vReal[idx]     =  re;
        vImag[idx]     =  im;
        // Simetría conjugada: X[N-k] = X*[k]
        vReal[N - idx] =  re;
        vImag[N - idx] = -im;
    }
}

// Hace IFFT, normaliza a 0-255 y saca por DAC a 500 µs/muestra
void outputDAC() {
    FFT.compute(FFTDirection::Reverse);

    float minVal = vReal[0], maxVal = vReal[0];
    for (int i = 1; i < N; i++) {
        if (vReal[i] < minVal) minVal = vReal[i];
        if (vReal[i] > maxVal) maxVal = vReal[i];
    }

    float range = maxVal - minVal;
    if (range < 1e-9f) range = 1.0f;  // señal plana → evitar división por cero

    for (int i = 0; i < N; i++) {
        uint8_t sample = (uint8_t)(((vReal[i] - minVal) / range) * 255.0f);
        dacWrite(DAC_PIN, sample);
        delayMicroseconds(500);
    }
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(UART2_BAUD, SERIAL_8N1, 16, 17);
    delay(1000);
    Serial.println("RX | N=256 Fs=2kHz UART2@921600");
}

void loop() {
    if (!waitSync()) {
        Serial.println("timeout esperando sync");
        return;
    }

    PacketHeader hdr;
    uint16_t numBins;
    if (!receivePacket(hdr, numBins)) {
        Serial.println("paquete invalido");
        return;
    }

    buildSpectrum(numBins);
    outputDAC();

    // Frecuencia del bin de mayor energía recibido
    uint16_t peakBin = rxBins[0].index;
    float peakFreq   = peakBin * SAMPLE_FREQ / N;
    Serial.printf("seq=%u bins=%u freq_est=%.1fHz\n", hdr.seq, numBins, peakFreq);
}
