#include <Arduino.h>
#include <arduinoFFT.h>
#include <BluetoothA2DPSource.h>

// ---- CONFIGURACIÓN -------------------------------------------------------
// *** CAMBIAR por el nombre exacto que aparece al escanear el módulo PAM8403
// *** desde el celular (Ajustes → Bluetooth → escanear)
const char *BT_DEVICE_NAME = "HCW Music";

const uint16_t N = 256;
const uint16_t HALF = N / 2;
const float SAMPLE_FREQ = 2000.0f;
const uint32_t UART2_BAUD = 921600;

// Upsampling de 2000 Hz a 44100 Hz: factor 22 → 256×22 = 5632 frames/bloque
// (error de tasa ~0.23 %, imperceptible para una demo de 250 Hz)
const int BUF_FRAMES = N * 22; // 5632

// ---- FFT -----------------------------------------------------------------
float vReal[N];
float vImag[N];
ArduinoFFT<float> FFT(vReal, vImag, N, SAMPLE_FREQ);

// ---- Protocolo binario (idéntico al transmisor) --------------------------
#pragma pack(push, 1)
struct PacketHeader
{
    uint16_t sync, n, k, seq;
    float etot;
};
struct BinPayload
{
    uint16_t index;
    int16_t re_q15, im_q15;
};
struct PacketTail
{
    uint16_t checksum;
};
#pragma pack(pop)

static BinPayload rxBins[HALF];

// ---- Double-buffer para A2DP ---------------------------------------------
// playBuf: A2DP lee de aquí  |  fillBuf: main loop escribe aquí
// El swap ocurre dentro del callback cuando el buffer se agota y hay uno nuevo.
static int16_t audioBuf[2][BUF_FRAMES];
static volatile uint8_t playBuf = 0;
static volatile uint8_t fillBuf = 1;
static volatile bool newFrame = false;
static volatile int32_t playPos = 0;

// ---- A2DP source ---------------------------------------------------------
BluetoothA2DPSource a2dp;

// Llamado desde la tarea Bluetooth — debe ser rápido y no bloqueante
int32_t provideFrames(Frame *frame, int32_t count)
{
    for (int32_t i = 0; i < count; i++)
    {
        // Swap de buffers cuando se agotó el actual y hay datos nuevos
        if (playPos >= BUF_FRAMES)
        {
            if (newFrame)
            {
                uint8_t tmp = playBuf;
                playBuf = fillBuf;
                fillBuf = tmp;
                newFrame = false;
            }
            playPos = 0;
        }
        int16_t s = audioBuf[playBuf][playPos++];
        frame[i].channel1 = s;
        frame[i].channel2 = s; // mono → estéreo (ambos canales iguales)
    }
    return count;
}

// ---- Helpers UART --------------------------------------------------------
bool readExact(uint8_t *buf, size_t len, uint32_t timeoutMs = 300)
{
    uint32_t t = millis();
    size_t n = 0;
    while (n < len)
    {
        if (millis() - t > timeoutMs)
            return false;
        if (Serial2.available())
            buf[n++] = (uint8_t)Serial2.read();
    }
    return true;
}

// Espera los bytes de sync 0x5A 0xA5 (= uint16_t 0xA55A en little-endian)
bool waitSync(uint32_t timeoutMs = 1000)
{
    uint8_t prev = 0, cur = 0;
    uint32_t t = millis();
    while (millis() - t < timeoutMs)
    {
        if (!Serial2.available())
            continue;
        prev = cur;
        cur = (uint8_t)Serial2.read();
        if (prev == 0x5A && cur == 0xA5)
            return true;
    }
    return false;
}

uint16_t xorChecksum(const uint8_t *d, size_t len, uint16_t init = 0)
{
    for (size_t i = 0; i < len; i++)
        init ^= d[i];
    return init;
}

bool receivePacket(PacketHeader &hdr, uint16_t &numBins)
{
    hdr.sync = 0xA55A;
    if (!readExact((uint8_t *)&hdr + 2, sizeof(hdr) - 2))
        return false;
    if (hdr.n != N)
        return false;
    numBins = hdr.k;
    if (numBins == 0 || numBins >= HALF)
        return false;
    if (!readExact((uint8_t *)rxBins, numBins * sizeof(BinPayload)))
        return false;
    PacketTail tail;
    if (!readExact((uint8_t *)&tail, sizeof(tail)))
        return false;
    uint16_t chk = xorChecksum((uint8_t *)&hdr, sizeof(hdr));
    chk = xorChecksum((uint8_t *)rxBins, numBins * sizeof(BinPayload), chk);
    return chk == tail.checksum;
}

// ---- Reconstrucción espectral + IFFT ------------------------------------
void buildSpectrum(uint16_t numBins)
{
    for (int i = 0; i < N; i++)
    {
        vReal[i] = 0.0f;
        vImag[i] = 0.0f;
    }
    for (uint16_t i = 0; i < numBins; i++)
    {
        uint16_t idx = rxBins[i].index;
        if (idx == 0 || idx >= HALF)
            continue;
        float re = rxBins[i].re_q15 / 32767.0f;
        float im = rxBins[i].im_q15 / 32767.0f;
        vReal[idx] = re;
        vImag[idx] = im;
        vReal[N - idx] = re; // simetría conjugada X[N-k] = X*[k]
        vImag[N - idx] = -im;
    }
}

// IFFT → normaliza a ±32767 → upsamplea a 44100 Hz con interpolación lineal
void processAndBuffer()
{
    FFT.compute(FFTDirection::Reverse);

    // Normalizar vReal[] a rango ±32767
    float mn = vReal[0], mx = vReal[0];
    for (int i = 1; i < N; i++)
    {
        if (vReal[i] < mn)
            mn = vReal[i];
        if (vReal[i] > mx)
            mx = vReal[i];
    }
    float range = mx - mn;
    float center = (mx + mn) * 0.5f;
    float scale = (range > 1e-9f) ? (32767.0f / (range * 0.5f)) : 1.0f;

    // Upsamplear a BUF_FRAMES con interpolación lineal entre muestras
    int16_t *dst = audioBuf[fillBuf];
    for (int j = 0; j < BUF_FRAMES; j++)
    {
        float pos = (float)j * N / BUF_FRAMES;
        int i0 = (int)pos;
        int i1 = (i0 + 1) % N;
        float frac = pos - (float)i0;
        float s = ((vReal[i0] * (1.0f - frac) + vReal[i1] * frac) - center) * scale;
        if (s > 32767.0f)
            s = 32767.0f;
        if (s < -32767.0f)
            s = -32767.0f;
        dst[j] = (int16_t)s;
    }

    newFrame = true; // avisa al callback que puede hacer swap
}

// ---- Setup & Loop --------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    Serial2.begin(UART2_BAUD, SERIAL_8N1, 16, 17);

    memset(audioBuf, 0, sizeof(audioBuf)); // silencio inicial

    a2dp.start(BT_DEVICE_NAME, provideFrames);

    Serial.printf("RX | conectando a BT: \"%s\"\n", BT_DEVICE_NAME);
}

void loop()
{
    if (!waitSync())
    {
        Serial.println("timeout esperando sync UART");
        return;
    }

    PacketHeader hdr;
    uint16_t numBins;
    if (!receivePacket(hdr, numBins))
    {
        Serial.println("paquete invalido");
        return;
    }

    buildSpectrum(numBins);
    processAndBuffer();

    float peakFreq = rxBins[0].index * SAMPLE_FREQ / N;
    Serial.printf("seq=%u bins=%u freq=%.1fHz BT=%s\n",
                  hdr.seq, numBins, peakFreq,
                  a2dp.is_connected() ? "conectado" : "buscando...");
}
