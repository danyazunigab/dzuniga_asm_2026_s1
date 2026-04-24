/*
 * ESP32-S3 — UART FFT Receiver + IFFT + Metricas + OLED Menu
 *
 * OLED SDA  → GPIO 15
 * OLED SCL  → GPIO 41
 * ENC CLK   → GPIO 0
 * ENC DT    → GPIO 21
 * ENC BTN   → GPIO 1
 * UART RX   → GPIO 2
 * UART TX   → GPIO 42
 *
 * Core 0 → taskDSP : UART RX → IFFT → metricas
 * Core 1 → taskUI  : encoder + boton + OLED
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <arduinoFFT.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"

// ════════════════════════════════════════════════════════
//  Configuración — ajustar aquí para cambiar parámetros
// ════════════════════════════════════════════════════════
#define SCREEN_W 128
#define SCREEN_H 64
#define OLED_ADDR 0x3C
#define OLED_SDA 15
#define OLED_SCL 41

#define ENC_CLK 0
#define ENC_DT 21
#define ENC_BTN 1

#define UART_RX 2
#define UART_TX 42
#define UART_BAUD 921600

#define FFT_SIZE 256     // debe coincidir con transmisor N=256
#define SAMPLE_RATE 2000 // debe coincidir con transmisor SAMPLE_FREQ=2000
#define WAVE_POINTS 64

#define DEBOUNCE_MS 50
#define LONG_PRESS_MS 600
#define WDT_TIMEOUT_S 30

// ════════════════════════════════════════════════════════
//  Estructuras del protocolo
// ════════════════════════════════════════════════════════
#pragma pack(push, 1)
struct PacketHeader
{
  uint16_t sync; // 0xA55A → wire: 5A A5
  uint16_t n;    // FFT_SIZE
  uint16_t k;    // número de bins
  uint16_t seq;  // secuencia
  float etot;    // energía total (ADC crudo, solo referencia)
};

struct BinPayload
{
  uint16_t index; // bin index [1, N/2)
  int16_t re_q15; // parte real  normalizada Q15
  int16_t im_q15; // parte imag  normalizada Q15
};

struct PacketTail
{
  uint16_t checksum; // XOR de todos los bytes anteriores
};
#pragma pack(pop)

// ════════════════════════════════════════════════════════
//  Buffers globales — nunca en stack de tarea
// ════════════════════════════════════════════════════════
static float fftRe[FFT_SIZE];
static float fftIm[FFT_SIZE];
static float reForMetrics[FFT_SIZE];
static float imForMetrics[FFT_SIZE];
static float timeDomain[FFT_SIZE];
static float imIFFT[FFT_SIZE];
static float waveSnapshot[WAVE_POINTS];
static float magBuf[FFT_SIZE / 2];
static BinPayload rxBins[FFT_SIZE / 2];

// ════════════════════════════════════════════════════════
//  Debug counters
// ════════════════════════════════════════════════════════
static uint32_t dbg_rxBytes = 0;
static uint32_t dbg_syncFound = 0;
static uint32_t dbg_syncTimeout = 0;
static uint32_t dbg_pktOk = 0;
static uint32_t dbg_pktFail = 0;
static uint32_t dbg_badChecksum = 0;

// ════════════════════════════════════════════════════════
//  arduinoFFT — instancia sobre los buffers globales
// ════════════════════════════════════════════════════════
ArduinoFFT<float> FFT(fftRe, fftIm, FFT_SIZE, (float)SAMPLE_RATE);

// ════════════════════════════════════════════════════════
//  Métricas compartidas
// ════════════════════════════════════════════════════════
struct Metrics
{
  float mse;
  float thd;
  float snr;
  float energy;
  float f0Hz;
  uint16_t lastSeq;
  uint32_t framesOk;
  uint32_t framesLost;
};

static SemaphoreHandle_t metricsMutex;
static SemaphoreHandle_t waveMutex;
static Metrics sharedMetrics = {};
static Metrics displayMetrics = {};

// ════════════════════════════════════════════════════════
//  Display
// ════════════════════════════════════════════════════════
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

const char *PAGE_TITLES[] = {"INICIO", "SENAL", "MSE", "THD", "SNR", "ENERGY"};
const uint8_t NUM_PAGES = sizeof(PAGE_TITLES) / sizeof(PAGE_TITLES[0]);

struct AppState
{
  int8_t currentPage = 0;
  bool needsRedraw = true;
  bool inMenu = false;
  int8_t menuItem = 0;
  uint8_t menuItemCount = 3;
} state;

// ════════════════════════════════════════════════════════
//  Encoder
// ════════════════════════════════════════════════════════
volatile int8_t encoderDelta = 0;
static int lastCLK = 0;

// ════════════════════════════════════════════════════════
//  Botón
// ════════════════════════════════════════════════════════
static uint32_t btnPressTime = 0;
static bool btnWasPressed = false;
static bool longFireDone = false;

// ════════════════════════════════════════════════════════
//  UART — helpers
// ════════════════════════════════════════════════════════
static uint16_t xorChecksum(const uint8_t *data, size_t len, uint16_t init = 0)
{
  for (size_t i = 0; i < len; i++)
    init ^= data[i];
  return init;
}

// Lee exactamente `len` bytes con timeout. Cede CPU cuando no hay datos.
static bool readExact(uint8_t *buf, size_t len, uint32_t timeoutMs)
{
  uint32_t start = millis();
  size_t received = 0;

  while (received < len)
  {
    if (millis() - start > timeoutMs)
      return false;
    if (Serial2.available())
    {
      buf[received++] = Serial2.read();
    }
    else
    {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    esp_task_wdt_reset();
  }
  return true;
}

// Busca sync 5A A5 con timeout global. Valida header completo
// para evitar falsos positivos dentro del payload.
static bool waitSyncAndValidate(PacketHeader &hdr, uint16_t &numBins)
{
  // sizeof(header) - 2 bytes de sync = 10 bytes
  constexpr size_t HDR_TAIL = sizeof(PacketHeader) - 2;
  uint8_t buf[HDR_TAIL];

  uint32_t globalStart = millis();

  for (;;)
  {
    // Timeout global: si el transmisor se detiene no bloqueamos para siempre
    if (millis() - globalStart > 5000)
    {
      dbg_syncTimeout++;
      return false;
    }

    // Buscar patrón 5A A5
    uint8_t prev = 0, cur = 0;
    uint32_t searchStart = millis();

    while (millis() - searchStart < 2000)
    {
      if (!Serial2.available())
      {
        vTaskDelay(pdMS_TO_TICKS(1));
        esp_task_wdt_reset();
        continue;
      }
      prev = cur;
      cur = Serial2.read();
      dbg_rxBytes++;
      if (prev == 0x5A && cur == 0xA5)
        break;
    }

    if (!(prev == 0x5A && cur == 0xA5))
      continue; // no encontró sync en 2s

    // Leer resto del header — a 921600 baud, 10 bytes = <1ms → timeout 20ms
    if (!readExact(buf, HDR_TAIL, 20))
      continue;

    // Reconstruir y validar
    hdr.sync = 0xA55A;
    memcpy((uint8_t *)&hdr + 2, buf, HDR_TAIL);

    // Filtros estrictos para descartar falsos positivos
    if (hdr.n != FFT_SIZE)
      continue;
    if (hdr.k == 0)
      continue;
    if (hdr.k >= FFT_SIZE / 2)
      continue;
    // etot puede ser grande (ADC crudo), solo descartamos NaN/infinito
    if (!isfinite(hdr.etot))
      continue;
    if (hdr.etot < 0.0f)
      continue;

    dbg_syncFound++;
    numBins = hdr.k;
    return true;
  }
}

// Lee bins + tail, verifica checksum
static bool receivePacketBody(const PacketHeader &hdr, uint16_t numBins)
{
  // Tiempo máximo: numBins × 6 bytes a 921600 baud ≈ numBins × 0.065ms
  // Con margen generoso: 100ms cubre hasta 127 bins con holgura
  size_t binsBytes = numBins * sizeof(BinPayload);

  if (!readExact((uint8_t *)rxBins, binsBytes, 100))
  {
    dbg_pktFail++;
    return false;
  }

  PacketTail tail;
  if (!readExact((uint8_t *)&tail, sizeof(tail), 20))
  {
    dbg_pktFail++;
    return false;
  }

  uint16_t chk = xorChecksum((uint8_t *)&hdr, sizeof(hdr));
  chk = xorChecksum((uint8_t *)rxBins, binsBytes, chk);

  if (chk != tail.checksum)
  {
    dbg_badChecksum++;
    return false;
  }

  dbg_pktOk++;
  return true;
}

// ════════════════════════════════════════════════════════
//  DSP — reconstrucción espectral
// ════════════════════════════════════════════════════════

// Reconstruye espectro completo con simetría hermitiana
// Los valores Q15 están normalizados por maxAbs del transmisor
// → amplitudes relativas correctas, escala absoluta desconocida
static void buildSpectrum(uint16_t numBins)
{
  memset(fftRe, 0, sizeof(fftRe));
  memset(fftIm, 0, sizeof(fftIm));

  for (uint16_t i = 0; i < numBins; i++)
  {
    uint16_t idx = rxBins[i].index;
    if (idx == 0 || idx >= FFT_SIZE / 2)
      continue;

    float re = rxBins[i].re_q15 / 32767.0f;
    float im = rxBins[i].im_q15 / 32767.0f;

    fftRe[idx] = re;
    fftIm[idx] = im;
    fftRe[FFT_SIZE - idx] = re; // conjugado simétrico
    fftIm[FFT_SIZE - idx] = -im;
  }
}

// Encuentra el bin de mayor magnitud en [1, N/2)
// Devuelve frecuencia en Hz por interpolación parabólica
static float findPeakHz(const float *re, const float *im, int *peakBinOut)
{
  float maxMag = 0.0f;
  int peak = 1;

  for (int i = 1; i < FFT_SIZE / 2; i++)
  {
    float mag = re[i] * re[i] + im[i] * im[i]; // magnitud² es suficiente para comparar
    magBuf[i] = mag;
    if (mag > maxMag)
    {
      maxMag = mag;
      peak = i;
    }
  }

  if (peakBinOut)
    *peakBinOut = peak;

  // Interpolación parabólica sobre magnitudes² para mayor precisión
  float f0 = (float)peak;
  if (peak > 1 && peak < FFT_SIZE / 2 - 1)
  {
    float ym1 = magBuf[peak - 1];
    float y0 = magBuf[peak];
    float yp1 = magBuf[peak + 1];
    float denom = 2.0f * (2.0f * y0 - ym1 - yp1);
    if (fabsf(denom) > 1e-10f)
      f0 += (ym1 - yp1) / denom;
  }

  return f0 * (float)SAMPLE_RATE / (float)FFT_SIZE;
}

// ════════════════════════════════════════════════════════
//  Métricas DSP
// ════════════════════════════════════════════════════════
static float computeTHD(const float *re, const float *im, int N, int f0Bin)
{
  if (f0Bin <= 0 || f0Bin * 2 >= N / 2)
    return 0.0f;

  // Magnitud lineal del fundamental
  float fundamental = sqrtf(re[f0Bin] * re[f0Bin] + im[f0Bin] * im[f0Bin]);
  if (fundamental < 1e-6f)
    return 0.0f;

  // Suma de POTENCIAS de armónicos, luego raíz → magnitud RMS
  float harmonicsPow = 0.0f;
  for (int h = 2; h * f0Bin < N / 2; h++)
  {
    int hBin = h * f0Bin;
    float hm = sqrtf(re[hBin] * re[hBin] + im[hBin] * im[hBin]);
    harmonicsPow += hm * hm; // FIX: sumar potencias, no magnitudes
  }
  return sqrtf(harmonicsPow) / fundamental * 100.0f;
}

static float computeSNR(const float *re, const float *im, int N, int f0Bin)
{
  if (f0Bin <= 0 || f0Bin >= N / 2)
    return 0.0f;

  float signalPow = 0.0f;
  float noisePow = 0.0f;

  // Definimos un ancho para la señal (la fundamental + vecinos inmediatos)
  // Esto equivale a integrar la potencia de la señal en Python
  for (int i = 1; i < N / 2; i++)
  {
    float binMagSq = re[i] * re[i] + im[i] * im[i];

    // Si el bin está cerca del pico, es SEÑAL
    if (i >= f0Bin - 1 && i <= f0Bin + 1)
    {
      signalPow += binMagSq;
    }
    // Si no, es RUIDO
    else
    {
      noisePow += binMagSq;
    }
  }

  // Lógica de Python: si Pn es 0, el SNR es infinito (o un tope práctico)
  if (noisePow < 1e-18f)
    return 99.9f;
  if (signalPow < 1e-18f)
    return 0.0f;

  // 10 * log10(Ps / Pn)
  return 10.0f * log10f(signalPow / noisePow);
}

// Calcula la suma de magnitudes al cuadrado de los bins complejos recibidos
static float computeSpectralEnergy(const BinPayload *bins, uint16_t numBins)
{
  float energy = 0.0f;
  for (uint16_t i = 0; i < numBins; i++)
  {
    // Convertimos de Q15 a float (rango -1.0 a 1.0)
    float re = bins[i].re_q15 / 32767.0f;
    float im = bins[i].im_q15 / 32767.0f;

    // La energía de un bin es Re² + Im²
    energy += (re * re + im * im);
  }
  return energy;
}

// MSE
static float computeMSE(const float *re, const float *im, const float *temporal, int N)
{
  // 1. Energía Espectral (Parseval)
  // Sumamos la magnitud al cuadrado de los bins.
  // No olvides el bin 0 (DC) y el bin N/2 (Nyquist) si están presentes.
  float eSpec = 0.0f;
  for (int i = 1; i < N / 2; i++)
  {
    eSpec += (re[i] * re[i] + im[i] * im[i]);
  }
  // Factor 2 por simetría (bins negativos) y /N por la definición de la IFFT de arduinoFFT
  eSpec = (2.0f * eSpec) / (float)N;

  // 2. Energía Temporal
  float eTime = 0.0f;
  for (int i = 0; i < N; i++)
  {
    eTime += (temporal[i] * temporal[i]);
  }
  eTime /= (float)N; // Promedio de energía por muestra

  // El MSE de energía es la diferencia promedio
  return fabsf(eSpec - eTime);
}

// ════════════════════════════════════════════════════════
//  Tarea DSP — Core 0
// ════════════════════════════════════════════════════════
void taskDSP(void *pvParams)
{
  esp_task_wdt_add(NULL);
  uint32_t lastReport = millis();

  for (;;)
  {
    PacketHeader hdr;
    uint16_t numBins = 0;

    // ── 1. Sincronizar y validar header ──────────────
    if (!waitSyncAndValidate(hdr, numBins))
    {
      esp_task_wdt_reset();
      continue;
    }

    // ── 2. Leer body y verificar checksum ────────────
    if (!receivePacketBody(hdr, numBins))
    {
      esp_task_wdt_reset();
      continue;
    }

    // ── 3. Reconstruir espectro ───────────────────────
    buildSpectrum(numBins);

    // ── 4. Guardar copia espectral pre-IFFT ──────────
    memcpy(reForMetrics, fftRe, sizeof(fftRe));
    memcpy(imForMetrics, fftIm, sizeof(fftIm));

    // ── 5. Detectar pico ANTES de la IFFT ────────────
    int f0Bin;
    float f0Hz = findPeakHz(reForMetrics, imForMetrics, &f0Bin);

    // ── 6. IFFT — opera in-place sobre fftRe/fftIm ──
    FFT.compute(FFTDirection::Reverse);
    // const float invN = 1.0f / (float)FFT_SIZE;
    // for (int i = 0; i < FFT_SIZE; i++)
    //   fftRe[i] *= invN;
    memcpy(timeDomain, fftRe, sizeof(timeDomain));
    memcpy(imIFFT, fftIm, sizeof(imIFFT));

    // ── 7. Restaurar espectro para métricas ──────────
    memcpy(fftRe, reForMetrics, sizeof(fftRe));
    memcpy(fftIm, imForMetrics, sizeof(fftIm));

    // ── 8. Calcular métricas ──────────────────────────
    float mse = computeMSE(reForMetrics, imForMetrics, timeDomain, FFT_SIZE);
    float thd = computeTHD(reForMetrics, imForMetrics, FFT_SIZE, f0Bin);
    float snr = computeSNR(reForMetrics, imForMetrics, FFT_SIZE, f0Bin);

    // Calculamos la energía de lo que acabamos de recibir por UART
    float eReceived = computeSpectralEnergy(rxBins, numBins);

    // Comparamos con la energía total que el transmisor midió (hdr.etot)
    // Aplicamos la misma lógica de seguridad que en Python para evitar división por cero
    float ratioEnergia = (hdr.etot > 1e-9f) ? (eReceived / hdr.etot) : 1.0f;

    // ── 9. Publicar métricas ──────────────────────────
    if (xSemaphoreTake(metricsMutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
      sharedMetrics.mse = mse;
      sharedMetrics.thd = thd;
      sharedMetrics.snr = snr;
      sharedMetrics.energy = ratioEnergia;
      sharedMetrics.f0Hz = f0Hz;
      sharedMetrics.lastSeq = hdr.seq;
      sharedMetrics.framesOk++;
      xSemaphoreGive(metricsMutex);
    }

    // ── 10. Publicar waveform ─────────────────────────
    if (xSemaphoreTake(waveMutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
      for (int i = 0; i < WAVE_POINTS; i++)
      {
        // Interpolación lineal entre muestras adyacentes
        float fIdx = (float)i * (float)FFT_SIZE / (float)WAVE_POINTS;
        int iIdx = (int)fIdx;
        float frac = fIdx - iIdx;
        int jIdx = (iIdx + 1 < FFT_SIZE) ? iIdx + 1 : iIdx;
        waveSnapshot[i] = timeDomain[iIdx] * (1.0f - frac) + timeDomain[jIdx] * frac;
      }
      xSemaphoreGive(waveMutex);
    }

    esp_task_wdt_reset();
    taskYIELD();
  }
}

// ════════════════════════════════════════════════════════
//  Dibujo OLED
// ════════════════════════════════════════════════════════
static void drawHeader(const char *title)
{
  int16_t x1, y1;
  uint16_t tw, th;
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.getTextBounds(title, 0, 0, &x1, &y1, &tw, &th);
  display.fillRect(0, 0, SCREEN_W, 12, WHITE);
  display.setTextColor(BLACK);
  display.setCursor((SCREEN_W - (int)tw) / 2, 2);
  display.print(title);
  display.setTextColor(WHITE);
}

static void drawPageDots(uint8_t cur, uint8_t total)
{
  const int ds = 4, sp = 7;
  int totalW = total * ds + (total - 1) * (sp - ds);
  int sx = (SCREEN_W - totalW) / 2;
  int y = SCREEN_H - 6;
  for (uint8_t i = 0; i < total; i++)
  {
    int x = sx + i * sp;
    if (i == cur)
      display.fillRect(x, y, ds, ds, WHITE);
    else
      display.drawRect(x, y, ds, ds, WHITE);
  }
}

// ────────────────────────────────────────────────────────
static void drawPage0()
{
  display.drawRect(1, 1, 126, 63, SSD1306_WHITE);
  display.setTextSize(5);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(22, 15);
  display.print(F("FFT"));
}

// ────────────────────────────────────────────────────────
static void drawPage1()
{
  drawHeader("SIGNAL");

  float wave[WAVE_POINTS];
  if (xSemaphoreTake(waveMutex, pdMS_TO_TICKS(5)) == pdTRUE)
  {
    memcpy(wave, waveSnapshot, sizeof(wave));
    xSemaphoreGive(waveMutex);
  }
  else
  {
    memset(wave, 0, sizeof(wave));
  }

  float wMax = 1e-6f;
  for (int i = 0; i < WAVE_POINTS; i++)
    if (fabsf(wave[i]) > wMax)
      wMax = fabsf(wave[i]);

  const int baseY = 37, height = 17;
  const int xStep = SCREEN_W / WAVE_POINTS;
  for (int i = 0; i < WAVE_POINTS - 1; i++)
  {
    int y0 = constrain(baseY - (int)(wave[i] / wMax * height), 13, 54);
    int y1 = constrain(baseY - (int)(wave[i + 1] / wMax * height), 13, 54);
    display.drawLine(i * xStep, y0, (i + 1) * xStep, y1, WHITE);
  }

  display.setTextSize(1);
  display.setCursor(1, 54);
  display.print("~");
  display.print((int)displayMetrics.f0Hz);
  display.print("Hz");
}

// ────────────────────────────────────────────────────────
static void drawPage2() // MSE
{
  drawHeader("MSE");

  if (xSemaphoreTake(metricsMutex, pdMS_TO_TICKS(5)) == pdTRUE)
  {
    displayMetrics = sharedMetrics;
    xSemaphoreGive(metricsMutex);
  }

  display.setTextSize(2);
  display.setCursor(10, 25);
  display.print(displayMetrics.mse, 5);
}

// ────────────────────────────────────────────────────────
static void drawPage3() // THD
{
  drawHeader("THD");

  if (xSemaphoreTake(metricsMutex, pdMS_TO_TICKS(5)) == pdTRUE)
  {
    displayMetrics = sharedMetrics;
    xSemaphoreGive(metricsMutex);
  }

  display.setTextSize(2);
  display.setCursor(10, 25);
  display.print(displayMetrics.thd, 2);
  display.print("%");
}

// ────────────────────────────────────────────────────────
static void drawPage4() // SNR
{
  drawHeader("SNR");

  if (xSemaphoreTake(metricsMutex, pdMS_TO_TICKS(5)) == pdTRUE)
  {
    displayMetrics = sharedMetrics;
    xSemaphoreGive(metricsMutex);
  }

  display.setTextSize(2);
  display.setCursor(10, 25);
  display.print(displayMetrics.snr, 1);
  display.print(" dB");
}
// ────────────────────────────────────────────────────────
static void drawPage5() // Energy
{
  drawHeader("ENERGY");

  if (xSemaphoreTake(metricsMutex, pdMS_TO_TICKS(5)) == pdTRUE)
  {
    displayMetrics = sharedMetrics;
    xSemaphoreGive(metricsMutex);
  }

  display.setTextSize(2);
  display.setCursor(10, 25);
  display.print(displayMetrics.energy * 100, 1);
  display.print(" %");
}

// ────────────────────────────────────────────────────────
static void drawCurrentPage()
{
  switch (state.currentPage)
  {
  case 0:
    drawPage0();
    break;
  case 1:
    drawPage2();
    break;
  case 2:
    drawPage3();
    break;
  case 3:
    drawPage4();
    break;
  case 4:
    drawPage5();
    break;
  case 5:
    drawPage1();
    break;
  }
  drawPageDots(state.currentPage, NUM_PAGES);
}

// ════════════════════════════════════════════════════════
//  Encoder handler
// ════════════════════════════════════════════════════════
void handleEncoder()
{
  static uint32_t lastStepTime = 0;
  const uint32_t DEBOUNCE_US = 2000; // 2 ms (ajustable)

  int clk = digitalRead(ENC_CLK);
  uint32_t now = micros();

  // Detectar flanco de subida con filtro
  if (clk != lastCLK && clk == HIGH)
  {
    if (now - lastStepTime > DEBOUNCE_US)
    {
      if (digitalRead(ENC_DT) == clk)
        encoderDelta++;
      else
        encoderDelta--;

      lastStepTime = now;
    }
  }

  lastCLK = clk;

  // Aplicar movimiento al sistema
  int8_t steps = encoderDelta;
  encoderDelta = 0;

  if (steps == 0)
    return;

  if (steps > 3)
    steps = 3;
  if (steps < -3)
    steps = -3;

  if (state.inMenu)
  {
    state.menuItem = constrain(
        state.menuItem + steps,
        0,
        (int)state.menuItemCount - 1);
  }
  else
  {
    state.currentPage =
        (state.currentPage + steps + NUM_PAGES) % NUM_PAGES;
  }

  state.needsRedraw = true;
}

// ════════════════════════════════════════════════════════
//  Boton handler
// ════════════════════════════════════════════════════════
static void handleButton()
{
  bool pressed = (digitalRead(ENC_BTN) == LOW);
  uint32_t now = millis();

  if (pressed && !btnWasPressed)
  {
    btnPressTime = now;
    btnWasPressed = true;
    longFireDone = false;
  }

  if (pressed && !longFireDone && (now - btnPressTime >= LONG_PRESS_MS))
  {
    longFireDone = true;
    state.inMenu = false;
    state.currentPage = 0;
    state.needsRedraw = true;
  }

  if (!pressed && btnWasPressed)
  {
    if (!longFireDone && (now - btnPressTime >= DEBOUNCE_MS))
    {
      if (state.currentPage == 3)
      {
        state.inMenu = !state.inMenu;
        state.menuItem = 0;
        state.needsRedraw = true;
      }
    }
    btnWasPressed = false;
  }
}

// ════════════════════════════════════════════════════════
//  TAREA CORE 1 — UI
// ════════════════════════════════════════════════════════
void taskUI(void *pvParams)
{
  esp_task_wdt_add(NULL);

  for (;;)
  {
    esp_task_wdt_reset();
    handleEncoder();
    handleButton();

    if (state.needsRedraw)
    {
      display.clearDisplay();
      drawCurrentPage();
      display.display();
      state.needsRedraw = false;
    }

    vTaskDelay(pdMS_TO_TICKS(16));
  }
}

// ════════════════════════════════════════════════════════
//  Setup
// ════════════════════════════════════════════════════════
void setup()
{
  Serial.begin(115200);
  delay(500);

  Serial.print("Reset reason: ");
  Serial.println(esp_reset_reason());

  esp_task_wdt_init(30, false);

  // UART
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);

  // Mutexes
  metricsMutex = xSemaphoreCreateMutex();
  waveMutex = xSemaphoreCreateMutex();

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR))
  {
    Serial.println("ERROR: SSD1306 no encontrado");
    while (true)
      delay(1000);
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(20, 28);
  display.print("Iniciando...");
  display.display();

  // Encoder: ambos pines en ISR (4 transiciones por detente)
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_BTN, INPUT_PULLUP);

  lastCLK = digitalRead(ENC_CLK);

  // Tareas FreeRTOS
  xTaskCreatePinnedToCore(taskDSP, "DSP", 16384, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskUI, "UI", 4096, NULL, 1, NULL, 1);

  Serial.println("Setup OK");
}

void loop()
{
  vTaskDelete(NULL);
}