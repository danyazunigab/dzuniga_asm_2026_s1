"""
=============================================================
  Compresión Espectral Adaptativa con FFT
=============================================================
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.ticker import FuncFormatter
import struct
import os
import time
from scipy.io import wavfile


# ─────────────────────────────────────────────
#  1.  Implementación FFT (Tomada del experimento anterior)
# ─────────────────────────────────────────────

def calculate_by_fft(x):
    """
    Implementación de FFT utilizada en el experimento de DFTvFFT
    """
    x_as_array = np.asarray(x, dtype=complex)
    N = len(x_as_array)

    if N < 0 and (N & (N - 1) != 0):
        raise ValueError(
            "Input signal length must be a power of 2. "
            "Current input length: {}".format(N)
        )

    if N == 1:
        return x_as_array.copy()

    x_as_array_even = x_as_array[0::2]
    x_as_array_odd  = x_as_array[1::2]

    E = calculate_by_fft(x_as_array_even)
    O = calculate_by_fft(x_as_array_odd)

    X     = np.zeros(N, dtype=complex)
    k     = np.arange(N // 2)
    W_N_k = np.exp(-2j * np.pi / N) ** k

    X[:N // 2] = E + W_N_k * O
    X[N // 2:] = E - W_N_k * O

    return X


def ifft(X):
    """
    IFFT usando la propiedad de conjugación:
        IFFT(X) = conj(FFT(conj(X))) / N

    Reutiliza calculate_by_fft.
    """
    N = len(X)
    return np.conj(calculate_by_fft(np.conj(X))) / N


def siguiente_potencia_de_2(n):
    p = 1
    while p < n:
        p <<= 1
    return p


def fft_con_padding(x):
    x      = np.asarray(x, dtype=float)
    N_orig = len(x)
    N_pad  = siguiente_potencia_de_2(N_orig)
    x_pad  = np.zeros(N_pad)
    x_pad[:N_orig] = x
    X = calculate_by_fft(x_pad)
    return X, N_orig, N_pad


# ─────────────────────────────────────────────
#  2.  Compresión Espectral Adaptativa
# ─────────────────────────────────────────────

def comprimir_espectro(X, fraccion_energia=0.95):
    """
    1. E[k] = |X[k]|²  para cada coeficiente
    2. Ordena k de mayor a menor E[k]
    3. Acumula hasta alcanzar fraccion_energia · E_total
    4. Pone a cero los coeficientes no seleccionados
    """
    energias = np.abs(X) ** 2
    E_total  = energias.sum()

    if E_total == 0:
        return X.copy(), np.arange(len(X)), 0.0

    orden    = np.argsort(energias)[::-1]
    acum     = np.cumsum(energias[orden])
    n_usados = int(np.searchsorted(acum, fraccion_energia * E_total)) + 1

    indices_usados = np.sort(orden[:n_usados])
    mascara        = np.zeros(len(X), dtype=bool)
    mascara[indices_usados] = True

    X_comp = X.copy()
    X_comp[~mascara] = 0.0

    ratio = 1.0 - n_usados / len(X)
    return X_comp, indices_usados, ratio


def energia_conservada(X_orig, X_comp):
    E_orig = (np.abs(X_orig) ** 2).sum()
    E_comp = (np.abs(X_comp) ** 2).sum()
    return float(E_comp / E_orig) if E_orig > 0 else 1.0


def mse(x_orig, x_rec):
    n = min(len(x_orig), len(x_rec))
    return float(np.mean((np.asarray(x_orig[:n]) - np.asarray(x_rec[:n])) ** 2))


def snr_db(x_orig, x_rec):
    n   = min(len(x_orig), len(x_rec))
    x_o = np.asarray(x_orig[:n], dtype=float)
    x_r = np.asarray(x_rec[:n],  dtype=float)
    Ps  = np.mean(x_o ** 2)
    Pn  = np.mean((x_o - x_r) ** 2)
    return float(10 * np.log10(Ps / Pn)) if Pn > 0 else float('inf')


# ─────────────────────────────────────────────
#  3.  Generación de Señales
# ─────────────────────────────────────────────

def señal_senoidal(freq=440.0, fs=4000, duracion=0.5, amplitud=1.0):
    n = np.arange(int(fs * duracion))
    return amplitud * np.sin(2 * np.pi * freq * n / fs)

def señal_suma_senoidales(freqs, amplitudes, fs=4000, duracion=0.5):
    n = np.arange(int(fs * duracion))
    return sum(A * np.sin(2 * np.pi * f * n / fs) for f, A in zip(freqs, amplitudes))

def señal_cuadrada(freq=440.0, fs=4000, duracion=0.5, amplitud=1.0):
    """Onda cuadrada como suma de armónicos impares: (4A/πk)·sin(2πkft)"""
    n = np.arange(int(fs * duracion))
    s = np.zeros(len(n))
    for k in range(1, 50, 2):
        if k * freq >= fs / 2:
            break
        s += (4 * amplitud / (np.pi * k)) * np.sin(2 * np.pi * k * freq * n / fs)
    return s

def señal_ruido_blanco(N=4096, amplitud=1.0, semilla=42):
    return amplitud * np.random.default_rng(semilla).uniform(-1, 1, N)


# ─────────────────────────────────────────────
#  4.  Lectura y escritura de archivos WAV
#  Usando scipy - https://www.youtube.com/watch?v=jabJcnmxpFg
# ─────────────────────────────────────────────

def leer_wav(ruta):
    sample_rate, datos = wavfile.read(ruta)
    n_canales = 1 if datos.ndim == 1 else datos.shape[1]

    # Tomar solo canal izquierdo si es estéreo
    muestras = datos[:, 0] if datos.ndim == 2 else datos

    # Normalizar a float [-1.0, 1.0] según dtype
    if muestras.dtype == np.int16:
        muestras = muestras.astype(float) / 32768.0
    elif muestras.dtype == np.uint8:
        muestras = (muestras.astype(float) - 128) / 128.0
    elif muestras.dtype == np.int32:
        muestras = muestras.astype(float) / 2147483648.0
    elif muestras.dtype == np.float32 or muestras.dtype == np.float64:
        muestras = muestras.astype(float)
    else:
        raise ValueError(f"Tipo de muestra no soportado: {muestras.dtype}")

    return muestras, sample_rate, n_canales


def escribir_wav(ruta, muestras, sample_rate=8000):
    muestras = np.asarray(muestras, dtype=float)

    mx = np.abs(muestras).max()
    if mx > 1.0:
        muestras = muestras / mx

    datos = np.clip(muestras * 32767, -32768, 32767).astype(np.int16)
    wavfile.write(ruta, sample_rate, datos)

# ─────────────────────────────────────────────
#  5.  Visualización con Matplotlib
# ─────────────────────────────────────────────

# Paleta consistente para todas las gráficas
C_ORIG  = '#2563EB'   # azul  → señal / espectro original
C_REC   = '#16A34A'   # verde → señal reconstruida
C_COMP  = '#DC2626'   # rojo  → espectro comprimido / coeficientes eliminados
C_KEPT  = '#2563EB'   # azul  → coeficientes conservados
C_ERR   = '#9333EA'   # púrpura → error de reconstrucción
C_ACUM  = '#EA580C'   # naranja → energía acumulada


def graficar_resultado(r, guardar_png=False):
    """
    Genera una figura con 5 subplots para un resultado de compresión:

      1. Señal en el tiempo: original vs reconstruida
      2. Espectro de magnitud: original vs comprimido (frecuencias positivas)
      3. Espectro en dB: muestra la dinámica real de la compresión
      4. Mapa de coeficientes: cuáles se conservaron y cuáles se zeroed
      5. Energía acumulada espectral: ilustra el criterio de corte
    """
    nombre   = r['nombre']
    fs       = r['fs']
    N_orig   = r['N_orig']
    N_pad    = r['N_pad']
    X        = r['X_original']
    X_comp   = r['X_comprimido']
    x_orig   = r['señal_original']
    x_rec    = r['señal_reconstruida']
    indices  = r['indices_usados']

    # ── Datos derivados ─────────────────────────────────────
    mitad   = N_pad // 2
    freqs   = np.arange(mitad) * fs / N_pad          # eje de frecuencias [Hz]
    t       = np.arange(N_orig) / fs                 # eje de tiempo [s]

    # Magnitudes normalizadas (amplitud real de cada componente)
    mags_orig = np.abs(X[:mitad]) * 2 / N_orig
    mags_comp = np.abs(X_comp[:mitad]) * 2 / N_orig

    # Espectro en dB (evita log(0))
    eps    = 1e-10
    db_orig = 20 * np.log10(mags_orig + eps)
    db_comp = 20 * np.log10(mags_comp + eps)

    # Energía acumulada (todos los coefs, no solo positivos)
    energias_ord = np.sort(np.abs(X) ** 2)[::-1]
    E_total      = energias_ord.sum()
    acum_pct     = np.cumsum(energias_ord) / E_total * 100
    pct_coefs    = np.linspace(0, 100, len(acum_pct))

    # Máscara de coeficientes conservados (solo frecuencias positivas)
    mascara_pos = np.zeros(mitad, dtype=bool)
    for k in indices:
        if k < mitad:
            mascara_pos[k] = True

    error = x_orig - x_rec

    # ── Layout ──────────────────────────────────────────────
    fig = plt.figure(figsize=(14, 11))
    fig.suptitle(
        f"Compresión Espectral Adaptativa  —  {nombre}\n"
        f"Coefs conservados: {r['coefs_usados']}/{N_pad}  "
        f"({(1-r['ratio'])*100:.1f}%)   |   "
        f"Energía conservada: {r['energia_conservada']*100:.2f}%   |   "
        f"SNR: {r['snr_db']:.1f} dB",
        fontsize=12, fontweight='bold', y=0.98
    )

    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.45, wspace=0.32,
                           top=0.91, bottom=0.06)

    ax1 = fig.add_subplot(gs[0, :])    # señal en el tiempo (ancho completo)
    ax2 = fig.add_subplot(gs[1, 0])    # espectro de magnitud
    ax3 = fig.add_subplot(gs[1, 1])    # espectro en dB
    ax4 = fig.add_subplot(gs[2, 0])    # mapa de coeficientes
    ax5 = fig.add_subplot(gs[2, 1])    # energía acumulada

    # ── [1] Señal en el tiempo ───────────────────────────────
    ax1.plot(t, x_orig, color=C_ORIG, lw=1.2, label='Original', zorder=3)
    ax1.plot(t, x_rec,  color=C_REC,  lw=1.0, label='Reconstruida',
             linestyle='--', alpha=0.85, zorder=4)
    ax1.fill_between(t, error, color=C_ERR, alpha=0.25, label='Error')
    ax1.set_xlabel('Tiempo (s)')
    ax1.set_ylabel('Amplitud')
    ax1.set_title('Señal en el tiempo')
    ax1.legend(loc='upper right', fontsize=9)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(t[0], t[100])

    # ── [2] Espectro de magnitud ─────────────────────────────
    ax2.plot(freqs, mags_orig, color=C_ORIG, lw=1.0, label='Original', zorder=3)
    ax2.fill_between(freqs, mags_comp, color=C_COMP, alpha=0.55,
                     label='Comprimido', zorder=2)
    ax2.set_xlabel('Frecuencia (Hz)')
    ax2.set_ylabel('Amplitud')
    ax2.set_title('Espectro de magnitud')
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(0, fs / 2)

    # ── [3] Espectro en dB ───────────────────────────────────
    ax3.plot(freqs, db_orig, color=C_ORIG, lw=1.0, label='Original', zorder=3)
    ax3.plot(freqs, db_comp, color=C_COMP, lw=0.8, label='Comprimido',
             linestyle='--', alpha=0.85, zorder=4)
    ax3.set_xlabel('Frecuencia (Hz)')
    ax3.set_ylabel('Magnitud (dB)')
    ax3.set_title('Espectro en dB')
    ax3.legend(fontsize=9)
    ax3.grid(True, alpha=0.3)
    ax3.set_xlim(0, fs / 2)

    # ── [4] Mapa de coeficientes conservados ─────────────────
    # Barras grises para todos, azules para los conservados
    ax4.bar(freqs, mags_orig, width=freqs[1]-freqs[0] if len(freqs)>1 else 1,
            color='#CBD5E1', label='Descartados', zorder=2)
    ax4.bar(freqs[mascara_pos], mags_orig[mascara_pos],
            width=freqs[1]-freqs[0] if len(freqs)>1 else 1,
            color=C_KEPT, label='Conservados', zorder=3)
    ax4.set_xlabel('Frecuencia (Hz)')
    ax4.set_ylabel('Amplitud')
    ax4.set_title(f'Coeficientes conservados ({mascara_pos.sum()} de {mitad})')
    ax4.legend(fontsize=9)
    ax4.grid(True, alpha=0.3, axis='y')
    ax4.set_xlim(0, fs / 2)

    # ── [5] Energía acumulada espectral ──────────────────────
    ax5.plot(pct_coefs, acum_pct, color=C_ACUM, lw=1.5, zorder=3)
    # Línea de corte: fracción de coeficientes usados
    corte_x = 100 * r['coefs_usados'] / N_pad
    corte_y = r['energia_conservada'] * 100
    ax5.axvline(corte_x, color=C_COMP, lw=1.2, linestyle='--', zorder=4,
                label=f'Corte: {corte_x:.1f}% coefs')
    ax5.axhline(corte_y, color=C_REC,  lw=1.2, linestyle=':', zorder=4,
                label=f'{corte_y:.1f}% energía')
    ax5.scatter([corte_x], [corte_y], color=C_COMP, s=50, zorder=5)
    ax5.set_xlabel('% de coeficientes usados')
    ax5.set_ylabel('% de energía acumulada')
    ax5.set_title('Energía acumulada vs coeficientes')
    ax5.set_xlim(0, 100)
    ax5.set_ylim(0, 101)
    ax5.legend(fontsize=9)
    ax5.grid(True, alpha=0.3)
    ax5.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f'{v:.0f}%'))

    plt.tight_layout()

    if guardar_png:
        ruta_png = f"{nombre}_analisis.png"
        fig.savefig(ruta_png, dpi=150, bbox_inches='tight')
        print(f"  [PNG] Guardado: {ruta_png}")

    return fig


def graficar_comparativo(resultados, guardar_png=False):
    """
    Figura comparativa cuando se procesan múltiples señales.
    Muestra para cada una: espectro original, espectro comprimido
    y métricas clave en un grid compacto.
    """
    n  = len(resultados)
    if n == 0:
        return

    fig, axes = plt.subplots(n, 3, figsize=(15, 3.2 * n))
    if n == 1:
        axes = axes[np.newaxis, :]  # garantizar 2D

    fig.suptitle('Comparativa de Compresión Espectral Adaptativa',
                 fontsize=13, fontweight='bold', y=1.01)

    for i, r in enumerate(resultados):
        fs     = r['fs']
        N_pad  = r['N_pad']
        N_orig = r['N_orig']
        X      = r['X_original']
        X_comp = r['X_comprimido']
        x_orig = r['señal_original']
        x_rec  = r['señal_reconstruida']

        mitad  = N_pad // 2
        freqs  = np.arange(mitad) * fs / N_pad
        t      = np.arange(N_orig) / fs

        mags_orig = np.abs(X[:mitad]) * 2 / N_orig
        mags_comp = np.abs(X_comp[:mitad]) * 2 / N_orig
        eps       = 1e-10
        db_orig   = 20 * np.log10(mags_orig + eps)
        db_comp   = 20 * np.log10(mags_comp + eps)

        nombre_corto = r['nombre'].replace('_', ' ')
        pct_coefs    = (1 - r['ratio']) * 100

        # Columna 0: señal en el tiempo
        ax = axes[i, 0]
        ax.plot(t, x_orig, color=C_ORIG, lw=0.9, label='Original')
        ax.plot(t, x_rec,  color=C_REC,  lw=0.8, linestyle='--',
                alpha=0.9, label='Reconstruida')
        ax.set_ylabel(nombre_corto, fontsize=9, fontweight='bold')
        ax.set_xlim(t[0], t[-1])
        ax.grid(True, alpha=0.25)
        if i == 0:
            ax.set_title('Señal en el tiempo', fontsize=10)
            ax.legend(fontsize=8, loc='upper right')

        # Columna 1: espectro de magnitud
        ax = axes[i, 1]
        ax.plot(freqs, mags_orig, color=C_ORIG, lw=0.9)
        ax.fill_between(freqs, mags_comp, color=C_COMP, alpha=0.5)
        ax.set_xlim(0, fs / 2)
        ax.grid(True, alpha=0.25)
        if i == 0:
            ax.set_title('Espectro de magnitud', fontsize=10)

        # Columna 2: espectro en dB + métricas
        ax = axes[i, 2]
        ax.plot(freqs, db_orig, color=C_ORIG, lw=0.9, label='Original')
        ax.plot(freqs, db_comp, color=C_COMP, lw=0.7, linestyle='--',
                alpha=0.9, label='Comprimido')
        ax.set_xlim(0, fs / 2)
        ax.grid(True, alpha=0.25)
        if i == 0:
            ax.set_title('Espectro en dB', fontsize=10)
            ax.legend(fontsize=8)

        # Anotación con métricas en el margen derecho
        ax.annotate(
            f"coefs: {r['coefs_usados']}/{N_pad} ({pct_coefs:.1f}%)\n"
            f"SNR: {r['snr_db']:.1f} dB\n"
            f"MSE: {r['mse']:.5f}",
            xy=(1.02, 0.5), xycoords='axes fraction',
            fontsize=8, va='center',
            bbox=dict(boxstyle='round,pad=0.3', fc='#F1F5F9', ec='#CBD5E1')
        )

    # Etiquetas de ejes solo en la última fila
    for ax in axes[-1, :]:
        ax.set_xlabel('Frecuencia (Hz)' if ax != axes[-1, 0] else 'Tiempo (s)',
                      fontsize=9)

    plt.tight_layout()

    if guardar_png:
        ruta_png = "comparativa_espectral.png"
        fig.savefig(ruta_png, dpi=150, bbox_inches='tight')
        print(f"  [PNG] Guardado: {ruta_png}")

    return fig


# ─────────────────────────────────────────────
#  6.  Experimento y Métricas
# ─────────────────────────────────────────────

def comprimir_señal(señal, fs, fraccion_energia=0.95, nombre="señal",
                    guardar_wav=True, verbose=True):
    señal = np.asarray(señal, dtype=float)
    t0    = time.time()

    print(f"\n{'='*60}")
    print(f"  {nombre}")
    print(f"  {len(señal):,} muestras · {fs} Hz · {len(señal)/fs:.3f} s")
    print(f"  Objetivo: conservar {fraccion_energia*100:.0f}% de la energía")
    print(f"{'='*60}")

    # Paso 1: FFT
    X, N_orig, N_pad = fft_con_padding(señal)
    t_fft = time.time() - t0
    print(f"  [FFT]  N_orig={N_orig}  N_pad={N_pad}  t={t_fft*1000:.1f} ms")

    # Paso 2: Compresión espectral
    X_comp, indices, ratio = comprimir_espectro(X, fraccion_energia)
    ec = energia_conservada(X, X_comp)
    print(f"  [Compresión]  {len(indices)}/{N_pad} coefs  "
          f"({(1-ratio)*100:.1f}% usados)  energía={ec*100:.2f}%")

    # Paso 3: IFFT
    t1 = time.time()
    señal_rec = ifft(X_comp)[:N_orig].real
    t_ifft = time.time() - t1

    # Paso 4: Métricas
    _mse = mse(señal, señal_rec)
    _snr = snr_db(señal, señal_rec)
    print(f"  [IFFT]  t={t_ifft*1000:.1f} ms  MSE={_mse:.6f}  SNR={_snr:.1f} dB")

    # Paso 5: WAV
    if guardar_wav:
        escribir_wav(f"{nombre}_original.wav",     señal,     fs)
        escribir_wav(f"{nombre}_reconstruida.wav", señal_rec, fs)
        print(f"  [WAV]  {nombre}_original.wav  /  {nombre}_reconstruida.wav")

    return {'nombre': nombre, 'N_orig': N_orig, 'N_pad': N_pad,
            'coefs_usados': len(indices), 'ratio': ratio,
            'energia_conservada': ec, 'mse': _mse, 'snr_db': _snr,
            'señal_original': señal, 'señal_reconstruida': señal_rec,
            'X_original': X, 'X_comprimido': X_comp,
            'indices_usados': indices, 'fs': fs}


# ─────────────────────────────────────────────
#  7.  Menú Interactivo
# ─────────────────────────────────────────────

def main():
    print("\n" + "="*60)
    print("  COMPRESIÓN ESPECTRAL ADAPTATIVA CON FFT")
    print("="*60)
    print("\n  Fuente de la señal:")
    print("   1. Sinusoide pura")
    print("   2. Suma de sinusoides (acorde do-mi-sol)")
    print("   3. Onda cuadrada (armónicos impares)")
    print("   4. Ruido blanco (peor caso para compresión espectral)")
    print("   5. Archivo .wav propio")
    print("   6. Todos los ejemplos (vista comparativa)")
    op = input("\n  Opción [1-7]: ").strip()

    v = input("  Energía a conservar (0.0–1.0) [0.95]: ").strip()
    try:
        fra = float(v)
        if not 0 < fra <= 1:
            raise ValueError
    except:
        fra = 0.95

    rs = []

    if op == '1':
        f = float(input("  Frecuencia (Hz) [440]: ").strip() or "440")
        rs.append(comprimir_señal(señal_senoidal(f), 4000, fra, f"senoidal_{int(f)}Hz"))

    elif op == '2':
        rs.append(comprimir_señal(
            señal_suma_senoidales([261, 329, 392, 523, 659], [1.0, 0.8, 0.9, 0.4, 0.3]),
            4000, fra, "acorde_doMiSol"))

    elif op == '3':
        f = float(input("  Frecuencia fundamental (Hz) [220]: ").strip() or "220")
        rs.append(comprimir_señal(señal_cuadrada(f), 4000, fra, f"cuadrada_{int(f)}Hz"))

    elif op == '4':
        rs.append(comprimir_señal(señal_ruido_blanco(4096), 4000, fra, "ruido_blanco"))

    elif op == '5':
        ruta = input("  Ruta al .wav: ").strip()
        if not os.path.exists(ruta):
            print(f"  ERROR: no existe '{ruta}'")
            return
        señal, fs, ch = leer_wav(ruta)
        print(f"  Leído: {len(señal):,} muestras · {fs} Hz · {ch} canal(es)")
        if len(señal) > 2 * fs:
            señal = señal[:2 * fs]
            print(f"  (Usando primeros 2 s = {2*fs} muestras)")
        nombre = os.path.splitext(os.path.basename(ruta))[0]
        rs.append(comprimir_señal(señal, fs, fra, nombre))

    elif op == '6':
        for nombre, señal, fs in [
            ("senoidal_440Hz",   señal_senoidal(250),                                   4000),
            ("acorde_doMiSol",   señal_suma_senoidales([261, 329, 392, 523, 659],
                                                        [1.0, 0.8, 0.9, 0.4, 0.3]),    4000),
            ("cuadrada_220Hz",   señal_cuadrada(250),                                   4000),
            ("chirp_100_2000Hz", señal_chirp(),                                         4000),
            ("ruido_blanco",     señal_ruido_blanco(4096),                              4000),
        ]:
            rs.append(comprimir_señal(señal, fs, fra, nombre))
    else:
        print("  Opción inválida.")
        return

    # ── Visualización ────────────────────────────────────────
    guardar = input("\n  ¿Guardar gráficas como PNG? [s/N]: ").strip().lower() == 's'

    if len(rs) == 1:
        # Vista detallada de 5 paneles para una sola señal
        graficar_resultado(rs[0], guardar_png=guardar)
    else:
        # Vista comparativa en grid cuando hay múltiples señales
        graficar_comparativo(rs, guardar_png=guardar)

    print("\n  Mostrando gráficas... cierra la ventana para terminar.")
    plt.show()
    print("  Experimento terminado.\n")


if __name__ == "__main__":
    main()