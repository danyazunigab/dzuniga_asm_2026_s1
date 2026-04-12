'''  <><><><><><><><><><><><><><><><>            IMPORTS             <><><><><><><><><><><><><><><><>   '''
'''  numpy: para manejo de arrays y operaciones matemáticas con numeros complejos.                      '''
'''  matplotlib.pyplot: Graficas y comparativas.                                                        '''
'''  time: Mediciones de tiempo.                                                                        '''

import numpy as np
import matplotlib.pyplot as plt
import time
import os

'''  <><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>   '''

'''  <><><><><><><><><><><><><><><><>   DISCRETE FOURIER TRANSFORM   <><><><><><><><><><><><><><><><>   '''

def calculate_by_dft(x):
    '''  Se convierte x a un arreglo complejo.                                                          '''
    x_as_array = np.asarray(x, dtype=complex)
    N = len(x_as_array)
    '''  Se inicializa el arreglo de salida X con ceros complejos.                                      '''
    X = np.zeros(N, dtype=complex)
    
    '''  Se declara el twiddle factor.                                                                  '''
    W_N = np.exp(-2j * np.pi / N)
    
    for k in range(N):
        for n in range(N):
            '''  Se determina el twiddle factor en funcion de n y k.                                    '''
            W_N_nk = W_N ** (n * k)
            
            '''  Se agrega al arreglo de salida.                                                        '''
            X[k] += x_as_array[n] * W_N_nk
    
    return X

'''  <><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>   '''

'''  <><><><><><><><><><><><><><><><>     FAST FOURIER TRANSFORM     <><><><><><><><><><><><><><><><>   '''

def calculate_by_fft(x):
    '''  Se convierte x a un arreglo complejo.                                                          '''
    x_as_array = np.asarray(x, dtype=complex)
    N = len(x_as_array)
    
    '''  Se verifica que N sea una potencia de 2.                                                       '''
    if N < 0 and (N & (N - 1) != 0):
        raise ValueError("Input signal length must be a power of 2. Current input length: {}".format(N))
    
    '''  Caso base: si el arreglo tiene un solo elemento, se devuelve ese elemento como resultado.      '''
    if N == 1:
        return x_as_array.copy()
    
    '''  Se divide el arreglo en sus subpartes pares e impares.                                         '''
    x_as_array_even = x_as_array[0::2]
    x_as_array_odd = x_as_array[1::2]
    
    '''  Se llama recursivamente a la función para calcular las DFT de las partes pares e impares.      '''
    E = calculate_by_fft(x_as_array_even)
    O = calculate_by_fft(x_as_array_odd)
    
    '''  Se inicializa el arreglo de salida X con ceros complejos.                                      '''
    X = np.zeros(N, dtype=complex)
    
    '''  Se utiliza un k que va desde 0 hasta N/2 - 1.                                                  '''
    k = np.arange(N // 2)
    
    '''  Se declara el twiddle factor y se eleva a todos los valores de k.                              '''
    W_N_k = np.exp(-2j * np.pi / N) ** k
    
    '''  Se determina la primera mitad de la salida.                                                    '''
    X[:N // 2] = E + W_N_k * O
    
    '''  Se determina la segunda mitad de la salida.                                                    '''
    X[N // 2:] = E - W_N_k * O
    
    return X


'''  <><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>   '''

'''  <><><><><><><><><><><><><><><><>           DFT V FFT            <><><><><><><><><><><><><><><><>   '''

def run_time_analysis_test(x):
    print("<>" * 8, "   RUNNING TIME ANALYSIS   ", "<>" * 8)
    print("\n")
    
    '''  Se mide el tiempo de ejecución de la DFT.                                                        '''
    start_time_dft = time.time()
    dft_result = calculate_by_dft(x)
    end_time_dft = time.time()
    dft_time = end_time_dft - start_time_dft
    
    '''  Se mide el tiempo de ejecución de la FFT.                                                        '''
    start_time_fft = time.time()
    fft_result = calculate_by_fft(x)
    end_time_fft = time.time()
    fft_time = end_time_fft - start_time_fft
    
    '''  Se imprime el resultado de ambos cálculos y los tiempos de ejecución.                            '''
    print("DFT RESULT:", dft_result)
    print("FFT RESULT:", fft_result)
    print("DFT EXECUTION TIME: {:.6f} seconds".format(dft_time))
    print("FFT EXECUTION TIME: {:.6f} seconds".format(fft_time))
    
    return fft_result


def run_benchmark_tests():
    print("<>" * 32, "   RUNNING BENCHMARK TESTS   ", "<>" * 32)
    print("\n\n")
    
    N  = 1024
    fs = 1000.0
    t  = np.arange(N) / fs
    freqs = np.arange(N) * fs / N
    freqs_half = freqs[:N // 2]
    
    '''  Señal 1: Señal sinusoidal simple.                                                                  '''
    f0 = 60.0
    s1 = 2.0 * np.cos(2 * np.pi * f0 * t)
    
    '''  Señal 2: Señal con diferentes frecuencias y amplitudes.                                            '''
    f1, f2, f3 = 60.0, 120.0, 200.0
    a1, a2, a3 = 3.0, 1.5, 0.8
    s2 = (  a1 * np.cos(2 * np.pi * f1 * t)
           + a2 * np.cos(2 * np.pi * f2 * t)
           + a3 * np.cos(2 * np.pi * f3 * t)
    )
    
    '''  Señal 3: Onda cuadrada.                                                                            '''
    f_4 = 25.0
    s3 = np.sign(np.sin(2 * np.pi * f_4 * t))
    
    
    signals = [
        ("Simple sinusoidal signal (60 Hz)", s1),
        ("Signal with varying f/a (60, 120, 200 Hz)", s2),
        ("Square wave signal (25 Hz)", s3),
    ]
    
    fig, axes = plt.subplots(len(signals), 3, figsize=(16, 4 * len(signals)))
    
    for i, (signal, x) in enumerate(signals):
        print("<>" * 16, "   PROCESSING SIGNAL: {}   ".format(signal), "<>" * 16)
        print("\n")
                
        X = run_time_analysis_test(x)
        
        '''  Se calcula el espectro de magnitud y fase.                                                     '''
        magnitude = np.abs(X[:N // 2]) * 2.0 / N
        phase = np.angle(X[:N // 2])
        umbral_phase = np.max(magnitude) * 0.01
        phase_clean = np.where(magnitude > umbral_phase, phase, 0)
        
        '''  Se genera la gráfica de señal en el tiempo.                                                    '''
        axes[i, 0].plot(t[:200] * 1000, x[:200], color='#2c3e50', linewidth=0.8)
        axes[i, 0].set_title(f'{signal}', fontsize=10, fontweight='bold')
        axes[i, 0].set_xlabel('Time (ms)')
        axes[i, 0].set_ylabel('Amplitude')
        axes[i, 0].grid(True, alpha=0.3)
        
        '''  Se genera la gráfica del espectro de magnitud.                                                 '''
        axes[i, 1].stem(freqs_half, magnitude, linefmt='-', markerfmt='o', basefmt='k-')
        axes[i, 1].set_title('Magnitude Spectrum |X[k]|', fontsize=10, fontweight='bold')
        axes[i, 1].set_xlabel('Frequency (Hz)')
        axes[i, 1].set_ylabel('Amplitude')
        axes[i, 1].set_xlim(0, 300)
        axes[i, 1].grid(True, alpha=0.3)
        
        ''' Se genera la gráfica del espectro de fase.                                                      '''
        axes[i, 2].stem(freqs_half, np.degrees(phase_clean), 
                        linefmt='-', markerfmt='o', basefmt='k-')
        axes[i, 2].set_title('Phase Spectrum ∠X[k]', fontsize=10, fontweight='bold')
        axes[i, 2].set_xlabel('Frequency (Hz)')
        axes[i, 2].set_ylabel('Phase (degrees)')
        axes[i, 2].set_xlim(0, 300)
        axes[i, 2].grid(True, alpha=0.3)
        
        print("\n")
    
    plt.tight_layout()
    output_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "data"))
    os.makedirs(output_dir, exist_ok=True)
    plt.savefig(os.path.join(output_dir, "benchmark_tests.png"), dpi=150, bbox_inches='tight')
    plt.close()
    
    print("\n")
    print("<>" * 32, "   BENCHMARK TESTS ENDED   ", "<>" * 32)

'''  <><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>   '''

'''  <><><><><><><><><><><><><><><><>         FUNCTION CALL          <><><><><><><><><><><><><><><><>   '''

run_benchmark_tests()

'''  <><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>   '''