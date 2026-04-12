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
