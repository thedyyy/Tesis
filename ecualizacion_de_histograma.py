#ecualizacion_de_histograma.py

import cv2
import numpy as np
import os


def histogram_saturated(imagen):
    A = np.copy(imagen)
    a, b = np.where(A >= 255)
    c, d = np.where(A <= 0)
    A[a, b] = 255
    A[c, d] = 0

    return A

def equal_hist_gray(imagen):
    im = cv2.imread(imagen, cv2.IMREAD_GRAYSCALE)

    # Saturar el histograma de la imagen en escala de grises
    ch_gray = histogram_saturated(im)

    L, B = im.shape

    gray_equalized = np.zeros((L, B)).astype(ch_gray.dtype)

    # Calcular el histograma acumulativo normalizado
    hist_gray_normalized = (np.cumsum(cv2.calcHist([im], [0], None, [256], [0, 256]))) / np.amax(
        (np.cumsum(cv2.calcHist([im], [0], None, [256], [0, 256]))))

    for i in range(L):
        for j in range(B):
            gray_equalized[i, j] = np.floor((255 - 1) * hist_gray_normalized[ch_gray[i, j]])

    return gray_equalized

if __name__ == "__main__":
    carpeta_entrada = "/home/benjamin/Desktop/black_and_white_procesadas/sobel (copy)"
    carpeta_salida = "/home/benjamin/Desktop/black_and_white_procesadas/sobel"

    # Asegurarse de que la carpeta de salida exista
    if not os.path.exists(carpeta_salida):
        os.makedirs(carpeta_salida)

    # Obtener la lista de archivos en la carpeta de entrada
    archivos = os.listdir(carpeta_entrada)

    for archivo in archivos:
        # Leer la imagen
        ruta_entrada = os.path.join(carpeta_entrada, archivo)
        imagen_procesada = equal_hist_gray(ruta_entrada)

        # Guardar la imagen procesada en la carpeta de salida
        ruta_salida = os.path.join(carpeta_salida, f"equalized_{archivo}")
        cv2.imwrite(ruta_salida, imagen_procesada)
