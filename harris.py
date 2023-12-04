import cv2
import os
import numpy as np

def unsharp_mask(image, sigma=1.0, strength=1.5):
    blurred = cv2.GaussianBlur(image, (0, 0), sigma)
    sharpened = cv2.addWeighted(image, 1.0 + strength, blurred, -strength, 0)
    return sharpened

def harris_corner_detector(image, k=0.1, threshold=0.0001):
    # Convertir la imagen a escala de grises
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(np.float32)
    else:
        gray = image.astype(np.float32)

    # Calcular las derivadas parciales
    Ix = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    Iy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)

    # Calcular productos de derivadas
    Ixx = Ix**2
    Ixy = Ix * Iy
    Iyy = Iy**2

    # Aplicar una ventana de Gauss a los productos de derivadas
    ksize = 3
    sigma = 1
    Ixx = cv2.GaussianBlur(Ixx, (ksize, ksize), sigma)
    Ixy = cv2.GaussianBlur(Ixy, (ksize, ksize), sigma)
    Iyy = cv2.GaussianBlur(Iyy, (ksize, ksize), sigma)

    # Calcular la matriz de covarianza para cada píxel
    det_M = Ixx * Iyy - Ixy**2
    trace_M = Ixx + Iyy
    R = det_M - k * trace_M**2

    # Aplicar umbral y encontrar las coordenadas de las esquinas
    corners = np.where(R > threshold * R.max())

    # Dibujar círculos en las esquinas encontradas
    result = image.copy()
    for y, x in zip(*corners):
        cv2.circle(result, (x, y), 5, (0, 0, 255), 2)

    return result

def procesar_imagenes_en_carpeta(carpeta_entrada, carpeta_salida):
    # Asegurarse de que la carpeta de salida exista
    if not os.path.exists(carpeta_salida):
        os.makedirs(carpeta_salida)

    # Obtener la lista de archivos en la carpeta de entrada
    archivos = os.listdir(carpeta_entrada)

    for archivo in archivos:
        # Leer la imagen
        ruta_entrada = os.path.join(carpeta_entrada, archivo)
        imagen = cv2.imread(ruta_entrada, cv2.IMREAD_GRAYSCALE)  # Leer la imagen en escala de grises

        # Aplicar filtro de ruido gaussiano
        imagen_suavizada = cv2.GaussianBlur(imagen, (3, 3), 0)

        # Aplicar unsharp mask para mejorar la nitidez
        imagen_nitida1 = unsharp_mask(imagen_suavizada)
        imagen_nitida = cv2.bilateralFilter(imagen_nitida1, d=9, sigmaColor=75, sigmaSpace=75)

        # Detectar esquinas de Harris
        bordes = harris_corner_detector(imagen_nitida)

        # Guardar la imagen procesada en la carpeta de salida
        ruta_salida = os.path.join(carpeta_salida, f"harris_{archivo}")
        cv2.imwrite(ruta_salida, bordes)

if __name__ == "__main__":
    carpeta_entrada = "/home/benjamin/Desktop/black_and_white_procesadas/stretched"
    carpeta_salida = "/home/benjamin/Desktop/black_and_white_procesadas/streched_harris"

    procesar_imagenes_en_carpeta(carpeta_entrada, carpeta_salida)
