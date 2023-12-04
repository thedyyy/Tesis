import cv2
import numpy as np
import glob
import os
import pandas as pd

# Especifica las dimensiones del tablero (número de esquinas internas)
dimensiones_esquinas = (10, 9)  # Ajusta estas dimensiones según tu tablero

# Lista para almacenar las coordenadas de las esquinas en el mundo 3D
objpoints = []

# Lista para almacenar las coordenadas de las esquinas en las imágenes
imgpoints = []

# Lista para almacenar el error de proyección de cada imagen
imagenes_validas = []
errores_proyeccion = []

# Crea un conjunto de puntos en el mundo real (esquinas del tablero)
objp = np.zeros((dimensiones_esquinas[0] * dimensiones_esquinas[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:dimensiones_esquinas[1], 0:dimensiones_esquinas[0]].T.reshape(-1, 2)

# Rutas de las imágenes
images = glob.glob('/home/benjamin/Desktop/useful/izq_izq_inf_inf/*.png')  # Cambia la ruta según tu ubicación

for fname in images:
    print('este es el nombre' + str(fname))
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Encuentra las esquinas del tablero
    ret, corners, meta = cv2.findChessboardCornersSBWithMeta(
        gray, dimensiones_esquinas, cv2.CALIB_CB_LARGER + cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    if ret and len(corners) == dimensiones_esquinas[0] * dimensiones_esquinas[1]:
        print('este es el nombre' + str(fname))
        objpoints.append(objp)
        imgpoints.append(corners)

        imagenes_validas.append(fname)
        # Encuentra la matriz de proyección
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        # Calcula el error de reproyección para la imagen actual
        imgpoints2, _ = cv2.projectPoints(objp, rvecs[-1], tvecs[-1], mtx, dist)
        error = cv2.norm(corners, imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        errores_proyeccion.append(error)
    else:
        print(f"No se encontraron esquinas válidas en la imagen: {fname}")

# Crear un DataFrame de Pandas con los errores de proyección
df_errores_proyeccion = pd.DataFrame({"Imagen": imagenes_validas, "Error de Proyección": errores_proyeccion})

# Guardar el DataFrame en un archivo CSV
output_file = "lista_proyecciones.csv"
df_errores_proyeccion.to_csv(output_file, index=False)

print(f"Errores de proyección guardados en: {output_file}")