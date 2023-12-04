#elimina_imagenes

import cv2
import numpy as np
import glob
import os

# Especifica las dimensiones del tablero (número de esquinas internas)
dimensiones_esquinas = (10, 9)  # Ajusta estas dimensiones según tu tablero

# Lista para almacenar las imágenes sin esquinas válidas
imagenes_sin_esquinas = []

# Crea un conjunto de puntos en el mundo real (esquinas del tablero)
objp = np.zeros((dimensiones_esquinas[0] * dimensiones_esquinas[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:dimensiones_esquinas[1], 0:dimensiones_esquinas[0]].T.reshape(-1, 2)

# Ruta de la carpeta que contiene las imágenes
carpeta_imagenes = '/home/benjamin/Desktop/useful/8bits_30_11'  # Cambia la ruta según tu ubicación

# Rutas de las imágenes
images = glob.glob(os.path.join(carpeta_imagenes, '*.png'))

# Lista para almacenar las coordenadas de las esquinas en el mundo 3D
objpoints = []

# Lista para almacenar las coordenadas de las esquinas en las imágenes
imgpoints = []

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Encuentra las esquinas del tablero
    ret, corners, meta = cv2.findChessboardCornersSBWithMeta(
        gray, dimensiones_esquinas, cv2.CALIB_CB_LARGER + cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    if ret and len(corners) == dimensiones_esquinas[0] * dimensiones_esquinas[1]:
        objpoints.append(objp)
        imgpoints.append(corners)
        #cv2.drawChessboardCorners(img, dimensiones_esquinas, corners, ret)
        #cv2.imshow('img', img)
        #cv2.waitKey(0)
    else:
        print(f"No se encontraron esquinas válidas en la imagen: {fname}")
        imagenes_sin_esquinas.append(fname)

# Cierra la ventana de la imagen
cv2.destroyAllWindows()

# Elimina las imágenes sin esquinas válidas
for img_sin_esquinas in imagenes_sin_esquinas:
    os.remove(img_sin_esquinas)

print(f"Se eliminaron {len(imagenes_sin_esquinas)} imágenes sin esquinas válidas.")
