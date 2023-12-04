import cv2
import numpy as np
import glob
import os

# Especifica las dimensiones del tablero (número de esquinas internas)
dimensiones_esquinas = (10,9)  # Ajusta estas dimensiones según tu tablero

# Lista para almacenar las coordenadas de las esquinas en el mundo 3D
objpoints = []

# Lista para almacenar las coordenadas de las esquinas en las imágenes
imgpoints = []

# Crea un conjunto de puntos en el mundo real (esquinas del tablero)
objp = np.zeros((dimensiones_esquinas[0] * dimensiones_esquinas[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:dimensiones_esquinas[1], 0:dimensiones_esquinas[0]].T.reshape(-1, 2)

# Rutas de las imágenes
images = glob.glob('/home/benjamin/Desktop/black_and_white_procesadas/Visual_imga/*.png')  # Cambia la ruta según tu ubicación

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
        cv2.drawChessboardCorners(img, dimensiones_esquinas, corners, ret)
        cv2.imshow('img', img)
        cv2.waitKey(0)        
    else:
        print(f"No se encontraron esquinas válidas en la imagen: {fname}")

# Encuentra los parámetros intrínsecos de la cámara
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Calcula el error de reproyección
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    mean_error += error

print(f"Error de reproyección: {mean_error / len(objpoints)}")

# Muestra la matriz de calibración
print("Coeficientes de distorsión:", dist)
print("Matriz de calibración:")
print(mtx)

# Crea una carpeta para las imágenes desdistorcionadas
output_folder = "undistorted_images"
os.makedirs(output_folder, exist_ok=True)

# Guarda imágenes desdistorcionadas
for i, fname in enumerate(images):
    img = cv2.imread(fname)
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

    # desdistorciona
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

    # recorta la imagen
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    cv2.imwrite(f'{output_folder}/undistorted_{i}.png', dst)

# Cierra la ventana de la imagen
cv2.destroyAllWindows()