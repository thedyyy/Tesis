import os
import cv2
import numpy as np

# Valores intrínsecos
focal_length = np.array([469.3271, 471.1443])  # Focal length in pixels (fx, fy)
principal_point = np.array([318.3376, 254.2911])  # Principal point (cx, cy)
radial_distortion = np.array([-0.5151, 0.3666, -0.0626])  # Radial distortion (k1, k2, k3)

# Crear la matriz de la cámara K
camera_matrix = np.array([[focal_length[0], 0, principal_point[0]],
                          [0, focal_length[1], principal_point[1]],
                          [0, 0, 1]])

# Crear el vector de distorsión D
dist_coeffs = np.array([radial_distortion[0], radial_distortion[1], 0, 0, radial_distortion[2]])

# Directorio de las imágenes
carpeta_imagenes = "/home/benjamin/Desktop/black_and_white_procesadas/canny"  # Reemplaza con la ruta de tu carpeta

# Iterar sobre todas las imágenes en la carpeta
for nombre_archivo in os.listdir(carpeta_imagenes):
    if nombre_archivo.endswith(('.jpg', '.jpeg', '.png')):  # Filtrar por extensiones de imagen
        # Cargar la imagen
        ruta_imagen = os.path.join(carpeta_imagenes, nombre_archivo)
        imagen = cv2.imread(ruta_imagen)

        # Desdistorsionar la imagen
        imagen_desdistorsionada = cv2.undistort(imagen, camera_matrix, dist_coeffs)

        # Mostrar la imagen original y desdistorsionada
        cv2.imshow("Original", imagen)
        cv2.imshow("Desdistorsionada", imagen_desdistorsionada)
        cv2.waitKey(0)

# Cerrar las ventanas cuando termines
cv2.destroyAllWindows()
