import numpy as np
import matplotlib.pyplot as plt 
import matplotlib.image as mpimg
from scipy import ndimage
import os

# RGB --> grayscale
def rgb2gray(img):
    return (np.dot(img[...,:3], [0.2989, 0.5870, 0.1140])).astype('float32')

#display image                                                                  
def view_img(img_name):
    plt.imshow(img_name, cmap = plt.get_cmap('gray'))
    plt.show() 
    
'''
Part A) Filtered gradient: Compute x and y gradients Fx and Fy, the same as in the Canny edge detector.
'''    
# Method to do convolution of input image with sobel filter to smoothen the image and finding x & y gradients
def convolveWithGaussianDerivative(img):
  sobel_x = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]], np.float32)
  sobel_y = np.array([[1, 2, 1], [0, 0, 0], [-1, -2, -1]], np.float32)
  
  Ix = ndimage.filters.convolve(img, sobel_x)
  Iy = ndimage.filters.convolve(img, sobel_y)
  
  return Ix, Iy

'''
B) Find corners: For each pixel (x, y), look in a window of size 2m+1 x 2m+1around the pixel (you can use m = 4). Accumulate over
this window the covariance matrix C, which contains the average of the products of x and y gradients:
'''

# Method to determine R score for each pixel [R_mat]
def Calculate_Rscore_mat(img, Iy, Ix, m, k):
    Ixx = np.square(Ix)
    Ixy = Iy*Ix
    Iyy = np.square(Iy)
    height, width = img.shape[0], img.shape[1]

    R_mat = np.zeros([height, width],dtype='float64')

    for y in range(m, height-m):
        for x in range(m, width-m):
            Kxx = Ixx[y-m:y+m+1, x-m:x+m+1]
            Kxy = Ixy[y-m:y+m+1, x-m:x+m+1]
            Kyy = Iyy[y-m:y+m+1, x-m:x+m+1]
            Sxx = np.sum(Kxx)
            Sxy = np.sum(Kxy)
            Syy = np.sum(Kyy)

            determinant = (Sxx * Syy) - np.square(Sxy)
            trace = Sxx + Syy
            R_mat[y][x] =  determinant - (k*np.square(trace))
    return R_mat

# Function to detect corners based on threshold values and r score
def DetectCorners(img, r_mat,Th_ratio):
  corner_list = []
  Th = Th_ratio*r_mat.max()
  for y in range(img.shape[0]):
    for x in range(img.shape[1]):
      if r_mat[y][x] > Th:
        corner_list.append([y,x,r_mat[y][x]])
  return corner_list

# Function to highlight corners in the input image
def MakeCornerImg(img, corner_list):
  out1 = np.zeros(img.shape,img.dtype)
  out2 = img.copy()
  
  for i in range(len(corner_list)):
    y = corner_list[i][0]
    x = corner_list[i][1]
    out1[y][x][0] = 1.0
    out1[y][x][1] = 1.0
    out1[y][x][2] = 1.0
    out2[y][x][0] = 1.0
    out2[y][x][1] = 0.0
    out2[y][x][2] = 0.0
  return out1, out2

def GenerateResults(input_folder, output_folder, Th_ratio):
    # Asegurarse de que la carpeta de salida exista
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    img_names = os.listdir(input_folder)
    for img_name in img_names:
        if img_name.endswith(".bmp"):
            img_path = os.path.join(input_folder, img_name)
            img = mpimg.imread(img_path).astype('float32')/255.0  # Leer imagen y convertir a float
            Ix, Iy = convolveWithGaussianDerivative(rgb2gray(img))

            R_mat = Calculate_Rscore_mat(img, Ix, Iy, m=4, k=0.04)
            temp = R_mat / np.max(R_mat)

            result_path_R = os.path.join(output_folder, f"{img_name}_R_Value.jpg")
            plt.imsave(result_path_R, temp, cmap=plt.get_cmap('gray'))

            corner_list = DetectCorners(img, R_mat, Th_ratio)

            out1, out2 = MakeCornerImg(img, corner_list)
            result_path_corners_beforeNMS = os.path.join(output_folder, f"{img_name}_corners_beforeNMS.jpg")
            plt.imsave(result_path_corners_beforeNMS, out2)

            sorted_corner_list = sorted(corner_list, key=lambda x: x[2], reverse=True)
            final_corner_list = []  # final_l contains list after non-maximal suppression
            final_corner_list.append(sorted_corner_list[0][:-1])
            dis = 3
            xc, yc = [], []
            for i in sorted_corner_list:
                for j in final_corner_list:
                    if abs(i[0] - j[0]) <= dis and abs(i[1] - j[1]) <= dis:
                        break
                else:
                    final_corner_list.append(i[:-1])
                    xc.append(i[1])
                    yc.append(i[0])

            print(f'Corner pixels detected in {img_name}: {len(corner_list)}')
            print(f'Corner pixels detected after NMS in {img_name}: {len(final_corner_list)}')

            corner_img = np.zeros(img.shape)
            for i in final_corner_list:
                y, x = i[0], i[1]
                corner_img[y][x] = 1

            output_path_marked = os.path.join(output_folder, f"{img_name}_corner_points_marked.jpg")
            plt.imshow(img, cmap=plt.get_cmap('gray'))
            plt.plot(xc, yc, '+', color='red')
            plt.savefig(output_path_marked)
            plt.show()

            output_path_dots = os.path.join(output_folder, f"{img_name}_corner_points_dots.jpg")
            plt.imshow(corner_img, cmap=plt.get_cmap('gray'))
            plt.imsave(output_path_dots, corner_img)
            plt.show()

# Carpeta de entrada y salida
carpeta_entrada = "/home/benjamin/Desktop/useful/unidos"
carpeta_salida = "/home/benjamin/Desktop/black_and_white_procesadas/canny"

# Seleccionar la relación de umbral correspondiente a cada imagen
Th_ratios = {"bicycle": 0.05, "bird": 0.05, "dog": 0.05, "einstein": 0.05, "plane": 0.01, "toy_image": 0.001}

for img_name, Th_ratio in Th_ratios.items():
    GenerateResults(os.path.join(carpeta_entrada, img_name), carpeta_salida, Th_ratio)
