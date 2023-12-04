import cv2
import numpy as np
import glob
import save_calibration as sc
import cam_calibration as cc
import extract_template as et
import json
import copy
import os
import json

# Abre el archivo json y lo carga en la variable "diccionario"
with open('diccionario.json', 'r') as f:
    diccionario = json.load(f)

inputJSON='param.json'

nombre_archivo = 'diccionario.json'

try:
    with open(nombre_archivo, 'r') as archivo:
        try:
            diccionario = json.load(archivo)
        except json.JSONDecodeError:
            print(f"El archivo {nombre_archivo} está vacío o mal formateado.")
            diccionario = {}
except FileNotFoundError:
    print(f"El archivo {nombre_archivo} no se encontró. Creando un nuevo diccionario.")
    diccionario = {}

if __name__ == "__main__":
	fp = open(inputJSON)
	projectDict = json.load(fp)
	fp.close()
	img_folder    = projectDict["IMG_DIR"]
	template_folder=projectDict["TEMPLATE_DIR"]
	BOARD_SIZE_R=projectDict["BOARD_ROW"]
	BOARD_SIZE_C=projectDict["BOARD_COL"]
	MATCH_THRESH=projectDict["MATCH_THRESH"]
	VISUALIZE=projectDict["VISUALIZE"]
	img_type=projectDict["IMG_TYPE"]
	board_type=projectDict["BOARD_TYPE"]
	save_undistort_img=projectDict["SAVE_UNDISTORT_IMG"]
	save_result=projectDict["SAVE_CALIB_RESULT"]
	save_dir=projectDict["RESULT_DIR"]
	IMG_FORMAT=projectDict["IMG_FORMAT"]
	TEMPLATE_FORMAT=projectDict["TEMPLATE_FORMAT"]
	EXTRACT_TMP=projectDict["EXTRACT_TEMPLATE"]
	TEMPLATE_SIZE=projectDict["TEMPLATE_SIZE"]
	if(EXTRACT_TMP):
		et.extract_template(template_folder,img_folder,IMG_FORMAT,TEMPLATE_FORMAT,TEMPLATE_SIZE )
	#load calibration images and templates
	images=glob.glob(img_folder+'*'+IMG_FORMAT)
	print(images)
	template=glob.glob(template_folder+'*'+TEMPLATE_FORMAT)
	img=cv2.imread(images[0])  
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
	objp = np.zeros((BOARD_SIZE_R*BOARD_SIZE_C,3), np.float32)
	objp[:,:2] = np.mgrid[0:BOARD_SIZE_C,0:BOARD_SIZE_R].T.reshape(-1,2)

	# Arrays to store object points and image points from all the images.
	run_own=0
	imgpoints,objpoints=cc.opencv_find_crosspts(images,objp,BOARD_SIZE_R,BOARD_SIZE_C,draw_b=VISUALIZE)
	if(len(imgpoints)!=len(images)):
		print('Opencv missed!')
		imgpoints=[]
		objpoints=[]
		run_own=1
	else:
		print('Opencv handled.')
	if(run_own):
		images2 = [os.path.basename(img) for img in images]
		count=0
		img_count=0
		for fname in images2:
			print(fname)
			if fname in diccionario:
				print('ya se tiene el diccionario')
				#img=cv2.imread(fname)  
				#gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)				
				corners=diccionario[fname]
				print('estas son las esquinas de ' + str(fname) + str(corners))
				ret=True

			else:
				img=cv2.imread(fname)  
				gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
				ret, corners = cc.find_crosspoints_m(fname, BOARD_SIZE_R, BOARD_SIZE_C)
				corners_dict = corners
				print('este es el ret'+ str(ret))
                
				if ret == True:
					#corners_copy = np.copy(corners)
                    #diccionario[fname] = corners_copy.tolist()
					diccionario[fname] = corners_dict.copy()
					print(f"Pattern found: {fname}")


					
			#ret,corners = cc.find_crosspoints(fname,template,MATCH_THRESH,BOARD_SIZE_R,BOARD_SIZE_C,img_type,board_type)
			if(VISUALIZE and ret):
				cc.draw_board(img,corners,BOARD_SIZE_R,BOARD_SIZE_C,arrow=True)
				cv2.imshow('Draw_Board',img)
				cv2.waitKey(0)
		    # If found, add object points, image points (after refining them)
			if ret == True:
				# Make a deep copy of the corners before modifying them
				corners_copy = copy.deepcopy(corners)
				for i in range(0, len(corners_copy)):
					for j in range(0, len(corners_copy[i])):
						corners_copy[i][j] = (np.float32(corners_copy[i][j][1]), np.float32(corners_copy[i][j][0]))
				cor = np.array(corners_copy)
				objpoints.append(objp)
				imgpoints.append(cor)
				img_count += 1
				print(f"Pattern found: {fname}")
				print(f"Pattern found: {fname}")
			else:
				count+=1
				print(f"Pattern not found: {fname}")
		print("Num of invalid img: {}".format(count))
		print("Num of used img: {}".format(img_count))
	ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

	#########################################
	#save calibration results
	#########################################
	if(save_result):
		sc.save_calibration(save_dir+"calibration_result.txt",mtx,dist,rvecs,tvecs)
	print(mtx)
	print(dist)

	#########################################
	#display undistorted image
	#########################################
	if(save_undistort_img):
		count=0
		for fname in images:
			img=cv2.imread(fname)
			h,w=img.shape[:2]
			newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
			dst=cv2.undistort(img,mtx,dist,None,newcameramtx)
			x, y, w, h = roi
			cv2.imwrite('undistort'+str(count)+'.jpg',dst)
			count+=1

	#########################################
	#re-projection error
	#########################################
	mean_error = 0
	for i in range(len(objpoints)):
	    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
	    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
	    mean_error += error

	def default(obj):
		if isinstance(obj, np.float32):
			return float(obj)
		raise TypeError(f"Object of type '{type(obj).__name__}' is not JSON serializable")

	with open(nombre_archivo, 'w') as archivo:
		json.dump(diccionario, archivo, default=default)

	print( "mean error: {}".format(mean_error/len(objpoints)))

	cv2.destroyAllWindows()
