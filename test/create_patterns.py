import cv2
import numpy as np
import os

# Tamaño de la imagen blanca
img_size = 400
# Tamaño del marcador
marker_size = 200

# Crear una imagen blanca de 400x400
white_image = np.ones((img_size, img_size, 3), dtype=np.uint8) * 255

# Crear un objeto de diccionario
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Generar los marcadores
marker25 = cv2.aruco.generateImageMarker(aruco_dict, 25, marker_size)
marker30 = cv2.aruco.generateImageMarker(aruco_dict, 30, marker_size)
marker35 = cv2.aruco.generateImageMarker(aruco_dict, 35, marker_size)

# Convertir los marcadores a color
marker25_color = cv2.cvtColor(marker25, cv2.COLOR_GRAY2BGR)
marker30_color = cv2.cvtColor(marker30, cv2.COLOR_GRAY2BGR)
marker35_color = cv2.cvtColor(marker35, cv2.COLOR_GRAY2BGR)

# Coordenadas para centrar los marcadores en la imagen blanca
top_left = ((img_size - marker_size) // 2, (img_size - marker_size) // 2)

# Colocar los marcadores en la imagen blanca
white_marker25 = white_image.copy()
white_marker25[top_left[1]:top_left[1] + marker_size, top_left[0]:top_left[0] + marker_size] = marker25_color

white_marker30 = white_image.copy()
white_marker30[top_left[1]:top_left[1] + marker_size, top_left[0]:top_left[0] + marker_size] = marker30_color

white_marker35 = white_image.copy()
white_marker35[top_left[1]:top_left[1] + marker_size, top_left[0]:top_left[0] + marker_size] = marker35_color

# Crear la carpeta patterns si no existe
if not os.path.exists('patterns'):
    os.makedirs('patterns')

# Guardar las imágenes de los marcadores
cv2.imwrite('patterns/marker25.png', white_marker25)
cv2.imwrite('patterns/marker30.png', white_marker30)
cv2.imwrite('patterns/marker35.png', white_marker35)
