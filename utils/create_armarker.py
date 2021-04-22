import cv2

dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

markerImage = cv2.aruco.drawMarker(dictionary,2,100)
cv2.imwrite("marker2.png", markerImage)
