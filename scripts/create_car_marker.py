import cv2
import cv2.aruco as aruco
import sys

img = aruco.drawMarker(aruco.Dictionary_get(aruco.DICT_4X4_50), int(sys.argv[1]), 2000)
cv2.imwrite("car_marker_4x4_50_id_" + sys.argv[1] + ".jpg", img)