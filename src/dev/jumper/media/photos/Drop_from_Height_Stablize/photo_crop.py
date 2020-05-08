import cv2
import os


img_list = os.listdir(".")

#print(img_list)

x1,y1,x2,y2 = 50, 60, 1365, 770

for img in img_list:
	if(img[-3:] == ".py" or img[-8:-4] == "_mod"):
		print("python file / modified file -- skip")
		continue
	print(img)
	img_raw = cv2.imread(img)
	crop_img = img_raw[y1:y2, x1:x2]
	#cv2.imshow("cropped", img_raw)
	cv2.imwrite(img[:-4]+"_mod"+img[-4:], crop_img)
