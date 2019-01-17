import cv2

def hu_moments_classify(binary_img):
	moments = cv2.moments(binary_img)
	hu_mm = cv2.HuMoments(moments)

	if hu_mm[6] > 0:
		return 0
	elif hu_mm[6] < 0:
		return 1
	else:
		return None
