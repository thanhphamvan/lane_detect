import cv2
import numpy as np
import os

def circle_contour(img, img_blob):
    img_blob_clone = img_blob.copy()
    img_contour, contours, hierarchy = cv2.findContours(np.array(img_blob, dtype=np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) != 0:
        largest_area = 0
        largest_area_id = 0

        for i in range(len(contours)):
            if cv2.contourArea(contours[i]) > largest_area:
                largest_area = cv2.contourArea(contours[i])
                largest_area_id = i

        contours_mask = img_blob.copy()
        cv2.drawContours(contours_mask, contours, largest_area_id, 255, thickness=cv2.FILLED)
        contours_mask = cv2.morphologyEx(contours_mask, cv2.MORPH_CLOSE, np.ones((5, 5)))

        # get foreground
        fg = np.uint8(cv2.bitwise_or(img_blob, img_blob, mask=np.uint8(contours_mask)))

        # get background
        mask = cv2.bitwise_not(np.uint8(contours_mask))
        background = np.full(img_blob.shape, 255, dtype=np.uint8)
        bk = cv2.bitwise_or(background, background, mask=mask)        

        # combine
        sign = 255 - cv2.bitwise_or(fg, bk)

        return sign, contours_mask
    else:
        return None, None


def get_boundingbox(imblob):
    x, y = np.where(imblob)
    
    if x.shape[0] == 0 or y.shape[0] == 0:
        return None, None, None, None

    px = np.round((np.max(y) + np.min(y)) / 2)
    py = np.round((np.max(x) + np.min(x)) / 2)
    width = np.max(y) - np.min(y)
    height = np.max(x) - np.min(x)

    return px, py, width, height


def color_threshold(img):
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    blue_lower = np.array([100, 150, 0], np.uint8)
    blue_upper = np.array([140, 255, 255], np.uint8)

    mask_blue = cv2.inRange(img_hsv, blue_lower, blue_upper)

    blob = cv2.bitwise_and(img_hsv, img_hsv, mask=mask_blue)

    blob = cv2.cvtColor(blob, cv2.COLOR_HSV2BGR)

    return blob


def remove_noise(blob):
    blob = cv2.cvtColor(blob, cv2.COLOR_BGR2GRAY)
    blob = cv2.threshold(blob, 50, 255, cv2.THRESH_BINARY)[1]

    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(blob)

    img_remove_noise = np.zeros(labels.shape)
    max_cc = np.max(stats[:, 4])
    for i in range(num_labels):
        size_proportion = max(stats[i, 2] / stats[i, 3], stats[i, 3] / stats[i, 2])

        if stats[i, 4] >= 10 and stats[i, 4] != max_cc and 1 <= size_proportion < 2:
            img_remove_noise[labels == i] = 255

    return img_remove_noise


def display_cc(labels):
    # Map component labels to hue val
    label_hue = np.uint8(179*labels/np.max(labels))
    blank_ch = 255*np.ones_like(label_hue)
    labeled_img = cv2.merge([label_hue, blank_ch, blank_ch])

    # cvt to BGR for display
    labeled_img = cv2.cvtColor(labeled_img, cv2.COLOR_HSV2BGR)

    # set bg label to black
    labeled_img[label_hue == 0] = 0

    cv2.imshow('labeled.png', labeled_img)
    cv2.waitKey()
