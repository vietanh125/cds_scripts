import numpy as np
import cv2

IMG_W = 320
IMG_H = 160

def preprocess_img(img, ratio=8, cut=50):
    img = cv2.resize(img, (IMG_W, IMG_H))
    h, w = img.shape
    img = img[cut:, :]
    h -= cut
    img = cv2.resize(img, (w//ratio, h//ratio))
    return img

def remove_noise(img, k=3):
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(k,k))
    # img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    # img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    img = cv2.dilate(img, img)

    return img

def remove_ground(img, padding = 1, distance=2):
    h, w = img.shape
    for i in range(w):
        for j in range(h):
            if j >= h - distance:
                img[j][i] = 0
            else:
                if img[j][i] >= 1500:
                    img[j][i] = 0
                if img[j][i] >= img[j+distance][i] + 1:
                    smooth = True
                    for k in range(distance):
                        if img[j + k][i] < img[j+k+1][i]:
                            smooth = False
                            break
                    if smooth:
                        img[j][i] = 0
            if img[j][i] != 0:
                img[j][i] = 1
    return img

def obstackle_classify(pred, cnt):
    x, y, w, h = cnt
    # print cnt
    rec = pred[y: y+h, x: x+w]
    rec = cv2.Laplacian(rec, cv2.CV_64F)
    # print rec.shape
    rec = rec[1:-1, 1:-1]
    _, rec = cv2.threshold(rec, 0, 1, cv2.THRESH_BINARY)
    s = rec.sum()
    ratio = float(s/(h+w))
    if ratio > 0.6:
        return True
    return False

def find_obstacle(img, seg_img, k=3, min_area=10):
    # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
    # img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    rects = []
    _, contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if cv2.contourArea(cnt) > min_area and obstackle_classify(seg_img, cnt):
            rects.append(mapping_rect(cnt, 8, 50, 0.1))
    return rects



def mapping_rect(cnt, ratio, cut, padding):
    (x, y, w, h ) = cv2.boundingRect(cnt)

    x *= ratio
    w *= ratio
    y*= ratio
    h *= ratio

    y += cut

    x = max(0, int(x-w*padding))
    y = max(0, int(y-h*padding*16))
    w = min(IMG_W-1, int(w*(padding*2+1)))
    h = min(IMG_H-1, int(h*(padding*32+1)))
    return (x, y, w, h)


def get_restriction(rects):
    x_left = IMG_W // 2
    x_right = IMG_W // 2
    extra = 20
    for rect in rects:
        x, y, w, h = rect
        # print(rect)
        x = x
        w = w
        # cv2.rectangle(image, (x, 0), (x+w, IMG_H), (0, 255, 0), 1)

        if x > IMG_W // 2:
            if x > x_right:
                x_right = x

        if (x + w) < IMG_W // 2:
            if (x + w) < x_left:
                x_left = x + w

        if x < IMG_W // 2 and x + w > IMG_W / 2:
            if IMG_W / 2 - x > x + w - IMG_W / 2:
                x_left = x + w
                x_right = IMG_W - 1
            else:
                x_left = 0
                x_right = x

    if x_right != IMG_W // 2 and x_right <= 0.8 * IMG_W:
        i_right = x_right
        # i_right = 0.5 * IMG_W
    else:
        i_right = IMG_W - 1

    if x_left != IMG_W // 2 and x_right >= 0.2 * IMG_W:
        i_left = x_left
        # i_left = 0.5 * IMG_W
    else:
        i_left = 0
    return int(i_left), int(i_right)


