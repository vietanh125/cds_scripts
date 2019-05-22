import numpy as np
import cv2

IMG_W = 320
IMG_H = 160

def preprocess_img(img,ratio):
    h, w = img.shape
    img = img[h//5:, :]
    h = 4*h//5
    img = cv2.resize(img, (w//ratio, h//ratio), cv2.INTER_NEAREST)
    return img

def longest_array(a, padding, distance_from_center, k):
    start, end, length = 0, 0, 0

    i = 0
    j = 0
    window = 0

    while (j < len(a) - 1):
        if(a[i] == 0):
            i += 1
            j += 1
            continue

        j += 1
        # while a[j] > a[i] and j < len(a) - 1:
        # comment for new ideas
        if a[j] >= a[j-1] - padding + distance_from_center*k:
            if j - i > length:
                length = j - i
                start = i
                end = j
        else:
            i += 1
            j = i
        # end of comment
    if length > 2:
        return start, end, length
    else:
        return 0, 0, 0

def longest_array_hieu(a, padding, distance_from_center, k):
    start, end, length = 0, 0, 0
    i = len(a) - 1
    j = len(a) - 1
    while (j > 0):
        if (a[i] == 0):
            i -= 1
            j -= 1
            continue
        j -= 1
        if a[j] <= a[i] + padding - distance_from_center * k:
            if i - j > length:
                length = i - j
                start = j
                end = i
        else:
            i -= 1
            j = i
    if length > 5:
        return start, end, length
    else:
        return 0

def find_object(img, padding, k):
    h, w = img.shape
    res = []
    for i in range(w):
        res.append(longest_array(img[:, i], padding, abs(w//2 - i), k))

    filtered = np.zeros((h, w), dtype=np.uint8)
    for r in range(len(res)):
        if res[r] == 0:
            continue
        for j in range(res[r][0], res[r][1]+1):
            filtered[j, r] = 1

    return filtered


def remove_road(img, road_distance=17, padding=3, far_distance=60):
    h, w = img.shape
    i = 1
    while i < h:
        max_pix = -1
        for j in range(w):
            if img[-i, j] > far_distance or img[-i, j] == 0:
                img[-i, j] = road_distance
                continue
            if img[-i, j] <= road_distance + padding and img[-i,j] >= road_distance - padding:
                if max_pix < img[-i, j]:
                    max_pix = img[-i, j]
                img[-i, j] = road_distance
        if max_pix > road_distance:
            road_distance = max_pix
        i += 1

    return img

def remove_noise(img, k):
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(k,k))
    # img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    return img

def obstackle_classify(pred, cnt):
    x, y, w, h = cnt
    # print x, y, w, h
    # cv2.imshow("pred", pred)
    rec = pred[y: y+h, x: x+w]
    # cv2.imshow("cont", rec)
    # cv2.waitKey(1)
    rec = cv2.Laplacian(rec, cv2.CV_64F)
    # print rec.shape
    rec = rec[1:-1, 1:-1]
    _, rec = cv2.threshold(rec, 0, 1, cv2.THRESH_BINARY)
    s = rec.sum()
    ratio = float(s/(h+w))
    if ratio > 0.5:
        return True
    return False

def find_obstacles(img, pred, size_min, ratio):
    rects = []
    # _,img = cv2.threshold(img,1,255,0)
    _, contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if cv2.contourArea(cnt) > size_min:
            x, y, w, h = cv2.boundingRect(cnt)
            x *= ratio
            w *= ratio
            y *= ratio
            h *= ratio

            y += IMG_H//5
            extend = 0.1

            x = max(0, int(x - IMG_W*extend))
            y = max(0, int(y-IMG_H*extend))
            w = min(int(w+2*IMG_W*extend), IMG_W-x)
            h = min(int(h + 2*IMG_H*extend), IMG_H - y)
            if obstackle_classify(pred, [x, y, w, h]):
                rects.append([x, y, w, h])
        # print(x, y, w, h)
        # print(x, y, w, h)
    return rects

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

