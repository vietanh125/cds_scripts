import cv2
import numpy as np
from keras.models import load_model
import rospkg
rospack = rospkg.RosPack()
path = rospack.get_path('team107') + '/scripts/'
model = load_model(path + 'updated.h5')
model._make_predict_function()
model.predict(np.zeros((1, 24, 24, 1)))
n = 5
lowerBound = np.array([98, 109, 100])
upperBound = np.array([112, 255, 255])
kernelOpen = np.ones((n, n))
kernelClose = np.ones((n, n))


def detect(img):
    img_h, img_w, _ = img.shape
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(imgHSV, lowerBound, upperBound)
    # maskOpen = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernelOpen)
    # maskClose = cv2.morphologyEx(maskOpen, cv2.MORPH_CLOSE, kernelClose)
    _, conts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for i in range(0, len(conts)):
        x, y, w, h = cv2.boundingRect(conts[i])
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 1)
        crop = img[max(0, y - 1):min(y + h + 1, img_h), max(0, x - 1): min(x + w + 1, img_w)]
        crop = cv2.resize(crop, (24, 24))
        crop = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        crop = np.expand_dims(crop, 0)
        crop = np.expand_dims(crop, 3)
        pred = model.predict(crop)[0][0]
        s = w * h
        if pred >= 0.85:
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
            return -1, s
        elif pred <= 0.15:
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            return 1, s
        else:
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            return 0, 0


