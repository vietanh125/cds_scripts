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
lowerBound = np.array([90, 80, 50])
upperBound = np.array([110, 255, 255])

def detect(img):
    img_h, img_w, _ = img.shape
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(imgHSV, lowerBound, upperBound)
    # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernelOpen)
    # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernelClose)
    conts, dum2 = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    list_pred = []
    for i in range(0, len(conts)):
        x, y, w, h = cv2.boundingRect(conts[i])
        if w*h >= 1000 and (w/h > 0.7 and w/h < 1.3):
            crop = img[max(0, y - 1):min(y + h + 1, img_h), max(0, x - 1): min(x + w + 1, img_w)]
            crop = cv2.resize(crop, (24, 24))
            crop = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
            crop = np.expand_dims(crop, 0)
            crop = np.expand_dims(crop, 3)
            pred = model.predict(crop)[0][0]
            s = w * h
            acc = abs(0.5 - pred)
            signal = np.sign(0.5 - pred)
            list_pred.append([[x, y, h, w, signal], [acc]])
    if len(list_pred) > 0:
        argmax = np.argmax(list_pred, axis=0)
        [x, y, h, w, signal], [acc] = list_pred[argmax[1]]
        pred = signal * acc + 0.5
        if pred >= 0.7:
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
            return -1, s
        elif pred <= 0.3:
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            return 1, s
    return 0, 0
    # else:
    #     cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
    #     return 0, 0
