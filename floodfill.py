import cv2
import numpy as np
IMG_H, IMG_W = 160, 320

def fill(img):
    mask = np.zeros((IMG_H + 2, IMG_W + 2), np.uint8)
    bits = img[IMG_H-2:IMG_H-1, :]
    # ret, bits = cv2.threshold(bits, 0, 1, cv2.THRESH_BINARY)
    bounded = np.hstack(([0], bits.reshape(-1), [0]))
    difs = np.diff(bounded)
    run_starts = np.where(difs > 0)[0]
    run_ends = np.where(difs < 0)[0]
    if len(run_starts) == 0:
        return img
    i = np.argmax(run_ends-run_starts, axis=0)
    seed = int((run_ends[i] + run_starts[i])/2)
    if img[IMG_H - 2][seed] == 0:
        print "Wrong flood fill!!!"
    # seed = 160
    cv2.floodFill(img, mask, (seed, IMG_H-2), 1)
    # img_inv = cv2.bitwise_not(img)
    # fill_image = img | img_inv
    # cv2.imshow("my segment", mask*255.)
    return mask[1:161, 1:321]
