from time import time
from keras.models import load_model
t = time()
model = load_model("updated.h5")
import numpy as np
fps = 0
count = 0
while(True):
    t = time()
    model.predict(np.ones((256, 48, 48, 1)))
    fps += 1/(time()-t)
    count += 1
    if count == 50:
        print fps/count
        count = 0
        fps = 0