import matlab.engine
import time
import numpy as np
import pdb

# eng = matlab.engine.start_matlab()

t = 0

for i in range(100):

    t0 = time.time()
    Mnpy = np.random.uniform(size=(10, 10))
    # M = matlab.single(np.random.uniform(size=(50, 50)).tolist())
    Minv = np.linalg.inv(Mnpy)
    # Minv = eng.matrix_inverse(M)
    print("Iteration: " + str(i))
    print("Freq: " + str(1 / (time.time() - t0)))
    t += (time.time() - t0)

print(t / 100)
