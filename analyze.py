import numpy as np
import matplotlib.pyplot

backLegSensorValues = np.load("data/backleg.npy")
frontLegSensorValues = np.load("data/frontleg.npy")

matplotlib.pyplot.plot(backLegSensorValues, label = "Back Leg", linewidth = 2)
matplotlib.pyplot.plot(frontLegSensorValues, label = "Front Leg")
matplotlib.pyplot.legend()

matplotlib.pyplot.show()