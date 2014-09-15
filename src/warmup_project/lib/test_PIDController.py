import pylab
import numpy as np
from PIDController import PIDController

def test_PIDControl():
	target = 50
	pidController = PIDController(target, .6, .1, .02)
	time = np.linspace(0, 100, 100)
	position = [0] # starting position
	for i in range(1, len(time)):
		velocity = pidController.getFeedback(position[i - 1])
		position.append(position[i - 1] + velocity)
	pylab.ylim(0, 60)
	pylab.plot(time, [target for i in range(100)], 'r', time, position, 'b.')
	pylab.show()


if __name__ == "__main__":
	test_PIDControl()