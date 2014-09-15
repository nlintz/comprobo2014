class PIDController(object):
	""" PID Controller Class
	
	"""
	def __init__(self, target=0, pConst=0, iConst=0, dConst=0, tolerance=0):
		 # Initial target value for controller
		self.target = target
		
		# Coefficients for PID control
		self._pConst = pConst
		self._iConst = iConst
		self._dConst = dConst

		# Variables used for tracking errors over time
		self._tolerance = tolerance
		self._previousError = 0
		self._totalError = 0

	@property
	def p(self):
		return self._pConst

	@property 
	def i(self):
		return self._iConst

	@property
	def d(self):
		return self._dConst

	def getFeedback(self, reading):
		""" returns the feedback from control system for a given reading

		reading -- Number sensor reading
		"""
		error = self.target - reading
		if abs(error) < self._tolerance:
			error = 0
		feedback = self._proportionalTerm(error) + self._derivativeTerm(error) + self._integralTerm(error)
		return feedback

	def _proportionalTerm(self, error):
		""" returns the proportional term of the PID control loop

		error -- Number error between reading and target
		"""
		return error * self._pConst

	def _integralTerm(self, error):
		""" returns the integral term of the PID control loop

		error -- Number error between reading and target
		"""
		self._totalError += error
		return self._totalError * self._iConst

	def _derivativeTerm(self, error):
		""" returns the derivative term of the PID control loop

		error -- Number error between reading and target
		"""
		dError = error - self._previousError
		# print dError, dTime
		self._previousError = error
		return dError * self._dConst

	def _deltaTime(self):
		""" returns the <float> change in time since the last reading
		"""
		currentTime = time.time() 
		dTime = currentTime - self._previousTime
		self._previousTime = currentTime
		return float(dTime)



