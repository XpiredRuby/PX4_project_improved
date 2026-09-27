class PIDController:
    def __init__(
        self, Kp, Ki, Kd,
        setpoint=0.0,
        output_limits=(None, None),
        integral_limits=(None, None),
        derivative_filter_alpha=0.1,
        derivative_on_measurement=True
    ):

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.setpoint = setpoint

        self.derivative_on_measurement = derivative_on_measurement

        self._integral = 0.0

        self._prev_error = None
        self._prev_measurement = None

        self._derivative = 0.0
        if not (0.0 < derivative_filter_alpha <= 1.0):
            raise ValueError("derivative_filter_alpha must be between 0 and 1")
        self._alpha = derivative_filter_alpha

        self._integral_min, self._integral_max = integral_limits
        self._output_min, self._output_max = output_limits

    def reset(self):
        self._integral = 0.0

        self._prev_error = None
        self._prev_measurement = None

        self._derivative = 0.0

    def update(self, measurement, dt):
        if dt <= 0:
            raise ValueError("dt must be positive")

        error = self.setpoint - measurement

        # --------------------
        # P term
        # --------------------
        P = self.Kp * error

        # --------------------
        # I term
        # --------------------
        self._integral += error * dt
        if self._integral_min is not None:
            self._integral = max(self._integral_min, self._integral)

        if self._integral_max is not None:
            self._integral = min(self._integral_max, self._integral)
        I = self.Ki * self._integral

        # --------------------
        # D term
        # --------------------
        if self.derivative_on_measurement:
            if self._prev_measurement is None:
                raw_derivative = 0.0
            else:
                raw_derivative = -(measurement - self._prev_measurement) / dt
        else:
            if self._prev_error is None:
                raw_derivative = 0.0
            else:
                raw_derivative = (error - self._prev_error) / dt
        self._prev_measurement = measurement
        self._prev_error = error    
        # Low pass filter
        self._derivative = (self._alpha * raw_derivative + (1-self._alpha) * self._derivative)
        D = self.Kd * self._derivative

        output = P + I + D

        # Output limits
        if self._output_min is not None:
            output = max(self._output_min, output)
        if self._output_max is not None:
            output = min(self._output_max, output)

        return output