class PIDController:
    """PID controller with explicit term instrumentation.

    The returned output matches Vishnu's original implementation.  The
    additional last_* fields make P/I/D behavior observable in logs.
    """

    def __init__(
        self,
        Kp,
        Ki,
        Kd,
        setpoint=0.0,
        output_limits=(None, None),
        integral_limits=(None, None),
        derivative_filter_alpha=0.1,
        derivative_on_measurement=True,
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
        self.reset()

    def reset(self):
        self._integral = 0.0
        self._prev_error = None
        self._prev_measurement = None
        self._derivative = 0.0

        self.last_error = 0.0
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0
        self.last_raw_derivative = 0.0
        self.last_unsaturated_output = 0.0
        self.last_output = 0.0
        self.last_saturated = False

    def update(self, measurement, dt):
        if dt <= 0:
            raise ValueError("dt must be positive")

        error = self.setpoint - measurement
        p_term = self.Kp * error

        previous_integral = self._integral
        candidate_integral = previous_integral + error * dt
        if self._integral_min is not None:
            candidate_integral = max(
                self._integral_min, candidate_integral
            )
        if self._integral_max is not None:
            candidate_integral = min(
                self._integral_max, candidate_integral
            )
        i_term = self.Ki * candidate_integral

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
        self._derivative = (
            self._alpha * raw_derivative
            + (1.0 - self._alpha) * self._derivative
        )
        d_term = self.Kd * self._derivative

        unsaturated = p_term + i_term + d_term
        output = unsaturated
        if self._output_min is not None:
            output = max(self._output_min, output)
        if self._output_max is not None:
            output = min(self._output_max, output)

        # Conditional integration prevents a nonzero Ki from winding up
        # farther into a saturated output. Current baseline Ki values are
        # zero, so this does not alter the measured P-only behavior.
        drives_upper = (
            self._output_max is not None
            and unsaturated > self._output_max
            and error > 0.0
        )
        drives_lower = (
            self._output_min is not None
            and unsaturated < self._output_min
            and error < 0.0
        )
        if self.Ki != 0.0 and (drives_upper or drives_lower):
            candidate_integral = previous_integral
            i_term = self.Ki * candidate_integral
            unsaturated = p_term + i_term + d_term
            output = unsaturated
            if self._output_min is not None:
                output = max(self._output_min, output)
            if self._output_max is not None:
                output = min(self._output_max, output)

        self._integral = candidate_integral

        self.last_error = error
        self.last_p = p_term
        self.last_i = i_term
        self.last_d = d_term
        self.last_raw_derivative = raw_derivative
        self.last_unsaturated_output = unsaturated
        self.last_output = output
        self.last_saturated = output != unsaturated

        return output
