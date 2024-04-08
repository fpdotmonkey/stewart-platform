/// A simple PI servo controller for a pneumatic muscle cylinder
///
/// For the integration, this maintains an accumulator that tracks
/// error.
///
/// For a primer on how PI controllers work, see
/// <https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller>
pub struct CylinderPositionController {
    k_p: f64,
    k_i: f64,
    setpoint: f64,
    error_accumulator: f64,
}

/// The gains and the controller type you'd like
pub enum ControlGains {
    P(f64),
    PI(f64, f64),
}

impl CylinderPositionController {
    /// Generate a controller that regulates toward setpoint
    ///
    /// The gains should be selected with some degree of intelligence
    /// to get reasonable results.  See e.g.
    /// <https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method>
    /// for a procedure for how to select gains.
    pub fn new(gains: ControlGains, setpoint: f64) -> Self {
        let (k_p, k_i) = match gains {
            ControlGains::P(k_p) => (k_p, 0.0),
            ControlGains::PI(k_p, k_i) => (k_p, k_i),
        };
        Self {
            k_p,
            k_i,
            setpoint,
            error_accumulator: 0.0,
        }
    }

    /// This aims the controller to regulate toward a new value
    ///
    /// As a side-effect, it resets the integration accumulator.
    pub fn new_setpoint(&mut self, setpoint: f64) -> &mut Self {
        self.setpoint = setpoint.clamp(0.0, 1.0);
        self.error_accumulator = 0.0;
        self
    }

    /// Generate a control signal with a PI controller
    ///
    /// It's assumed, but not enforced, that measurement signal is in
    /// the interval [0.0, 1.0].
    pub fn control_signal(&mut self, measurement_signal: f64) -> f64 {
        let error: f64 = self.setpoint - measurement_signal;
        self.error_accumulator += error;
        self.k_p * error + self.k_i * self.error_accumulator
    }
}

#[cfg(test)]
mod tests {
    use float_cmp::approx_eq;

    use super::*;

    /// Assert `left ¤ right`, where ¤ == op is a boolean infix function
    macro_rules! assert_op {
        ($op:tt, $left:expr, $right:expr $(,)?) => ({
            match (&$left, &$right) {
                (left_val, right_val) => {
                    if !(*left_val $op *right_val) {
                        panic!("assertion `left {} right` failed\n  left: `{:?}`\n right: `{:?}`",
                               stringify!($op), left_val, right_val)
                    }
                }
            }
        });
    }

    #[test]
    fn controller_drives_in_the_correct_direction() {
        let mut controller = CylinderPositionController::new(ControlGains::P(1.0), 0.5);
        assert_op!(>, controller.control_signal(0.0), 0.0);
        assert_op!(<, controller.control_signal(1.0), 0.0);
        assert_eq!(controller.control_signal(0.5), 0.0);
    }

    #[test]
    fn controller_setpoint_changes() {
        let mut controller = CylinderPositionController::new(ControlGains::P(1.0), 0.5);
        assert_op!(<, controller.new_setpoint(0.0).control_signal(0.5), 0.0);
        assert_eq!(controller.new_setpoint(0.5).control_signal(0.5), 0.0);
        assert_op!(>, controller.new_setpoint(1.0).control_signal(0.5), 0.0);
    }

    #[test]
    fn control_increases_proportional_to_error() {
        let mut controller = CylinderPositionController::new(ControlGains::P(1.0), 0.0);
        for k in 0..10 {
            let k = k as f64;
            assert!(approx_eq!(
                f64,
                controller.control_signal(0.1) * k,
                controller.control_signal(k * 0.1),
                ulps = 2
            ));
        }
        let controller = controller.new_setpoint(1.0);
        for k in 0..10 {
            let k = k as f64;
            assert!(approx_eq!(
                f64,
                controller.control_signal(0.9) * k,
                controller.control_signal(1.0 - (k * 0.1)),
                ulps = 2
            ));
        }
    }

    #[test]
    fn control_increases_with_integral_of_error() {
        let mut controller = CylinderPositionController::new(ControlGains::PI(1.0, 1.0), 0.0);
        let controller = controller.new_setpoint(0.0);
        let mut previous_control_signal: f64 = 0.0;
        for _ in 0..10 {
            let current_control_signal = controller.control_signal(0.1);
            assert_op!(>, current_control_signal.abs(), previous_control_signal.abs());
            previous_control_signal = current_control_signal;
        }
        let controller = controller.new_setpoint(1.0);
        let mut previous_control_signal: f64 = 0.0;
        for _ in 0..10 {
            let current_control_signal = controller.control_signal(0.9);
            assert_op!(>, current_control_signal.abs(), previous_control_signal.abs());
            previous_control_signal = current_control_signal;
        }
    }

    #[test]
    fn control_increases_with_higher_gains() {
        // proportional gain
        let mut controller0 = CylinderPositionController::new(ControlGains::P(1.0), 0.0);
        let mut controller1 = CylinderPositionController::new(ControlGains::P(10.0), 0.0);

        assert_op!(>, controller1.control_signal(1.0).abs(), controller0.control_signal(1.0).abs());

        // integral gain
        let mut controller0 = CylinderPositionController::new(ControlGains::PI(1.0, 1.0), 0.0);
        let mut controller1 = CylinderPositionController::new(ControlGains::PI(1.0, 10.0), 0.0);
        let _ = controller0.control_signal(1.0);
        let _ = controller1.control_signal(1.0);
        assert_op!(>, controller1.control_signal(1.0).abs(), controller0.control_signal(1.0).abs());
    }
}
