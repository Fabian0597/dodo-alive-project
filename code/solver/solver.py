import numpy as np
from scipy.integrate import solve_ivp


class AbstractSolver:
    """
    abstract solver class/interface to implement different types of solver
    """

    def integrate(self, fun, t_span, x0):
        """
        solve ordinary differential equations for a given time span or until events (guard functions) are triggered
        :param fun: flow function of the system
        :param t_span: time span to solve
        :param x0: initial state
        :return: time and states of all time steps
        """
        pass


class OwnSolverRK45(AbstractSolver):
    """
    own solver suing RK 45 method with fixed step_size but without root finding (exact state where event = 0)
    """

    def __init__(self, step_size):
        self.step_size = step_size
        self.previous_guard_value = dict()

    def integrate(self, fun, t_span, x0, events, step_callback=None):
        t = np.arange(t_span[0], t_span[1] + self.step_size, self.step_size)
        steps = len(t)
        x = [x0]
        for k in range(steps - 1):
            # do a single step
            v4, v5 = self._do_pri45_step(fun, t[k], x[k], t[k + 1] - t[k])
            x.append(x[k] + (t[k + 1] - t[k]) * v5)

            # search for zero points in event functions if given
            if events:
                for event in events:
                    guard_value = event(t[k+1], x[k+1])
                    if event in self.previous_guard_value.keys():
                        sign_change = (self.previous_guard_value[event] < 0 and guard_value > 0) or \
                                      (self.previous_guard_value[event] > 0 and guard_value < 0)
                        if sign_change:
                            # end integrator because event / guard function is changing sign of value (found zero point)
                            return t[:len(x)], np.array(x)
                    self.previous_guard_value[event] = guard_value

            if step_callback:
                step_callback(t[k+1], x[k+1])
        return t, np.array(x)

    @staticmethod
    def _do_pri45_step(f, t, x, h):
        k1 = f(t, x)
        k2 = f(t + 1. / 5 * h, x + h * (1. / 5 * k1))
        k3 = f(t + 3. / 10 * h, x + h * (3. / 40 * k1 + 9. / 40 * k2))
        k4 = f(t + 4. / 5 * h, x + h * (44. / 45 * k1 - 56. / 15 * k2 + 32. / 9 * k3))
        k5 = f(t + 8. / 9 * h, x + h * (19372. / 6561 * k1 - 25360. / 2187 * k2 + 64448. / 6561 * k3 - 212. / 729 * k4))
        k6 = f(t + h, x + h * (9017. / 3168 * k1 - 355. / 33 * k2 + 46732. / 5247 * k3 + 49. / 176 * k4 - 5103. / 18656 * k5))

        v5 = 35. / 384 * k1 + 500. / 1113 * k3 + 125. / 192 * k4 - 2187. / 6784 * k5 + 11. / 84 * k6
        k7 = f(t + h, x + h * v5)
        v4 = 5179. / 57600 * k1 + 7571. / 16695 * k3 + 393. / 640 * k4 - 92097. / 339200 * k5 + 187. / 2100 * k6 + 1. / 40 * k7

        return v4, v5


class SolverIVP(AbstractSolver):
    """
    using python ivp solver function
    """

    def integrate(self, fun, t_span, x0, events):
        result = solve_ivp(
            fun=fun,
            t_span=t_span, y0=x0,
            max_step=0.2,
            events=events,
        )
        return result.t, result.y.T
