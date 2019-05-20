# Example one
import numpy
import random
import pylab
import math


class KalmanFilterLinear:
    def __init__(self, _A, _B, _H, _x, _P, _Q, _R):
        self.A = _A  # state transition matrix
        self.B = _B  # control matrix
        self.H = _H  # observation matrix
        self.current_state_estimate = _x  # initial state estimate
        self.current_prob_estimate = _P  # invitial covariance estimate
        self.Q = _Q  # estimated error in process
        self.R = _R  # estimated error in measurements
        self.covariance = []

    def get_current_state(self):
        return self.current_state_estimate

    def step(self, control_vector, measurement_vector):
        # predict it
        predicted_state_estimate = (
            self.A * self.current_state_estimate + self.B * control_vector
        )
        predicted_prob_estimate = (
            self.A * self.current_prob_estimate
        ) * numpy.transpose(self.A) + self.Q
        # observe it
        innovation = measurement_vector - self.H * predicted_state_estimate
        innovation_covariance = (
            self.H * predicted_prob_estimate * numpy.transpose(self.H) + self.R
        )
        self.covariance = innovation_covariance
        # update it
        kalman_gain = (
            predicted_prob_estimate
            * numpy.transpose(self.H)
            * numpy.linalg.inv(innovation_covariance)
        )
        self.current_state_estimate = (
            predicted_state_estimate + kalman_gain * innovation
        )
        # update the other one
        size = self.current_prob_estimate.shape[0]
        self.current_prob_estimate = (
            numpy.eye(size) - kalman_gain * self.H
        ) * predicted_prob_estimate


class Voltmeter:
    def __init__(self, _truevoltage, _noiselevel):
        self.truevoltage = _truevoltage
        self.noiselevel = _noiselevel

    def get_voltage(self):
        return self.truevoltage

    def get_voltage_with_noise(self):
        return random.gauss(self.get_voltage(), self.noiselevel)


def example_one():
    A = numpy.matrix([1])
    H = numpy.matrix([1])
    B = numpy.matrix([0])
    Q = numpy.matrix([0.00001])
    R = numpy.matrix([0.1])
    xhat = numpy.matrix([3])
    P = numpy.matrix([1])

    voltmeter = Voltmeter(1.25, 1.0)
    voltmeter_filter = KalmanFilterLinear(A, B, H, xhat, P, Q, R)

    measured_voltage = []
    true_voltage = []
    kalman = []

    numsteps = 60
    for iteration in range(numsteps):
        measured = voltmeter.get_voltage_with_noise()
        measured_voltage.append(measured)
        true_voltage.append(voltmeter.get_voltage())
        kalman.append(voltmeter_filter.get_current_state()[0, 0])
        voltmeter_filter.step(numpy.matrix([0]), numpy.matrix([measured]))
    pylab.plot(
        range(numsteps),
        measured_voltage,
        "b",
        range(numsteps),
        true_voltage,
        "r",
        range(numsteps),
        kalman,
        "g",
    )
    pylab.xlabel("Time")
    pylab.ylabel("Voltage")
    pylab.title("Voltagey Kalmans")
    pylab.legend(("measured", "true voltage", "kalman"))
    pylab.show()


class Cannon:
    angle = 45  # degrees
    muzzle_velocity = 100  # units/sec
    gravity = [0, -9.81, 0]  # m/sec squared
    vel = [
        muzzle_velocity * math.cos(angle * math.pi / 180),
        muzzle_velocity * math.sin(angle * math.pi / 180),
        muzzle_velocity * math.cos(angle * math.pi / 180),
    ]
    loc = [0, 0, 0]  # position vector
    acceleration = [0, 0, 0]

    def __init__(self, _timeslice, _noiselevel):
        self.timeslice = _timeslice
        self.noiselevel = _noiselevel

    def get_x(self):
        return self.loc[0]

    def get_y(self):
        return self.loc[1]

    def get_z(self):
        return self.loc[2]

    def get_x_velocity(self):
        return self.vel[0]

    def get_y_velocity(self):
        return self.vel[1]

    def get_z_velocity(self):
        return self.vel[2]

    def get_x_with_noise(self):
        return random.gauss(self.get_x(), self.noiselevel)

    def get_y_with_noise(self):
        return random.gauss(self.get_y(), self.noiselevel)

    def get_z_with_noise(self):
        return random.gauss(self.get_z(), self.noiselevel)

    def add(self, mat_one, mat_two):
        return mat_one + mat_two

    def mult(self, mat_one, mat_two):
        return mat_one * mat_two

    def step(self):
        timeslicevec = [self.timeslice, self.timeslice, self.timeslice]

        sliced_gravity = list(map(self.mult, self.gravity, timeslicevec))

        sliced_acceleration = sliced_gravity

        self.vel = list(map(self.add, self.vel, sliced_acceleration))

        sliced_velocity = list(map(self.mult, self.vel, timeslicevec))

        self.loc = list(map(self.add, self.loc, sliced_velocity))

        if self.loc[1] < 0:
            self.loc[1] = 0


def example_two():
    # Let's go over the physics behind the cannon shot, just to make sure it's
    # correct:
    # sin(45)*100 = 70.710 and cos(45)*100 = 70.710
    # vf = vo + at
    # 0 = 70.710 + (-9.81)t
    # t = 70.710/9.81 = 7.208 seconds for half
    # 14.416 seconds for full journey
    # distance = 70.710 m/s * 14.416 sec = 1019.36796 m

    timeslice = 0.5  # time between steps
    iterations = 50  # num steps to go for
    noiselevel = 30
    angle = 45  # degrees
    muzzle_velocity = 100  # units/sec
    x = []
    y = []
    z = []
    nx = []
    ny = []
    nz = []
    kx = []
    ky = []
    kz = []
    fro_norm = []

    cannon = Cannon(timeslice, noiselevel)

    # confirm the speed of x and y
    speed_x = muzzle_velocity * math.cos(angle * math.pi / 180)
    speed_y = muzzle_velocity * math.sin(angle * math.pi / 180)
    speed_z = muzzle_velocity * math.cos(angle * math.pi / 180)

    # This is the state transition vector, which represents part of the kinematics.
    # 1, ts, 0,  0  =>  x(n+1) = x(n) + vx(n)
    # 0,  1, 0,  0  => vx(n+1) =        vx(n)
    # 0,  0, 1, ts  =>  y(n+1) =              y(n) + vy(n)
    # 0,  0, 0,  1  => vy(n+1) =                     vy(n)
    # 0,  0, 1, ts  =>  z(n+1) =                           z(n) + vz(n)
    # 0,  0, 0,  1  => vz(n+1) =                                        vz(n)
    # Remember, acceleration gets added to these at the control vector.
    state_transition = numpy.matrix(
        [
            [1, timeslice, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, timeslice, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, timeslice],
            [0, 0, 0, 0, 0, 1],
        ]
    )
    control_matrix = numpy.matrix(
        [
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
        ]
    )

    # The control vector, which adds acceleration to the kinematic equations.
    # 0          =>  x(n+1) =  x(n+1)
    # 0          => vx(n+1) = vx(n+1)
    # -9.81*ts^2 =>  y(n+1) =  y(n+1) + 0.5*-9.81*ts^2
    # -9.81*ts   => vy(n+1) = vy(n+1) + -9.81*ts
    # 0          =>  x(n+1) =  z(n+1)
    # 0          => vx(n+1) = vz(n+1)
    control_vector = numpy.matrix(
        [[0], [0], [0.5 * -9.81 * timeslice * timeslice], [-9.81 * timeslice], [0], [0]]
    )

    # After state transition and control, here are the equations:
    #  x(n+1) = x(n) + vx(n)
    # vx(n+1) = vx(n)
    #  y(n+1) = y(n) + vy(n) - 0.5*9.81*ts^2
    # vy(n+1) = vy(n) + -9.81*ts
    #  z(n+1) = z(n) + vz(n)
    # vz(n+1) = vz(n)
    # Which, if you recall, are the equations of motion for a parabola.  Perfect.

    # Observation matrix is the identity matrix, since we can get direct
    # measurements of all values in our example.
    observation_matrix = numpy.eye(6)
    initial_state = numpy.matrix([[0], [speed_x], [0], [speed_y], [0], [speed_z]])
    initial_probability = numpy.eye(6)

    process_covariance = numpy.zeros(6)
    measurement_covariance = numpy.eye(6) * 0.2

    cannon_filter = KalmanFilterLinear(
        state_transition,
        control_matrix,
        observation_matrix,
        initial_state,
        initial_probability,
        process_covariance,
        measurement_covariance,
    )

    for i in range(iterations):
        x.append(cannon.get_x())
        y.append(cannon.get_y())
        z.append(cannon.get_z())
        newest_x = cannon.get_x_with_noise()
        newest_y = cannon.get_y_with_noise()
        newest_z = cannon.get_z_with_noise()
        nx.append(newest_x)
        ny.append(newest_y)
        nz.append(newest_z)
        cannon.step()
        kx.append(cannon_filter.get_current_state()[0, 0])
        ky.append(cannon_filter.get_current_state()[2, 0])
        kz.append(cannon_filter.get_current_state()[4, 0])
        cannon_filter.step(
            control_vector,
            numpy.matrix(
                [
                    [newest_x],
                    [cannon.get_x_velocity()],
                    [newest_y],
                    [cannon.get_y_velocity()],
                    [newest_z],
                    [cannon.get_z_velocity()],
                ]
            ),
        )

        fro_norm.append(numpy.linalg.norm(cannon_filter.covariance,"fro"))

    # This import registers the 3D projection, but is otherwise unused.
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

    import numpy as np
    import matplotlib.pyplot as plt

    plt.rcParams["legend.fontsize"] = 12

    fig = plt.figure()
    ax = fig.gca(projection="3d")

    ax.set_title("Measurement of a Cannonball in Flight")
    ax.plot(z, x, y, "-")
    ax.plot(nz, nx, ny, ":")
    ax.plot(kz, kx, ky, "--")
    ax.plot(kz, kx,fro_norm,"r")
    ax.legend(("true", "measured", "kalman","covariance"))

    plt.show()


def _js_round(value):
    x = math.floor(value)
    if (value - x) < .50:
        return x
    else:
        return math.ceil(value)

def eccen_anom(ec, m, dp):
    # Eccentric Anomaly

    # arguments:
    # ec - eccentricity
    # m - mean anomaly
    # dp - number of decimal places

    # returns:
    # E - eccentric anomaly
    pi = math.pi
    K = pi / 180.0

    max_iter = 30
    delta = 10 ** -dp

    m = m / 360.0
    m = 2.0 * pi * (m - math.floor(m))

    if ec < 0.8:
        E = m
    else:
        E = pi

    F = E - ec * math.sin(m) - m

    i = 0
    while (math.fabs(F) > delta) and (i < max_iter):
        E = E - F / (1.0 - ec * math.cos(E))
        F = E - ec * math.sin(E) - m
        i = i + 1

    E = E / K
    # S = math.sin(E)
    # C = math.cos(E)
    # fak = math.sqrt(1.0 - ec * ec)
    # phi = math.atan2(fak * S, C - ec) / K

    return _js_round((E * 10 ** dp) / 10 ** dp)


def true_anom(ec, E, dp):
    # arguments:
    # ec - eccentricity
    # m - mean anomaly
    # dp - number of decimal places

    # returns:
    # phi - true anomaly

    K = math.pi / 180.0
    S = math.sin(E)
    C = math.cos(E)
    fak = math.sqrt(1.0 - ec ** 2)
    phi = math.atan2(fak * S, C - ec) / K

    return _js_round((phi * (10 ** dp))/ (10 ** dp))


def position(a, ec, E):
    # arguments:
    # a - semimajor axis
    # ec - eccentricity
    # E - eccentric anomaly

    # returns:
    # x,y - coordinates of the planet with respect to the Sun

    C = math.cos(E)
    S = math.sin(E)
    x = a * (C - ec)
    y = a * math.sqrt(1.0 - ec ** 2) * S

    return (x, y)

def example_three():
    n = 60

def main():
    print("starting kalman filter...")
    # example_two()
    ec = 0.5
    m = 27
    dp = 5
    E = eccen_anom(ec, m, dp)
    phi = true_anom(ec, E, dp)
    a = 1.006
    print(E)
    print(phi)
    print(position(a, ec, E))
    print("done!")


if __name__ == "__main__":
    main()
