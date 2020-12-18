from vehicles.point_car import PointCar
import numpy as np
import openmdao.api as om
import dymos as dm
from path_finding import find_valid_trajectory


class Vehicle(om.ExplicitComponent):
    """
    https://openmdao.github.io/dymos/examples/brachistochrone/brachistochrone.html
    """

    def initialize(self):
        # super().initialize()
        self.options.declare('num_nodes', types=int)

    def setup(self):
        nn = self.options['num_nodes']
        self.add_input('theta', val=np.ones(nn), desc='angle of wire', units='rad')
        self.add_input('x', val=np.ones(nn), desc='x coordinate', units='rad')
        self.add_input('y', val=np.ones(nn), desc='y coordinate', units='rad')
        self.add_input('v', val=np.ones(nn), desc='velocity', units='m/s')


        self.add_output('xdot', val=np.zeros(nn), desc='velocity component in x', units='m/s',
                        tags=['state_rate_source:x', 'state_units:m'])
        self.add_output('ydot', val=np.zeros(nn), desc='velocity component in y', units='m/s',
                        tags=['state_rate_source:y', 'state_units:m'])
        self.add_output('vdot', val=np.zeros(nn), desc='acceleration magnitude', units='m/s**2',
                        tags=['state_rate_source:v', 'state_units:m/s'])


        arange = np.arange(self.options['num_nodes'])
        self.declare_partials(of='xdot', wrt='v', rows=arange, cols=arange)
        self.declare_partials(of='xdot', wrt='theta', rows=arange, cols=arange)

        self.declare_partials(of='ydot', wrt='v', rows=arange, cols=arange)
        self.declare_partials(of='ydot', wrt='theta', rows=arange, cols=arange)

        self.declare_partials(of='vdot', wrt='theta', rows=arange, cols=arange)
        pass

    def compute(self, inputs, outputs):
        dx_min = 6.0755 # m
        a_max = 7.0 # m/s^2

        # compute dynamics
        theta = inputs['theta']
        x1, y1 = inputs['x'], inputs['y']
        v = inputs['v']

        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        dx = dx_min * sin_theta
        dy = -dx_min * cos_theta

        dist = np.sqrt(dx**2 + dy**2)
        
        steering_correction = 1.0 - (d_theta / (self.max_turning_rate * dist))
        v_gain = self.v + (dist / max(self.v, self.max_a)) * self.max_a * steering_correction
        v_adj = (steering_correction * self.v + min(v_gain, steering_correction * self.max_v)) / 2.0
        dt = car.get_time(
            (
                x1 + dx_min * sin_theta, 
                y1 - dx_min * cos_theta
            ))
        
        outputs['xdot'] = dx
        outputs['ydot'] = dy
        outputs['vdot'] = np.sqrt(dx**2 + dy**2) / dt - v

    def compute_partials(self, inputs, partials):
        theta = inputs['theta']

        partials['check', 'theta'] = 
        pass


if __name__ == "__main__":
    IMG_PATH = "./tracks/loop.png"
    track = load_track(IMG_PATH)
    # Set to a valid point in trajectory
    car = PointCar(150, 200)
    trellis = find_valid_trajectory(car, track, states=11)

    inner_x = []
    inner_y = []

    alpha = []
    xr = [] # holds inner contour point
    xl = [] # holds coor along outter contour
    for site in trellis:
        inner_x.append(site[0][0])
        inner_y.append(site[0][1])
        width = distance(site[0], site[-1])
        xl.append(site[-1])
        init_coor = site[len(site)//2]
        alpha.append(distance(site[0], init_coor) / width)
        xr.append(init_coor)

    print(xl[:5])
    print(alpha[:5])
    print(xr[:5])


    p_base = om.Problem(model=om.Group())