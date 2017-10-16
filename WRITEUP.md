
### Model

The state is defined as:

`double x = state[0]; // position x
double y = state[1]; // position y
double psi = state[2]; // orientation
double v = state[3]; // speed
double cte = state[4]; // cross track error
double epsi = state[5]; // orientation error`

The actuators are the steering angle and throttle level:

`
auto vars = mpc.Solve(state, coeffs);
steer_value = vars[0];
throttle_value = vars[1];
`

The updates:

`
x1 = x0 + v0*cos(psi0)*dt
y1 = y0 + v0*sin(psi0)*dt
psi1 = psi0 + v0/Lf * delta* dt
v1 = v0 + a0*dt
cte1 = f0 - y0 + v0 * sin(eps0)*dt
eps1 = psi0 - psides + v0 / Lf * delta * dt
`


### N and dt

I reduced N to 10 and increased dt accordingly to 0.1 as per Udacity instructions. This results in less iterations making the MPC exectute faster.

Experimented also with having a longer prediction horizon (increasing N or dt, eg. N=10 dt=0.3 or N=30 dt=0.1) which resulted in rather unstable behaviour as distant points became completely inaccurate. (at least with the cost function I have)

Having a shorter prediction kind of worked (eg. decreasing dt=-.05), but was insatisfactory as did not take into account the more distant part of longer turns only tried to folloq short term which again was rather unstable driving behaviour.

### Waypoint preprocessing

Waypoints are transformed to the cars coordinates, so x and y and psi all start at 0.


### Latency

The control latency is handled y using the kinematic model to predict the state to the latency period and using that state in the MPC model.

This was done in main.cpp:122 using the 0.1 sec latency and the last steering and turning actuator values.