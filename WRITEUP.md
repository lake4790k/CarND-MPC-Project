
### Model

The state is defined as:

```
double x = state[0]; // position x
double y = state[1]; // position y
double psi = state[2]; // orientation
double v = state[3]; // speed
double cte = state[4]; // cross track error
double epsi = state[5]; // orientation error
```

The actuators are the steering angle and throttle level:

```
auto vars = mpc.Solve(state, coeffs);
steer_value = vars[0];
throttle_value = vars[1];
```

The updates:

```
x1 = x0 + v0*cos(psi0)*dt
y1 = y0 + v0*sin(psi0)*dt
psi1 = psi0 + v0/Lf * delta* dt
v1 = v0 + a0*dt
cte1 = f0 - y0 + v0 * sin(eps0)*dt
eps1 = psi0 - psides + v0 / Lf * delta * dt
```


### N and dt

I reduced N to 10 and increased dt accordingly to 0.1 as per Udacity instructions. This results in less iterations making the MPC exectute faster.

### Waypoint preprocessing

Waypoints are transformed to the cars coordinates, so x and y and psi all start at 0.


### Latency

The control latency is handled by using the kinematic model to predict the state to the latency period and using that state in the MPC model.
