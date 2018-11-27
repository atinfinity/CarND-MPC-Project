# CarND-Controls-MPC

## Project Submission
1. Clone/fork the project's template files from the [project repository](https://github.com/udacity/CarND-MPC-Project). (Note: Please do not submit your project as a pull request against our repo!)
1. Choose your state, input(s), dynamics, constraints and implement MPC.
1. Test your solution on basic examples.
1. Test your solution on the simulator!
1. When the vehicle is able to drive successfully around the track, submit! ***Remember to include a separate file in .txt, .md, .html, or .pdf format addressing the questions in the [rubric](https://review.udacity.com/#!/rubrics/896/view).***
Try to see how fast you get the vehicle to **SAFELY** go!

---

## [Rubric Points](https://review.udacity.com/#!/rubrics/896/view)
### Compilation
* Your code should compile.

### Implementation
#### The Model
* Student describes their model in detail. This includes the state, actuators and update equations.

#### Timestep Length and Elapsed Duration (N & dt)
* Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

#### Polynomial Fitting and MPC Preprocessing
* A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

#### Model Predictive Control with Latency
* The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

### Simulation
* No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle). The car can't go over the curb, but, driving on the lines before the curb is ok.

---

## Writeup
### Implementation
#### The Model
The model equations are as follow.

```
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi[t+1] = psi[t] + (v[t] / Lf) * delta[t] * dt
v[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - atan(f(x')) + (v[t] / Lf) * delta[t] * dt
```

Where:

- `(x,y)` : 2D position of vehicle
- `psi` : heading direction of vehicle
- `v` : velocity of vehicle
- `cte` : Cross Track Error
- `epsi` : Orientation error

And, this model outputs the following state.

- `a` : acceleration of vehicle
- `delta` : steering angle

#### Timestep Length and Elapsed Duration (N & dt)
In this project, the latency is set to 100ms.  
So, I set `dt` as `0.1`(=100ms). After that, I tuned `N` manually.

#### Polynomial Fitting and MPC Preprocessing
I preprocessed those waypoints to the vehicle coordinate system.  
As a result, the calculation for polynomial fitting is easy.  
My implementation is as follow.

```c++
for (size_t i = 0; i < ptsx.size(); i++) {
  // rotate the difference between the waypoints and the car position by -psi.
  double diff_x = ptsx[i]-px;
  double diff_y = ptsy[i]-py;
  ptsx[i] = (diff_x * cos(-psi)-diff_y * sin(-psi));
  ptsy[i] = (diff_x * sin(-psi)+diff_y * cos(-psi));
}
```

And, a polynomial is fitted to preprocessed waypoints.  
My implementation is as follow.

```c++
Eigen::Map<Eigen::VectorXd> ptsx_trans(ptsx.data(), ptsx.size());
Eigen::Map<Eigen::VectorXd> ptsy_trans(ptsy.data(),ptsy.size());
auto coeffs = polyfit(ptsx_trans, ptsy_trans, 3);
```

#### Model Predictive Control with Latency
In this system has 100ms latency.  
Therefore, the state values are calculated using the delay interval to handle this latency.  
My implementation is as follow.

```c++
double delay_x = v * delay_t;
double delay_y = 0;
double delay_psi = -v * steer_value / Lf * delay_t;
double delay_v = v + throttle_value * delay_t;
double delay_cte = cte + v * sin(epsi) * delay_t;
double delay_epsi = epsi-v * steer_value /Lf * delay_t;

Eigen::VectorXd state(6);
state << delay_x, delay_y, delay_psi, delay_v, delay_cte, delay_epsi;
auto vars = mpc.Solve(state, coeffs);
```
