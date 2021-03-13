## Project: Building a Controller

---

# Required Scenarios for a Passing Submission:
1. [PASS] Intro
2. AttitudeControl
3. PositionControl
4. Nonidealities
5. TrajectoryFollow
6. TestManyQuads
7. TestMavlnk
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1643/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---

### Writeup

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.

You are reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Implemented Controller

#### 1. Implemented body rate control in C++. (Scenario 2)

- Python reference implementation:

```
def body_rate_controller(self,
                        p_c,
                        q_c,
                        r_c,
                        p_actual,
                        q_actual,
                        r_actual):

   p_err= p_c - p_actual
   u_bar_p = self.k_p_p * p_err

   q_err= q_c - q_actual
   u_bar_q = self.k_p_q * q_err

   r_err= r_c - r_actual
   u_bar_r = self.k_p_r * r_err

   return u_bar_p, u_bar_q, u_bar_r
```

- related parameter
```

[QuadControlParams]
kpPQR = 65, 65, 5

```

#### 2. Implement roll pitch control in C++. (Scenario 2)

- Python reference implementation:
```
def roll_pitch_controller(self,
                          b_x_c,
                          b_y_c,
                          rot_mat):
    b_x = rot_mat[0,2]
    b_x_err = b_x_c - b_x
    b_x_p_term = self.k_p_roll * b_x_err

    b_y = rot_mat[1,2]
    b_y_err = b_y_c - b_y
    b_y_p_term = self.k_p_pitch * b_y_err

    b_x_commanded_dot = b_x_p_term
    b_y_commanded_dot = b_y_p_term

    rot_mat1=np.array([[rot_mat[1,0],-rot_mat[0,0]],[rot_mat[1,1],-rot_mat[0,1]]])/rot_mat[2,2]

    rot_rate = np.matmul(rot_mat1,np.array([b_x_commanded_dot,b_y_commanded_dot]).T)
    p_c = rot_rate[0]
    q_c = rot_rate[1]

    return p_c, q_c
```

- related parameter
```

[QuadControlParams]
kpBank = 12

```

#### 3. Implement altitude controller in C++. (Scenario 2)

- Python reference implementation:
```
def attitude_controller(self,
                       b_x_c_target,
                       b_y_c_target,
                       psi_target,
                       psi_actual,
                       p_actual,
                       q_actual,
                       r_actual,
                       rot_mat):

    p_c, q_c = self.roll_pitch_controller(b_x_c_target,
                                          b_y_c_target,
                                          rot_mat)

    r_c = self.yaw_controller(psi_target,
                              psi_actual)

    u_bar_p, u_bar_q, u_bar_r = self.body_rate_controller(p_c,
                                                          q_c,
                                                          r_c,
                                                          p_actual,
                                                          q_actual,
                                                          r_actual)

    return u_bar_p, u_bar_q, u_bar_r
```

- related parameter
```

[QuadControlParams]
kpPosZ = 1
kpVelZ = 4

```
#### 4. Implement lateral position control in C++.

#### 5. Implement yaw control in C++.

#### 6. Implement calculating the motor commands given commanded thrust and moments in C++.  (Scenario 2)

- Python reference implementation:

```
def f_1(self):
    f = self.k_f*self.omega[0]**2
    return f

def f_2(self):
    f = self.k_f*self.omega[1]**2
    return f

def f_3(self):
    f = self.k_f*self.omega[2]**2
    return f

def f_4(self):
    f = self.k_f*self.omega[3]**2
    return f

# collective force
def f_total(self):
    f_t = self.f_1 + self.f_2 + self.f_3 + self.f_4
    return f_t

def set_propeller_angular_velocities(self,
                                    c,
                                    u_bar_p,
                                    u_bar_q,
                                    u_bar_r):

    c_bar = -c * self.m / self.k_f
    p_bar = u_bar_p * self.i_x / (self.k_f * self.l)
    q_bar = u_bar_q * self.i_y / (self.k_f * self.l)
    r_bar = u_bar_r * self.i_z / self.k_m

    omega_4 = (c_bar + p_bar - r_bar - q_bar)/4
    omega_3 = (r_bar - p_bar)/2 + omega_4
    omega_2 = (c_bar - p_bar)/2 - omega_3
    omega_1 = c_bar - omega_2 - omega_3 - omega_4

    self.omega[0] = -np.sqrt(omega_1)
    self.omega[1] = np.sqrt(omega_2)
    self.omega[2] = -np.sqrt(omega_3)
    self.omega[3] = np.sqrt(omega_4)
```

- related parameter
```

[QuadControlParams]
kappa = 0.016

```

### Flight Evaluation

#### Your C++ controller is successfully able to fly the provided test trajectory and visually passes inspection of the scenarios leading up to the test trajectory.

