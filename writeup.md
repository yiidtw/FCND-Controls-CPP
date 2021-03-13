## Project: Building a Controller

---

# Required Scenarios for a Passing Submission:
1. [PASS] Intro
2. [PASS] AttitudeControl
3. [PASS] PositionControl
4. [PASS] Nonidealities
5. [PASS] TrajectoryFollow
6. [PASS] TestManyQuads
7. [PASS] TestMavlnk
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1643/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---

### Writeup

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.

You are reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

The code snippets for passing each scenario are seperated into different commits. Please use "git log --oneline" to check commit history.

### Implemented Controller

#### 1. Implemented body rate control in C++. (Scenario 2)

- Related parameter
```
[QuadControlParams]
kpPQR = 65, 65, 5
```

- C++ implementation
```
V3F I;
I.x = Ixx;
I.y = Iyy;
I.z = Izz;

momentCmd = kpPQR * I * (pqrCmd - pqr);
```

- Python reference implementation
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


#### 2. Implement roll pitch control in C++. (Scenario 2)

- Related parameter
```
[QuadControlParams]
kpBank = 12
```

- C++ implementation
```
if (collThrustCmd > 0) {

    float accel = -collThrustCmd / mass;

    float b_x_c = CONSTRAIN(accelCmd.x / accel, -maxTiltAngle, maxTiltAngle);
    float b_x = R(0, 2);
    float b_x_err = b_x_c - b_x;
    float b_x_p_term = kpBank * b_x_err;

    float b_y_c = CONSTRAIN(accelCmd.y / accel, -maxTiltAngle, maxTiltAngle);
    float b_y = R(1, 2);
    float b_y_err = b_y_c - b_y;
    float b_y_p_term = kpBank * b_y_err;

    pqrCmd.x = (R(1,0) * b_x_p_term - R(0,0) * b_y_p_term) / R(2,2);
    pqrCmd.y = (R(1,1) * b_x_p_term - R(0,1) * b_y_p_term) / R(2,2);

} else {
    pqrCmd.x = 0;
    pqrCmd.y = 0;
}

pqrCmd.z = 0;
```

- Python reference implementation
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

#### 3. Implement altitude controller in C++. (Scenario 2, 3, 4)

- Related parameter
```
[QuadControlParams]
kpPosZ = 31
kpVelZ = 10
KiPosZ = 40
```

- C++ implementation
```
integratedAltitudeError += ((posZCmd - posZ) * dt);
float i_term = KiPosZ * integratedAltitudeError;

float p_term = kpPosZ * (posZCmd - posZ);
float d_term = velZ + kpVelZ * (velZCmd - velZ);

float u_bar = accelZCmd + p_term + i_term + d_term;

float b_z = R(2,2);
float capAccelZ = CONSTRAIN((u_bar - CONST_GRAVITY) / b_z, - maxAscentRate / dt, maxAscentRate / dt);

thrust = - mass * capAccelZ;
```

- Python reference implementation
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

#### 4. Implement lateral position control in C++. (Scenario 3)

- Related parameter
```
[QuadControlParams]
kpPosXY = 33
kpVelXY = 12
```

- C++ implementation
```
V3F capVelCmd = velCmd.mag() > maxSpeedXY ? velCmd.norm() * maxSpeedXY : velCmd;
V3F uncapAccelCmd = accelCmdFF + kpPosXY * (posCmd - pos) + kpVelXY * (capVelCmd - vel);

accelCmd = uncapAccelCmd.mag() > maxAccelXY ? uncapAccelCmd.norm() * maxAccelXY : uncapAccelCmd;
```

- Python reference implementation
```
def lateral_controller(self,
  x_target,
  x_dot_target,
  x_dot_dot_target,
  x_actual,
  x_dot_actual,
  y_target,
  y_dot_target,
  y_dot_dot_target,
  y_actual,
  y_dot_actual,
  c):

x_err = x_target - x_actual
x_err_dot = x_dot_target - x_dot_actual

p_term_x = self.x_k_p * x_err
d_term_x = self.x_k_d * x_err_dot

x_dot_dot_command = p_term_x + d_term_x + x_dot_dot_target

b_x_c = x_dot_dot_command/c


y_err = y_target - y_actual
y_err_dot = y_dot_target - y_dot_actual

p_term_y = self.y_k_p * y_err
d_term_y = self.y_k_d * y_err_dot

y_dot_dot_command = p_term_y + d_term_y + y_dot_dot_target

b_y_c = y_dot_dot_command/c

return b_x_c, b_y_c
```

#### 5. Implement yaw control in C++. (Scenario 3)

- Related parameter
```
[QuadControlParams]
kpYaw = 2
```

- C++ implementation
```
float psi_err = fmodf((yawCmd - yaw), 2.f * F_PI);
yawRateCmd = kpYaw * psi_err;
```

- Python reference implementation
```
def yaw_controller(self,
                   psi_target,
                   psi_actual):

    psi_err = psi_target - psi_actual
    r_c = self.k_p_yaw * psi_err

    return r_c
```

#### 6. Implement calculating the motor commands given commanded thrust and moments in C++.  (Scenario 2)

- Related parameter
```
[QuadControlParams]
kappa = 0.016
```

- C++ implementation
```
float f_t = collThrustCmd;

float l = L / sqrt(2.f);
float f_x = momentCmd.x / l;
float f_y = momentCmd.y / l;
float f_z = -momentCmd.z / kappa;

float f_0, f_1, f_2, f_3;
f_0 = (f_t + f_x + f_y + f_z) / (4.f);
f_1 = (f_t - f_x + f_y - f_z) / (4.f);
f_2 = (f_t + f_x - f_y - f_z) / (4.f);
f_3 = (f_t - f_x - f_y + f_z) / (4.f);

cmd.desiredThrustsN[0] = f_0;
cmd.desiredThrustsN[1] = f_1;
cmd.desiredThrustsN[2] = f_2;
cmd.desiredThrustsN[3] = f_3;
```

- Python reference implementation
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

### Flight Evaluation

#### Your C++ controller is successfully able to fly the provided test trajectory and visually passes inspection of the scenarios leading up to the test trajectory.

![](https://i.imgur.com/JbXSH3e.png)

