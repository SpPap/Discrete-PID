# Discrete PID Controller
Here is an implementation of a discrete PID in MATLAB

 Continuous time:
 $$u(t) = K_pe(t)+K_i\int_{0}^{t} e(\tau)d\tau+K_d\frac{de(t)}{dt}$$  



For the integral term, 2 methods are being used for numerical
integration (Trapezoidal rule and Composite Simpson's 1/3 rule). We have 
an advanced integral term handling based on whether k is even or odd.

The derivative term is implemented using backward Euler.
