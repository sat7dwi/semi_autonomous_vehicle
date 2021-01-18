function [x,P]= EKF_predict(x,P,v,g,Q,dt)
% Inputs:
%   x, P - state and covariance
%   v, g - control inputs: velocity and gamma (steer angle)
%   Q - covariance matrix for velocity and gamma
%   dt - timestep
%
% Outputs: 
%   x, P - predicted state and covariance
 
% <------------------------- TO DO -------------------------->

global a;
global b; 

alpha = atan2(a*tan(g),b);

% jacobians   
J = [ 1     0    -v*dt*sin(alpha+x(3))/cos(alpha) ; ...
      0     1     v*dt*cos(alpha+x(3))/cos(alpha) ; ...
      0     0                1                  ];
  
F = [ dt*cos(alpha+x(3))/cos(alpha)       v*dt*( -a*sin(alpha+x(3))*cos(alpha)/(b*cos(g)^2) + a*cos(alpha+x(3))*sin(alpha)/(b*cos(g)^2) ) ; ... 
      dt*sin(alpha+x(3))/cos(alpha)       v*dt*(  a*cos(alpha+x(3))*cos(alpha)/(b*cos(g)^2) + a*cos(alpha+x(3))*sin(alpha)/(b*cos(g)^2) ) ; ...
              dt*tan(g)/b                                                        v*dt/(b*cos(g)^2)                                        ]; 

% predict state
% x = [ x(1) + v*dt*cos(g+x(3)) ; ... 
%       x(2) + v*dt*sin(g+x(3)) ; ...
%          pi_to_pi(g+x(3))     ];  

x = [ x(1) + v*dt*cos(alpha+x(3))/cos(alpha) ; ...
      x(2) + v*dt*sin(alpha+x(3))/cos(alpha) ; ...
         pi_to_pi(x(3) + v*dt*tan(g)/b)     ];

% predict covariance
P = J*P*J' + F*Q*F';

 
  

 
