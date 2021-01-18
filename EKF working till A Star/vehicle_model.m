function xv= vehicle_model(xv, V,G,dt)
% Used to calculate the true pose of the robot.
% INPUTS:
%   xv - vehicle pose [x;y;phi]
%   V - velocity
%   G - steer angle (rad)
%   WB - wheelbase
%   dt - change in time
%
% OUTPUTS:
%   xv - new vehicle pose
% 
% a = distance of back wheel from centre
% b = distance between wheels
global a;
global b;

alpha = atan2(a*tan(G),b);
xv = [ xv(1) + V*dt*cos(alpha+xv(3))/cos(alpha) ; ...
       xv(2) + V*dt*sin(alpha+xv(3))/cos(alpha) ; ...
          pi_to_pi(xv(3) + V*dt*tan(G)/b)   ];
 
 