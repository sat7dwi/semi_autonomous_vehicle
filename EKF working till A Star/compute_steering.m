function [G,V,iwp]= compute_steering(x, wp, iwp, V, minD, G, rateG, maxG, dt)
 
% % INPUTS:
% %   x - true position
% %   wp - waypoints
% %   iwp - index to current waypoint
% %   minD - minimum distance to current waypoint before switching to next
% %   G - current steering angle
% %   rateG - max steering rate (rad/s)
% %   maxG - max steering angle (rad)
% %   dt - timestep(sec)
% %
% % OUTPUTS:
% %   G - new current steering angle
% %   iwp - new current waypoint
% 
% 
% 
% % determine if current waypoint reached
global a;
global b; 
global Xt;
global Xi;

Xt= wp(:,iwp);
d1= (Xi(1)- x(1))^2 + (Xi(2) - x(2))^2; % Distance between robot and waypoint
d2= (Xt(1)- x(1))^2 + (Xt(2) - x(2))^2; % Distance between robot and waypoint

if x(1) >= Xt(1)
    iwp= iwp+1; % switch to next
    if iwp > size(wp,2) % reached final waypoint, flag and return
        iwp=0;
        return;
    end 
    Xi= Xt;
    Xt= wp(:,iwp); % next waypoint
end
% 
% % compute change in G to point towards current waypoint
%  deltaG= pi_to_pi(atan2(cwp(2)-x(2), cwp(1)-x(1)) - x(3));
%  
% % limit rate
% maxDelta= rateG*dt;
% if abs(deltaG) > maxDelta
%     deltaG= sign(deltaG)*maxDelta;
% end

% persistent flag;
persistent prevErr;
persistent errSum;


if(isempty(prevErr))
    prevErr = 0;
    errSum = 0;
end

d = min(d1, d2);

% if isempty(flag)
%     flag = 0;
% end
% 
% if(x(1) >= 100)
%     flag = 1;
% end
% if flag == 1
%     Xi = [100, 100];
%     Xt = [200, 50];
% end
% 
% if(x(1) >= 200)
%     Xt = [0, 0];
% end
 

O = atan2((Xt(2)-Xi(2)),(Xt(1)-Xi(1)));

T = [  cos(O)   -sin(O)     0     Xi(1) ;  ...
       sin(O)    cos(O)     0     Xi(2) ;  ...
         0         0        1       O   ;  ...
         0         0        0       1   ];

Y = T\[x; 1]; 
Y = Y(2:3);

V2 = d/50;
if (V2-V) > 2
    V = V + 2;
elseif (V2-V) < -2
    V = V - 2;
else
    V = V2;
end
if V > 5
    V = 5;
elseif V < 0.5
    V = 0.5;
end


A = [ 0     V ; ...
      0     0 ];

B = [ a*V/b ;
       V/b  ]; 

C = [ 1     0 ];

Kp = 18;
Kd = 0.53;
Ki = 0.02;


% Kp = 56;
% Kd = 40;
% Ki = 1.5;

% r = 15;
% V = 15;
% G = 3.08/r;
err = -C*Y;
errSum = errSum + err*dt;
G = Kp*err + Ki*errSum + Kd*(err - prevErr)/dt;
prevErr = err;

if(G < -maxG)
    G = -maxG;
elseif(G > maxG)
    G = maxG;
end

end


