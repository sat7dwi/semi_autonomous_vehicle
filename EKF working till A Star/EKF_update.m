function [x,P]= EKF_update(x,P,z,R,idf,lm)
% correcting the predicted pose using Kalman gain.

% Inputs:
%   x, P -  state and covariance
%   z, R - range-bearing measurements and covariances
%   idf - indecies of each landmark from which measurements have arrived 
 
% Outputs:
%   x, P - updated state and covariance


% <---------------------------- TO DO -------------------------->

%      Compute expected measurement and Jacobian of measurement model representeda as H       
z_hat = x;      
H = eye(3,3);
    
%      Compute Kalman Gain
 K = P*H'/(H*P*H'+R);

 % Update pose x 
 delz = z-z_hat;
 delz(3) = pi_to_pi(delz(3));
 x = x + K*(delz);
 x(3) = pi_to_pi(x(3));     

 % Update covariance P
 P = (eye(3) - K*H)*P;
   


     
    
    
  

         