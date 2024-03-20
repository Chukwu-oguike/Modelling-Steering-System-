function [ dx, y ] = steering_col(t,x,u,Js,Bs,Ks,Jm,Bm,Km,Mr,Br,Kr,Ki,R,Rg,Rp,L,varargin)
%{
  This is the the seven-state linear state space model (although I used non-linear grey-box identification)
 of the steering system which comprises of
  the power steering motor, rack, pinion and steering column. I assumed
  that Driver torq is zero (since value isn't measured)
  
  The aligning, 1525.416*5*pi*x(1), torque was also included as a force in
  the rack assuming the vehicle operates in the linear zone of the tires

  Note: In this function the state 'x(1)' is used to estimate the slip
  angle (the aligning torque is a function of slip angle). In state space
  model x(1) represents the steering angle

  parameters = {'Steering column inertia',Js;'Steering column viscous damping',Bs;
  'Steering column stiffness',Ks;'Motor inertia',Jm;'Motor viscous damping',Bm;
  'motor shaft stiffness',Km; 'Rack inertia',Mr; 'rack viscous damping',Br; 
  'rack stiffness',Kr;'current constant ',Ki;'terminal resistance ', R;
  'motor gear ratio', Rg;'pinion radius',Rp;'motor inductance',L};
%}

y = -1*x(1)/2079.0047;  %steering anglewas scaled to small (probably because the steering 
                         %ratio is not incorporated in the model

dx = zeros(7,1);

dx(1,1) = x(2); 
dx(2,1) = -Ks/Js*x(1)-Bs/Js*x(2)+Ks/(Js*Rp)*x(5); 
dx(3,1) = x(4); 
dx(4,1) = -Km/Jm*x(3)-Bm/Jm*x(4)+Km*Rg/(Jm*Rp)*x(5)+Ki/Jm*x(7); 
dx(5,1) = x(6);
dx(6,1) = Ks/(Mr*Rp)*x(1)+Km*Rg/(Mr*Rp)*x(3)-(Kr/Mr+Ks/(Mr*Rp*Rp)+Km*Rg*Rg/(Mr*Rp*Rp))*x(5)...
   -Br/Mr*x(6)-1525.416*5*pi*x(1); 
dx(7,1) = -Ki/L*x(4)-R/L*x(7)+u(1)/L;


end
