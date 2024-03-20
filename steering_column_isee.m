function [ A, B, C, D, K ] = steering_column_isee(Js,Bs,Ks,Jm,Bm,Mr,Br,Kr,Ki,R,N,Rp,L,Tr,Sr,Ts)
%{
  This is the the five-state linear state space model of a COMLUMN-TYPE steering system which comprises of
  the power steering motor, rack, pinion and steering column. I assumed
  that Driver torq is zero (since value isn't measured)

  This functions models aligning torque as a torque applied to the tires
  and converts this torque into a force on the rack using the parameters in
  the model.

  This function also includes the steering ratio as a parameter in the
  model and linearly superimposes the inertial properties of the rack and
  the assist motor inorder to reduce the order of the intial steering
  system from 7 to 5;

  The system of equations and states I used are described in the read me file.

  parameters = {'Steering column inertia',Js;'Steering column viscous damping',Bs;
  'Steering column stiffness',Ks;'Motor inertia',Jm;'Motor viscous damping',Bm;
  'motor shaft stiffness',Km; 'Rack inertia',Mr; 'rack viscous damping',Br; 
  'rack stiffness',Kr;'current constant ',Ki;'terminal resistance ', R;
  'motor gear ratio', Rg;'pinion radius',Rp;'motor inductance',L; 'steering ratio', Sr;};

%}


A = zeros(5,5);

A(1,2) = 1; A(2,1) = -Ks/Js; A(2,2) = -Bs/Js; A(2,3) = Ks/(Js*N*Sr); A(3,4) = 1;
A(4,1) = Ks*Sr/(N*(Jm +(Rp*Rp*Mr/(N*N)))); A(4,3) = -(Ks/(N*N) + Kr*Rp*Rp/(N*N))/(Jm+(Rp*Rp*Mr/(N*N))); 
A(4,4) = -(Bm +(Rp*Rp*Br/(N*N)))/(Jm +(Rp*Rp*Mr/(N*N))); A(4,5) = Ki/(Jm +(Rp*Rp*Mr/(N*N)));
A(5,4) = -Ki/L; A(5,5) = -R/L;

B = zeros(5,2);

B(5,1) = -1/L;

B(4,2) = Rp/(N*(Jm +(Rp*Rp*Mr/(N*N))))*Tr;

C = zeros(1,5);

C(1,1) = 1;

D = zeros(1,2);

K = zeros(5,1);
end

