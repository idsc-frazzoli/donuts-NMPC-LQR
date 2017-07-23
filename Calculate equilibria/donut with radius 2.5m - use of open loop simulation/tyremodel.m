%%%%% 
% 
%  Written by Panagiotis Bountouris
%  Semester Project: Go-kart modeling and MPC for donuts drifting maneuvers
%
%%%%%

%%
function [ fFy, fRy] = tyremodel( Ux, beta, r, delta , fRx )


%%  Define vehicle's parameters

m=850;            % mass of kart (kg)
lF=1.5;             % distance between front axle and center of mass (m)
lR=0.9;             % distance between rear axle and center of mass (m)
rF=0.311;         % diameter of front wheel (m)
rR=0.311;         % diameter of rear wheel (m)
R=rF;               % diameter of wheels in the bicycle model equations (m)
Iz=1400;           % moment of inertia of kart (kg*m^2)
Iw=0.6;             % moment of inertia of rear wheel (kg*m^2)
Cd=50;             % electronic differential coefficient  (N*m/(rad/s)^(1/2))
B=4;                 % B coefficient of Pacejka's formula ( )
C=1.3;              % C coefficient of Pacejka's formula ( )
D=0.6;              % D coefficient of Pacejka's formula ( )
g=9.81;             % acceleration of gravity (m/sec^2) 
h=0.5;               % height of center of mass (m)


mfriction= 0.7;   % friction coefficient between rubber and asphault in dry case 

mfriction_front = 0.70 ;
mfriction_rear = 0.67 ;



%%

% find vertical forces fFz and fRz
fFz = [ m * g * lR  ] / ( lF + lR )  ;
fRz = [ m * g * lF ] / ( lF + lR )   ;


% find slip angles aF aR
aF = atan( beta +  (lF * r ) / Ux ) - delta ;
aR = atan( beta - ( lR * r ) / Ux );



% calculate rear lateral force fRy
fRymax = sqrt( ( mfriction_rear * fRz )^2 - ( fRx )^2 )  ;

fRypaj = - fRz * D * sin( C * atan( B * aR ) ) ; 

fRy = min( fRymax , fRypaj ) ;


% calculate front lateral forces fFy 
fFypaj = -  fFz* D * sin( C * atan( B * aF ) ) ; 

fFy = min ( mfriction_front * fFz , fFypaj ) ;

end


