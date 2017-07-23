%%%%% 
% 
%  Written by Panagiotis Bountouris
%  Semester Project: Go-kart modeling and MPC for donuts drifting maneuvers
%
%%%%%

%%



function xdot = continuous_dynamics ( x, u )  


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

%% 

% assign inputs to variables
delta = u( 1 );
fRx= u( 2 );
Ux = x( 1 );
beta = x( 2 );
r = x( 3 );
psi = x( 4 );
xi= x(5);
yi= x( 6 );


% solve tyre modeling
[ fFy, fRy] = tyremodel( Ux, beta, r, delta , fRx ); 


% solve dynamic modeling
xdot = [ 1/m * [ - fFy * sin(delta) + fRx ] + r * beta* Ux ;
            1/( Ux * m ) * [  fFy + fRy ] - r ;
            1/Iz * [ lF * fFy * cos(delta) - lR * fRy] ; 
             r ;
            Ux * ( cos( psi ) - tan( beta ) * sin( psi ) ) ;
            Ux * ( sin( psi ) + tan( beta ) * cos( psi ) ) ];
                
        

end
