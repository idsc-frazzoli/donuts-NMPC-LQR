%%%%% 
% 
%  Written by Panagiotis Bountouris
%  Semester Project: Go-kart modeling and MPC for donuts drifting maneuvers
%
%%%%%

%%

clear all
 close all 
 clc
 

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
h=0.5;  

%% Define how dense we want the phase plots

density = 25 ; 

%% Find values of the states Ux , beta , r to plot the phase plots ( or regions of attractions ) [ delta =  0 deg ]

fRx = 1812 ;                           % fRx in steady state 
delta = deg2rad( + 0 ) ;          % delta in steady state 
Ux= 8.17 ;

% initially we create a grid of the 2 states as dense as we want 
[  beta , r ] = meshgrid(  linspace( -1.5 , 1.5 , density )  ,  linspace(  -1.5 , 1.5 , density )   ) ;  

% then solve the nonlinear model with these values of the states
[ fFy, fRy] = tyremodel( Ux, beta, r, delta , fRx ); 

Uxdot = 1/m * [ - fFy .* sin(delta) + fRx ] + r .* beta .* Ux ; 
betadot = 1/( Ux * m ) .* [  fFy + fRy ] - r ; 
rdot = 1/Iz * [ lF * fFy * cos(delta) - lR * fRy] ;


% 2D phase plot ( beta - r ) in degrees 

figure(1) ; 

quiver( rad2deg(beta), rad2deg(r) , rad2deg(betadot), rad2deg(rdot) )
hold on
scatter(-25 , 39 , 'r' , 'filled' )         
scatter(+25 , -39 , 'r' )  
scatter( rad2deg( 0 ) , rad2deg( 0 ) , 'r' )
title(' Phase plot (beta - r) [delta = 0 (deg)] '); xlabel(' beta (deg) ','FontSize', 16); ylabel(' r (deg/s) ','FontSize', 16);


%% Find values of the states Ux , beta , r to plot the phase plots ( or regions of attractions ) [ delta =  + 7.5 deg ]

fRx = 1812 ;                           % fRx in steady state 
delta = deg2rad( + 7.5 ) ;          % delta in steady state 
Ux= 8.17 ;

% initially we create a grid of the 2 states as dense as we want 
[  beta , r ] = meshgrid(  linspace( -1.5 , 1.5 , density )  ,  linspace(  -1.5 , 1.5 , density )   ) ;  

% then solve the nonlinear model with these values of the states
[ fFy, fRy] = tyremodel( Ux, beta, r, delta , fRx ); 

Uxdot = 1/m * [ - fFy .* sin(delta) + fRx ] + r .* beta .* Ux ; 
betadot = 1/( Ux * m ) .* [  fFy + fRy ] - r ; 
rdot = 1/Iz * [ lF * fFy * cos(delta) - lR * fRy] ;

% 2D phase plot ( beta - r ) in degrees 

figure(2) ; 

quiver( rad2deg(beta), rad2deg(r) , rad2deg(betadot), rad2deg(rdot) )
hold on
scatter(-25 , 39 , 'r', 'filled' )         
scatter(-4 , 25 , 'r' )  
scatter( +40, -40 , 'r' )
title(' Phase plot (beta - r) [delta = +7.5 (deg)] '); xlabel(' beta (deg) ','FontSize', 16); ylabel(' r (deg/s) ','FontSize', 16);




%% Find values of the states Ux , beta , r to plot the phase plots ( or regions of attractions ) [ delta =  + 15 deg ]

fRx = 1812 ;                           % fRx in steady state 
delta = deg2rad( + 15 ) ;          % delta in steady state 
Ux= 8.17 ;

% initially we create a grid of the 2 states as dense as we want 
[  beta , r ] = meshgrid(  linspace( -1.5 , 1.5 , density )  ,  linspace(  -1.5 , 1.5 , density )   ) ;  

% then solve the nonlinear model with these values of the states
[ fFy, fRy] = tyremodel( Ux, beta, r, delta , fRx ); 

Uxdot = 1/m * [ - fFy .* sin(delta) + fRx ] + r .* beta .* Ux ; 
betadot = 1/( Ux * m ) .* [  fFy + fRy ] - r ; 
rdot = 1/Iz * [ lF * fFy * cos(delta) - lR * fRy] ;

% 2D phase plot ( beta - r ) in degrees 

figure(3) ; 

quiver( rad2deg(beta), rad2deg(r) , rad2deg(betadot), rad2deg(rdot) )
hold on
scatter( rad2deg( -0.28 ) , rad2deg( 0.68 ) , 'r' ,'filled' )         % steady state Ux, beta , r
scatter( +45, -40 , 'r' )
title(' Phase plot (beta - r) [delta = +15 (deg)] ');  xlabel(' beta (deg) ','FontSize', 16); ylabel(' r (deg/s) ','FontSize', 16);


