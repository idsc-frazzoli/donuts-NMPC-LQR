%%%%% 
% 
%  Written by Panagiotis Bountouris
%  Semester Project: Go-kart modeling and MPC for donuts drifting maneuvers
%
%%%%%

%%


clear 
close all
clc

%% Define the range of radius R and the vehicle's parameters


R=1:0.5:20;

l=2.4*ones( 1 , length(R) );
w=0.74*2*ones( 1 , length(R) );


%% Calculate delta 1, delta 2 , delta 

for i=1:length(R)
    
d1(i) = rad2deg( atan( l(i) / ( R(i) - w(i)/2 ) ) ) ;

d2(i) = rad2deg( atan( l(i) / ( R(i) + w(i)/2 ) ) ) ;

end

d=(d1+d2)/2;

plot(R,d1,R,d2,R,d)

legend('delta 1','delta 2','delta')


grid on; title(' "Ackerman steering mechanism" (Delta angle - Radius) ');xlabel(' Radius (m) '); ylabel(' delta (deg) '); hold on;  