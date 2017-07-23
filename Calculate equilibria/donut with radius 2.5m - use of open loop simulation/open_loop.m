%%%%% 
% 
%  Written by Panagiotis Bountouris
%  Semester Project: Go-kart modeling and MPC for donuts drifting maneuvers
%
%%%%%

%%


clc
close all 
clear all

%%  Define vehicle's parameters 

m=850;            % mass of kart (kg)
lF=1.5;             % distance between front axle and center of mass (m)
lR=0.9;             % distance between rear axle and center of mass (m)
Iz=1400;           % moment of inertia of kart (kg*m^2)
Iw=0.6;             % moment of inertia of rear wheel (kg*m^2)
B=4;                 % B coefficient of Pacejka's formula ( )
C=1.3;              % C coefficient of Pacejka's formula ( )
D=0.6;              % D coefficient of Pacejka's formula ( )
g=9.81;             % acceleration of gravity (m/sec^2) 
h=0.5;               % height of center of mass (m)

%% Initial Values for states and inputs

% control inputs inserted to system
delta = deg2rad(15);
fRx= 2800;


% initial values of states
Ux = 1;
beta = 0;
r = 0;
psi = 0;
x = 0;
y = 0;

%% Initialize state matrix

X(1,1) = Ux;
X(2,1) = beta;
X(3,1)= r;
X(4,1) = psi;
X(5,1) = x;
X(6,1) = y;

F(1,1) = 0 ;
F(2,1) = fRx ;
F(3,1) = 0 ;
fR(1)=sqrt(F(2,1)^2+F(3,1)^2);

 id(1)=1;

%% 

dt = 0.001;             % define integration stepsize
times = 0:dt:50;     % define total time of simulation

for t = 2:length(times)


  
            delta = deg2rad(15);     % steady steering angle control input
            fRx=2800;                     % steady rear drive force control input
            id(t)=2;


 
 % solve tyre modeling
   [ fFy, fRy]= tyremodel( Ux, beta, r, delta , fRx );


   xdot = [ 1/m * [ - fFy * sin(delta) + fRx ] + r * beta* Ux ;
            1/( Ux * m ) * [  fFy + fRy ] - r ;
            1/Iz * [ lF * fFy * cos(delta) - lR * fRy] ;  
             r ;
            Ux * ( cos( psi ) - tan( beta ) * sin( psi ) ) ;
            Ux * ( sin( psi ) + tan( beta ) * cos( psi ) ) ];
      
        
     % integrate   
     X( : , t ) = X( : , t-1)  + xdot(:) * dt;


      % update states 
      Ux = X( 1 , t );
      beta = X( 2 , t );
      r = X( 3 , t );
      psi = X( 4 , t );
      x = X( 5 , t );
      y = X( 6 , t );
        
      
      
    % calculate tire friction forces  
   F(1,t) = fFy;
   F(2,t) = fRx ;
   F(3,t) = fRy ;   
   fR(t)=sqrt(F(2,t)^2+F(3,t)^2);
                
end

%% Define friction coefficients

m=850; 

% find vertical forces fFz and fRz
fFz = [ m * g * lR  ] / ( lF + lR )  ;
fRz = [ m * g * lF ] / ( lF + lR )   ;

mfriction_front = 0.70 ;
mfriction_rear = 0.67 ;

%% Plot graphs

% plot all together
figure(1); %clf;

% plot velocity Ux
subplot(3 ,1 , 1); 
plot(times, X(1,:)); grid on; xlabel(' time (s) ','FontSize', 16); ylabel(' U_x^e^q (m/s) ','FontSize', 16); hold on;  

% plot beta angle
subplot(3 , 1 , 2); 
plot(times,rad2deg(X(2,:))); grid on; xlabel(' time (s) ','FontSize', 16); ylabel(' beta^e^q (deg) ','FontSize', 16); hold on;  

% plot r
subplot(3 , 1 , 3); 
plot(times,rad2deg(X(2,:))); grid on; xlabel(' time (s) ','FontSize', 16); ylabel(' r^e^q (deg/s) ','FontSize', 16); hold on;  

%% plot forces
figure(2); clf;

% plot fFy
subplot(2 ,1 , 1); 
plot(times,F(1,:)); grid on; xlabel(' time (s) ','FontSize', 16); ylabel(' f_F_y^e^q  (N) ','FontSize', 16); hold on;  
plot(times, ones(length(times),1) * (fFz* mfriction_front) , ' -- k ', 'LineWidth',1  ) ;
plot(times, - ones(length(times),1) * (fFz* mfriction_front) , ' -- k ','LineWidth',1 ) ;

% plot fR
subplot(2 ,1 , 2); 
 grid on; xlabel(' time (s) ','FontSize', 16); ylabel(' f_R^e^q  (N) ','FontSize', 16); hold on; 
 
 
plot(times, fR);
plot(times, ones(length(times),1) * (fRz* mfriction_rear) , ' -- k ', 'LineWidth',1  ) ;
% plot(times, - ones(length(times),1) * (fRz* mfriction_rear) , ' -- k ','LineWidth',1 ) ;

%% plot inputs
figure(3); clf;

%delta
subplot(2 ,1 , 1); 
plot(times,ones(length(times),1) * (rad2deg(delta))); grid on; xlabel(' time (s) ','FontSize', 16); ylabel(' delta^e^q  (deg) ','FontSize', 16); hold on; 

% plot fR
subplot(2 ,1 , 2); 
plot(times,F(2,:)); grid on; xlabel(' time (s) ','FontSize', 16); ylabel(' f_R_x^e^q  (N) ','FontSize', 16); hold on; 





