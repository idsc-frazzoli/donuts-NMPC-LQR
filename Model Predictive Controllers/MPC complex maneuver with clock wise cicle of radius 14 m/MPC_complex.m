%%%%% 
% 
%  Written by Panagiotis Bountouris
%  Semester Project: Go-kart modeling and MPC for donuts drifting maneuvers
%
%%%%%

%%


clear; clc; close all;
import casadi.*;

rad2deg = @(x) x/pi*180;
deg2rad = @(x) x/180*pi;


%% Generate solver for first straight line


% the following parts generate the solver of the first straigh line 


%% Problem dimensions

model.N = 15;            % horizon length
model.nvar = 8;          % number of variables
model.neq  = 6;          % number of equality constraints

%% Define cost/objective function 

model.objective = @(z) 10000000000*(z(1)-0)^2 + 100*(z(2)-1900)^2+ 100*(z(3)-8.17)^2+10000000000*(z(4)-0)^2 + 10000000000*(z(5)-0)^2+ 10000000000*(z(6)-0)^2 ;              


%% Define dynamics and equality constraints 

integrator_stepsize = 0.1;

next_continuous_dynamics = @continuous_dynamics ;

%integrate using Runge Kutta of 4th order 
model.eq = @(z) RK4( z(3:8), z(1:2), next_continuous_dynamics, integrator_stepsize);

% Indices on LHS of dynamical constraint - for efficiency reasons the matrix E has structure [0 I] where I is the identity matrix.
model.E = [zeros(6,2), eye(6)];

%% Define inequality constraints
% upper/lower variable bounds lb <= x <= ub
%            inputs    |        states
%                       d                   fRX     Ux             b                       r                         psi                        x              y   
model.lb = [  -deg2rad(50) , -3500 , -30, -deg2rad(70), -deg2rad(100), deg2rad(-360 * 10), -1000000 , -1000000  ];
model.ub = [ deg2rad(50) , 3500 , 30,  deg2rad(50),  deg2rad(100), +deg2rad(360 * 10), 1000000 , 1000000 ];



%% Initial and final conditions

% Initial condition on vehicle states
model.xinitidx = 3:8; % specify on which variables initial conditions are imposed

% Final condition on vehicle states
model.xfinal = [ 8.17, 0, 0, 0  ]';% set final conditions
model.xfinalidx = 3:6; %  specify on which variables final conditions are imposed


%% Define solver options

codeoptions = getOptions('MPC_straightline_solver');
codeoptions.maxit = 200;    % Maximum number of iterations in optimization problem
codeoptions.printlevel = 0; % define if information is desired to be printed 
codeoptions.optlevel = 2; % 2: optimize for speed
codeoptions.cleanup = 0;

%% Generate forces solver for first straight line

FORCES_NLP(model, codeoptions);


%% Generate MPC solver for donuts case


% the following parts generate the solver fot the donuts


%% Problem dimensions

model.N = 15;            % horizon length
model.nvar = 8;          % number of variables
model.neq  = 6;          % number of equality constraints

%% Define cost/objective function 

model.objective = @(z) 100000000*(z(1)-deg2rad(- 13))^2 + 100*(z(2)-1900)^2+ 10*(z(3)-8.708)^2+100000000*( z(4) - (0.310) )^2 + 100000000*(z(5)- ( -0.6485) )^2 ;             


%% Define dynamics  equality constraints 
% We use an explicit RK4 integrator here to discretize continuous dynamics:


integrator_stepsize = 0.1;

% use of an explicit RK4 integrator to discretize continuous dynamics
next_continuous_dynamics = @continuous_dynamics ;
                          
model.eq = @(z) RK4( z(3:8), z(1:2), next_continuous_dynamics, integrator_stepsize);

% Indices on LHS of dynamical constraint - for efficiency reasons the matrix E has structure [0 I] where I is the identity matrix.
model.E = [zeros(6,2), eye(6)];

%% Define inequality constraints
% upper/lower variable bounds lb <= x <= ub
%            inputs    |        states
%                       d                   fRX     Ux             b                       r                         psi                        x              y   
model.lb = [  -deg2rad(50) , -3500 , -30, -deg2rad(70), -deg2rad(100), -deg2rad(360*100), -1000000 , -1000000  ];
model.ub = [ deg2rad(50) , 3500 , 30,  deg2rad(70),  deg2rad(100), +deg2rad( 360 * 100), 1000000 , 1000000 ];



%% Initial and final conditions

% Initial condition on vehicle states
model.xinitidx = 3:8; % specify on which states initial conditions are imposed

% Final condition on vehicle states
model.xfinal = [ 8.71,  0.31, -0.65    ]'; % specify final conditions of states
model.xfinalidx = 3:5; % specify on which states final conditions are imposed


%% Define solver options

codeoptions = getOptions('FORCESNLPsolver_cl_r14');
codeoptions.maxit = 200;    % Maximum number of iterations in optimization problem
codeoptions.printlevel = 0; % define if information is desired to be printed 
codeoptions.optlevel = 2; % 2: optimize for speed
codeoptions.cleanup = 0;

%% Generate forces solver for donuts

FORCES_NLP(model, codeoptions);



%% Generate  solver for 2nd straight line


% the following parts generate the solver for the second straigh line


%% Problem dimensions

model.N = 15;            % horizon length
model.nvar = 8;          % number of variables
model.neq  = 6;          % number of equality constraints

%% Define cost / objective function 

model.objective = @(z)   +30*( z(7)-0 )^2 + 50*( z(8) - ( - 28 ) )^2;              


%% Define dynamics and equality constraints 

integrator_stepsize = 0.1;

next_continuous_dynamics = @continuous_dynamics ;

% use of an explicit RK4 integrator to discretize continuous dynamics:
model.eq = @(z) RK4( z(3:8), z(1:2), next_continuous_dynamics, integrator_stepsize);

% Indices on LHS of dynamical constraint - for efficiency reasons, the matrix E has structure [0 I] where I is the identity matrix.
model.E = [zeros(6,2), eye(6)];

%% Inequality constraints
% upper/lower variable bounds lb <= x <= ub
%            inputs    |        states
%             F    s       x     y     v    theta
model.lb = [  -deg2rad(50) , -3500 , -30, -deg2rad(70), -deg2rad(100), deg2rad(-360 * 10), -1000000 , -1000000  ];
model.ub = [ deg2rad(50) , 3500 , 30,  deg2rad(50),  deg2rad(100), +deg2rad(360 * 10), 1000000 , 1000000 ];



%% Initial and final conditions

% Initial conditions on vehicle states
model.xinitidx = 3:8; % specify on which states initial conditions are imposed

% final conditions on vehicle states
model.xfinal = [ 0 , -28 ]';  % specify final conditions of states
model.xfinalidx =  7:8 ;     %  specify on which states final conditions are imposed


%% Define solver options

codeoptions = getOptions('MPC_2ndstraightline_solver');
codeoptions.maxit = 200;    % Maximum number of iterations in optimization problem
codeoptions.printlevel = 0; % define if information is desired to be printed 
codeoptions.optlevel = 2; % 2: optimize for speed
codeoptions.cleanup = 0;


%% Generate forces solver for second straight line

FORCES_NLP(model, codeoptions);


%% Define vehicle's parameters

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

%% Start simulation


time = 0 ;

times = 0 : 0.001 : integrator_stepsize ;

i = 1 ;

% initial conditions
X(:,1) = [  1 ;    % Ux = 0 results in singularity
                0 ;
                0 ;
                0 ;
                0 ;
                0 ] ;
 
            
%% Simulate straight line

while X(1,i) < 8.17 

[output,exitflag,info] = solver_straightline(X(:,i))    % call solver for first straight line

% extract the whole control strategy from solver
for k=1:model.N
   TEMP(:,k) = output.(['x',sprintf('%02d',k)]);
end

% extract only the first control inputs from solver
U(1:2,i) = TEMP(1:2,1);
    
dx(:,1) = X(:,i);

delta=U(1,i);
fRx=U(2,i);

Ux = X(1,i);
beta = X(2,i);
r = X(3,i);
psi = X(4,i);
x = X(5,i);
y = X(6,i);

for t=2:length(times)


% solve tyre modeling
[ fFy, fRy] = tyremodel( Ux, beta, r, delta , fRx ); 

FR(i)=sqrt( fRy^2 + fRx^2 ) ;
FFy(i)=fFy ;

% solve dynamic modeling
xdot = [ 1/m * [ - fFy * sin(delta) + fRx ] + r * beta* Ux ;
            1/( Ux * m ) * [  fFy + fRy ] - r ;
            1/Iz * [ lF * fFy * cos(delta) - lR * fRy] ; 
             r ;
            Ux * ( cos( psi ) - tan( beta ) * sin( psi ) ) ;
            Ux * ( sin( psi ) + tan( beta ) * cos( psi ) ) ];

             
     Ux = Ux  + xdot(1) *  0.001 ;
     beta = beta  + xdot(2) *  0.001 ;
     r = r  + xdot(3) *  0.001 ;
     psi = psi  + xdot(4) *  0.001 ;
     x = x  + xdot(5) *  0.001 ;
     y = y  + xdot(6) *  0.001 ;
    
    
end

X( 1, i + 1 ) = Ux ;
X( 2 , i +1 ) = beta ;
X( 3 , i +1 ) = r ;
X( 4 , i +1 ) = psi ;
X(5 , i +1 ) = x ;
X( 6 , i +1 ) = y ;

time = time + integrator_stepsize ; 

id(i)=1;

i = i + 1 ;


end

%% Simulate donuts 

while X( 4 , i ) >= deg2rad( - 360 * 3 - 180 + 60  )   

[output,exitflag,info] = solver_cl_r14(X(:,i))      % call solver for donuts

% extract the whole control strategy from solver
for k=1:model.N
   TEMP(:,k) = output.(['x',sprintf('%02d',k)]);
end

% extract only the first control inputs from solver
U(1:2,i) = TEMP(1:2,1);
    
dx(:,1) = X(:,i);

delta=U(1,i);
fRx=U(2,i);

Ux = X(1,i);
beta = X(2,i);
r = X(3,i);
psi = X(4,i);
x = X(5,i);
y = X(6,i);

[ FFy(i), FRy(i)] = tyremodel( Ux, beta, r, delta , fRx ); 

FR(i)=sqrt( FRy(i)^2 + fRx^2 ) ;
FRx(i)=fRx ;

for t=2:length(times)


% solve tyre modeling
[ fFy, fRy] = tyremodel( Ux, beta, r, delta , fRx ); 

FR(i)=sqrt( fRy^2 + fRx^2 ) ;
FFy(i)=fFy ;

% solve dynamic modeling
xdot = [ 1/m * [ - fFy * sin(delta) + fRx ] + r * beta* Ux ;
            1/( Ux * m ) * [  fFy + fRy ] - r ;
            1/Iz * [ lF * fFy * cos(delta) - lR * fRy] ; 
             r ;
            Ux * ( cos( psi ) - tan( beta ) * sin( psi ) ) ;
            Ux * ( sin( psi ) + tan( beta ) * cos( psi ) ) ];

             
     Ux = Ux  + xdot(1) *  0.001 ;
     beta = beta  + xdot(2) *  0.001 ;
     r = r  + xdot(3) *  0.001 ;
     psi = psi  + xdot(4) *  0.001 ;
     x = x  + xdot(5) *  0.001 ;
     y = y  + xdot(6) *  0.001 ;
    
    
end

X( 1, i + 1 ) = Ux ;
X( 2 , i +1 ) = beta ;
X( 3 , i +1 ) = r ;
X( 4 , i +1 ) = psi ;
X(5 , i +1 ) = x ;
X( 6 , i +1 ) = y ;

time = time + integrator_stepsize ; 

id(i)=2;

i = i + 1 ;


end


%% Simulate 2nd straight line


while X(5,i-1) > 0 

[output,exitflag,info] = solver_2ndstraightline(X(:,i))    % call solver for 2nd staight line
 
% extract the whole control strategy from solver for  the 2nd staight line
for k=1:model.N
   TEMP(:,k) = output.(['x',sprintf('%02d',k)]);
end

% extract only the first control inputs from solver
U(1:2,i) = TEMP(1:2,1);
    
dx(:,1) = X(:,i);

delta=U(1,i);
fRx=U(2,i);

Ux = X(1,i);
beta = X(2,i);
r = X(3,i);
psi = X(4,i);
x = X(5,i);
y = X(6,i);

for t=2:length(times)


% solve tyre modeling
[ fFy, fRy] = tyremodel( Ux, beta, r, delta , fRx ); 

FR(i)=sqrt( fRy^2 + fRx^2 ) ;
FFy(i)=fFy ;

% solve dynamic modeling
xdot = [ 1/m * [ - fFy * sin(delta) + fRx ] + r * beta* Ux ;
            1/( Ux * m ) * [  fFy + fRy ] - r ;
            1/Iz * [ lF * fFy * cos(delta) - lR * fRy] ; 
             r ;
            Ux * ( cos( psi ) - tan( beta ) * sin( psi ) ) ;
            Ux * ( sin( psi ) + tan( beta ) * cos( psi ) ) ];

             
     Ux = Ux  + xdot(1) *  0.001 ;
     beta = beta  + xdot(2) *  0.001 ;
     r = r  + xdot(3) *  0.001 ;
     psi = psi  + xdot(4) *  0.001 ;
     x = x  + xdot(5) *  0.001 ;
     y = y  + xdot(6) *  0.001 ;
    
    
end

X( 1, i + 1 ) = Ux ;
X( 2 , i +1 ) = beta ;
X( 3 , i +1 ) = r ;
X( 4 , i +1 ) = psi ;
X(5 , i +1 ) = x ;
X( 6 , i +1 ) = y ;

time = time + integrator_stepsize ; 

id(i)=4;


i = i + 1 ;


end



%% Plot inputs

time = linspace(0,time, size( X , 2 ) ) ;

figure(1); 

% plot delta
subplot(2 ,1 , 1);
plot(time(1:end-1) , rad2deg(U(1,:))); grid on; title(' delta - t ');xlabel(' time (s) '); ylabel(' delta (deg) '); hold on;  

% plot fRx
subplot(2 ,1 , 2);
plot(time(1:end-1) , U(2,:)); grid on; title(' fRx - t ');xlabel(' time (s) '); ylabel(' fRx (N) '); hold on;  

%% Plot all states

figure(2);  

% plot Ux
subplot(3 ,3 , 1); 
plot(time , X(1,:)); grid on; title(' Ux - t ');xlabel(' time (s) '); ylabel(' Ux (m/s) '); hold on;  

% plot beta
subplot(3 , 3 , 2); 
plot(time ,rad2deg(X(2,:))); grid on; title(' beta - t ');xlabel(' time (s) '); ylabel(' beta (deg) '); hold on;  

% plot r
subplot(3 , 3 , 3); 
plot(time ,rad2deg(X(3,:))); grid on; title(' r - t ');xlabel(' time (s) '); ylabel(' r (deg/s) '); hold on;  

% plot psi
subplot(3 , 3 , 4); 
plot(time ,rad2deg(X(4,:))); grid on; title(' psi - t ');xlabel(' time (s) '); ylabel(' psi (deg) '); hold on;  

% plot x
subplot(3 , 3 , 5); 
plot(time ,X(5,:)); grid on; title(' x - t ');xlabel(' time (s) '); ylabel(' x (m) '); hold on;  

% plot y
subplot(3 , 3 , 6); 
plot(time ,X(6,:)); grid on; title(' y - t ');xlabel(' time (s) '); ylabel(' y (m) '); hold on;  

% plot x-y
subplot(3 , 3 , 7:9 ); 
plot(X(5,:),X(6,:)); grid on; title(' x - y ');xlabel(' x (m) '); ylabel(' y (m) '); hold on;  


%% Plot vehicle's trajectory

figure(3); 

translucency = 0.1;
% distance from COG to front end [m]
params.frontL = 1.5;

% distance from COG to rear end [m]
params.rearL = 0.9;

% width of the vehicle [m]
params.width = 1.48;

axis equal
grid on
hold on

axis([-10 140 -60 10 ])

N = max(size(id));

for i=1:1:N;
    
   
     draw_car(X(5,i),X(6,i),X(4,i),params.frontL, params.rearL, params.width,id(i),translucency);


h1 = draw_car( X(5,end),X(6,end),X(4,end),params.frontL, params.rearL, params.width,id(i),translucency);

M(i) = getframe;


end

%% Plot forces

mfriction= 0.7;   % friction coefficient between rubber and asphault in dry case 

mfriction_front = 0.70 ;
mfriction_rear = 0.67 ;

% find vertical forces fFz and fRz
fFz = [ m * g * lR  ] / ( lF + lR )  ;
fRz = [ m * g * lF ] / ( lF + lR )   ;

% better visualization
for i=1:length(FR)
    if FR(i)>fRz*mfriction_rear
        FR(i)=fRz*mfriction_rear ;
    end
end


figure(4); %clf;

% plot fFy
subplot(2 ,1 , 1);
plot(time(1:end-1) , FFy(:)); grid on; title(' fFy - t ');xlabel(' time (s) '); ylabel(' fFy (N) '); hold on;  
plot( time, ones(length(time),1) * fFz * mfriction_front , '-- k') ;
plot( time, -ones(length(time),1) * fFz * mfriction_front , '-- k') ;

% plot fR
subplot(2 ,1 , 2);
plot(time(1:end-1) , FR(:)); grid on; title(' fR - t ');xlabel(' time (s) '); ylabel(' fR (N) '); hold on;  
plot( time, ones(length(time),1) * fRz * mfriction_rear , '-- k') ;
plot( time, -ones(length(time),1) * fRz * mfriction_rear , '-- k') ;


%%  Create txt file for gazebo simulation


fileID = fopen('MPCcomplexmaneuver.txt','wt');

for i=1:1:max(size(X))

fprintf(fileID,'%4.2f %4.2f 0 %4.2f\n',X(5,i),X(6,i),X(4,i)); % extract x, y, psi 

end

fclose(fileID)

