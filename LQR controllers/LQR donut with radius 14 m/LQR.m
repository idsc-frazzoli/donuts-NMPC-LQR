%%%%% 
% 
%  Written by Panagiotis Bountouris
%  Semester Project: Go-kart modeling and MPC for donuts drifting maneuvers
%
%%%%%

%%


clear all 
clc
close all

%% Define vehicle's parameters

m=850;            % mass of kart (kg)
lF=1.5;             % distance between front axle and center of mass (m)
lR=0.9;             % distance between rear axle and center of mass (m)
wL=0.74;          % distance between left wheel and center of mass (m)   
wR=0.74;         % distance between right wheel and center of mass (m) 
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

%%  Linearization


A = [-0.202906453380169	-5.79060791942776	2.67532196129239	0	0	0 ;
            0.0750418327651881	-0.267291505056722	-0.992420693752605	0	0	0 ;
            -0.0143361539408358	0.363767534120305	-0.192490178334635	0	0	0 ;
            0	0	1.00000000000000	0	0	0 ;
            1.04706930335663	2.23126286499031	0	-0.689878543965961	0	0 ;
            0.0792195865838563	9.33934836447203	0	9.11833395729356	0	0 ] ;

B = [ 2.24543995544868	0.00117647058817116 ;
            0.0755928044459120	0 ;
            0.143173325749818	0 ;
            0	0 ;
            0	0 ;
            0	0 ] ;
    
C = [ 0 0 0 0 1 0;
         0 0 0 0 0 1 ];
 
D = [ 0  0;
         0  0];
     
     
states = { 'Ux' 'beta' 'r' 'psi' 'x' 'y' };
inputs = { 'delta' 'fRx' };
outputs = { 'x'; 'y'} ;

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);



%%

Q = diag([100 1000000 100 1 1 1]); 
R = diag([1000000  1000]);
Co = ctrb(A,B);
rank(Co) ;

K = lqr(A, B,Q,R);

K( : , 4:6 ) = 0 ;  % there is no reference for psi, x, y


%% Simulation


% steady states  for drifting case (tracking only for Ux, b and r)
Xs  = [ 8.70843548540520 ;
0.310028535481809 ;
-0.648581154313099 ;
0 ;
0 ;
0 ] ;
       
Us =  [   deg2rad( - 13 );
           1900 ] ; 

       
 % initial states from zero     
X0 =  [  1;   % singularity for Ux=0
            0;
             0;
              0 ;
               0 ; 
               0 ] ;       
       
       


%           delta            fRx
delta = Us(1);
fRx = Us(2);

dt = 0.001 ; % integration stepsize
t = 80 ; % time of simulation 

time = 0 : dt : t ; % integration instances

X(:,1) =  X0 ; 

for i=2:length(time)
    
    e = X(:,i-1) - Xs ; 
    
    U( : , i - 1 ) = K * e + Us ;   
    
   % inputs
    delta=U(1,i-1);
    fRx=U(2,i-1);

    % states
    Ux = X(1,i-1);
    beta = X(2,i-1);
    r = X(3,i-1);
    psi = X(4,i-1);
    x = X(5,i-1);
    y = X(6,i-1);
    
   %calculate friction forces through solving tyre modeling
   [ fFy, fRy] = tyremodel( Ux, beta, r, delta , fRx ) ;


    % integrate and find next states
    xdot = [ 1/m * [ - fFy * sin(delta) + fRx ] + r * beta* Ux ;
                 1/( Ux * m ) * [  fFy + fRy ] - r ;
                 1/Iz * [ lF * fFy * cos(delta) - lR * fRy] ; 
                   r ;
                  Ux * ( cos( psi ) - tan( beta ) * sin( psi ) ) ;
                  Ux * ( sin( psi ) + tan( beta ) * cos( psi ) ) ];
    
    X(:,i) = X(:,i-1) + dt * xdot ; % update the states 
    
    fFym(i-1) = fFy ;
    fRym(i-1) = fRy ;
    
end


%% Plots

% plot all together
figure(1); %clf;

% plot Ux
subplot(3 ,3 , 1); 
plot(time, X(1,:)); grid on; title(' Ux - t ');xlabel(' time (s) '); ylabel(' Ux (m/s) '); hold on;  

% plot beta
subplot(3 , 3 , 2); 
plot(time,rad2deg(X(2,:))); grid on; title(' beta - t ');xlabel(' time (s) '); ylabel(' beta (deg) '); hold on;  

% plot r
subplot(3 , 3 , 3); 
plot(time,rad2deg(X(3,:))); grid on; title(' r - t ');xlabel(' time (s) '); ylabel(' r (deg/s) '); hold on;  

% plot psi
subplot(3 , 3 , 4); 
plot(time,rad2deg(X(4,:))); grid on; title(' psi - t ');xlabel(' time (s) '); ylabel(' psi (deg) '); hold on;  

% plot x
subplot(3 , 3 , 5); 
plot(time,X(5,:)); grid on; title(' x - t ');xlabel(' time (s) '); ylabel(' x (m) '); hold on;  

% plot y
subplot(3 , 3 , 6); 
plot(time,X(6,:)); grid on; title(' y - t ');xlabel(' time (s) '); ylabel(' y (m) '); hold on;  

% plot x-y
subplot(3 , 3 , 7:9 ); 
plot(X(5,:),X(6,:)); grid on; title(' x - y ');xlabel(' x (m) '); ylabel(' y (m) '); hold on;  


figure(2)


% plot U
subplot(2 , 1 , 1); 
plot(time,[rad2deg(U(1,:)) U(1,end)]); grid on; title(' delta - t ');xlabel(' time (s) '); ylabel(' delta (deg) '); hold on;  

subplot(2 , 1 , 2); 
plot(time,[U(2,:) U(1,end)]  ); grid on; title(' fRx - t ');xlabel(' time (s) '); ylabel(' fRx (N) '); hold on;  


%% Plot vehicle's trajectory in video plot

figure(3);clf;
% draw car

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

N = max(size(X));

for i=1:100:0.5*N;
    
   
     draw_car(X(5,i),X(6,i),X(4,i),params.frontL, params.rearL, params.width,1,translucency);


h1 = draw_car( X(5,end),X(6,end),X(4,end),params.frontL, params.rearL, params.width,1,translucency);

M(i)=getframe; % show each frame

end

grid on; xlabel(' x (m) ' , 'fontsize' , 16 ); ylabel(' y (m) ' ,  'fontsize' , 16 ); hold on;  
plot(X(5,:),X(6,:),'- r'); grid on;



%% Create txt file for gazebo simulation


fileID = fopen('bigcirlce.txt','wt');

for i=1:1:max(size(X))

fprintf(fileID,'%4.2f %4.2f 0 %4.2f\n',X(5,i),X(6,i),X(4,i)); % extract x, y, psi 

end

fclose(fileID)




%%
time=time(1:end-1);
%% Plot forces 
mfriction= 0.7;   % friction coefficient between rubber and asphault in dry case  

mfriction_front = 0.70 ;
mfriction_rear = 0.67 ;

% find vertical forces fFz and fRz
fFz = [ m * g * lR  ] / ( lF + lR )  ;
fRz = [ m * g * lF ] / ( lF + lR )   ;


fR(:)=sqrt( ( U(2,:) ).^2 + (fRym(:))'.^2 ) ; 

% better visualization
for i=1:length(fR)
    if fR(i)>fRz*mfriction_rear
        fR(i)=fRz*mfriction_rear;
    end
end


figure(4);clf;

% plot U
subplot(2 , 1 , 1); 
plot( time, fFym(:) ); grid on; title(' fFy - t ');xlabel(' time (s) '); ylabel(' fFy (N) '); hold on;  
plot( time, ones(length(time),1) * fFz * mfriction_front , '-- k', 'LineWidth',1) ;
plot( time, -ones(length(time),1) * fFz * mfriction_front ,'-- k', 'LineWidth',1) ;

subplot(2 , 1 , 2); 
plot( time, fR(:) ); grid on; title(' fR - t ');xlabel(' time (s) '); ylabel(' fR (N) '); hold on;  
plot( time, ones(length(time),1) * fRz * mfriction_rear , '-- k', 'LineWidth',1) ;
plot( time, -ones(length(time),1) * fRz * mfriction_rear , '-- k', 'LineWidth',1) ;

%% PLOT INDIVIDUALLY FOR REPORT

X=X(:,1:end-1);

% plot all together
figure(5); %clf;

% plot Ux
plot(time, X(1,:)); grid on; xlabel(' time (s) ' , 'fontsize' , 16 ); ylabel(' Ux (m/s) ' ,  'fontsize' , 16 ); hold on;  

figure(6); %clf;


% plot beta

plot(time,rad2deg(X(2,:))); grid on; xlabel(' time (s) ' , 'fontsize' , 16); ylabel(' beta (deg) ', 'fontsize' , 16); hold on;  

figure(7); %clf;

% plot r
plot(time,rad2deg(X(3,:))); grid on;xlabel(' time (s) ', 'fontsize' , 16); ylabel(' r (deg/s) ', 'fontsize' , 16); hold on;  

figure(8); %clf;

% plot psi
plot(time,rad2deg(X(4,:))); grid on; xlabel(' time (s) ', 'fontsize' , 16); ylabel(' psi (deg) ', 'fontsize' , 16); hold on;  

figure(9); %clf;

% plot x
plot(time,X(5,:)); grid on; xlabel(' time (s) ', 'fontsize' , 16); ylabel(' x (m) ', 'fontsize' , 16); hold on;  

figure(10); %clf;

% plot y
plot(time,X(6,:)); grid on; xlabel(' time (s) ', 'fontsize' , 16); ylabel(' y (m) ', 'fontsize' , 16); hold on;  

figure(11);

% plot U
subplot(2 , 1 , 1); 
plot(time,rad2deg(U(1,:))); grid on; xlabel(' time (s) ' , 'fontsize' , 16); ylabel(' delta (deg) ', 'fontsize' , 16); hold on;  

subplot(2 , 1 , 2); 
plot(time,U(2,:)); grid on; xlabel(' time (s) ', 'fontsize' , 16); ylabel(' fRx (N) ', 'fontsize' , 16); hold on;  


figure(12);

plot( time, fFym(:) ); grid on; xlabel(' time (s) ' , 'fontsize' , 16); ylabel(' fFy (N) ' , 'fontsize' , 16); hold on;  
plot( time, ones(length(time),1) * fFz * mfriction_front , '-- k', 'LineWidth',1) ;
plot( time, -ones(length(time),1) * fFz * mfriction_front , '-- k', 'LineWidth',1) ;

figure(13);

plot( time, fR(:) ); grid on; xlabel(' time (s) ', 'fontsize' , 16); ylabel(' fR (N) ', 'fontsize' , 16); hold on;  
plot( time, ones(length(time),1) * fRz * mfriction_rear ,'-- k', 'LineWidth',1) ;
plot( time, -ones(length(time),1) * fRz * mfriction_rear , '-- k', 'LineWidth',1) ;


