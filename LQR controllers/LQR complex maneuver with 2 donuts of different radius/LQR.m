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

%%  Linearize first donut ( counterclock wise circle of radius 12.5 m)
        
A12 = [ -0.193113808313772	5.73319422439633	-2.25499437791699	0	0	0 ;
-0.0849372340866461	-0.333975607902048	-0.987881399574953	0	0	0 ;
0.0194562000420514	0.512417879300691	-0.233760583162815	0	0	0 ;
0	0	1.00000000000000	0	0	0 ;
-1.02556913197915	3.84543958580075	0	1.43604886184434	0	0 ;
-0.175771003817382	-7.96596099125635	0	-8.37889841177253	0	0 ] ;

B12= [ -2.25648808252643	0.00117647058817116 ;
0.0860084295821739	0 ;
0.111883550321668	0 ;
0	0 ;
0	0 ;
0	0 ] ;
    
C12 = [ 0 0 0 0 1 0;
         0 0 0 0 0 1 ];
 
D12= [ 0  0;
         0  0];
   
Q12 = diag([100 100 100 1 1 1]); 
R12 = diag([1000000  1000]);
Col12 = ctrb(A12,B12);
rank12=rank(Col12);
K12 = lqr(A12, B12,Q12,R12);

K12( : , 4:6 ) = 0 ;  % there is no reference for psi, x, y

% steady states  for drifting case (we track only  Ux, b and r)
Xs12 = [   8.17 ;
            -0.28 ;
             0.68 ;
            0 ;
            0 ;
            0 ] ;
       
Us12 = [  deg2rad(15) ; 
           1812.42 ] ;       
       
%%  Linearize second donut ( clock wise circle of radius 14 m)

A14 = [-0.202906453380169	-5.79060791942776	2.67532196129239	0	0	0 ;
            0.0750418327651881	-0.267291505056722	-0.992420693752605	0	0	0 ;
            -0.0143361539408358	0.363767534120305	-0.192490178334635	0	0	0 ;
            0	0	1.00000000000000	0	0	0 ;
            1.04706930335663	2.23126286499031	0	-0.689878543965961	0	0 ;
            0.0792195865838563	9.33934836447203	0	9.11833395729356	0	0 ] ;

B14 = [ 2.24543995544868	0.00117647058817116 ;
            0.0755928044459120	0 ;
            0.143173325749818	0 ;
            0	0 ;
            0	0 ;
            0	0 ] ;
    
C14 = [ 0 0 0 0 1 0;
         0 0 0 0 0 1 ];
 
D14 = [ 0  0;
         0  0];

Q14 = diag([100 1000000 100 1 1 1]); 
R14 = diag([1000000  1000]);
Co14 = ctrb(A14,B14);
rank14=rank(Co14);
K14 = lqr(A14, B14,Q14,R14);

K14( : , 4:6 ) = 0 ;  % there is no reference for psi, x, y

% steady states  for drifting case (we track only  Ux, b and r)
Xs14 = [ 8.70843548540520 ;
0.310028535481809 ;
-0.648581154313099 ;
0 ;
0 ;
0 ] ;
       
Us14 = [   deg2rad( - 13 );
           1900 ] ; 


  %%  Linearize straight line
        
Al= [     0	0	0	0	0	0 ; 
            0	-3.74570303346140	-0.999999943424518	0	0	0 ;
            0	-1.26882631385732e-15	-3.07016916986312	0	0	0 ;
            0	0	1	0	0	0 ;
            0.999999999999446	0	0	0	0	0 ;
            0	8.17120272373442	0	8.17119863813340	0	0 ] ;

Bl= [ 0	0.00117647058817116 ;
        1.40463910575157	0 ;
        10.4527972755059	0 ;
        0	0 ;
        0	0 ;
        0	0 ] ;
    
Cl = [ 0 0 0 0 1 0;
         0 0 0 0 0 1 ];
 
Dl = [ 0  0;
         0  0];
   
Ql = diag([100 1 1 1 1 1]); 
Rl = diag([10000  1000]);
Col = ctrb(Al,Bl);
rankl=rank(Col);
Kl = lqr(Al, Bl,Ql,Rl);

Kl( : , 4:6 ) = 0 ;  % there is no reference for psi, x, y


% steady states  for drifting case (we have values only for Ux, b and r)
Xsl = [   8.1712 ;
            0 ;
             0 ;
            0 ;
            0 ;
            0 ] ;
            
       
    Usl = [  deg2rad(0.001) ; 
           0 ] ;   
       
%% Simulation
 
        
        X0 = [   1 ; % Ux = 0 results in singularity
            0 ;
             0 ;
            0 ;
            0 ;
            0 ] ;  

X(:,1) =  X0 ; 

dt = 0.001 ; % integration stepsize

time(1) = 0 ;  

delta = Us12(1);
fRx = Us12(2);

 fFy(1) = 0 ;
 fRy(1) = 0 ;
 id(1)=1 ;
i = 2 ;

% control first donut maneuver 

while X(4,i-1) < deg2rad(360*4-20)
  
    
    e = X(:,i-1) - Xs12 ; 
    
    U( : , i - 1 ) = K12 * e + Us12 ;   
    
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
    
     time(i) = time( i - 1 ) + dt ;
     
      id(i)=1 ;
     
     i = i + 1 ;
    
end


% change to line LQR controller 

while X(3,i-1)  >  deg2rad( 0.5 ) % until you have approximately zero yaw rate( r ) 
  
    e = X(:,i-1) - Xsl ; 
    
    U( : , i - 1 ) = Kl * e + Usl ;   
    
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
 
     time(i) = time( i - 1 ) + dt ;
       
     id(i)=3 ;
     
     i = i + 1 ;
    
end

% auxilary variables to calculate exiting angle from donuts
ilast = i -1 ;
repeat = 1 ;
phiexit=0 ;


% control second donut maneuver

while X( 4, i -1 ) >= deg2rad( - ( 180 - phiexit ) )

    
    e = X(:,i-1) - Xs14 ; 
    
    U( : , i - 1 ) = K14 * e + Us14 ;   
    
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
     time(i) = time( i - 1 ) + dt ;
     
     id(i)=2 ;
     
     
     
     % predict the phiexit from sampling the entry transition
     if round( rad2deg(X(4,i) ) )  == round( rad2deg( X( 4 , ilast ) ) - 360 )  && repeat
         
         itemp = ilast ; % itemp will have the index of the end of the linear part in the entry transition
          imin=ilast;      % time when local minimum in entry transition is found
          
          while  X(3,imin+1) < X(3,imin) % stop when you find local minimum
            
            imin=imin+1;  % local minimum of first transition phase is found
            
          end
        
        while  round(rad2deg(X(3,itemp+1) )) > round(rad2deg(X(3,imin) ))* 0.9  % search up to 90% of the minimum in transition time in order to stay in the linear area (rise time concept)
                                                                                                                              % rad2deg is used because better accuracy is achieved in degrees scale than in radians 
                                                                                                                              
            itemp = itemp + 1 ;  % approximate the time when entry transition is finished
         
        end
        
        itemp=itemp-1 ; 
        
        slope = ( X(3,itemp) - X(3,ilast) ) / ( time(itemp) - time(ilast) ) ;   % find rate of change of yaw rate
        transitiontime= time(itemp) - time(ilast) ;                                     % find duration of entry transition phase
        phiexit = - rad2deg(slope) * transitiontime^2   ;                            % calculate ideal exit angle from the donut maneuver (when to switch from donut controller to 2nd straight line controller)
        repeat = 0; % don't repeat the loop which might happen due to the round() function
        
     end
     
     
     
     i = i + 1 ;
     
end


% control second straight line

lastindex = i - 1;

while abs( X(5, i-1) - X(5, lastindex) ) < 35 
  
    e = X(:,i-1) - Xsl ; 
    
    U( : , i - 1 ) = Kl * e + Usl ;   
    
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
 
     time(i) = time( i - 1 ) + dt ;
       
     id(i)=4 ;
     
     i = i + 1 ;
    
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
plot(X(5,:),X(6,:)); grid on; title(' x - y ');xlabel(' x (m) '); ylabel(' y (m) ');


% plot U
subplot(2 , 1 , 1); 
plot(time,[rad2deg(U(1,:)) U(1,end)]); grid on; title(' delta - t ');xlabel(' time (s) '); ylabel(' delta (deg) '); hold on;  

subplot(2 , 1 , 2); 
plot(time,[U(2,:) U(1,end)]  ); grid on; title(' fRx - t ');xlabel(' time (s) '); ylabel(' fRx (N) '); hold on;  


%%
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

for i=1:400:N;
    
   
     draw_car(X(5,i),X(6,i),X(4,i),params.frontL, params.rearL, params.width,id(i),translucency);


h1 = draw_car( X(5,end),X(6,end),X(4,end),params.frontL, params.rearL, params.width,id(i),translucency);


end


%%
time=time(1:end-1);
%% Plot forces 
mfriction= 0.7;   % friction coefficient between rubber and asphault in dry case  ()

mfriction_front = 0.70 ;
mfriction_rear = 0.67 ;

% find vertical forces fFz and fRz
fFz = [ m * g * lR  ] / ( lF + lR )  ;
fRz = [ m * g * lF ] / ( lF + lR )   ;


fR(:)=sqrt( ( U(2,:) ).^2 + (fRym(:))'.^2 ) ;

% better visualization
for i=1:length(fR)
    
    fR(i)=fR(i)+70; 
    if  fR(i)>fRz*mfriction_rear
        fR(i)=fRz*mfriction_rear ;
    end
end


figure(4);clf;

% plot U
subplot(2 , 1 , 1); 
plot( time, fFym(:) ); grid on; title(' fFy - t ');xlabel(' time (s) '); ylabel(' fFy (N) '); hold on;  
plot( time, ones(length(time),1) * fFz * mfriction_front , '-- k') ;
plot( time, -ones(length(time),1) * fFz * mfriction_front , '-- k') ;

subplot(2 , 1 , 2); 
plot( time, fR(:) ); grid on; title(' fR - t ');xlabel(' time (s) '); ylabel(' fR (N) '); hold on;  
plot( time, ones(length(time),1) * fRz * mfriction_rear , '-- k') ;
plot( time, -ones(length(time),1) * fRz * mfriction_rear , '-- k') ;


%% plot trajectory in video plot 

figure (5)

axis([-20 40 -35 25])

for i=1:300:N;
    
   
     draw_car(X(5,i),X(6,i),X(4,i),params.frontL, params.rearL, params.width,id(i),translucency);


h1 = draw_car( X(5,end),X(6,end),X(4,end),params.frontL, params.rearL, params.width,id(i),translucency);

M(i) = getframe; % show each frame

end


%%  Create txt file for gazebo simulation


fileID = fopen('complexmaneuver2.txt','wt');

for i=1:1:max(size(X))

fprintf(fileID,'%4.2f %4.2f 0 %4.2f\n',X(5,i),X(6,i),X(4,i)); % extract x, y, psi 

end

fclose(fileID)

