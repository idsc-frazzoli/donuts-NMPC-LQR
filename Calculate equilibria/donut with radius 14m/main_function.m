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

%% Define friction coefficients

mfriction= 0.7;   % friction coefficient between rubber and asphault in dry case  ()

mfriction_front = 0.70 ;
mfriction_rear = 0.67 ;


% find vertical forces fFz and fRz
fFz = [ m * g * lR  ] / ( lF + lR )  ;
fRz = [ m * g * lF ] / ( lF + lR )   ;


%% Main function to solve numerically the system
 
 Ux =8.71 ;
deltavalues = deg2rad( -20:1:20 ) ; % delta belongs to the space [-20, 20] degrees

for i=1:length(deltavalues)

delta = deltavalues(i) ;
    
% case 1 : fFy=fFypaj and fRy=fRypaj
[ be1, r1, fFy1, fRy1, fRx1, fR1 ] = fFypaj_fRypaj ( Ux , delta ) ; 

if fRx1<0          % check for numerical erros since fRx cannot be negative
    fFy1=inf ;
    fRy1=inf ;
end

% case 2 : fFy=fFypaj and fRy=fRymax
[ be2, r2, fFy2, fRy2, fRx2, fR2 ] = fFypaj_fRymax ( Ux , delta ) ;

if fRx2<0          % check for numerical erros since fRx cannot be negative
    fFy2=inf ;
    fRy2=inf ;
end

% case 3 : fFy=fFymax and fRy=fRymax
[ be3, r3, fFy3, fRy3, fRx3, fR3 ] = fFymax_fRymax ( Ux , delta ) ;

if fRx3<0          % check for numerical erros since fRx cannot be negative
    fFy3=inf ;
    fRy3=inf ;
end

% case 4 : fFy=fFymax and fRy=fRypaj
[ be4, r4, fFy4, fRy4, fRx4, fR4 ] = fFymax_fRypaj ( Ux , delta ) ;

if fRx4<0          % check for numerical erros since fRx cannot be negative
    fFy4=inf ;
    fRy4=inf ;
end

fFytemp= [ fFy1, fFy2, fFy3, fFy4 ] ;     % define vector with the 4 cases of fFy
fRytemp= [ fRy1, fRy2, fRy3, fRy4 ] ;   % define vector with the 4 cases of fRy

% 1st case leads to mimimum friction forces
if ( fFy1==min(fFytemp) ) && ( fRy1==min(fRytemp) )
    
    betaangle= be1; 
    r=r1;
    fFy=fFy1;
    fRy=fRy1; 
    fRx=fRx1; 
    fR=fR1;

  % 2nd case leads to mimimum friction forces
elseif ( fFy2==min(fFytemp) ) && ( fRy2==min(fRytemp) )
   
    betaangle= be2; 
    r=r2; 
    fFy=fFy2;
    fRy=fRy2;
    fRx=fRx2; 
    fR=fR2;
   
   % 3nd case leads to mimimum friction forces
elseif ( fFy3==min(fFytemp) ) && ( fRy3==min(fRytemp) )
   
    betaangle= be3;  
    r=r3; 
    fFy=fFy3;
    fRy=fRy3;
    fRx=fRx3; 
    fR=fR3;    

  % 4th case leads to mimimum friction forces   
elseif ( fFy4==min(fFytemp) ) && ( fRy4==min(fRytemp) )
   
    betaangle= be4;  
    r=r4; 
    fFy=fFy4; 
    fRy=fRy4;
    fRx=fRx4;
    fR=fR4;   
    
end
 

% better visualization
if fR > (mfriction_rear * fRz)
    fR = mfriction_rear * fRz ;
    fRx = sqrt ( (fR)^2 - fRy^2 ) ;
end

% extract the results for each steering angle
 Uxmatrix(i) = Ux ;
 betaanglematrix(i) = betaangle ;
 rmatrix(i) = r ;
 fFymatrix(i)= fFy ;
 fRxmatrix(i)= fRx ;
 fRymatrix(i)= fRy ;
 fRmatrix(i)= fR ;
 
end

%%  Copy matrices to auxilary matrices

Uxmatrix10 = Uxmatrix ;
 betaanglematrix10 =  betaanglematrix ;
 rmatrix10 = rmatrix ;
 fFymatrix10 =  fFymatrix ;
 fRxmatrix10 =  fRxmatrix ;
 fRymatrix10 = fRymatrix ;
 fRmatrix10 = fRmatrix ;


%% Plot with discrimination in asterisk and triangle 

mfriction= 0.7;   % friction coefficient between rubber and asphault in dry case 

mfriction_front = 0.70 ;
mfriction_rear = 0.67 ;

m=850; 
% find vertical forces fFz and fRz
fFz = [ m * g * lR  ] / ( lF + lR )  ;
fRz = [ m * g * lF ] / ( lF + lR )   ;

% better visualization 
for i=1:length(fRmatrix10)
    fRmatrix10(i)=fRmatrix10(i)+10;     
end


% plot beta
figure(1); %clf;

for i=1:length(deltavalues) 
    

if  ( fRmatrix10(i) ) <  ( fRz * mfriction_rear  )
    
        plot(rad2deg(deltavalues(i)), rad2deg( betaanglematrix10(i)), ' * b ' ); grid on; xlabel(' delta^e^q (deg) ','FontSize', 16); ylabel(' beta^e^q (deg) ','FontSize', 16); hold on;  
        
else
        plot(rad2deg(deltavalues(i)), rad2deg( betaanglematrix10(i)), ' ^ b' ); grid on; xlabel(' delta^e^q  (deg) ','FontSize', 16); ylabel(' beta^e^q  (deg) ','FontSize', 16); hold on;  
end

end

% plot r
figure(2); %clf;

for i=1:length(deltavalues)    

if ( fRmatrix10(i) ) <  ( fRz * mfriction_rear )

         plot(rad2deg(deltavalues(i)), rad2deg( rmatrix10(i)), ' * b' ); grid on; xlabel(' delta^e^q (deg) ','FontSize', 16); ylabel(' r^e^q (deg/s) ','FontSize', 16); hold on;  
else
       plot(rad2deg(deltavalues(i)), rad2deg( rmatrix10(i)), ' ^ b' ); grid on; xlabel(' delta^e^q (deg) ','FontSize', 16); ylabel(' r^e^q (deg/s) ','FontSize', 16); hold on; 
       
end

end

% plot fFy
figure(3); %clf;

for i=1:length(deltavalues)    

if ( fRmatrix10(i) ) <  ( fRz * mfriction_rear )

plot(rad2deg(deltavalues(i)), fFymatrix10(i), ' * b' ); grid on; xlabel(' delta^e^q (deg) ','FontSize', 16); ylabel(' f_F_y^e^q (N) ','FontSize', 16); hold on;  

else
plot(rad2deg(deltavalues(i)), fFymatrix10(i), ' ^ b'); grid on; xlabel(' delta^e^q (deg) ','FontSize', 16); ylabel('  f_F_y^e^q (N) ','FontSize', 16); hold on;  
end
end

plot(rad2deg(deltavalues(:)), ones(length(deltavalues)) * (fFz* mfriction_front) , ' -- k ', 'LineWidth',1  ) ;
plot(rad2deg(deltavalues(:)), - ones(length(deltavalues)) * (fFz* mfriction_front) , ' -- k ','LineWidth',1 ) ;

% plot fRx
figure(4); %clf;

for i=1:length(deltavalues)    

if ( fRmatrix10(i) ) <  ( fRz * mfriction_rear )
    
plot(rad2deg(deltavalues(i)), fRxmatrix10(i), ' * b' ); grid on; xlabel(' delta^e^q (deg) ','FontSize', 16); ylabel(' f_R_x^e^q (N) ','FontSize', 16); hold on;  

else
    
plot(rad2deg(deltavalues(i)), fRxmatrix10(i), ' ^ b' ); grid on; xlabel(' delta^e^q (deg) ','FontSize', 16); ylabel(' f_R_x^e^q (N) ','FontSize', 16); hold on;  
end
end

% plot fR
figure(5); %clf;

for i=1:length(deltavalues)    

if ( fRmatrix10(i) ) <  ( fRz * mfriction_rear )
    
plot(rad2deg(deltavalues(i)), fRmatrix10(i), ' * b'   ); grid on; xlabel(' delta^e^q (deg) ','FontSize', 16); ylabel(' f_R^e^q  (N) ','FontSize', 16); hold on;  

else
plot(rad2deg(deltavalues(i)), fRz * mfriction_rear, ' ^ b' ); grid on; xlabel(' delta^e^q (deg) ','FontSize', 16); ylabel(' f_R^e^q  (N) ','FontSize', 16); hold on;  
end
end

plot(rad2deg(deltavalues(:)), ones(length(deltavalues)) * (fRz* mfriction_rear)  , ' -- k ', 'LineWidth',1 ) ;


% plot fRy
figure(6); %clf;

for i=1:length(deltavalues)    

if ( fRmatrix10(i) ) < ( fRz * mfriction_rear  )
    
plot(rad2deg(deltavalues(i)), fRymatrix(i), ' * b'   ); grid on; xlabel(' delta^e^q (deg) ','FontSize', 16); ylabel(' f_R_y^e^q  (N) ','FontSize', 16); hold on;  

else
plot(rad2deg(deltavalues(i)), fRymatrix(i), ' ^ b ' ); grid on; xlabel(' delta^e^q (deg) ','FontSize', 16); ylabel(' f_R_y^e^q  (N) ','FontSize', 16); hold on;  
end
end

plot(rad2deg(deltavalues(:)), ones(length(deltavalues)) * (fRz* mfriction_rear) , ' -- k ', 'LineWidth',1  ) ;
plot(rad2deg(deltavalues(:)), -ones(length(deltavalues)) * (fRz* mfriction_rear) , ' -- k ', 'LineWidth',1  ) ;



