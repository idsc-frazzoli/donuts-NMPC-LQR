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

%% Define steady states

Xs = [  2.690 ;
            -0.729 ;
            1.4790 ;
            0 ;
            0 ;
            0 ] ;
       
Us = [  deg2rad(15) ; 
           2800 ] ;      
 
 %% Auxilary vector for linearization of states
 
 vector1 = zeros(6,1)   ;
 vector1(1) = 1 ;
 
 vector2 = zeros(6,1)   ;
 vector2(2) = 1 ;
 
 vector3 = zeros(6,1)   ;
 vector3(3) = 1 ;
 
 vector4 = zeros(6,1)   ;
 vector4(4) = 1 ;
 
 vector5 = zeros(6,1)   ;
 vector5(5) = 1 ;
 
 vector6 = zeros(6,1)   ;
 vector6(6) = 1 ;
 
 dt = 0.001 ;
 
 %% Auxilary vector for linearization of inputs
 
 vector1u = zeros(2,1)   ;
 vector1u(1) = 1 ;
 
 vector2u = zeros(2,1)   ;
 vector2u(2) = 1 ;
 
 %% central difference numerical linearization for A
 
xdot1col = [ continuous_dynamics( Xs + dt*vector1 , Us ) - continuous_dynamics( Xs - dt*vector1 , Us ) ] / ( 2 * dt ) ;
 
xdot2col = [ continuous_dynamics( Xs + dt*vector2 , Us ) - continuous_dynamics( Xs - dt*vector2 , Us ) ] / ( 2 * dt ) ;
 
xdot3col = [ continuous_dynamics( Xs + dt*vector3 , Us ) - continuous_dynamics( Xs - dt*vector3 , Us ) ] / ( 2 * dt ) ;
  
xdot4col = [ continuous_dynamics( Xs + dt*vector4 , Us ) - continuous_dynamics( Xs - dt*vector4 , Us ) ] / ( 2 * dt ) ;
 
xdot5col = [ continuous_dynamics( Xs + dt*vector5 , Us ) - continuous_dynamics( Xs - dt*vector5 , Us ) ] / ( 2 * dt ) ;
    
xdot6col = [ continuous_dynamics( Xs + dt*vector6 , Us ) - continuous_dynamics( Xs - dt*vector6 , Us ) ] / ( 2 * dt ) ;
 
linfA= [ xdot1col , xdot2col , xdot3col  , xdot4col  , xdot5col  , xdot6col  ] ;

A = linfA 

  %% central difference numerical linearization for B 
  
  xdot1colu = [ continuous_dynamics( Xs  , Us + dt*vector1u  ) - continuous_dynamics( Xs  , Us - dt*vector1u ) ] / ( 2 * dt ) ;
  
  xdot2colu = [ continuous_dynamics( Xs  , Us + dt*vector2u  ) - continuous_dynamics( Xs  , Us - dt*vector2u ) ] / ( 2 * dt ) ;
  
  linfB= [ xdot1colu , xdot2colu  ] ;

  B = linfB

  