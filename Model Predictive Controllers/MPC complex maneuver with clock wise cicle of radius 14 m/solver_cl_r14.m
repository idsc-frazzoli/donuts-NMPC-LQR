%%%%% 
% 
%  Written by Panagiotis Bountouris
%  Semester Project: Go-kart modeling and MPC for donuts drifting maneuvers
%
%%%%%

%% Define solver of donut

function  [output,exitflag,info] = solver_cl_r14(x_state)


model.N = 15;                                             % horizon length
model.xfinal = [ 8.708,  0.31, -0.648    ]';    % final conditions for Ux, b, r states

%%

% Set initial guess to start solver
x0i=[ deg2rad(-13) ,1900 , 8.708 ,  0.310 ,  -0.6485,-56.78, 65.17 , -2.533 ];
x0=repmat(x0i',model.N,1);
problem.x0=x0; 


problem.xinit = x_state;
problem.xfinal = model.xfinal;

% solve the NLP
[output,exitflag,info] = FORCESNLPsolver_cl_r14(problem);


end