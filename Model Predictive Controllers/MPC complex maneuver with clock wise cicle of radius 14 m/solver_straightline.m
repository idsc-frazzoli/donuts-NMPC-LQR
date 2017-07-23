%%%%% 
% 
%  Written by Panagiotis Bountouris
%  Semester Project: Go-kart modeling and MPC for donuts drifting maneuvers
%
%%%%%


%% Define solver of first straight line

function  [output,exitflag,info] = solver_straightline(x_state)


model.N = 15;                                       % horizon length
model.xfinal = [ 8.17, 0, 0, 0  ]';             % final conditions



%%

% Set initial guess to start solver 
x0i=[ deg2rad(0) ,1900 , 8.17,  deg2rad(0),  deg2rad(0), +deg2rad(0), 0 , 0 ];
x0=repmat(x0i',model.N,1);
problem.x0=x0; 


problem.xinit = x_state;
problem.xfinal = model.xfinal;

% Time to solve the NLP
[output,exitflag,info] = MPC_straightline_solver(problem);


end