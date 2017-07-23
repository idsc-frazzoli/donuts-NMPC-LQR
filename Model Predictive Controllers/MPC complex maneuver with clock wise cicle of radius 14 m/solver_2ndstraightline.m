%% Call solver

function  [output,exitflag,info] = solver_2ndstraightline(x_state)

%%

model.N = 15;            % horizon length
model.xfinal = [ 0 , -28 ]'; 



%%

% Set initial guess to start solver from:
x0i=[ deg2rad(0.5) ,1000 , 8.17,  deg2rad(0),  deg2rad(0), +deg2rad( - 360 * 3 ), 0 , -28 ];
x0=repmat(x0i',model.N,1);
problem.x0=x0; 



problem.xinit = x_state;
problem.xfinal = model.xfinal;

% solve the NLP
[output,exitflag,info] = MPC_2ndstraightline_solver(problem);


end