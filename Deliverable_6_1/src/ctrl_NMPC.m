function [ctrl, traj] = ctrl_NMPC(quad)
%%
    import casadi.*
    
    opti = casadi.Opti(); % Optimization problem
    
    N = 10; % MPC horizon [SET THIS VARIABLE]
    
    % −−−− decision variables −−−−−−−−−
    X = opti.variable(12,N+1); % state trajectory variables
    U = opti.variable(4, N);   % control trajectory
    
    X0 = opti.parameter(12,1); % initial state
    REF = opti.parameter(4,1); % reference position [x,y,z,yaw]
    
    %%%%%%%%%%%%%%%%%%%%%%%%
    %%%% YOUR CODE HERE %%%%
    %%%%%%%%%%%%%%%%%%%%%%%%
    
    % observation matrix
    ref_idx = [10, 11, 12, 6]; %[x,y,z,yaw]
    
    % discretize quad.f 
    f_discrete = @(x,u) RK4(x,u,quad);
%     f_discrete = @(x,u) Euler(x,u,quad);
    
    % optimization variable: you can only call opti.minimize() once!!!!!!!!
    minimization = 0;

    % weights 
    Q  = speye(12); 
    Q(12,12) = 200;                             % extra weight on z tracking 
    Qf = 200*Q;
    R  = 1;
    Rs = 200*R;

    % find steady-state
    Xs = opti.variable(12,1);
    Us = opti.variable(4,1);
    opti.subject_to(Xs == f_discrete(Xs, Us));  % steady-state constraint
    opti.subject_to(Xs(ref_idx) == REF);        % reference tracking constraint
    opti.subject_to(0 <= Us <= 1.5);            % input constraint
    minimization = minimization + Us'*Rs*Us;    % optimization of ss

    % optimization problem
    opti.subject_to(0 <= U <= 1.5);             % input constraints
    opti.subject_to(X(:,1) == X0);              % initial state
    for k=1:N
        opti.subject_to(X(:,k+1) == f_discrete(X(:,k), U(:,k)));
        minimization = minimization + (X(:,k+1)-Xs)'*Q*(X(:,k+1)-Xs) + (U(:,k)-Us)'*R*(U(:,k)-Us);
    end
    minimization = minimization + (X(:,end)-Xs)'*Qf*(X(:,end)-Xs);
    opti.minimize( minimization );              % minimization
    

    %%%%%%%%%%%%%%%%%%%%%%%%
    %%%% YOUR CODE HERE %%%%
    %%%%%%%%%%%%%%%%%%%%%%%%   
    
    ctrl = @(x,ref) eval_ctrl(x, ref, opti, X0, REF, X, U);
end
