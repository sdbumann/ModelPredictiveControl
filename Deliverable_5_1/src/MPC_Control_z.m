classdef MPC_Control_z < MPC_Control
  properties
    A_bar, B_bar, C_bar % Augmented system for disturbance rejection    
    L                   % Estimator gain for disturbance rejection
  end
  
  methods
    function mpc = MPC_Control_z(sys, Ts, draw)
        if ~exist('draw', 'var') 
            draw = false;
        end
      mpc = mpc@MPC_Control(sys, Ts, draw);
      
      [mpc.A_bar, mpc.B_bar, mpc.C_bar, mpc.L] = mpc.setup_estimator();
    end
    
    % Design a YALMIP optimizer object that takes a steady-state state
    % and input (xs, us) and returns a control input
    function ctrl_opt = setup_controller(mpc, draw)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   x(:,1) - initial state (estimate)
      %   d_est  - disturbance estimate
      %   xs, us - steady-state target
      % OUTPUTS
      %   u(:,1) - input to apply to the system
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      [n,m] = size(mpc.B);
      
      % Steady-state targets (Ignore this before Todo 3.3)
      xs = sdpvar(n, 1);
      us = sdpvar(m, 1);
      
      % Disturbance estimate (Ignore this before Part 5)
      d_est = sdpvar(1);

      % SET THE HORIZON HERE
      N = 10;
      
      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);
      

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are 
      %       the DISCRETE-TIME MODEL of your system

      % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE

        epsil = 1e-11;  % get rid of " errorcode=4: numerical problems "
                        % we still retain theoretical precision of < 1 um, so we're probably okay 
                        
        Hu = [1; -1];
        ku = [0.3; 0.2];

        Q = eye(size(mpc.A));
        R = 20;

        sys = LTISystem('A', mpc.A, 'B', mpc.B);
        
        sys.u.min = -0.2;
        sys.u.max =  0.3;
        sys.x.penalty = QuadFunction(Q);
        sys.u.penalty = QuadFunction(R);

        Xf = sys.LQRSet;
        Ff = Xf.A;
        ff = Xf.b;
        Qf = sys.LQRPenalty.weight;

        %plot terminal set
        if draw
            figure
                Xf.plot();
                xlabel("vel z [m s^{-1}]"); ylabel("z [m]");
                title(sprintf("%s and %s for |Q| = %.2f and R = %.2f", 'vel z', 'z', max(max(Q)), R))
        end

        con = ((x(:,2)-xs) <= mpc.A*(x(:,1)-xs) + mpc.B*(u(:,1)-us) + epsil) + ((x(:,2)-xs) >= mpc.A*(x(:,1)-xs) + mpc.B*(u(:,1)-us) - epsil);
        con = con + (Hu*u(:,1) <= ku);
        obj = (u(:,1)'*R*u(:,1));
        for i = 2:N-1
            con = con + ((x(:,i+1)-xs) <= mpc.A*(x(:,i)-xs) + mpc.B*(u(:,i)-us) + epsil) + ((x(:,i+1)-xs) >= mpc.A*(x(:,i)-xs) + mpc.B*(u(:,i)-us) - epsil);
            con = con + (Hu * u(:,i) <= ku);        % the constraint on u is absolute 
            obj = obj + (x(:,i)-xs)'*Q*(x(:,i)-xs) + (u(:,i)'*R*u(:,i));
        end
%         con = con + (Ff*(x(:,N)-xs) <= ff);   % remove terminal constraint
        obj = obj + (x(:,N)-xs)'*Qf*(x(:,N)-xs); 
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      ctrl_opt = optimizer(con, obj, sdpsettings('solver','sedumi'), ...
        {x(:,1), xs, us, d_est}, u(:,1));
    end
    
    
    % Design a YALMIP optimizer object that takes a position reference
    % and returns a feasible steady-state state and input (xs, us)
    function target_opt = setup_steady_state_target(mpc)
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   ref    - reference to track
      %   d_est  - disturbance estimate
      % OUTPUTS
      %   xs, us - steady-state target
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % Steady-state targets
      n = size(mpc.A,1);
      xs = sdpvar(n, 1);
      us = sdpvar;
      
      % Reference position (Ignore this before Todo 3.2)
      ref = sdpvar;
            
      % Disturbance estimate (Ignore this before Part 5)
      d_est = sdpvar(1);
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
      
        epsil = 1e-11;   % get rid of " errorcode=4: numerical problems "
                         % we still retain theoretical precision of < 1 um, so we're probably okay 
      
        Hu = [1; -1];
        ku = [0.3; 0.2];
        
        con = (xs <= mpc.A*xs + mpc.B*(us+d_est) + epsil) + (xs >= mpc.A*xs + mpc.B*(us+d_est) - epsil);
        con = con + (ref <= mpc.C*xs + mpc.D + epsil) + (ref >= mpc.C*xs + mpc.D - epsil); % Cd is 0 --> we dont add it in this line
        con = con + (Hu*us <= ku);
        obj = (us'*us); % no cost for the position, as it is already constrained

      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'sedumi'), {ref, d_est}, {xs, us});
    end
    
    
    % Compute augmented system and estimator gain for input disturbance rejection
    function [A_bar, B_bar, C_bar, L] = setup_estimator(mpc)
      
      %%% Design the matrices A_bar, B_bar, L, and C_bar
      %%% so that the estimate x_bar_next [ x_hat; disturbance_hat ]
      %%% converges to the correct state and constant input disturbance
      %%%   x_bar_next = A_bar * x_bar + B_bar * u + L * (C_bar * x_bar - y);
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
      
      Bd = mpc.B;
      Cd = 0;
      
      A_bar = [mpc.A, Bd; zeros(1,2), eye(1)];
      B_bar = [mpc.B;  0];
      C_bar = [mpc.C, Cd];
      L_eigenvalues = [0.1; 0.2; 0.3];
      L = -place(A_bar', C_bar', L_eigenvalues')';
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end

    
  end
end
