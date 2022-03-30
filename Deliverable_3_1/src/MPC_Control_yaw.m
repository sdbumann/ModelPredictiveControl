classdef MPC_Control_yaw < MPC_Control
  
  methods
    % Design a YALMIP optimizer object that takes a steady-state state
    % and input (xs, us) and returns a control input
    function ctrl_opt = setup_controller(mpc, draw)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   x(:,1) - initial state (estimate)
      %   xs, us - steady-state target
      % OUTPUTS
      %   u(:,1) - input to apply to the system
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      [n,m] = size(mpc.B);
      
      % Steady-state targets (Ignore this before Todo 3.2)
      xs = sdpvar(n, 1);
      us = sdpvar(m, 1);
      
      % SET THE HORIZON HERE
      N = 10;
      
      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);
      

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are 
      %       the DISCRETE-TIME MODEL of your system

      % WRITE THE CONSTRAINTS AND OBJECTIVE HERE

        epsil = 1e-11;  % get rid of " errorcode=4: numerical problems "
                        % we still retain theoretical precision of < 1 um, so we're probably okay 
      
        Hu = [1; -1];
        ku = 0.2 * ones(2,1);

        Q = eye(size(mpc.A));
        R = 20;

        sys = LTISystem('A', mpc.A, 'B', mpc.B);
        
        sys.u.min = -0.2;
        sys.u.max =  0.2;
        sys.x.penalty = QuadFunction(Q);
        sys.u.penalty = QuadFunction(R);

        Xf = sys.LQRSet;
        Ff = Xf.A;
        ff = Xf.b;
        Qf = sys.LQRPenalty.weight;

        %plot terminal set
        if draw
            figure;
                Xf.plot();
                xlabel("vel yaw [rad s^{-1}]"); ylabel("yaw [rad]");
                title(sprintf("%s and %s for |Q| = %.2f and R = %.2f", 'vel yaw', 'yaw', max(max(Q)), R))
                saveas(gcf, "imgs/4_vel_yaw_and_yaw.png");
        end
        
        con = (x(:,2) <= mpc.A*x(:,1) + mpc.B*u(:,1) + epsil) + (x(:,2) >= mpc.A*x(:,1) + mpc.B*u(:,1) - epsil);
        con = con + (Hu*u(:,1) <= ku);
        obj = (u(:,1)'*R*u(:,1));
        for i = 2:N-1
            con = con + (x(:,i+1) <= mpc.A*x(:,i) + mpc.B*u(:,i) + epsil) + (x(:,i+1) >= mpc.A*x(:,i) + mpc.B*u(:,i) - epsil);
            con = con + (Hu * u(:,i) <= ku);        
            obj = obj + (x(:,i)'*Q*x(:,i)) + (u(:,i)'*R*u(:,i));
        end
        con = con + (Ff*x(:,N) <= ff);
        obj = obj + (x(:,N)'*Qf*x(:,N));
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      ctrl_opt = optimizer(con, obj, sdpsettings('solver','sedumi'), ...
        {x(:,1), xs, us}, u(:,1));
    end
    
    
    % Design a YALMIP optimizer object that takes a position reference
    % and returns a feasible steady-state state and input (xs, us)
    function target_opt = setup_steady_state_target(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   ref    - reference to track
      % OUTPUTS
      %   xs, us - steady-state target
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % Steady-state targets
      n = size(mpc.A,1);
      xs = sdpvar(n, 1);
      us = sdpvar;
      
      % Reference position (Ignore this before Todo 3.2)
      ref = sdpvar;            
            
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE       
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D

        con = [];
        obj = 0;     
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'sedumi'), ref, {xs, us});
      
    end
  end
end
