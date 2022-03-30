classdef MPC_Control_y < MPC_Control
  
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
        ku = 0.3 * ones(2,1);
        Hx = [0 1 0 0; 0 -1 0 0];
        kx = 0.035 * ones(2,1);

        Q = eye(size(mpc.A));
        R = 20;

        sys = LTISystem('A', mpc.A, 'B', mpc.B);
        
        sys.x.min = -[Inf; 0.035; Inf; Inf];
        sys.x.max =  [Inf; 0.035; Inf; Inf];
        sys.u.min = -0.3;
        sys.u.max =  0.3;
        sys.x.penalty = QuadFunction(Q);
        sys.u.penalty = QuadFunction(R);

        Xf = sys.LQRSet;
        Ff = Xf.A;
        ff = Xf.b;
        Qf = sys.LQRPenalty.weight;

        %plot terminal set
        if draw 
            f1 = figure;
                hold on;
                Xf.projection(1:2).plot(); 
                plot([-1.5, 1.5],  [0.035, 0.035], '--b');
                plot([-1.5, 1.5], -[0.035, 0.035], '--b');
                legend("terminal invariant set", "roll constraints" );
                xlabel("vel roll [rad s^{-1}]"); ylabel("roll [rad]");
                title(sprintf("%s and %s for |Q| = %.2f and R = %.2f", 'vel roll', 'roll', max(max(Q)), R))
                hold off;
                saveas(f1, "imgs/2_vel_roll_and_roll.png");
            f2 = figure;
                hold on;
                Xf.projection(2:3).plot();
                plot( [0.035, 0.035], [-1.5, 1.5], '--b');
                plot(-[0.035, 0.035], [-1.5, 1.5], '--b');
                legend("terminal invariant set", "roll constraints" );
                xlabel("roll [rad]"); ylabel("vel y [m s^{-1}]");
                title(sprintf("%s and %s for |Q| = %.2f and R = %.2f", 'roll', 'vel y', max(max(Q)), R))
                hold off;
                saveas(f2, "imgs/2_roll_and_vel_y.png");
            f3 = figure;
                Xf.projection(3:4).plot();
                hold off;
                xlabel("vel y [m s^{-1}]"); ylabel("y [m]");
                title(sprintf("%s and %s for |Q| = %.2f and R = %.2f", 'vel y', 'y', max(max(Q)), R))
                saveas(f3, "imgs/2_vel_y_and_y.png");
        end 

        con = (x(:,2) <= mpc.A*x(:,1) + mpc.B*u(:,1) + epsil) + (x(:,2) >= mpc.A*x(:,1) + mpc.B*u(:,1) - epsil);
        con = con + (Hu*u(:,1) <= ku);
        obj = (u(:,1)'*R*u(:,1));
        for i = 2:N-1
            con = con + (x(:,i+1) <= mpc.A*x(:,i) + mpc.B*u(:,i) + epsil) + (x(:,i+1) >= mpc.A*x(:,i) + mpc.B*u(:,i) - epsil);
            con = con + (Hx * x(:,i+1) <= kx);      
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
