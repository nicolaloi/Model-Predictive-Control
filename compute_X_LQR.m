% BRRIEF:
%   Template for explicit invariant set computation. You MUST NOT change
%   the output.
% INPUT:
%   Q, R: State and input weighting matrix, dimension (3,3)
% OUTPUT:
%   A_x, b_x: Describes polytopic X_LQR = {x| A_x * x <= b_x}

function [A_x, b_x] = compute_X_LQR(Q, R)
    
    param = compute_controller_base_parameters;
    
    K = dlqr(param.A, param.B, Q, R);
      
    system = LTISystem('A', param.A - param.B*K);

    % compute invariant set
    
    %https://www.mpt3.org/UI/Filters
    poly = Polyhedron([-K; K; eye(3); -eye(3)], [param.Ucons(:,2); -param.Ucons(:,1); param.Xcons(:,2); -param.Xcons(:,1)]);
    system.x.with('setConstraint');
    system.x.setConstraint = poly;    
    InvSet = system.invariantSet();

    plot = false;

    if plot
        figure(9)
        init1 = plot3(-2.25, 1.75, 0.75,'g.'); hold on
        set(init1,'MarkerSize',40);
        init2 = plot3(1.5, 2.75, -0.25,'b.'); hold on
        set(init2,'MarkerSize',40);
        InvSet.plot(); 

        hold off
    end
    
  
    % set matrices
    A_x = InvSet.A;
    b_x = InvSet.b;
    
    
end 