% BRIEF:
%   Controller function template. Input and output dimension MUST NOT be
%   modified.
% INPUT:
%   Q: State weighting matrix, dimension (3,3)
%   R: Input weighting matrix, dimension (3,3)
%   T: Measured system temperatures, dimension (3,1)
%   N: MPC horizon length, dimension (1,1)
% OUTPUT:
%   p: Heating and cooling power, dimension (3,1)

function p = controller_mpc_2(Q, R, T, N, ~)
%N = 30;
% controller variables
persistent param yalmip_optimizer

% initialize controller, if not done already
    if isempty(param)
        [param, yalmip_optimizer] = init(Q, R, N);
    end

% evaluate control action by solving MPC problem
    [u_mpc,errorcode] = yalmip_optimizer(T-param.T_sp);
    if (errorcode ~= 0)
        warning('MPC2 infeasible');
    end
    p = u_mpc + param.p_sp;
end

function [param, yalmip_optimizer] = init(Q, R, N)

% get basic controller parameters
    param = compute_controller_base_parameters;

% implement your MPC using Yalmip here
    %https://www.mpt3.org/Main/CustomMPC
    nx = size(param.A,1);
    nu = size(param.B,2);
    U = sdpvar(repmat(nu,1,N-1),ones(1,N-1),'full');
    X = sdpvar(repmat(nx,1,N),ones(1,N),'full');
    T0 = sdpvar(nx,1,'full');
    
    objective = 0; 
    constraints = [ X{1} == T0];
    
    for k = 1:N-1
        % assing domain
        constraints = [constraints; X{k+1} == param.A*X{k} + param.B*U{k} ];

        % input constraints
        constraints = [constraints; param.Ucons(:,1) <= U{k} <= param.Ucons(:,2) ];

        % state constraints
        constraints = [constraints; param.Xcons(:,1) <= X{k+1} <= param.Xcons(:,2)];
        
        % objective (updating the cost function bu summing up stage costs)
        objective = objective +  X{k}'*Q*X{k} + U{k}'*R*U{k};
    end
    
    % terminal set
    constraints = [constraints, X{end}==zeros(3,1)];

    objective = objective;
    ops = sdpsettings('verbose',0,'solver','quadprog');
    yalmip_optimizer = optimizer(constraints,objective,ops,T0,U{1});

end