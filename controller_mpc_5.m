% BRIEF:
%   Controller function template. Input and output dimension MUST NOT be
%   modified.
% INPUT:
%   Q: State weighting matrix, dimension (3,3)
%   R: Input weighting matrix, dimension (3,3)
%   T: Measured system temperatures, dimension (3,1)
%   N: MPC horizon length, dimension (1,1)
%   d: Disturbance matrix, dimension (3,N)
% OUTPUT:
%   p: Heating and cooling power, dimension (3,1)

function p = controller_mpc_5(Q, R, T, N, d)
    % controller variables
    persistent param yalmip_optimizer
    
    % initialize controller, if not done already
    if isempty(param)
        [param, yalmip_optimizer] = init(Q, R, N);
    end

% evaluate control action by solving MPC problem
[u_mpc,errorcode] = yalmip_optimizer({T-param.T_sp,d(:,1:N-1)});
if (errorcode ~= 0)
    warning('MPC5 infeasible');
end
p = u_mpc + param.p_sp;
end

function [param, yalmip_optimizer] = init(Q, R, N)
% get basic controller parameters
param = compute_controller_base_parameters;

% get terminal cost
[~, P] = dlqr(param.A, param.B, Q, R);

% get terminal set
clear compute_X_LQR
[A_x, b_x] = compute_X_LQR(Q, R);
%cla

S = diag([100, 10000, 10000]);

% implement your MPC using Yalmip here
nx = size(param.A,1);
nu = size(param.B,2);
U = sdpvar(repmat(nu,1,N-1),ones(1,N-1),'full');
X = sdpvar(repmat(nx,1,N),ones(1,N),'full');
eps = sdpvar(repmat(nx,1,N),ones(1,N),'full');
v = sdpvar(1,1,'full');
T0 = sdpvar(nx,1,'full');
d = sdpvar(repmat(nx,1,N-1),ones(1,N-1),'full');
objective = 0;
constraints = [];

% initial condition 
constraints = [constraints, X{1} == T0];

for k = 1:N-1
    constraints = [constraints, X{k+1} == param.A*X{k} + param.B*U{k} + d{k}]; % dynamics
    constraints = [constraints, X{k+1} >= param.Xcons(:,1) - eps{k}];
    constraints = [constraints, X{k+1} <= param.Xcons(:,2) + eps{k}];
    constraints = [constraints, eps{k} >= zeros(nx,1)];
    constraints = [constraints, U{k} >= param.Ucons(:,1)];
    constraints = [constraints, U{k} <= param.Ucons(:,2)];
    objective = objective +  X{k}'*Q*X{k} + U{k}'*R*U{k};
    objective = objective + v*norm(eps{k},1) + eps{k}'*S*eps{k};
end
constraints = [constraints, A_x * X{end} <= b_x];
constraints = [constraints, eps{end} >= zeros(nx,1), v == 10000];
objective = objective + X{end}'*P*X{end} + v*norm(eps{end},1) + eps{end}'*S*eps{end}; % terminal cost

ops = sdpsettings('verbose',0,'solver','quadprog');

input = {T0, [d{:}]};
yalmip_optimizer = optimizer(constraints,objective,ops,input,U{1});

end