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

function p = controller_mpc_6(Q,R,T,N,~)
% controller variables
    persistent param yalmip_optimizer aug
    
    % initialize controller, if not done already
    if isempty(param)
        [param, aug, yalmip_optimizer] = init(Q,R,T,N);  % additional aug structure to keep data
    end
    
    % compute T_est and d_est each loop before calling the optimizer
    % steady state is different, need to use it and not the normal one
    % compute steady state each time
    % for first iteration done in init
    
       
    % evaluate control action by solving MPC problem
    [u_mpc,errorcode] = yalmip_optimizer({param.T_est- aug.T_sp, param.d_est, aug.T_sp, aug.p_sp});  % then subtract Ts previously computed
    if (errorcode ~= 0)
        warning('MPC6 infeasible');
    end
    p = u_mpc + aug.p_sp;  

    % observer update
    S1 = [param.T_est ; param.d_est];
    y = aug.C*[T;param.d] + aug.D*p;
    
    S2 = aug.A*S1 + aug.B*p + aug.L*(y - aug.C*S1);
    param.T_est = S2(1:3);
    param.d_est = S2(4:6);
    

    % set point update
    H = param.C_ref;
    r = param.b_ref;
    %C = eye(3);
    
    mat1 = [param.A - eye(3), param.B;
         H*param.C,  zeros(3)];
     
    mat2 = [-param.Bd * param.d_est;
         r-H*zeros(3)*param.d_est];
    
    x_sp = mat1\mat2;
    aug.T_sp = x_sp(1:3);
    aug.p_sp = x_sp(4:6);
    
end

function [param, aug, yalmip_optimizer] = init(Q,R,T,N)

    % get basic controller parameters
    param = compute_controller_base_parameters;
    
    param.C = eye(3);
    
    
    % get terminal cost
    [ ~, P] = dlqr(param.A , param.B , Q , R);
    
    % get terminal set
    clear compute_X_LQR
    [A_x, b_x] = compute_X_LQR(Q, R);
    
    % design disturbance observer
    aug.A = [param.A, param.Bd;
             zeros(3), eye(3)];
    aug.B = [param.B;
             zeros(3)];
    aug.C = [eye(3), zeros(3)];
    aug.D = 0;
    
    
    %q = [-0.3-0.1i, -0.3+0.1i, -0.4, +0.2, +0.1+0.1i, +0.1-0.1i];
    %q = [0.1+0.1i,0.1-0.1i, -0.2, -0.1+0.1i, -0.1-0.1i, -0.3];
    %q = [0.3, 0.4, 0.5, 0.4+0.2i, 0.4-0.2i, 0.45];
    q = [0.4, 0.5, 0.6, 0.5+0.2i, 0.5-0.2i, 0.55];  % best combination to reduce input scattering
    %q = [0.001, 0.0008, 0.0005, -0.001, -0.0008, -0.0005];
    
    aug.L = place(aug.A',aug.C',q).';
    disp("Error dynamics poles: ")
    disp(eig(aug.A-aug.L*aug.C))
    
    
    % init state and disturbance estimate variables
    
    param.T_est = T;  
    param.d_est = param.d;
        
    H = param.C_ref;
    r = param.b_ref;
    
    mat1 = [param.A - eye(3), param.B;
         H*param.C,  zeros(3)];        % C doesn't exist
     
    mat2 = [-param.Bd * param.d_est;
         r-H*zeros(3)*param.d_est];
    
    x_sp = mat1\mat2;
    aug.T_sp = x_sp(1:3);
    aug.p_sp = x_sp(4:6);
    

    % implement your MPC using Yalmip here
    nx = size(param.A,1);
    nu = size(param.B,2);
    U = sdpvar(repmat(nu,1,N-1),ones(1,N-1),'full');
    X = sdpvar(repmat(nx,1,N),ones(1,N),'full');
    D = sdpvar(repmat(nu,1,N),ones(1,N),'full');
    T0 = sdpvar(nx,1,'full');
    d = sdpvar(nu,1,'full');
    T_sp = sdpvar(nx,1,'full');
    p_sp = sdpvar(nu,1,'full');
    
    constraints = [X{1} == T0];  
    constraints = [constraints, D{1} == d];
    objective = 0;
    for k = 1:N-1
        constraints = [constraints, X{k+1} == param.A*X{k} + param.B*U{k} + 0*param.Bd*D{k}]; % don't need to consider the constant disturbance in delta formulation 
        constraints = [constraints, D{k+1} == D{k}]; % constant disturbace time invariant
        constraints = [constraints, param.Tcons(:,1)- T_sp <= X{k+1} <= param.Tcons(:,2)- T_sp];
        constraints = [constraints, param.Pcons(:,1)- p_sp <= U{k} <= param.Pcons(:,2)- p_sp];
        
        objective = objective + X{k}'*Q*X{k} + U{k}'*R*U{k};
    end
    constraints = [constraints,  A_x * X{end} <= b_x];
    objective = objective + X{end}'*P*X{end};
    ops = sdpsettings('verbose',0,'solver','quadprog');
    yalmip_optimizer = optimizer(constraints,objective,ops,{T0, d, T_sp, p_sp},U{1});
end