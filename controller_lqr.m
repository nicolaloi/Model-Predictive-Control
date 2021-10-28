% BRIEF:
%   Controller function template. Input and output dimension MUST NOT be
%   modified.
% INPUT:
%   Q: State weighting matrix, dimension (3,3)
%   R: Input weighting matrix, dimension (3,3)
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Heating and cooling power, dimension (3,1)

function p = controller_lqr(Q, R, T, ~, ~)
    % controller variables
    persistent param;

    % initialize controller, if not done already
    if isempty(param)
        param = init(Q, R);
    end

    % compute control action    
    p = -param.K*(T-param.T_sp) + param.p_sp; % working in delta formulation

end

function param = init(Q, R)
    % get basic controller parameters
    param = compute_controller_base_parameters;
    param.K = dlqr(param.A, param.B, Q, R);

end