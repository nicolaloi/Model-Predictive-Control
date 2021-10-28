% BRRIEF:
%   Template for tuning of Q and R matrices for LQR controller as described 
%   in task 6.
% INPUT:
%   n_samples:  Number of samples considered for the tuning.
%   T0:         Initial condition
%   T_sp:       Set point
%   scen:       Disturbance scenario
% OUTPUT:
%   Q, R: Describes stage cost of LQR controller (x^T Q x + u^T R u)

function [Q, R] = heuristic_LQR_tuning(n_samples, T0, T_sp, scen)

    figure(6); %set(gcf, 'WindowStyle' ,'docked'); 
    grid on; hold on
    xlabel('Energy consumption [kWh]'); 
    ylabel('Relative norm of steady state deviation');
    R = eye(3);

    dT_old = Inf;
    power_good = 0;

    fprintf('\nStep: ')
    for index = 1:n_samples

        Q_temp = diag([randi([1 10e6]), randi([1 10e6]), randi([1 10e6])]);

        clear controller_lqr
        [T, p, ~, ~, T_v, p_v] = simulate_building(T0, @controller_lqr, Q_temp, R, scen, 0);
        dT_relative = norm(T_sp - T(:,15)) / norm(T_sp - T0);
        power_sum = sum(abs(p),'all')/1000/60;

        % check if best 
        if T_v == 0 % && p_v == 0
            if (dT_relative < dT_old && power_sum <= 16)
                Q = Q_temp;
                dT_old = dT_relative;  % store best up to now
                power_good = power_sum;
            end
        end

        % plot
        if T_v
            color = 'r';
        elseif p_v
            color = 'b';
        else
            color = 'g';
        end

        scatter(power_sum, dT_relative,[], color)

        % print loading bar
        if index>1
            for j=0:(1+log10(n_samples-1))
                fprintf('\b');
            end
            for j=0:log10(index-1)
                fprintf('\b');
            end
        end
        fprintf('%d/%d', index, n_samples);

    end

    circle_best = plot(power_good, dT_old, 'ko');  % mark chosen one (too small)
    set(circle_best,'MarkerSize',25);
    cross_best = plot(power_good, dT_old, 'k*');  % mark chosen one (too small)
    set(cross_best,'MarkerSize',25);
    hold off

end
