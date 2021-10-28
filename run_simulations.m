%% Init
clear all
close all
addpath(genpath(cd));
rng(1234)
load('system/parameters_scenarios.mat')
param = compute_controller_base_parameters;

dT0_1 = [-2.25; 1.75; 0.75];
dT0_2 = [1.5; 2.75; -0.25];
T_sp = param.T_sp;
T0_1 = T_sp + dT0_1; 
T0_2 = T_sp + dT0_2;


N = 30;

%problem n. 1   2  3  4  5  6  7
problem = [NaN, 1, 1, 1, 1, 1, 1];


%% Example
% figure; set(gcf, 'WindowStyle' ,'docked');
% clear persisten variables of function controller_example
% clear controller_example
% execute simulation
% [T,~,~,t] = simulate_building(T0_example,@controller_example);


%% Unconstrained optimal control

if problem(2)
    fprintf('\n2) Unconstraint optimal control\n');

    % Uncontrolled system
    warning('off','all')
    fprintf("\n- Simulation uncontrolled temperature\n\n")
    figure(5); %set(gcf, 'WindowStyle' ,'docked');
    [T,~,~,t] = simulate_building(T0_1);
    warning('on','all')
    %a = 4;

    %Tuning of LQR on first initial condition
    fprintf("\n- Heuristic LQR Tuning\n")
    n_samples = 2500;
    [Q,R] = heuristic_LQR_tuning(n_samples, T0_1, T_sp, scen1);
    clear controller_lqr;
    
    fprintf("\n\n- Simulating T0_1 with LQR\n")
    figure(7); %set(gcf, 'WindowStyle' ,'docked');   

    [T1, p1] = simulate_building(T0_1, @controller_lqr, Q, R, scen1, 1);   
    %dT_relative = norm(T_sp - T(:,15)) / norm(T_sp - T_01);

    % Tuning LQR on second initial condition
    %[Q2,R] = heuristic_LQR_tuning(n_samples, T0_2, T_sp, scen1);
    %clear controller_lqr;

    fprintf("\n- Simulating T0_2 with LQR\n")
    figure(8); %set(gcf, 'WindowStyle' ,'docked');

    clear controller_lqr;
    [T, p] = simulate_building(T0_2, @controller_lqr, Q, R, scen1, 1);

    fprintf('\nSystem paused, press key to continue ...\n')
    pause;
else
    
    R = eye(3);
    Q = diag([4973679 , 5427908, 4949349]); %best state constraint Q 

    
end

%% From LQR to MPC
if problem(3)
    fprintf('\n3) First MPC\n'); 

    param.K = dlqr(param.A, param.B, Q, R);
    
    % task 9
    fprintf("\n- Computing invariant set\n")
    
    [A_x, b_x] = compute_X_LQR(Q,R);
    
    % task 10-11
    N=30;
    
    fprintf("\n- Simulating T0_1 with simple MPC\n")

    figure(111); %set(gcf, 'WindowStyle' ,'docked');
    clear controller_mpc_1;
    [T2, p2, J, ~, ~, ~] = simulate_building(T0_1, @controller_mpc_1, Q, R, scen1,1,N);
    
    fprintf("\n- Simulating T0_2 with simple MPC\n")
    figure(112); %set(gcf, 'WindowStyle' ,'docked');
    clear controller_mpc_1;
    simulate_building(T0_2, @controller_mpc_1, Q, R, scen1,1,N);


    fprintf('\nSystem paused, press key to continue ...\n')
    pause;
end


%% MPC with guarantees
if problem(4)
    fprintf('\n4) MPC with guarantees\n');
    
    %Defining a Matrices for the steady state values (for the trajectories)
    T_steady = zeros(3,61);
    for i = 1:1:61
        T_steady(: , i) = param.T_sp;
    end
    
    p_steady = zeros(3,60);
    for i = 1:1:60
        p_steady(: , i) = param.p_sp;
    end
    
    
    % Missing components for the cost J(T - T_sp)
    % Computing the terminal cost associated to the LQR controller
    [K , P] = dlqr(param.A , param.B , Q , R);
    
    
    %SIMULATION USING MPC2
    fprintf('Task 13\n');
    N=30;
    
    fprintf("\n- Simulating T0_1 with MPC2\n")
    figure(131); %set(gcf, 'WindowStyle' ,'docked');
    clear controller_mpc_2;
    [T_traj1, p_traj1, J_MPC2_T_01, ~, ~, ~] = simulate_building(T0_1, @controller_mpc_2, Q, R, scen1,1,N);
    X_traj1 = T_traj1 - T_steady;
    U_traj1 = p_traj1 - p_steady;
    
    fprintf("\n- Simulating T0_2 with MPC2\n")
    figure(132); %set(gcf, 'WindowStyle' ,'docked');
    clear controller_mpc_2;
    [T_traj2, p_traj2, J_MPC2_T_02, ~, ~, ~] = simulate_building(T0_2, @controller_mpc_2, Q, R, scen1,1,N);
    X_traj2 = T_traj2 - T_steady;
    U_traj2 = p_traj2 - p_steady;

    
    
    %SIMULATION USING MPC3
    fprintf('Task 14\n');
    N=30;

    
    fprintf("\n- Simulating T0_1 with MPC3\n")
    figure(141); %set(gcf, 'WindowStyle' ,'docked');
    clear controller_mpc_3;
    [T_traj3, p_traj3, J_MPC3_T_01, ~, ~, ~] = simulate_building(T0_1, @controller_mpc_3, Q, R, scen1,1,N);
    X_traj3 = T_traj3 - T_steady;
    U_traj3 = p_traj3 - p_steady;
    
    
    fprintf("\n- Simulating T0_2 with MPC3\n")
    figure(142); %set(gcf, 'WindowStyle' ,'docked');
    clear controller_mpc_3;
    [T_traj4, p_traj4, J_MPC3_T_02, ~, ~, ~] = simulate_building(T0_2, @controller_mpc_3, Q, R, scen1,1,N);
    X_traj4 = T_traj4 - T_steady;
    U_traj4 = p_traj4 - p_steady;
        
    
   
    fprintf('Task 15\n');
    % Simulate Building to obtain trajectories for cost input and states
    
    %Simulation MPC1 for T_01
    figure(151); %set(gcf, 'WindowStyle' ,'docked');
    clear controller_mpc_1;
    [T_traj5, p_traj5, J_MPC1_T_01, ~, ~, ~] = simulate_building(T0_1, @controller_mpc_1, Q, R, scen1,1,N);
    X_traj5 = T_traj5 - T_steady;
    U_traj5 = p_traj5 - p_steady;
    
    
    
    %Simulation MPC1 for T_02
    figure(152); %set(gcf, 'WindowStyle' ,'docked');
    clear controller_mpc_1;
    [T_traj6, p_traj6, J_MPC1_T_02, ~, ~, ~] = simulate_building(T0_2, @controller_mpc_1, Q, R, scen1,1,N);
    X_traj6 = T_traj6 - T_steady;
    U_traj6 = p_traj6 - p_steady;
    
    
    % PLOT TRAJECTORIES
    
    % compute invariant set
    K = dlqr(param.A, param.B, Q, R);     
    system = LTISystem('A', param.A - param.B*K);
    poly = Polyhedron([-K; K; eye(3); -eye(3)], [param.Ucons(:,2); -param.Ucons(:,1); param.Xcons(:,2); -param.Xcons(:,1)]);
    system.x.with('setConstraint');
    system.x.setConstraint = poly;    
    InvSet = system.invariantSet();
    
    figure(1500); %set(gcf, 'WindowStyle' ,'docked'); 
    hold on;
    fprintf("\n- Plotting trajectories of the three MPC Controllers with T0_1 as initial condition");
    plot3(X_traj5(1,:) , X_traj5(2,:) , X_traj5(3,:) , 'b- .','LineWidth',8);
    plot3(X_traj1(1,:) , X_traj1(2,:) , X_traj1(3,:) , 'k- .','LineWidth',8); 
    plot3(X_traj3(1,:) , X_traj3(2,:) , X_traj3(3,:) , 'g- .','LineWidth',8);   
    %InvSet.plot()
    
    title('State Trajectories');
    legend('Controller MPC1 starting from T01','Controller MPC2 starting from T01','Controller MPC3 starting from T01','Location','SouthEastOutside');
    axis([-Inf +Inf -Inf +Inf -Inf +Inf]);
    grid on;
    xlabel('x_1');
    ylabel('x_2');
    zlabel('x_3');
    hold off;
    
    fprintf("\n- Plotting trajectories of the three MPC Controllers with T0_2 as initial condition\n");
    figure(1501); %set(gcf, 'WindowStyle' ,'docked'); 
    hold on;
    plot3(X_traj6(1,:) , X_traj6(2,:) , X_traj6(3,:) , 'b- .','LineWidth',8);
    plot3(X_traj2(1,:) , X_traj2(2,:) , X_traj2(3,:) , 'k- .','LineWidth',8); 
    plot3(X_traj4(1,:) , X_traj4(2,:) , X_traj4(3,:) , 'g- .','LineWidth',8);
    %InvSet.plot()
 
    title('State Trajectories');
    legend('Controller MPC1 starting from T02','Controller MPC2 starting from T02','Controller MPC3 starting from T02','Location','SouthEastOutside');
    axis([-Inf +Inf -Inf +Inf -Inf +Inf]);
    grid on;
    xlabel('x_1');
    ylabel('x_2');
    zlabel('x_3');
    hold off;

% COMPARISON OF TRAJECTORIES OF MPC1 AND MPC3

%difference when initial condition is T_01
difference_T_01 = zeros(3,61);
for i=1:61
    difference_T_01(:,i) = T_traj3(:,i) - T_traj5(:,i);
end 
difference_T_01;

%difference when initial condition is T_01
difference_T_02 = zeros(3,61);
for i=1:61
    difference_T_02(:,i) = T_traj4(:,i) - T_traj6(:,i);
end 
difference_T_02;

%     figure(1502); set(gcf, 'WindowStyle' ,'docked'); 
%     hold on;
%     [T_traj3, p_traj3, J_MPC3_T_01, ~, ~, ~] = simulate_building(T0_1, @controller_mpc_3, Q, R, scen1,1,N);
%     [T_traj5, p_traj5, J_MPC1_T_01, ~, ~, ~] = simulate_building(T0_1, @controller_mpc_1, Q, R, scen1,1,N);
%     [T_traj4, p_traj4, J_MPC3_T_02, ~, ~, ~] = simulate_building(T0_2, @controller_mpc_1, Q, R, scen1,1,N);
%     [T_traj6, p_traj6, J_MPC1_T_02, ~, ~, ~] = simulate_building(T0_2, @controller_mpc_1, Q, R, scen1,1,N);
%     hold off;
  
%     %INPUT TRAJECTORIES
%     fprintf('Plotting Input Trajectories');
%     
%     %Trajectory Input Sequence from MPC1
%     figure(401); hold on;
%     stairs(1:1:60, U_traj1(1 , :));
%     stairs(1:1:60, U_traj1(2 , :));
%     stairs(1:1:60, U_traj1(3 , :));
%     title('Closed-loop input trajectory for MPC1 from T01');
%     legend('Control input for Vaccination Center','Control input for Fridge 1','Control input for Fridge 2' , 'Location' , 'SouthEastOutside')
%     grid on;
%     xlabel('time k');
%     ylabel('u(k)');
%     hold off;
%     
%     %Trajectory Input Sequence from MPC1
%     figure(402); hold on;
%     stairs(1:1:60, U_traj4(1 , :));
%     stairs(1:1:60, U_traj4(2 , :));
%     stairs(1:1:60, U_traj4(3 , :));
%     title('Closed-loop input trajectory for MPC1 from T02');
%     legend('Control input for Vaccination Center','Control input for Fridge 1','Control input for Fridge 2' , 'Location' , 'SouthEastOutside')
%     grid on;
%     xlabel('time k');
%     ylabel('u(k)');
%     hold off;
%     
%     %Trajectory Input Sequence from MPC2
%     figure(403); hold on;
%     stairs(1:1:60, U_traj2(1 , :));
%     stairs(1:1:60, U_traj2(2 , :));
%     stairs(1:1:60, U_traj2(3 , :));
%     title('Closed-loop input trajectory for MPC2 from T01');
%     legend('Control input for Vaccination Center','Control input for Fridge 1','Control input for Fridge 2' , 'Location' , 'SouthEastOutside')
%     grid on;
%     xlabel('time k');
%     ylabel('u(k)');
%     hold off;
%     
%     
%     %Trajectory Input Sequence from MPC2
%     figure(404); hold on;
%     stairs(1:1:60, U_traj5(1 , :));
%     stairs(1:1:60, U_traj5(2 , :));
%     stairs(1:1:60, U_traj5(3 , :));
%     title('Closed-loop input trajectory for MPC2 from T02');
%     legend('Control input for Vaccination Center','Control input for Fridge 1','Control input for Fridge 2' , 'Location' , 'SouthEastOutside')
%     grid on;
%     xlabel('time k');
%     ylabel('u(k)');
%     hold off;
%     
%     %Trajectory Input Sequence from MPC3
%     figure(405); hold on;
%     stairs(1:1:60, U_traj3(1 , :));
%     stairs(1:1:60, U_traj3(2 , :));
%     stairs(1:1:60, U_traj3(3 , :));
%     title('Closed-loop input trajectory for MPC3 from T01');
%     legend('Control input for Vaccination Center','Control input for Fridge 1','Control input for Fridge 2' , 'Location' , 'SouthEastOutside')
%     grid on;
%     xlabel('time k');
%     ylabel('u(k)');
%     hold off;
%     
%     %Trajectory Input Sequence from MPC3
%     figure(406); hold on;
%     stairs(1:1:60, U_traj6(1 , :));
%     stairs(1:1:60, U_traj6(2 , :));
%     stairs(1:1:60, U_traj6(3 , :));
%     title('Closed-loop input trajectory for MPC3 from T02');
%     legend('Control input for Vaccination Center','Control input for Fridge 1','Control input for Fridge 2' , 'Location' , 'SouthEastOutside')
%     grid on;
%     xlabel('time k');
%     ylabel('u(k)');
%     hold off;
    
%% 
    test_cost = true;
    if test_cost
    %CALCULATE J
    
    N = 30;
    
    
    [T_traj1, p_traj1, J_MPC1_T_01, ~, ~, ~] = simulate_building(T0_1, @controller_mpc_1, Q, R, scen1,0,N);
    [T_traj2, p_traj2, J_MPC2_T_01, ~, ~, ~] = simulate_building(T0_1, @controller_mpc_2, Q, R, scen1,0,N);
    [T_traj3, p_traj3, J_MPC3_T_01, ~, ~, ~] = simulate_building(T0_1, @controller_mpc_3, Q, R, scen1,0,N);
    T_steady = zeros(3,61);
    
    for i = 1:1:61
        T_steady(: , i) = param.T_sp;
    end
    
    p_steady = zeros(3,60);
    for i = 1:1:60
        p_steady(: , i) = param.p_sp;
    end
    
    X_traj3 = T_traj3 - T_steady;
    U_traj3 = p_traj3 - p_steady;
    X_traj2 = T_traj2 - T_steady;
    U_traj2 = p_traj2 - p_steady;
    X_traj1 = T_traj1 - T_steady;
    U_traj1 = p_traj1 - p_steady;

% OPTIMAL COST TO GO FOR T_01
    JMPC3_x_T_01 = 0;
    JMPC3_p_T_01 = 0;
    JMPC3_u_T_01 = 0;
    JMPC2_x_T_01 = 0;
    JMPC2_p_T_01 = 0;
    JMPC2_u_T_01 = 0;
    JMPC1_x_T_01 = 0;
    JMPC1_p_T_01 = 0;
    JMPC1_u_T_01 = 0;
    
    figure(666);
    hold on;
    for i= 1:60
        JMPC3_x_T_01 = JMPC3_x_T_01 + X_traj3(:,i+1)'*Q*X_traj3(:,i+1);
        JMPC3_p_T_01 = JMPC3_p_T_01 + p_traj3(:,i)'*R*p_traj3(:,i);
        JMPC3_u_T_01 = JMPC3_u_T_01 + U_traj3(:,i)'*R*U_traj3(:,i);
        plot(i, X_traj3(:,i+1)'*Q*X_traj3(:,i+1), 'r*' , i, U_traj3(:,i)'*R*U_traj3(:,i) , 'ro');
        JMPC2_x_T_01 = JMPC2_x_T_01 + X_traj2(:,i+1)'*Q*X_traj2(:,i+1);
        JMPC2_p_T_01 = JMPC2_p_T_01 + p_traj2(:,i)'*R*p_traj2(:,i);
        JMPC2_u_T_01 = JMPC2_u_T_01 + U_traj2(:,i)'*R*U_traj2(:,i);
        plot(i, X_traj2(:,i+1)'*Q*X_traj2(:,i+1), 'b*' , i, U_traj2(:,i)'*R*U_traj2(:,i) , 'bo');
        JMPC1_x_T_01 = JMPC1_x_T_01 + X_traj1(:,i+1)'*Q*X_traj1(:,i+1);
        JMPC1_p_T_01 = JMPC1_p_T_01 + p_traj1(:,i)'*R*p_traj1(:,i);
        JMPC1_u_T_01 = JMPC1_u_T_01 + U_traj1(:,i)'*R*U_traj1(:,i);
    end
    legend('MPC3 x from T_01', 'MPC3 u from T_01', 'MPC2 x', 'MPC2 u from T_01')
    grid on
    title('Cost-To-Go comparison over the simulation length')
    hold off;
    
    %[~,P,~] = dlqr(param.A, param.B, Q, R);
    %JMPC3_x = JMPC3_x +X_traj3(:,31)'*P*X_traj3(:,31);
    
    disp('MPC1 from T_01: ')
    disp([JMPC1_x_T_01, J_MPC1_T_01(1);
     JMPC1_p_T_01, J_MPC1_T_01(2);
     JMPC1_u_T_01 , J_MPC1_T_01(2)])
    
    disp('MPC2 from T_01: ')
    disp([JMPC2_x_T_01, J_MPC2_T_01(1);
     JMPC2_p_T_01, J_MPC2_T_01(2)
     JMPC2_u_T_01 , J_MPC2_T_01(2)])
 
    disp('MPC3 from T_01: ')
    disp([JMPC3_x_T_01, J_MPC3_T_01(1);
     JMPC3_p_T_01, J_MPC3_T_01(2);
     JMPC3_u_T_01 , J_MPC3_T_01(2)])
 
    
% OPTIMAL COST TO GO FOR T_02
[T_traj4, p_traj4, J_MPC1_T_02, ~, ~, ~] = simulate_building(T0_2, @controller_mpc_1, Q, R, scen1,0,N);
[T_traj5, p_traj5, J_MPC2_T_02, ~, ~, ~] = simulate_building(T0_2, @controller_mpc_2, Q, R, scen1,0,N);
[T_traj6, p_traj6, J_MPC3_T_02, ~, ~, ~] = simulate_building(T0_2, @controller_mpc_3, Q, R, scen1,0,N);

X_traj6 = T_traj6 - T_steady;
U_traj6 = p_traj6 - p_steady;
X_traj5 = T_traj5 - T_steady;
U_traj5 = p_traj5 - p_steady;
X_traj4 = T_traj4 - T_steady;
U_traj4 = p_traj4 - p_steady;

JMPC3_x_T_02 = 0;
JMPC3_p_T_02 = 0;
JMPC3_u_T_02 = 0;
JMPC2_x_T_02 = 0;
JMPC2_p_T_02 = 0;
JMPC2_u_T_02 = 0;
JMPC1_x_T_02 = 0;
JMPC1_p_T_02 = 0;
JMPC1_u_T_02 = 0;
    
    figure(667);
    hold on;
    for i= 1:60
        JMPC3_x_T_02 = JMPC3_x_T_02 + X_traj6(:,i+1)'*Q*X_traj6(:,i+1);
        JMPC3_p_T_02 = JMPC3_p_T_02 + p_traj6(:,i)'*R*p_traj6(:,i);
        JMPC3_u_T_02 = JMPC3_u_T_02 + U_traj6(:,i)'*R*U_traj6(:,i);
        plot(i, X_traj6(:,i+1)'*Q*X_traj6(:,i+1), 'r*' , i, U_traj6(:,i)'*R*U_traj6(:,i) , 'ro');
        JMPC2_x_T_02 = JMPC2_x_T_02 + X_traj5(:,i+1)'*Q*X_traj5(:,i+1);
        JMPC2_p_T_02 = JMPC2_p_T_02 + p_traj5(:,i)'*R*p_traj5(:,i);
        JMPC2_u_T_02 = JMPC2_u_T_02 + U_traj5(:,i)'*R*U_traj5(:,i);
        plot(i, X_traj5(:,i+1)'*Q*X_traj5(:,i+1), 'b*' , i, U_traj5(:,i)'*R*U_traj5(:,i) , 'bo');
        JMPC1_x_T_02 = JMPC1_x_T_02 + X_traj4(:,i+1)'*Q*X_traj4(:,i+1);
        JMPC1_p_T_02 = JMPC1_p_T_02 + p_traj4(:,i)'*R*p_traj4(:,i);
        JMPC1_u_T_02 = JMPC1_u_T_02 + U_traj4(:,i)'*R*U_traj4(:,i);
    end
    legend('MPC3 x from T_02', 'MPC3 u from T_02', 'MPC2 x', 'MPC2 u from T_02')
    grid on
    title('Cost-To-Go comparison over the simulation length')
    hold off;
    
    %[~,P,~] = dlqr(param.A, param.B, Q, R);
    %JMPC3_x = JMPC3_x +X_traj3(:,31)'*P*X_traj3(:,31);
    
    disp('MPC1 from T_02: ')
    disp([JMPC1_x_T_02, J_MPC1_T_02(1);
     JMPC1_p_T_02, J_MPC1_T_02(2);
     JMPC1_u_T_02 , J_MPC1_T_02(2)])
    
    disp('MPC2 from T_02: ')
    disp([JMPC2_x_T_02, J_MPC2_T_02(1);
     JMPC2_p_T_02, J_MPC2_T_02(2)
     JMPC2_u_T_02 , J_MPC2_T_02(2)])
 
    disp('MPC3 from T_02: ')
    disp([JMPC3_x_T_02, J_MPC3_T_02(1);
     JMPC3_p_T_02, J_MPC3_T_02(2);
     JMPC3_u_T_02 , J_MPC3_T_02(2)])
 
    fprintf('Press to continue\n');
    pause
    end
    
%% TEST
    if test_cost
    [T_traj0, p_traj0, J_MPC0, ~, ~, ~] = simulate_building(param.T_sp, @controller_mpc_3, Q, R, scen1,0,N);
    J_MPC0;
    
    pow = 0;
    for i = 1:60
        pow = pow + param.p_sp'*R*param.p_sp;
    end
    
    pow;
    
    % CONCLUSION: state cost considers only the value fed to yalmip
    % optimizer (T - T_sp), while input cost considers the absolute one (so
    % Yalmip output + p_sp). For the state, the first one is not considered
    % (it's the initilization), maybe because there is no optimization over
    % it (this condition is indeed fixed). 
    end
%% 
    Andrea_trial  = false; 
    if (Andrea_trial)
    fprintf("\n- Plotting Costs associated to the three MPC Controllers with T0_1 as initial condition");
    % COST for MPC 1 at T_01
    %Concerning the cost we have the one coming from T, but we want the one
    %coming from the difference in temperature, namely referred to X.
    J_vect_MPC1_T01 = zeros(61 , 1);
    for k = 1:1:61
        J_MPC = zeros(2 ,1 );
        [~, ~, J_MPC, ~, ~, ~] = simulate_building(T_traj5(: , k), @controller_mpc_1, Q, R, scen1);
        J_vect_MPC1_T01(k,1) = J_MPC(1,1) + J_MPC(2,1);
    end
    fprintf('\n\nDone\n');
    [JMPC1, sum(J_MPC1_T_01)]
    
    % COST for MPC 2 at T_01
    J_vect_MPC2_T01 = zeros(61 , 1);
    for k = 1:1:61
        J_MPC = zeros(2 ,1 );
        [~, ~, J_MPC, ~, ~, ~] = simulate_building(T_traj1(: , k), @controller_mpc_2, Q, R, scen1);
        J_vect_MPC2_T01(k,1) = J_MPC(1,1) + J_MPC(2,1);
    end
    fprintf('Done\n');
    %J_vect_MPC2_T01
    
    % COST for MPC 3 at T_01
    J_vect_MPC3_T01 = zeros(61 , 1);
    for k = 1:1:61
        J_MPC = zeros(2 ,1 );
        [~, ~, J_MPC, ~, ~, ~] = simulate_building(T_traj3(: , k), @controller_mpc_3, Q, R, scen1);
        J_vect_MPC3_T01(k,1) = J_MPC(1,1) + J_MPC(2,1);
    end
    fprintf('Done\n');
    %J_vect_MPC3_T01
    
%     figure(1510); set(gcf, 'WindowStyle' ,'docked'); hold on;
%     plot(0:1:60 , J_vect_MPC1_T01, 'b- .'); 
%     plot(0:1:60 , J_vect_MPC2_T01, 'r- .'); 
%     plot(0:1:60 , J_vect_MPC3_T01, 'g- .');
%     title('Optimal Cost along the trajectory');
%     axis([-Inf Inf -Inf Inf]);
%     legend('MPC 1 from T01' , 'MPC 2 from T01' , 'MPC 3 from T01');
%     grid on;
%     hold off;
    
    
    fprintf("\n- Plotting Costs associated to the three MPC Controllers with T0_2 as initial condition\n");
    % COST for MPC 1 at T_02
    J_vect_MPC1_T02 = zeros(61 , 1);
    for k = 1:1:61
        J_MPC = zeros(2 ,1 );
        [~, ~, J_MPC, ~, ~, ~] = simulate_building(T_traj6(: , k), @controller_mpc_1, Q, R, scen1);
        J_vect_MPC1_T02(k,1) = J_MPC(1,1) + J_MPC(2,1);
    end
    fprintf('Done\n');
    
    
    % COST for MPC 2 at T_02
    J_vect_MPC2_T02 = zeros(61 , 1);
    for k = 1:1:61
        J_MPC = zeros(2 ,1 );
        [~, ~, J_MPC, ~, ~, ~] = simulate_building(T_traj2(: , k), @controller_mpc_2, Q, R, scen1);
        J_vect_MPC2_T02(k,1) = J_MPC(1,1) + J_MPC(2,1);
    end
    fprintf('Done\n');
    
    
    % COST for MPC 3 at T_02
    J_vect_MPC3_T02 = zeros(61 , 1);
    for k = 1:1:61
        J_MPC = zeros(2 ,1 );
        [~, ~, J_MPC, ~, ~, ~] = simulate_building(T_traj4(: , k), @controller_mpc_3, Q, R, scen1);
        J_vect_MPC3_T02(k,1) = J_MPC(1,1) + J_MPC(2,1);
    end
    fprintf('Done\n');
    
%     figure(1511); set(gcf, 'WindowStyle' ,'docked'); hold on;
%     plot(0:1:60 , J_vect_MPC1_T02, 'b- .');
%     plot(0:1:60 , J_vect_MPC2_T02, 'r- .'); 
%     plot(0:1:60 , J_vect_MPC3_T02, 'g- .');
%     title('Optimal Cost along the trajectory');
%     axis([-Inf Inf -Inf Inf]);
%     legend('MPC 1 from T02' , 'MPC 2 from T02' , 'MPC 3 from T02');
%     grid on;
%     hold off;
    
    
     fprintf('\nSystem paused, press key to continue ...\n');


    pause;
    end
end


%% Soft-constrained MPC

if problem(5)

    fprintf('\n5) Soft-constrained MPC\n');
    
    N = 30;

    % ex17, it is unfeasible (warnings off)
    warning('off','all')
    figure(17); %set(gcf, 'WindowStyle' ,'docked');
    clear controller_mpc_3
    [T, p] = simulate_building(T0_1, @controller_mpc_3, Q, R, scen2, 1, N);
    warning('on','all')
    
    % soft contraints
    figure(18); %set(gcf, 'WindowStyle' ,'docked');
    clear controller_mpc_4
    [T, p] = simulate_building(T0_1, @controller_mpc_4, Q, R, scen2, 1, N);

    d = zeros(size(param.A,1),length(T)-1); % estimated disturbance

    for i = 1:length(T)-1
        d(:,i) = T(:,i+1) - (param.A*T(:,i) + param.B*p(:,i));  % difference between real and expected value
    end
    
    
    % use real disturbances
%     d = [scen2.d_VC_scen;
%         scen2.d_F1_scen;
%         scen2.d_F2_scen];
    
    fprintf("\n- Comparison for no-disturbance case\n")

    figure(191); %set(gcf, 'WindowStyle' ,'docked');  
    clear controller_mpc_3
    [T, p] = simulate_building(T0_1, @controller_mpc_3, Q, R, scen1, 1, N);
    
    figure(192); %set(gcf, 'WindowStyle' ,'docked');
    clear controller_mpc_4
    [T, p] = simulate_building(T0_1, @controller_mpc_4, Q, R, scen1, 1, N);
    
    
    % find constant prediction error and subtract it
    d_fix = zeros(size(param.A,1),length(T));
    for i = 1:length(T)-1
        d_fix(:,i) = T(:,i+1) - (param.A*T(:,i) + param.B*p(:,i));  % difference between real and expected value
    end
    
    d = d - d_fix(:,1);
    
    % keep only bigger values (discard random noise)
    [M,I] = max(abs(d),[],2);
    for i = 1:length(d)
        if (abs(d(1,i)) < 0.8*M(1))
            d(1,i) = 0;
        end
        if (abs(d(2,i)) < 0.8*M(2))
            d(2,i) = 0;
        end
        if (abs(d(3,i)) < 0.8*M(3))
            d(3,i) = 0;
        end
    end    
    
    d = [d, zeros(size(param.A,1),N-1)]; % adjust length considering prediction horizon
    
    fprintf("\n- Soft contraints with disturbance knowledge\n")
    
    figure(20); %set(gcf, 'WindowStyle' ,'docked'); 
    clear controller_mpc_5
    [T, p] = simulate_building(T0_1, @controller_mpc_5, Q, R, scen2, 1, N, d);

    fprintf('\nSystem paused, press key to continue ...\n')

    pause;

end

%% Offset-free MPC
if problem(6)
    N=30;
    fprintf('\n6) Offset-free MPC\n');
    
    % need to tune the estimator (faster response)
    figure(230); %set(gcf, 'WindowStyle' ,'docked'); 
%     hold on
    clear controller_mpc_6
    [T, p] = simulate_building(T0_1, @controller_mpc_6, Q, R, scen3, 1, N);
%     hold off
     
    % test difference with other controllers
    figure(231); %set(gcf, 'WindowStyle' ,'docked');
    clear controller_mpc_3
    [T, p] = simulate_building(T0_1, @controller_mpc_3, Q, R, scen3, 1, N);


    
    
    fprintf('\nSystem paused, press key to continue ...\n')
    pause;
end

%% Comparison using forces
if problem(7)
    fprintf('\n7) Forces\n');
    N = 30;
    
    %Simuation with MPC1 for T_02
    figure(241); %set(gcf, 'WindowStyle' ,'docked');
    clear controller_mpc_1
    [~,~] = simulate_building(T0_2, @controller_mpc_1, Q, R, scen1, 0, N);
    t_sim = zeros(1,6);
    
    for i = 1:5
        [~,~,~,t_sim(i)] = simulate_building(T0_2, @controller_mpc_1, Q, R, scen1,0,N);
    end
    [~,~,~,t_sim(6)] = simulate_building(T0_2, @controller_mpc_1, Q, R, scen1,1,N);
    
    
    %Simuation with MPC1_FORCES for T_02
    figure(242); %set(gcf, 'WindowStyle' ,'docked');
    clear controller_mpc_1_forces
    [~,~] = simulate_building(T0_2, @controller_mpc_1_forces, Q, R, scen1, 0, N);
    t_sim_forces = zeros(1,6);
    
    for i = 1:5
        [~,~,~,t_sim_forces(i)] = simulate_building(T0_2, @controller_mpc_1_forces, Q, R, scen1,0,N);
    end
    [~,~,~,t_sim_forces(6)] = simulate_building(T0_2, @controller_mpc_1_forces, Q, R, scen1,1,N);
    
    % README    to compare the times I have initialized both the
    % controllers before, so init function is not considered in any of
    % them. It's needed for FORCES, could be eventually deleted in the one
    % before. 
    
    fprintf("\n\n\n ---------------------------- \n\n")
    fprintf("Time needed for Yalmip MPC: %f\n", mean(t_sim))
    fprintf("Time needed for FORCES MPC: %f\n", mean(t_sim_forces))
    fprintf("\n--> FORCES is %f times faster\n\n", mean(t_sim)/mean(t_sim_forces))

    
    fprintf('\nSystem paused, press key to continue ...\n')
    pause;  
end
