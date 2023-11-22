%% ------------------------------------------------

% Matlab implementation of the paper

% Mengxue Hou, Tony Lin, Enlu Zhou and Fumin Zhang, Mori-Zwanzig Approach for Belief Abstraction with
%Application to Belief Space Planning

% Mengxue Hou
% Georgia Institute of Technology
% 2023/06


%% ------------------------------------------------

addpath('utils')
clc; clear all; close all;

% Set up partitions

% For the algorithm generating the partitions, please refer to our ICRA paper:
% M. Hou, T. X. Lin, H. Zhou, W. Zhang, C. R. Edwards and F. Zhang, "Belief Space
% Partitioning for Symbolic Motion Planning," 2021 IEEE International Conference on Robotics
% and Automation (ICRA), Xi'an, China, 2021, pp. 8245-8251, doi: 10.1109/ICRA48506.2021.9561121.
partition_setup;

global v_agent
partition_setup;

%% -----------------------------------
view_particles_agent = 0;
visualize_setup = 0;
visual = 0;
video_particle = 0;
video2 = 0;
video3 = 0;
training = 0;

%% ----------------------------------
% domain setup
nx = 100; ny = 100;

x = linspace(-20, 20, nx);
y = linspace(-20, 20, ny);

dx = 0.4;
dy = 0.4;

[yy, xx] = meshgrid(x, y);

% # of sample points
nSample = 300;
% # of steps
nSteps = 300; %1500;
training_idx = 1:round(2/3*nSteps);
test_idx = round(2/3*nSteps)+1:nSteps-1;
modeling_idx = 1:300; % index for identifying the GCM model
% # of agent states
nStatesAgent = 2;
% # of hotspot states
nStatesHotspot = 2;
% sample time
dt = 0.5;
% scale factor for NN training
ScaleFactor = 10;

%% -----------------------------------
% flow conditions
% double gyre
A = 0.1;
eps = 0.005;
omega = 0;

for tt = 1:nSteps
    time = tt * dt;
    a = eps*sin(omega*time);
    b = 1 - 2*eps * sin(omega*time);
    for ii = 1:nx
        for jj = 1:ny
            f = a*x(ii)^2/100 + b*x(ii)/10;
            F(ii,jj,tt) = -pi*A*sin(pi*f)*cos(pi*y(jj)/10)...
                + 1j*( pi*A*cos(pi*f)*sin(pi*y(jj)/10) * (2*x(ii)/10*a+b) ); % flow speed
        end
    end
end

%% -----------------------------------
% agent state setup
% # of partition
nPartition_agent = 20;

% initial conditions
sigma_Xinit = [0.9, 0;
    0, 0.9;];

x0_mean = [-6,5]';
% u10: [-8, 4]';

% precess noise
sigma_x = [0.1, 0;  0,   0.1];

% actual state
x_true = zeros(2,100);
x_true(:,1) = x0_mean;


% correct detection rate inside detection range
p_detect_in = 0.95;
% correct detection rate outside detection range
p_detect_out = 0.95;

% feasible inputs
U = 0.1;
theta_set = [0, 0.25, 0.5, 0.75, 1, 1.25, 1.5, 1.75].*pi;

% initialization
pf_agent_{1} = PF_agent(nSample, nStatesAgent, sigma_x, sigma_Xinit, x0_mean, dt);

% belH threshold
pH = 0.7;

if training

    %% step 1: identify the markovian transition
    for u_idx = 1:numel(theta_set)
        u =  [real(U*exp(1j*theta_set(u_idx))), imag(U*exp(1j*theta_set(u_idx)))]';
        for t =  modeling_idx
            pf_agent_{t+1} = pf_agent_{t}.motion_update(u, reshape(F(:,:,t),[nx, ny]), x, y);

            if t ~= modeling_idx(end)

                pf_t_agent{1} = pf_agent_{t};
                pf_t_agent{2} = pf_agent_{t+1};
                % get the transition mat
                trans_agent{t} = transition_mat_agent(pf_t_agent);
            end
        end

        mean_trans = zeros(size(trans_agent{1}));
        cnt = zeros(size(mean_trans));
        for ii = modeling_idx(1:end-1)
            for jj = 1:size(trans_agent{ii},1)
                for kk = 1:size(trans_agent{ii},2)
                    if trans_agent{ii}(jj,kk) ~=0
                        mean_trans(jj,kk) = mean_trans(jj,kk) + trans_agent{ii}(jj,kk);
                        cnt(jj,kk) = cnt(jj,kk) + 1;
                    end
                end
            end
        end

        for jj = 1:size(trans_agent{ii},1)
            for kk = 1:size(trans_agent{ii},2)
                if cnt(jj,kk) ~= 0
                    mean_trans(jj,kk) = real(mean_trans(jj,kk))/imag(mean_trans(jj,kk));
                end
            end
        end
        % normalize
        mean_trans = Normalize(mean_trans);
        GCM_trans_mat{u_idx} = mean_trans;

    end
    clear pf_agent_
    pf_agent_{1} = PF_agent(nSample, nStatesAgent, sigma_x, sigma_Xinit, x0_mean, dt);

    %% step 2: train LSTM
    for n_sim = 1:500
        r = randi([1 8],1,nSteps);
        for t = 1:nSteps
            % randomly pick a uk inside the feasible set to generate the
            % training data
            u_sequence(:,t) = [real(U*exp(1j*theta_set(r(t)))), imag(U*exp(1j*theta_set(r(t))))]';
            pf_agent_{t+1} = pf_agent_{t}.motion_update(u_sequence(:,t), reshape(F(:,:,t),[nx, ny]), x, y);

            %         if visual
            %             % plot particles
            %             if view_particles_agent
            %                 scatter(pf_agent_{t+1}.particles(1,:), pf_agent_{t+1}.particles(2,:), ...
            %                     [], 'g', 'MarkerEdgeAlpha', .5);
            %                 hold on;
            %             end
            %             hold on;
            %         end
            %
            %         if video_particle
            %             print(gcf, '-dpng', [num2str(t), '.png'], '-r200');
            %         end
            %
            %     end

            % get the ground truth of symbols
            Theta_agent(:,t) = proj_agent(pf_agent_(t), nPartition_agent, v_agent);
            err_vec_full = zeros(nPartition_agent, nSteps);

            % get resolved dynamics: T b_k
            res_state_agent(:,t+1) = Normalize(GCM_trans_mat{r(t)}*Theta_agent(:,t));

            if video3 % visualize symbols

                %         frame = getframe(gcf);
                %         writeVideo(avi, frame)
                f=figure;
                visualize_symbols(res_state_agent(:,tt));
                h3 = quiver(xx(1:2:end, 1:2:end), yy(1:2:end, 1:2:end), real(F(1:2:end, 1:2:end,t)), imag(F(1:2:end, 1:2:end,t)));
                set(h3, 'color','b')
                xlim([-10 0]);
                ylim([0,10]);
                pbaspect([1 1 1])
                print(gcf, '-dpng', [num2str(tt), '.png'], '-r200');
                close(f)
            end


            err_GCM(:,t) = Theta_agent(:,t) - res_state_agent(:,t);
            err_GCM_norm(t) = norm(err_GCM(:,t), 1);

        end

        Xtrain{n_sim,1} = [Theta_agent(:,training_idx); u_sequence(:,training_idx)];%res_state(:,tt-N:tt);
        Ytrain{n_sim,1} = err_GCM(:,max(training_idx)+1);

        Xtest{n_sim,1} = [Theta_agent(:,test_idx); u_sequence(:,test_idx)];%res_state(:,tt-N:tt);
        Ytest{n_sim,1} = err_GCM(:,max(test_idx)+1);

    end

    %% LSTM training
    % ----------------------------------------------------------------

    % scale up the Xtrain, Ytrain, Xtest data to avoid numerical issues

    for tt = 1:numel(Xtrain)
        Xtrain{tt,1} = Xtrain{tt,1}*ScaleFactor;
        Ytrain{tt,1} = Ytrain{tt,1}*ScaleFactor;
    end

    for tt = numel(Xtest)
        Xtest{tt,1} = Xtest{tt,1}*ScaleFactor;
    end

    numFeatures = nPartition_agent+2;
    numHiddenUnits = 40;
    numResponses = nPartition_agent;

    layers = [ ...
        sequenceInputLayer(numFeatures)
        lstmLayer(numHiddenUnits,'OutputMode','last')
        fullyConnectedLayer(numResponses)
        regressionLayer];


    options = trainingOptions('adam', ...
        'MaxEpochs', 500, ...
        'InitialLearnRate',0.1, ...
        'SequenceLength','longest', ...
        'Shuffle','every-epoch', ...
        'Plots','training-progress',...
        'Verbose',1);


    Ytrain_mat = zeros(20,n_sim);
    for ii = 1:n_sim
        a = Ytrain{ii};
        Ytrain_mat(:,ii) = a;
    end

    net = trainNetwork(Xtrain,Ytrain_mat',layers,options);

    % Test set validation
    % -----------------------------------------------------------------
    for n_sim = 1:500
        pred_test(ii,:) = predict(net, Xtest{n_sim})./ScaleFactor;
    end
    for ii = 1:500
        a = Ytest{ii};
        Ytest_mat(:,ii) = a;
    end
    err_test = pred_test' - Ytest_mat;
    for ii = 1:500
        err_test_norm(ii) = norm(err_test(:,ii),1);
    end

    figure
    histogram(err_test_norm);
    xlabel('L1-norm error')
    ylabel('Frequency')
    grid on; set(gca, 'FontSize', 16)

    % compare with GCM for a randomly generated dataset

    nSteps = 200;
    r = randi([1 8],1,nSteps);
    pf_agent_{1} = PF_agent(nSample, nStatesAgent, sigma_x, sigma_Xinit, x0_mean, dt);
    gcm_theta(:,1) = proj_agent(pf_agent_(1), nPartition_agent, v_agent);
    for t = 1:nSteps
        u_sequence(:,t) = [real(U*exp(1j*theta_set(r(t)))), imag(U*exp(1j*theta_set(r(t))))]';
        pf_agent_{t+1} = pf_agent_{t}.motion_update(u_sequence(:,t), reshape(F(:,:,t),[nx, ny]), x, y);

        % get the ground truth of symbols
        Theta_val(:,t) = proj_agent(pf_agent_(t), nPartition_agent, v_agent);

        % compute GCM result
        gcm_theta(:,t+1) = Normalize(GCM_trans_mat{r(t)}*gcm_theta(:,t));

        err_GCM_val(:,t) = Theta_val(:,t) - gcm_theta(:,t);
        err_GCM_val_norm(t) = norm(err_GCM_val(:,t),1);

        Xval = [Theta_val(:,1:t); u_sequence(:,1:t)].*ScaleFactor;%res_state(:,tt-N:tt);
        pred_test(t,:) = predict(net, Xval)./ScaleFactor;

        M_Z_theta(:,t) = Normalize(GCM_trans_mat{r(t)}*Theta_val(:,t)+ pred_test(t,:)');

        err_MZ(:,t) = Theta_val(:,t) - M_Z_theta(:,t);
        err_MZ_norm(t) = norm(err_MZ(:,t),1);
    end

    figure;
    h1 = plot(1:nSteps, err_MZ_norm, 'LineWidth',2, 'color', 'b');hold on;
    h2 = plot(1:nSteps, err_GCM_val_norm, 'LineWidth',2, 'color', 'r');
    legend([h1,h2],{'LSTM-based Modeling','Generalized Cell Mapping Method'})
    set(gca,'FontSize',16);
    xlabel('Timesteps');
    ylabel('Model reduction error');
    grid on;


else
    load('lstm_net.mat')
    load('GCM_mat')
end


% --------------------------------------------------------------------
%% simulated real-time estimation & planning

nx = 100; ny = 100;

x = linspace(-20, 20, nx);
y = linspace(-20, 20, ny);

% # of beacons
Nlandmarks = 4;
% beancon position
landmarks(:,1) = [-7, 8];
landmarks(:,2) = [-6, 4];
landmarks(:,3) = [-2, 2];
landmarks(:,4) = [-8, 2];
% detection range
detect_range_landmark = [1.5, 2, 3, 1.5];

% ------------------------------------
if visualize_setup
    figure
    h1 = plot(x_true(1,1), x_true(2,1), 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');hold on;
    % plot landmarks
    h2 = plot(landmarks(1,:), landmarks(2,:), 'kh', 'MarkerSize', 10, 'MarkerFaceColor', 'k');hold on;
    circle(landmarks(1,:), landmarks(2,:), detect_range_landmark, [0.5,0.5,0.5], '-');hold on;
    legend([h1, h2, h3],{'Vehicle position', 'Beacon position'})
    set(gca,'FontSize',16);
    xlabel('x');ylabel('y');
    grid on;
    xlim([-10,0]);ylim([0,10]);
end

% Initialization
Theta_agent(:,1) = proj_agent(pf_agent_(1), nPartition_agent, v_agent);
u(:,1) = [real(U*exp(1j*theta_set(1))), imag(U*exp(1j*theta_set(1)))]';
center_target = [-7.3011, 9.0274]; % center of the target set

tic
for tt = 1:nSteps

    % true dynamics
    % --------------------------------------------------------------------
    idx_x = interp1(x, 1:nx, x_true(1,tt),'nearest');
    idx_y = interp1(y, 1:ny, x_true(2,tt),'nearest');

    if isnan(idx_x) + isnan(idx_y) > 0
        flow = 0;
    else
        flow = F(idx_x, idx_y, tt);
    end
    x_true(:,tt+1) = x_true(:,tt) + (u(:,tt) + [real(flow), imag(flow)]' + sigma_x * randn(2, 1)) * dt;

    % observation of landmark ---------------------------------------------
    detect_landmark = detect(x_true(:,tt+1), landmarks, detect_range_landmark);
    obs_landmark = detect_landmark;

    for ii = 1:numel(detect_landmark)
        v = rand(1);
        if detect_landmark(ii) == 1
            obs_landmark(ii) = ( v > (1-p_detect_in)) * 1 +...
                ( v > p_detect_in) * 0;
        else
            obs_landmark(ii) = ( v > (1-p_detect_out)) * 0 + ...
                ( v > p_detect_out) * 1;
        end

    end

    % bayesian update ----------------------------------------------------
    Theta_pos(:,tt) = BayesianUpdate(Theta_agent(:,tt), landmarks, obs_landmark, p_detect_in, ...
        detect_range_landmark, nPartition_agent);

    % planning -----------------------------------------------------------
    planning_horizon = 1;
    % BFS to generate the states
    cnt = 1;
    node{1} = struct('self_idx', 1, 'parent_idx', 0, 'action', 0, 'state', [Theta_pos(:,1:tt); u(:,1:tt)], 'depth', 0, 'est_cost', 0);
    node_list = node{1}.self_idx;
    current_node_idx = node{1}.self_idx;
    maxF = 0.4442;

    min_cost = 100;
    min_cost_idx = 0;

    %     while ~isempty(node_list)
    current_node = node{node_list(end)};
    node_list(end)= [];%remove current_node

    if current_node.depth<=planning_horizon
        for ii = 1:numel(theta_set)
            curr_time = tt+current_node.depth;
            u_plan =  [real(U*exp(1j*theta_set(ii))), imag(U*exp(1j*theta_set(ii)))]'; % candidate actions in the planning horizon
            X_online = current_node.state;
            pred_train = predict(net, X_online)./ScaleFactor;
            new_theta = Normalize(GCM_trans_mat{ii} * X_online(1:20,end) + pred_train');
            new_node = current_node;
            new_node.self_idx = numel(node_list) + 1;
            new_node.parent_idx = current_node.self_idx;
            state_next = [new_theta; u_plan];
            new_node.state = [current_node.state, state_next];
            new_node.action = theta_set(ii);
            new_node.depth = current_node.depth+1;

            % compute the heuristics
            % find jmax pos
            [~,j_max] = max(new_theta);
            vertex_jmax = v_agent{j_max};
            center_jmax = [mean(vertex_jmax(:,1)), mean(vertex_jmax(:,2))];

            h = norm(center_target-center_jmax)/(maxF + U);
            new_node.est_cost = h;
            node_list = [node_list, new_node.self_idx];
            node{new_node.self_idx} = new_node;

            if new_node.est_cost <= min_cost
                min_cost = new_node.est_cost;
                min_cost_idx = new_node.self_idx;
            end
        end
    end
    %     end

    % select the action that maximize the est_cost
    best_node = node{min_cost_idx};
    u(:,tt+1) =  [real(U*exp(1j*best_node.action)), imag(U*exp(1j*best_node.action))]';

    Theta_agent(:,tt+1) = best_node.state(1:20,end);

    if Theta_agent(1,tt+1) > pH
        toc
        disp(['The total travel cost: ', num2str(tt+1)])

        figure;
        visualize_theta(Theta_agent(:,tt+1));
        h1=plot(x_true(1,1), x_true(2,1), 'ko', 'MarkerSize', 8, 'MarkerFaceColor','k'); hold on;
        h2=plot(x_true(1,1:tt+1), x_true(2,1:tt+1), 'k-', 'LineWidth',2); hold on;
        h3 = quiver(xx(1:2:end, 1:2:end), yy(1:2:end, 1:2:end), ...
            real(F(1:2:end, 1:2:end,tt)), imag(F(1:2:end, 1:2:end,tt)), 'color','b','LineWidth',1.5);
        xlim([-10,0]);ylim([0,10]);
        xlabel('x'); ylabel('y');
        legend([h2, h3], {'True trajectory', 'Flow field'})

        for ii = 1:size(v_agent{1}, 1)-1
            d(ii) = point_to_line([x_true(:,tt+1)', 0], [v_agent{1}(ii,:),0], [v_agent{1}(ii+1,:),0]);
        end
        [min_d, idx] = min(d);
        [in, ~] = inpolygon(x_true(1,tt+1), x_true(2,tt+1), v_agent{1}(:, 1), v_agent{1}(:,2));
        if in==1
            min_d = 0;
        end
        disp(['dist_to_target: ', num2str(min_d)])
        keyboard;
    end

end
