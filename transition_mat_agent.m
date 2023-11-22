function T = transition_mat_agent(pf_)

%% transition_mat_agent:
% Find the transition matrix of a MDP (infinite dimensional) using the GCM
% (generalized cell maping method). Refer to Sun JQ, Xiong FR, Sch ̈utze O, et al (2018) Cell
% mapping methods. Springer for the algorithm. 

% Input: pf_: struct containing simulated particle positions in two
% consecutive timesteps. 

% Output: T: each element [i,j] is a complex number: real part is # of
% particles go from j to i, imag part is total # of particles in j 

% Mengxue Hou
% Georgia Institute of Technology
% 2023/06/22

global v_agent nPartition_agent

nSteps = length(pf_);
nParticles = pf_{1}.Nparticles;

% p = zeros(nPartition_agent, nSteps);
% total_pr = zeros(nSteps, 1);


for tt = 1:nSteps-1
    
    % If the particles end up out of the domain, put them back to the
    % domain. 

    pf_{tt}.particles(1,(pf_{tt}.particles(1,:)<-10)) = -9.9999;
    pf_{tt}.particles(1,(pf_{tt}.particles(1,:)>0)) = -0.0001;
    pf_{tt}.particles(2,(pf_{tt}.particles(2,:)<0)) = -0.0001;
    pf_{tt}.particles(2,(pf_{tt}.particles(2,:)>10)) = 9.99999;
    
    for ii = 1:nPartition_agent
        % particles that start from cell ii
        [IN_ii, ~] = inpolygon(pf_{tt}.particles(1,:), pf_{tt}.particles(2,:), v_agent{ii}(:,1), v_agent{ii}(:,2));
        for jj = 1:nPartition_agent
            % particles that end up in cell jj
            [IN_jj, ~] = inpolygon(pf_{tt+1}.particles(1,:), pf_{tt+1}.particles(2,:), v_agent{jj}(:,1), v_agent{jj}(:,2));
            % # of particles in ii move to jj at next step / total # of
            % particles in ii
                T(jj, ii) = sum((IN_ii == IN_jj) .* (IN_ii == 1) ) + 1j*sum(IN_ii==1);
        end
    end
end

% Visualize the transition matrix

% figure;
% for ii = 1:nPartition
%     for jj = 1:nPartition
%         h = scatter(ii, jj, 'filled');
%         h.Marker = 's';
%         h.MarkerFaceColor = 'b';
%         h.SizeData = 350;
%         h.MarkerFaceAlpha = mean_trans(ii, jj); hold on;
%     end
% end
% clb = colorbar

% title(['timestep ', num2str(tt)])
% hold off
% pause(0.01)
% grid on

end