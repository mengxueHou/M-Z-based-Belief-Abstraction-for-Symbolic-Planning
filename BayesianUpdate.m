function Theta_pos = BayesianUpdate(Theta_agent, landmarks, obs_landmark, p_correct, detect_range_landmark, noPartition_agent)

global v_agent  domain


% figure;
% visualize_theta(Theta_agent);

domain = [-10, 0, 0, 10];

Theta_pos = [];

nx = 50;
ny = 50;

xx = linspace(domain(1), domain(2), nx);
yy = linspace(domain(3), domain(4), ny);


likelihood = zeros(nx, ny, 4);
likelihood_partition = zeros(4, noPartition_agent);%xi
idx = cell(noPartition_agent,1);

% update agent belief with landmark detection
for m = 1:nx
    for n = 1:ny
        s = detect([xx(m), yy(n)]', landmarks, detect_range_landmark);
        likelihood(m,n,:) = (s~=obs_landmark).*(1-p_correct) + (s==obs_landmark).*p_correct;
        %likelihood(m,n) = prod(likelihood_fcn);
        for jj = 1:noPartition_agent
            [IN_ii, ~] = inpolygon(xx(m), yy(n), v_agent{jj}(:,1), v_agent{jj}(:,2));            
            if IN_ii 
                idx{jj} = [idx{jj}; [m,n]];
                likelihood_partition(:,jj) = likelihood_partition(:,jj) + reshape(likelihood(m,n,:), [4,1]);
%                 nome(m,n) = Theta_agent(jj) .* likelihood(m,n);
%                 deno = deno + Theta_agent(jj) .* likelihood(m,n) ;
            end
                
        end
        
    end
end

% for ii = 1:noPartition_agent
%     likelihood_partition(ii) = likelihood_partition(ii)/size(idx{ii},1);
% end
prior = Theta_agent;
for ii = 1:numel(obs_landmark)
    deno_i = likelihood_partition(ii,:)* prior;
    nome_i = likelihood_partition(ii,:) .* prior';
    posterior = nome_i'/deno_i;
    prior = posterior;
end

Theta_pos = posterior;


% figure;
% visualize_theta(Theta_agent_pos);

% draw likelihood function

% figure
% imagesc(likelihood(:,:,2))
% set(gca,'FontSize',16);
% grid on;


% Update hotspot position
% ---------------------------------------------------------------------

% figure;
% visualize_theta(Theta_hotspot);
% 
% likelihood = zeros(nx, ny);
% deno = 0;
% nome = zeros(nx, ny);
% idx = cell(noPartition_hotspot,1);
% 
% 
% 
% % update hotspot belief with landmark detection
% for m = 1:nx
%     for n = 1:ny
%         s = detect([xx(m), yy(n)]', landmarks, detect_range_landmark);
%         likelihood_fcn = (s~=obs_hotspot).*(1-p_correct) + (s==obs_hotspot).*p_correct;
%         likelihood(m,n) = prod(likelihood_fcn);
%         for jj = 1:noPartition_hotspot
%             [IN_ii, ~] = inpolygon(xx(m), yy(n), v_hotspot{jj}(:,1), v_hotspot{jj}(:,2));            
%             if IN_ii 
%                 idx{jj} = [idx{jj}; [m,n]];
%                 nome(m,n) = Theta_hotspot(jj) .* likelihood(m,n);
%                 deno = deno + Theta_hotspot(jj) .* likelihood(m,n);
%             end
%                 
%         end
%         
%     end
% end
% for m = 1:nx
%     for n = 1:ny
%         post_unparam(m,n) = nome(m,n) ./ deno;
%     end
% end
% 
% Theta_hotspot_pos = zeros(noPartition_hotspot, 1);
% for jj = 1:noPartition_hotspot
%     for kk = 1:size(idx{jj},1)
%         Theta_hotspot_pos(jj) = Theta_hotspot_pos(jj) + post_unparam(idx{jj}(kk,1), idx{jj}(kk,2));
%     end
%     Theta_hotspot_pos(jj) = Theta_hotspot_pos(jj);
% end
% 
% 
% figure;
% visualize_theta(Theta_hotspot_pos);
% 
% Theta_pos = Normalize(reshape(Theta_agent_pos*Theta_hotspot_pos', [size(Theta_agent_pos,1)*size(Theta_hotspot_pos,1),1]));

end