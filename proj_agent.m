function Theta = proj_agent(pf_, nPartition, v_agent)


nParticles = pf_{1}.Nparticles;
nSteps = numel(pf_);

%% partitions
% for ii = 1:nPartition
%     plot(v{ii}(:,1), v{ii}(:,2),'k-'); hold on;
% end


Theta = zeros(nPartition, nSteps);

for tt = 1:numel(pf_)
    
    pf_{tt}.particles(1,(pf_{tt}.particles(1,:)<-10)) = -10;
    pf_{tt}.particles(1,(pf_{tt}.particles(1,:)>0)) = 0;
    pf_{tt}.particles(2,(pf_{tt}.particles(2,:)<0)) = 0;
    pf_{tt}.particles(2,(pf_{tt}.particles(2,:)>10)) = 10;
    
    for ii = 1:nPartition
        % particles that start from cell ii
        [IN_ii, ON_ii] = inpolygon(pf_{tt}.particles(1,:), pf_{tt}.particles(2,:), v_agent{ii}(:,1), v_agent{ii}(:,2));
        Theta(ii, tt) = (sum(IN_ii) + sum(ON_ii))/nParticles;        
    end
end
end