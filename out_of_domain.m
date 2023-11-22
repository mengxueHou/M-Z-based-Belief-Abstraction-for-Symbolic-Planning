function particles_out = out_of_domain(particles, limx, limy)
% detect particles that go out of the domain. remove these particles
particles_out = [];
for ii = 1:size(particles,1)
    if particles(ii, 1)>max(limx) || particles(ii,1)<min(limx) ...
            || particles(ii, 2)>max(limy) || particles(ii,2)<min(limy)
    else
        particles_out = [particles_out; particles(ii,:)];
    end
           
end

end