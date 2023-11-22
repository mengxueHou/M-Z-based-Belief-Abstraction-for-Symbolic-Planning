classdef PF_agent
    
    properties
        % filter values
        Nparticles
        Nstates
        
        sigma_x
        
        % filter elements
        particles
        % timestep
        dt
        
    end
    
    methods
        function obj = PF_agent(Nparticles, Nstates, sigma_x, sigma_Xinit, particles, dt)
            % setup the particle filter object
            obj.Nparticles = Nparticles;
            obj.Nstates = Nstates;
            obj.sigma_x = sigma_x;            
            % draw particle samples using Xinit
            obj.particles = particles .* ones(Nstates, Nparticles) + sigma_Xinit * randn(Nstates, Nparticles);           
            obj.dt = dt;
        end

        function obj = motion_update(obj, U, F, x, y)
            % update particles according to displacement input
            % U: Nstates x 1
            % F: flow field N*N
            % x/y: lon/lat
            
            nx = numel(x);
            ny = numel(y);
            
            idx_x = interp1(x, 1:nx, obj.particles(1,:),'nearest');
            idx_y = interp1(y, 1:ny, obj.particles(2,:),'nearest');            
            
            for ii = 1:obj.Nparticles
                if isnan(idx_x(ii)) + isnan(idx_y(ii)) > 0
                    flow = 0;
                else
                    flow = F(idx_x(ii), idx_y(ii));
                end
                obj.particles(:,ii) = obj.particles(:,ii) + ([real(flow), imag(flow)]' + U + obj.sigma_x * randn(obj.Nstates, 1))*obj.dt;
            end
        end
        
    end
end

