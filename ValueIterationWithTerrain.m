% ------ State space breakdown ------ %
% The state space is a function of the following: 
%     - position of the rover 
%     - the number of possible time steps
%     - the number of possible loading configurations.
%
% ===== state struct =====
% state.pos  : position of rover in gridworld
%               [row,col] = ind2sub([env.rows, env.rows], state.pos)
%               pos       = sub2ind([env.rows, env.rows], row, col)
% state.cargo: index of which cargo value we're carrying
% state.visited: index of which samples have been visited
% state.t    : time used so far. game ends at state.t = env.missionLength
%
% The position of the rover is expressed using a single number, and the
% row-column position can simply be found using ind2sub. 

% The number of possible time values is the mission length divided by the 
% time step. 

% The number of unique possible loading configurations is 2^(# of samples). 
% So if there are 5 samples, there are 2^5, or 32 different loading
% configurations, including an emtpy load. 
%
% Loading configuration notation method: sample # -> bit
%                state.visited: 10101
%                               |||||
%                     sample #: 54321
% Loading configuration number: 21
% This represents that samples, 1, 3, and 5 have been visited.
%

%% ValueIteration()
% outputs: policy - optimal policy for every state
function [policy] = ValueIterationWithTerrain(env, terr)
    tic
    
    numActions = 4; % [1 = up, 2 = right, 3 = down, 4 = left]
    
    % find state space size
    gridSize = env.rows*env.rows; %number of possible positions
    num_tsteps = env.missionLength/env.ts; % Number of time steps
    num_samples = length(env.samples(1,:)); % number of samples
    num_sloads = 2^num_samples; % Number of possible loading configurations
    cargos = [0, env.samples(2,1)*[1:num_samples]]; % TODO: this assumes constant sample value
    num_cargos = length(cargos);
        
    % initialize U (pos, time, cargo load, cargo value)
    U = zeros(gridSize, num_tsteps, num_sloads, num_cargos);
    
    gamma = 0.99; %set gamma value, because time is a function of state, gamma = 1
        
    % calculate time to get home from each sample (do it once now)
    % row[i] = [min time, max time] to get home from sample i
    % min time == max time if terrain (:. time) is known
    timeToGetHome = time_to_home_terr(env, terr);
        
    k = 1; % iteration count
    while(k<num_tsteps)
        for load = 1:num_sloads  % iterate over all sample configurations
            for cargo = 1:num_cargos % iterate over sizes of cargo we could be carrying
                
                % we can't have more cargo than samples we've visited
                if cargos(cargo) > sum(bitget(load-1, 1:length(env.samples(1,:))))*env.samples(2,1)
                    break;
                end
                
                for t = 1:num_tsteps  % iterate over all timesteps
                    for pos = 1:gridSize % iterate over locations
                    
                    state.pos = pos;
                    state.t = t;
                    state.visited = load-1;
                    state.cargo = cargo;
                    
                    rew = R(env, timeToGetHome, state); % reward of current state
            
                    Usa = zeros(numActions,1); % Usa will hold the U of each future possible state
                    for a = 1:numActions 
                        
                        % find the distribution of next states that action a can go to
                        sprime = next_state(env, terr, state, a);
                        
                        for i = 1:length(sprime)
                            if sprime{i}.pos ~= pos % don't count reward if we hit the edge, or if the game is over
                                T = 1/length(sprime); % equal probability for every unknown terrain
                                Usa(a) = Usa(a) + T*(U(sprime{i}.pos, sprime{i}.t, sprime{i}.visited+1, sprime{i}.cargo));
                            end
                        end
                    end
                    
                    U(pos, t, load, cargo) = max(rew + gamma*Usa); % Bellman equation
                   
                    end
                end
            end
        end
        
        disp(['iteration: ' num2str(k) '. time since start: ' num2str(toc)]);
        k = k+1;
    end


    % find optimal policy based on U (similar loop structure to U loop)
    policy = zeros(size(U));
    for t = 1:num_tsteps  % iterate over all timestep
        for load = 1:num_sloads  % iterate over all sample configurations
            for cargo = 1:num_cargos % iterate over cargo we could be carrying
                for pos = 1:gridSize % iterate over locations

                state.pos = pos;
                state.t = t;
                state.visited = load-1;
                state.cargo = cargo;
                
                Usa = zeros(numActions,1); % Usa will hold the U of each future possible state
                for a = 1:numActions
                    % find the distribution of next states that action a can go to
                    sprime = next_state(env, terr, state, a);
                        
                    for i = 1:length(sprime)
                        if sprime{i}.pos ~= pos % don't count reward if we hit the edge, or if the game is over
                            T = 1/length(sprime); % equal probability for every unknown terrain
                            Usa(a) = Usa(a) + T*(U(sprime{i}.pos, sprime{i}.t, sprime{i}.visited+1, sprime{i}.cargo));
                        end
                    end
                end
                            
                % see which action will give highest U
                [~, policy(pos, t, load, cargo)] = max(gamma*Usa); % reward not a function of action so not necessary
                
                end
            end
        end
    end
        
    toc;
end

%% next_state(s, a)
% return distribution over next states after taking action a
% essentially just a wrapped version of moveOneSqure, accounting for cargo
% pickup and returning full state structs

% input: env, state struct, action
% output: sprime = [s'1, s'2, s'3, ...] (possible new states)
%         new states have equal distribution
function [sprime] = next_state(env, terr, s, a)

    [dt, posnew] = moveOneSquare(env, terr, s.pos, a);
    tnew = s.t + dt; % vector of possible new times

    % ========== Location-based stuff (deterministic) ==========
    if s.pos == env.home
        % don't double-count samples; reset cargo
        s.cargo = 1;
    end

    % sample in this state?
    sample = find(env.samples(1,:) == posnew);
    % no sample, or we already picked it up
    if (isempty(sample)) || (bitand(bitshift(1,sample-1),s.visited) ~= 0)
        cargonew = s.cargo;
        visited = s.visited;
    else % new position has a sample! pick it up
        num_samples = length(env.samples(1,:)); % number of samples
        num_cargos = num_samples+1;
        cargonew = min(s.cargo + 1, num_cargos); % increase index of cargo by one (+1 cargo)
        visited = bitor(s.visited,bitshift(1,sample-1));
    end

    % ========== Time-based stuff (probabilistic) =========
    num_tsteps = env.missionLength/env.ts;
    sprime = cell(1, length(dt));
    for i = 1:length(dt)
        if tnew(i) <= num_tsteps % make sure end time not exceeded
            sprime{i}.pos = posnew;
            sprime{i}.t = tnew(i);
            sprime{i}.cargo = cargonew;
            sprime{i}.visited = visited;
        else %if past end time, you're stuck! 
            sprime{i} = s;
        end
    end

end

%% R(s,a)
% The reward associated with being in a certain state:
% - if s == home, reward = value of our cargo
% - if s ~= home, reward = P(make it home)*(value of sample at s)
%
% Note: reward is independent of action
function reward = R(env, timeToGetHome, s)
    if(s.pos == env.home)
        num_samples = length(env.samples(1,:)); % number of samples
        cargos = [0, env.samples(2,1)*(1:num_samples)]; % TODO: this assumes constant sample value
        reward = cargos(s.cargo);
    else
        num_tsteps = env.missionLength/env.ts;

        % find value of sample at this position
        sample = find(env.samples(1,:) == s.pos); % sample at this location
        
        % if no sample or sample has already been picked up
        if (isempty(sample)) || (bitand(bitshift(1,sample-1),s.visited) ~= 0)
            reward = 0;
        else  
            reward = env.samples(2,sample);
            timeRange = timeToGetHome(sample,:); % min/max time it will take to get home from this sample
            tmin = s.t + timeRange(1);
            tmax = s.t + timeRange(2);
            
            if (tmin == tmax)
                P = num_tsteps >= tmin;
            else
                if tmin > num_tsteps
                    P = 0;
                elseif tmax <= num_tsteps
                    P = 1;
                else % tmin <= num_tsteps < tmax
                    P = min((num_tsteps - tmin + 1)/(tmax-tmin + 1), 1);
                end
            end
            
            % only consider the reward if we'll be able to make it home
            reward = P*reward;
        end
    end
end

%% moveOneSquare
% distribution over new times and determinisitic position after taking
% action a from position pos
% Check that we're not moving off the grid
function [dt, posnew] = moveOneSquare(env, terr, pos, a)
    [row,col] = ind2sub([env.rows, env.rows], pos);
    rownew = row;
    colnew = col;
    m = 0;
    
    switch a
        case 1
            % move up
            if (row~=1) % don't go off the map
                m = (env.topo(row-1,col) - env.topo(row,col))/env.d; % slope of this move
                rownew = row-1;
            end
        
        case 2
            % move right
            if (col~=env.rows) % don't go off the map
                m = (env.topo(row,col+1) - env.topo(row,col))/env.d; % slope of this move
                colnew = col+1;
            end
        
        case 3
            % move down
            if (row~=env.rows) % don't go off the map
                m = (env.topo(row+1,col) - env.topo(row,col))/env.d; % slope of this move
                rownew = row+1;
            end

        otherwise
            % move left
            if (col~=1) % don't go off the map
                m = (env.topo(row,col-1) - env.topo(row,col))/env.d;
                colnew = col-1;
            end
    end
    
    % NOTE: THIS HAS BEEN CHANGED.
    % posnew = pos if it hits the edge, which should be enough
    % we used to have dt = 0 also
    posnew = sub2ind([env.rows, env.rows], rownew, colnew);
    m = max(0,m);

    if (terr(env.zones(rownew,colnew)) < 0)
        % we don't know terrain, could be {0, 1, 2, 3}
        dt = (0:3) + m + 1; % see definition of speed
    else
        dt = terr(env.zones(rownew,colnew))+m+1;  % see definition of speed
    end
    
end