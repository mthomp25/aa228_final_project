% without unknown terrain, we can perform value iteration until
% convergence to get the optimal policy

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
% outputs: policy - ideal policy for every state
%          p_seq  - ideal policy for a sequence of actions starting at home
%                   p_seq gives an "actual runthrough" of the game
%          score  - value of cargo at end of p_seq
function [U, policy, p_seq, sts, score] = ValueIteration()
    tic
    env = setupEnv(2);
    
    numActions = 4; % [1 = up, 2 = right, 3 = down, 4 = left]
    % find state space size
    gridSize = env.rows*env.rows; %number of possible positions
    num_tsteps = env.missionLength/env.ts; % Number of time steps
    num_samples = length(env.samples(1,:)); % number of samples
    num_sloads = 2^num_samples; % Number of possible loading configurations
    cargos = [0, env.samples(2,1)*[1:num_samples]]; % TODO: this assumes constant sample value
    num_cargos = length(cargos);
    
    % find time home from each sample
    t_home = time_to_home(env);
    
    % initialize U
    U = zeros(gridSize, num_tsteps, num_sloads, num_cargos); %U(pos, time, cargo load, cargo value)
    
    gamma = 0.99; %set gamma value, because time is a function of state, gamma = 1
    
    disp(['Value Iteration, gamma ' num2str(gamma)])
    
    k = 1; % iteration count
    while(k<25)
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
                    
                    rew = R(env, t_home, state); % reward of current state
            
                    Usa = zeros(numActions,1); % Usa will hold the U of each future possible state
                    for a = 1:numActions 
                        new_state = next_state(env, state, a); %find state that action a goes to

                        if new_state.pos == pos % rover hit edge, or the game is over
                            Usa(a) = 0;
                        else
                            Usa(a) = gamma*U(new_state.pos, new_state.t, new_state.visited+1, new_state.cargo);
                        end
                        
                    end
                    
                    U(pos, t, load, cargo) = max(rew + Usa); % Bellman equation
                   
                    end
                end
            end
        end
        
        disp(['iteration: ' num2str(k)]);
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
                    new_state = next_state(env, state, a);

                    if new_state.pos == pos % if rover hit edge
                        Usa(a) = 0;
                    else
                        Usa(a) = gamma*U(new_state.pos, new_state.t, new_state.visited+1, new_state.cargo);
                    end
                end
                            
                % see which action will give highest U
                [~, policy(pos, t, load, cargo)] = max(Usa); % reward not a function of action so not necessary
                
                end
            end
        end
    end
        
    [p_seq, sts, score] = policy_seq(env, policy); % find policy sequence
    plot_path(env, sts);
    toc;
end

%% next_state(s, a)
% return next state after taking action a
% essentially just a wrapped version of moveOneSqaure, accounting for cargo
% pickup and returning a full state struct

% input: env, state struct, action
% output: state struct of new state
function s_new = next_state(env, s, a)

    [dt, posnew] = moveOneSquare(env, s.pos, a);
    tnew = s.t + dt;

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

num_tsteps = env.missionLength/env.ts;
if tnew <= num_tsteps % make sure end time not exceeded
    s_new.pos = posnew;
    s_new.t = tnew;
    s_new.cargo = cargonew;
    s_new.visited = visited;
else %if past end time, you're stuck! 
    s_new = s;
end

end

%% R(s,a)
% The reward associated with being in a certain state:
% - if s == home, reward = value of our cargo
% - if s ~= home, reward = P(make it home)*(value of sample at s)
function reward = R(env, t_home, s)
    if(s.pos == env.home)
        num_samples = length(env.samples(1,:)); % number of samples
        cargos = [0, env.samples(2,1)*[1:num_samples]]; % TODO: this assumes constant sample value
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
            % timeToGetHome = timeToGetThere(env, s.pos, env.home);
            timeToGetHome = t_home(sample);
            P = (num_tsteps >= (s.t + timeToGetHome)); % probability that we can get home in time (0 or 1)
            
            % only consider the reward if we'll be able to make it home
            reward = P*reward;
        end
    end
end

%% moveOneSquare
% new time and position after taking action a from position pos
% Check that we're not moving off the grid
function [dt, posnew] = moveOneSquare(env, pos, a)
    [row,col] = ind2sub([env.rows, env.rows], pos);
    dt = 0;
    posnew = pos;
    
    switch a
        case 1
            % move up
            if (row~=1) % don't go off the map
                m = (env.topo(row-1,col) - env.topo(row,col))/env.d; % slope of this move
                m = max(0,m);
                dt = m+1;  % see definition of speed
                posnew = sub2ind([env.rows, env.rows], row-1, col);
            end
        
        case 2
            % move right
            if (col~=env.rows) % don't go off the map
                m = (env.topo(row,col+1) - env.topo(row,col))/env.d; % slope of this move
                m = max(0,m);
                dt = m+1;  % see definition of speed
                posnew = sub2ind([env.rows, env.rows], row, col+1);
            end
        
        case 3
            % move down
            if (row~=env.rows) % don't go off the map
                m = (env.topo(row+1,col) - env.topo(row,col))/env.d; % slope of this move
                m = max(0,m);
                dt = m+1;  % see definition of speed
                posnew = sub2ind([env.rows, env.rows], row+1, col);
            end

        otherwise
            % move left
            if (col~=1) % don't go off the map
                m = (env.topo(row,col-1) - env.topo(row,col))/env.d;
                m = max(0,m);
                dt = m+1;  % see definition of speed
                posnew = sub2ind([env.rows, env.rows], row, col-1);
            end
    end
end


%% policy_seq(env, policy)
% Create a sequence of actions from the optimal policy
% converts the optimal policy over all states to the optimal sequence of
% actions.
% Starts at home state, takes optimal action, calculates which state it
% ends up in, then take optimal action of that state.

%  input: env, policy (optimal policy over all states)
% output: p_seq-sequence of actions, sts-states visited, score-value of aquired cargo

function [p_seq, sts, score] = policy_seq(env, policy)

num_tsteps = env.missionLength/env.ts; % Number of time steps

% start in home state with no samples visited
state.pos = env.home;
state.t = 1;
state.visited = 0;
state.cargo = 1;

pol = zeros(num_tsteps, 1); % sequence of actions can't be longer than number of time steps (will assuredly be shorter)
sts = zeros(num_tsteps, 1); % positions visited

r_home = 0; %record when the rover reaches home
score = 0;

for i = 1:num_tsteps %position in sequence
    
    pol(i) = policy(state.pos, state.t, state.visited+1, state.cargo); %optimal action for this state
    sts(i) = state.pos;
    
    % record earliest time the max score is reached and rover is home
    score_int = R(env, zeros(length(env.samples(1,:))), state);
    if state.pos == env.home
        r_home = i;
        score = score + score_int;
    end
    
    state = next_state(env, state, pol(i));
end

% if the rover made it home, clip policy at that point
disp(pol);
disp(score);
if r_home > 1
    p_seq = [pol(1:r_home); 0];
    sts = sts(1:r_home);
else
    disp('Did not return home!!!!')
    disp(score);
    p_seq = pol;
    score = 0;
end

end

%% plot_path(env, sts)
% plot path of rover
% TAKEN FROM displayMap Function
%input: env, sts (states visited)
function plot_path(env, sts)

pos_c = zeros(length(sts),1);
pos_r = zeros(length(sts),1);

for i = 1:length(sts)
    [pos_r(i), pos_c(i)] = ind2sub([env.rows, env.rows], sts(i));
end

[rh,ch] = ind2sub([env.rows,env.rows],env.home);

nS = size(env.samples,2);

clf

figure(1); hold on

subplot(3,2,[1 2 3 4]); hold on
    grid on
    plot(pos_c-.5, -(pos_r-.5), 'r--')
    plot(ch-0.5,-(rh-0.5),'ko') %home
    for s = 1:nS
        [rs,cs] = ind2sub([env.rows,env.rows],env.samples(1,s));
        plot(cs-0.5,-(rs-0.5),'g*') %sample
    end
    axis([0 env.rows -env.rows 0])
    axis square
hold off
subplot(3,2,5); hold on
    imagesc(flipud(env.topo))
    plot(pos_c, -(pos_r)+env.rows+1, 'r--')
    title('Topography')
    set(gca,'YTickLabel',[]);
    set(gca,'XTickLabel',[]);
    axis([0.5 env.rows+0.5 0.5 env.rows+0.5])
    axis square
hold off
subplot(3,2,6); hold on
    imagesc(flipud(env.zones))
    plot(pos_c, -(pos_r)+env.rows+1, 'r--')
    title('Terrain Zones')
    set(gca,'YTickLabel',[]);
    set(gca,'XTickLabel',[]);
    axis([0.5 env.rows+0.5 0.5 env.rows+0.5])
    axis square
hold off
hold off


end
