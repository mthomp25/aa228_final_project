# aa228_final_project
code for aa228 final project

Kaitlin Dennison, Stephanie Schneider, and Michael Thompson

Stanford University 

Please see final paper for problem setup and method description 

Extremely general overview of usage:

To run value iteration with uniform terrain, type this command in the matlab command window:
[U, policy, p_seq, sts, score] = ValueIteration()

U is the final utility matrix, policy is the optimal policy, p_seq is a ideal policy for a sequence 
of actions starting at home, giving an "actual runthrough" of the game, and score is the value of 
cargo at end of p_seq. A full run will take about 40 minutes and automatically plot the rover path

To run a simulation with unkown terrain and open loop planning, use command: 
[p_seq, score] = OpenLoopPlan(level)

level is any integer 0-6 and determines the world setup. The output variables are the same as normal 
value iteration. Depending on the level, a full run can take up to 10 hours

Be sure to add the Dec5Sim folder to the matlab path (use addpath)









