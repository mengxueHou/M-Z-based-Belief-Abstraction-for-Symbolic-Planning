# M-Z-based-Belief-Abstraction-for-Symbolic-Planning
Code for the paper Mori-Zwanzig Approach for Belief Abstraction with Application to Belief Space Planning.
Preprint of the paper can be found at https://www.researchsquare.com/article/rs-3266750/v1

The code is for a learning-based method to extract symbolic representations of the belief state and its dynamics in order to solve planning problems in a 
continuous-state partially observable Markov decision processes (POMDP) problem. To improve  accuracy of the abstracted representation, we introduce a memory-dependent abstraction approach to mitigate the modeling error. We use a Neural Network based method to learn the non-Markovian transition model based on the Mori-Zwanzig (M-Z) formalism. The code implement simulation experiment of a underwater vehicle navigation problem as an instance of a continuous-state POMDP to validate the performance of the proposed belief abstraction algorithms.
<img width="938" alt="Alg_overview" src="https://github.com/mengxueHou/M-Z-based-Belief-Abstraction-for-Symbolic-Planning/assets/68844002/1e48a1ba-2d52-4211-ad48-d1867eea3c57">

# Tips on the current simulation implementation

The proposed algorithm solves a continuous-state POMDP in two phases. 

1) During the off-line training phase, a set of simulated experiences are collected from simulator. The simulated experiences are used to learn the Markovian transitions via Maximum Likelihood Estimation. Then the simulated trajectories are fed as training dataset into an LSTM (Long-short term memory) to identify a non-Markovian model of the belief dynamics.
  
  To turn on the simulator, please set "training = 1" in main.m. 
  When "training = 0", the toolbox can directly call the trained network, and skip the data collection and modeling phase. 

2) The second phase of the code is on-line execution phase. During online execution, we simulate real-time observations according to the observation model. Then the projected Bayesian update is used to incorporate real-time measurements to update the belief state. Given the computed belief state, the POMDP is solved by a symbolic planner.
 
# Extension to other application scenarios

The proposed algorithm is a general solution to the continuous-state POMDP problems. It can be extended and tailored to other application scenarios. 

To do so, please follow these steps:
1) replace the offline simulator to generate experiences with arbitrary process model: PF_agent.m
2) tailor the focused finite partition approach to specific application scenario in partition_setup.m. Dimension of the state space will need to be updated according to the applcation.
3) replace the simulator (detect.m) to generate online observations. Arbitrary observation model can be incorporated here.
4) performance of the proposed approach depends on some of the key parameters in generateing symbolic representation of the belief state. Please refer to our manuscript on some insight on tuning such parameters. 
