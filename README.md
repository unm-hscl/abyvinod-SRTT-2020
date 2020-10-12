# abyvinod-SRTT-2020

| Title      | Stochastic reachability of a target tube: Theory and Computation |
|------------|------------------------------------------------------------------|
| Authors    | Abraham P. Vinod & Meeko M. K. Oishi                             |
| Journal    | Automatica (in review), https://arxiv.org/abs/1810.05217         |

## Requires

- SReachTools https://sreachtools.github.io (version 1.3.1)
    - SReachTools requires MPT3 https://www.mpt3.org/ and CVX http://cvxr.com/cvx/
- StocHy https://github.com/natchi92/stochy
    - Please use
      [https://github.com/abyvinod/stochy.git](https://github.com/abyvinod/stochy.git)
      instead of the official repository.
- Python and MATLAB
    - Relevant python conda environment is given in `./stochy.yml`.


## Instructions to reproduce figures

To generate the figures associated with the computations, run the following
MATLAB/Python scripts:

1. Figure 2
    - Illustrates the dynamic programming solution to the stochastic
      reachability of target tube problem
    - Run `FigureIllustrateDP.m` to generate the figure using `SReachTools`
1. Figure 6a and 6b and Table 3 (left half)
    - Comparison of Algorithms 1 and 2 with dynamic programming solution for
      double integrator dynamics
    - Run `FigureDI.m` to parse matfile `matfiles/DI_example_201011_111336.mat`
      and generate data and figures used in the paper.
    - To regenerate matfile and figures, 
        - Run `ExampleDI.m`
        - Edit `FigureDI.m` to update `datestr_to_load` variable with the
          date-time string from the newly generated matfile.
        - Rerun `FigureDI.m`.
1. Figure 6c and 6d and Table 3 (right half)
    - Comparison of Algorithms 1 and 2 with dynamic programming solution for a
      chain of integrator dynamics
    - Run `FigureCI.m` to parse matfile `matfiles/CI_example_201011_115942.mat`
      and generate data and figures used in the paper.
    - To regenerate matfile and figures, 
        - Run `ExampleCI.m`
        - Edit `FigureCI.m` to update `datestr_to_load` variable with the
          date-time string from the newly generated matfile.
        - Rerun `FigureCI.m`.
1. Figure 7
    - Demonstration of the advantage of being grid-free approach 
    - Comparison of Algorithm 1 vs `StocHy` for a stochastic viability problem
      (n=2)
    - Run `./FigureStocHy/Figure7_StochHy_and_SReachTools.py` to produce the
      figures from the previously generated matfiles in
      `./FigureStocHy/Figure7_data`.
    - To regenerate data,
        - For `SReachTools`, run `./FigureStocHy/Figure7_SReachTools.m` with
          appropriate choice of `low_dir_vecs` set to True (for |D|=8) or False
          (for |D|=32).
        - For `StocHy`, see section below.
        - Update `./FigureStocHy/Figure7_StochHy_and_SReachTools.py` as provided
          in comments.
1. Figure 8
    - Demonstration of the scalability of SReachTools 
    - Comparison of Algorithm 1 vs `StocHy` for a stochastic viability problem
      (n=2 to n=10)
    - Run `./FigureStocHy/Figure8_StochHyVsSReachTools_scalability.m` to produce
      the figures from the previously generated matfiles in
      `./FigureStocHy/Figure8_data`.
    - To regenerate data,
        - For `SReachTools`, run
          `./FigureStocHy/Figure8_SReachToolsCode_StocHyVsSReachTools_scalability.m`.
        - For `StocHy`, see section below.
        - Update
          `./FigureStocHy/Figure8_SReachToolsCode_StocHyVsSReachTools_scalability.m`
          as provided in comments.
1. Figure 9
    - Run the codes in Figure 9 folder.
1. Figure 10 (top, down)
    - Comparison of Algorithm 1 with Lesser et. al. CDC 2013 on a spacecraft
      rendezvous problem
    - Run `FigureCWH.m` to parse matfiles
      `matfiles/CWH_example_201011_133225_zero_vel.mat` and
      `matfiles/CWH_example_201011_130417_nonzero_vel.mat` and generate data and
      figures used in the paper. Use `problem_setup` to switch between the two
      figures.
    - To regenerate matfile and figures, 
        - Run `ExampleCWH.m` with `problem_setup` set to `top` or `bottom` to
          get the respective figures.
        - Edit `FigureCWH.m`:
            - Change `problem_setup` set to `top` or `bottom` to get the
              respective figures.
            - Update `datestr_to_load` variable with the date-time string from
              the newly generated matfiles in `./matfiles` folder from
              `ExampleCWH`.
        - See `CDC2013_repeatability/` for computing the solution from [27].
        - Rerun `FigureCWH.m`.
1. Figure 11
    - Implementation of Algorithm 1 on the stochastic reachability of
      time-varying target sets with a linear time-varying system (Dubins' car) 
    - Run `FigureCar.m` to parse matfile
      `matfiles/Car_example_201011_145310.mat` and generate data and figures
      used in the paper.
    - To regenerate matfile and figures, 
        - Run `ExampleCar.m`
        - Edit `FigureCar.m` to update `datestr_to_load` variable with the
          date-time string from the newly generated matfile.
        - Rerun `FigureCar.m`.

### StocHy code

Figures 7 to 9 require [StocHy](https://github.com/natchi92/stochy) as well.
Please use
[https://github.com/abyvinod/stochy.git](https://github.com/abyvinod/stochy.git)
instead of the official repository.
1. Clone the repository along with the submodules.
1. Change directory into `<PROJECT_HOME>/stochy`.
1. Run `./build_debug.sh`.
1. Change directory into `<PROJECT_HOME>/stochy/build/`.
1. Open `../src/stochy/Main_verifyLTI.cpp`, and change `prob_case` to 1, 2, 31, 32, 33 to run the relevant experiments.
1. Run `make stochy_verifyLTI` to build the edited script.
1. Run `./bin/stochy_verifyLTI` to run the relevant experiments using StocHy.

Repeat the last three steps whenever a new experiment is required. All results
are collected in `<PROJECT_HOME>/stochy/results`.
    
## Contact

Contact Abraham P. Vinod at aby.vinod@gmail.com, if you have any questions.
