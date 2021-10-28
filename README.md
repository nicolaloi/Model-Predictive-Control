# Model Predictive Control
Implementation of a variety of MPC controllers for temperature regulation of a building. Specifically, the controller tracks the reference temperatures while satisfying safety constraints at all time steps. The system is modeled using heat flows between the rooms and the environment, and taking into considerations also exogenous disturbances, e.g. solar radiation.

##

Project rated with full marks.

## Tasks

### Unconstrained Optimal Control with LQR
Design of a discrete-time infinite horizon Linear Quadratic Regulator for reference tracking.
Implementation of an heuristic LQR approach for controller tuning. This method aims at finding diagonal cost matrices such that the LQR minimizes the deviation from the desired stady-state.
<p align= "center">
<img height="225" src="https://github.com/loinicola/MPC/blob/main/plots/figure_7.png"/>
</p>

### MPC with Theoretical Closed-Loop Guarantees
Design of MPC controllers with feasibility and stability guarantees in closed loop. Closed loop trajectories will satisfy state and input constraints for all time steps and the target temperatures are asymptotically stable equilibrium points for the closed-loop system. Several variants to this problem are addressed, e.g. different terminal sets are considered.
<p align= "center">
<img height="225" src="https://github.com/loinicola/MPC/blob/main/plots/figure_14_1.png"/>
</p>

### Soft Constraints with disturbance
Design of a soft-constrained MPC controller to avoid the infeasibility caused by additional unmodeled disturbances.
Soft constraints are introduced to tackle the fact that in practice optimization problems can become infeasible despite the theoretical guarantees on constraints' satisfaction and stability due to model mismatch or unmodeled disturbances.  
<p align= "center"> 
<em>Normal MPC</em>
</p>
<p align= "center">
<img height="225" src="https://github.com/loinicola/MPC/blob/main/plots/figure_17.png"/>
</p>
<p align= "center"> 
<em>Soft MPC</em>
</p>
<p align= "center">
<img height="225" src="https://github.com/loinicola/MPC/blob/main/plots/figure_18.png"/>
</p>
  
#### Soft Constraints with disturbance knowledge
Extend the soft-constraint MPC controller by leveraging the knowledge of the future disturbances in a practical manner, enhancing the performance and the constraints' satisfaction.  

<p align= "center">
<img height="225" src="https://github.com/loinicola/MPC/blob/main/plots/figure_20.png"/>
</p>

### Offset-Free MPC
Disturbances in the heat flows are never known exactly. Therefore, estimation of the actual disturbances is needed to ensure offset-free reference tracking.
The heat transfer between different zones is typically not modelled or known exactly. Estimate the resulting disturbance such that offset-free tracking is ensured, modeling it as a time-invariant unknown disturbance. The simulation of the system additionally includes unmodeled time-varying disturbances, hence the observer dynamics cannot be chosen arbitrarily fast and perfect tracking cannot be expected.
<p align= "center"> 
<em>Normal MPC</em>
</p>
<p align= "center">
<img height="225" src="https://github.com/loinicola/MPC/blob/main/plots/figure_23_1.png"/>
<p align= "center"> 
<em>Offset-free MPC</em>
</p>
</p>
<p align= "center">
<img height="225" src="https://github.com/loinicola/MPC/blob/main/plots/figure_23_0.png"/>
</p>

## Setup

Further info on this programming exercise can be found in [Assignment.pdf](Assignment.pdf). Solutions as well as considerations on the problem itself are instead in [Report.pdf](Report.pdf).  
  
Run the whole project from [run_simulations.m](run_simulations.m).  
  
Some functions require the installation of [Yalmip](https://yalmip.github.io/), [MPT Toolbox](https://www.mpt3.org/), and [Forces PRO](https://www.mathworks.com/products/connections/product_detail/forces-pro.html).
