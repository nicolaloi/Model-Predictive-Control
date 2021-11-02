# Model Predictive Control
Implementation of a variety of MPC controllers for temperature regulation of a building. Specifically, the controller tracks the reference temperatures while satisfying safety constraints at all time steps. The system is modeled using heat flows between the rooms and the environment, and taking into considerations also exogenous disturbances, e.g. solar radiation.

##

Project rated with full marks.

##

## Tasks

Further info: [Assignment.pdf](Assignment.pdf) and [Report.pdf](Report.pdf).  

### Unconstrained Optimal Control with LQR
Design of a discrete-time infinite horizon Linear Quadratic Regulator for reference tracking.
Implementation of an heuristic LQR approach for controller tuning. This method aims at finding diagonal cost matrices such that the LQR minimizes the deviation from the desired stady-state.
<p align= "center">
<img height="225" src="https://user-images.githubusercontent.com/79461707/139393634-739efb00-4a79-45f4-80bb-03ade0fd0fd4.png"/>
</p>

### MPC with Theoretical Closed-Loop Guarantees
Design of MPC controllers with feasibility and stability guarantees in closed loop. Closed loop trajectories will satisfy state and input constraints for all time steps and the target temperatures are asymptotically stable equilibrium points for the closed-loop system. Several variants to this problem are addressed, e.g. different terminal sets are considered.
<p align= "center">
<img height="225" src="https://user-images.githubusercontent.com/79461707/139393673-04e43576-ad75-4860-9644-b6eedfada756.png"/>
</p>

### Soft Constraints with disturbance
Design of a soft-constrained MPC controller to avoid the infeasibility caused by additional unmodeled disturbances.
Soft constraints are introduced to tackle the fact that in practice optimization problems can become infeasible despite the theoretical guarantees on constraints' satisfaction and stability due to model mismatch or unmodeled disturbances.  
<p align= "center"> 
<img height="225" src="https://user-images.githubusercontent.com/79461707/139402043-c9806c21-c8df-44b4-a9a3-b6befa2456ae.png"/>
</p>

#### Soft Constraints with disturbance knowledge
Extend the soft-constraint MPC controller by leveraging the knowledge of the future disturbances in a practical manner, enhancing the performance and the constraints' satisfaction.  

<p align= "center">
<img height="225" src="https://user-images.githubusercontent.com/79461707/139393837-6dd308ec-4c22-41ac-b20d-7a88f92984c6.png"/>
</p>

### Offset-Free MPC
Disturbances in the heat flows are never known exactly. Therefore, estimation of the actual disturbances is needed to ensure offset-free reference tracking.
The heat transfer between different zones is typically not modelled or known exactly. Estimate the resulting disturbance such that offset-free tracking is ensured, modeling it as a time-invariant unknown disturbance. The simulation of the system additionally includes unmodeled time-varying disturbances, hence a perfect tracking cannot be expected.
<p align= "center"> 
<img height="225" src="https://user-images.githubusercontent.com/79461707/139401712-e07473f9-3ee5-4a11-b327-86e040ecf538.png"/>
</p>

## Setup
  
Run the whole project from [run_simulations.m](run_simulations.m).  
  
Some functions require the installation of [Yalmip](https://yalmip.github.io/), [MPT Toolbox](https://www.mpt3.org/), and [Forces PRO](https://www.mathworks.com/products/connections/product_detail/forces-pro.html).
