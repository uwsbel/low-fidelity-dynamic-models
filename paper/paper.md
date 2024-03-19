---
title: 'A Library of Lower Fidelity Dynamics Models (LFDMs) For On-Road Vehicle Dynamics Targeting Faster Than Real-Time Applications'
tags:
  - C++
  - CUDA
  - Vehicle Dynamics
  - Controls
  - Reinforcement Learning
authors:
  - name: Huzaifa Mustafa Unjhawala
    orcid: 0009-0004-4273-1212
    corresponding: true
    affiliation: 1
  - name: Ishaan Mahajan
    affiliation: 2
  - name: Radu Serban
    orcid: 0000-0002-4219-905X
    affiliation: 3
  - name: Dan Negrut
    orcid: 0000-0003-1565-2784
    affiliation: 4
affiliations:
 - name: PhD. Student, Mechanical Engineering, University of Wisconsin-Madison, Madison, USA
   index: 1
 - name: Undergraduate Student, Computer Science, University of Wisconsin-Madison, Madison, USA
   index: 2
 - name: Distinguished Scientist, University of Wisconsin-Madison, Madison, USA
   index: 3
 - name: Professor, University of Wisconsin-Madison, Madison, USA
   index: 4
date: 25 Jan 2024
bibliography: paper.bib
---

# Summary
Lower Fidelity Dynamic Models (LFDM) is a library of on-road wheeled vehicles that is written in C++ and CUDA and wrapped to Python using SWIG. Each of these on-road wheeled-vehicle dynamics models is described as a set of Ordinary Differential Equations (ODEs) that take a driver input - a normalized throttle between 0 and 1, a normalized steering between -1 and 1 (with -1 representing a left turn), and a normalized braking input between 0 and 1, and subsequently advance the state of the vehicle (its position and velocity) forward in time.<br>

In mathematical notation, these ODEs are of second order and are expressed as

\begin{equation}
\mathbf{\"x} = f(\mathbf{x},\mathbf{\dot{x}}, \mathbf{u}, \mathbf{P}) \; ,
\end{equation}

where $\mathbf{x} \in \mathbb{R}^d$ is the $d$ dimensional state of the vehicle, $\mathbf{u} \in \mathbb{R}^3$ contains the driver inputs, and $\mathbf{P} \in \mathbb{R}^k$ contains the $k$ model parameters. The ODEs evolving the vehicle dynamics models are cast as an Initial Value Problems (IVPs) by providing an initial state $\mathbf{x}$ and are solved using implicit and semi-implicit numerical methods.<br>

The LFDM contains three dynamic vehicle models, differentiated by their Degrees of Freedom (DoF) counts: 11 DoF, 18 DoF, and 24 DoF. Each can be run on the CPU or an NVIDIA GPU.

11 DoF Model: This is a foundational, single-track model with two wheels, primarily employed in controller design. It features a rigid chassis with three DoFs at the Center of Mass (C.M) – yaw, lateral, and longitudinal. The model includes a kinematic driveline, integrating a map-based engine and torque converter, gearbox, and differential, channeling torque to the wheels. It utilizes one of the two versions of the TMeasy non-linear tire model for traction-force generation. Steering inputs are converted to front wheel angles through a steering map.

18 DoF Model: An advancement to the 11 DoF, this double-track model includes four wheels and introduces a roll DoF, enhancing chassis dynamics. It shares the driveline structure with the 11 DoF model but adds front and rear differentials for torque distribution across wheels (See Fig. \autoref{fig:driveline}). The steering map in this model turns both front wheels by the same angle. Other subsystems remain consistent with the 11 DoF model.

24 DoF Model: The most comprehensive, the 24 DoF model incorporates the features of the 18 DoF and extends to predict vehicle heave and pitch motions, attributing to its all-encompassing six chassis DoFs: lateral, longitudinal, vertical, yaw, roll, and pitch. Additionally, it includes a DoF at each corner for vertical suspension travel. Subsystems, excluding the suspension aspect, are identical to the 18 DoF model.

![A flow chart of the torques (blue arrows) and angular velocities (red arrows) that are exchanged all across the drive line. An overview of the symbols is provided below.\label{fig:driveline}](images/driveline.png){ width=80% }

| Symbol | Description                                    |
|--------|------------------------------------------------|
| $J_{in}$ | Motor input shaft inertia ($kg \cdot m^2$) |
| $T$      | Torque Ratio (-)                              |
| $\hat{g}$ | Current gear ratio (-)                       |
| $T$      | Torque Ratio (-)                              |
| $\eta$   | Differential split (-)                        |  

Each model's fidelity level varies, catering to different aspects of vehicle dynamics simulation, while maintaining a consistent foundation in their core subsystems. Importantly, certain features within these models are optional and customizable based on user preferences. For example, the inclusion of the torque converter can be modified, and drivetrain configurations such as four-wheel drive (4WD), rear-wheel drive (RWD), or front-wheel drive (FWD) can be selected and adjusted through parameter configurations in a JSON file, offering flexibility to suit various simulation requirements.

First, we discuss a [Statement of need](#statement-of-need) describing in what applications these models are most useful. In Section [LFDM Accuracy](#LFDM-Accuracy), we present a comparison of the LFDMs' accuracy with the High-Fidelity Vehicle Model, Chrono::Vehicle [@Serban:2019]. We then demonstrate in [LFDM Speed and Scaling](#LFDM-Speed-and-Scaling) that the LFDMs, while closely matching the accuracy of Chrono::Vehicle, operate approximately 3000 times faster. Additionally, by utilizing the GPU version of the models, it is possible to simulate about 300,000 vehicles in real-time, meaning simulating one second of dynamics for 300,000 vehicles takes only one real-world second. Further details on the model formulation are available in Chapter Two of [@huzaifa:2023].

# Statement of need
These models aim to provide open-source, accurate vehicle simulations that exceed real-time speeds, ideal for state estimation, control, reinforcement learning, and traffic simulations. Influenced by various models in literature [@Pepy:2006; @Kong:2015; @Jin:2019], they introduce additional benefits tailored for faster-than-real-time applications.

1. Literature typically favors simplistic single-track models with linear tires or fully kinematic models for their speed, despite a trade-off in accuracy. [@Liu:2016] found that while a double-track model with non-linear tires is more accurate, its Real Time Factor (RTF) of 1 limits its use in Control stacks. The LFDM library, with efficient C++ coding, offers faster-than-real-time double-track models with accurate non-linear tires and realistic subsystems, including drivelines, engines, and torque converters.

2. Existing literature often lacks associated open-source code and documentation for vehicle models, hindering their use by researchers. Where open-source options exist, models of varying fidelity are scattered across multiple repositories. The LFDM library addresses this by providing a centralized source for researchers to access and select vehicle models of different fidelities, tailored to their specific speed-accuracy requirements and hardware capabilities. 

3. To the best of our knowledge, there is currently no open-source software capable of executing large-scale, parallel simulations of on-road vehicle dynamics on GPUs. The LFDMs bridge this gap, facilitating the real-time simulation of nearly 300,000 vehicles. This capability significantly enhances the potential for large-scale reinforcement learning and comprehensive traffic simulations.

4. Vehicle models in literature typically use explicit solvers for solving IVPs, but these are unstable for non-linear, stiff ODEs [@Ascher:1998]. The LFDM library addresses this by offering two stable solvers: a semi-implicit solver with constant time-stepping for real-time simulations, and an implicit solver with adaptive time-stepping via Sundials [@Hindmarsh:2005]. While the Sundials solver, due to its adaptive time-stepping nature, is slower and unsuitable for faster-than-real-time simulation, it ensures stability and supports Forward Sensitivity Analysis (FSA). Both solvers also generate system RHS Jacobians $\frac{\partial f}{\partial \mathbf{x}}$ and $\frac{\partial f}{\partial \mathbf{u}}$, crucial for gradient-based Model Predictive Control (MPC) methods.

5. Commercial platforms like MSC Adams and MATLAB provide vehicle models of varying fidelities, complete with benchmarks. However, their closed-source nature limits the integration of these models into control stacks unless the same tools are employed throughout. 

# LFDM Accuracy
To assess the LFDMs' accuracy, we used the high-fidelity Chrono::Vehicle simulator to generate ground-truth data. Each LFDM was calibrated through a Bayesian Inference framework [@Unjhawala:2023] to approximate a Sedan, akin to an Audi A3, modeled with Chrono::Vehicle. We conducted five throttle and steering maneuvers, varying throttle levels and steering directions, to calibrate the LFDMs. These maneuvers, ranging from 30 km/h to 65 km/h, helped fine-tune parameters for various turn speeds and calibrate engine and steering maps.

Post-calibration, we tested the LFDMs in two scenarios: a high-speed 90 km/h acceleration and a 40 km/h double lane change, following ISO 3888-2 standards using a PID controller for the latter. The tests applied the same control inputs to both the Sedan and LFDMs. Results, illustrated in Fig.\autoref{fig:acc} and \autoref{fig:dl}, reveal that the 24 DoF and 18 DoF models closely mimic the Sedan's dynamics, outperforming the commonly used 11 DoF model. However, in the straight-line acceleration test, the Chrono::Vehicle Sedan exhibited non-zero yaw and lateral velocities due to engine reaction torque, a detail not captured by the LFDMs. The LFDMs' accuracy was also compared against other vehicle types like the US Army's HMMWV, as detailed in Chapter 5 of [@huzaifa:2023].

![The LFDMs integrated using the semi-implicit solver with a time step of $1e^{-3}$ compared to a high-fidelity Chrono::Vehicle simulation of a Sedan integrated at a time step of $1e^{-4}$ for a high speed acceleration maneuver.\label{fig:acc}](images/accTest.png){ width=80% }

![The LFDMs integrated using the semi-implicit solver with a time step of $1e^{-3}$ compared to a high-fidelity Chrono::Vehicle simulation of a Sedan integrated at a time step of $1e^{-4}$ for a ISO standard double lane change maneuver.\label{fig:dl}](images/dlTest.png){ width=80% }

# LFDM Speed and Scaling
To demonstrate the speed and accuracy of the LFDMs, we benchmarked them against Chrono::Vehicle. A 10-second acceleration maneuver was simulated for both, and the Real Time Factor (RTF) — the time required to simulate one second of dynamics — was computed. Both systems used a $1e^{-3}$ time step, with the LFDMs employing a semi-implicit integrator and Chrono::Vehicle using its standard Differential Algebraic Equation (DAE) solver. The results, summarized in the table below, show that the LFDMs, when optimized with O3 and run on a 13th Gen Intel(R) Core(TM) i7-13700K, are at least 2000 times faster than real-time.

| Model            | Simulation time (ms) | Run time (ms) | RTF  | $1/RTF$ |
|------------------|----------------------|---------------|------|---------|
| Chrono::Vehicle | 10,000 | $5134.29 \pm 72$ | $5.1e^{-1}$ | 2 |
| 24 DoF | 10,000 | $3.67 \pm 0.0003$ | $3.6e^{-3}$ | 2720 |
| 18 DoF | 10,000 | $2.6 \pm 0.0006$ | $2.6e^{-3}$ | 3835 |
| 11 DoF | 10,000 | $1.5 \pm 0.0002$ | $1.5e^{-3}$ | 6461 |

Further, the GPU version of the LFDMs enables large-scale parallel simulation, vital for Reinforcement Learning and traffic simulations. As shown in Fig. \autoref{fig:gpu_scale}, around 330,000 11DoF vehicle models can be simulated on an Nvidia A100 GPU with an RTF of 1.

![Scaling analysis of the GPU versions of the LFDMs shows that about 330,000 11 DoF vehicles can be simulated in Real-Time.\label{fig:gpu_scale}](images/gpuScale.png){ width=60% }

# Acknowledgements
This work has been partially supported by NSF projects FW-HTF-R2222541 and CMMI2153855.

# References
