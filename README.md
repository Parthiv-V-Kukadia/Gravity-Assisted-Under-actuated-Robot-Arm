# Gravity-Assisted-Under-actuated-Robot-Arm

This project simulates the movement of a two-link robot arm, driven by a user-defined controller. The simulation environment is built in MATLAB, providing a platform for designing, implementing, and testing control strategies for robotic manipulators.

## Overview

The `DesignProblem02.m` script serves as the main entry point for running the robot arm simulation. It requires the name of a MATLAB function that defines the control logic as its primary input. Additionally, it accepts several optional parameters to customize the simulation behavior, including data logging, movie and snapshot saving, enabling diagnostics plots, introducing disturbances, defining a reference trajectory, setting initial conditions, and controlling the display.

The simulation models the dynamics of a two-link robot arm, tracking the joint angles (q1, q2) and joint velocities (v1, v2). The user-defined controller function receives sensor data (current joint states) and a reference value for the second joint angle (q2) and outputs a torque command (tau1) for the first joint. The second joint is unactuated in this simulation.

**Key Features:**

* **Customizable Controller:** Users can implement their own control algorithms by creating a MATLAB function with specific `init` and `run` sub-functions.
* **Simulation Parameter Tuning:** Various optional parameters allow for flexible experimentation with different simulation scenarios.
* **Data Logging:** Simulation data can be saved to a `.mat` file for subsequent analysis.
* **Visualization:** A graphical representation of the robot arm's movement is displayed during the simulation.
* **Diagnostics:** Optional plots of the joint states and the control torque can be shown for debugging and performance evaluation.
* **Disturbance Modeling:** An option to introduce an unknown disturbance torque on the first joint allows for testing the robustness of the controller.
* **Reference Tracking:** The controller aims to make the second joint angle follow a time-varying reference trajectory defined by a user-provided function.

## Getting Started

1.  **MATLAB Environment:** Ensure you have MATLAB installed.
2.  **Controller Function:** Create a MATLAB function file (e.g., `MyArmController.m`) that contains your control logic. This file must define two sub-functions: `initControlSystem` and `runControlSystem`, adhering to the input and output specifications detailed within `DesignProblem02.m`.
3.  **Run the Simulation:** Execute the `DesignProblem02.m` script from the MATLAB command window, providing the name of your controller function as the first argument.

    ```matlab
    DesignProblem02('MyArmController');
    ```

## Controller Interface

Your controller function must define the following sub-functions:

* **`initControlSystem(parameters, data)`:** This function is called once at the beginning of the simulation. It's used to initialize controller parameters and internal data structures.
    * **Inputs:**
        * `parameters`: A struct containing physical constants and simulation settings (e.g., `tStep`, `tauMax`, `symEOM`, `numEOM`).
        * `data`: A struct for storing controller-specific data.
    * **Output:**
        * `data`: The (potentially modified) data struct.

* **`runControlSystem(sensors, references, parameters, data)`:** This function is called at each time step of the simulation. It implements your control algorithm.
    * **Inputs:**
        * `sensors`: A struct containing the current state of the robot arm (e.g., `t`, `q1`, `q2`, `v1`, `v2`).
        * `references`: A struct containing the desired reference value for the second joint angle (`q2`).
        * `parameters`: The same parameters struct passed to `initControlSystem`.
        * `data`: The data struct updated in `initControlSystem` and previous calls to `runControlSystem`.
    * **Outputs:**
        * `actuators`: A struct containing the torque command for the first joint (`tau1`).
        * `data`: The (potentially modified) data struct for use in the next time step.

## Optional Parameters

You can customize the simulation using parameter-value pairs when calling `DesignProblem02`:

* `'team'`: A string specifying a team name to display on the figure window.
* `'datafile'`: A string specifying the filename for saving simulation data (e.g., `'data.mat'`).
* `'moviefile'`: A string specifying the filename for saving a movie of the simulation (e.g., `'movie.mp4'`).
* `'snapshotfile'`: A string specifying the filename for saving a PDF snapshot of the final simulation frame (e.g., `'snap.pdf'`).
* `'controllerdatatolog'`: A cell array of strings specifying fields in `controller.data` to log in the data file (if `'datafile'` is defined).
* `'diagnostics'`: A logical flag (`true` or `false`) to enable or disable the display of diagnostic plots for joint states and the control torque (default is `false`).
* `'tStop'`: A positive scalar number specifying the simulation stop time (default is `30`).
* `'disturbance'`: A logical flag (`true` or `false`) to add an unknown disturbance torque to the first joint (default is `false`).
* `'reference'`: A function handle that takes time as input and returns the desired angle of the second joint (e.g., `@(t) sin(t)`). The default is `@(t)0`.
* `'initial'`: A 4x1 numerical matrix `[q1; q2; v1; v2]` specifying the initial joint angles and velocities. The default is `[0.1; 0.01; 0.01; 0.01].*randn(4,1)`.
* `'display'`: A logical flag (`true` or `false`) to enable or disable the live simulation display (default is `true`).

**Example with optional parameters:**

```matlab
DesignProblem02('MyArmController', 'team', 'RoboTeam', 'datafile', 'arm_data.mat', 'tStop', 20, 'diagnostics', true, 'reference', @(t) 0.5*sin(2*t), 'initial', [0.5; 0.2; 0; 0]);
```
## Understanding the Code

The `DesignProblem02.m` script is the core of the robot arm simulation, handling the entire process from setup to visualization and data logging. Here's a breakdown of its key functions:

* **`SetupSimulation(process)`:**
    * Initializes the simulation environment.
    * Defines the physical properties and constants of the robot arm.
    * Loads or computes the equations of motion (EOMs) that govern the arm's dynamics.
    * Sets up the user-provided controller function for interaction with the simulation.

* **`RunSimulation(process, controller)`:**
    * Executes the main simulation loop.
    * At each time step, it calls the user-defined controller to obtain the control torque for the robot arm.
    * Updates the state of the robot arm (joint angles and velocities) based on the applied torque.

* **`UpdateProcess(process, controller)`:**
    * Integrates the robot arm's equations of motion.
    * Uses the `ode45` numerical solver to determine the new joint angles and velocities based on the control torque and any external disturbances.

* **`GetSensors(process)`:**
    * Extracts the current state of the robot arm.
    * Provides the current joint angles (`q1`, `q2`) and velocities (`v1`, `v2`) as sensor feedback to the controller.

* **`GetReferences(process)`:**
    * Determines the desired state for the robot arm based on the current simulation time.
    * Evaluates the user-defined reference function (for `q2`) to provide the target trajectory to the controller.

* **`GetInput(process, actuators)`:**
    * Takes the desired control torque (`tau1`) from the controller.
    * Applies any constraints or limits on the torque (e.g., maximum torque).
    * Incorporates any specified disturbance torque that might be acting on the robot arm.

* **`UpdateFigure(process, controller, fig)`:**
    * Manages the graphical display of the simulation.
    * Updates the visualization of the robot arm's configuration in real-time.
    * If enabled, it also updates diagnostic plots showing the joint states and control torque over time.

* **`UpdateDatalog(process, controller)`:**
    * Records simulation data at each time step.
    * Stores information such as time, joint angles, joint velocities, and control torques for later analysis or saving to a file.

## Included Test Code (`Test code:` section)

The `DesignProblem02.m` script includes a section at the end that demonstrates a basic example of how to define a controller (`Controller` function) and run the simulation. This test code typically illustrates how to:

* Call the `DesignProblem02` function, specifying the name of the controller function.
* Use the `'datafile'` parameter to specify a file for saving simulation data.
* Use the `'tStop'` parameter to set the duration of the simulation.
* Use the `'diagnostics'` parameter to enable the display of diagnostic plots.
* Use the `'initial'` parameter to define the starting configuration (joint angles and velocities) of the robot arm.

By examining and running this test code, users can understand the basic workflow of the simulation and how to integrate their own controller implementations.
