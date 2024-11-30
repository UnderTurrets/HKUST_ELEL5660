Code explanations

readonly folder: supposed to be "read only". DON'T MODIFY THEM!!!
    quadModel_readonly.m: Parameters of a 500g quadrotor
    quadEOM_readonly.m: Dynamic model of a quadrotor.
    run_trajectory_readonly: Solve the equation of motion, receive desired trajectory, run your controller, iteratively. Visualization is also included.

utils: Some useful functions.

test_trajectory.m: The main entry point.

-----------------------------------------------------------------------------------------------------------
controller.m: What you need to work with. Calculate force and moment given current and desired state vector of the quadrotor.

*_trajectory.m: They design trajectories for the quadrotor given waypoints and shape parameters. 
They calculate desired state given current state vector and current time. 
You can also design additional trajectories, which will gives you bonus.

Contact TAs with any questions you may have.

