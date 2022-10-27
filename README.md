# oat_semester_project_Fan

## RRT

~~~
run main.m
~~~

Note: you can change the parameters **dim**, **start_coords**, and **end_coords**, where dim represents the manipulator’s degrees of freedom, start_coords = [theta2, theta1] and end_coords = [theta2, theta1] denote the starting and end pose, respectively.

The code will calculate path using function **RRT().**

Two figures will be created:

- Figure 1 visualises created manipulator and its joint-space, where white and black area represent the free space and obstacle.
- Figure 2 represents the RRT progress.


## RRT_variant (including RRT, RRT_star, and RRT_grad_heuristic)

~~~
run main.m
~~~

Note: delete the file "RRT/rand_state.mat" if you changed **start_coords** or **end_coords**. It saves random number generator to have repeatable samples. So if you want to generate new path, you should also delete it.

Controlling parameters
(in RRT_variant/RRT/RRT.m)

- vis1, vis2, vis3, vis4: whether to plot figure 1, 2, 3, 4.
- NN_check: when equals 1, checking collision using NN; when equals 0, checking collision using traditional method.
- grad_heuristic: when equals 1, introducing grad heuristic.
- RRT_star: when equals 1, runing RRT* ; when equals 0, runing RRT.

Four figures will be created:

- Figure 1 visualises created manipulator and its joint-space, where white and black area represent the free space and obstacle.
- Figure 2 represents the search progress in configuration space, where red point '*' denotes collied point, arrow visualizes calculated grad, red point 'o' represents generated new point using grad heuristic (if grad_heuristic == 1), and green lines denote rewriten edges (if RRT_star == 1).
- Figure 3 represents the search progress in task space.
- Figure 4 represents the result of NN when colliding.

## RRT_7D (Can plan in any dimension case)

~~~
run main.m
~~~

Note: you can set starting and end pose by **cfg.start_coords** and **cfg.end_coords**.

Controlling parameters
(in mai.m)

- cfg.dim: planning in cfg.dim-D case.
- cfg.circle_obs: when equals to true, assuming the obstacle as a sphere;  when equals to false, dividing it to many points.
- cfg.display1, cfg.display2, cfg.display3: whether to plot figure 1, 2, 3 during planning.
- cfg.display4: whether to plot figure after planning.
- cfg.maxSample: the maximum number of random samples.
- cfg.RRT_star: when equals to 1, running RRT*; when equals to 0, running RRT.
- cfg.NN_check: when equals to 1, checking collision using NN; when equals to 0, checking using traditional methods
- cfg.grad_heuristic: when equals to 1, introducing grad heuristic.

If cfg.display1, cfg.display2, cfg.display3 = 0, three figures will be created:
- Figure 1 represents the result of NN when colliding.
- Figure 2 represents the search progress in configuration space, where red point '*' denotes collied point, arrow visualizes calculated grad, red point 'o' represents generated new point using grad heuristic (if grad_heuristic == 1), and green lines denote rewriten edges (if RRT_star == 1).
- Figure 3 represents the search progress in task space, where red point '*' denotes collied point.

## Kinodynamic_RRT

~~~
run main.m
~~~

Note: you can set starting and end pose by **cfg.start_coords** and **cfg.end_coords**.

Controlling parameters
(in mai.m)

- cfg.display5: whether to plot DOF's s-t curve.
Others are as same as RRT_7D