# Three axis attitude control
This is a attitude dynamics framework developed by a senior-3 student, who has just finished his high school, with the assistance of DeepSeek——R1 and Kimi.

Two Dynamic Link libraries are provided to accelerate the calculation. "32", "64" refers to the data structure, the former one is based on float, while the latter is based on double. Source codes are available in the corresponding folders.

To create a Python API, "dynamics.py" is created. It provides several important functions based on the "dynamics32.dll". It provides a torch.autograd.Function class, which enables you to train a intelligent controller. 

PD_control uses cascade PID algorithm to control the attitude, while Adaptive_PID_control uses adaptive PID algorithm.

Through several trials, I noticed that the algorithm works well only under specific targets, including but not limited to (1, 0, 0, 0), which means no rotation. So you can reduce your control problem imto a regulation problem.
