# Three axis attitude control
This is a attitude dynamics framework developed by a senior-3 student, who has just finished his high school, with the assistance of DeepSeek——R1 and Kimi.

Two Dynamic Link libraries are provided to accelerate the calculation. "32", "64" refers to the data structure, the former one is based on float, while the latter is based on double. Source codes are available in the corresponding folders.

To create a Python API, "dynamics.py" is created. It provides several important functions based on the "dynamics32.dll". It provides a torch.autograd.Function class, which enables you to train a intelligent controller. 

PD_control uses cascade PID algorithm to control the attitude.
