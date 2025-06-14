from ctypes import *
import numpy as np
import torch

dll = cdll.LoadLibrary("./dynamics32.dll")

# 设置函数的参数类型
dll.set_I.argtypes = (c_float, c_float, c_float)
dll.f.argtypes = (POINTER(c_float), POINTER(c_float))
dll.gradient.argtypes = (POINTER(c_float), POINTER(c_float))

# 设置`add`函数的返回类型
dll.f.restype = POINTER(c_float)
dll.gradient.restype = POINTER(c_float)

A = torch.cat((torch.zeros((4,4)), torch.eye(4)), dim=1)

def norm_q(x: torch.FloatTensor, epsilon: float=1e-8) -> torch.FloatTensor:
    x = x.detach().clone()
    x, left = x[:, :4], x[:, 4:]
    s = torch.sum(x*x, dim=1)
    s = torch.sqrt(s).unsqueeze(dim=-1) + epsilon
    x = x / s
    return torch.cat((x, left), dim=1)

def set_I(Ix, Iy, Iz):
    dll.set_I(Ix, Iy, Iz)

def f_one(x_u: torch.FloatTensor) -> list:
    # torch转numpy
    inp = x_u.detach().numpy().astype(np.float32)
    
    # 创建 ctypes 的 c_double 指针
    inx = (c_float * 8)(*(inp[0: 8]))
    inu = (c_float * 3)(*(inp[8:]))
    
    op = dll.f(inx, inu)
    return op[:4]


def gradient_one(x_u: torch.FloatTensor) -> np.ndarray:
    # torch转numpy
    inp = x_u.detach().numpy().astype(np.float32)
    
    # 创建 ctypes 的 c_double 指针
    inx = (c_float * 8)(*(inp[0: 8]))
    inu = (c_float * 3)(*(inp[8:]))
    
    op = dll.gradient(inx, inu)
    op = np.array(op[:44], dtype=np.float32).reshape(4, 11)
    return op

def f(x: torch.FloatTensor, u: torch.FloatTensor) -> torch.FloatTensor:
    x_u = torch.cat((x, u), dim=1)
    op = map(f_one, x_u)
    return torch.tensor(tuple(op), dtype=torch.float32)
    
def gradient(x: torch.FloatTensor, u:torch.FloatTensor) -> torch.FloatTensor:
    x_u = torch.cat((x, u), dim=1)
    op = map(gradient_one, x_u)
    op = np.array(tuple(op), dtype=np.float32)
    return torch.tensor(op, dtype=torch.float32)

class state_equation(torch.autograd.Function):
    @staticmethod
    def forward(ctx, x: torch.FloatTensor, u: torch.FloatTensor) -> torch.FloatTensor:
        ctx.save_for_backward(gradient(x, u))
        op = f(x, u)
        v = x[:, 4:8]
        op = torch.cat((v, op), dim=1)
        return op
    
    @staticmethod
    def backward(ctx, grad_output: torch.FloatTensor) -> torch.FloatTensor:
        batch_size = len(grad_output)
        grad, = ctx.saved_tensors
        grad = grad_output[:, 4:8].unsqueeze(dim=1) @ grad
        grad = grad.squeeze(dim=1)
        grad_x, grad_u = torch.split(grad, (8, 3), dim=-1)
        grad_x += torch.cat((torch.zeros((batch_size,4)), grad_output[:, :4]), dim=1)
        return grad_x, grad_u
        
if __name__ == "__main__":
    set_I(1,1,1)
    x=norm_q(torch.tensor([[1.,2,3,4,.1,.2,0,0],
                           [2,3,4,5,.3,.4,1.2,1.7],
                           [10,1,1,3,0,.1,.2,.5]])).detach().requires_grad_(True)
    u=torch.tensor([[1.,2,3],
                    [6,7,8],
                    [6,2,0]], requires_grad=True)
    y = state_equation.apply(x, u)
    y = (y*y).sum()
    y.backward()
    print(x.grad)
    print("")
    print(u.grad)
