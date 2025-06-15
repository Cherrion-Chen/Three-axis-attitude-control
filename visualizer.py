import numpy as np
import plotly.graph_objects as go
from scipy.spatial.transform import Rotation as R

def quaternion_to_rotation_matrix(q):
    """
    将四元数 q 转换为旋转矩阵。
    输入：
        q: numpy 数组，形状为 (4,)，表示四元数 [qw, qx, qy, qz]。
    输出：
        rot_matrix: numpy 数组，形状为 (3, 3)，表示旋转矩阵。
    """
    qw, qx, qy, qz = q
    rot_matrix = np.array([
        [1 - 2 * (qy**2 + qz**2),   2 * (qx * qy - qw * qz),   2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz),   1 - 2 * (qx**2 + qz**2),   2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy),   2 * (qy * qz + qw * qx),   1 - 2 * (qx**2 + qy**2)]
    ])
    return rot_matrix

def show(q):
    """输入四元数，展示空间姿态"""
    # 将四元数转换为旋转矩阵
    rot_matrix = quaternion_to_rotation_matrix(q)
    
    # 定义 RGB 三色箭头的初始方向（单位向量）
    arrows = np.eye(3)  # [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    
    # 应用旋转矩阵
    rotated_arrows = arrows @ rot_matrix
    
    # 创建 Plotly 图表
    fig = go.Figure()
    
    # 添加 RGB 三色箭头
    colors = ['red', 'green', 'blue']
    for i in range(3):
        fig.add_trace(go.Scatter3d(
            x=[0, rotated_arrows[i, 0]],
            y=[0, rotated_arrows[i, 1]],
            z=[0, rotated_arrows[i, 2]],
            mode='lines',
            line=dict(color=colors[i], width=5),
            name=f'Arrow {i+1}'
        ))
    
    # 设置图表布局
    fig.update_layout(
        title='Quaternion Rotation Visualization',
        scene=dict(
            xaxis=dict(range=[-1.5, 1.5], autorange=False),
            yaxis=dict(range=[-1.5, 1.5], autorange=False),
            zaxis=dict(range=[-1.5, 1.5], autorange=False),
            aspectmode='cube'
        )
    )
    
    # 显示图表
    fig.show()

def create_animation_frames(q_list):
    """生成动画帧数据"""
    frames = []
    for i, q in enumerate(q_list):
        rot_matrix = quaternion_to_rotation_matrix(q)
        rotated_arrows = np.eye(3) @ rot_matrix
        
        frame = go.Frame(
            data=[
                go.Scatter3d(
                    x=[0, rotated_arrows[0,0]], y=[0, rotated_arrows[0,1]], z=[0, rotated_arrows[0,2]],
                    mode='lines', line=dict(color='red', width=5), name='X轴'
                ),
                go.Scatter3d(
                    x=[0, rotated_arrows[1,0]], y=[0, rotated_arrows[1,1]], z=[0, rotated_arrows[1,2]],
                    mode='lines', line=dict(color='green', width=5), name='Y轴'
                ),
                go.Scatter3d(
                    x=[0, rotated_arrows[2,0]], y=[0, rotated_arrows[2,1]], z=[0, rotated_arrows[2,2]],
                    mode='lines', line=dict(color='blue', width=5), name='Z轴'
                )
            ],
            name=f'frame_{i}'
        )
        frames.append(frame)
    return frames

def anim(q_list, s=0.01):
    """输入一系列四元数，生成旋转动画"""
    frame_duration = s * 1000
    # 创建初始图形
    fig = go.Figure(
        data=[
            go.Scatter3d(x=[0,1], y=[0,0], z=[0,0], mode='lines', 
                        line=dict(color='red', width=5), name='X轴'),
            go.Scatter3d(x=[0,0], y=[0,1], z=[0,0], mode='lines', 
                        line=dict(color='green', width=5), name='Y轴'),
            go.Scatter3d(x=[0,0], y=[0,0], z=[0,1], mode='lines', 
                        line=dict(color='blue', width=5), name='Z轴')
        ],
        layout=go.Layout(
            scene=dict(
                xaxis=dict(range=[-1.5,1.5]), 
                yaxis=dict(range=[-1.5,1.5]), 
                zaxis=dict(range=[-1.5,1.5]),
                aspectmode='cube'
            ),
            updatemenus=[dict(
                type="buttons",
                buttons=[
                    dict(
                        label="播放",
                        method="animate",
                        args=[None, {"frame": {"duration": frame_duration}}]
                    ),
                    dict(
                        label="重播",
                        method="animate",
                        args=[None, {"frame": {"duration": frame_duration}, 
                                    "fromcurrent": False, "mode": "immediate"}]
                    )
                ]
            )]
        ),
        frames=create_animation_frames(q_list)
    )
    
    fig.update_layout(
        title_text="四元数旋转动画",
        showlegend=True,
        scene_camera=dict(eye=dict(x=1.5, y=1.5, z=1.5))
    )
    
    fig.show()

# 示例使用
if __name__ == "__main__":
    # 示例四元数
    q = np.array([0.7071, 0.0, 0.7071, 0.0])  # 90度绕y轴旋转
    #q = np.array([1., 0, 0, 0])
    #show(q)
    
    # 示例动画
    q_list = [
        np.array([1.0, 0.0, 0.0, 0.0]),  # 初始无旋转
        np.array([0.7071, 0.0, 0.7071, 0.0]),  # 90度绕z轴旋转
        np.array([0.0, 0.0, 1.0, 0.0]),  # 180度绕z轴旋转
        np.array([-0.7071, 0.0, 0.7071, 0.0]),  # 270度绕z轴旋转
        np.array([-1.0, 0.0, 0.0, 0.0])  # 360度绕z轴旋转
    ]
    anim(q_list, s=0.5)
