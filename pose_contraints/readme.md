# 关于姿态约束的相关求导
本段程序为在添加se3姿态约束的过程中一些求导过程以及相应的优化方法.
## 1. 问题描述
在构建地图的过程中, 通常情况下会有一层底图(root), 其他图层的底图需要在这层底图的先验位置的基础上构建, 保证所有层的底图处于同一个尺度下. 因此对于估计位置添加先验位姿.

定位当前估计位姿为$T_{wr}$, $T$为4x4矩阵, 表示旋转和平移, $w$表示world, $r$表示robot. 有先验位姿$\hat{T}_{wr}$. 那么构建误差方程:
$$
e = log(T_{rw} \cdot \hat{T}_{wr})^\vee
$$

本文中$(.)^\wedge$为反对称, $(.)^\vee$与反对称运算相反.

### 1.1 对于局部扰动求导
下面对于求解误差$e$对于局部扰动$\xi_r$的导数
$$
\begin{aligned}
\frac{\partial e}{\partial \xi_r} &= \frac{log(exp(\xi_r^\wedge)T_{rw} \cdot \hat{T}_{wr})^\vee}{\partial \xi_r} \\
& \approx \frac{log(exp(\xi_r^\wedge))^\vee}{\partial \xi_r} \\
&= I
\end{aligned}
$$
这里认为$T_{rw} \cdot \hat{T}_{wr}$的误差很小, 得到雅克比矩阵.

### 1.2 对于全局扰动求导
下面对于求解误差$e$对于全部扰动$\xi_w$的导数
$$
\begin{aligned}
\frac{\partial e}{\partial \xi_w} &= \frac{log(T_{rw} \cdot exp(\xi_w^\wedge) \cdot \hat{T}_{wr})^\vee}{\partial \xi_w} \\
& = \frac{log(T_{rw} \hat{T}_{wr}\cdot \hat{T}_{rw}exp(\xi_r^\wedge)\hat{T}_{wr})^\vee}{\partial \xi_w} \\
& \approx \frac{log(\hat{T}_{rw}exp(\xi_w^\wedge)\hat{T}_{wr})^\vee}{\partial \xi_w} \\
& = \frac{log\left(exp\left((\hat{\mathcal{T}}_{rw}\xi_w)^\wedge\right)\right)^\vee}{\partial \xi_w} \\
&=\frac{\hat{\mathcal{T}}_{rw}\xi_w}{\partial \xi_w} = \hat{\mathcal{T}}_{rw}
\end{aligned}
$$
其中$\hat{\mathcal{T}}_{rw}$是$\hat{T}_{rw}$的伴随阵.
$$
\begin{aligned}
T &= \begin{bmatrix}
R & t \\
0 & 1  
\end{bmatrix}_{4\times4} \\
\mathcal{T} &= \begin{bmatrix}
R & t^\wedge  \\
0 & R
\end{bmatrix}_{6\times6} 
\end{aligned}
$$

### 1.3 通过最小二乘求解
最小二乘最终求得的对应关系为
$$
\begin{aligned}
(J^T \Omega J) \Delta x + J^T \Omega e = 0\\
(J^T \Omega J) \Delta x = - J^T \Omega e
\end{aligned}
$$
本问题中$J = \frac{\partial e}{\partial \xi}$, J为2x6的矩阵. $\Omega$为权重, 为2x2的对角阵.

### 1.4 对于全局扰动和局部扰动的不同迭代方法
对于局部扰动和全局扰动最终的迭代略有不同, 另优化后局部扰动计算的步长为$\Delta \xi_r$, 全局扰动为$\Delta \xi_w$, 那么:
$$
\begin{aligned}
T_{rw} &= exp(\Delta \xi_r^\wedge)T_{rw} \\
T_{rw} &= T_{rw}exp(\Delta \xi_w^\wedge)
\end{aligned}
$$