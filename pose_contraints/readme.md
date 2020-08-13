# 旋转约束与姿态约束的误差方程及其相关求导
## 1. 问题描述
在构建地图的过程中, 可能会存在先验的姿态或者先验的位置, 这些先验信息可能来自不同的传感器, 也可能来自同一传感器的不同算法. 比如使用激光雷达建图, 提供先验位置给视觉建图; 或者使用线特征估计Manhattan World进而提供先验姿态. 本文先后求解对于姿态和位置约束的雅克比矩阵

## 2. 姿态约束
已知当前估计位置为$R_{wr}$为$3\times3$的矩阵, 表示平移, $w$表示world, $r$表示robot. 此时有先验姿态$\hat{R}_{wr}$. 那么构造误差方程为:
$$
\begin{aligned}
e &= log(R_{wr}^T \cdot \hat{R}_{wr})^\vee \\
&= log(R_{rw} \cdot \hat{R}_{wr})^\vee
\end{aligned}
$$
本文中$^\wedge$为反对成运算, $^\vee$为其反运算.

## 2.1 对于局部扰动的求导
下面求解误差$e$对于局部扰动$\phi_r$的导数
$$
\begin{aligned}
\frac{\partial e}{\partial \phi_r} &=  
\frac{\partial \left(log(exp(\phi_r^\wedge)R_{rw} \cdot \hat{R}_{wr})^\vee \right)}{\partial \phi_r} \\
&\approx \frac{\partial \left(log(exp(\phi_r^\wedge))^\vee \right)}{\partial \phi_r} \\
&= I
\end{aligned}
$$
这里认为$R_{rw} \cdot \hat{R}_{wr} \approx I$, 进而求得雅克比矩阵.

## 2.2 对于全局扰动的求导
下面求解误差$e$关于全局扰动$\phi_r$的求导
$$
\begin{aligned}
\frac{\partial e}{\partial \phi_w} &=  
\frac{\partial \left(R_{rw} \cdot log(exp(\phi_w^\wedge) \cdot\hat{R}_{wr})^\vee \right)}{\partial \phi_w} \\
&=  \frac{\partial \left(R_{rw} \cdot log(exp(\phi_w^\wedge) \cdot\hat{R}_{rw}^T)^\vee \right)}{\partial \phi_w} \\
&= \frac{\partial \left(log(R_{rw}\hat{R}_{rw}^T \cdot \boxed{\hat{R}_{rw} exp(\phi_w^\wedge) \hat{R}_{rw}^T})^\vee \right)}{\partial \phi_w} \\
&\approx \frac{\partial \left(log \left(exp((\hat{R}_{rw}\cdot \phi_w)^{\wedge})\right)^\vee\right)}{\partial  \phi_w} \\
&=\frac{\partial \hat{R}_{rw}\phi_w}{\partial \phi_w} \\
&= \hat{R}_{rw}
\end{aligned}
$$
参考[State Estimation for Robotics](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf)表格7-2中的公式$exp((Ru)^\wedge) = R\cdot exp(u^\wedge)\cdot R^T$. 另外近似$R_{rw} \cdot \hat{R}_{wr} \approx I$.

## 3. 姿态约束的误差方程及其求导
定位当前估计位姿为$T_{wr}$, $T$为4x4矩阵, 表示旋转和平移, $w$表示world, $r$表示robot. 有先验位姿$\hat{T}_{wr}$. 那么构建误差方程:
$$
e = log(T_{rw} \cdot \hat{T}_{wr})^\vee
$$

本文中$(.)^\wedge$为反对称, $(.)^\vee$与反对称运算相反.

### 3.1 对于局部扰动求导
下面对于求解误差$e$对于局部扰动$\xi_r$的导数
$$
\begin{aligned}
\frac{\partial e}{\partial \xi_r} &= \frac{\partial \left(log(exp(\xi_r^\wedge)T_{rw} \cdot \hat{T}_{wr})^\vee\right)}{\partial \xi_r} \\
& \approx \frac{\partial \left(log(exp(\xi_r^\wedge))^\vee\right)}{\partial \xi_r} \\
&= I
\end{aligned}
$$
这里认为$T_{rw} \cdot \hat{T}_{wr}$的误差很小, 得到雅克比矩阵.

### 3.2 对于全局扰动求导
下面对于求解误差$e$对于全部扰动$\xi_w$的导数
$$
\begin{aligned}
\frac{\partial e}{\partial \xi_w} &= \frac{\partial \left(log(T_{rw} \cdot exp(\xi_w^\wedge) \cdot \hat{T}_{wr})^\vee\right)}{\partial \xi_w} \\
& = \frac{\partial \left(log(T_{rw} \hat{T}_{wr}\cdot \hat{T}_{rw}exp(\xi_r^\wedge)\hat{T}_{wr})^\vee\right)}{\partial \xi_w} \\
& \approx \frac{\partial \left(log(\hat{T}_{rw}exp(\xi_w^\wedge)\hat{T}_{wr})^\vee\right)}{\partial \xi_w} \\
& = \frac{\partial \left(log\left(exp\left((\hat{\mathcal{T}}_{rw}\xi_w)^\wedge\right)\right)^\vee\right)}{\partial \xi_w} \\
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
注意以上伴随阵的表达方式和$\xi = \begin{bmatrix}\phi & \tau\end{bmatrix}^T$对应, 其中$\xi$为6维向量, $\phi$为三维, 表示旋转. $\tau$为两维, 表示平移.

## 4 最小二乘求解
最小二乘最终求得的对应关系为
$$
\begin{aligned}
(J^T \Omega J) \Delta x + J^T \Omega e = 0\\
(J^T \Omega J) \Delta x = - J^T \Omega e
\end{aligned}
$$
本问题中$J = \frac{\partial e}{\partial \xi}$, J为2x6的矩阵. $\Omega$为权重, 为2x2的对角阵.

### 4.1 对于全局扰动和局部扰动的不同迭代方法
对于局部扰动和全局扰动最终的迭代略有不同, 另优化后局部扰动计算的步长为$\Delta \xi_r$, 全局扰动为$\Delta \xi_w$, 那么:
$$
\begin{aligned}
T_{rw} &= exp(\Delta \xi_r^\wedge)T_{rw} \\
T_{rw} &= T_{rw}exp(\Delta \xi_w^\wedge)
\end{aligned}
$$

姿态同理.