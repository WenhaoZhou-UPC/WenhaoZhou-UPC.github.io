---
layout: post
title: 2024-4-1-LARVIO算法学习总结(二)
---


# LARVIO算法学习总结(二)



接上一篇文章，讲完imu初始化后，完美接着讲下面的章节。

#### 3.3 IMU状态传播


​ IMU状态传播主要是利用两帧图像之间的所有imu数据(包括加速度和角速度)，对当前的状态向量和协方差矩阵进行迭代预测。

##### 3.3.1 IMU运动模型推导

3.3.1.1 状态真实值的系统模型

各状态量真实值之间的物理关系如下：

$$\left\{\begin{matrix} _G^I\dot{q}(t)=\frac12\Omega(\omega(t))_G^Iq(t)\\ ^G\dot{v}_I(t)={}^Ga_I(t)\\ \quad{}^G\dot{p}_I(t)={}^Gv_I(t)\\ \quad\dot{b}_g(t)=n_{wg}(t)\\ \quad\dot{b}_a(t)=n_{wa}(t)\\ \end{matrix}\right.$$

其中$^Ga_I$是Global系下的加速度，$w=\begin{bmatrix} w_x&w_y&w_z \end{bmatrix}^T$是IMU系的角速度，$\Omega(\omega)=\begin{bmatrix}-\lfloor\omega_\times\rfloor&\omega\\-\omega^T&0\end{bmatrix},$$\quad\lfloor\omega_\times\rfloor=\begin{bmatrix}0&-\omega_z&\omega_y\\\omega_z&0&-\omega_x\\-\omega_y&\omega_x&0\end{bmatrix}$

IMU观测量的定义如下： 

$$\begin{aligned} &\omega_{m}= \left.\omega+C\left(\begin{smallmatrix}I\\G\end{smallmatrix}\right.q\right)\omega_{G}+b_{g}+n_{g} \\ &a_{m}= C\left(_{G}^{I}q\right)\left({}^{G}a_{I}-{}^{G}g+2\left[\omega_{G}\times\right]{}^{G}v_{I}+\left\lfloor\omega_{G}\times\right\rfloor{}^{2G}p_{I}\right)+b_{a}+n_{a} \end{aligned}$$

其中$C(.)$表示四元数旋转矩阵，$n_g,n_a$为IMU测量误差（高斯白噪声）,在论文[2]考虑了地球自旋速度，所以多了一项$w_G$，而LARVIO则忽略了该项，可以简化为：

$$\omega_m=\omega+b_g+n_g$$



$$a_m=C(_G^Iq)(^Ga_I-^Gg)+b_a+n_a$$


3.3.1.2 **状态估计量的系统模型**

状态估计量的系统模型： 

$$\left\{\begin{matrix} _G^I\dot{\hat{q}}=\frac{1}{2}\Omega(\hat{\omega})_G^I\hat{q}\\ {}^G\dot{\hat{v}}_I=C\bigl(\begin{matrix}I\\G\end{matrix}\hat{q}\bigr)^T\hat{a}+{}^Gg\\ \quad{}^G\dot{\hat{p}}_I={}^G\hat{v}_I\\ \quad\dot{\hat{b}}_g=0_{3\times1}\\ \quad\dot{\hat{b}}_a=0_{3\times1} \end{matrix}\right.$$

 其中，加速度和角速度估计值定义如下： 

$$\begin{array}{l}\hat{\omega}=\omega_m-\hat{b}_g\Rightarrow\omega=\hat{\omega}-\tilde{b}_g-n_g\\\hat{a}=a_m-\hat{b}_a\Rightarrow{}^Ga_I={}_G^IR^T(\hat{a}-\tilde{b}_a-n_a)+{}^Gg\end{array}$$


3.3.1.3 IMU误差状态的**运动模型推导**

根据状态估计量的系统模型推导IMU误差状态的运动模型

$$\dot{\tilde{X}}_{IMU}=F\tilde{X}_{IMU}+Gn_{IMU}$$

具体表达式为：


![./figures\ccecc7161d73482c9998cccede3c8f0e.png](./figures\ccecc7161d73482c9998cccede3c8f0e.png)


其中$n_{IMU}$向量中依次是陀螺仪误差、陀螺仪bias误差、加速计误差、加速计bias误差.

##### 3.3.2 **状态向量预测**


​直接对**状态估计量**进行迭代预测 (而非误差状态向量)，协方差是根据**误差状态**的运动模型进行迭代，每一帧的imu数据预测量包括姿态$q$，速度$v$，位置$p$，$b_a$和$b_g$保持不变。
3.3.2.1 IMU预积分

**定义**：给定微分方程$y'(t)=f(t,y(t))$以及初值$y(t_0)$, 求$\Delta t$时刻后的

$$y(t_0+\Delta t)=y(t_0)+\int_{t_0}^{t_0+\Delta t}y'(t)dt$$

举例：假设已知当前的位置$p_0$和速度曲线$v(t)$，则位置曲线为$p(t)=p_0+\int_{0}^{t}v(t)dt$。但实际情况并没有解析式$v(t)$，只能从传感器中获取离散的采样值，因此需要通过离散积分尽可能逼近真实的连续积分。

​那么要如何去逼近真实情况呢？基本的解决思路是：用一个恒定数值$k$来近似$(t,t+\Delta t)$时间段内的微分$y'(t)$，相当于曲线$y(t)$在$\Delta t$时间段内近似为一条直线，则数值积分为：

$$y_{n+1}=y_n+\Delta t×k_n$$

​由于其结果是通过逼近来近似的，那么就会存在一定的误差，那么要如何尽可能的减少误差呢？

+ 让离散时间间隔$\Delta t$尽可能的小。可以通过提高采样频率。+ 计算准确的斜率$k_n$，主要分为Euler积分、Mid-Point积分和Runge-Kutta积分


**常用数值积分方法**：常用的三种数值积分：Euler积分、Mid-Point积分、Runge-Kutta积分(4阶)。它们的精度依次从低到高、计算量依次从小到大。令$k_2$的权重为1，其他为0，RK4就退化为Mid-Point法；令$k_1$的权重为1，其他为0，RK4就退化为Euler积分。

**1.Euler积分**

​Euler积分利用$t_n$时刻的斜率来作为$t_n$到$t_{n+1}$整段的斜率(前向Euler)

$$k_n=y'(t)=f(t_n,y_n)$$

Euler积分计算量很小，但近似误差会比较大。

**2.Mid-Point积分**

​Mid-Point积分利用中点$t_n+\frac{1}{2}\Delta t$时刻的斜率$t'(t_n+\frac{1}{2}\Delta t)$来近似整段斜率：

$$k_n=y'(t_n+\frac{\Delta}{2})=f(t_n+\frac{\Delta t}{2},y(t_n,\frac{\Delta t}{2}))\approx f(t_n+\frac{\Delta t}{2},y_n+\frac{\Delta t}{2}×f(t_n,y_n))$$



由于$y(t_n,\frac{\Delta t}{2})$是未知的，所以先通过Euler积分近似$y(t_n,\frac{\Delta t}{2})\approx y_n+\frac{\Delta t}{2}×f(t_n,y_n)$,用中点斜率来近似整段斜率显然比比Euler积分用端点斜率来近似要更合理。

**3.Runge-Kutta积分**

​常用的是4阶Runge-Kutta积分, 即RK4积分. 它将多个斜率加权来近似整段斜率

$$y_{n+1}=y_n+\frac{\Delta t}{6}(k_1+2k_2+2k_3+k_4)$$

其中$k_1=f(y_n,t_n),k_2=f(t_n+\frac{\Delta t}{2},y_n+\frac{\Delta t}{2}k_1),k_3=f(t_n+\frac{\Delta t}{2},y_n+\frac{\Delta t}{2}k_2),k_4=f(t_n+\Delta t,y_n+\Delta tk_3)$.

+ $k_1$是起始点的斜率，$k_2,k_3$都是中点斜率的“近似”，$k_4$是结束点斜率的“近似”。注意这里的近似，中点和结束点的真实斜率未知，这是由于中点和结束点的真实$y(t_n+\frac{1}{2}\Delta t)$和$y_{n+1}$是未知的，+ $k_2$中近似为$y(t_n+\frac{\Delta t}{2})=y_n+\frac{\Delta t}{2}×k_1$, 相当于前向Euler积分, 取的$[t_n,t_n+\frac{\Delta t}{2}]$的前点斜率;+ $k_3$中近似为$y(t_n+\frac{\Delta t}{2})=y_n+\frac{\Delta t}{2}×k_2$, 相当于后向Euler积分, 取的的后点斜率(注意, 这里用$k_2$来近似后点斜率);+ $k_4$中近似为$y_{n+1}=y_n+\Delta t×k_3$, 相当于Mid-Point积分, 取的中点的斜率. (注意, 这里用$k_3$来近似中点斜率).


​RK4加权将中点斜率$k_2,k_3$的权重设为2，两端的斜率$k_1,k_4$设为1。它的近似精度更高，但显然计算量也更大。*RK4中的$k_1$就是Euler法的斜率，$k_2$就是中点法的斜率。

​LARVIO中所采用的预积分方法就是Runge-Kutta积分。
3.3.2.2 IMU状态向量预测
1.姿态预测

​姿态四元数采用的0阶积分(假设角速度在单位时间内恒定不变)，推导过程见论文[3]中的1.6.1小节。

​ 当$|\hat{w}|>10^{-5}$时：$_G^I\hat{q}(t+\Delta t)=\left(cos\left(\frac{|\hat{\omega}|}{2}\Delta t\right)\cdot I_{4\times4}+\frac{1}{|\hat{\omega}|}sin\left(\frac{|\hat{\omega}|}{2}\Delta t\right)\cdot\Omega(\hat{\omega})\right)_G^I\hat{q}(t)$

​ 当$|\hat{w}|\le10^{-5}$时：$_G^I\hat{q}(t+\Delta t)=\left(I_{4\times4}+\frac{\Delta t}{2}\Omega(\hat{\omega})\right)_G^I\hat{q}(t)$

其中$\hat{\omega}=\omega_m-\hat{b}_g$
2.位置预测

​采用的4阶Runge-Kutta积分：

$$\left\{\begin{matrix} \hat{p}(t+\Delta t)=\hat{p} +\frac{\Delta t}6(k_{p_1}+2k_{p_2}+2k_{p_3}+k_{p_4}) \\ k_{p_1} =\hat{v}(t) \\ k_{p_2} =\hat{v}(t)+k_{v_1}\Delta t/2\\ k_{p_3} =\hat{v}(t)+k_{v_2}\Delta t/2\\ k_{p_4} =\hat{v}(t)+k_{v_3}\Delta t \end{matrix}\right.$$


3.速度预测

​ 采用的4阶Runge-Kutta积分： 

$$\left\{\begin{matrix} \hat{v}(t+\Delta t) )=\hat{v}(t)+\frac{\Delta t}6(k_{v_1}+2k_{v_2}+2k_{v_3}+k_{v_4}) \\ k_{v_1}={}_G^I\hat{R}(t)\hat{a}+g \\ k_{v_2}=\frac IG\hat{R}(t+\Delta t/2)\hat{a}+g\\ k_{v_3} =\frac IG\hat{R}(t+\Delta t/2)\hat{a}+g\\ k_{v_4} =\frac IG\hat{R}(t+\Delta t)\hat{a}+g \end{matrix}\right.$$



##### 3.3.3 IMU状态协方差预测


​协方差是根据**误差状态**的运动模型进行迭代的，LARVIO提出了一种闭式的IMU误差状态转移公式，采用了**双采样旋转矩阵拟合法**求解其中的积分项，得到一个完全线性化的闭式形式公式。详细推导过程可见论文[4]。从而计算出误差状态转移矩阵$\mathbf{\Phi}(t_{k+1},t_k)$如下： 

$$\left.\mathbf{\Phi}\left(k+1,k\right)=\left[\begin{array}{ccccc}\mathbf{I}_{3\times3}&\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}&\mathbf{\Phi}_{\theta b_{g}}&\mathbf{0}_{3\times3}\\\mathbf{\Phi}_{v\theta}&\mathbf{I}_{3\times3}&\mathbf{0}_{3\times3}&\mathbf{\phi}_{vb_{g}}&\mathbf{\phi}_{vb_{a}}\\\mathbf{\Phi}_{p\theta}&\Delta t\mathbf{I}_{3\times3}&\mathbf{I}_{3\times3}&\mathbf{\phi}_{pb_{g}}&\mathbf{\phi}_{pb_{a}}\\\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}&\mathbf{I}_{3\times3}&\mathbf{0}_{3\times3}\\\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}&\mathbf{I}_{3\times3}\end{array}\right.\right]$$



$$\left.\mathbf{n}_{IMU}\left(k\right)=\left[\begin{array}{ccccc}{\mathbf{n}_{\theta k}^{T}}&{\mathbf{n}_{vk}^{T}}&{\mathbf{n}_{pk}^{T}}&{\mathbf{n}_{b_{g}k}^{T}}&{\mathbf{n}_{b_{a}k}^{T}}\\\end{array}\right.\right]^{T}.$$



其状态协方差预测公式如下： 其中更新前的协方差矩阵为

$$\mathbf{P}_{k|k}=\begin{bmatrix}\mathbf{P}_{II_{k|k}}&\mathbf{P}_{IC_{k|k}}\\\mathbf{P}_{IC_{k|k}}^T&\mathbf{P}_{CC_{k|k}}\end{bmatrix}_.$$

 更新后的协方差矩阵为

$$\mathbf{P}_{k+1|k}=\begin{pmatrix}\mathbf{P}_{II_{k+1|k}}&\mathbf{\Phi}_{k}\mathbf{P}_{IC_{k|k}}\\\mathbf{P}_{IC_{k|k}}^\top\mathbf{\Phi}_{k}^\top&\mathbf{P}_{CC_{k|k}}\end{pmatrix}$$

其中$\mathbf{P}_{II_{k+1|k}}=\mathbf{\Phi}_k\mathbf{P}_{II_{k|k}}\mathbf{\Phi}_k^\top+\mathbf{Q}_k$，$\mathbf{P}_k=\Phi(t_{k+1},t_k)$，$\mathbf{Q}_k=\int_{t_k}^{t_{k+1}}\mathbf{\Phi}(t_{k+1},\tau)\mathbf{GQG\Phi}(t_{k+1},\tau)^\top d\tau \approx \mathbf{\Phi}(t_{k+1},t_k)\mathbf{GQG\Phi}(t_{k+1},t_k)^\top$.

​每来一帧imu数据都需要进行imu状态传播、imu状态协方差更新。

#### 3.4 状态向量增广与剪枝


​Larvio本质上时hybrid VIO，其基于误差状态EKF，其系统状态主要分为以下三大部分：

$$x=\begin{bmatrix}x_{\mathrm{leg}}^\mathrm{T},x_{\mathrm{pos}}^\mathrm{T},x_{\mathrm{aug}}^\mathrm{T}\end{bmatrix}^\mathrm{T}$$

其中$x_{leg}$为legacy状态向量，包括了当前imu姿态、速度、imu零偏、相机与imu的外参和时延；$x_{pos}$为新增的姿态状态向量，包括一个滑窗内所有imu姿态；$x_{aug}$为增广的feature状态向量。

##### 3.4.1 初始状态向量


​legacy状态向量的定义为： 

$$x_\mathrm{leg}=\begin{bmatrix}q_\mathrm{w}^\mathrm{bT},v_\mathrm{b}^\mathrm{wT},p_\mathrm{b}^\mathrm{wT},b_\mathrm{g}^\mathrm{T},b_\mathrm{a}^\mathrm{T},q_\mathrm{b}^\mathrm{cT},p_\mathrm{c}^\mathrm{bT},t_\mathrm{d}\end{bmatrix}^\mathrm{T}$$

其中$q_w^b$为世界坐标系到imu坐标系的旋转矩阵、$v_b^w、p_b^w$ 为imu的速度和位置、$b_g、b_a$为陀螺仪和加速度计的零偏、$q_b^c、p_c^b$为相机与imu的外参、$t_d$为i相机与imu的时延。

​其误差状态向量为： 

$$\delta\boldsymbol{x}_\mathrm{leg}=\begin{bmatrix}\delta\boldsymbol{\theta}_\mathrm{b}^\mathrm{wT},\delta\boldsymbol{v}_\mathrm{b}^\mathrm{wT},\delta\boldsymbol{p}_\mathrm{b}^\mathrm{wT},\delta\boldsymbol{b}_\mathrm{g}^\mathrm{T},\delta\boldsymbol{b}_\mathrm{a}^\mathrm{T},\delta\boldsymbol{\theta}_\mathrm{c}^\mathrm{bT},\delta\boldsymbol{p}_\mathrm{c}^\mathrm{bT},\delta\boldsymbol{t}_\mathrm{d}\end{bmatrix}^\mathrm{T}$$



##### 3.4.2 状态向量扩增


​在Larvio中，除了会将imu姿态加入到状态向量中，还会将跟踪长度超过滑动窗口的feature加入到状态向量。

​增广的imu姿态通过一个滑动窗口将其数量限制为N，其状态向量如下： 

$$x_{\mathrm{pos}}=\left[q_{\mathrm{w}}^{\mathrm{b_1T}},p_{\mathrm{b_1}}^{\mathrm{wT}},q_{\mathrm{w}}^{\mathrm{b_2T}},p_{\mathrm{b_2}}^{\mathrm{wT}},\cdots,q_{\mathrm{w}}^{\mathrm{b_NT}},p_{\mathrm{b_N}}^{\mathrm{wT}}\right]^{\mathrm{T}}$$

其误差状态向量如下： 

$$\delta x_{\mathrm{pos}}=\begin{bmatrix}\delta\theta_{\mathrm{b_1}}^{\mathrm{wT}},\delta p_{\mathrm{b_1}}^{\mathrm{wT}},\delta\theta_{\mathrm{b_2}}^{\mathrm{wT}},\delta p_{\mathrm{b_2}}^{\mathrm{wT}},\cdots,\delta\theta_{\mathrm{b_N}}^{\mathrm{wT}},\delta p_{\mathrm{b_N}}^{\mathrm{wT}}\end{bmatrix}^{\mathrm{T}}$$

增广的feature状态向量如下： 

$$x_{\mathrm{aug}}=[\rho_1,\rho_2,\cdots,\rho_M]^\mathrm{T}$$

采用了一维逆深度参数化方法对其进行建模，具体建模方法可见论文[5]，其中$\rho_i$代表在第i个增广feature在它的host相机坐标系下的逆深度。其误差状态向量为： 

$$\delta x_{\mathrm{aug}}=[\delta\rho_{1},\delta\rho_{2},\cdots,\delta\rho_{M}]^{\mathrm{T}}$$

只有当该feature的跟踪长度大于滑动窗口的长度的时候才会添加到系统的状态向量中，并且feature的总数量也限制在了M个，其余的会被当做MSCKF feature。

##### 3.4.3 **协方差扩增**



![./figures\70a18acae6c64eb1a763d35274cd5825.png](./figures\70a18acae6c64eb1a763d35274cd5825.png)
 如上图所示，当imu姿态和feature的状态向量进行增广和剪枝时，相对应的协方差矩阵也会进行增广和剪枝。当imu姿态增广时，其对应的协方差矩阵更新如图Fig.2(a)所示。当滑动窗口的大小滑过N个imu时，那么就会删除掉最旧的imu姿态，进行相关的测量更新，并且删除对应的协方差块，如图Fig.2(b)所示。当feature的跟踪长度超过N时，它就会被添加到状态向量中，并且相对应的协方差矩阵也会进行更新，如图Fig.2（c）所示。当feature lost之后则会被剪枝掉，相对应的协方差矩阵更新如图Fig.2(d)所示。

##### 3.4.4 状态扩增的意义


​MSCKF中，IMU状态传播只更新IMU状态向量和其对应的协方差，与相机无关；而Measurement Updata的观测模型是残差相对于相机状态的观测模型，与IMU状态没有直接关联。因此**状态扩增在相机和IMU状态之间起一个桥梁的作用**，通过关联协方差 $P_{IC}$描述相机和IMU状态之间的关系，每一个相机状态都与IMU状态形成关联，这样在观测更新相机状态的同时，会间接更新IMU状态。

#### 3.5 测量更新


​在传统的hybrid VIO中的测量更新会涉及到三种特征残差，包括MSCKF特征残差、旧的增广特征残差和新的增广特征残差。而Larvio则新增了**ZUPT零速矫正中的零速、相同位置和相同旋转的残差量**，不过它只在静态场景下进行该测量更新。

##### 3.5.1 ZUPT零速矫正


​为了更好地处理静态场景，larvio提出了一种新的**闭式零速矫正**方法，它同样被建模为滤波器的测量更新，弥补了在静止情况下imu漂移的问题。
3.5.1.1 静态场景检测

​为了能够进行ZUPT操作，首先需要进行静态场景的检测，通常可以利用imu数据来检测当前是否处于静止状态，然而由于imu是存在噪声的，并且在一些退化运动中，如匀速运动，通过imu数据难以判断当前是处于运动还是静止的。而如果处于静止状态的话，图像匹配点之间的像素差异是非常小的，可以通过图像特征运动来区分当前的状态，检测出静态场景。

​Larvio中采用了一个比较简单的方法， 就是计算相邻图像中匹配特征点的像素差，并使用阈值来判断当前场景是否处于静止。然而，静态场景中移动的物体可能会导致检测失败，因为如果匹配的特征点是移动的物体，则会得到较大的像素差异。为了应对这种情况，提出了一种简单的策略：删除十个像素差最大的匹配对，并将剩余匹配对的最大像素差与阈值进行比较，以判断相机是否静止。
3.5.1.2 ZUPT测量更新

​只有检测到当前处于静止状态才会启动ZUPT测量更新，其他情况则还是按照传统的基于特征的测量更新。只要启动了ZUPT，则不管滑动窗口是否达到最大值，它都会直接删除第二新的IMU姿态，只更新**速度残差、位置残差和旋转残差**。基于这三者的约束更新当前的状态向量和协方差矩阵。

**1.速度残差**

理想速度观测和imu速度预测如下： 

$$\left.\left\{\begin{aligned}&z_{\mathrm{zupt\_v}}=\nu_{\mathrm{b}}^{\mathrm{w}}=0_{3\times1}\\&\hat{z}_{\mathrm{zupt\_v}}=\hat{\nu}_{\mathrm{b}}^{\mathrm{w}}\end{aligned}\right.\right.$$

测量更新的速度残差如下： 

$$r_{\mathrm{zupt}\_v}=z_{\mathrm{zupt}\_v}-\hat{z}_{\mathrm{zupt}_{v}}=-\hat{v}_{\mathrm{b}}^{\mathrm{w}}=v_{\mathrm{b}}^{\mathrm{w}}-\hat{v}_{\mathrm{b}}^{\mathrm{w}}=\delta v_{\mathrm{b}}^{\mathrm{w}}$$

 **2.位置残差** 理想位置观测和位置预测如下： 

$$\left.\left\{\begin{aligned}z_\mathrm{zupt-p}&=p_\mathrm{b}^\mathrm{w}(n)-p_\mathrm{b}^\mathrm{w}(n-1)=0_{3\times1}\\\hat{z}_\mathrm{zupt-p}&=\hat{p}_\mathrm{b}^\mathrm{w}(n)-\hat{p}_\mathrm{b}^\mathrm{w}(n-1)\end{aligned}\right.\right.$$

测量更新的位置残差如下： 

$$\begin{aligned} r_{\mathrm{zupt_{-p}}}& =z_{\mathrm{zupt\_p}}-\hat{z}_{\mathrm{zupt\_p}}=-\hat{p}_{\mathrm{b}}^{\mathrm{w}}(n)+\hat{p}_{\mathrm{b}}^{\mathrm{w}}(n-1) \\ &=\left(p_\mathrm{b}^\mathrm{w}(n)-\hat{p}_\mathrm{b}^\mathrm{w}(n)\right)-\left(p_\mathrm{b}^\mathrm{w}(n-1)-\hat{p}_\mathrm{b}^\mathrm{w}(n-1)\right) \\ &=\delta p_{\flat}^{\mathrm{w}}(n)-\delta p_{\flat}^{\mathrm{w}}(n-1) \end{aligned}$$

 **3.旋转残差**

理想的旋转观测和旋转预测如下： 

$$\left.\left\{\begin{aligned}&z_\mathrm{zupt\_q}=q_\mathrm{b}^\mathrm{w}(n)\otimes\left(q_\mathrm{b}^\mathrm{w}(n-1)\right)^{-1}=[1,0,0,0]^\mathrm{T}\\&\hat{z}_\mathrm{zupt\_q}=\hat{q}_\mathrm{b}^\mathrm{w}(n)\otimes\left(\hat{q}_\mathrm{b}^\mathrm{w}(n-1)\right)^{-1}\end{aligned}\right.\right.$$

测量更新的旋转残差如下(具体的推导过程可见论文[5])： 

$$\begin{aligned} r_{\mathrm{zupt\_q}}& =\left\{z_\mathrm{zupt_q}^{-1}\otimes\hat{z}_\mathrm{zupt_q}\right\}_{(234)} \\ &=\left\{q_\mathrm{b}^\mathrm{w}(n-1) \otimes (\boldsymbol{q}_b^w(n))^{-1} \otimes \hat{q}_b^w(n) \otimes (\hat{q}_b^w(n-1))^{-1}\right\}_{(234)} \\ &=\left\{(\delta q_b^w(n))^{-1} \otimes \delta q_b^w(n-1)\right\}_{(234)} \\ &=\left\{\hat{q}_{\mathrm{b}}^{\mathrm{w}}(n)\otimes\left(\hat{q}_{\mathrm{b}}^{\mathrm{w}}(n-1)\right)^{-1}\right\}_{(234)} \\ &=\left(-\delta\boldsymbol{\theta}_\mathrm{b}^\mathrm{w}(n)+\delta\boldsymbol{\theta}_\mathrm{b}^\mathrm{w}(n-1)\right)/2 \end{aligned}$$



##### 3.5.2 基于特征的测量更新


​当不是处于静止状态的时候则切换回传统的hybrid VIO的测量更新方法。更新MSCKF特征残差、旧的和新的增广特征残差。

**1. 增广特征残差**

​ 传统的hybrid VIO采用的是三维逆深度参数化(3D IDP)进行特征的增广，而Larvio则采用了一维逆深度参数化(1D IDP)进行特征的增广。旧的和新的增广特征残差都是基于重投影误差进行约束。其残差的线性模型为： 

$$\begin{aligned} r_{\mathrm{ekf}}^{(c_{k}j)}& =\boldsymbol{H}_{\rho}^{(\mathbf{c}_{k}j)}\delta\rho_{j}+\boldsymbol{H}_{\mathbf{x}_{\mathrm{ho}}}^{(\mathbf{c}_{k}j)}\delta\boldsymbol{x}_{\mathrm{pos}_{\mathrm{ho}}}+\boldsymbol{H}_{\mathbf{x}_{k}}^{(\mathbf{c}_{k}j)}\delta\boldsymbol{x}_{\mathrm{pos}_{k}}+\boldsymbol{H}_{\mathrm{e}}^{(\mathbf{c}_{k}j)}\delta\boldsymbol{x}_{\mathrm{e}} +H_{t_{\mathrm{d}}}^{(\mathrm{c}_{k}j)}\delta t_{\mathrm{d}}+n_{\mathrm{rep}j}^{(\mathrm{c}_{k}j)} \end{aligned}$$

其中$\delta \rho_j$为第j个增广特征的逆深度残差，$\delta x_{pos_{ho}}$为host IMU姿态的残差状态，$\delta x_{pos_k}$为第k个IMU姿态的残差状态，$\delta x_e$为相机和IMU外参的残差状态，$\delta t_d$为相机和IMU之间的时间残差，$n_{repj}^{c_kj}$为线性重投影残差的噪声，H项是对应的雅可比，具体如下： 

$$\left.\left\{\begin{aligned}H_{\rho}^{(\mathfrak{c}_{k}j)}&=J_{k}^{(j)}J_{d_{j}}^{(\mathfrak{c}_{k}j)}J_{\rho_{j}}^{d_{\mathrm{ho}}}\\H_{\mathrm{xho}}^{(\mathfrak{c}_{k}j)}&=J_{k}^{(j)}J_{x_{\text{ho}}}^{(\mathfrak{c}_{k}j)}\\H_{x_{k}}^{(\mathfrak{c}_{k}j)}&=J_{k}^{(j)}J_{x_{k}}^{(\mathfrak{c}_{k}j)}\\H_{\mathrm{e}}^{(\mathfrak{c}_{k}j)}&=J_{k}^{(j)}J_{\mathrm{e}}^{(\mathfrak{c}_{k}j)}\\H_{l_{\mathrm{d}}}^{(\mathfrak{c}_{k}j)}&=V_{\mathrm{c}_{k}}^{(j)}\end{aligned}\right.\right.$$




![./figures\e4852d31478e4cf5bae70b3e1b8bb977.png](./figures\e4852d31478e4cf5bae70b3e1b8bb977.png)


**什么是逆深度，为什么用逆深度误差而不是深度误差**

​所谓逆深度就是深度的倒数。

​之所以用逆深度误差是因为可以**较小的数值来表达精度范围**，可以用来降低较远深度点造成的误差。比如在自然场景中物体的深度本身变化比较大，50米和100米，它们在图像中体现出来的视差其实很小，只有1-2个像素，如果直接采用深度来表达的话差距就是100-50=50，而用逆深度表达的话只有1/50-1/100=0.01，相对50就是一个非常小的数值，符合定义能量最小化函数。

​除此之外，逆深度参数表达还具有**优化变量少、能表达非常远的点以及分布接近高斯分布**等优点。比如特征点在归一化相机坐标系与在相机坐标系下的坐标关系为： 

$$\begin{bmatrix}x\\y\\z\end{bmatrix}=\frac{1}{\lambda } \begin{bmatrix}u\\v\\1\end{bmatrix}$$

 其中$\lambda = \frac{1}{z}$称为逆深度。

+ 采用逆深度的方式，表达一个点的坐标(x,y,z)变成成了$\frac{1}{\lambda } \begin{bmatrix}u\\v\\1\end{bmatrix}$(如上所示)，将3个优化变量，变成了一个优化变量(u,v为归一化相机坐标系下的三维点的坐标，通过观测数据是可知的)，所以优化变量少。+ 同时，对于深度很大的点，在数值上很大，采用逆深度的方式，倒一下，数值上变小了，有利于优化过程中数值的稳定性，不会因为很远的点(深度值很大)导致一次优化过程中出现较大的误差函数的变换，因此再远的点(如天空中的点)也能表达了，所以能表达非常远的点。+ 将深度值倒数一下，变成小数，更接近高斯分布的函数表达形式，方便优化。


**2.MSCKF特征残差**

​MSCKF特征的残差是从特征$f_j$的叠加残差中边缘化出3D特征位置$p^w_{f_j}$，然后应用零空间投影来降低所有MSCKF特征的叠加边缘化残差的维数。以下是其残差模型： 

$$\begin{aligned}\boldsymbol{r}_\mathrm{msckf}^{(\mathbf{c}_kj)}&=\boldsymbol{T}_f^{(\mathbf{c}_kj)}\delta\boldsymbol{p}_{f_j}^\mathrm{w}+\boldsymbol{T}_{\mathbf{x}_k}^{(\mathbf{c}_kj)}\delta\boldsymbol{x}_{\mathrm{pos}_k}+\boldsymbol{T}_\mathrm{e}^{(\mathbf{c}_kj)}\delta\boldsymbol{x}_\mathrm{e}+\boldsymbol{T}_{t_\mathrm{d}}^{(\mathbf{c}_kj)}\delta t_\mathrm{d}+\boldsymbol{n}_{\mathrm{rep}j}^{(\mathbf{c}_kj)}\end{aligned}$$



$$\left.\left\{\begin{aligned}T_f^{(\mathrm{c}_kj)}&=J_k^{(j)}L_f^{(\mathrm{c}_kj)}\\T_{x_k}^{(\mathrm{c}_kj)}&=J_k^{(j)}L_{x_k}^{(\mathrm{c}_kj)}\\T_{e}^{(\mathrm{c}_kj)}&=J_k^{(j)}L_{e}^{(\mathrm{c}_kj)}\\T_{t_\mathrm{d}}^{(\mathrm{c}_kj)}&=V_{c_k}^{(j)}\end{aligned}\right.\right.$$

 
![./figures\2192d16d20bb4c82b6f4d431ece31c8a.png](./figures\2192d16d20bb4c82b6f4d431ece31c8a.png)


其中$\hat{p}^w_{f_j}$是从三角化中获取的，具体三角化方法、边缘化方法以及零空间投影方法可见博客：https://zhuanlan.zhihu.com/p/77040286或者论文[6]

### 4 总结


+ 首先读取所有的图像和imu数据+ 然后进行循环的特征处理和状态估计 
  
+ 对于输入的每一张图像都采用LK光流跟踪，经过一系列的过滤操作（imu预测、反向光流、ORB描述子过滤、F矩阵RANSAC过滤）后筛选出精度较好的特征点，push到feature中+ 再根据筛选出来的feature进行状态估计，首先是使用4阶龙格库塔方法进行imu预积分，获取当前系统状态向量，包括相机的p、v、q、加速度计和陀螺仪的数据+ 将获取到的feature添加到系统状态向量+ 从系统状态向量中获取imu状态向量，并对其进行增广，将相机状态增加到imu状态向量中，并更新系统状态向量的协方差矩阵+ 若处于静止状态，则采用ZUPT零速修正，通过速度、位置、旋转残差进行约束，利用EKF更新当前系统状态向量的状态向量和对应的协方差矩阵，包括Legacy状态、增广的imu状态和对应的相机状态、增广的feature状态+ 若不是静止状态，基于增广的新旧feature和msckf_feature进行测量更新，对于新旧feature采用一维逆深度参数化求解雅可比矩阵和残差矩阵，其余的msckf_feature则采用MSCKF的方法求解雅可比矩阵和残差矩阵，构建重投影误差模型进行测量更新。它们都是通过由滑动窗口限制一定数量的特征点和不同时刻的相机位姿态进行几何约束，以此构建一个观测模型来进行状态的更新，包括imu姿态、相机姿态、特征点位置、imu与相机的外参、时延+ 剔除旧的imu状态，同步更新系统的状态向量和协方差矩阵
 



**参考文献** [1] VINS-Mono: A Robust and Versatile Monocular [2] A Multi-State Constraint Kalman Filter [3] Indirect Kalman Filter for 3D Attitude Estimation [4] Monocular Visual-Inertial Odometry with an Unbiased Linear System Model and Robust Feature Tracking Front-End [5] Lightweight hybrid visual-inertial odometry with closed-form zero velocity update [6] A multi-state constraint Kalman filter for vision-aided inertial navigation


