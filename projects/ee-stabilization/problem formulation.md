# 空中操作的控制

### problem formulation

Given the current system state $\theta$, $$\dot{\theta}$$.

the objective is to design a control input torque $\tau$ 

such that the state asymptotically tracks the desired reference trajectory defined by $\theta^*$ and $\dot{\theta}^*$, which means $\theta \to \theta^*, \dot{\theta} \to \dot{\theta^*}$. 



**Error Definition:** We define the position tracking error $e(t)$ and velocity tracking error $\dot{e}(t)$ as: $$e(t) = \theta^*(t) - \theta(t)$$, $$\dot{e}(t) = \dot{\theta}^*(t) - \dot{\theta}(t)$$ 

**Control Objective:** The goal is to formulate a feedback control law $\tau = \pi(\theta, \dot{\theta}, \theta^*, \dot{\theta}^*)$ that guarantees the tracking errors converge to zero as time approaches infinity: $$\lim_{t \to \infty} e(t) = 0, \quad \lim_{t \to \infty} \dot{e}(t) = 0$$​.



There are three modes:

1. Position control
2. Pose control
3. Pose + force control / Impedance control

![image-20260708202608247](/Users/chenhaoyu/Library/Application Support/typora-user-images/image-20260708202608247.png)

#### For Position Control

只关心末端执行器的$$(x,y,z)$$，不关心姿态，只需要惩罚位置误差$$p_d-p$$。

#### For Pose Control

除了末端执行器的$$(x,y,z)$$，还要关注姿态的角度，$$e_R = \left[ \frac{1}{2} (R_d^T R - R^T R_d) \right]^\vee$$，$$ e = [e_p, e_R]^T$$. 通过CLIK方法来得到关节角的error。

控制输出$$\tau = -k_p(\theta-\theta^*)-k_v(\dot{\theta}-\dot{\theta^*})+\tau_{ff}(重力补偿)$$

#### For Pose + Force Control

在动力学方程上需要加上力的部分，即在方程中加入规划得到的力和Jacobian矩阵：$$M(\theta)\ddot{\theta} + C(\theta, \dot{\theta})\dot{\theta} + G(\theta) + J^T(\theta)F_{ext} = \tau$$

我们的控制目标实际就是原来的动力学加上外力：$$M_d \ddot{e}(t) + D_d \dot{e}(t) + K_d e(t) = F_{ext}$$

$$\tau = -k_p(\theta-\theta^*)-k_v(\dot{\theta}-\dot{\theta^*})+\tau_{ff}(重力补偿)+J^TF_{Ed}$$



我们可以跳过Position Control从Pose Control开始做，然后再拓展到力控



除了控制角度，这里也罗列逆运动学方法：
$$
V_d = J \dot{\theta_d}
$$

$$
\dot{\theta_d} = J^\dagger (v_d-k_pe)
$$

$$
v_d=[v_d^T, \omega_d^T]^T
$$

$$
e = [e_p, e_R]^T
$$

After having $$\dot{\theta_d}$$, we can get $$\theta_d$$ from that:
$$
\theta_d = \theta + \dot{\theta_d}\Delta t
$$
在工程中，我们一般用前一时刻的$\theta_{d,last}$:
$$
\theta_d = \theta_{d,last}+\dot{\theta_d} \Delta t
$$

#### 阻抗控制（Impedance control）

接触丰富的操作时，这种顺从性至关重要

#### 现阶段的问题

对几何控制、阻抗导纳控制还不了解，最近在详细了解中。

