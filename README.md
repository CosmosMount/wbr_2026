# 偏置并联构型轮腿机器人控制

## 动力学建模与线性化

### 1. 符号定义

为了建立动力学模型，首先定义系统的广义坐标、控制输入以及相关的物理参数。

#### 1.1 系统状态与输入

* 以左视图向左为$x$轴正方向，向上为$z$轴正方向建立右手系
* 对一切旋转（角度、扭矩）相关定义，以旋转轴俯视，逆时针为正方向

| 符号 | 描述 |
| --- | --- |
| $x, \dot{x}, \ddot{x}$ | 机器人的纵向位移、速度、加速度 | 
| $\phi, \dot{\phi}, \ddot{\phi}$ | 机器人的偏航角（Yaw）、角速度、角加速度 |
| $\theta_{l,l}, \dot{\theta}_{l,l}, \ddot{\theta}_{l,l}$ | 左腿转角、角速度、角加速度 |
| $\theta_{l,r}, \dot{\theta}_{l,r}, \ddot{\theta}_{l,r}$ | 右腿转角、角速度、角加速度 |
| $\theta_b, \dot{\theta}_b, \ddot{\theta}_b$ | 机体俯仰角、角速度、角加速度 |
| $\tau_{w,l}, \tau_{w,r}$ | 左、右轮控制力矩 |
| $\tau_{l,l}, \tau_{l,r}$ | 左、右腿（膝/髋关节）控制力矩 |

* 腿转角定义为腿与$z$轴负方向夹角
* 机体俯仰角 (Pitch) 定义为机体与$x$轴正方向夹角

#### 1.2 物理参数

* **常数**: $g=9.78$ (重力加速度，对于重力，向下为$mg$)
* **几何尺寸**: $R_w$ (轮半径), $R_b$ (轮距的一半), $l_l, l_r$ (左/右腿长)
* **质量参数**: $m_b$ (机体质量), $m_l$ (腿部质量), $m_w$ (轮子质量)
* **惯量参数**: $I_b$ (机体俯仰转动惯量), $I_{l,l}, I_{l,r}$ (腿部转动惯量), $I_w$ (轮子转动惯量), $I_\phi$ (整体偏航转动惯量)
* **质心偏移参数**: $d_b, \theta_{b}^0$ (机体质心偏移量及角度), $d_{l,l}, d_{l,r}, \theta_{l,l}^0, \theta_{l,r}^0$ (腿部质心偏移量及角度)

---

### 2. 运动学约束

**腿部加速度约束**


$$a_l^h = l_l \cos(\theta_{l,l})\ddot{\theta}_{l,l} - l_l \sin(\theta_{l,l})\dot{\theta}_{l,l}^2 + l_r \cos(\theta_{l,r})\ddot{\theta}_{l,r} - l_r \sin(\theta_{l,r})\dot{\theta}_{l,r}^2$$

$$a_{l,l}^v = -l_l \sin(\theta_{l,l})\ddot{\theta}_{l,l} - l_l \cos(\theta_{l,l})\dot{\theta}_{l,l}^2$$

$$a_{l,r}^v = -l_r \sin(\theta_{l,r})\ddot{\theta}_{l,r} - l_r \cos(\theta_{l,r})\dot{\theta}_{l,r}^2$$

**左右轮角加速度约束**

$$\ddot{\theta}_{w,l} = \frac{\ddot{x} - R_b\ddot{\phi} + \frac{1}{2}a_l^h}{R_w}$$

$$\ddot{\theta}_{w,r} = \frac{\ddot{x} + R_b\ddot{\phi} + \frac{1}{2}a_l^h}{R_w}$$

---

### 3. 动力学方程

利用牛顿-欧拉法或拉格朗日法推导得到的 5 个核心自由度的动力学方程$F(q,\dot{q},\ddot{q},u)=0$：

**1. 水平动量方程：**


$$(\tau_{w,l} - I_w\ddot{\theta}_{w,l} + \tau_{w,r} - I_w\ddot{\theta}_{w,r}) - \left[ \left( \frac{1}{2}m_b + m_l + m_w \right)R_w(\ddot{\theta}_{w,l} + \ddot{\theta}_{w,r}) + \left( \frac{1}{2}m_b + m_l \right)a_l^h \right] R_w = 0$$

**2. 偏航 (Yaw) 转动方程：**


$$I_\phi\ddot{\phi} - \left( (\tau_{w,r} - I_w\ddot{\theta}_{w,r}) - (\tau_{w,l} - I_w\ddot{\theta}_{w,l}) \right) \frac{R_b}{R_w} = 0$$

**3. 左腿转动方程：**


$$\tau_{l,l} - \tau_{w,l} - m_l g d_{l,l} \sin(\theta_{l,l} + \theta_{l,l}^0) - \left( \frac{\tau_{w,l} - I_w\ddot{\theta}_{w,l}}{R_w} - m_l R_w\ddot{\theta}_{w,l} \right) l_l \cos(\theta_{l,l}) + \left( \frac{1}{2}m_b + m_l \right) \left( g + \frac{a_{l,l}^v + a_{l,r}^v}{2} \right) l_l \sin(\theta_{l,l}) - I_{l,l}\ddot{\theta}_{l,l} = 0$$

**4. 右腿转动方程：**


$$\tau_{l,r} - \tau_{w,r} - m_l g d_{l,r} \sin(\theta_{l,r} + \theta_{l,r}^0) - \left( \frac{\tau_{w,r} - I_w\ddot{\theta}_{w,r}}{R_w} - m_l R_w\ddot{\theta}_{w,r} \right) l_r \cos(\theta_{l,r}) + \left( \frac{1}{2}m_b + m_l \right) \left( g + \frac{a_{l,l}^v + a_{l,r}^v}{2} \right) l_r \sin(\theta_{l,r}) - I_{l,r}\ddot{\theta}_{l,r} = 0$$

**5. 俯仰 (Pitch) 转动方程：**


$$(-\tau_{l,l} - \tau_{l,r} + m_b g d_b \cos(\theta_b + \theta_{b}^0)) - I_b\ddot{\theta}_b = 0$$

---

### 4. 平衡点解算

为了进行局部线性化，需要求解系统在静止直立状态下的平衡点。此时速度与加速度项均为零。通过静态力矩平衡可求得左右腿的稳态偏角：

$$\theta_{l,l}^{eq} = \arctan\left( \frac{m_l d_{l,l} \sin(\theta_{l,l}^0)}{\left( \frac{1}{2}m_b + m_l \right)l_l - m_l d_{l,l} \cos(\theta_{l,l}^0)} \right)$$

$$\theta_{l,r}^{eq} = \arctan\left( \frac{m_l d_{l,r} \sin(\theta_{l,r}^0)}{\left( \frac{1}{2}m_b + m_l \right)l_r - m_l d_{l,r} \cos(\theta_{l,r}^0)} \right)$$

机体的俯仰平衡点由设定值给出，对应 $\theta_b = \frac{\pi}{2} - \theta_{b}^0$。

---

### 5. 状态空间线性化与矩阵提取

将非线性动力学方程组整理为如下的隐式微分方程矩阵形式：


$$M(q)\ddot{q} + D(q, \dot{q})\dot{q} + K(q)q + H u = 0$$

其中：

* **状态向量**: $q = [x, \phi, \theta_{l,l}, \theta_{l,r}, \theta_b]^\tau$
* **控制输入**: $u = [\tau_{w,l}, \tau_{w,r}, \tau_{l,l}, \tau_{l,r}]^\tau$

通过对上述方程关于 $\ddot{q}, \dot{q}, q, u$ 求雅可比矩阵 (Jacobian)，并在第 4 节求得的平衡点（$q=q_{eq}, \dot{q}=0, \ddot{q}=0, u=0$）处进行泰勒展开和代入物理参数，可以得到定常矩阵 $M_{eq}, D_{eq}, K_{eq}, H_{eq}$。

#### 5.1 构建标准状态空间方程

定义 10 维状态向量 $X$：


$$X = [x, \dot{x}, \phi, \dot{\phi}, \theta_{l,l}, \dot{\theta}_{l,l}, \theta_{l,r}, \dot{\theta}_{l,r}, \theta_b, \dot{\theta}_b]^T$$

将其转化为标准的连续线性时不变 (LTI) 系统模型：


$$\dot{X} = A X + B u$$

根据 $\ddot{q} = -M_{eq}^{-1}K_{eq}q - M_{eq}^{-1}D_{eq}\dot{q} - M_{eq}^{-1}H_{eq}u$，系统矩阵 $A$ 和 $B$ 的结构如下：

* **状态矩阵 $A \in \mathbb{R}^{10 \times 10}$**: 奇数行为运动学积分项，偶数行为动力学加速度项。
* **输入矩阵 $B \in \mathbb{R}^{10 \times 4}$**: 仅在偶数行有对应控制输入的增益项。

$$A_{(2k, :)} = \text{Corresponding } [-M_{eq}^{-1}K_{eq}, -M_{eq}^{-1}D_{eq}] \text{ Coeffs}$$

$$B_{(2k, :)} = -M_{eq}^{-1}H_{eq}$$
