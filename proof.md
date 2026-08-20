$\dot{x_i} = A_ix_i+B_iu_i+D_iv$
$y_i = Cx_i$
$e_i = Cx_i+Fv$
$\dot{v} = Sv$
控制器：
$u_i = -K_{x}x_i-K_{z}z_i$
$z_i = Sz_i + G_2 \hat{e}_i$
其中：
$x_i \in \mathbb{R}^{n_x}$，$u_i \in \mathbb{R}$，$y_i \in \mathbb{R}$，$v \in \mathbb{R}^{n_v}$，$z_i \in \mathbb{R}^{n_z}$。

$\hat{e}_i \;=\; \frac{\displaystyle\sum_{j\in\mathcal{N}_i} a_{ij}\,(y_i - y_j)}{\displaystyle\sum_{j=0}^N a_{ij}} = \frac{\displaystyle a_{i0}\,(y_i - y_0)}{\displaystyle\sum_{j=0}^N a_{ij}} + \frac{\displaystyle\sum_{j\in\mathcal{N}_i} a_{ij}\,Fv}{\displaystyle\sum_{j=0}^N a_{ij}}= \frac{\sum_{j\in\mathcal{N}_i} a_{ij}y_i}{\sum_{j=0}^N a_{ij}}-\frac{\sum_{j\in\mathcal{N}_i} a_{ij}y_j}{\sum_{j=0}^N a_{ij}}=y_i - \frac{\sum_{j\in\mathcal{N}_i} a_{ij}y_j}{\sum_{j=0}^N a_{ij}} = y_i -\sum_{j\in\mathcal{N}_i} \alpha_{ij}y_j = C_ix_i - \sum_{j\in\mathcal{N}_i} \alpha_{ij}y_j$
$\alpha_{ij} = \frac{a_{ij}}{\sum_{j=0}^N a_{ij}}$
## 图论
定义一个图${\mathcal{G}}=\{{\mathcal{V}},{\mathcal{E}}\}.{\mathcal{V}}=\{0,1,\ldots,N\}$,结点0作为领导者也是外部系统，剩下的结点作为跟随者。${\mathcal{E}}\subset\mathcal{V}\times\mathcal{V}$作为边的集合矩阵。定义邻接矩阵${\mathcal{A}}=[a_{i j}]\,\in\,\mathbb{R}^{(N+1)\times(N+1)}$,$a_{i j}>0$作为权重，如果ij相连的话。不相连则为0
定义拉普拉斯矩阵${\mathcal{L}}={\left[\begin{array}{l l}{\sum_{j=1}^{N}a_{0j}}&{-[a_{01},\ldots,a_{0N}]}\\ {-\Delta{\mathbf{1}}_{N}}&{{\mathcal{H}}}\end{array}\right]}$
## 向量化
$vec(ABC) = (C^T \otimes A)vec(B)$
$v^T P v = vecv(v)^T vecs(P)$
## 重要参数
reset值，这个值的大小决定了最大的P_k的上限，当出现超过这个上限的P_k时会触发重置。设置过小会频繁触发。
数据收集的时间长度，数据收集的时间长度决定了每次更新时使用的数据量，过短可能导致数据不足以准确估计系统参数，过长则可能导致系统发散，导致切换时出现很大的输出尖峰。
## 输出调节
### 分布式观测器
定义S矩阵的分布式观测器$\dot{S}_i = \mu_1 \sum_{j=0}^{N} a_{ij}(t)\big(S_j - S_i\big), \quad i = 1, 2, \ldots, N$
其中的 $S_0 = S$ 是领导者系统矩阵，$S_i$ 是跟随者 $i$ 的观测器状态。$\mu_1 > 0$ 是观测器增益，$a_{ij}(t)$ 是通信权重。
### Lemma1
Given the leader system $\dot{v} = Sv$ and the distributed observer $\dot{S}_i = \mu_1 \sum_{j=0}^{N} a_{ij}\big(S_j - S_i\big)$. Let $\hat{S}_i= e^{\gamma t}(S_i-S)=e^{\gamma t}\tilde{S}_i$ for any initial conditions $\hat{S}(0)$ we have for any $\mu_1 > \frac{\gamma}{\alpha_H}$, for i =1,....,N, $\lim_{t \to \infty} \hat{S}_i = 0$. $\alpha_H$ is  defined later.
#### proof
Let $\tilde{S}=col(\tilde{S}_1,....,\tilde{S}_N)$. Then $\dot{\tilde{S}_i}=\mu_1\sum_{j=0}^Na_{ij}[\tilde{S}_j-\tilde{S}_i]$ can be put in the form of $\dot{\tilde{S}} = -\mu_1(H\otimes I_q)\tilde{S}$.又因为$\hat{S}=e^{\gamma t}\tilde{S}$,则$\dot{\hat{S}}=\gamma\hat{S}-\mu_1(H\otimes I_q)\hat{S}=(\gamma I-\mu_1(H\otimes I_q))\hat{S}$。设矩阵H的特征值的集合为$\lambda(H)$, 则$\gamma-\mu_1(H\otimes I_q)$的特征值集合为$\lambda'=\gamma-\mu_1\lambda(H)$ 根据假设4，根据hu&huang的引理4，H矩阵所有特征值均有正实部，定义$\alpha_H = \min_i \Re(\lambda(H))$,又因为$\mu_1 > \frac{\gamma}{\alpha_H}$, 所以对于H矩阵的所有特征值都满足$\mu_1 \Re(\lambda(H)) > \gamma$，即$\Re(\lambda') < 0$。因此$\lim_{t \to \infty} \hat{S} = 0$.也就是说，for i=1,....,N. $\lim_{t \to \infty} \hat{S}_i = 0$.
### Theorem1
under assumption 1-5,the augmented system
$\dot{x_i} = A_ix_i+B_iu_i+D_iv$
$\dot{z_i} = S_iz_i + G_2 \hat{e}_i$
$\dot{S_i} = \mu_1 \sum_{j=0}^{N} a_{ij}(t)\big(S_j - S_i\big)$
$\dot{v} = Sv$
$e_i = Cx_i+Fv$
in closed-loop with $u_i = -K_{x_i}x_i-K_{z_i}z_i$ achieves cooperative output regulation with convergence rate $\gamma$ for any $0 \leq \gamma < \gamma^*$ if the matrix $A_{ci} + \gamma I = \left[\begin{matrix} A_i-B_iK_{x_i}+\gamma I & -B_iK_{z_i}\\ G_2C_i & S+\gamma I \end{matrix}\right]$is Hurwitz
$\gamma^*$ will be specified later
#### proof:
Under Assumption 1-5, for i=1,2,....N, 调节器方程
$X_iS = A_iX_i +B_iU_i+D_i$
$0 = C_iX_i + F$
有唯一解$(X_i,U_i)$ 
因为$Aci+\gamma I$是Hurwitz矩阵，所以Aci的特征值实部小于$-\gamma$，Aci也是Hurwitz矩阵。
根据黄杰lemma1.27，下列矩阵方程：
$X_iS = (A_i-B_iK_{x_i})X_i-B_iK_{z_i}Z_i+D_i$
$Z_iS = SZ_i+G_2(C_iX_i+F)$
$0 = C_iX_i + F$
存在唯一解$(\hat{X_i},Z_i)$
则有$X_iS = A_iX_i +B_i(-K_{x_i}X_i-K_{z_i}Z_i)+D_i$
可见，当Aci是Hurwitz矩阵时，调节器方程的解($X_i, U_i$)为$\hat{X_i}$和$-K_{x_i}X_i-K_{z_i}Z_i$。又因为调节器方程的解唯一，我们可以得出三元组$(X_i,U_i,Z_i)$ is uniquely determined by (9)–(12).
我们定义下列误差状态：
$\tilde{x}_i = e^{\gamma t}(x_i - X_i v), \tilde{z}_i = e^{\gamma t}(z_i - Z_i v)$
$\tilde{S}_i = (S_i - S)$
$\tilde{\eta}_i = \left[\begin{matrix} \tilde{x}_i \\ \tilde{z}_i \end{matrix}\right]$。
我们有：
$\dot{\tilde{x}}_i = \gamma e^{\gamma t}(x_i - X_i v)+e^{\gamma t}( A_ix_i+B_iu_i+D_iv-X_iSv) \\= \gamma\tilde{x}_i+e^{\gamma t}(A_ix_i+B_i(-K_{x_i}x_i-K_{z_i}z_i)+D_iv-X_iSv)\\=\gamma\tilde{x}_i+e^{\gamma t}[(A_i-B_iK_{x_i})x_i-B_iK_{z_i}z_i+D_iv-X_iSv]\\=\gamma\tilde{x}_i+(A_i-B_iK_{x_i})\tilde{x_i}-B_iK_{z_i}\tilde{z_i}$

$\dot{\tilde{z}}_i = \gamma e^{\gamma t}(z_i - Z_i v)+e^{\gamma t}( S_iz_i+G_2\hat{e}_i -Z_iSv) 
\\= \gamma \tilde{z}_i+e^{\gamma t}[(S_i-S)z_i+G_2\hat{e}_i-Z_iSv+Sz_i]
\\= \gamma \tilde{z}_i+e^{\gamma t}[Sz_i+G_2C_ix_i-G_2\sum_{j\in\mathcal{N}_i} \alpha_{ij}C_jx_j-Z_iSv+(S_i-S)z_i]$
因为$Z_iS = SZ_i+G_2(C_iX_i+F)$，所以$G_2C_iX_iv+SZ_iv-G_2\sum_{j\in\mathcal{N}_i} \alpha_{ij}C_jX_jv-Z_iSv = 0$。
因此
$\gamma \tilde{z}_i+e^{\gamma t}[S(z_i-Z_iv)+G_2C_i(x_i-X_iv)-G_2\sum_{j\in\mathcal{N}_i} \alpha_{ij}C_j(x_j-X_jv)+(S_i-S)z_i+[SZ_iv-Z_iSv+G_2C_iX_iv-G_2\sum_{j\in\mathcal{N}_i} \alpha_{ij}C_jX_jv]]
\\= \gamma \tilde{z}_i+S\tilde{z}+G_2C_i\tilde{x}_i-G_2\sum_{j\in\mathcal{N}_i} \alpha_{ij}C_j\tilde{x}_i+e^{\gamma t}(S_i-S)z_i
\\=(\gamma I+S)\tilde{z}_i+G_2C_i\tilde{x}_i-G_2\sum_{j\in\mathcal{N}_i} \alpha_{ij}C_j\tilde{x}_i+e^{\gamma t}(S_i-S)z_i
\\=(\gamma I+S)\tilde{z}_i+G_2C_i\tilde{x}_i-G_2\sum_{j\in\mathcal{N}_i} \alpha_{ij}C_j\tilde{x}_i+e^{\gamma t}(S_i-S)(z_i-Z_iv)+e^{\gamma t}(S_i-S)Z_iv$
因此，误差系统的动力学可以写成如下紧凑形式：
$
\dot{\tilde{\eta}}_i = (A_{ci}+\gamma I) \tilde{\eta}_i - \sum_{j\in\mathcal{N}_i} \alpha_{ij} \begin{bmatrix} 0 \\ G_2\bar{C_j} \end{bmatrix} \tilde{\eta}_j + \begin{bmatrix} 0 &0\\0&\tilde{S}_i \end{bmatrix}\tilde{\eta_i}+\begin{bmatrix} 0\\ e^{\gamma t}\tilde{S}_i{Z_iv} \end{bmatrix}
\\:=A^{\gamma}_{ci}+L^0_i(t)\tilde{\eta}_j+L^1_i(t)\tilde{\eta}_i+L^2_i(t) 
$
$e^{\gamma}_i = \bar{C}_i\tilde{\eta}_i$
其中$A_{ci} = \left[\begin{matrix} A_i-B_iK_{x_i} & -B_iK_{z_i}\\ G_2C_i & S \end{matrix}\right]$。
根据假设3，我们可以获得所有跟随者的标签为$i<j$，if $(i,j)\in\mathcal{E}.$ 然后我们可以表示整个多智能体系统为
$\dot{\tilde{\eta}}=A^{\gamma}_c\tilde{\eta}+L^1(t)\tilde{\eta}+L^2(t)$
其中$\tilde{\eta}=[\tilde{\eta}^T_1,\tilde{\eta}^T_2....,\tilde{\eta}^T_N]^T$
$
\begin{array}{l}{{L^1=\mathrm{blockdiag}(L^1_1,L^1_2,\ldots,L^1_N)}}\\ {{L^2=\left[{L^2_1}^T,{L^2_2}^T,\ldots,{L^2_N}^T\right]^{\mathrm{T}}}}\end{array}
$
and $A^{\gamma}_c$ is a block lower triangular matrix with submatrices $A^{\gamma}_{ci}$ on the diagonal, for any i = 1, 2, . . . , N.
根据假设4，外部系统状态v(t)是有界的。根据lemma1，The estimaon eroor satisfies $\lim_{t \to \infty} \hat{S}_i = 0$. 根据Cai2017 lemma2，$\lim_{t \to \infty} \tilde{S}_i = 0$.因此我们可得$\lim_{t \to \infty} L^1(t)= 0$, $\lim_{t \to \infty} L^2(t) = 0$.因为$A^\gamma_{ci}$ is Hurwitz矩阵，所以$A^\gamma_c$ is Hurwitz矩阵。因此根据cai2017的lemma1，我们可以获得系统 $\dot{\tilde{\eta}}=(A^{\gamma}_c +L^1(t))\tilde{\eta}$ is exponentially stable.因此$\lim_{t \to \infty} \tilde{\eta} = 0$.又因为$e_i = \bar{C}_i\tilde{\eta}_i$,所以$\lim_{t \to \infty} e_i = 0$.因此系统实现了协同输出调节。
## 增广系统
$
\dot{\eta} = \left[\begin{matrix} \dot{x} \\ \dot{z} \end{matrix}\right] = \left[\begin{matrix} A_i & 0 \\ G_2C_i & S \end{matrix}\right]\left[\begin{matrix} x \\ z \end{matrix}\right]+ \left[\begin{matrix} B_i \\ 0 \end{matrix}\right]u_i + \left[\begin{matrix} D_i \\ 0 \end{matrix}\right]v - \left[\begin{matrix} 0 \\ G_2\sum_{j\in\mathcal{N}_i} \alpha_{ij}y_j  \end{matrix}\right]
$

define $\left[\begin{matrix} 0 \\ -G_2\sum_{j\in\mathcal{N}_i} \alpha_{ij}y_j  \end{matrix}\right] = \theta_i, \bar{A_i} = \left[\begin{matrix} A_i & 0 \\ G_2C_i & S \end{matrix}\right], \bar{B_i} = \left[\begin{matrix} B_i \\ 0 \end{matrix}\right], \bar{D_i} = \left[\begin{matrix} D_i \\ 0 \end{matrix}\right]$
then we have
## VI
define $V_i = \eta_i^T P^k_i \eta_i$, P > 0
$\dot{V_i} = \eta_i^T (P^k_i\bar{A_i} + \bar{A_i}^TP^k_i)\eta_i + 2u_i^T\bar{B_i}^TP^k_i\eta_i + 2v_i^T\bar{D_i}^TP^k_i\eta_i + 2\theta_i^TP^k_i\eta_i$
let $H^k_{i} = P^k_i\bar{A_i} + \bar{A_i}^TP^k_i $
$\int_{t0}^{t1} \dot{\eta}_i(t) dt \\= (vecv(\eta_i)_{t1} - vecv(\eta_i)_{t0})vecs(P^k_i)\\=(\int_{t0}^{t1}vecv(\eta_i)d\tau)^T vecs(H^k_{i}) \\+2(\int_{t0}^{t1} kron(\eta_i,u_i)d\tau)^T vec(\bar{B_i}^TP^k_i)\\+ 2(\int_{t0}^{t1} kron(\eta_i,v_i)d\tau)^T vec(\bar{D_i}^TP^k_i)\\+2(\int_{t0}^{t1} kron(\eta_i,\theta_i)d\tau)^T vec(P^k_i)$

define 
$\Delta_i = \left[\begin{matrix} vecv(\eta_i)_{t1} - vecv(\eta_i)_{t0}\\vecv(\eta_i)_{t2} - vecv(\eta_i)_{t1} \\ vecv(\eta_i)_{t3} - vecv(\eta_i)_{t4} \\ vecv(\eta_i)_{t5} - vecv(\eta_i)_{t4}\\....\\vecv(\eta_i)_{ts} - vecv(\eta_i)_{t_{s-1}} \end{matrix}\right]$
define
$
\Gamma_{\eta_i\eta_i} =  \left[\begin{matrix} \int_{t0}^{t1} vecv(\eta_i)d\tau \\ \int_{t1}^{t2} vecv(\eta_i)d\tau \\ \int_{t2}^{t3} vecv(\eta_i)d\tau \\ \int_{t3}^{t4} vecv(\eta_i)d\tau \\....\\\int_{t_{s-1}}^{ts} vecv(\eta_i)d\tau \end{matrix}\right],
$$\Gamma_{\eta_iu_i} =  \left[\begin{matrix} \int_{t0}^{t1} kron(\eta_i,u_i)d\tau \\ \int_{t1}^{t2} kron(\eta_i,u_i)d\tau \\ \int_{t2}^{t3} kron(\eta_i,u_i)d\tau \\ \int_{t3}^{t4} kron(\eta_i,u_i)d\tau \\....\\\int_{t_{s-1}}^{ts} kron(\eta_i,u_i)d\tau \end{matrix}\right],$$\Gamma_{\eta_iv_i} =  \left[\begin{matrix} \int_{t0}^{t1} kron(\eta_i,v_i)d\tau \\ \int_{t1}^{t2} kron(\eta_i,v_i)d\tau \\ \int_{t2}^{t3} kron(\eta_i,v_i)d\tau \\ \int_{t3}^{t4} kron(\eta_i,v_i)d\tau \\....\\\int_{t_{s-1}}^{ts} kron(\eta_i,v_i)d\tau \end{matrix}\right],$
$
\Gamma_{\eta_i\theta_i} =  \left[\begin{matrix} \int_{t0}^{t1} kron(\eta_i,\theta_i)d\tau \\ \int_{t1}^{t2} kron(\eta_i,\theta_i)d\tau \\ \int_{t2}^{t3} kron(\eta_i,\theta_i)d\tau \\ \int_{t3}^{t4} kron(\eta_i,\theta_i)d\tau \\....\\\int_{t_{s-1}}^{ts} kron(\eta_i,\theta_i)d\tau \end{matrix}\right]
$
then we have
$\Delta_i vecs(P^k_i) = \Gamma_{\eta_i\eta_i} vecs(H^k_{i}) + 2\Gamma_{\eta_iu_i} vec(\bar{B_i}^TP^k_i) + 2\Gamma_{\eta_iv_i} vec(\bar{D_i}^TP^k_i) + 2\Gamma_{\eta_i\theta_i} vec(P^k_i)$
又因为$K^k_i = R^{-1}\bar{B_i}^TP^k_i$,所以$\bar{B_i}^TP^k_i = R K^k_i$
有
$\Delta_i vecs(P^k_i) = \Gamma_{\eta_i\eta_i} vecs(H^k_{i}) + 2\Gamma_{\eta_iu_i} vec(R K^k_i) + 2\Gamma_{\eta_iv_i} vec(\bar{D_i}^TP^k_i) + 2\Gamma_{\eta_i\theta_i} vec(P^k_i)$
因为$vec(AXB) = (B^T \otimes A)vec(X)$
所以有
$\Delta_i vecs(P^k_i) = \Gamma_{\eta_i\eta_i} vecs(H^k_{i}) + 2\Gamma_{\eta_iu_i} (I_{n+n_z} \otimes R) vec(K^k_i) + 2\Gamma_{\eta_iv_i} vec(\bar{D_i}^TP^k_i) + 2\Gamma_{\eta_i\theta_i} vec(P^k_i)$

define $vec(P^k_i) = Mvecs(P^k_i)$
then we have
$\Delta_i vecs(P^k_i) = \Gamma_{\eta_i\eta_i} vecs(H^k_{i}) + 2\Gamma_{\eta_iu_i}vec(\bar{B_i}^TP^k_i) + 2\Gamma_{\eta_iv_i} vec(\bar{D_i}^TP^k_i) + 2\Gamma_{\eta_i\theta_i} Mvecs(P^k_i)$
then we have
${\Psi_i} \left[\begin{matrix}vecs(H^k_{i})\\ vec(\bar{B_i}^TP^k_i) \\ vec(\bar{D_i}^TP^k_i)  \end{matrix}\right]={\Phi^k_i} $
where $\hat{\Psi_i} = \left[\begin{matrix} \Gamma_{\eta_i\eta_i} & 2\Gamma_{\eta_iu_i} & 2\Gamma_{\eta_iv_i} \end{matrix}\right], \hat{\Phi^k_i} = \Delta_i vecs(P^k_i) - 2\Gamma_{\eta_i\theta_i} Mvecs(P^k_i)$
### VI秩条件
the matrix $\hat{\Psi^k_i}$ has full column rank for any k = 0, 1, 2, ... if the following condition holds:
$rank\left(\left[\begin{matrix} \Gamma_{\eta_i\eta_i} & 2\Gamma_{\eta_iu_i} & 2\Gamma_{\eta_iv_i} \end{matrix}\right]\right) = \frac{(n+n_z)(n+n_z+1)}{2}+(n_v+1)(n+n_z)$
#### VI秩条件证明
给出任意列向量a以及任意对称矩阵Q，我们可以得出$a^TQa=(a\otimes a)^Tvec(Q)=vecv(a)^Tvecs(Q)$。然后，又有$vec(Q)=Mvecs(Q)$。所以我们有$(a\otimes a)^TMvecs(Q)=vecv(a)^Tvecs(Q)$。因为Q是任意的对称矩阵，所以${\Psi_i}=\left[\begin{matrix} \Gamma_{\eta_i\eta_i} & \Gamma_{\eta_iu_i} & \Gamma_{\eta_iv_i} \end{matrix}\right]\Theta$
其中$\Theta=[I_{\frac{(n+n_z)(n+n_z+1)}{2}},2I_{n+n_z},2I_{n_v(n+n_z)}]$
所以我们有$rank({\Psi_i})=rank([I_{\frac{(n+n_z)(n+n_z+1)}{2}},I_{n+n_z},I_{n_v(n+n_z)}])$
## 改进VI首次迭代
k=0时: 选择一个$P_i=I_{n+n_z}$
我们有$\dot{V_i} = \eta_i^T (\bar{A_i} + \bar{A_i}^T)\eta_i + 2u_i^T\bar{B_i}^T\eta_i + 2v_i^T\bar{D_i}^T\eta_i + 2\theta_i^TP^0_i\eta_i = \eta_i^T (\bar{A_i} + \bar{A_i}^T)\eta_i + 2u_i^T{B_i}^Tx_i + 2v_i^T{D_i}^Tx_i  + 2\theta_i^T \eta_i$
${\Psi^0_i} \left[\begin{matrix}vecs(\bar{A}+\bar{A_i}^T)\\ vec({B_i}^T) \\ vec({D_i}^T)  \end{matrix}\right]={\Phi^0_i} $
其中:$\Psi^0_i=[\Gamma_{\eta_i\eta_i}, 2\Gamma_{x_iu_i}, 2\Gamma_{x_iv_i}]$
### 首次迭代秩条件
如果满足下列条件，矩阵${\Psi^0_i}$满列秩：
$rank\left(\left[\begin{matrix} \Gamma_{\eta_i\eta_i} & 2\Gamma_{x_iu_i} & 2\Gamma_{x_iv_i} \end{matrix}\right]\right) = \frac{(n+n_z)(n+n_z+1)}{2}+n(1+n_v)$
证明类似于VI的秩条件证明。
解得$vec({B_i}^T), vec({D_i}^T)$,也就解得了$vec(\bar{B_i}^T)和vec(\bar{D_i}^T)$
$P^1_i = P^0_i+\epsilon_0(H^0_i-P^0_i{\bar{B_i}}R^{-1}{\bar{B}^T_i}P^0_i+Q)$
$K^1_i = R^{-1}{\bar{B}^T_i}P^1_i$
k>0时：
$\Delta_i vecs(P^k_i) = \Gamma_{\eta_i\eta_i} vecs(H^k_{i}) + 2\Gamma_{\eta_iu_i}(I_{n+n_z} \otimes{\bar{B}_i^T})Mvecs(P^k_i) + 2\Gamma_{\eta_iv_i}(I_{n+n_z} \otimes{\bar{D}_i^T})Mvecs(P^k_i) + 2\Gamma_{\eta_i\theta_i} Mvecs(P^k_i)$
写成紧凑的形式：
$\hat{\Psi^k_i} \left[\begin{matrix}vecs(H^k_{i})\end{matrix}\right]=\hat{\Phi^k_i} $
其中$\hat{\Psi^k_i} = \left[\begin{matrix} \Gamma_{\eta_i\eta_i}  \end{matrix}\right], \hat{\Phi^k_i} = \Delta_i vecs(P^k_i) - 2\Gamma_{\eta_i\theta_i} Mvecs(P^k_i)-2\Gamma_{\eta_iu_i}(I_{n+n_z} \otimes{\bar{B}_i^T}) Mvecs(P^k_i) - 2\Gamma_{\eta_iv_i}(I_{n+n_z} \otimes{\bar{D}_i^T})Mvecs(P^k_i)$
然后，$H^k_i$可以被不断的迭代解出。  
### 随后迭代秩条件
如果满足下列条件，矩阵$\hat{\Psi^k_i}$满列秩：
$rank\left(\Gamma_{\eta_i\eta_i}\right) = \frac{(n+n_z)(n+n_z+1)}{2}$
证明类似于VI的秩条件证明。
### 好处
一旦首次迭代秩条件满足，那么首次迭代的秩条件就是我们唯一需要满足的秩条件，这个条件比起传统VI迭代的秩条件来说更为宽松，减少了n_z(1+n_v)个维度。
总的来说，在迭代获取H_k的时候，改进的VI迭代算法比传统VI迭代算法降低了(n+n_z)(1+n_v)个维度的秩条件要求。
## PI
重写增广系统方程：
$
\dot{\eta} = (\bar{A_i}-\bar{B}_i\bar{K}^k_i)\eta_i + \bar{B_i}(u_i+\bar{K}^k_i\eta_i) + \bar{D_i}v + \theta_i
$
define $\bar{A_{ci}} = \bar{A}_i - \bar{B}_i\bar{K}^k_i$
$
\dot{V_i} = \eta_i^T (P^k_i\bar{A_{ci}} + \bar{A_{ci}}^TP^k_i)\eta_i + 2(u_i+\bar{K}^k_i\eta_i)^T\bar{B_i}^TP^k_i\eta_i + 2v_i^T\bar{D_i}^TP^k_i\eta_i + 2\theta_i^TP^k_i\eta_i
$
两侧同时积分：
$\int_{t0}^{t1} \dot{V_i} dt = \int_{t0}^{t1} \eta_i^T (P^k_i\bar{A_{ci}} + \bar{A_{ci}}^TP^k_i)\eta_i dt + 2\int_{t0}^{t1}(u_i+\bar{K}^k_i\eta_i)^T\bar{B_i}^TP^k_i\eta_i dt + 2\int_{t0}^{t1}v_i^T\bar{D_i}^TP^k_i\eta_i dt + 2\int_{t0}^{t1}\theta_i^TP^k_i\eta_i dt
$
根据PI更新规则
$\bar{A_{ci}}^TP^k_i+P^k_i\bar{A_{ci}}= -Q_i+\bar{K^k_i}^TR_i\bar{K}^k_i$
$K^{k+1}_i = R^{-1}\bar{B}_i^TP^k_i$

然后有
$
vecv(\eta_i)_{t1} - vecv(\eta_i)_{t0} = \int_{t0}^{t1}vecv(\eta_i)d\tau^T vecs(-Q_i+\bar{K^k_i}^TR_i\bar{K}^k_i) + 2(\int_{t0}^{t1} kron(\eta_i,u_i)d\tau^T+\int_{t0}^{t1} kron(\eta_i,\eta_i)d\tau^T(I\otimes\bar{K^k_i}^T)) vec(\bar{B_i}^TP^k_i) + 2\int_{t0}^{t1} kron(\eta_i,v_i)d\tau^T vec(\bar{D_i}^TP^k_i) + 2\int_{t0}^{t1} kron(\eta_i,\theta_i)d\tau^T vec(P^k_i)
$
define $\int_{t0}^{t1} kron(\eta_i,\eta_i)^Td\tau = \Gamma_{\eta_i}$
那么有：
$
\delta_i vecs(P^k_i) = \Gamma_{\eta_i\eta_i} vecs(-Q_i+\bar{K^k_i}^TR_i\bar{K}^k_i) + 2(\Gamma_{\eta_iu_i}+\Gamma_{\eta_i}(I\otimes\bar{K^k_i}^T)) vec(\bar{B_i}^TP^k_i) + 2\Gamma_{\eta_iv_i} vec(\bar{D_i}^TP^k_i) + 2\Gamma_{\eta_i\theta_i} vec(P^k_i)
$
写成紧凑形式:
$
{\Psi_i} \left[\begin{matrix}vecs(P^k_i)\\ vec(\bar{B}^T_iP^k_i)\\ vec(\bar{D}^T_iP^k_i)\\ vec(P^k_i)\end{matrix}\right]={\Phi_i}
$
其中${\Psi_i}=[\delta_i,-2(\Gamma_{\eta_iu_i}+\Gamma_{\eta_i}(I\otimes\bar{K^k_i}^T)),-2\Gamma_{\eta_iv_i},-2\Gamma_{\eta_i\theta_i}]$
由于$\bar{B_i}与\bar{D_i}$在VI阶段已求得，可以进一步简化表达式。
$\delta_i vecs(P^k_i) = \Gamma_{\eta_i\eta_i} vec(-Q_i+\bar{K^k_i}^TR_i\bar{K}^k_i) + 2(\Gamma_{\eta_iu_i}+\Gamma_{\eta_i}(I\otimes\bar{K^k_i}^T)) vec(\hat{\bar{B}_i}^TP^k_i) + 2\Gamma_{\eta_iv_i} vec(\hat{\bar{D}_i}^TP^k_i) + 2\Gamma_{\eta_i\theta_i} vec(P^k_i)
$
$
vec(\hat{\bar{B}_i}^TP^k_i) = (I \otimes \hat{\bar{B}_i}^T)Mvecs(P^k_i), vec(\hat{\bar{D}_i}^TP^k_i) = (I \otimes \hat{\bar{D}_i}^T)Mvecs(P^k_i)
$
$
\delta_i vecs(P^k_i) = \Gamma_{\eta_i\eta_i} vec(-Q_i+\bar{K^k_i}^TR_i\bar{K}^k_i) + 2(\Gamma_{\eta_iu_i}+\Gamma_{\eta_i}(I\otimes\bar{K^k_i}^T)) (I \otimes \hat{\bar{B}_i}^T)Mvecs(P^k_i) + 2\Gamma_{\eta_iv_i} (I \otimes \hat{\bar{D}_i}^T)Mvecs(P^k_i) + 2\Gamma_{\eta_i\theta_i} Mvecs(P^k_i)
$
合并同类项得：
$[\delta_i - 2((\Gamma_{\eta_iu_i}+\Gamma_{\eta_i}(I\otimes\bar{K^k_i}^T)) (I \otimes \hat{\bar{B}_i}^T) + \Gamma_{\eta_iv_i} (I \otimes \hat{\bar{D}_i}^T) + \Gamma_{\eta_i\theta_i}) M]vecs(P^k_i) = \Gamma_{\eta_i\eta_i} vec(-Q_i+\bar{K^k_i}^TR_i\bar{K}^k_i) 
$
define $\hat{\Psi^k_i} = \delta_i - 2((\Gamma_{\eta_iu_i}+\Gamma_{\eta_i}(I\otimes\bar{K^k_i}^T)) (I \otimes \hat{\bar{B}_i}^T) + \Gamma_{\eta_iv_i} (I \otimes \hat{\bar{D}_i}^T) + \Gamma_{\eta_i\theta_i}) M$
$\hat{\Phi^k_i} = \Gamma_{\eta_i\eta_i} vec(-Q_i+\bar{K^k_i}^TR_i\bar{K}^k_i)$
then we have
$\hat{\Psi^k_i} vecs(P^k_i) = \hat{\Phi^k_i}$
迭代求解$P^k_i$,并迭代更新$K^k_i$。
### PI秩条件
如果满足下列条件，矩阵$\hat{\Psi^k_i}$满列秩：
证明:
设W为任意对称矩阵，根据前述矩阵向量化定义，有Mvecs(W) = vec(W)。
令$\hat{\Psi^k_i} vecs(W) = 0$
展开得
$[\delta_i - 2((\Gamma_{\eta_iu_i}+\Gamma_{\eta_i}(I\otimes\bar{K^k_i}^T)) (I \otimes \hat{\bar{B}_i}^T) + \Gamma_{\eta_iv_i} (I \otimes \hat{\bar{D}_i}^T) + \Gamma_{\eta_i\theta_i})M]vecs(W) = 0 $
有
$\delta_ivecs(W) = 2((\Gamma_{\eta_iu_i}+\Gamma_{\eta_i}(I\otimes\bar{K^k_i}^T)) (I \otimes \hat{\bar{B}_i}^T) + \Gamma_{\eta_iv_i} (I \otimes \hat{\bar{D}_i}^T) + \Gamma_{\eta_i\theta_i})vec(W)$
根据前述$vec(ABC) = (C^T \otimes A)vec(B)$
$\Gamma_{\eta_i}(I\otimes\bar{K^k_i}^T)) (I \otimes \hat{\bar{B}_i}^T)vec(W)= \Gamma_{\eta_i}(I\otimes\bar{K^k_i}^T))vec(\hat{\bar{B_i}}^TW) $
$\Gamma_{\eta_iv_i} (I \otimes \hat{\bar{D}_i}^T)vec(W)= \Gamma_{\eta_iv_i}vec(\hat{\bar{D_i}}^TW) $
$\Gamma_{\eta_i\theta_i} Mvecs(W)= \Gamma_{\eta_i\theta_i}vec(W)$
因此
$\delta_ivecs(W) = 2((\Gamma_{\eta_iu_i}+\Gamma_{\eta_i}(I\otimes\bar{K^k_i}^T))vec(\hat{\bar{B_i}}^TW) + \Gamma_{\eta_iv_i}vec(\hat{\bar{D_i}}^TW) + \Gamma_{\eta_i\theta_i}vec(W))$
又因为$\delta_ivecs(W) =\dot{\eta}_i^T X \eta_i + \eta_i^T X \dot{\eta}_i
\\=\Gamma_{\eta_i\eta_i} vecs(\bar{A}_{ci}^T W + W \bar{A}_{ci}) + 2(\Gamma_{\eta_iu_i}+\Gamma_{\eta_i}(I\otimes\bar{K}^k_i)^T) vec(\bar{B}_i^T W) + 2\Gamma_{\eta_iv_i} vec(\bar{D}_i^T W) + 2\Gamma_{\eta_i\theta_i} vec(W)
\\=2((\Gamma_{\eta_iu_i}+\Gamma_{\eta_i}(I\otimes\bar{K^k_i}^T))vec(\hat{\bar{B_i}}^TW) + \Gamma_{\eta_iv_i}vec(\hat{\bar{D_i}}^TW) + \Gamma_{\eta_i\theta_i}vec(W))$
因为$\hat{\bar{B}_i}=\bar{B}_i$, $\hat{\bar{D}_i}=\bar{D}_i$,所以
$\Gamma_{\eta_i\eta_i} vecs(\bar{A}_{ci}^T W + W \bar{A}_{ci})=0$
因为我们的条件为$rank(\Gamma_{\eta_i\eta_i}) = \frac{(n+n_z)(n+n_z+1)}{2}$
所以矩阵$\Gamma_{\eta_i\eta_i}$列满秩。因此必须满足$vecs(\bar{A}_{ci}^T W + W \bar{A}_{ci})=0$
即$\bar{A}_{ci}^T W + W \bar{A}_{ci}=0$
由于VI阶段的迭代更新规则，$K^k_i$会不断的迭代更新，最终会收敛到一个使得$\bar{A}_{ci}$ Hurwitz的控制器。
因此$\bar{A}_{ci}$是Hurwitz矩阵，所以$\bar{A}_{ci}^T W + W \bar{A}_{ci}=0$的唯一解是$W=0$。因此$\hat{\Psi^k_i}$列满秩。
## HI迭代过程
1. i=1
2. 选择一个小值$\varepsilon_i>0$作为收敛阈值。令$P^0_i=I_{n+n_z},选择一个\hat{Q}^0_i={\hat{Q}^0_i}^T>Q_i,k=0,reset=0$
3. repeat
4. Use $u^0_i = -K^0_i\eta_i+noise$ on $[t_0,t_s]$ to collect data to satisfy the rank condition
5. Solve B and D from the equation ${\Psi^0_i} \left[\begin{matrix}vecs(\bar{A}+\bar{A_i}^T)\\ vec({B_i}^T) \\ vec({D_i}^T)  \end{matrix}\right]={\Phi^0_i}$ and get ${\bar{B}_i},{\bar{D}_i}$
6. loop
7. Solve $H^k_i$ from the equation $\hat{\Psi^k_i} \left[\begin{matrix}vecs(H^k_{i})\end{matrix}\right]=\hat{\Phi^k_i}$
8. $\tilde{P}^{k+1}_i = P^k_i+\epsilon_k(H^k_i+\hat{Q}_i-P^k_i\bar{B}_iR^{-1}\bar{B}^T_iP^k_i)$
9.  if $\tilde{P}^{k+1}_i\notin \mathfrak{B}_r$ then reset $P^k_i$ to $I_{n+n_z}$, reset = reset+1.
10. else $P^{k+1}_i = \tilde{P}^{k+1}_i$ end if
11. k=k+1
12. until $\frac{P^{k}_i-P^{k-1}_i}{\epsilon_{k-1}}\prec  \hat{Q}_i$，并标记第k次迭代时切换到PI阶段。
13. repeat
14. Solve $P^k_i$ and update $K^{k+1}_i$ from the equation $\hat{\Psi^k_i} vecs(P^k_i) = \hat{\Phi^k_i}$ 
15. k=k+1
16. until $||P^{k}_i-P^{k-1}_i||<{\varepsilon}_i$
17. $k = k^*$
18. The learned controlled is $u_i = -K^{k^*}_i\eta_i$
19. i=i+1
20. until i>N
### HI收敛证明
根据已有文献，随着k趋向于无穷大，$||P^{k}_i-P^*_i||$会趋向于0。那么一定存在一个k达成切换条件，根据切换条件until $\frac{P^{k}_i-P^{k-1}_i}{\epsilon_{k-1}}\prec  \hat{Q}_i$,以及VI的更新规则。我们可以得到$P^{k-1}_i\bar{A_i} + \bar{A_i}^TP^{k-1}_i-P^{k-1}_i\bar{B}_iR^{-1}\bar{B}^T_iP^{k-1}_i\prec 0$。此时根据PI的增益更新规则，有$K^{k}_i = R^{-1}\bar{B}_i^TP^{k-1}_i$。
那么左边等于
$P^{k-1}_i(\bar{A_i}-\bar{B}_iK^{k}_i)+(\bar{A_i}-\bar{B_i}K^{k}_i)^TP^{k-1}_i+P^{k-1}_i\bar{B}_iK^k_i$