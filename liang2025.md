IEEE TRANSACTIONS ON VEHICULAR TECHNOLOGY

# Cooperative adaptive cruise control of connected and autonomous vehicles via hybrid iteration

Xue Liang, Weinan Gao, Senior Member, IEEE, Chuan Hu, and Tianyou Chai, Life Fellow, IEEE

Abstract—This paper proposes a novel data-driven approach, named hybrid iteration (HI), for the implementation of cooperative adaptive cruise control (CACC) in connected and autonomous vehicles (CAVs). The proposed method leverages the integration of adaptive dynamic programming (ADP), internal model principle, and distributed control technique. By collecting input-state data, an approximately optimal control policy is synthesized through the online iterative technique, HI, to optimize the performance of CACC by minimizing a predefined cost function. Note that the HI preserves the benefits of two established learning algorithms: policy iteration (PI) and value iteration (VI). The learning process initiates with the VI algorithm, followed by the application of the PI algorithm to expedite convergence. HI eliminates the necessity for an initial stabilizing control policy, which is a prerequisite in PI, and exhibits a faster convergence rate than VI. The efficiency and safety of the HI method in CACC systems have been demonstrated through platoon vehicle testing and verification in traffic congestion and cut-in scenarios.

Index Terms—Hybrid iteration, adaptive dynamic programming, connected and autonomous vehicles, cooperative adaptive cruise control.

## I. INTRODUCTION

THE progression of autonomous vehicle technology is imperative for the deployment of intelligent transportation systems (ITSs), aiming to reduce traffic accidents made by human drivers and fuel consumption. To further enhance the traffic safety and reliability, the introduction of wireless communication technology into vehicles has sparked significant interest among researchers across disciplines, including control systems $[1]$ – $[4]$ , optimization $[5]$ – $[7]$ , and communications $[8]$ – $[11]$ , in the design of connected and autonomous vehicle (CAV) systems. Among recent research efforts on CAVs to reduce road congestion and improve traffic efficiency, cooperative adaptive cruise control (CACC) is considered as one of the most empirical technologies for ITS applications $[12]$ – $[14]$ . CACC can be regarded as an extension of adaptive cruise control (ACC), enhancing its functionality with vehicle-to-vehicle (V2V) communication and cooperative control. By leveraging V2V technology, vehicles can collaborate more intelligently to enhance the driving experience as well as the overall efficiency and safety of the traffic system, with CACC relying not only on sensors but also on communication to exchange information, enabling vehicles to adapt to those ahead and mitigate the shockwave effect. A group of vehicles coordinated by CACC is referred to as a vehicular platoon. It is indicated that, in addition to reduced fuel consumption due to smoother traffic flow, a decrease in air resistance can also reduced fuel consumption when vehicles are closer within a platoon [15], [16]. The advent of CAV technologies has greatly improved traffic in terms of energy efficiency and road safety. The ecological control problem of CAV is studied, and the nonlinear optimal control problem is transformed into a convex problem by spatial domain modeling to improve the computational efficiency [17]. By adjusting speed in real-time and maintaining appropriate distances, the overall road efficiency is improved by minimizing sudden braking and acceleration.

There are several control challenges when designing CACC for vehicular platoons, including sensor failures $[18]$ , communication loss $[19]$ , communication delay $[20]$ , $[21]$ , and time-varying topologies $[22]$ . To be more specific, in $[18]$ , the authors establish a CACC framework that can tolerate sensor failures. By employing switched system theory and Lyapunov stability theory, a robust state-feedback controller is developed and a sufficient condition is presented such that the closed-loop system achieves string stability and zero steady-state error. Considering the case that the intraplatoon communication is lost such that the topology is changed, dynamic communication topology-based CAV-following model is constructed in $[19]$ to maintain the safety of vehicular platoons. In $[20]$ , the authors have combined optimal control and CACC to ensure its robustness against communication delay and mobility. In $[21]$ , the authors consider a multi-agent Markov decision process with internal delays in CACC, and designs a delay-aware multi-agent reinforcement learning (RL) method for CAVs to enhance platoon safety and stability. In $[22]$ , the authors integrate RL with distributed control technique, considering time-varying topological structures, to develop RL-based CACC that ensures stability of the closed-loop system.

Despite the extensive research on CACC, several technical challenges persist its practical implementation, necessitating further investigation. Firstly, current CACC approaches are predominantly model-based. The cornerstone of model-based control policies involves developing a mathematical representation of the system. However, even minor inaccuracies in system modeling can lead to significant issues. Controllers derived from inaccurate models may destabilize the vehicle system, posing risks to the safety of autonomous vehicles. Secondly, to enhance the dynamic performance of automated vehicles and to reduce fuel consumption, there is an imperative to design optimal controllers that minimize certain predefined cost functions. A pivotal requirement for most optimal control techniques is precise knowledge of the system model. To overcome the limitation of model-based CACC methods, data-driven CACC has been studied via RL $[22]$ – $[24]$ , iterative learning control $[25]$ . RL constitutes a methodology within machine learning that is usually employed for training agents to acquire optimal strategies through learning during their interactions with the environment. RL methods have been used to develop a CACC strategy to safely perform car-following. Function approximation techniques combined with gradient-descent learning algorithms are used as a means to directly modify the control policy and optimize its performance $[26]$ . Policy iteration (PI) and value iteration (VI) are two primary algorithmic approaches within RL that facilitate agents in identifying the optimal behavioral policies to achieve objectives within a specified environment. PI guarantees convergence to the optimal policy after a finite number of iterations, but it requires alternating between policy evaluation and policy improvement, which can result in a significant computational burden. VI is generally more efficient than PI because it updates the value function at each step without the need for an explicit policy representation. However, in some cases, VI may require more iterations to converge to the optimal value function. The hybrid iteration (HI) proposed in this paper effectively combines the advantages of both PI and VI. It first employs VI to learn a stabilizing policy and then initiates PI to accelerate the convergence speed.

Comparing with existing studies on CACC, this paper has three primary contributions.

\- Firstly, in contrast to the model-based CACC approaches in the existing literature [27]–[29], we have devised an efficient, data-driven HI learning method for the deployment of CACC in CAVs. The HI algorithm is capable of deriving an optimal controller with a satisfactory transient response even in the absence of explicit vehicle dynamics knowledge, utilizing data gathered online from vehicle trajectories without the requirement for model information.

\- Secondly, we have originally combined HI and distributed internal model for CACC design. Internal model principle is an important tool to address output regulation problem that concerns designing a controller such that the closed-loop system rejects external disturbance while asymptotically tracks some predefined references. Considering CAVs modeled as multi-agent systems, distributed control and internal model can be combined to develop distributed internal model. In contrast to centralized control, which is considered vulnerable due to its reliance on data from every autonomous vehicle within the network, distributed internal model-based control policies provide enhanced robustness against communication breakdowns or information loss. This is because they operate based solely on information from neighboring autonomous vehicles.

\- Thirdly, the proposed algorithm outperforms existing RL-based CACC algorithms such as PI and VI. In detail, VI does not require a stabilizing control policy, but its convergence speed is slower than that of PI. PI converges faster than VI, but it necessitates a stabilizing control policy. The proposed control policy in this paper merges the principles of PI and VI. Initially, a stabilizing control policy is inferred through the application of VI, which is subsequently refined by PI to enhance the convergence rate. HI capitalizes on the strengths of both VI and PI, thereby facilitating the realization of the desired control objectives. It can approximate the optimal control policy and corresponding cost function iteratively without needing an accurate knowledge of vehicle dynamics. In this paper, the actual traffic scenarios, such as traffic congestion and vehicle cut-in situations are introduced to deeply verify the efficiency and safety performance of the HI method in CACC. These scenarios are designed to simulate the complex traffic conditions in the real world, so as to comprehensively evaluate the performance of the HI method in dealing with emergency situations and maintaining the safe distance between vehicles. Through validation under complex scenarios, the HI method demonstrates significant performance advantages in CACC systems, effectively enhancing system efficiency while ensuring road safety and driving smoothness.

The rest of this paper is organized as follows. In Section II, we provide the system model and some preliminary knowledge, present the principles of distributed internal models, and review PI and VI algorithms. The design of data-driven CACC and the convergence analysis of algorithm are covered in Section III. In Section IV, simulation and traffic congestion scenarios involving cut-in maneuvers, the effectiveness and superior performance of the proposed control method are verified. Section V contains the conclusion and suggests future research directions.

Notations. Throughout this paper, R denotes the set of real numbers, while $Z_{+}$ denotes the set of nonnegative integers. $P^{n}$ denotes the set of all $n \times n$ real, symmetric and positive semidefinite matrices. $\|\cdot\|$ represents the Euclidean norm for vectors and the induced norm for matrices. $\otimes$ indicates the Kronecker product operator and $\operatorname{vec}(A) = [a_{1}^{T}, a_{2}^{T}, \cdots, a_{m}^{T}]^{T}$ , where $a_{i} \in R^{n}$ are the columns of $A \in R^{n \times m}$ . For a symmetric matrix $P = P^{T} \in R^{m \times m}$ , $\operatorname{vecs}(P) = [p_{11}, 2p_{12}, \cdots, 2p_{1m}, p_{22}, 2p_{23}, \cdots, 2p_{m-1,m}, p_{mm}]^{T} \in R^{\frac{1}{2}m(m+1)}$ . $\operatorname{vecv}(v) = [v_{1}^{2}, v_{1}v_{2}, \cdots, v_{1}v_{m}, v_{2}^{2}, v_{2}v_{3}, \cdots, v_{m-1}v_{m}, v_{m}^{2}]^{T} \in R^{\frac{1}{2}m(m+1)}$ . $P \succ (\succeq)0$ indicates that P is a positive definite (semidefinite) matrix, $P \prec (\preceq)0$ indicates that P is negative definite (semidefinite).

## II. PROBLEM FORMULATION AND PRELIMINARIES

In this paper, we consider a platoon of n vehicles traveling along a straight line. The dynamics of the i-th autonomous

IEEE TRANSACTIONS ON VEHICULAR TECHNOLOGY

vehicle can be described as follows [30]:

$$
\begin{array}{r l} & {\dot {p} _ {i} = v _ {i},} \\ & {\dot {v} _ {i} = a _ {i},} \\ & {\dot {a} _ {i} = - \tau_ {i} ^ {- 1} a _ {i} + \tau_ {i} ^ {- 1} u _ {i} - \frac {d _ {m i}}{m _ {i}} \tau_ {i} ^ {- 1}} \\ & {\quad := - \tau_ {i} ^ {- 1} a _ {i} + \tau_ {i} ^ {- 1} u _ {i} + \tilde {d} _ {i}} \end{array}\tag{1}
$$

where $p_{i}$ represents the position, $v_{i}$ denotes the speed of vehicle i, $a_{i}$ is the acceleration, $\tau_{i}$ denotes the time constant associated with its engine, $d_{mi}$ the mechanical drag, and $m_{i}$ represents the mass of vehicle.

Based on (1), we derive the following state-space representation for the i-th vehicle in the platoon.

$$
\dot {x} _ {i} = A _ {i} x _ {i} + B _ {i} u _ {i} + D _ {i} v, \quad i = 1, 2 \dots , N\tag{2}
$$

where $\tau_h$ is time headway, $v^{*}$ is desired velocity of the platoon, the state is $x_{i}^{T} = [p_{i} + iv^{*}\tau_{h}, v_{i}, a_{i}]$ , and control input $u_{i} \in \mathbb{R}$ ,

$$
A _ {i} = \left[ \begin{array}{c c c} 0 & 1 & 0 \\ 0 & 0 & 1 \\ 0 & 0 & - \tau_ {i} ^ {- 1} \end{array} \right], B _ {i} = \left[ \begin{array}{c} 0 \\ 0 \\ \tau_ {i} ^ {- 1} \end{array} \right], D _ {i} = \left[ \begin{array}{c c} 0 & 0 \\ 0 & 0 \\ 0 & \tilde {d} _ {i} \end{array} \right].
$$

Define $e_{i}$ is the tracking error and the state equation of the exosystem as follows

$$
\begin{array}{l} e _ {i} = C _ {i} x _ {i} + F v, \quad i = 1, 2 \dots , N \\ \dot {v} = E v \end{array}\tag{3}
$$

(4)

where

$$
C _ {i} = \left[ \begin{array}{c c c} 1 & 0 & 0 \end{array} \right], F = \left[ \begin{array}{c c} - v ^ {*} & 0 \end{array} \right], E = \left[ \begin{array}{c c} 0 & 1 \\ 0 & 0 \end{array} \right],
$$

and $e_{i} \in R$ are the tracking error of the i-th subsystem. The state vector $v \in R^{2}$ represents the state of an exosystem that generates the disturbance $D_{i}v$ and the reference signal $y_{0} = -Fv$ , which is to be tracked by the output $y_{i} = C_{i}x_{i}$ of each subsystem. Considering the exosystem (4) and the plant (2), we introduce a directed graph $G = \{V, E\}$ . The node set $V = \{0, 1, \cdots, N\}$ comprises node 0, which corresponds to the leader governed by the exosystem (4), along with the additional N node, which represent the followers characterized by the plant (2). The edge set $E \subset V \times V$ denotes the connections between the node. The adjacency matrix $A = [a_{ij}] \in \mathbb{R}^{(N+1) \times (N+1)}$ is defined such that $a_{ij} = 1$ if $(j, i) \in \mathcal{E}$ and otherwise $a_{ij} = 0$ . Then, the corresponding Laplacian L of G is

$$
\mathcal {L} = \left[ \begin{array}{c c} \sum_ {j = 1} ^ {N} a _ {0 j} & - [ a _ {0 1} \dots , a _ {0 N} ] \\ - \Delta \mathbf {1} _ {N} & \mathcal {H} \end{array} \right]
$$

where $\Delta = \operatorname{diag}\left\{a_{10}, a_{20}, \cdots, a_{N0}\right\}$ , and $H = [h_{ij}] \in R^{N \times N}$ is defined by $h_{ii} = \left(\sum_{j=0}^{N} a_{ij}\right) - a_{ii}$ and $h_{ij} = -a_{ij}$ for all $i \neq j$ . $N_i$ defines a set of neighbors of the i-th agent, and $N_i^+ := N_i / \{0\}$ .

The following standard assumptions are required for addressing the CACC problem for CAVs using data-driven approaches.

Assumption 1. $(A_{i}, B_{i})$ is stabilizable, $\forall 1 \leq i \leq N$ .

$$
\begin{array}{l} \text {Assumption 2. rank} \left[ \begin{array}{c c} A _ {i} - \lambda I & B _ {i} \\ C _ {i} & 0 \end{array} \right] = n _ {i} + 1,   \forall \lambda \in \sigma (E), \\ \forall 1 \leq i \leq N. \end{array}
$$

Assumption 3. The digraph G contains a directed spanning tree with the node 0 as the root.

If each subsystem independently maintains its own internal model based on data from local sensors and limited external interactions (information from neighboring nodes), the system can operate without relying on global information from a central server or other nodes. This allows for the construction of a decentralized internal model using output regulation theory, as described below:

$$
\dot {z} _ {i} = E z _ {i} + G _ {2} e _ {i}, \quad i = 1, 2, \dots , N.\tag{5}
$$

where $G_{2} = \left[ \begin{array}{ll}0 & 1 \end{array} \right]^{T}$ .

The implementation of a decentralized internal model (5) is based on the presumption that all followers can directly communicate with the leader to acquire $e_{i}$ . Nonetheless, this presumption may not hold in light of the communication constraints among agents. Therefore, one can not use $e_{i}$ as a feedback to design internal model. In our earlier research [31], we define $\hat{e}_{i}$ as a weighted sum of tracking errors of followers.

$$
\hat {e} _ {i} = \sum_ {j \in \mathcal {N} _ {i}} \frac {a _ {i j} (y _ {i} - y _ {j})}{\sum_ {j = 0} ^ {N} a _ {i j}}.\tag{6}
$$

Then, we introduce a distributed internal model as follows

$$
\dot {z} _ {i} = E z _ {i} + G _ {2} \hat {e} _ {i}, \quad i = 1, 2, \dots , N.\tag{7}
$$

The following Lemma 1 is given to show the cooperative output regulation problem [32]–[34] can be addressed by developing a state-feedback controller with distributed internal model.

Lemma 1. [31, Theorem 1] Under Assumptions 1-3, the augmented system

$$
\begin{array}{r l} & {\dot {v} = E v} \\ & {\dot {x} _ {i} = A _ {i} x _ {i} + B _ {i} u _ {i} + D _ {i} v,} \\ & {\dot {z} _ {i} = E z _ {i} + G _ {2} \hat {e} _ {i},} \\ & {e _ {i} = C _ {i} x _ {i} + F v, \quad i = 1, 2 \dots , N} \end{array}\tag{8}
$$

in closed-loop with the state-feedback controller

$$
u _ {i} = - K _ {x i} x _ {i} - K _ {z i} z _ {i}, i = 1, 2, \dots , N,\tag{9}
$$

achieves cooperative output regulation if the matrix

$$
A _ {c i} = \left[ \begin{array}{c c} A _ {i} - B _ {i} K _ {x i} & - B _ {i} K _ {z i} \\ G _ {2} C _ {i} & E \end{array} \right]
$$

is Hurwitz.

To facilitate the presentation, we define the following vectors and matrices to consolidate the states, inputs, and system

IEEE TRANSACTIONS ON VEHICULAR TECHNOLOGY

matrices.

$$
\begin{array}{l} \tilde {u} = [ \tilde {u} _ {1}, \tilde {u} _ {2}, \dots , \tilde {u} _ {N} ] ^ {T}, \\ \bar {A} = \text {blockdiag} (\bar {A} _ {1}, \bar {A} _ {2}, \dots , \bar {A} _ {N}), \\ \bar {B} = \text {blockdiag} (\bar {B} _ {1}, \bar {B} _ {2}, \dots , \bar {B} _ {N}), \\ Q = \text {blockdiag} (Q _ {1}, Q _ {2}, \dots , Q _ {N}), \\ R = \text {blockdiag} (R _ {1}, R _ {2}, \dots , R _ {N}), \end{array}
$$

where $\tilde{u}_{i}=u_{i}-U_{i}v$ , $Q_{i}=Q_{i}^{T}>0$ , $R_{i}=R_{i}^{T}>0$ . Define $U_{i}=-K_{xi}X_{i}-K_{zi}Z_{i}$ , for $i=1,2,\cdots,N$ .

$$
\begin{array}{r l} & {\tilde {x} _ {i} = x _ {i} - X _ {i} v, \quad \tilde {z} _ {i} = z _ {i} - Z _ {i} v,} \\ & {\tilde {\xi} _ {i} = \left[ \begin{array}{l} \tilde {x} _ {i} \\ \tilde {z} _ {i} \end{array} \right] \in \mathbb {R} ^ {5}, \quad K _ {i} = \left[ \begin{array}{l l} K _ {x i} & K _ {z i} \end{array} \right],} \\ & {\bar {A} _ {i} = \left[ \begin{array}{l l} A _ {i} & 0 \\ G _ {2} C _ {i} & E \end{array} \right], \quad \bar {B} _ {i} = \left[ \begin{array}{l} B _ {i} \\ 0 \end{array} \right], i = 1, 2, \dots , N,.} \end{array}\tag{10}
$$

Remark 1. Notably, under Assumptions 1-3, the pair $(\bar{A}_{i}, \bar{B}_{i})$ is stabilizable for all $i = 1, 2, \cdots, N$ . This guarantees the existence of a feedback gain $K_{i}$ such that the closed-loop matrix $A_{ci}$ is Hurwitz. Furthermore, the matrices $(X_{i}, U_{i})$ correspond to the unique solutions of the regulator equations. For a rigorous proof of this result, see [Lemma 1.26] in [35] for more details.

Now, we can design an optimal controller by minimizing the following cost function.

Problem 1.

$$
\begin{array}{r l} & {\underset {\tilde {u}} {\min} \int_ {0} ^ {\infty} \left(\tilde {\xi} ^ {T} Q \tilde {\xi} + \tilde {u} ^ {T} R \tilde {u}\right) d t} \\ {s. t.} & {\dot {\tilde {\xi}} = \bar {A} \tilde {\xi} + \bar {B} \tilde {u}} \end{array}\tag{11}
$$

(12)

where

$$
\tilde {\xi} = [ \tilde {\xi} _ {1} ^ {T}, \tilde {\xi} _ {2} ^ {T}, \dots , \tilde {\xi} _ {N} ^ {T} ] ^ {T}, f o r i = 1, 2, \dots , N.
$$

Based on the linear optimal control theory, the optimal control policy solving Problem 1 is

$$
\tilde {u} ^ {*} = - K ^ {*} \tilde {\xi}.
$$

Since A, B, Q and R are block diagonal matrices, we can derive that $K^{*}$ is also a block diagonal matrix representing by $K^{*} = \text{blockdiag}(K_{1}^{*}, K_{2}^{*}, \cdots, K_{N}^{*})$ , where the optimal control gain for the i-th follower is

$$
K _ {i} ^ {*} = R _ {i} ^ {- 1} \bar {B} _ {i} ^ {T} P _ {i} ^ {*} := \left[ \begin{array}{c c} K _ {x i} ^ {*} & K _ {z i} ^ {*} \end{array} \right].\tag{13}
$$

The matrix $P_{i}^{*}$ is the unique solution to the following algebraic Riccati equation

$$
\bar {A} _ {i} ^ {T} P _ {i} ^ {*} + P _ {i} ^ {*} \bar {A} _ {i} + Q _ {i} - P _ {i} ^ {*} \bar {B} _ {i} R _ {i} ^ {- 1} \bar {B} _ {i} ^ {T} P _ {i} ^ {*} = 0.\tag{14}
$$

Therefore, the optimal controller is equivalent to

$$
\begin{array}{r l} & u _ {i} ^ {*} = \tilde {u} _ {i} ^ {*} + U _ {i} v \\ & \quad = - K _ {i} ^ {*} \tilde {\xi} _ {i} + U _ {i} v \\ & \quad = - K _ {x i} ^ {*} \tilde {x} _ {i} - K _ {z i} ^ {*} \tilde {z} _ {i} + U _ {i} v \\ & \quad = - K _ {x i} ^ {*} x _ {i} - K _ {z i} ^ {*} z _ {i}, \quad i = 1, 2, \dots , N, \end{array}\tag{15}
$$

which is similar to the controller (9).

Moreover, since the pair $(\bar{A}_{i},\bar{B}_{i})$ is stabilizable, it is straightforward to have the state matrix of closed-loop system

$$
A _ {c i} ^ {*} = \left[ \begin{array}{c c} A _ {i} - B _ {i} K _ {x i} ^ {*} & - B _ {i} K _ {z i} ^ {*} \\ G _ {2} C _ {i} & E \end{array} \right]
$$

is Hurwitz for all the followers. In light of Lemma 1, the optimal controller (15) ensures the cooperative output regulation is achieved. It is worth noting that (14) nonlinear in $P_{i}^{*}$ . This makes it difficult to find $P_{i}^{*}$ directly from equation (14). To solve (14), two successive approximation methods, VI and PI, can be applied and their details are given in the following subsections.

## A. Policy Iteration

The PI algorithm is a commonly used method for approximating the optimal feedback control law. It consists of two steps: policy evaluation and policy improvement. For policy evaluation, Kleinman [36] first introduced the iterative method in linear systems, followed by the use of value functions to estimate the feedback law. For policy improvement, the estimate of the value function is updated to reduce the approximation error. The main ideas of PI algorithm are briefly reviewed below.

• Policy Evaluation: Solve $P_{i}^{(k)}$ from

$$
\begin{array}{r l} 0 = & P _ {i} ^ {(k)} \left(\bar {A} _ {i} - \bar {B} _ {i} K _ {i} ^ {(k)}\right) + \left(\bar {A} _ {i} - \bar {B} _ {i} K _ {i} ^ {(k)}\right) ^ {T} P _ {i} ^ {(k)} + Q _ {i} \\ & + \left(K _ {i} ^ {(k)}\right) ^ {T} R _ {i} K _ {i} ^ {(k)} \end{array} \tag {16}
$$

\- Policy Improvement: Update the control gain matrix by

$$
K _ {i} ^ {(k + 1)} = R _ {i} ^ {- 1} \bar {B} _ {i} ^ {T} P _ {i} ^ {(k)}\tag{17}
$$

## B. Value Iteration

Unlike PI, VI eases the learning process by addressing the challenge of knowing the stable control gain matrix $K_{i}^{(0)}$ for each subsystem. The fundamental approach involves solving the problem by a step-by-step approach. It starts with an initial value matrix $P_{i}^{(0)}$ . The solution is then gradually updated according to value update until the solution meets certain stopping criterion. In this way, one can gradually approach to the optimal value. The iteration process for VI is as follows.

\- Value Update: Given $P_{i}^{(0)}$ , update the value matrix using

$$
\begin{array}{r l} & P _ {i} ^ {(k + 1)} \leftarrow \epsilon_ {k} (P _ {i} ^ {(k)} \bar {A} _ {i} + \bar {A} _ {i} ^ {T} P _ {i} ^ {(k)} \\ & \qquad + Q _ {i} - P _ {i} ^ {(k)} \bar {B} _ {i} R _ {i} ^ {- 1} \bar {B} _ {i} ^ {T} P _ {i} ^ {(k)}) \\ & \qquad + P _ {i} ^ {(k)}, i = 1, 2, \ldots , N, \end{array}\tag{18}
$$

where $\epsilon_{k}$ is deterministic sequence and satisfied

$$
\epsilon_ {k} > 0, \quad \sum_ {k = 0} ^ {\infty} \epsilon_ {k} = \infty , \quad \sum_ {k = 0} ^ {\infty} \epsilon_ {k} ^ {2} <   \infty .\tag{19}
$$

The convergence proof of the VI algorithm has been provided in [37].

## III. MAIN RESULTS

In this Section, we propose a novel HI approaches for CACC design, which is mainly divided into two states. In the first phase, VI is used to find a stabilizing control policy. once the stabilizing control gain is found, the PI is switched until it converges to the optimal solution.

We start from model-based HI for CACC design, then move to data-driven HI.

## A. Model-Based HI for CACC

1) Phase 1: Finding a stabilizing control policy.

Throughout the first phase, the value matrix is iteratively updated until a stabilizing control policy is found. It is worth emphasizing that the idea at this phase uses stochastic approximation for value updating. Firstly, $\{B_{r}\}_{0}^{\infty}$ is defined as a set non-empty internally bounded set, which satisfies

$$
\mathfrak {B} _ {r} \subset \mathfrak {B} _ {r + 1} \in \mathcal {P} ^ {n}, r \in \mathbb {Z} _ {+}, \lim _ {q \rightarrow \infty} \mathfrak {B} _ {r} = \mathcal {P} ^ {n}.
$$

and $\varepsilon_{i} > 0$ is a small threshold. Additionally, the selected deterministic sequence $\epsilon_{k}$ satisfies condition (19).

Since the primary objective of Phase 1 is to find a stabilizing policy for the i-th subsystem, we can assume that there exists a $\hat{Q}_{i}$ such that $\hat{Q}_{i} > Q_{i}$ . Replace $\hat{Q}_{i}$ with $Q_{i}$ and continue the update steps until a stabilizing control policy is achieved. Once a stabilizing control policy is obtained, conclude Phase 1 and then implement this stabilizing control policy during Phase 2.

2) Phase 2: The second phase explores the optimal control policy CACC.

In the second phase, PI will be used, using the control policy obtained from the first phase. Repeat the policy evaluation in (16) and the policy improvement in (17) until the value matrix $P_{i}^{(k)}$ is close enough to $P_{i}^{(*)}$ , $i = 1, 2, \ldots, N$ . Algorithm 1 gives the detailed steps of the model-based HI algorithm for CACC.

Remark 2. The HI algorithm offers two distinct advantages over traditional algorithms: it exhibits faster learning speed compared to the VI algorithm, and unlike the PI algorithm, it does not require a stabilizing policy to be pre-established.

Remark 3. Algorithm 1 is designed to iteratively optimize the matrices $P_i^{(*)}$ and $K_i^{(*)}$ for developing stable controllers. The outer loop starts at $i = 1$ and continues until $i = N + 1$ , ensuring all models are processed. The inner structure comprises two nested loops. The first inner loop, parameters are initialized by selecting a sufficiently small $\varepsilon_i > 0$ and positive-definite matrices $P_i^{(0)}$ and $Q_i^{(0)}$ . The matrix $P_i$ is then updated iteratively via the Riccati equation. After each update, $\bar{P}_i^{(k+1)}$ is checked to verify if it remains within the stability region $\mathfrak{B}_r$ . If violated, $P_i$ is reset to its initial value $P_i^{(0)}$ , and the counter $r$ is incremented. The loop terminates at Step 9 when the convergence criterion is met. The second inner loop focuses on validating the controller gain $K_i^{(*)}$ . At Step 12, $P_i$ is further updated using the Lyapunov equation to enforce closed-loop stability. The loop stops at Step 13 when the variation $||P_i^{(k)} - P_i^{(k-1)}|| < \varepsilon_i$ , confirming the convergence of the control gain $K_{i}^{(*)}$ . The algorithm concludes after processing all models.

<div class="mineru-algorithm" style="white-space: pre-wrap; font-family:monospace;">
Algorithm 1 Model-Based HI
1:  $i \leftarrow 1$ 
2: repeat
3: Select a small  $\varepsilon_{i} &gt; 0$ ,  $P_{i}^{(0)} = P_{i}^{T(0)} \succ 0$ , and  $\hat{Q}_{i}^{(0)} = \hat{Q}_{i}^{T(0)} \succ Q_{i}$ .  $k, r \leftarrow 0$ .
4: repeat
5:  $\bar{P}_{i}^{(k+1)} \leftarrow \epsilon_{k}(P_{i}^{(k)}\bar{A}_{i} + \bar{A}_{i}^{T}P_{i}^{(k)} + \hat{Q}_{i} - P_{i}^{(k)}\bar{B}_{i}R_{i}^{-1}\bar{B}_{i}^{T}P_{i}^{(k)}) + P_{i}^{(k)}$ ,
6: if  $\bar{P}_{i}^{(k+1)} \notin \mathfrak{B}_{r}$  then  $P_{i}^{(k+1)} \leftarrow P_{i}^{(0)}$ ,  $r \leftarrow r + 1$ .
7: else  $P_{i}^{(k+1)} \leftarrow \bar{P}_{i}^{(k+1)}$ , endif
8:  $k \leftarrow k + 1$ 
9: until  $\bar{P}_{i}^{(k)} - P_{i}^{(k-1)} / \epsilon_{k-1} &lt; \hat{Q}_{i}$ 
10: repeat
11:  $K_{i}^{(k)} \leftarrow R_{i}^{-1}\bar{B}_{i}^{T}P_{i}^{(k-1)}$ 
12: Solve  $P_{i}^{(k)}$  from  $P_{i}^{(k)}(\bar{A}_{i} - \bar{B}_{i}K_{i}^{(k)}) + (\bar{A}_{i} - \bar{B}_{i}K_{i}^{(k)})^{T}P_{i}^{(k)} + Q_{i} + (K_{i}^{(k)})^{T}R_{i}K_{i}^{(k)} = 0. k \leftarrow k + 1.$ 
13: until  $||P_{i}^{(k)} - P_{i}^{(k-1)}|| &lt; \varepsilon_{i}$ 
14:  $i \leftarrow i + 1$ 
15: until  $i = N + 1$
</div>

![](images/f511fcdf7411ed2f68a8a31b588f25940b70ffcf7bfb04d1d7ffe1def9e37513.jpg)  
Fig. 1. Illustration of a HI method

Theorem 1. Sequences $\left\{P_{i}^{(k)}\right\}_{k=0}^{\infty}$ and $\left\{K_{i}^{(k)}\right\}_{k=1}^{\infty}$ computed by Algorithm 1 converge to $P_{i}^{*}$ and $K_{i}^{*}, i=1,2,\cdots,N$ , respectively.

Proof: According to [37], As $k$ goes to infinity, we have $\| P_i^{(k)} - P_i^*\| \to 0$ . Based on the condition in step 9, that is, $\bar{P}_i^{(k)} - P_i^{(k - 1)} / \epsilon_{k - 1} < \hat{Q}_i$ , we can obtain

$$
P _ {i} ^ {(k - 1)} \bar {A} _ {i} + \bar {A} _ {i} ^ {T} P _ {i} ^ {(k - 1)} - P _ {i} ^ {(k - 1)} \bar {B} _ {i} R _ {i} ^ {- 1} \bar {B} _ {i} ^ {T} P _ {i} ^ {(k - 1)} \prec 0,
$$

which implies the following inequality:

$$
\begin{array}{l} P _ {i} ^ {(k - 1)} (\bar {A} _ {i} - \bar {B} _ {i} K _ {i} ^ {(k)}) + (\bar {A} _ {i} - \bar {B} _ {i} K _ {i} ^ {(k)}) ^ {T} P _ {i} ^ {(k - 1)} \\ \prec (K _ {i} ^ {(k)}) ^ {T} R _ {i} K _ {i} ^ {(k)} \preceq 0 \end{array}
$$

where $K_{i}^{(k)} = R_{i}^{-1}B_{i}^{T}P_{i}^{k - 1}$ . Therefore, given any $\hat{Q}_i =$ $\hat{Q}_i^T > 0$ , a stabilizing control gain $K_i^{(k_a)} = R_i^{-1}\bar{B}_i^T P_i^{(k_a - 1)}$ can always be obtained such that the matrix $\bar{A}_i - \bar{B}_i K_i^{(k_a)}$ is Hurwitz when steps 3-9 are finished at iteration $k_a \in \mathbb{Z}_+$ . Starting from the stabilizing control gain $K_i^{(k_a)}$ , according to the [36, Theorem], repeating steps 10-13 will lead to the convergence to the optimal solution $\left(P_i^{(*)}, K_i^{(*)}\right)$ .

Theorem 2. Sequences $\left\{P_{i}^{(k)}\right\}_{k=0}^{\infty}$ and $\left\{K_{i}^{(k)}\right\}_{k=1}^{\infty}$ , computed by HI Algorithm 1, converge to $P_{i}^{*}$ and $K_{i}^{*}$ , respectively, at a quadratic convergence rate for $i=1,2,\cdots,N$ .

Proof: Using Algorithm 1, when switching to PI at steps 10-13, the following results can be obtained from equations (16) and (17) of the PI method. We can get

$$
\begin{array}{l} (\bar {A} _ {i} - \bar {B} _ {i} K _ {i} ^ {(k + 1)}) ^ {T} (P _ {i} ^ {(k + 1)} - P _ {i} ^ {(*)}) \\ \quad + (P _ {i} ^ {(k + 1)} - P _ {i} ^ {(*)}) (\bar {A} _ {i} - \bar {B} _ {i} K _ {i} ^ {(k + 1)}) \\ = (\bar {A} _ {i}) ^ {T} P _ {i} ^ {(k + 1)} - (\bar {A} _ {i}) ^ {T} P _ {i} ^ {(*)} - (K _ {i} ^ {(k + 1)}) ^ {T} \bar {B} _ {i} ^ {T} P _ {i} ^ {(k + 1)} \\ \quad + (K _ {i} ^ {(k + 1)}) ^ {T} \bar {B} _ {i} ^ {T} P _ {i} ^ {(*)} + P _ {i} ^ {(k + 1)} \bar {A} _ {i} - P _ {i} ^ {(k + 1)} \bar {B} _ {i} K _ {i} ^ {(k + 1)} \\ \quad - P _ {i} ^ {(*)} \bar {A} _ {i} + P _ {i} ^ {(*)} \bar {B} _ {i} K _ {i} ^ {(k + 1)} + (K _ {i} ^ {(*)}) ^ {T} \bar {B} _ {i} ^ {T} P _ {i} ^ {(*)} \\ \quad - (K _ {i} ^ {(*)}) ^ {T} \bar {B} _ {i} ^ {T} P _ {i} ^ {(*)} + P _ {i} ^ {(*)} \bar {B} _ {i} K _ {i} ^ {(*)} - P _ {i} ^ {(*)} \bar {B} _ {i} K _ {i} ^ {(*)} \\ = - Q _ {i} - (K _ {i} ^ {(k + 1)}) ^ {T} R _ {i} K _ {i} ^ {(k + 1)} - (\bar {A} _ {i} - \bar {B} _ {i} K _ {i} ^ {(*)}) ^ {T} P _ {i} ^ {(*)} \\ \quad - P _ {i} ^ {(*)} (\bar {A} _ {i} - \bar {B} _ {i} K _ {i} ^ {(*)}) - (K _ {i} ^ {(*)} - K _ {i} ^ {(k + 1)}) ^ {T} \bar {B} _ {i} ^ {T} P _ {i} ^ {(*)} \\ \quad - P _ {i} ^ {(*)} \bar {B} _ {i} (K _ {i} ^ {(*) - K _ {i}} ^ {(k + 1)}) \\ = - Q _ {i} - (K _ {i} ^ {(k + 1)}) ^ {T} R _ {i} K _ {i} ^ {(k + 1)} + Q _ {i} + K _ {i} ^ {(*) *} R _ {i} K _ {i} ^ {(*) *} \\ \quad - (K _ {i} ^ {(*) - K _ {i}} ^ {(k + 1)}) ^ {T} R _ {i} K _ {i} ^ {*} - K _ {i} ^ {*} R _ {i} (K _ {i} ^ {(*) - K _ {i}} ^ {(k + 1)}) \\ = - (K _ {i} ^ {(k + 1)} - K _ {i} ^ {(*)}) ^ {T} R _ {i} (K _ {i} ^ {(k + 1)} - K _ {i} ^ {*}) \\ = - (P _ {i} ^ {(k)} - P _ {i} ^ {(*)}) \bar {B} _ {i} R _ {- 1} (\bar {B} _ {-}) ^ \top T (P _ - i k l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l / c m e d. \end{array}\tag{20}
$$

According to (20), we can obtain

$$
\begin{array}{l} P _ {i} ^ {(k + 1)} - P _ {i} ^ {(*)} = \int_ {0} ^ {\infty} \mathrm{e} ^ {(\bar {A} _ {i} - \bar {B} _ {i} K _ {i} ^ {(k + 1)}) ^ {\tau}} (P _ {i} ^ {(k)} - P _ {i} ^ {(*)}) \\ \times \bar {B} _ {i} R _ {i} ^ {- 1} (\bar {B} _ {i}) ^ {T} (P _ {i} ^ {(k)} \\ - P _ {i} ^ {(*)}) \mathrm{e} ^ {(\bar {A} _ {i} - \bar {B} _ {i} K _ {i} ^ {(k + 1)}) ^ {\tau}} \mathrm{d} \tau \end{array}
$$

This means that there exist constants $\gamma_{p}$ and $\gamma_{k}$ , such as,

$$
\lim _ {k \to \infty} \frac {\| P _ {i} ^ {(k + 1)} - P _ {i} ^ {(*)} \|}{\| P _ {i} ^ {(k)} - P _ {i} ^ {(*)} \| ^ {2}} = \gamma_ {p},
$$

$$
\lim _ {k \to \infty} \frac {\| K _ {i} ^ {(k + 1)} - K _ {i} ^ {(*)} \|}{\| K _ {i} ^ {(k)} - K _ {i} ^ {(*)} \| ^ {2}} = \gamma_ {k}.
$$

Thus, the sequences $\left\{P_{i}^{(k)}\right\}_{k=0}^{\infty}$ and $\left\{K_{i}^{(k)}\right\}_{k=1}^{\infty}$ both converge quadratically to their optimal values. So the proof is done.

Remark 4. In the generalized policy iteration (GPI) method, policy evaluation is replaced by iterative approximation; if a sufficiently large number of iterations are used for approximation, it is guaranteed that the updated policy is admissible. Compared to existing PI and GPI methods [38], the proposed HI algorithm no longer relies on an admissible control policy to initiate the learning process. The online implementation of the GPI assumes the dynamics are known. However, this assumption is completely removed in our proposed off-policy HI Algorithm 2.

## B. Data-Driven CACC design

The key point to address Problem 1 is to solve the Riccati equation (14), which usually depends on the knowledge of system matrices. In this section, we will design a data-driven HI algorithm to explore the optimal CACC policy. The developed approaches are capable of approximating the control gains $K_{i}^{*}$ for each follower without requiring knowledge of the system matrices $A_{i}, B_{i}$ and $D_{i}$ .

The leader's state information is required by all followers. To accommodate this requirement, we develop the following estimator of leader's state $v$ .

$$
\dot {\zeta} _ {i} = E \zeta_ {i} + \mu \sum_ {j \in \mathcal {N} _ {i}} a _ {i j} (\zeta_ {j} - \zeta_ {i}), i = 1, 2, \dots , N,\tag{21}
$$

with $\zeta_{0}=v$ , where $\mu>0$ , as time approaches infinity, the estimation error $\tilde{\zeta}_{i}:=\zeta_{i}-v$ converges to zero.

Then, we rewrite the i-th subsystem and use the internal model (7):

$$
\begin{array}{l} \dot {\xi} _ {i} = \bar {A} _ {i} \xi_ {i} + \bar {B} _ {i} u _ {i} + \left[ \begin{array}{c} D _ {i} \\ \alpha_ {i 0} G _ {2} F \end{array} \right] v - \sum_ {j \in \mathcal {N} _ {i} ^ {+}} \alpha_ {i j} \left[ \begin{array}{c} 0 \\ G _ {2} \bar {C} _ {j} \end{array} \right] \xi_ {j} \\ = \bar {A} _ {i} ^ {(k)} \xi_ {i} + \bar {B} _ {i} \left(K _ {i} ^ {(k)} \xi_ {i} + u _ {i}\right) + \left[ \begin{array}{c} D _ {i} \\ \alpha_ {i 0} G _ {2} F \end{array} \right] \zeta_ {i} \\ - \sum_ {j \in \mathcal {N} _ {i} ^ {+}} \alpha_ {i j} \left[ \begin{array}{c} 0 \\ G _ {2} \bar {C} _ {j} \end{array} \right] \xi_ {j} + \left[ \begin{array}{c} - D _ {i} \tilde {\zeta} _ {i} \\ - \alpha_ {i 0} G _ {2} F \tilde {\zeta} _ {i} \end{array} \right] \\ = \bar {A} _ {i} ^ {(k)} \xi_ {i} + \bar {B} _ {i} \left(K _ {i} ^ {(k)} \xi_ {i} + u _ {i}\right) + \Delta_ {i 0} \zeta_ {i} \\ - \sum_ {j \in \mathcal {N} _ {i} ^ {+}} \Delta_ {i j} \xi_ {j} + \left[ \begin{array}{c} - D _ {i} \tilde {\zeta} _ {i} \\ - \alpha_ {i 0} G _ {2} F \tilde {\zeta} _ {i} \end{array} \right] \end{array}
$$

where, $\bar{A}_{i}^{(k)} = \bar{A}_{i} - \bar{B}_{i}K_{i}^{(k)}, \bar{C}_{j} = [C_{j} 0] \in \mathbb{R}^{1 \times 5}, \Delta_{ij} = \begin{bmatrix} 0 \\ \alpha_{ij}G_{2}\bar{C}_{j} \end{bmatrix}$ and $\Delta_{i0} = \begin{bmatrix} D_{i} \\ \alpha_{i0}G_{2}F \end{bmatrix}$ , and $\alpha_{ij} = a_{ij} / (\sum_{j=0}^{N} a_{ij})$ for $i = 1, 2, \cdots, N$ .

By changing the value of $\mu$ , the value of $\tilde{\zeta}_i$ can be altered, so it can be denoted as

$$
\Upsilon_ {i} (t) := \left[ \begin{array}{c} - D _ {i} \tilde {\zeta} _ {i} \\ - \alpha_ {i 0} G _ {2} F \tilde {\zeta} _ {i} \end{array} \right],
$$

By tuning $\mu$ , it will converge to zero at any desired rate.

The model can then be approximated by

$$
\dot {\xi} _ {i} = \bar {A} _ {i} ^ {(k)} \xi_ {i} + \bar {B} _ {i} \left(K _ {i} ^ {(k)} \xi_ {i} + u _ {i}\right) + \Delta_ {i 0} \zeta_ {i} - \sum_ {j \in \mathcal {N} _ {i} ^ {+}} \Delta_ {i j} \xi_ {j}.\tag{22}
$$

1) Phase 1: Find a Stabilizing CACC Policy

Firstly, a data-driven VI method is proposed to solve the CACC problem. The main purpose of using this method is to obtain a stabilizing control policy. Set up a Lyapunov function

IEEE TRANSACTIONS ON VEHICULAR TECHNOLOGY

$V_{k}(\xi_{i}) = \xi_{i}^{T}P_{i}^{(k)}\xi_{i}$ , where $k\in \mathbb{Z}_{+},i = 1,2,\ldots ,N$ , next, we use mathematical techniques, we get the following

$$
\begin{array}{l} \dot {V} _ {k} (\xi_ {i}) = \dot {\xi} _ {i} ^ {T} P _ {i} ^ {(k)} \xi_ {i} + \xi_ {i} ^ {T} P _ {i} ^ {(k)} \dot {\xi} _ {i} \\ = \left(\bar {A} _ {i} ^ {(k)} \xi_ {i} + \bar {B} _ {i} \left(K _ {i} ^ {(k)} \xi_ {i} + u _ {i}\right) + \Delta_ {i 0} \zeta_ {i} - \sum_ {j \in \mathcal {N} _ {i} ^ {+}} \Delta_ {i j} \xi_ {j} \right. \\ \left. + \Upsilon_ {i}\right) ^ {T} P _ {i} ^ {(k)} \xi_ {i} + \xi_ {i} ^ {T} P _ {i} ^ {(k)} \left(\bar {A} _ {i} ^ {(k)} \xi_ {i} \right. \\ \left. + \bar {B} _ {i} \left(K _ {i} ^ {(k)} \xi_ {i} + u _ {i}\right) + \Delta_ {i 0} \zeta_ {i} - \sum_ {j \in \mathcal {N} _ {i} ^ {+}} \Delta_ {i j} \xi_ {j} + \Upsilon_ {i}\right) \\ = \xi_ {i} ^ {T} \left(\bar {A} _ {i} ^ {T} P _ {i} ^ {(k)} + P _ {i} ^ {(k)} \bar {A} _ {i}\right) \xi_ {i} + 2 u _ {i} ^ {T} R _ {i} K _ {i} ^ {(k + 1)} \xi_ {i} \\ + 2 \zeta_ {i} ^ {T} \Delta_ {i 0} ^ {T} P _ {i} ^ {(k)} \xi_ {i} - 2 \sum_ {j \in \mathcal {N} _ {i} ^ {+}} \xi_ {j} ^ {T} \Delta_ {i j} ^ {T} P _ {i} ^ {(k)} \xi_ {i} \\ + 2 \Upsilon_ {i} ^ {T} P _ {i} ^ {(k)} \xi_ {i}. \end{array} \tag {23}
$$

Letting

$$
\begin{array}{c} \psi_ {i} = [ \xi_ {j 1} ^ {T}, \xi_ {j _ {2}} ^ {T}, \dots , \xi_ {j _ {| \mathcal {N} _ {i} ^ {+} |}} ^ {T} ] ^ {T} \in \mathbb {R} ^ {p _ {i}}, \\ \Xi_ {i} ^ {(k)} = \left[ P _ {i} ^ {(k)} \Delta_ {i j _ {1}}, P _ {i} ^ {(k)} \Delta_ {i j _ {2}}, \dots , P _ {i} ^ {(k)} \Delta_ {i j _ {| \mathcal {N} _ {i} ^ {+} |}} \right] ^ {T} \in \mathbb {R} ^ {p _ {i} \times 5}. \end{array}\tag{24}
$$

Given $\Upsilon_{i} = 0$ , by Kronecker product representation, we get

$$
\begin{array}{l} \xi_ {i} ^ {T} \left(\bar {A} _ {i} ^ {T} P _ {i} ^ {(k)} + P _ {i} ^ {(k)} \bar {A} _ {i}\right) \xi_ {i} = (\xi_ {i} ^ {T} \otimes \xi_ {i} ^ {T}) \operatorname{vec} \left(\bar {A} _ {i} ^ {T} P _ {i} ^ {(k)} \right. \\ \quad \left. + P _ {i} ^ {(k)} \bar {A} _ {i}\right), \\ u _ {i} ^ {T} R _ {i} K _ {i} ^ {(k + 1)} \xi_ {i} = (\xi_ {i} ^ {T} \otimes u _ {i} ^ {T}) (I \otimes R _ {i}) \operatorname{vec} (K _ {i} ^ {(k + 1)}), \\ \zeta_ {i} ^ {T} \Delta_ {i 0} ^ {T} P _ {i} ^ {(k)} \xi_ {i} = (\xi_ {i} ^ {T} \otimes \zeta_ {i} ^ {T}) \operatorname{vec} (\Delta_ {i 0} ^ {T} P _ {i} ^ {(k)}), \\ \sum_ {j \in \mathcal {N} _ {i} ^ {+}} \xi_ {j} ^ {T} \Delta_ {i j} ^ {T} P _ {i} ^ {(k)} \xi_ {i} = (\xi_ {i} ^ {T} \otimes \psi_ {i} ^ {T}) \operatorname{vec} (\Xi_ {i} ^ {(k)}). \end{array} \tag {25}
$$

Let us integrate $t_0 - t_1$ on both sides of (23)

$$
\begin{array}{c} \xi_ {i} (t _ {1}) ^ {T} P _ {i} ^ {(k)} \xi_ {i} (t _ {1}) - \xi_ {i} (t _ {0}) ^ {T} P _ {i} ^ {(k)} \xi_ {i} (t _ {0}) \\ = \int_ {t _ {0}} ^ {t _ {1}} (\xi_ {i} ^ {T} \otimes \xi_ {i} ^ {T}) \mathrm{vec} \bigg (\bar {A} _ {i} ^ {T} P _ {i} ^ {(k)} + P _ {i} ^ {(k)} \bar {A} _ {i} \bigg) \mathrm{d} \tau \\ + \int_ {t _ {0}} ^ {t _ {1}} 2 (\xi_ {i} ^ {T} \otimes u _ {i} ^ {T}) (I \otimes R _ {i}) \mathrm{vec} (K _ {i} ^ {(k + 1)}) \mathrm{d} \tau \\ + \int_ {t _ {0}} ^ {t _ {1}} 2 (\xi_ {i} ^ {T} \otimes \zeta_ {i} ^ {T}) \mathrm{vec} (\Delta_ {i 0} ^ {T} P _ {i} ^ {(k)}) \mathrm{d} \tau \\ - \int_ {t _ {0}} ^ {t _ {1}} 2 (\xi_ {i} ^ {T} \otimes \psi_ {i} ^ {T}) \mathrm{vec} \left(\Xi_ {i} ^ {(k)}\right) \mathrm{d} \tau \end{array}\tag{26}
$$

for any a, b and a sufficiently large number s, define the following matrix:

$$
\begin{array}{l} \delta_ {a} = [ \operatorname{vecv} (a (t _ {1})) - \operatorname{vecv} (a (t _ {0})), \dots , \\ \qquad \operatorname{vecv} (a (t _ {s})) - \operatorname{vecv} (a (t _ {s - 1})) ] ^ {T}, \\ \Gamma_ {a, b} = [ \int_ {t _ {0}} ^ {t _ {1}} a \otimes b d \tau , \int_ {t _ {1}} ^ {t _ {2}} a \otimes b d \tau , \dots , \int_ {t _ {s - 1}} ^ {t _ {s}} a \otimes b d \tau ] ^ {T}, \\ \text {where} t _ {0} <   t _ {1} <  , \ldots , <   t _ {s}. \end{array}
$$

According to (26), the following equation can be obtained

$$
\Psi_ {V I, i} \left[ \begin{array}{c} \operatorname{vec} (\bar {A} _ {i} ^ {T} P _ {i} ^ {(k)} + P _ {i} ^ {(k)} \bar {A} _ {i}) \\ \operatorname{vec} (K _ {i} ^ {(k + 1)}) \\ \operatorname{vec} \left(\Delta_ {i 0} ^ {T} P _ {i} ^ {(k)}\right) \\ \operatorname{vec} \left(\Xi_ {i} ^ {(k)}\right) \end{array} \right] = \Phi_ {V I, i} ^ {(k)}\tag{27}
$$

where

$$
\begin{array}{l} \Psi_ {V I, i} = [ \Gamma_ {\xi_ {i}, \xi_ {i}}, 2 \Gamma_ {\xi_ {i}, u _ {i}} (I \otimes R _ {i}), 2 \Gamma_ {\xi_ {i}, \zeta_ {i}}, - 2 \Gamma_ {\xi_ {i}, \psi_ {i}} ], \\ \Phi_ {V I, i} ^ {(k)} = \delta_ {\xi_ {i}} \operatorname{vecs} \left(P _ {i} ^ {(k)}\right). \end{array}
$$

The uniqueness of the solution to equation (27) is assured under certain rank conditions, as demonstrated in the following section. Due to limited space, we omit the proof of the Lemma 2, which aligns with the proof methodologies presented in [39], [40].

Lemma 2. For all $k \in \mathbb{Z}_+$ , if there exists a $s^* \in \mathbb{Z}_+$ such that for all $s > s^*$ ,

$$
\mathrm{rank} ([ \Gamma_ {\xi_ {i} \xi_ {i}}, \Gamma_ {\xi_ {i} u _ {i}}, \Gamma_ {\xi_ {i} \zeta_ {i}}, \Gamma_ {\xi_ {i}, \psi_ {i}} ]) = 3 0 + 5 p _ {i},\tag{28}
$$

then the matrix $\Psi_{VI,i}$ has full column rank for all $k\in \mathbb{Z}_{+}$ .

Remark 5. To satisfy the rank condition in (28), exploration noise is added to the input during the learning phase. Exploration noise is usually a sum of random noise, random sinusoidal signals, or sinusoidal signals of different frequencies.

2) Phase 2: Explore the Optimal CACC Policy.

Given the stabilizing CACC policy, one can initiate the PI learning process. Consider the Lyapunov function $V_{k}(\xi_{i}) = \xi_{i}^{T} P_{i}^{(k)} \xi_{i}$ , and the derivative of $V_{k}(\xi_{i})$ and model (22):

$$
\begin{array}{l} \dot {V} _ {k} (\xi_ {i}) = \dot {\xi} _ {i} ^ {T} P _ {i} ^ {(k)} \xi_ {i} + \xi_ {i} ^ {T} P _ {i} ^ {(k)} \dot {\xi} _ {i} \\ = \left(\bar {A} _ {i} ^ {(k)} \xi_ {i} + \bar {B} _ {i} \left(K _ {i} ^ {(k)} \xi_ {i} + u _ {i}\right) + \Delta_ {i 0} \zeta_ {i} \right. \\ \left. - \sum_ {j \in \mathcal {N} _ {i} ^ {+}} \Delta_ {i j} \xi_ {j} + \Upsilon_ {i}\right) ^ {T} P _ {i} ^ {(k)} \xi_ {i} \\ + \xi_ {i} ^ {T} P _ {i} ^ {(k)} \left(\bar {A} _ {i} ^ {(k)} \xi_ {i} + \bar {B} _ {i} \left(K _ {i} ^ {(k)} \xi_ {i} + u _ {i}\right) \right. \\ + \Delta_ {i 0} \zeta_ {i} - \sum_ {j \in \mathcal {N} _ {i} ^ {+}} \Delta_ {i j} \xi_ {j} + \Upsilon_ {i}) \\ = \xi_ {i} ^ {T} \left((\bar {A} _ {i} ^ {(k)}) ^ {T} P _ {i} ^ {(k)} + P _ {i} ^ {(k)} \bar {A} _ {i} ^ {(k)}\right) \xi_ {i} \\ + 2 (u _ {i} + K _ {i} ^ {(k)} \xi_ {i}) ^ {T} R _ {i} K _ {i} ^ {(k + 1)} \xi_ {i} + 2 \zeta_ {i} ^ {T} \Delta_ {i 0} ^ {T} P _ {i} ^ {(k)} \xi_ {i} \\ - 2 \sum_ {j \in \mathcal {N} _ {i} ^ {+}} \xi_ {j} ^ {T} \Delta_ {i j} ^ {T} P _ {i} ^ {(k)} \xi_ {i} + 2 \Upsilon_ {i} ^ {T} P _ {i} ^ {(k)} \xi_ {i}. \end{array} \tag {29}
$$

The equation (29) above can be rewritten by (16),

$$
\begin{array}{r l} & {\dot {V} _ {k} (\xi_ {i}) = - \xi_ {i} ^ {T} \bigg (Q _ {i} + (K _ {i} ^ {(k)}) ^ {T} R _ {i} K _ {i} ^ {(k)} \bigg) \xi_ {i}} \\ & {\qquad + 2 (u _ {i} + K _ {i} ^ {(k)} \xi_ {i}) ^ {T} R _ {i} K _ {i} ^ {(k + 1)} \xi_ {i} + 2 \zeta_ {i} ^ {T} \Delta_ {i 0} ^ {T} P _ {i} ^ {(k)} \xi_ {i}} \end{array}
$$

$$
- 2 \sum_ {j \in \mathcal {N} _ {i} ^ {+}} \xi_ {j} ^ {T} \Delta_ {i j} ^ {T} P _ {i} ^ {(k)} \xi_ {i} + 2 \Upsilon_ {i} ^ {T} P _ {i} ^ {(k)} \xi_ {i}.\tag{30}
$$

Next, by Kronecker product representation, we get

$$
\begin{array}{c} \xi_ {i} ^ {T} \bigg (Q _ {i} + (K _ {i} ^ {(k)}) ^ {T} R _ {i} K _ {i} ^ {(k)} \bigg) \xi_ {i} = (\xi_ {i} ^ {T} \otimes \xi_ {i}) \text {vec} \bigg (Q _ {i} \\ \qquad \qquad \qquad + (K _ {i} ^ {(k)}) ^ {T} R _ {i} K _ {i} ^ {(k)} \bigg), \end{array}
$$

$$
\begin{array}{l} (u _ {i} + K _ {i} ^ {(k)} \xi_ {i}) ^ {T} R _ {i} K _ {i} ^ {(k + 1)} \xi_ {i} \\ \qquad = (\xi_ {i} ^ {T} \otimes u _ {i} ^ {T}) (I \otimes R _ {i}) \mathrm{vec} (K _ {i} ^ {(k + 1)}) \\ \qquad + (\xi_ {i} ^ {T} \otimes \xi_ {i} ^ {T}) (I \otimes (K _ {i} ^ {(k)}) ^ {T} R _ {i}) \\ \qquad \times \mathrm{vec} (K _ {i} ^ {(k + 1)}), \end{array}
$$

letting $\Upsilon_{i} = 0$ , we obtain

$$
\Psi_ {P I, i} ^ {(k)} \left[ \begin{array}{c} \operatorname{vecs} (P _ {i} ^ {(k)}) \\ \operatorname{vec} (K _ {i} ^ {(k + 1)}) \\ \operatorname{vec} \left(\Delta_ {i 0} ^ {T} P _ {i} ^ {(k)}\right) \\ \operatorname{vec} \left(\Xi_ {i} ^ {(k)}\right) \end{array} \right] = \Phi_ {P I, i} ^ {(k)}\tag{31}
$$

where

$$
\begin{array}{l} \Psi_ {P I, i} ^ {(k)} = [ \delta_ {\xi_ {i}}, - 2 \Gamma_ {\xi_ {i}, \xi_ {i}} (I \otimes (K _ {i} ^ {(k)}) ^ {T} R _ {i}) - 2 \Gamma_ {\xi_ {i}, u _ {i}} (I \otimes R _ {i}), \\ \qquad - 2 \Gamma_ {\xi_ {i}, \zeta_ {i}}, 2 \Gamma_ {\xi_ {i}, \psi_ {i}} ], \\ \Phi_ {P I, i} ^ {(k)} = -   \Gamma_ {\xi_ {i}, \xi_ {i}} \mathrm{vec} \left(Q _ {i} + (K _ {i} ^ {(k)}) ^ {T} R _ {i} K _ {i} ^ {(k)}\right). \end{array}
$$

<div class="mineru-algorithm" style="white-space: pre-wrap; font-family:monospace;">
Algorithm 2 Data-Driven HI Algorithm
1:  $i \leftarrow 1$ .
2: Choose  $\varepsilon_{i} &gt; 0$ ,  $P_{i}^{(0)} = P_{i}^{T(0)} \succ 0$ , and  $\hat{Q}_{i} = \hat{Q}_{i}^{T} \succ Q_{i}$ 
3: repeat
4: Use  $u_{i} = -K_{i}^{(0)}\xi_{i} + w_{i}$  on  $[t_{0} - t_{s}]$  with  $w_{i}$  an exploration noise.
5: repeat
6: Solve  $\bar{A}_{i}^{T}P_{i}^{(k)} + P_{i}^{(k)}\bar{A}_{i}$  and  $K_{i}^{(k)}$  from (27).
7:  $\bar{P}_{i}^{(k+1)} \leftarrow P_{i}^{(k)} + \epsilon_{k}(\bar{A}_{i}^{T}P_{i}^{(k)} + P_{i}^{(k)}\bar{A}_{i} + \hat{Q}_{i} - (K_{i}^{(k+1)})^{T}R_{i}K_{i}^{(k+1)})$ 
8: if  $\bar{P}_{i}^{(k+1)} \notin B_{r}$  then  $P_{i}^{(k+1)} \leftarrow P_{i}^{(0)}, r \leftarrow r + 1$ .
9: else  $P_{i}^{(k+1)} \leftarrow \bar{P}_{i}^{(k+1)}$  end if
10:  $k \leftarrow k + 1$ 
11: until  $(P_{i}^{(k)} - P_{i}^{(k-1)}) / \epsilon_{k} \prec \hat{Q}_{i}$ ,
12: repeat
13: Solve  $P_{i}^{(k)}$  and  $K_{i}^{(k+1)}$  from (31).  $k \leftarrow k + 1$ .
14: until  $||P_{i}^{(k)} - P_{i}^{(k-1)}|| &lt; \hat{\varepsilon}_{i}$ 
15:  $k \leftarrow k^{*}, j \leftarrow 1$ 
16: The learned controlled is (7), (21) and
 $u_{i} = K_{i}^{(k+1)}\xi_{i}$ 
17:  $i \leftarrow i + 1$ 
18: until  $i = N + 1$
</div>

TABLE I  
COMPARISON BETWEEN ALGORITHMS

<table><tr><td></td><td>HI</td><td>VI</td><td>PI</td></tr><tr><td>Stabilizing  $K_{i}^{0}$ </td><td>Not required</td><td>Not required</td><td>Required</td></tr><tr><td>Convergence rate</td><td>Quadratic</td><td>Sub-linear</td><td>Quadratic</td></tr></table>

Theorem 3. If (28) is satisfied, sequences $\left\{P_{i}^{(k)}\right\}_{k=0}^{\infty}$ and $\{K_{i}^{(k)}\}_{k=1}^{\infty}$ computed by Algorithm 2 converge to $P_{i}^{*}$ and $K_{i}^{*}$ for $i=1,2,\cdots,N$ , respectively.

Proof: If the rank condition (28) is satisfied, it can be determined that (27) has a unique solution. Then the pair $\left((\bar{A}_{i}^{T}P_{i}^{(k)}+P_{i}^{(k)}\bar{A}_{i}),K_{i}^{(k+1)}\right)$ satisfies (23) with $K_{i}^{(k+1)}=R_{i}^{-1}\bar{B}_{i}^{T}P_{i}^{(k)}$ . Therefore, both $\bar{P}_{i}^{(k+1)}$ and $P_{i}^{(k+1)}$ solved from steps 5-11 at each iteration are equivalent to those in steps 4-9 of Algorithm 1. One can also observe that the pair $\left(P_{i}^{(k)},K_{i}^{(k+1)}\right)$ derived from steps 13-16 is equivalent to the pair produced in steps 9-13 of Algorithm 1 [40]. Since the convergence of $\left\{P_{i}^{(k)}\right\}_{k=0}^{\infty}$ and $\left\{K_{i}^{(k)}\right\}_{k=1}^{\infty}$ obtained from Algorithm 1 has been proved in Theorem 1, the convergence of these sequences under the data-driven Algorithm 2 is also ensured. The proof is thus completed.

Before closing this section, the differences between the HI, VI and PI algorithms is summarized in Table I.

## IV. ILLUSTRATIVE EXAMPLES

To validate our theoretical results, we provide an example involving a platoon of four vehicles.

The initial condition of the exosystem is set by $v = [0 \quad 1]^{T}$ . The values of $\tau_{i}$ and $\tilde{d}_{i}$ are illustrated in Tab. II. The topology of the vehicle platoon is depicted in Figure 2, where each vehicle attempts to maintain a safe distance of 30 meters at steady state. From t = 0s to t = 1s, we allow the exostate estimation to converge. We collect data from t = 1s to t = 2s for HI learning. The convergence of $P_{i}$ for each vehicle is shown in Figure 3. By applying the data-driven HI Algorithm 2, we determine that the control policy is admissible when satisfying the condition in step 12. To be more specific, an admissible control policy is achieved at the iteration $k_{1} = 376$ , $k_{2} = 349$ , $k_{3} = 323$ , and $k_{4} = 288$ , for i = 1, 2, 3, 4, respectively.

The comparison of the learned value $P_{i}^{(k)}$ with its optimal value $P_{i}^{(*)}$ of each vehicle is shown in the Figure 3 which indicates that all vehicles can learn towards the corresponding optimal control values by iteration. According to the definition of quadratic convergence, we introduce a parameter $r_{k} = \frac{|P_{i}^{k+1} - P_{i}^{*}|}{|P_{i}^{k} - P_{i}^{*}|^{2}}$ , from Figure 4, we see that $r_{k}$ converges to constant 4.3 after 4 iterations, which shows that the proposed method retains the quadratic convergence rate. Figures 5 and 6 demonstrate that the HI algorithm maintains robust performance across different initial conditions. The solid line represents the actual trajectory of the connected autonomous vehicles, whereas the dashed line corresponds to their reference trajectory. The learned control policy is applied after t = 2s and the vehicle trajectories can be checked in Figure 5, showing that all vehicles can follow their references after t = 12s, which demonstrates the efficiency of the proposed data-driven control approach. As shown in Figures 7 and 8, under identical test conditions, the data-driven HI method achieves the target speed of 15 m/s within 20 seconds, whereas the method in [41] requires nearly 30 seconds to reach the same velocity. The integral of squared error (ISE) is introduced as a performance metric for quantifying tracking accuracy, where a smaller ISE indicates better system performance in minimizing deviations between actual and desired speeds. Notably, in dynamic scenarios, the proposed data-driven HI-based CACC demonstrates a 4 times reduction in ISE compared to the data-driven CACC method reported in reference [41]. This comparative analysis demonstrates the faster convergence speed and enhanced control efficiency of our data-driven approach in CACC implementation.

![](images/e452ef28dbb57bc371b7cf38dc6e42ef840af321da182987b5c2be8dc5c0a750.jpg)  
Fig. 2. The structure of a vehicular platoon.

To demonstrate the effectiveness and safety of our algorithm, we will incorporate two scenarios with CACC-equipped vehicles.

Firstly, in Figure 9, we observe that the vehicles track their desired speeds during cooperative driving operations. However, when traffic congestion occurred at the 40-second mark, the speed exhibited a significant decrease, demonstrating that the vehicles executed braking maneuvers to adapt to changing traffic conditions. As the congestion gradually dissipated, indicating that the vehicles steadily resumed their original speed.

Secondly, we test the performance of the proposed control method via cut-in scenarios. In Figure 10 verifies the scenario when the two vehicles cut in. At 50 seconds, cut-in vehicle 1 and cut-in vehicle 2 simultaneously merged into the vehicle platoon. To preserve formation integrity, vehicle 3 and vehicle 4 proactively decelerated, ultimately enabling all six vehicles to maintain a collision-free coordinated motion pattern. This validates the effectiveness of the data-driven control method proposed in this paper and demonstrates that it can mitigate the interference caused by the cutting-in vehicle.

TABLE II  
SYSTEM PARAMETERS

<table><tr><td>Parameter</td><td>Value</td><td>Parameter</td><td>Value</td><td>Parameter</td><td>Value</td></tr><tr><td> $\tau_1[s]$ </td><td>0.1</td><td> $\tau_2[s]$ </td><td>0.11</td><td> $\tau_3[s]$ </td><td>0.12</td></tr><tr><td> $\tau_4[s]$ </td><td>0.13</td><td> $\tilde{d}_1[m/s^3]$ </td><td>3</td><td> $\tilde{d}_2[m/s^3]$ </td><td>3.1</td></tr><tr><td> $\tilde{d}_3[m/s^3]$ </td><td>3.2</td><td> $\tilde{d}_4[m/s^3]$ </td><td>3.3</td><td></td><td></td></tr></table>

![](images/6d77bddc44a0dfdf3488585f07d9f07de0c543dcf861b343ad22756621525885.jpg)

![](images/c031f9cdb096b67788f2a6d4676f5eb7aef68cbdfd40cfaae4b4821df8d16199.jpg)

![](images/ac2fde72c9e48ac4f0f93ab50d60f61d3add43ab3eb065d6a238eafcf28f231c.jpg)

![](images/9096fa813b1d8016ff28a8e2d6206217919d662167880ec2b1f80f15aee09384.jpg)  
Fig. 3. $||P_i^{(k)} - P_i^{(*)}||$ of each follower $(i = 1,2,3,4)$ using HI Algorithm.

![](images/e61ae0dae15b3bd4b1a4d89e8957406b7c8312bf058a6dfc8ee2bc28b6918782.jpg)  
Fig. 4. The evolution of convergence rate constant of the vehicle #4.

![](images/8fee3db6cdb99725c62833c4ef4cab017e030265ee674c866589ec9c173346c7.jpg)  
Fig. 5. Trajectories of CAVs and their references with the initial condition $[p_{1}(0), p_{2}(0), p_{3}(0), p_{4}(0)]=[83, 35, 10, 7]$ .

![](images/4afbab7aa6920c5a2250650f78dbc3bd5c1d8bd37bdce0464f749a287cf2ca94.jpg)  
Fig. 6. Trajectories of CAVs and their references with the initial condition $[p_{1}(0), p_{2}(0), p_{3}(0), p_{4}(0)]=[83, 57, 18, 7]$ .

![](images/43f1f5e3fd1a338f19861544b7d1d38344bb0a703445259c55d46b0d4a609b1b.jpg)  
Fig. 7. Velocities of vehicles controlled by Algorithm 2

![](images/2433ea1e2ea579c11e0e71c60334f21e07334e32aac6a6ee06326afc2c5c17e2.jpg)  
Fig. 8. Velocities of vehicles controlled by data-driven CACC method in [41].

![](images/5bd7cfc58e5d0bac8dc07ac5f1159f34196fc14d42ff99bbfb34487d22f8dfa2.jpg)  
Fig. 9. Trajectories of CAVs during congestion.

![](images/b2653979f61f48967788375ee15289ebc3ddf234aba941aaa4c038e810b511ce.jpg)  
Fig. 10. Trajectory of CAVs in two-vehicle cut-in scenarios.

## V. CONCLUSION

This paper proposes a distributed data-driven control method, hybrid iteration, using internal model principle and reinforcement learning techniques to address cooperative adaptive cruise control (CACC) problems for connected and autonomous vehicles, in the absence of precise knowledge of system dynamics, the desired controller is obtained from data through reinforcement learning. Moreover, through the verification under specific scenarios, the high-efficiency and safety performance of the HI method in the CACC have been shown. The limitation of the study is that it involves vehicle simulation and has not been applied in practice. Future work includes the development of advanced CACC methods, the use of hybrid driving and autonomous driving with non-linear models, and their application in real-world environments.

## REFERENCES

[1] T. Yoshioka and K. Suzuki, “Traffic control on main line of highway using cooperative adaptive cruise control (CACC) for the purpose of merge assistance-influence of traffic flow management on stability and response of platoon,” International Journal of Automotive Engineering, vol. 15, no. 3, pp. 116–124, 2024.

[2] J. Chen, A. Behal, Z. Li, and C. Li, “Active battery cell balancing by real-time model predictive control for extending electric vehicle driving range,” IEEE Transactions on Automation Science and Engineering, 2023.

[3] A.-T. Nguyen, J. Rath, T.-M. Guerra, R. Palhares, and H. Zhang, "Robust set-invariance based fuzzy output tracking control for vehicle autonomous driving under uncertain lateral forces and steering constraints," IEEE Transactions on Intelligent Transportation Systems, vol. 22, no. 9, pp. 5849–5860, 2020.

[4] C. Huang, H. Huang, P. Hang, H. Gao, J. Wu, Z. Huang, and C. Lv, "Personalized trajectory planning and control of lane-change maneuvers for autonomous driving," IEEE Transactions on Vehicular Technology, vol. 70, no. 6, pp. 5511–5523, 2021.

[5] Y. Zhou, S. Ahn, M. Chitturi, and D. A. Noyce, “Rolling horizon stochastic optimal control strategy for ACC and CACC under uncertainty,” Transportation Research Part C: Emerging Technologies, vol. 83, pp. 61–76, 2017.

[6] F. Xu and T. Shen, "Look-ahead prediction-based real-time optimal energy management for connected hevs," IEEE Transactions on Vehicular Technology, vol. 69, no. 3, pp. 2537-2551, 2020.

[7] C. Hu, Z. Wang, X. Bu, J. Zhao, J. Na, and H. Gao, “Optimal tracking control for autonomous vehicle with prescribed performance via adaptive dynamic programming,” IEEE Transactions on Intelligent Transportation Systems, 2024.

[8] C. Wang, S. Gong, A. Zhou, T. Li, and S. Peeta, “Cooperative adaptive cruise control for connected autonomous vehicles by factoring communication-related constraints,” Transportation Research Procedia, vol. 38, pp. 242–262, 2019.

[9] C. Hu, Y. Wang, J. Na, G. Guo, Z. Zuo, H. Gao, and X. Zhang, “Optimal adaptive cruise control in mixed traffic with communication latency and driver reaction,” IEEE Transactions on Intelligent Transportation Systems, 2024.

[10] C. Chakraborty and J. J. Rodrigues, “A comprehensive review on device-to-device communication paradigm: trends, challenges and applications,” Wireless Personal Communications, vol. 114, no. 1, pp. 185–207, 2020.

[11] H. Song, R. Srinivasan, T. Sookoor, and S. Jeschke, Smart cities: foundations, principles, and applications. John Wiley & Sons, 2017.

[12] V. Milanés, S. E. Shladover, J. Spring, C. Nowakowski, H. Kawazoe, and M. Nakamura, “Cooperative adaptive cruise control in real traffic situations,” IEEE Transactions on Intelligent Transportation Systems, vol. 15, no. 1, pp. 296–305, 2013.

[13] K. C. Dey, L. Yan, X. Wang, Y. Wang, H. Shen, M. Chowdhury, L. Yu, C. Qiu, and V. Soundararaj, “A review of communication, driver characteristics, and controls aspects of cooperative adaptive cruise control (CACC),” IEEE Transactions on Intelligent Transportation Systems, vol. 17, no. 2, pp. 491–509, 2015.

[14] Q. Liu, C. Gao, H. Wang, Y. Cai, L. Chen, and C. Lv, “Learning from trajectories: How heterogeneous cacc platoons affect the traffic flow in highway merging area,” IEEE Transactions on Vehicular Technology, 2024.

[15] C. Bonnet and H. Fritz, “Fuel consumption reduction in a platoon: Experimental results with two electronically coupled trucks at close spacing,” tech. rep., SAE technical paper, 2000.

[16] F. Browand, J. McArthur, and C. Radovich, “Fuel saving achieved in the field test of two tandem trucks,” 2004.

[17] H. Sun, B. Li, H. Zhang, L. Dai, G. Fedele, W. Zhuang, and B. Chen, "Ecological electric vehicle platooning: An adaptive tube-based distributed model predictive control approach," IEEE Transactions on Transportation Electrification, 2024.

[18] G. Guo and W. Yue, “Sampled-data cooperative adaptive cruise control of vehicles with sensor failures,” IEEE Transactions on Intelligent Transportation Systems, vol. 15, no. 6, pp. 2404–2418, 2014.

[19] R. Liu, Y. Ren, H. Yu, Z. Li, and H. Jiang, “Connected and automated vehicle platoon maintenance under communication failures,” Vehicular Communications, vol. 35, p. 100467, 2022.

[20] Y. Zhang, Y. Bai, M. Wang, and J. Hu, “Cooperative adaptive cruise control with robustness against communication delay: An approach in the space domain,” IEEE Transactions on Intelligent Transportation Systems, vol. 22, no. 9, pp. 5496–5507, 2020.

[21] J. Liu, Z. Wang, P. Hang, and J. Sun, “Delay-aware multi-agent reinforcement learning for cooperative adaptive cruise control with model-based stability enhancement,” arXiv preprint arXiv:2404.15696, 2024.

[22] W. Gao, J. Gao, K. Ozbay, and Z. P. Jiang, “Reinforcement-learning-based cooperative adaptive cruise control of buses in the lincoln tunnel corridor with time-varying topology,” IEEE Transactions on Intelligent Transportation Systems, vol. 20, no. 10, pp. 3796–3805, 2019.

[23] W. Gao, C. Deng, Y. Jiang, and Z. P. Jiang, “Resilient reinforcement learning and robust output regulation under denial-of-service attacks,” Automatica, vol. 142, p. 110366, 2022.

[24] U. Ahmad, M. Han, A. Jolfaei, S. Jabbar, M. Ibrar, A. Erbad, H. H. Song, and Y. Alkhrijah, “A comprehensive survey and tutorial on smart vehicles: Emerging technologies, security issues, and solutions using machine learning,” IEEE Transactions on Intelligent Transportation Systems, 2024.

[25] R. Wang, Z. Zhuang, H. Tao, W. Paszke, and V. Stojanovic, “Q-learning based fault estimation and fault tolerant iterative learning control for mimo systems,” ISA transactions, vol. 142, pp. 123–135, 2023.

[26] C. Desjardins and B. Chaib-Draa, “Cooperative adaptive cruise control: A reinforcement learning approach,” IEEE Transactions on intelligent transportation systems, vol. 12, no. 4, pp. 1248–1260, 2011.

[27] M. Wang, “Infrastructure assisted adaptive driving to stabilise heterogeneous vehicle strings,” Transportation Research Part C: Emerging Technologies, vol. 91, pp. 276–295, 2018.

[28] J. Ge and G. Orosz, “Dynamics of connected vehicle systems with delayed acceleration feedback,” Transportation Research Part C: Emerging Technologies, vol. 46, pp. 46–64, 2014.

[29] M. Wang, W. Daamen, S. P. Hoogendoorn, and B. van Arem, “Rolling horizon control framework for driver assistance systems. part ii: Cooperative sensing and cooperative control,” Transportation research part C: emerging technologies, vol. 40, pp. 290–311, 2014.

[30] W. Gao, Z. P. Jiang, F. L. Lewis, and Y. Wang, “Cooperative optimal output regulation of multi-agent systems using adaptive dynamic programming,” in 2017 American Control Conference (ACC), pp. 2674–2679, IEEE, 2017.

[31] W. Gao, M. Mynuddin, D. C. Wunsch, and Z. P. Jiang, “Reinforcement learning-based cooperative optimal output regulation via distributed adaptive internal model,” IEEE Transactions on Neural Networks and Learning Systems, vol. 33, no. 10, pp. 5229–5240, 2021.

[32] R. Zhang and J. Huang, “Event-triggered output-based distributed observer over jointly connected networks and its application to the cooperative output regulation problem,” Neurocomputing, p. 128072, 2024.

[33] T. Enderes, J. Gabriel, and J. Deutscher, “Cooperative output regulation for networks of hyperbolic systems using adaptive cooperative observers,” Automatica, vol. 162, p. 111506, 2024.

[34] W. Gao, Y. Liu, A. Odekunle, Y. Yu, and P. Lu, “Adaptive dynamic programming and cooperative output regulation of discrete-time multiagent systems,” International Journal of Control, Automation and Systems, vol. 16, no. 5, pp. 2273–2281, 2018.

[35] J. Huang, Nonlinear Output Regulation: Theory and Applications. Philadelphia, PA: SIAM, 2004.

[36] D. Kleinman, “On an iterative technique for riccati equation computations,” IEEE Transactions on Automatic Control, vol. 13, no. 1, pp. 114–115, 1968.

[37] T. Bian and Z. P. Jiang, “Value iteration and adaptive dynamic programming for data-driven adaptive optimal control design,” Automatica, vol. 71, pp. 348–360, 2016.

[38] D. Vrabie and F. L. Lewis, “Generalized policy iteration for continuous-time systems,” in 2009 International Joint Conference on Neural Networks, pp. 3224–3231, IEEE, 2009.

[39] W. Gao and Z. P. Jiang, “Adaptive dynamic programming and adaptive optimal output regulation of linear systems,” IEEE Transactions on Automatic Control, vol. 61, no. 12, pp. 4164–4169, 2016.

[40] Y. Jiang and Z. P. Jiang, “Computational adaptive optimal control for continuous-time linear systems with completely unknown dynamics,” Automatica, vol. 48, no. 10, pp. 2699–2704, 2012.

[41] J. Lan, “Data-driven cooperative adaptive cruise control for unknown nonlinear vehicle platoons,” IET Intelligent Transport Systems, vol. 18, no. 11, pp. 2114–2123, 2024.

![](images/23e9564a2506ca2e02e59188b7f50b1d128bcb17c77bf9911927290e6265d0f4.jpg)

Xue Liang received a B.S. in Mathematics and Applied Mathematics from Suihua University in 2021, and an M.S. in Mathematics from Heilongjiang University (Harbin, China) in 2024. She is currently pursuing her Ph.D. at the State Key Laboratory of Synthetical Automation for Process Industries, Northeastern University. Her research interests include data-driven methods, optimal control and reinforcement learning.

![](images/74fb94655bfc34fb7b1027d806f5e8d6de661ac3c1dc5c7973593ed4e6c5d2ab.jpg)

Weinan Gao (Senior Member, IEEE) received the B.Sc. degree in automation and the M.Sc. degree in control theory and control engineering from North eastern University, Shenyang, China, in 2011 and 2013, respectively, and the Ph.D. degree in electrical engineering from New York University, Brooklyn, NY, USA, in 2017. He is a Professor with the State Key Laboratory of Synthetical Automation for Process Industries at Northeastern University, Shenyang, China. Previously, he was an Assistant Professor of Mechanical and Civil Engineering at

Florida Institute of Technology, Melbourne, FL, USA, an Assistant Professor of Electrical and Computer Engineering at Georgia Southern University, Statesboro, GA, USA, and a Visiting Professor of Mitsubishi Electric Research Laboratory (MERL), Cambridge, MA, USA. His research interests include reinforcement learning, adaptive dynamic programming, optimal control, co-operative adaptive cruise control, intelligent transportation systems, sampled data control systems, and output regulation theory. He was a recipient of the Best Paper Award in IEEE International Conference on Real-Time Computing and Robotics (RCAR) in 2018 and the David Goodman Research Award at New York University in 2019. He was a recipient of the U.S.-NSF Engineering Research Initiation Award. He is a member of editorial board of Neural Computing and Applications and a Technical Committee Member in IEEE Control Systems Society on Nonlinear Systems and Control and in IFAC TC 1.2 Adaptive and Learning Systems. He was an Associate Editor/a Guest Editor of IEEE/CAA Journal OF Automatica Sinica, IEEE Trans. on Neural Network and Learning systems, IEEE Trans. on Circuits and systems II: express Briefs, Neurocomputing, and Control Engineering Practice.

![](images/e18aa8c34cdf45cc35f2d01639adcbba1e90bf3e161186b43d0a3cff788766b9.jpg)

Chuan Hu is now a tenure-track Associate Professor at the School of Mechanical Engineering, Shanghai Jiao Tong University, Shanghai, China, starting from July, 2022. Before that, he was an Assistant Professor at the Department of Mechanical Engineering, University of Alaska Fairbanks, Fairbanks, AK, USA from June, 2020 to June, 2022. He was a Postdoctoral Fellow at Department of Mechanical Engineering, University of Texas at Austin, Austin, USA, from August, 2018 to June, 2020, and a Postdoctoral Fellow in the Department of Systems

Design Engineering, University of Waterloo, Waterloo, Canada from July, 2017 to July, 2018. He received the Ph.D. degree in Mechanical Engineering, McMaster University, Hamilton, Canada in 2017, the M.S. degree in Vehicle Operation Engineering from the China Academy of Railway Sciences, Beijing, in 2013, and the B.S. degree in Automotive Engineering from Tsinghua University, Beijing, China, in 2010. His research interest includes the perception, decision-making, path planning, and motion control of Intelligent and Connected Vehicles (ICVs), Autonomous Driving (AD), eco-driving, human-machine trust and cooperation, shared control, and machine-learning applications in ICVs. He has published more than 60 papers in these research areas. He is currently an Associate Editor for several leading IEEE Trans. journals in this filed: IEEE Trans. on Neural Networks and Learning Systems, IEEE Trans. on Vehicular Technology, IEEE Trans. on Transportation Electrification, IEEE Trans. on Intelligent Transportation Systems and IEEE Trans. on Intelligent Vehicles.

![](images/0f7e123c1d21f940f1aaf4ac8a5c8e93d44cc175f65197174c79ea4a2f35cb55.jpg)

Tianyou Chai (Life Fellow, IEEE) received the Ph.D. degree in control theory and engineering from Northeastern University, Shenyang, China, in 1985. Since 1985, he has been with the Research Center of Automation, Northeastern University, where he became a Professor in 1988 and a Chair Professor in 2004. He is the Founder and the Director of the Center of Automation, which became a National Engineering and Technology Research Center in 1997. His current research interests include adaptive control, intelligent decoupling control, inte grated

plant control and systems, integrated automation of complex industrial processes, and the development of control technologies with applications to various industrial processes. Dr. Chai was the recipient of four prestigious awards of the National Science and Technology Progress and National Technological Innovation, the 2007 Industry Award for Excellence in Transitional Control Research from IEEE Multiple-Conference on Systems and Control, and the 2017 Wook Hyun Kwon Education Award from the Asian Control Association for his contributions. He is also the recipient of three prestigious awards of National Science and Technology Progress, the 2002 Technological Science Progress Award from the Ho Leung Ho Lee Foundation, the 2007 Industry Award for Excellence in Transitional Control Research from IEEE Control Systems Society, and the 2010 Yang Jia-Chi Science and Technology Award from the Chinese Association of Automation. His paper titled hybrid intelligent control for optimal operation of shaft furnace roasting process was selected as one of three best papers for the Control Engineering Practice Paper Prize from 2011 to 2013. He is a Member of the Chinese Academy of Engineering and a Fellow of the International Federation of Automatic Control. He is a Member of the Chinese Academy of Engineering and an Academician of the International Eurasian Academy of Sciences. He is a Distinguished Visiting Fellow of the Royal Academy of Engineering (U.K.) and an Invitational Fellow of the Japan Society for the Promotion of Science.