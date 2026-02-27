## Math

### PPO math
#### Probability Basics

- [Random variables](#random-variable)
- Expectation E[x]
- Gaussian distribution
- Log-probability

Why?: PPO optimizes expected reward and uses log probabilities of actions.

#### Basic Calculus

- Derivatives
- Chain rule
- Gradient

Why?: PPO is just gradient ascent on expected return.

#### Linear Algebra
- Vectors
- [Dot products](#dot-product)
- [Matrix multiplication](#matrix-multiplication)

Why?
Neural networks = matrix operations.

---

## Reinforcement Learning Foundations

### Markov Decision Process (MDP)
- State $$s$$
- Action $$a$$
- Reward $$r$$
- Transition probability
- Policy $$\pi(a|s)$$

### Value function
- State value: $$V^\pi(s)$$
- Action value: $$Q^\pi(s,a)$$
- Advantage function: $$A(s,a) = Q(s,a) - V(s)$$

why?
**PPO** heavily relies on the **Advantage**

### Policy Gradient (CRITICAL)

#### REINFORCE algorithm

$$\nabla J(\theta) = \mathbb{E} [ \nabla \log \pi_\theta(a|s) R ]$$


---

### Actor-Critic Architecture

PPO is an actor-critic method.

You must understand:

- Actor = policy network
- Critic = value network
- Why we need critic (variance reduction)

---

### Random variable
A function that assigns a numerical value to the outcome of a random experiment.

It converts uncertainty → numbers.


---

## Linear algebra

### Dot product
The **dot product** take two vectors of the same size and produces a **signal number**

$$\mathbf{a} =
\begin{bmatrix}
a_1 \\
a_2 \\
a_3
\end{bmatrix}
\quad
\mathbf{b} =
\begin{bmatrix}
b_1 \\
b_2 \\
b_3
\end{bmatrix}$$


$$ a \cdot b
=a_1 b_1 + a_2 b_2 + a_3 b_3$$

### Matrix multiplication

Applies a linear transformation.

$$A =
\begin{bmatrix}
1 & 2 \\
3 & 4
\end{bmatrix}$$

$$B =
\begin{bmatrix}
5 & 6 \\
7 & 8
\end{bmatrix}$$

result

$$AB =
\begin{bmatrix}
19 & 22 \\
43 & 50
\end{bmatrix}$$

first element for example

$$1×5 + 2×7 = 19$$