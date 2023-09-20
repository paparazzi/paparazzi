# Efficiently solve Optimal Control Allocation with Active-Set

Solves
$$
\min_u\ || Bu - \nu ||_{W_\nu}^2 + \gamma^2 || u - u_d ||_{W_u}^2\\
s.t.\quad \underbar{u}_i \leq u_i \leq \overline{u}_i \quad \forall i
$$

where $W_\nu$, $W_u$ diagonal and positive definite. $u,\nu$ vectors and $\gamma>0$ scalar and small. $\underbar{u}_i,\overline{u}_i$ bound the decision variables $u_i$.

## Features
- Provides setup routines (`setupWLS.h`) to turn the weighted least-squares control allocation problem above into a standard least-squares form $||Au-b||$, s.t. $\underbar{u}\leq u \leq\overline{u}$.
- Exploits sparsity from diagonal weighing matrices.
- `QR` and `CHOL` choices use efficiently updating QR and Cholesky factorisations.

## Limitations
- Does not solve general active-set problems, since highly optimised for control allocation where diagonal matrix $W_u$ gives rise to sparsity.
- `CG` no reliable. Only available when compiling with with `-DAS_INCLUDE_CG`
- `CHOL` less numerically stable with single floats. Choose low `cond_bound` and see `solveActiveSet.h`
