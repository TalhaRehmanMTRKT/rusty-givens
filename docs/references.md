# References

## Core Textbooks

- Abur, A. & Expósito, A. G. (2004). *Power System State Estimation: Theory and Implementation*. Taylor & Francis. — The primary reference for Chapters 2–3 (WLS, Normal Equations, QR/Givens, Peters-Wilkinson, Equality-Constrained, Fast Decoupled, DC estimation).

- Monticelli, A. (1999). *State Estimation in Electric Power Systems: A Generalized Approach*. Springer. — Reference for observability analysis (§4.6), decoupled P-θ / Q-V models, and zero-injection handling.

## Redundancy and Bad Data Detection

- Handschin, E. & Bongers, C. (1990). *Über den Einfluß der Meßtopologie bei der Zustandsestimation*. — Sensitivity matrix method for measurement redundancy classification (w_ii detection sensitivity, k_ik correlation coefficients) and the M₀/M₁/M₂ Messwerttopologie framework.

## Frameworks and Implementations

- Cosovic, M. et al. (2025). *JuliaGrid: An Open-Source Julia-Based Framework for Power System State Estimation*. arXiv:2502.18229v2. — [JuliaGrid documentation](https://mcosovic.github.io/JuliaGrid.jl/stable/)

- Thurner, L. et al. (2018). *pandapower — An Open-Source Python Tool for Convenient Modeling, Analysis, and Optimization of Electric Power Systems*. IEEE Transactions on Power Systems, 33(6), 6510–6521. — [pandapower documentation](https://pandapower.readthedocs.io/)

## Network Data

- Bukhsh, W. A. & McKinnon, K. I. M. (2013). *Network data of real transmission networks*. University of Edinburgh. — Source for the Great Britain transmission network case study.

## Linear Algebra

- `faer` — High-performance dense and sparse linear algebra library for Rust. Used for Cholesky, LU, and QR factorizations. [docs.rs/faer](https://docs.rs/faer/)

- `sprs` — Sparse matrix library for Rust (CSC/CSR). Used for Jacobian and gain matrix storage. [docs.rs/sprs](https://docs.rs/sprs/)
