# References

The mechanics in this repository is standard rigid-body theory. Nothing here is novel; the
works below are where the results are derived in full. They are cited by author and title
only — no text is reproduced in this repository.

## Rigid-body kinematics and dynamics

- H. Goldstein, C. Poole and J. Safko, *Classical Mechanics*, 3rd ed., Addison-Wesley.
  Orientation, Euler angles, the inertia tensor and Euler's equations of motion.
- J. L. Meriam and L. G. Kraige, *Engineering Mechanics: Dynamics*, Wiley.
  Mass moments and products of inertia, the parallel-axis theorem, three-dimensional
  rigid-body kinetics, work and energy.
- O. Rodrigues (1840), the rotation formula that carries his name — the axis-and-angle
  form used in `rbsim.math.rotAxisAngle`.
- M. D. Shuster, "A Survey of Attitude Representations", *Journal of the Astronautical
  Sciences*, 41(4), 1993. Why a rotation matrix is carried as the state and Euler angles
  are produced only as output.

## Manipulators

- J. J. Craig, *Introduction to Robotics: Mechanics and Control*, Pearson.
  Frame assignment, composition of link transforms, velocity and acceleration propagation
  along a chain, and the Newton–Euler formulation of manipulator dynamics.
- J. Y. S. Luh, M. W. Walker and R. P. C. Paul, "On-line Computational Scheme for
  Mechanical Manipulators", *ASME Journal of Dynamic Systems, Measurement, and Control*,
  102(2), 1980, pp. 69–76. The recursive outward–inward algorithm implemented in
  `rbsim.robot.newtonEuler`.
- R. Featherstone, *Rigid Body Dynamics Algorithms*, Springer, 2008.
  Background for assembling the joint-space inertia matrix from an inverse-dynamics pass.

## Linkages

- F. Freudenstein, "An Analytical Approach to the Design of Four-Link Mechanisms",
  *Transactions of the ASME*, 76, 1954. The displacement equation solved in
  `rbsim.fourbar.position`.
- J. J. Uicker, G. R. Pennock and J. E. Shigley, *Theory of Machines and Mechanisms*,
  Oxford University Press. Loop-closure equations, branches and transmission angle.
- R. L. Norton, *Design of Machinery*, McGraw-Hill. Grashof's criterion and the four-bar
  classification reproduced in `rbsim.fourbar.grashof`.

## Numerical integration

- E. Hairer, S. P. Nørsett and G. Wanner, *Solving Ordinary Differential Equations I:
  Nonstiff Problems*, Springer. The classical fourth-order Runge–Kutta rule and its order
  of accuracy, which `test_rigid_body_dynamics` measures.
- L. F. Shampine and M. W. Reichelt, "The MATLAB ODE Suite", *SIAM Journal on Scientific
  Computing*, 18(1), 1997. The behaviour of `ode45`, used here only as an independent
  reference solution.
