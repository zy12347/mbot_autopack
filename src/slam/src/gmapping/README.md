# Particle Filtering 

The Rao-Blackwellized particle filter for SLAM makes use of the following factorization
p(x1:t, m | z1:t,u1:t−1) = p(m | x1:t,z1:t) p(x1:t | z1:t,u1:t−1).
This factorization allows us to first estimate only the trajectory of the robot and then to compute the map given that trajectory.

# Gmapping
1. initial guess

2. scan match if faliure -> 3 else -> 4

3. sample

4. compute proposal

5.
