This project uses Graduated Non-convexity(GNC) for optimization problem.
The code uses GNC pipe line with g2o and compares GNC with traditional robust kernel.

## 1. GNC
We use the Geman McClure GNC(GM-GNC) for its simplify.
The scheme of GNC is marked as following:
1. build optimization by g2o.
2. compute the error of each edge, find the max error $r_{max}$.
3. set the value $c$, which is 1 in our code.
4. initial $\mu = 2 \cdot r^2_{max}/c^2$.
5. compute edge's weight $w$ by
$$
w_i = \left(\frac{\mu c^2}{r^2 + \mu c^2}\right)^2
$$
where $i$ is the edge index.
6. optimize!
7. $\mu\leftarrow\mu/1.4$
8. loop from step 5 until the $\mu < 1$, and set $\mu = 1$ in the last iteration.

## 2. MatPlot
This code uses the python's  `matplotlibcpp` to plot. See more detail in the folder `ThirdParty/matplotlib-cpp`.
