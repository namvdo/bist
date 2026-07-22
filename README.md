# Bounded Invariant Set Toolbox (BIST)

<div align="center">
  <img src="./frontend/public/bist-logo-full.png" alt="BIST - Bounded Invariant Set Toolbox logo" width="520" />
</div>

Iteractive web-based visualization tool for exploring set-valued dynamical systems with additive bounded noise, developed as part of the Advanced Computing Project (ACP2) research course at the University of Oulu.

#### Live at: [https://namvdo.github.io/set-valued-viz](https://namvdo.github.io/set-valued-viz)
#### Technical report: [https://namvdo.github.io/bist_technical_report_24042026.pdf](https://namvdo.github.io/bist_technical_report_24042026.pdf)

## Mathematical Background

In classical analysis, a **single-valued function** (or simply a function) $f: X \to Y$ assigns each point $x \in X$ to exactly one point $y \in Y$, written $y = f(x)$. Traditional dynamical systems using single-valued maps to describe deterministic evolution: given initial state $x_0$, the trajectory is uniquuely determined as $x_1 = f(x_0), x_2 = f(x_1), x_3= f(x_2)$ and so forth.

In contrast, a **set-valued function** (or **multivalued map**) $F: X \to \mathcal{(Y)}$ assigns to each point $x \in X$ a **subset** $F(x) \subseteq Y$ where $\mathcal{P}(Y)$ denotes the power set of $Y$. Rather than producing a single output, set-valued functions produce **a set of possible outputs**:

$F(A) = \bigcup_{x \in A} F(x)$
In our setting, we model bounded additive noise through set-valued map:
$F(x) = B_\epsilon(f(x)) = \{f(x) + \xi : \|\xi\| \leq \epsilon\}$ where $f: \mathbb{R}^n \to \mathbb{R}^n$ is the underlying single-valued deterministic map (the Hénon map in our case), and $B_\epsilon(f(x))$ represent all possible perturbed states within distance $\epsilon$ of the deterministic image.

Rather than tracking every possible point within the noise ball $B_\epsilon(f(x))$ which would be computationally expensive to compute as the noise balls grow, we instead track the boundary evolution through an extended boundary map $F(x,y,nx,ny)=\bigl(f(x,y)+\varepsilon\,\mathbf{nx}',\mathbf{ny}'\bigr)$. Since the maximum uncertainty occurs at the boundary $\partial B_\epsilon(f(x))$ (points at distance exactly $\epsilon$ from the deterministic image), we focus exclusively on tracking how these boundary points evolve.

### Unstable manifold visualization for boundary map evolution with a=0.4, b=0.3 and epsilon=0.0625

![Set-valued dynamical system with additive bounded noise Visualization](./images/unstable_manifold_for_boundary_map.png)

### Geometric offset contours around the MIS

After computing a closed unstable-manifold approximation of the MIS boundary, the **Geometric offsets** panel computes

$$G_k=M_0\oplus\overline B_{k\epsilon},\qquad \partial G_k=\{x:\operatorname{sdist}_{M_0}(x)=k\epsilon\}.$$

Thus consecutive contours have the requested set-distance gap \(\epsilon\), up to the reported numerical residual and grid uncertainty. The panel controls the number of levels, grid resolution, and contour visibility, and reports every target distance, area, component count, residual, and uncertainty. The current view is the computation domain, so widen it if a requested contour reaches the edge. See the rigorous mathematical and technical note as [LaTeX source](./docs/geometric_offset_contours.tex), [compiled PDF](./output/pdf/geometric_offset_contours.pdf), or [concise implementation note](./docs/geometric_offset_contours.md).

### Deterministic extended-map basin approximation

The **Basin of attraction** panel works in the extended boundary-map state space \((x,y,\theta)\), where \((x,y)\) is the boundary position and \(\theta\) specifies its unit normal direction. Interval inverse images discover predecessor candidates lazily, and a bidirectional consistency check requires every candidate's conservative forward row to reach the frontier box that generated it. Rows are stored exactly as merged row-major successor ranges. Forward verification reports a finite-capture inner set and a possible-capture outer set; their difference remains explicit uncertainty. The backend separately reports numerical target-sampling quality, forward-invariant trapping, a sufficient local contraction bound, domain containment, graph convergence, and a combined end-to-end verification flag, so finite capture is not confused with a complete attraction proof.

The panel provides **Draft**, **Standard**, and **Fine** accuracy presets plus validated numerical controls for the base position grid, normal-angle grid, persistent refinement passes, and boundary-sample count. It shows the effective three-dimensional cell count before computation and rejects settings above the backend's per-axis or 2,000,000-cell guards. A separate target-enclosure group exposes the position radius and normal tolerance with a warning that these values change the target set rather than merely increasing precision. Standard mode refines a `24 x 24 x 16` base grid to a persistent `48 x 48 x 32` graph and compares its angularly averaged areas with the coarser grid. Saturated yellow records verified finite capture and pale yellow records possible/unresolved capture; the legend identifies the two meanings separately, while one visibility switch controls both. **Compute basin** still expands until no new boxes are found, becomes **Cancel basin computation** while active, and reports clearly when the private resource guard, domain, resolution, target, or contraction checks prevent a complete conclusion. See [the implementation note](./docs/basin_approximation.md), [the rigorous LaTeX note](./docs/basin_approximation.tex), and [the compiled PDF](./output/pdf/basin_approximation.pdf).

### An example of a 4-periodic point found for the boundary map with A = 1.4 and B = 0.3, epsilon=0.0625

![4-periodic point](./images/periodic_orbit_visualization.png)

### ULAM method integration for stationary measure in continuous dynamical systems

![ULAM method integration for stationary measure in continuous dynamical systems](./images/ulam_integration_for_continuous_ds.png)

### Continuous boundary differential equation simulation

![Continuous boundary differential equation simulation](./images/boundary_differential_equation_visualization.png)

### Parameter sweeping for finding fixed and periodic orbits for boundary map of the discrete dynamical systems

![Parameter sweeping for finding fixed and periodic orbits for boundary map of the discrete dynamical systems](./images/parameter_sweep.png)

## **Getting Started**

### **1. Clone the Repository**

```bash
git clone <repository-url>
cd set-valued-viz
```

### **2. Build WebAssembly Module**

```bash
cd frontend && npm install && npm run build-wasm
```

This creates the WebAssembly module in `frontend/pkg/` so the worker and UI use the current Rust implementation.

### **3. Start the frontend server**

```bash
cd frontend && npm install && npm run dev
```

## License

MIT License — see [LICENSE](./LICENSE) for the full text.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
