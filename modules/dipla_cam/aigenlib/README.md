# Aigen: Focused Numerics, Familiar Code

Aigen is a focused alternative to Eigen for applications that need a defined
set of numerical operations, a lean dependency list, and clear control over
memory use. It powers Diplacam's finite-grid Gaussian-process calculations by
default, while keeping Eigen available as an easy-to-select alternative.

The library lives in `aigenlib`; the familiar C++ interface is called `Aigen`.
Its purpose is straightforward: give the calculations this application needs
a small, inspectable home, without introducing a general-purpose numerical
dependency. Keep the useful familiarity. Make the implementation easier to own.

## Why Choose Aigen?

While Eigen is an excellent, broad numerical library; Aigen takes a narrower brief:
implement the operations used here, make their costs and limits explicit, and
check their results against Eigen. That focus is the reason to consider it,
not an apology for what it does not attempt.

- **A lean numerical dependency list.** The standalone library needs no Eigen,
  Armadillo, BLAS, LAPACK, OpenCV nor ONNX Runtime dependency. Its comparison
  tests use Eigen, but your Aigen-only build does not need to carry it.
- **Deliberate memory use.** Bounded storage and explicit workspaces make large
  allocations easier to reason about. GP conditioning avoids creating another
  full covariance-sized product just to update the existing one.
- **Familiar code.** Names such as `MatrixXd`, `Vector3d`, and `Matrix4f` make
  the supported interface recognizable to Eigen users.
- **Checks where they matter.** Checked element access, input validation, and
  explicit solver failures help expose bad inputs rather than hide them.
- **A permissive foundation.** The code uses Apache-2.0, as does the
  OpenCV project. Attribution to our inspiration for Jacobi rotation
  as in OPenCV is retained;
- **A reversible choice.** You can try Aigen, compare it with Eigen, and switch
  back without maintaining two versions of the supported calculation code.

Choose Aigen when its supported operations match your application and you value
a numerical layer that is focused, testable, and easy to inspect. Choose Eigen
when you need its wider capabilities. Keeping that choice open is a feature.

## Why the Name Aigen?

The name nods to the Austro-Bavarian spelling *Aigen*, associated with freehold
property: something you own. It is also a deliberate echo of *Eigen*, reflecting
the familiar interface while giving this independent implementation its own
name. We think the name fits the project: a focused numerical layer you can
understand, inspect, and take responsibility for.

## Try Aigen, Keep Eigen

Trying a different numerical library should not feel like signing a lifetime
contract. In Diplacam, the choice is already built in: Aigen is the default,
and Eigen remains available. No farewell ceremony for Eigen required.

For the operations both implementations support, your application keeps the
same calculation code. You choose which library to use, rebuild, and run your
usual checks. Want to compare results? Build once with each. Prefer Eigen for
your workload? Select it again and rebuild. Want to return to Aigen later?
The same choice is still there.

This does not require changing your models, settings, or data formats. Nor do
you need to delete Eigen from your development machine to give Aigen a try.
The choice takes effect when you build the application, not while it is running.

In your own project, the same idea is to keep the library choice in one shared
place instead of spreading it throughout your calculations. Once your code uses
the common, supported interface, switching back and forth can stay a small
configuration change rather than another migration project.

There is one important boundary: Aigen supports a subset of Eigen's interface.
Code that relies on Eigen-only features may need an initial adaptation or may
be better left on Eigen. Easy switching is the goal for that shared subset,
not a promise that every existing Eigen program works unchanged.

## YES great, but how? Quick Start

Run these commands from the Sensoraia root:

```sh
make diplacam                              # use Aigen, the default
make diplacam DIPLA_MATH_BACKEND=eigen     # try Eigen
make diplacam DIPLA_MATH_BACKEND=aigenlib  # switch back to Aigen
./bin/diplacam --build-info                # confirm the compiled choice
```

The root Makefile reconfigures the existing build directory, so switching needs
no clean. Build the two choices one after the other, not concurrently into the
same directory or output file.

## Familiar C++ Interface

For supported code, one namespace alias gives the calculations a common name:

```cpp
#include "aigenlib/compat.hpp"
// #include <Eigen/Dense>

namespace Math = Aigen;
// namespace Math = Eigen;

Math::Matrix4f myMatrix = Math::Matrix4f::Identity();
const Math::Matrix4f product = myMatrix.transpose() * myMatrix;
```

To use Eigen in this example, include `<Eigen/Dense>` and change the alias to
`namespace Math = Eigen`. Supply Eigen's include directory when building.
`Aigen` always names the aigenlib implementation, even in an Eigen-selected
application, so the two can also coexist for direct comparisons.

### Supported Types

| Type | What It Provides |
| --- | --- |
| `MatrixXd` | Bounded, owning, row-major double-precision matrix storage. |
| `VectorXd` | A bounded, owning double-precision vector. |
| `Vector3d` | Three-coordinate double-precision point arithmetic. |
| `Matrix4f` | A fixed 4-by-4 matrix with actual float storage and arithmetic. |
| `Matrix4d` | A fixed 4-by-4 double-precision matrix. |

The fixed-size matrices provide checked access, zero initialization, `Zero`,
`Constant`, `Identity`, `rows`, `cols`, `size`, `allFinite`, `trace`, `transpose`,
and matrix multiplication. They use column-major storage and return owning
values rather than lazy expressions. Arithmetic can still overflow; use
`allFinite` to check results.

The compatibility layer does not emulate Eigen's general expression system,
block views, decompositions, or full templated `Matrix` API. The supported
subset is intentionally explicit so you can judge the fit before adopting it.

### Build Integration

Direct CMake builds accept `-DDIPLA_MATH_BACKEND=aigenlib` or `eigen`. A fresh
configuration defaults to Aigen; an explicitly selected choice stays in CMake's
cache. Invalid choices fail rather than silently selecting something else.

Diplacam's `aipp::math` namespace follows the build selection and exports the
Eigen-style names plus the existing planner aliases `Matrix`, `Vector`, and
`Point`. Using `namespace Math = aipp::math` lets that selection reach callers
without editing their source. Use the `dipla_math` CMake interface target so
all translation units receive a consistent `DIPLA_USE_EIGEN` definition.

Packages identify the backend from the actual executable's build-info record
and write it to `share/diplacam/math-backend.txt`. The selection does not change
how ONNX Runtime is chosen or remove its separate dependency. Diplacam metadata
uses JSONbourne; neither numerical backend requires an OpenCV library link.

## Build the Library

You can build the numerical library separately from the application:

```sh
make -j32 diplacam-aigenlib
make -j32 diplacam-aigenlib-arm64
```

The native archive is `.build/diplacam-aigenlib/libdipla_aigenlib.a`; the ARM64
archive is `.build/arm64/diplacam-aigenlib/libdipla_aigenlib.a`.

ARM64 builds use the prepared Trixie SDK. To build the full application, use
`make diplacam-arm64 DIPLA_MATH_BACKEND=aigenlib`. These commands build only;
they do not deploy anything.

## Numerical Operations

- Owning, initialized, contiguous row-major `Matrix` and standard-vector `Vector`.
- Checked element/row access and three-coordinate `Point` arithmetic.
- `solve_symmetric`: scaled Cholesky with multiple RHS, then cyclic Jacobi
  spectral pseudoinverse if the matrix is not positive definite.
- `matern32`: symmetric Matern-3/2 covariance over supplied cell positions.
- Sparse uniform-average `Sensor`, `project`, `innovation`, and `observe`.
- `expected_reduction` and optional-mean `condition` matching the GP equations.

To reproduce the current planar grid, pass zero-altitude positions in row-major
cell order. Grid and footprint construction, confidence masks, noise sampling,
splines, and CMA-ES stay in the application and are shared by both backends.
Switching the numerical engine does not introduce a second planner to maintain.

## Confidence Built on Comparison

Aigen's tests use Eigen as an independent reference for the implemented
operations. That is a more useful promise than a blanket compatibility score:
you can inspect what is checked, run the comparisons, and decide whether the
coverage fits your application.

The differential suite covers:

- Storage, copying, moves, bounds, invalid inputs, and nonfinite values.
- Point arithmetic and float `Matrix4f` products compared directly with Eigen.
- Empty systems, multiple right-hand sides, and positive-definite and indefinite
  systems up to 64 dimensions.
- Scales from `1e-200` to `1e200`, rank deficiency, rotated null spaces,
  negative eigenvalues, and the spectral cutoff.
- GP covariance, projection, innovation, observations, information gain, and
  posterior updates, including three consecutive updates on the full
  2601-cell grid and duplicate noiseless observations on a smaller grid.

Both application backends also use the same GP, spline, CMA-ES, DDIM, settings,
and worker regression suites. Numerical comparisons use the tests' stated
tolerances; they are not a promise of bit-for-bit identity or universal Eigen
API compatibility.

### Performance and Resource Expectations

Aigen's contiguous kernels are designed to support compiler vectorization and
avoid unnecessary expression temporaries. These are concrete implementation
choices, not a claim to beat Eigen on every workload. Matched desktop and real
MORA benchmarks are still needed before claiming a speed advantage.

Covariance symmetrization uses 32-by-32 tiles to keep paired rows in cache
instead of repeatedly walking the full matrix by column. Each pair retains
the same arithmetic. Single-row observations fuse their covariance update with
this tiled pass, computing both triangle entries before the original average.
Opposite-row addresses advance through the already validated matrix storage.
This avoids a separate matrix-update traversal and the scratch row without
changing measurement admission, sensor noise, precision or reduction order.
No covariance-sized temporary or additional thread is introduced.

A matched Pi Zero 2 W test on 2026-10-28 measured single-row conditioning CPU
time falling from 359.538 ms to 258.428 ms, about 28%, with bit-for-bit mean and
covariance checks. Three complete fixed-input plans retained exactly the same
eight CSV points; full-plan CPU savings were about 3%, not 28%. A real UART
exercise with 100 seconds following used 99.65 CPU-seconds over a 180.82-second
survey interval before the change and 91.72 CPU-seconds over 175.72 seconds
after it. Both completed all eight points, HOME and landing. These are measured
runs, not a guarantee of the same speedup for every footprint or mission.
An attempted fused finiteness scan was faster on the PC but slower on MORA and
was discarded; all original input and posterior finiteness checks remain.

The current ARM64 build explicitly tunes for the Raspberry Pi Zero
2 W's Cortex-A53 as the reference target; MORA is not restricted to that board.
Other ARM64 boards need an appropriate build profile and runtime validation.
This tuning does not enable fast-math or reduce numerical precision.

Storage is dynamic but bounded, not heap-free. One 2601-by-2601 double covariance
occupies 54,121,608 bytes, about 51.6 MiB. Conditioning adds two
measurement-by-cell matrices, small solver matrices, and 4 KiB scratch per active
update lane, rather than another full covariance product. Large multi-observation
updates use at most three lanes, sharing read-only projection/solution matrices
and writing disjoint covariance rows. They join before symmetry restoration;
unavailable threads fall back to serial work. Dot-product order is unchanged.
Thread stacks and runtime storage are additional to scratch. The planner's
64-row look-ahead cap is not a universal
limit on observations; large observation systems remain expensive.

Host tests and ARM64 builds do not establish target timing, planning quality,
flight readiness, or whole-board memory headroom. On the Pi Zero 2 W reference,
the shared 512 MiB must also accommodate Linux, camera buffers, other services
and peak allocations. Other MORA boards have different resource budgets that
need their own measurements. Removing Eigen alone does not make the whole
application static.

## Numerical Contract

For integration and review, these are the important operating boundaries:

- Dimensions are capped at 2601 before multiplication and allocation. Shapes,
  sensor indices, noise variances, and the finite values used by operations are
  checked. Candidate scoring checks the projected rows and selected diagonal,
  not unrelated covariance entries; conditioning validates the full covariance
  before mutation.
- Element access throws in release and debug builds. Borrowed row pointers
  require correctly bounded column loops and valid storage lifetimes.
- Each operation owns its workspace. There are no global scratch buffers,
  persistent background workers, or fast-math assumptions. Large conditioning
  updates may use the bounded CPU lanes described above. Allocation failures
  propagate normally; synchronize shared belief access externally.
- The solver checks symmetry at `64 * epsilon` relative to the largest input
  entry and uses the lower triangle. It normalizes the scale and first attempts
  Cholesky without jitter. If that fails, a signed spectral inverse uses the
  strict cutoff `abs(lambda) > 1e-15 * max(abs(lambda))`.
- The spectral solver reports failure if it does not converge within 80 sweeps.
  Successful Cholesky does not apply the spectral cutoff. Modes close to that
  cutoff remain sensitive to floating-point rounding; this is not an
  arbitrary-precision or general SVD implementation.
- Validation and allocation precede belief mutation. Arithmetic overflow during
  a covariance update throws without rolling back the full matrix; discard
  that trial belief rather than continuing with a partial update.

## Licensing

The library is licensed under Apache-2.0. OpenCV 4.5.0 and later, including
OpenCV 4.14.0 used for the Jacobi rotation inspiration, are also licensed under
Apache-2.0; see OpenCV's [official licensing page](https://opencv.org/license/).

Armadillo inspired us to implement the separation of owning storage from named
operations; it is not imported code or a dependency. Also no Eigen source is
copied into aigenlib or its compatibility layer.

This gives projects that prefer a permissively licensed numerical core another
option. It does not mean that Eigen's MPL-2.0 is automatically incompatible with
Apache-2.0; obligations depend on the code and how it is modified and distributed.

See [NOTICE.md](NOTICE.md) for the project notices and the repository's
[LICENSE](../../../LICENSE) for Apache-2.0. When distributing the derived code,
retain the applicable upstream copyright statements and source-file notices.
Eigen-selected builds retain their separate Eigen obligations, and other
application dependencies keep their own licenses. This is implementation
guidance, not a legal compliance audit.
