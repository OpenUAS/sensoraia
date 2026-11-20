# Aigen: Notices and Attribution

Aigen keeps its numerical scope focused and its credits visible. This notice
explains which parts belong to the project, where inspiration and adapted code
come from, and which notices should travel with a distribution.

For features, examples, and easy switching between Aigen and Eigen, start with
the [README](README.md). This file covers attribution and licensing; it
accompanies the licenses rather than replacing them.

## Project Code and License

Copyright 2026 OpenUAS.

The storage, point arithmetic, Cholesky implementation, sparse
projection, Gaussian-process kernels, and compatibility code are licensed under
Apache-2.0. OpenCV 4.14.0 also uses Apache-2.0 as its project license. The
OpenCV-derived rotation is credited below, together with the older notice
still present in the particular upstream source file.

The repository's [LICENSE](../../../LICENSE) contains the Apache License 2.0.
Individual source files carry their applicable copyright statements and SPDX
identifiers; retain those notices as well as this project notice.

## Familiar to Eigen Users, Independently Implemented

Aigen provides a deliberately limited, Eigen-style interface for the operations
used by this application. No Eigen implementation is copied into aigenlib or
the `Aigen` compatibility layer. Familiar names make supported code easier to
switch; they do not make Aigen a complete replacement for every Eigen feature.

Eigen has two separate roles here: it is the independent reference used by the
numerical comparison tests, and it remains an optional application backend.
The default backend is aigenlib. An Aigen-only library build does not require
Eigen; comparison tests and Eigen-selected builds do.

Choosing Eigen retains the licensing obligations that apply to the Eigen code
you use and distribute. Choosing Aigen does not remove the obligations of
other application dependencies. Switching the engine does not switch off the
credits.

## Armadillo: Design Inspiration

Armadillo's Apache-2.0 design inspired the separation of owning storage from
named numerical operations and the effort to avoid unnecessary intermediate
matrices. No Armadillo source is copied or linked, and Armadillo is not a
dependency of aigenlib.

This is an acknowledgment of design inspiration, not a claim that Aigen includes
Armadillo's implementation or offers its complete capabilities.

## OpenCV: Adapted Jacobi Rotation

**OpenCV 4.5.0 and later are licensed under Apache License 2.0.** This includes
OpenCV 4.14.0, the version referenced here. OpenCV's
[official licensing page](https://opencv.org/license/) documents the change;
the release's [LICENSE](https://github.com/opencv/opencv/blob/4.14.0/LICENSE)
contains the Apache-2.0 terms. OpenCV 4.4.0 and earlier used BSD-3-Clause.

The stable Jacobi rotation in `aigenlib.cpp` is adapted from `JacobiImpl_` in
OpenCV 4.14.0, in `modules/core/src/lapack.cpp`:

<https://github.com/opencv/opencv/blob/4.14.0/modules/core/src/lapack.cpp>

That specific file still contains the Intel and Willow Garage copyright
statements and the older BSD-style notice reproduced below. We retain that
source-file notice as part of the adaptation's provenance; it is not a statement
that OpenCV 4.14.0 as a library is BSD-licensed. OpenCV's current Apache-2.0
project license and the notice retained in this source file are documented
separately here rather than conflated.

The standalone aigenlib library does not require an OpenCV library dependency;
the attribution concerns the adapted source.

The adaptation uses:

- A C++17 implementation with bounded, owning, row-major storage and
  double-precision solver arithmetic.
- Cyclic rotations rather than cached-maximum pivot selection, with
  eigenvectors stored as columns.
- Scaled inputs, scale-relative convergence checks, and an explicit failure
  when the iteration limit is reached.
- Direct application of the spectral pseudoinverse to multiple right-hand
  sides, avoiding unnecessary intermediate products.

The following notice is reproduced from that specific upstream source file
without changing its wording:

```text
IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.

By downloading, copying, installing or using the software you agree to this license.
If you do not agree to this license, do not download, install,
copy or use the software.


License Agreement
For Open Source Computer Vision Library

Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
Copyright (C) 2009, Willow Garage Inc., all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistribution's of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistribution's in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * The name of the copyright holders may not be used to endorse or promote products
    derived from this software without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are disclaimed.
In no event shall the Intel Corporation or contributors be liable for any direct,
indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
```

## What to Include When Distributing

For a distribution containing the Aigen implementation, retain this notice,
the applicable source-file notices, and the Apache License 2.0. For the
OpenCV-derived rotation, preserve the upstream copyright statements and
retained source notice reproduced above. This records the source attribution
alongside OpenCV's Apache-2.0 project license, not instead of it.

For an Eigen-selected build, retain the notices required by the Eigen code
actually included. Other dependencies, including ONNX Runtime and JSONbourne in
the full application, have their own distribution requirements. Diplacam no
longer links an OpenCV library; that does not remove the source attribution above.

The Diplacam packager uses the backend recorded in the actual executable to
select its backend-specific notices, rather than guessing from an environment
setting. Check the resulting package and any additional dependencies before
redistributing it; packaging automation is helpful, but is not a legal review.

These notes explain this implementation's provenance and distribution context.
They do not replace the license terms or constitute a legal compliance audit.
