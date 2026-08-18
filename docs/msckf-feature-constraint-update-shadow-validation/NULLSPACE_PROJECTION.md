# Phase 22 Null-Space Projection

Date: July 19, 2026

## Method

The implementation computes an SVD of the stacked feature Jacobian `H_f`. The left null-space basis is taken from the trailing columns of `U`, and the projected system is:

```text
r_o = A^T r
H_o = A^T H_x
R_o = A^T R A
```

The projected Jacobian width equals the augmented covariance width.

## Validation

The unit suite checks `A^T H_f` annihilation and `A^T A` orthogonality. Current local MSVC results pass with tolerances below `1.0e-8` in the targeted null-space test.

## Rejection Conditions

Projection rejects empty tracks, failed SVD, non-positive projected dimension, non-finite projected matrices, annihilation tolerance failure, orthogonality tolerance failure, and projected covariance symmetry failure.
