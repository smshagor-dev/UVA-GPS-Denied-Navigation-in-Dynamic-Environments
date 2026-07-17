# Estimator Invariants

- The quaternion must remain normalized.
- Covariance must remain finite and symmetric.
- Invalid measurements must not modify the estimator state.
- Timestamps must not move backwards.
- Measurement corrections must remain bounded.
- The estimator must never output NaN or infinity.
- Experimental estimators must not control navigation.
- Loop closure must not directly hard-reset the local estimator.
- Software validation evidence does not imply flight safety.
