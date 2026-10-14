import numpy as np
from scipy.spatial import cKDTree
from go2_fastlio_localization.pcd_icp_localizer import (
    ICPResult, icp_rejection_reason, run_icp)


def cloud():
    return np.random.default_rng(42).uniform(-2., 2., (500, 3))


def test_matching_cloud_converges_with_final_residual():
    points = cloud()
    initial = np.eye(4)
    initial[0, 3] = .03
    result = run_icp(points, cKDTree(points), initial, .5)
    assert result.converged and result.inlier_ratio == 1.
    assert result.rmse < 1e-8
    assert not icp_rejection_reason(result, initial, .45, .6, .5, .35)


def test_iteration_limit_does_not_masquerade_as_convergence():
    points = cloud()
    initial = np.eye(4)
    initial[0, 3] = .03
    result = run_icp(points, cKDTree(points), initial, .5, max_iterations=1)
    assert result.rmse < .45 and not result.converged
    assert icp_rejection_reason(result, initial, .45, .6, .5, .35) == "not converged"


def test_low_residual_cannot_hide_low_overlap_or_large_correction():
    result = ICPResult(np.eye(4), .01, 60, .12, True)
    assert icp_rejection_reason(result, np.eye(4), .45, .6, .5, .35) == "insufficient overlap"
    result.inlier_ratio = .9
    result.transform[0, 3] = .51
    assert icp_rejection_reason(result, np.eye(4), .45, .6, .5, .35) == "correction too large"


def test_insufficient_matches_are_rejected():
    points = cloud()
    result = run_icp(points+10., cKDTree(points), np.eye(4), .1)
    assert not result.converged and result.inliers == 0
