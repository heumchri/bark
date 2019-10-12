#load("//:bazel/ceres.bzl", "ceres_library")

internal = 'external/%s/internal' % repository_name().lstrip('@')

schur_eliminator_copts=["-DCERES_RESTRICT_SCHUR_SPECIALIZATION"]
schur_sources = [
    "internal/ceres/generated/schur_eliminator_d_d_d.cc",
    "internal/ceres/generated/partitioned_matrix_view_d_d_d.cc",
]

CERES_SRCS = ["internal/ceres/" + filename for filename in [
    #at least used by...#

    #used by solver#
    "minimizer.cc", 
    "trust_region_minimizer.cc",
    "trust_region_preprocessor.cc",
    "trust_region_step_evaluator.cc",
    "trust_region_strategy.cc",
    "line_search.cc",
    "line_search_direction.cc",
    "line_search_minimizer.cc",
    "schur_eliminator.cc",
    "eigensparse.cc",
    "visibility_based_preconditioner.cc",
    "visibility.cc",
    "partitioned_matrix_view.cc",
    "cgnr_solver.cc",
    "conjugate_gradients_solver.cc",
    "dense_normal_cholesky_solver.cc",
    "dense_qr_solver.cc",
    "dense_sparse_matrix.cc",
    "detect_structure.cc",
    "dynamic_sparse_normal_cholesky_solver.cc",
    "sparse_normal_cholesky_solver.cc",

    #used by evaluator#
    "stringprintf.cc",
    "blas.cc",
    "block_evaluate_preparer.cc",
    "block_jacobian_writer.cc",
    "block_jacobi_preconditioner.cc",
    "block_random_access_dense_matrix.cc",
    "block_random_access_diagonal_matrix.cc",
    "block_random_access_matrix.cc",
    "block_random_access_sparse_matrix.cc",
    "block_sparse_matrix.cc",
    "block_structure.cc",
    "dynamic_compressed_row_jacobian_writer.cc",
    "iterative_schur_complement_solver.cc",
    "linear_solver.cc",
    "schur_complement_solver.cc",

    #used by blas#
    "canonical_views_clustering.cc",
    
    #used by trust_region_preprocessor#
    "coordinate_descent_minimizer.cc",

    #used by residual_block#
    "corrector.cc",

    #used by trust_region#
    "dogleg_strategy.cc",

    #used by dynamic_compressed_row_jacobian_writer#
    "dynamic_compressed_row_sparse_matrix.cc",
    
    #used by trust_region_minimizer#
    "file.cc",

    #used by line_search_#
    "function_sample.cc",

    # used by gradient_checking_cost_function and others#
    "gradient_checker.cc",
    "gradient_problem.cc",
    "gradient_problem_solver.cc",

    #used by graident checker#
    "is_close.cc",

    #used by iterative_schur_complement_solver#
    "implicit_schur_complement.cc",

    #used by sparse_normal_cholesky_solver#
    "inner_product_computer.cc",
    
    #used by dense_qr_solver#
    "lapack.cc",

    #used by many sources#
    "levenberg_marquardt_strategy.cc",
    "linear_operator.cc",
    "local_parameterization.cc",
    "sparse_cholesky.cc",
    "sparse_matrix.cc",
    "triplet_sparse_matrix.cc",

    #used by levenberg_marquardt_strategy#
    "linear_least_squares_problems.cc",

    #used by preprocessor#
    "line_search_preprocessor.cc",

    #used by line_search_direction#
    "low_rank_inverse_hessian.cc",

    #used by dogleg_strategy#
    "polynomial.cc",

    #used by trust_region_preprocessor#
    "preconditioner.cc",

    #used by trust_region_preprocessor
    "reorder_program.cc",

    #used by preconditioner#
    "schur_jacobi_preconditioner.cc",
    "subset_preconditioner.cc",

    #used by visibility_based_preconditioner#
    "single_linkage_clustering.cc",

    
    "suitesparse.cc",
    "thread_pool.cc",
    "thread_token_provider.cc",
]]

# CERES PROBLEM#

CERES_PROBLEM_SRCS = ["internal/ceres/" + filename for filename in [
    "problem.cc",
    "problem_impl.cc",
    "context.cc",
    "context_impl.cc",
    "types.cc",
    "compressed_row_jacobian_writer.cc",
    "compressed_row_sparse_matrix.cc",
    "evaluator.cc",
    "callbacks.cc",
    "array_utils.cc",
    "loss_function.cc",
    "parameter_block_ordering.cc",
    "program.cc",
    "residual_block.cc",
    "residual_block_utils.cc",
    "scratch_evaluate_preparer.cc",
]]

cc_library(
    name="ceres_problem",
    srcs= CERES_PROBLEM_SRCS + schur_sources,
    hdrs= glob(["include/ceres/*.h"]) + glob(["include/ceres/internal/*.h"]) 
        + glob(["internal/ceres/*.h"]) + ["config/ceres/internal/config.h"],
    copts = [ "-I" + internal,
            "-Wno-sign-compare",
        ] + schur_eliminator_copts,
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_eigen_eigen//:eigen",
        "@com_github_google_glog//:glog",
    ],
    includes = [
            "config",
            "include",
    ],
    defines = [
            "CERES_NO_SUITESPARSE",
            "CERES_NO_CXSPARSE",
            "CERES_NO_ACCELERATE_SPARSE",
            "CERES_NO_LAPACK",
            "CERES_USE_EIGEN_SPARSE",
            "CERES_NO_THREADS",
            "CERES_GFLAGS_NAMESPACE=gflags",
            "CERES_STD_UNORDERED_MAP",
        ],
)

CERES_SOLVER_SRCS = ["internal/ceres/" + filename for filename in [
    "solver.cc",
    "solver_utils.cc",
    "gradient_checking_cost_function.cc",
    "preprocessor.cc",
    "wall_time.cc",
    "schur_templates.cc",
]]

cc_library(
    name="ceres_solver",
    srcs = CERES_SOLVER_SRCS + schur_sources,
    hdrs =  glob(["include/ceres/*.h"]) + glob(["include/ceres/internal/*.h"])
            + ["config/ceres/internal/config.h"] + glob(["internal/ceres/*.h",])
            ,
    copts = [ "-I" + internal,
            "-Wno-sign-compare",
            ] + schur_eliminator_copts,
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_eigen_eigen//:eigen",
        "@com_github_google_glog//:glog",
    ],
    includes = [
            "config",
            "include",
    ],
    defines = [
            "CERES_NO_SUITESPARSE",
            "CERES_NO_CXSPARSE",
            "CERES_NO_ACCELERATE_SPARSE",
            "CERES_NO_LAPACK",
            "CERES_USE_EIGEN_SPARSE",
            "CERES_GFLAGS=OFF",
            "CERES_GFLAGS_NAMESPACE=gflags",
            "CERES_NO_THREADS",
            "CERES_STD_UNORDERED_MAP",
        ],
)

cc_library(
    name="ceres",
    srcs = CERES_SRCS + schur_sources,
    hdrs = glob(["include/ceres/*.h"]) + glob(["include/ceres/internal/*.h"]) +
            ["config/ceres/internal/config.h"] + glob(["internal/ceres/*.h",]),
    copts = [ "-I" + internal,
            "-Wno-sign-compare",
            ] + schur_eliminator_copts,
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_eigen_eigen//:eigen",
        "@com_github_google_glog//:glog",
        ":ceres_solver",
    ],
    includes = [
            "config",
            "include",
    ],
    defines = [
            "CERES_NO_SUITESPARSE",
            "CERES_NO_CXSPARSE",
            "CERES_NO_ACCELERATE_SPARSE",
            "CERES_NO_LAPACK",
            "CERES_USE_EIGEN_SPARSE",
            "CERES_GFLAGS=OFF",
            "CERES_GFLAGS_NAMESPACE=gflags",
            "CERES_NO_THREADS",
            "CERES_STD_UNORDERED_MAP",
        ],
)





#ceres_library(
#    name = "ceres",
#    restrict_schur_specializations = True,
#)