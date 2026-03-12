//! Linear system solvers for the gain equation G·Δx = rhs.

use crate::kernel::types::Factorization;

/// Solve G x = b using faer's dense Cholesky (with LU fallback).
pub(crate) fn solve_dense_faer(gain_flat: &[f64], rhs: &[f64], s: usize) -> Vec<f64> {
    use faer::linalg::solvers::Solve;
    let mut mat = faer::Mat::zeros(s, s);
    for r in 0..s {
        for c in 0..s {
            mat[(r, c)] = gain_flat[r * s + c];
        }
    }
    let mut rhs_mat = faer::Mat::zeros(s, 1);
    for i in 0..s {
        rhs_mat[(i, 0)] = rhs[i];
    }
    if let Ok(llt) = mat.as_ref().llt(faer::Side::Lower) {
        let sol = llt.solve(&rhs_mat);
        return (0..s).map(|i| sol[(i, 0)]).collect();
    }
    let lu = mat.as_ref().partial_piv_lu();
    let sol = lu.solve(&rhs_mat);
    (0..s).map(|i| sol[(i, 0)]).collect()
}

/// Solve G x = b using faer's sparse Cholesky or LU factorization.
///
/// Converts the sprs CSC matrix to faer's sparse format, factorizes, and solves.
/// The symbolic factorization is cached across iterations (sparsity pattern is constant).
pub(crate) fn solve_sparse_faer(
    gain: &sprs::CsMatView<f64>,
    rhs: &[f64],
    s: usize,
    backend: Factorization,
    sym_llt: &mut Option<faer::sparse::linalg::solvers::SymbolicLlt<usize>>,
    sym_lu: &mut Option<faer::sparse::linalg::solvers::SymbolicLu<usize>>,
) -> Vec<f64> {
    use faer::linalg::solvers::Solve;
    use faer::sparse::linalg::solvers as sp;

    // Convert sprs CSC → faer SparseColMatRef
    let col_ptrs: Vec<usize> = gain.indptr().raw_storage().to_vec();
    let row_indices = gain.indices();
    let values = gain.data();

    let faer_mat = faer::sparse::SparseColMatRef::new(
        unsafe {
            faer::sparse::SymbolicSparseColMatRef::new_unchecked(
                s, s, &col_ptrs, None, row_indices,
            )
        },
        values,
    );

    match backend {
        Factorization::SparseCholesky => {
            // Symbolic factorization: compute once, reuse
            if sym_llt.is_none() {
                *sym_llt = Some(
                    sp::SymbolicLlt::try_new(faer_mat.symbolic(), faer::Side::Lower)
                        .expect("sparse symbolic Cholesky failed"),
                );
            }

            let llt = sp::Llt::try_new_with_symbolic(
                sym_llt.clone().unwrap(),
                faer_mat,
                faer::Side::Lower,
            )
            .expect("sparse numeric Cholesky failed");

            let mut rhs_mat = faer::Mat::zeros(s, 1);
            for i in 0..s { rhs_mat[(i, 0)] = rhs[i]; }
            let sol = llt.solve(&rhs_mat);
            (0..s).map(|i| sol[(i, 0)]).collect()
        }
        Factorization::SparseLU => {
            if sym_lu.is_none() {
                *sym_lu = Some(
                    sp::SymbolicLu::try_new(faer_mat.symbolic())
                        .expect("sparse symbolic LU failed"),
                );
            }
            let lu = sp::Lu::try_new_with_symbolic(
                sym_lu.clone().unwrap(),
                faer_mat,
            )
            .expect("sparse numeric LU failed");

            let mut rhs_mat = faer::Mat::zeros(s, 1);
            for i in 0..s { rhs_mat[(i, 0)] = rhs[i]; }
            let sol = lu.solve(&rhs_mat);
            (0..s).map(|i| sol[(i, 0)]).collect()
        }
        _ => unreachable!(),
    }
}
