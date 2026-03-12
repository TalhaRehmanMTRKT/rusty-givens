//! Gain matrix assembly (G = J^T W J) for the Gauss-Newton WLS solver.

use sprs::CsMat;

/// Pre-computed CSC structure for the gain matrix G = J^T W J.
/// The sparsity pattern is determined by network topology and measurement placement,
/// so it is identical across Gauss-Newton iterations. We allocate the structure once
/// and only refill the values array.
pub(crate) struct GainCscCache {
    pub col_ptr: Vec<usize>,
    pub row_idx: Vec<usize>,
    pub values: Vec<f64>,
    pub s: usize,
}

impl GainCscCache {
    /// Build the cache from the first Jacobian by computing symbolic G.
    pub(crate) fn from_jacobian(j_csr: &sprs::CsMatI<f64, usize>, s: usize, slack: usize) -> Self {
        use std::collections::BTreeSet;
        let k = j_csr.rows();

        // Collect nonzero column pairs per-row → set of (col, row) in CSC
        let mut col_entries: Vec<BTreeSet<usize>> = vec![BTreeSet::new(); s];

        for meas_row in 0..k {
            if let Some(row_view) = j_csr.outer_view(meas_row) {
                let idx = row_view.indices();
                let nnz = idx.len();
                for a in 0..nnz {
                    let ca = idx[a];
                    if ca == slack { continue; }
                    for b in a..nnz {
                        let cb = idx[b];
                        if cb == slack { continue; }
                        col_entries[cb].insert(ca);
                        if ca != cb { col_entries[ca].insert(cb); }
                    }
                }
            }
        }
        // Slack bus diagonal
        col_entries[slack].insert(slack);

        // Build CSC arrays
        let mut col_ptr = Vec::with_capacity(s + 1);
        let mut row_idx = Vec::new();
        col_ptr.push(0);
        for col in 0..s {
            for &row in &col_entries[col] {
                row_idx.push(row);
            }
            col_ptr.push(row_idx.len());
        }
        let nnz = row_idx.len();
        let values = vec![0.0f64; nnz];

        GainCscCache { col_ptr, row_idx, values, s }
    }

    /// Fill gain values from Jacobian J (CSR), weights, and residual.
    pub(crate) fn fill_values(
        &mut self,
        j_csr: &sprs::CsMatI<f64, usize>,
        w_diag: &[f64],
        w_off: &[(usize, usize, f64)],
        slack: usize,
        rhs_buf: &mut [f64],
        wr: &[f64],
        j_csc: &CsMat<f64>,
    ) {
        self.values.iter_mut().for_each(|v| *v = 0.0);
        let k = j_csr.rows();
        let s = self.s;
        let col_ptr = &self.col_ptr;
        let row_idx = &self.row_idx;

        // rhs = J^T (W r)
        for col in 0..s {
            if let Some(j_col) = j_csc.outer_view(col) {
                let mut v = 0.0;
                for (i, &j_val) in j_col.iter() { v += j_val * wr[i]; }
                rhs_buf[col] = v;
            }
        }
        rhs_buf[slack] = 0.0;

        // G = J^T W J
        for meas_row in 0..k {
            let w_i = w_diag[meas_row];
            if let Some(row_view) = j_csr.outer_view(meas_row) {
                let idx = row_view.indices();
                let vals = row_view.data();
                let nnz = idx.len();
                for a in 0..nnz {
                    let ca = idx[a];
                    if ca == slack { continue; }
                    let wj = w_i * vals[a];
                    for b in a..nnz {
                        let cb = idx[b];
                        if cb == slack { continue; }
                        let val = wj * vals[b];
                        let i1 = csc_index(col_ptr, row_idx, ca, cb);
                        self.values[i1] += val;
                        if ca != cb {
                            let i2 = csc_index(col_ptr, row_idx, cb, ca);
                            self.values[i2] += val;
                        }
                    }
                }
            }
        }

        // Off-diagonal W (correlated PMUs)
        for &(r1, r2, w_val) in w_off {
            if let (Some(row1), Some(row2)) = (j_csr.outer_view(r1), j_csr.outer_view(r2)) {
                for (ca, &j1a) in row1.iter() {
                    if ca == slack { continue; }
                    for (cb, &j2b) in row2.iter() {
                        if cb == slack { continue; }
                        let val = w_val * j1a * j2b;
                        let i1 = csc_index(col_ptr, row_idx, ca, cb);
                        self.values[i1] += val;
                        if ca != cb {
                            let i2 = csc_index(col_ptr, row_idx, cb, ca);
                            self.values[i2] += val;
                        }
                    }
                }
            }
        }

        // Slack diagonal = 1
        let i_slack = csc_index(col_ptr, row_idx, slack, slack);
        self.values[i_slack] = 1.0;
    }

    /// Return a reference as an sprs CsMatView (zero-copy).
    pub(crate) fn as_csc_view(&self) -> sprs::CsMatView<'_, f64> {
        let shape = (self.s, self.s);
        sprs::CsMatView::new(shape, &self.col_ptr, &self.row_idx, &self.values)
    }
}

/// Look up the values-array index for entry (row, col) in a CSC matrix.
#[inline]
pub(crate) fn csc_index(col_ptr: &[usize], row_idx: &[usize], row: usize, col: usize) -> usize {
    let start = col_ptr[col];
    let end = col_ptr[col + 1];
    let slice = &row_idx[start..end];
    start + slice.binary_search(&row).expect("entry not in gain sparsity pattern")
}

pub(crate) fn build_gain_dense(
    j: &CsMat<f64>, w_diag: &[f64], w_off: &[(usize, usize, f64)],
    r: &[f64], k: usize, s: usize,
    gain_flat: &mut [f64], rhs_buf: &mut [f64],
) {
    let mut wr = vec![0.0f64; k];
    for i in 0..k { wr[i] = w_diag[i] * r[i]; }
    for &(r1, r2, val) in w_off {
        wr[r1] += val * r[r2];
        wr[r2] += val * r[r1];
    }
    for col in 0..s {
        if let Some(j_col) = j.outer_view(col) {
            let mut v = 0.0;
            for (i, &j_val) in j_col.iter() { v += j_val * wr[i]; }
            rhs_buf[col] = v;
        }
    }
    let j_csr = j.to_csr();
    for meas_row in 0..k {
        let w_i = w_diag[meas_row];
        if let Some(row_view) = j_csr.outer_view(meas_row) {
            let idx = row_view.indices();
            let vals = row_view.data();
            let nnz = idx.len();
            for a in 0..nnz {
                let ca = idx[a];
                let wj = w_i * vals[a];
                for b in a..nnz {
                    let cb = idx[b];
                    let val = wj * vals[b];
                    gain_flat[ca * s + cb] += val;
                    if ca != cb { gain_flat[cb * s + ca] += val; }
                }
            }
        }
    }
    for &(r1, r2, w_val) in w_off {
        if let (Some(row1), Some(row2)) = (j_csr.outer_view(r1), j_csr.outer_view(r2)) {
            for (ca, &j1a) in row1.iter() {
                for (cb, &j2b) in row2.iter() {
                    let val = w_val * j1a * j2b;
                    gain_flat[ca * s + cb] += val;
                    if ca != cb { gain_flat[cb * s + ca] += val; }
                }
            }
        }
    }
}

pub(crate) fn apply_slack_dense(gain: &mut [f64], rhs: &mut [f64], slack: usize, s: usize) {
    for i in 0..s {
        gain[slack * s + i] = 0.0;
        gain[i * s + slack] = 0.0;
    }
    gain[slack * s + slack] = 1.0;
    rhs[slack] = 0.0;
}
