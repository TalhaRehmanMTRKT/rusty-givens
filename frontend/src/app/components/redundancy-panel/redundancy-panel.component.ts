import {
  Component,
  Input,
  OnChanges,
  SimpleChanges,
  ViewChild,
} from '@angular/core';
import { CommonModule } from '@angular/common';
import { MatCardModule } from '@angular/material/card';
import { MatChipsModule } from '@angular/material/chips';
import { MatIconModule } from '@angular/material/icon';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatInputModule } from '@angular/material/input';
import { MatSelectModule } from '@angular/material/select';
import { MatPaginator, MatPaginatorModule } from '@angular/material/paginator';
import { MatSort, MatSortModule } from '@angular/material/sort';
import { MatTableDataSource, MatTableModule } from '@angular/material/table';
import { MatTooltipModule } from '@angular/material/tooltip';
import { MatButtonModule } from '@angular/material/button';
import { FormsModule } from '@angular/forms';

import {
  RedundancyResult,
  MeasRedundancy,
  LocalRedundancy,
  DisplayLevel,
  getDisplayLevel,
} from '../../models/network.model';

interface MeasRow extends MeasRedundancy {
  display_level: DisplayLevel;
}

@Component({
  selector: 'app-redundancy-panel',
  standalone: true,
  imports: [
    CommonModule,
    FormsModule,
    MatCardModule,
    MatChipsModule,
    MatIconModule,
    MatFormFieldModule,
    MatInputModule,
    MatSelectModule,
    MatTableModule,
    MatPaginatorModule,
    MatSortModule,
    MatTooltipModule,
    MatButtonModule,
  ],
  templateUrl: './redundancy-panel.component.html',
  styleUrl: './redundancy-panel.component.scss',
})
export class RedundancyPanelComponent implements OnChanges {
  @Input() data: RedundancyResult | null = null;

  @ViewChild('measPaginator') measPaginator!: MatPaginator;
  @ViewChild('measSort') measSort!: MatSort;
  @ViewChild('busPaginator') busPaginator!: MatPaginator;
  @ViewChild('busSort') busSort!: MatSort;

  levelFilter: DisplayLevel | '' = '';
  methodExpanded = false;

  nCritical = 0;
  nWeak = 0;
  nSingle = 0;
  nMultiple = 0;

  measColumns = [
    'index',
    'measurement_label',
    'measurement_type',
    'sub_system',
    'display_level',
    'w_ii',
    'max_abs_k_ik',
    'detection_reliable',
  ];
  measSource = new MatTableDataSource<MeasRow>([]);

  busColumns = [
    'bus_index',
    'n_connected_measurements',
    'min_w_ii',
    'n_critical',
    'n_simply_redundant',
    'n_multiply_redundant',
  ];
  busSource = new MatTableDataSource<LocalRedundancy>([]);

  private allMeasRows: MeasRow[] = [];

  // ── Tooltip content ────────────────────────────────────────────

  readonly TIP_ETA = 'Redundancy ratio η = m / (2n − 1), where m = measurements and n = buses. ' +
    'η ≥ 1.0 is required for observability; η > 2.0 indicates good redundancy for bad data detection.';

  readonly TIP_DOF = 'Degrees of freedom f = m − (2n − 1). ' +
    'Represents the number of surplus measurements beyond the minimum needed. f > 0 is necessary for any redundancy.';

  readonly TIP_MEAS = 'Total number of active measurement equations (m) fed into the state estimator. ' +
    'Includes voltmeters, wattmeters, varmeters, ammeters and PMU pairs.';

  readonly TIP_SV = 'State variables = 2n − 1: n voltage magnitudes + n voltage angles, minus one angle fixed at the slack bus.';

  readonly TIP_SUFF = 'A measurement set is sufficient when m ≥ 2n − 1 (η ≥ 1.0). ' +
    'This is a necessary condition for observability, but not sufficient — measurements must also be well-distributed (local redundancy).';

  readonly TIP_CARD_CRITICAL = 'Classification: w_ii < ε (ε = 10⁻⁶)\n\n' +
    'This measurement has near-zero detection sensitivity. ' +
    'Losing it would make at least one state variable unobservable. ' +
    'Bad data in this measurement cannot be detected.';

  readonly TIP_CARD_WEAK = 'Classification: w_ii < 0.3 (detection sensitivity minimum)\n\n' +
    'A redundant partner exists (w_ii > ε), but the sensitivity is too low for ' +
    'reliable bad data detection. The Largest Normalized Residual test would miss errors here.';

  readonly TIP_CARD_SINGLE = 'Classification: M₁ — max|k_ik| ≥ λ_K (λ_K = 1/√2 ≈ 0.707) and w_ii ≥ 0.3\n\n' +
    'This measurement has one dominant correlated partner. ' +
    'Bad data can be reliably DETECTED, but not uniquely identified — ' +
    'the error might be attributed to the partner instead.';

  readonly TIP_CARD_MULTIPLE = 'Classification: M₂ — max|k_ik| < λ_K and w_ii ≥ 0.3\n\n' +
    'No single partner dominates the correlation. The measurement error ' +
    'is distributed across multiple neighbors, enabling both DETECTION and IDENTIFICATION ' +
    'of the faulty measurement via the normalized residual test.';

  readonly TIP_COL_ROW = 'Equation row index in the Jacobian matrix H. ' +
    'Measurements are ordered: voltmeters → ammeters → wattmeters → varmeters → PMU.';

  readonly TIP_COL_LABEL = 'Measurement identifier: type and location (bus injection or branch flow).';

  readonly TIP_COL_TYPE = 'Physical quantity: Voltmeter (|V|), Ammeter (|I|), Wattmeter (P), Varmeter (Q), or PMU (|V| + θ).';

  readonly TIP_COL_SS = 'Sub-system under the decoupled model:\n' +
    '• U — voltage magnitude (voltmeters, PMU magnitude)\n' +
    '• P — active power (wattmeters, PMU angle)\n' +
    '• Q — reactive power (varmeters)\n' +
    '• Mixed — ammeters couple P and Q, breaking the decoupling\n\n' +
    'Correlation k_ik is only computed between measurements in the same sub-system.';

  readonly TIP_COL_LEVEL = 'Four-level classification combining the M₀/M₁/M₂ class with detection sensitivity:\n' +
    '• Critical — w_ii ≈ 0 (M₀)\n' +
    '• No Eff. Redundancy — w_ii < 0.3\n' +
    '• Single Redundancy — M₁ with w_ii ≥ 0.3\n' +
    '• Multiple Redundancy — M₂ with w_ii ≥ 0.3';

  readonly TIP_COL_WII = 'Detection sensitivity w_ii = Ω_ii / σ²_i ∈ [0, 1]\n\n' +
    'Computed from the diagonal of the residual covariance Ω = R − H G⁻¹ Hᵀ, ' +
    'where R = diag(σ²) is the measurement noise covariance, ' +
    'H is the Jacobian, and G = Hᵀ W H is the gain matrix.\n\n' +
    'w_ii = 0 → critical (no error detection)\n' +
    'w_ii < 0.3 → unreliable detection (amber)\n' +
    'w_ii ≥ 0.3 → reliable detection';

  readonly TIP_COL_KIK = 'Correlation coefficient k_ik = Ω_ik / √(Ω_ii · Ω_kk)\n\n' +
    'Measures how strongly two measurements share information through the network model. ' +
    'Computed only for "neighbors" — measurements that share at least one state variable (Jacobian column).\n\n' +
    '|k_ik| ≥ 0.707 → simply redundant (M₁, one dominant partner)\n' +
    '|k_ik| < 0.707 → multiply redundant (M₂, distributed)';

  readonly TIP_COL_BDD = 'Bad Data Detection reliability (w_ii ≥ 0.3).\n\n' +
    '✓ = The Largest Normalized Residual (rᴺ) test can reliably detect errors in this measurement.\n' +
    '✗ = Detection sensitivity is too low; bad data may go undetected even with large errors.';

  readonly TIP_BUS = 'Bus index in the internal network model. Each bus has associated voltage magnitude and angle state variables.';

  readonly TIP_BUS_CONNECTED = 'Number of measurement equations that reference this bus ' +
    '(bus injections + adjacent branch flows). More connected measurements → better local redundancy.';

  readonly TIP_BUS_MIN_WII = 'Lowest w_ii among all measurements associated with this bus. ' +
    'A low value indicates the bus has at least one weakly observed measurement.';

  readonly TIP_BUS_M0 = 'M₀ — Critical measurements at this bus. If > 0, the bus has measurements whose loss causes unobservability.';

  readonly TIP_BUS_M1 = 'M₁ — Simply-redundant measurements at this bus. Error detection is possible, but identification is limited to a pair.';

  readonly TIP_BUS_M2 = 'M₂ — Multiply-redundant measurements at this bus. Both error detection AND identification are possible.';

  // ── Lifecycle ──────────────────────────────────────────────────

  ngOnChanges(changes: SimpleChanges): void {
    if (changes['data'] && this.data) {
      this.allMeasRows = this.data.measurements.map((m) => ({
        ...m,
        display_level: getDisplayLevel(m),
      }));

      this.nCritical = this.allMeasRows.filter(
        (r) => r.display_level === 'critical',
      ).length;
      this.nWeak = this.allMeasRows.filter(
        (r) => r.display_level === 'weak',
      ).length;
      this.nSingle = this.allMeasRows.filter(
        (r) => r.display_level === 'single',
      ).length;
      this.nMultiple = this.allMeasRows.filter(
        (r) => r.display_level === 'multiple',
      ).length;

      this.applyLevelFilter();
      this.busSource.data = this.data.local;

      setTimeout(() => {
        if (this.measPaginator)
          this.measSource.paginator = this.measPaginator;
        if (this.measSort) this.measSource.sort = this.measSort;
        if (this.busPaginator)
          this.busSource.paginator = this.busPaginator;
        if (this.busSort) this.busSource.sort = this.busSort;
      });
    }
  }

  applyLevelFilter(): void {
    if (this.levelFilter) {
      this.measSource.data = this.allMeasRows.filter(
        (r) => r.display_level === this.levelFilter,
      );
    } else {
      this.measSource.data = this.allMeasRows;
    }
    if (this.measPaginator) this.measPaginator.firstPage();
  }

  filterMeas(event: Event): void {
    const val = (event.target as HTMLInputElement).value.trim().toLowerCase();
    this.measSource.filter = val;
  }

  filterBus(event: Event): void {
    const val = (event.target as HTMLInputElement).value.trim().toLowerCase();
    this.busSource.filter = val;
  }

  levelLabel(level: DisplayLevel): string {
    switch (level) {
      case 'critical':
        return 'Critical';
      case 'weak':
        return 'No Eff. Redundancy';
      case 'single':
        return 'Single Redundancy';
      case 'multiple':
        return 'Multiple Redundancy';
    }
  }

  levelIcon(level: DisplayLevel): string {
    switch (level) {
      case 'critical':
        return 'error';
      case 'weak':
        return 'warning';
      case 'single':
        return 'verified';
      case 'multiple':
        return 'shield';
    }
  }

  onCardClick(level: DisplayLevel): void {
    this.levelFilter = this.levelFilter === level ? '' : level;
    this.applyLevelFilter();
  }
}
