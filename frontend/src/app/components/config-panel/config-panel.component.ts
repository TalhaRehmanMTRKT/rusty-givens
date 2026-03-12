import { Component, EventEmitter, Input, Output } from '@angular/core';
import { FormsModule } from '@angular/forms';
import { MatButtonModule } from '@angular/material/button';
import { MatCardModule } from '@angular/material/card';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatInputModule } from '@angular/material/input';
import { MatProgressBarModule } from '@angular/material/progress-bar';
import { MatSelectModule } from '@angular/material/select';
import { MatTooltipModule } from '@angular/material/tooltip';
import { EstimateRequest, Factorization, SolverFormulation } from '../../models/network.model';

@Component({
  selector: 'app-config-panel',
  standalone: true,
  imports: [
    FormsModule,
    MatCardModule,
    MatFormFieldModule,
    MatSelectModule,
    MatInputModule,
    MatButtonModule,
    MatProgressBarModule,
    MatTooltipModule,
  ],
  templateUrl: './config-panel.component.html',
  styleUrl: './config-panel.component.scss',
})
export class ConfigPanelComponent {
  @Input() running = false;
  @Output() runEstimate = new EventEmitter<EstimateRequest>();

  formulation: SolverFormulation = 'NormalEquations';
  factorization: Factorization = 'SparseCholesky';
  maxIterations = 50;
  tolerance = 1e-4;

  readonly formulationOptions: { value: SolverFormulation; label: string; hint: string }[] = [
    {
      value: 'NormalEquations',
      label: 'Normal Equations (NE)',
      hint: 'Standard WLS via Gain matrix G = HᵀWH. Chapter 2.6.',
    },
    {
      value: 'OrthogonalQR',
      label: 'Orthogonal QR (Givens)',
      hint: 'QR factorization of weighted Jacobian via Givens rotations. Avoids squaring κ(G). Chapter 3.2.',
    },
    {
      value: 'PetersWilkinson',
      label: 'Peters & Wilkinson',
      hint: 'LU decomposition of H̃; LᵀL is better conditioned than G. Chapter 3.4.',
    },
    {
      value: 'EqualityConstrained',
      label: 'Equality-Constrained',
      hint: 'Zero injections as explicit constraints via Lagrangian KKT system. Chapter 3.5.',
    },
    {
      value: 'FastDecoupled',
      label: 'Fast Decoupled (FD)',
      hint: 'P-θ / Q-V decoupled sub-problems with constant gain sub-matrices. Chapter 2.7.',
    },
    {
      value: 'DcEstimation',
      label: 'DC Estimation',
      hint: 'Linear model: active power only, flat voltages. Single solve, no iteration. Chapter 2.8.',
    },
  ];

  readonly factorizationOptions: { value: Factorization; label: string }[] = [
    { value: 'SparseCholesky', label: 'Sparse Cholesky (LLT)' },
    { value: 'SparseLU', label: 'Sparse LU' },
    { value: 'DenseCholesky', label: 'Dense Cholesky (LLT)' },
  ];

  get showFactorization(): boolean {
    return this.formulation === 'NormalEquations' || this.formulation === 'EqualityConstrained';
  }

  get showIterationParams(): boolean {
    return this.formulation !== 'DcEstimation';
  }

  onRun(): void {
    this.runEstimate.emit({
      factorization: this.factorization,
      formulation: this.formulation,
      max_iterations: this.maxIterations,
      tolerance: this.tolerance,
    });
  }
}
