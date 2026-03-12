import { Component, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import { MatIcon } from '@angular/material/icon';
import { MatTabGroup, MatTab } from '@angular/material/tabs';
import { MatSnackBar, MatSnackBarModule } from '@angular/material/snack-bar';

import { environment } from '../environments/environment';
import { ApiService } from './services/api.service';
import {
  EstimateRequest,
  NetworkData,
  SeResult,
  RedundancyResult,
} from './models/network.model';
import { GridViewComponent } from './components/grid-view/grid-view.component';
import { ConfigPanelComponent } from './components/config-panel/config-panel.component';
import { SummaryCardComponent } from './components/summary-card/summary-card.component';
import { ResultsTableComponent } from './components/results-table/results-table.component';
import { RedundancyPanelComponent } from './components/redundancy-panel/redundancy-panel.component';

@Component({
  selector: 'app-root',
  standalone: true,
  imports: [
    CommonModule,
    MatIcon,
    MatTabGroup,
    MatTab,
    MatSnackBarModule,
    GridViewComponent,
    ConfigPanelComponent,
    SummaryCardComponent,
    ResultsTableComponent,
    RedundancyPanelComponent,
  ],
  templateUrl: './app.component.html',
  styleUrl: './app.component.scss',
})
export class AppComponent implements OnInit {
  readonly isPro = environment.pro;
  networkData: NetworkData | null = null;
  seResult: SeResult | null = null;
  redundancyResult: RedundancyResult | null = null;
  running = false;
  loadingRedundancy = false;

  constructor(
    private api: ApiService,
    private snackBar: MatSnackBar,
  ) {}

  ngOnInit(): void {
    this.api.getNetwork().subscribe({
      next: data => {
        this.networkData = data;
        this.snackBar.open(
          `Loaded: ${data.n_buses} buses, ${data.n_branches} branches`,
          'OK', { duration: 3000 },
        );
      },
      error: err => {
        this.snackBar.open(
          'Failed to connect to API server. Is it running on port 3001?',
          'Dismiss', { duration: 8000 },
        );
        console.error(err);
      },
    });
  }

  onRunEstimate(req: EstimateRequest): void {
    this.running = true;
    this.seResult = null;
    this.redundancyResult = null;
    this.api.runEstimate(req).subscribe({
      next: result => {
        this.seResult = result;
        this.running = false;
        this.snackBar.open(
          result.converged
            ? `Converged in ${result.iterations} iters (${result.se_time_seconds.toFixed(3)}s)`
            : 'SE did NOT converge!',
          'OK', { duration: 5000 },
        );
        if (result.converged && this.isPro) {
          this.fetchRedundancy();
        }
      },
      error: err => {
        this.running = false;
        this.snackBar.open(`SE failed: ${err.message || err.statusText}`, 'Dismiss', { duration: 8000 });
        console.error(err);
      },
    });
  }

  private fetchRedundancy(): void {
    this.loadingRedundancy = true;
    this.api.runRedundancy().subscribe({
      next: result => {
        this.redundancyResult = result;
        this.loadingRedundancy = false;
        this.snackBar.open(
          `Redundancy: ${result.n_critical} critical, ${result.n_simply_redundant} M₁, ${result.n_multiply_redundant} M₂`,
          'OK', { duration: 4000 },
        );
      },
      error: err => {
        this.loadingRedundancy = false;
        console.error('Redundancy analysis failed:', err);
      },
    });
  }
}
