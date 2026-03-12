import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable } from 'rxjs';
import {
  EstimateRequest,
  NetworkData,
  SeResult,
  RedundancyRequest,
  RedundancyResult,
  MeasurementExport,
} from '../models/network.model';

@Injectable({ providedIn: 'root' })
export class ApiService {
  private readonly base = '/api';

  constructor(private http: HttpClient) {}

  getNetwork(): Observable<NetworkData> {
    return this.http.get<NetworkData>(`${this.base}/network`);
  }

  runEstimate(req: EstimateRequest): Observable<SeResult> {
    return this.http.post<SeResult>(`${this.base}/estimate`, req);
  }

  runRedundancy(req: RedundancyRequest = {}): Observable<RedundancyResult> {
    return this.http.post<RedundancyResult>(`${this.base}/redundancy`, req);
  }

  getMeasurementExport(): Observable<MeasurementExport[]> {
    return this.http.get<MeasurementExport[]>(`${this.base}/measurements`);
  }
}
