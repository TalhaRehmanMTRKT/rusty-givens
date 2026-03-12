import {
  AfterViewInit, Component, ElementRef, Input, OnChanges, OnDestroy,
  SimpleChanges, ViewChild,
} from '@angular/core';
import { CommonModule } from '@angular/common';
import { NetworkData, BusNode, BusResult } from '../../models/network.model';
import * as L from 'leaflet';

const HV_THRESHOLD = 275;

/** Palette from Cyber Guard / index.html */
const PALETTE = {
  blue:       '#003B5C',
  blueMid:    '#004e7c',
  graphite:   '#2C3E50',
  graphiteLt: '#3d5166',
  orange:     '#E67E22',
  orangeLt:   '#f0944d',
  silver:     '#ECF0F1',
  silverDk:   '#d5dde0',
  textMid:    '#4a6070',
  white:      '#FFFFFF',
};

@Component({
  selector: 'app-grid-view',
  standalone: true,
  imports: [CommonModule],
  templateUrl: './grid-view.component.html',
  styleUrl: './grid-view.component.scss',
})
export class GridViewComponent implements AfterViewInit, OnChanges, OnDestroy {
  @ViewChild('mapContainer', { static: true }) container!: ElementRef;
  @Input() networkData: NetworkData | null = null;
  @Input() busResults: BusResult[] | null = null;

  visibleBusCount = 0;
  visibleBranchCount = 0;

  private map: L.Map | null = null;
  private busLayer = L.layerGroup();
  private branchLayer = L.layerGroup();
  private busMarkers = new Map<number, L.CircleMarker>();
  private initialized = false;
  private pendingData = false;
  private visibleBusIndices = new Set<number>();

  ngAfterViewInit(): void {
    this.initMap();
    this.initialized = true;
    if (this.pendingData) {
      this.buildLayers();
      this.pendingData = false;
    }
  }

  ngOnChanges(changes: SimpleChanges): void {
    if (changes['networkData'] && this.networkData) {
      if (this.initialized) {
        this.buildLayers();
      } else {
        this.pendingData = true;
      }
    }
    if (changes['busResults']) {
      this.colorByResults();
    }
  }

  ngOnDestroy(): void {
    this.map?.remove();
  }

  private initMap(): void {
    this.map = L.map(this.container.nativeElement, {
      crs: L.CRS.Simple,
      zoomControl: true,
      attributionControl: false,
      zoomSnap: 0.25,
      zoomDelta: 0.5,
      minZoom: -2,
      maxZoom: 6,
    });

    this.branchLayer.addTo(this.map);
    this.busLayer.addTo(this.map);

    L.control.attribution({ prefix: 'GB Transmission Network · Leaflet' }).addTo(this.map);
  }

  private buildLayers(): void {
    if (!this.networkData || !this.map) return;

    this.busLayer.clearLayers();
    this.branchLayer.clearLayers();
    this.busMarkers.clear();

    const hvBuses = this.networkData.buses.filter(
      b => b.vn_kv >= HV_THRESHOLD && b.geo_x != null && b.geo_y != null
    );
    this.visibleBusIndices = new Set(hvBuses.map(b => b.index));
    this.visibleBusCount = hvBuses.length;

    const busPositions = new Map<number, L.LatLngExpression>();
    for (const b of hvBuses) {
      const pos: L.LatLngExpression = [b.geo_y!, b.geo_x!];
      busPositions.set(b.index, pos);
    }

    // Branches (drawn first so buses render on top)
    const hvSet = this.visibleBusIndices;
    let branchCount = 0;
    for (const br of this.networkData.branches) {
      if (!hvSet.has(br.from_bus) || !hvSet.has(br.to_bus)) continue;
      const from = busPositions.get(br.from_bus);
      const to = busPositions.get(br.to_bus);
      if (!from || !to) continue;

      L.polyline([from, to], {
        color: PALETTE.textMid,
        weight: 1.4,
        opacity: 0.5,
      }).addTo(this.branchLayer);
      branchCount++;
    }
    this.visibleBranchCount = branchCount;

    // Bus markers
    for (const b of hvBuses) {
      const pos = busPositions.get(b.index)!;
      const is400 = b.vn_kv >= 350;
      const isSlack = b.bus_type === 3;
      const isPV = b.bus_type === 2;

      const radius = isSlack ? 7 : isPV ? 4.5 : 3;
      const fillColor = isSlack ? PALETTE.orange
        : is400 ? PALETTE.blueMid : PALETTE.graphiteLt;
      const strokeColor = isSlack ? PALETTE.blue
        : is400 ? PALETTE.silver : PALETTE.silverDk;
      const strokeWeight = isSlack ? 2.5 : 1.2;

      const marker = L.circleMarker(pos, {
        radius,
        fillColor,
        fillOpacity: 0.95,
        color: strokeColor,
        weight: strokeWeight,
      });

      const busTypeLabel = isSlack ? 'Slack' : isPV ? 'PV' : 'PQ';
      marker.bindTooltip(
        `<b>Bus ${b.label}</b> (${b.vn_kv} kV)<br>Type: ${busTypeLabel}`,
        { className: 'gis-tooltip', direction: 'top', offset: [0, -6] },
      );

      marker.addTo(this.busLayer);
      this.busMarkers.set(b.index, marker);
    }

    // Fit map bounds with padding
    if (hvBuses.length > 0) {
      const lats = hvBuses.map(b => b.geo_y!);
      const lngs = hvBuses.map(b => b.geo_x!);
      const bounds = L.latLngBounds(
        [Math.min(...lats), Math.min(...lngs)],
        [Math.max(...lats), Math.max(...lngs)],
      );
      this.map.fitBounds(bounds, { padding: [30, 30], animate: true });
    }
  }

  private colorByResults(): void {
    if (!this.busResults || this.busResults.length === 0) return;

    const visible = this.busResults.filter(b => this.visibleBusIndices.has(b.index));
    if (visible.length === 0) return;

    const maxErr = Math.max(...visible.map(b => b.vm_error));

    for (const b of visible) {
      const marker = this.busMarkers.get(b.index);
      if (!marker) continue;

      const ratio = maxErr > 0 ? b.vm_error / maxErr : 0;
      const r = Math.round(230 * ratio + 30 * (1 - ratio));
      const g = Math.round(180 * (1 - ratio) + 60 * ratio);
      const bl = 34 + Math.round(50 * (1 - ratio));

      marker.setStyle({
        fillColor: `rgb(${r},${g},${bl})`,
        fillOpacity: 0.95,
        color: PALETTE.silver,
        weight: 1.5,
      });

      marker.unbindTooltip();
      marker.bindTooltip(
        `<b>Bus ${b.label}</b><br>` +
        `VM: ${b.est_vm.toFixed(4)} p.u.<br>` +
        `VA: ${b.est_va_deg.toFixed(2)}°<br>` +
        `VM err: ${b.vm_error.toExponential(2)} p.u.<br>` +
        `VA err: ${b.va_error_deg.toFixed(4)}°`,
        { className: 'gis-tooltip', direction: 'top', offset: [0, -6] },
      );
    }
  }
}
