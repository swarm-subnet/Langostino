#!/usr/bin/env python3
"""
Analyze GPS data from flight logs to assess quality and variability.
"""

import json
import numpy as np
from collections import defaultdict

def parse_gps_logs(jsonl_file):
    """Extract GPS data from JSONL flight log."""
    gps_data = []

    with open(jsonl_file, 'r') as f:
        for line in f:
            try:
                entry = json.loads(line)
                if entry.get('log_type') == 'FC_GPS':
                    data = entry.get('data', {})
                    gps_data.append({
                        'timestamp': entry['timestamp'],
                        'latitude': data.get('latitude'),
                        'longitude': data.get('longitude'),
                        'altitude': data.get('altitude'),
                        'fix_status': data.get('status', {}).get('status', -1)
                    })
            except json.JSONDecodeError:
                continue

    return gps_data

def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate distance in meters between two GPS coordinates."""
    from math import radians, sin, cos, sqrt, atan2

    R = 6371000  # Earth radius in meters

    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat/2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))

    return R * c

def analyze_gps_quality(gps_data):
    """Analyze GPS data quality and variability."""

    if len(gps_data) < 2:
        print("❌ Insufficient GPS data")
        return

    print("=" * 70)
    print("GPS DATA ANALYSIS")
    print("=" * 70)

    # Basic stats
    print(f"\n📊 DATASET OVERVIEW")
    print(f"   Total GPS samples: {len(gps_data)}")

    timestamps = [d['timestamp'] for d in gps_data]
    duration = timestamps[-1] - timestamps[0]
    print(f"   Duration: {duration:.1f} seconds")
    print(f"   Average sample rate: {len(gps_data)/duration:.2f} Hz")

    # Fix status
    fix_statuses = [d['fix_status'] for d in gps_data]
    print(f"\n🛰️  FIX STATUS")
    print(f"   Status 0 (NO_FIX): {fix_statuses.count(0)}")
    print(f"   Status 1 (2D FIX): {fix_statuses.count(1)}")
    print(f"   Status 2+ (3D FIX): {sum(1 for s in fix_statuses if s >= 2)}")

    # Position variability
    lats = np.array([d['latitude'] for d in gps_data])
    lons = np.array([d['longitude'] for d in gps_data])
    alts = np.array([d['altitude'] for d in gps_data])

    print(f"\n📍 POSITION STATISTICS")
    print(f"   Latitude:")
    print(f"      Mean: {np.mean(lats):.8f}°")
    print(f"      Std:  {np.std(lats):.8f}° ({np.std(lats) * 111320:.3f} m)")
    print(f"      Range: {np.max(lats) - np.min(lats):.8f}° ({(np.max(lats) - np.min(lats)) * 111320:.3f} m)")

    print(f"   Longitude:")
    print(f"      Mean: {np.mean(lons):.8f}°")
    print(f"      Std:  {np.std(lons):.8f}° ({np.std(lons) * 111320 * np.cos(np.radians(np.mean(lats))):.3f} m)")
    print(f"      Range: {np.max(lons) - np.min(lons):.8f}° ({(np.max(lons) - np.min(lons)) * 111320 * np.cos(np.radians(np.mean(lats))):.3f} m)")

    print(f"   Altitude:")
    print(f"      Mean: {np.mean(alts):.2f} m")
    print(f"      Std:  {np.std(alts):.3f} m")
    print(f"      Range: {np.max(alts) - np.min(alts):.2f} m")

    # Sample-to-sample variation
    distances = []
    for i in range(1, len(gps_data)):
        d = haversine_distance(
            gps_data[i-1]['latitude'], gps_data[i-1]['longitude'],
            gps_data[i]['latitude'], gps_data[i]['longitude']
        )
        distances.append(d)

    print(f"\n📏 SAMPLE-TO-SAMPLE MOVEMENT")
    print(f"   Mean distance: {np.mean(distances):.3f} m")
    print(f"   Std distance: {np.std(distances):.3f} m")
    print(f"   Max distance: {np.max(distances):.3f} m")
    print(f"   95th percentile: {np.percentile(distances, 95):.3f} m")

    # Initial period analysis (first 5 seconds for origin setting)
    initial_samples = [d for d in gps_data if d['timestamp'] - gps_data[0]['timestamp'] <= 5.0]
    if len(initial_samples) >= 2:
        init_lats = np.array([d['latitude'] for d in initial_samples])
        init_lons = np.array([d['longitude'] for d in initial_samples])
        init_alts = np.array([d['altitude'] for d in initial_samples])

        print(f"\n🎯 INITIAL 5 SECONDS (for origin averaging)")
        print(f"   Samples: {len(initial_samples)}")
        print(f"   Lat std: {np.std(init_lats) * 111320:.3f} m")
        print(f"   Lon std: {np.std(init_lons) * 111320 * np.cos(np.radians(np.mean(init_lats))):.3f} m")
        print(f"   Alt std: {np.std(init_alts):.3f} m")

        # Simulate averaging benefit
        avg_lat = np.mean(init_lats)
        avg_lon = np.mean(init_lons)
        avg_alt = np.mean(init_alts)

        single_sample_error = haversine_distance(
            init_lats[0], init_lons[0], avg_lat, avg_lon
        )
        print(f"   First sample vs average: {single_sample_error:.3f} m error")
        print(f"   ✅ Averaging reduces origin error by ~{single_sample_error:.2f}m")

    # Recommendations
    print(f"\n💡 RECOMMENDATIONS")
    horizontal_std = np.sqrt((np.std(lats) * 111320)**2 +
                              (np.std(lons) * 111320 * np.cos(np.radians(np.mean(lats))))**2)

    if horizontal_std > 2.0:
        print(f"   ⚠️  High GPS noise detected ({horizontal_std:.2f}m std)")
        print(f"   → Recommend averaging initial origin over 2-3 seconds")
        print(f"   → Consider EMA filtering (alpha=0.7) for live position")
    elif horizontal_std > 1.0:
        print(f"   ⚠️  Moderate GPS noise ({horizontal_std:.2f}m std)")
        print(f"   → Recommend averaging initial origin over 1-2 seconds")
    else:
        print(f"   ✅ Good GPS quality ({horizontal_std:.2f}m std)")
        print(f"   → Origin averaging still recommended (reduces error)")

    if all(s == 0 for s in fix_statuses):
        print(f"   ❌ NO GPS FIX detected!")
        print(f"   → CRITICAL: Wait for fix_status >= 2 before flight")

    print(f"\n   📌 SATELLITE COUNT DATA NOT AVAILABLE IN LOGS")
    print(f"   → Modify fc_comms_node to publish satellite count")
    print(f"   → Add check: require num_satellites >= 6")

    print("=" * 70)

if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1:
        logfile = sys.argv[1]
    else:
        logfile = "/Users/rafaeltorrecilla/Documents/swarm/swarm-ros/flight-logs/blackbox_20251007_165037_000.jsonl"

    print(f"📂 Loading: {logfile}\n")
    gps_data = parse_gps_logs(logfile)

    if gps_data:
        analyze_gps_quality(gps_data)
    else:
        print("❌ No GPS data found in log file")
