# Factory Layout and Production Flow

**Author:** Muskaan Maheshwari

## Overview

The factory simulates a realistic EV battery manufacturing facility with specialized production stations, traffic networks, and operational zones. The layout is designed for continuous material flow from raw components through assembly, integration, quality testing, to final shipping.

## Physical Dimensions

- **Total Width**: 100 meters (east-west)
- **Total Height**: 80 meters (north-south)
- **Grid Resolution**: 0.5 meters per cell
- **Total Area**: 8,000 m²

## Station Layout

```
NORTH (Y: 80m)
^
|     TESTING/QC AREA          SHIPPING AREA
|   ┌─────────────────┐      ┌──────────────┐
|   │  TEST_QC_1      │      │ SHIPPING_1   │
|   │  TEST_QC_2      │      │ SHIPPING_2   │
|   └─────────────────┘      └──────────────┘
|
|     ASSEMBLY STATIONS        INTEGRATION AREA
|   ┌─────────────────┐      ┌──────────────┐
|   │ CELL_ASSY_1     │      │ PACK_INT_1   │
|   │ CELL_ASSY_2     │      │ PACK_INT_2   │
|   │ MODULE_PACK_1   │      │ PACK_INT_3   │
|   │ MODULE_PACK_2   │      └──────────────┘
|   └─────────────────┘
|
|     INCOMING/RECEIVING      CHARGING & PARKING
|   ┌─────────────────┐      ┌──────────────┐
|   │ MATERIAL_IN_1   │      │ CHARGER_1    │
|   │ MATERIAL_IN_2   │      │ CHARGER_2    │
|   │ MATERIAL_IN_3   │      │ PARKING_BAY  │
|   └─────────────────┘      └──────────────┘
|
└──────────────────────────────────────────────> EAST (X: 100m)
WEST (X: 0m)
```

## Station Types and Operations

### Incoming Material Stations

**Locations**: (5m, 5m), (5m, 15m), (5m, 25m)

**Stations**: MATERIAL_IN_1, MATERIAL_IN_2, MATERIAL_IN_3

**Function**: Receive raw battery cells, modules, and pack components from suppliers.

**Characteristics**:
- High-volume receipt points
- 24/7 operation
- Manual receiving interface
- Staging area for incoming materials

**AMR Operations**: Load raw materials, transport to assembly (typical 200–400 kg payload, frequency: every 2–3 minutes)

### Cell Assembly Stations

**Locations**: (20m, 10m), (20m, 20m), (20m, 30m)

**Stations**: CELL_ASSEMBLY_1, CELL_ASSEMBLY_2, CELL_ASSEMBLY_3

**Function**: Assemble individual battery cells into grouped modules with electrical interconnects.

**Characteristics**:
- Precision workstations
- Automated assembly machinery
- ~15 minute cycle per batch
- Requires material input/output via AMR

**AMR Operations**: Pickup raw cells, deliver to assembly station at 0° heading, wait for assembly (~15 min), unload modules.

### Module Packing Stations

**Locations**: (40m, 10m), (40m, 20m), (40m, 30m)

**Stations**: MODULE_PACKING_1, MODULE_PACKING_2, MODULE_PACKING_3

**Function**: Pack assembled modules into thermal/structural cases with electronics.

**Characteristics**:
- Medium-speed assembly
- Thermal shrink wrapping
- ~12 minute cycle per unit

**AMR Operations**: Pickup modules (5m from assembly), transport to packing, wait (~12 min), unload packed module.

### Pack Integration Stations

**Locations**: (60m, 15m), (60m, 30m), (60m, 45m)

**Stations**: PACK_INTEGRATION_1, PACK_INTEGRATION_2, PACK_INTEGRATION_3

**Function**: Integrate multiple packed modules into complete battery packs with BMS (Battery Management System).

**Characteristics**:
- High-value assembly step
- BMS programming and electrical integration
- ~20 minute cycle per pack

**AMR Operations**: Pickup packed modules, transport (20m distance), wait for integration (~20 min), deliver completed pack.

### Testing & Quality Control

**Locations**: (75m, 10m), (75m, 25m), (75m, 40m)

**Stations**: TESTING_QC_1, TESTING_QC_2, TESTING_QC_3

**Function**: High-voltage testing, capacity verification, and 100% quality assurance.

**Characteristics**:
- Automated test equipment
- High-voltage isolation chambers
- ~8 minute test cycle
- Pass/Fail sorting

**AMR Operations**: Pickup integrated packs, transport to QC, wait for testing (~8 min), route passed packs to shipping or rework.

### Shipping/Dispatch

**Locations**: (85m, 10m), (85m, 25m)

**Stations**: SHIPPING_1, SHIPPING_2

**Function**: Stage completed, tested packs for shipment to vehicle assembly plants.

**Characteristics**:
- Outbound staging area
- Pallet aggregation
- Truck loading interface
- ~30 minute hold time per batch

**AMR Operations**: Pickup tested packs, transport to shipping dock, queue for truck loading, return to parking/charging.

### Charging Stations

**Locations**: (10m, 70m), (30m, 70m), (50m, 70m)

**Stations**: CHARGER_1, CHARGER_2, CHARGER_3

**Function**: Recharge AMR batteries during operation.

**Characteristics**:
- 48V lithium fast chargers
- 45-minute charge cycle (20% to 100%)
- Automatic contact alignment via docking

**AMR Operations**: Navigate to charger when battery < 30%, dock with turret alignment, wait for charging (~30 min typical), auto-release at 80% charge, return to task queue.

### Parking Bay

**Location**: (70m, 70m)

**Station**: PARKING_BAY

**Function**: Staging area for idle robots during low-demand periods.

**Characteristics**:
- Capacity: 10+ robot positions
- Grid spacing: 2m per position
- Idle battery discharge: minimal (~0.1%/hour)

**AMR Operations**: Park when no tasks available, quick availability for new tasks.

## Production Pipeline

### Primary Flow

```
INCOMING_MATERIAL (manual receive, 10 min)
        ↓
        └─→ [AMR Transport 1] (5 min)
                ↓
        CELL_ASSEMBLY (15 min automated)
                ↓
        [AMR Transport 2] (5 min, 5m distance)
                ↓
        MODULE_PACKING (12 min automated)
                ↓
        [AMR Transport 3] (8 min, 20m distance)
                ↓
        PACK_INTEGRATION (20 min automated)
                ↓
        [AMR Transport 4] (6 min, 15m distance)
                ↓
        TESTING_QC (8 min automated)
                ↓
        [Decision: PASS / FAIL]
                ├─→ PASS: [AMR Transport 5] → SHIPPING
                │          (8 min, 15m distance)
                │
                └─→ FAIL: [Rework/Scrap handling]
```

### Total Production Time

| Stage | Duration | Notes |
|-------|----------|-------|
| Receiving/Staging | 10 min | Manual, external |
| Cell Assembly | 15 min | Automated |
| Transport (assembly) | 5 min | AMR (5m) |
| Module Packing | 12 min | Automated |
| Transport (packing) | 8 min | AMR (20m) |
| Pack Integration | 20 min | Automated BMS |
| Transport (integration) | 6 min | AMR (15m) |
| Testing/QC | 8 min | Automated |
| Transport (shipping) | 8 min | AMR (15m) |
| Shipping Stage | 30 min | Manual truck load |
| **Total Cycle** | **~127 min** | **~2.1 hours** |

**Throughput**: ~14 packs/shift (8-hour operation, assuming single shift)

## Factory Zones

Factory operations are divided into typed zones with different speed limits and safety controls:

```
┌─────────────────────────────────────┐
│      RESTRICTED ZONE (0.2 m/s)     │
│  (High-voltage testing, safety)     │
│  ┌───────────────────────────────┐  │
│  │  PRODUCTION: TESTING_QC       │  │
│  │  Hazardous materials, barrier │  │
│  └───────────────────────────────┘  │
└─────────────────────────────────────┘

┌──────────────────┐  ┌──────────────────┐
│  PRODUCTION ZONE │  │  PRODUCTION ZONE │
│  (Cell Assembly) │  │ (Module Packing) │
│  Speed: 0.5 m/s │  │  Speed: 0.5 m/s  │
└────────┬─────────┘  └────────┬─────────┘
         │                     │
    ┌────┴─────────────────────┴────┐
    │    AISLE ZONE (1.5 m/s)       │
    │  Main traffic arteries        │
    │  Double-wide (2m)             │
    └────┬────────────────┬─────────┘
         │                │
    ┌────┴──────┐   ┌─────┴──────┐
    │INTERSECTION│   │INTERSECTION│
    │(0.5 m/s)  │   │ (0.5 m/s)  │
    │Radius 3.0m│   │Radius 3.0m │
    └────────────┘   └────────────┘

┌──────────────────────────────────┐
│   BUFFER ZONE (1.0 m/s)          │
│  Safety margins around stations  │
│  Width: 1.0m                     │
└──────────────────────────────────┘
```

### Zone Types

| Zone | Speed Limit | Characteristics | Safety Features |
|------|-------------|-----------------|-----------------|
| **PRODUCTION** | 0.5 m/s | Assembly stations, workers | Speed governors, interlocks |
| **AISLE** | 1.5 m/s | Main traffic routes | Lane markings, double width |
| **INTERSECTION** | 0.5 m/s | Converging aisles | Reservation system, priority |
| **RESTRICTED** | 0.2 m/s | High-voltage areas | Emergency stop, barriers |
| **BUFFER** | 1.0 m/s | Safety margins | Collision detection, warnings |

## Traffic Network

### Aisle Grid

The factory uses a 3×3 grid of intersections spaced 30 meters apart:

```
        0,80   10,80   20,80   30,80
         N00    N01    N02    N03
        ┌────┬────┬────┬────┐
        │    │    │    │    │
        ├────┼────┼────┼────┤
 0,60   │ N10│ N11│ N12│ N13│
        ├────┼────┼────┼────┤
        │ N20│ N21│ N22│ N23│
        │    │    │    │    │
        ├────┼────┼────┼────┤
        │ N30│ N31│ N32│ N33│
        └────┴────┴────┴────┘
     0,0            30,0 →E
```

**Specifications**:
- **Intersections**: 9 (3×3 grid)
- **Aisle Waypoints**: 12 (linear segments)
- **Aisles**: 2m width
- **Safe turning radius**: 2m (enforced via Dubins curves)
- **Spacing**: 30m between intersections

## Obstacle and Collision Model

### Static Obstacles

- Station boundaries (4m perimeter around each station)
- Wall obstacles (1m width perimeter walls)
- Restricted zone barriers (high-voltage test area)
- Loading dock edges

### Safety Margins

| Hazard | Minimum Distance |
|--------|-----------------|
| Robot-to-robot | 1.5 m |
| Robot-to-obstacle | 0.5 m |
| Robot-to-human | 1.0 m (critical safety) |

### Collision Detection

- Circle-based safety zones (primary)
- Footprint-based checking (secondary)
- Proximity warning zones (1.5m buffer)

## Expected Performance

### Throughput

**Configuration**: 5 robots, 3 assembly stations, 3 packing, 3 integration

- **Packs per hour**: 12–15 (high demand), 6–10 (normal)
- **Robot utilization**: 60–80%
- **Station utilization**: 85–95%
- **Average task time**: 25–30 minutes per pack

### Bottlenecks (in order of impact)

1. **Shipping/Staging** (30 min) — Manual truck loading
2. **Pack Integration** (20 min) — Precision assembly bottleneck
3. **Cell Assembly** (15 min) — High-demand bottleneck
4. **QC Testing** (8 min) — Quality gate
5. **Transport** (2–8 min) — Distance-dependent

### Optimization Opportunities

- **Increase robot count**: +2 robots = +30% throughput
- **Reduce integration time**: 20 min → 15 min = +15% throughput
- **Parallel QC testing**: Add 2nd test bay = +25% throughput
- **Improve shipping**: 30 min → 15 min = +20% throughput

## Extension Points

### Adding New Stations

1. Define station location and type in `FactoryEnvironment`
2. Add to production flow pipeline if mandatory
3. Configure approach point and docking orientation
4. Register with fleet coordinator task assignment
5. Update path planner waypoint graph if needed

### Modifying Zone Restrictions

1. Update `FactoryZone` enum for new zone types
2. Modify speed governors in traffic manager
3. Adjust collision detection safety radius
4. Update RViz visualization config

### Multi-Floor Scaling

1. Add Z-axis (elevation) to coordinate system
2. Create elevator queue system
3. Separate traffic management per floor
4. Extend path planner for 3D navigation
5. Update battery discharge model for elevation

## References

- Factory Layout: Based on real automotive battery manufacturing designs
- Production Times: Estimated from Tesla, LG, CATL published specifications
- Safety Standards: ANSI B56.5 (AMR Safety Standard)
- Traffic Control: Vehicular intersection models adapted for AMR
