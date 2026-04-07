# Quick Start: Running Visualization Demos

Fast reference guide for generating the 3 new advanced visualizations.

## One-Command Execution

```bash
# Run all 3 new visualization demos
python3 main.py --demo navigation
python3 main.py --demo obstacles
python3 main.py --demo alerts
```

## Individual Demos

### 1. Navigation Demo (Dubins Path Planning)
```bash
python3 main.py --demo navigation
```
**Output:** `docs/images/navigation_demo.png` (203 KB)
- 2×3 grid of 6 robot progress frames
- Dubins curve path visualization
- Real-time distance and heading info

### 2. Obstacle Avoidance Demo (DWA Algorithm)
```bash
python3 main.py --demo obstacles
```
**Output:** `docs/images/obstacle_avoidance_demo.png` (229 KB)
- 2×2 grid showing 4-frame avoidance sequence
- Emergency stop response
- Safety zone visualization

### 3. Alert System Demo (Fleet Coordination)
```bash
python3 main.py --demo alerts
```
**Output:** `docs/images/alert_demo.png` (210 KB)
- 3-robot fleet coordination
- Alert timeline visualization
- Dynamic path rerouting

## Generate All Visualizations

```bash
# Run all 9 demos (original + new)
python3 main.py --demo all

# Or just the original 6 demos
python3 main.py --demo factory
python3 main.py --demo amr
python3 main.py --demo dubins
python3 main.py --demo docking
python3 main.py --demo traffic
python3 main.py --demo full
```

## Direct Script Execution

For headless batch processing:

```bash
python3 src/demos/navigation_demo.py
python3 src/demos/obstacle_avoidance_demo.py
python3 src/demos/alert_demo.py
```

## Viewing Results

All images are saved to: `docs/images/`

```bash
# List all generated images
ls -lh docs/images/

# View an image (on systems with image viewer)
xdg-open docs/images/navigation_demo.png      # Linux
open docs/images/navigation_demo.png          # macOS
start docs/images/navigation_demo.png         # Windows
```

## Help Text

```bash
python3 main.py --help
```

Output includes:
- All available demo options
- Parameter descriptions
- Example commands
- New visualization demos

## Requirements

- Python 3.8+
- matplotlib 3.5+
- numpy 1.20+

Check installation:
```bash
python3 -c "import matplotlib; import numpy; print('OK')"
```

## Image Gallery

See `docs/VISUALIZATIONS.md` for detailed descriptions:
- Technical specifications
- Algorithm explanations
- Portfolio recommendations
- Usage guidelines

## Troubleshooting

**Issue:** Missing images directory
```bash
mkdir -p docs/images
```

**Issue:** Import errors
```bash
pip install -r requirements.txt
```

**Issue:** Slow execution
- Demos are headless (no display needed)
- First run caches Python bytecode
- Subsequent runs are faster

## Execution Times

Expected runtime:
- Navigation demo: ~2 seconds
- Obstacle avoidance demo: ~3 seconds
- Alert system demo: ~4 seconds
- All original demos: ~30 seconds

Total for all 9 demos: ~40 seconds

## CI/CD Integration

Add to your CI pipeline:

```bash
# Generate all visualizations for documentation
python3 main.py --demo navigation
python3 main.py --demo obstacles
python3 main.py --demo alerts

# Copy to documentation folder
cp docs/images/*.png docs/generated/
```

## Next Steps

1. Review generated images in `docs/images/`
2. Read detailed guide in `docs/VISUALIZATIONS.md`
3. View implementation notes in `DEMO_IMPLEMENTATION_SUMMARY.md`
4. Use images in your portfolio/presentations

---

**Last Updated:** 2026-04-07
**All demos tested and working:** ✓
