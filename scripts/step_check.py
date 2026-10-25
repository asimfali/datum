"""STEP files read back by OpenCASCADE (D130): each gives valid solids whose volume, surface area
and centroid match the validation properties written in the file.

Usage: python -I scripts/step_check.py file.step...   (needs OCP: pip install cadquery-ocp)
"""
import re
import sys

from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer

# Relative agreement asked of OCCT's integrals: they converge far below it on exact geometry.
AGREE = 1e-6


def written(text, pattern):
    return re.search(pattern, text).group(1)


failed = 0
for path in sys.argv[1:]:
    text = open(path, encoding="ascii").read()
    volume = float(written(text, r"VOLUME_MEASURE\(([^)]*)\)"))
    area = float(written(text, r"AREA_MEASURE\(([^)]*)\)"))
    centroid = [float(x) for x in written(text, r"CARTESIAN_POINT\('centre point',\(([^)]*)\)\)").split(",")]
    reader = STEPControl_Reader()
    if reader.ReadFile(path) != IFSelect_RetDone:
        print(f"FAIL {path}: OCCT cannot read it")
        failed += 1
        continue
    reader.TransferRoots()
    shape = reader.OneShape()
    solids, ex = 0, TopExp_Explorer(shape, TopAbs_SOLID)
    while ex.More():
        solids, _ = solids + 1, ex.Next()
    valid = BRepCheck_Analyzer(shape).IsValid()
    vp, sp = GProp_GProps(), GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, vp)
    BRepGProp.SurfaceProperties_s(shape, sp)
    c = vp.CentreOfMass()
    dv, da = abs(vp.Mass() - volume) / volume, abs(sp.Mass() - area) / area
    dc = max(abs(a - b) for a, b in zip((c.X(), c.Y(), c.Z()), centroid)) / max(1.0, volume ** (1 / 3))
    ok = valid and solids > 0 and max(dv, da, dc) < AGREE
    failed += not ok
    print(f"{'ok  ' if ok else 'FAIL'} {path}: {solids} solid(s), valid {valid}, volume {dv:.1e}, area {da:.1e}, centroid {dc:.1e}")
sys.exit(1 if failed else 0)
