"""Make the exporter importable outside Fusion: stub the adsk modules and put the
repo root on sys.path so Fusion_URDF_Exporter_ROS2 imports as a namespace package."""
import sys
import types
from pathlib import Path

adsk = types.ModuleType('adsk')
adsk.core = types.ModuleType('adsk.core')
adsk.fusion = types.ModuleType('adsk.fusion')
adsk.fusion.CalculationAccuracy = types.SimpleNamespace(VeryHighCalculationAccuracy=3)
adsk.fusion.JointOrigin = type('JointOrigin', (), {})
sys.modules.update({'adsk': adsk, 'adsk.core': adsk.core, 'adsk.fusion': adsk.fusion})

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
