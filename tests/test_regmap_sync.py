import os, re
from iiot import vib_regmap as R

HDR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "firmware", "vibration-node", "include", "regmap.h")


def test_header_matches_python():
    defs = dict(re.findall(r"#define\s+((?:IR|HR)_\w+)\s+(\d+)", open(HDR).read()))
    assert defs, "no defines parsed"
    for name, val in defs.items():
        assert getattr(R, name) == int(val), name
    py = {k for k in dir(R) if re.match(r"(IR|HR)_[A-Z0-9_]+$", k) and k != "HR_DEFAULTS"}
    assert py == set(defs)
