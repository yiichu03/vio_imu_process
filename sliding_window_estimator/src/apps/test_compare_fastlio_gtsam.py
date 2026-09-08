#!/usr/bin/env python3
"""Black-box regression for comparator pass/failure/invalid-input handling.

Fixtures are synthetic parser-test inputs, NOT FAST-LIO experiment evidence.
Run: python3 test_compare_fastlio_gtsam.py /path/to/compare_fastlio_gtsam
"""
import pathlib
import subprocess
import sys
import tempfile


def block(name, rows, cols, identity=False):
    lines = [f"{name} ({rows}x{cols})"]
    for r in range(rows):
        lines.append(" ".join("1" if identity and r == c else "0"
                              for c in range(cols)))
    return "\n".join(lines) + "\n\n"


def pack(fastlio):
    tag = "fastlio" if fastlio else "gtsam"
    sigma = "Sigma_z_fastlio_gtsam" if fastlio else "Sigma_z_gtsam"
    return (block(sigma, 15, 15, True)
            + block("JincBias_ba_bg_" + tag, 9, 6)
            + block("dR_" + tag, 3, 3, True)
            + block("dP_" + tag, 3, 1)
            + block("dV_" + tag, 3, 1)
            + block("DT_" + tag, 1, 1, True))


def main():
    binary = str(pathlib.Path(sys.argv[1]).resolve())
    fast, reference = pack(True), pack(False)
    first_row = "1 " + "0 " * 13 + "0"
    mismatch = fast.replace(first_row, "2 " + "0 " * 13 + "0", 1)
    cases = [
        ("identical synthetic matrices", fast, reference, [], 0),
        ("changed covariance", mismatch, reference, [], 1),
        ("missing bias block", fast.replace("JincBias_ba_bg_fastlio", "unknown_bias"), reference, [], 1),
        ("wrong expected dimensions", fast.replace(block("JincBias_ba_bg_fastlio", 9, 6),
                                                  block("JincBias_ba_bg_fastlio", 1, 1)), reference, [], 1),
        ("NaN", fast.replace(first_row, first_row.replace("1", "nan", 1), 1), reference, [], 1),
        ("infinity", fast.replace(first_row, first_row.replace("1", "inf", 1), 1), reference, [], 1),
        ("duplicate block", fast + block("DT_fastlio", 1, 1, True), reference, [], 1),
        ("truncated input", fast[:50], reference, [], 1),
        ("extra row tokens", fast.replace(first_row, first_row + " 0", 1), reference, [], 1),
        ("negative tolerance", fast, reference, ["--rel_tol", "-1"], 1),
        ("nonfinite tolerance", fast, reference, ["--abs_tol", "nan"], 1),
        ("zero interval", fast.replace("DT_fastlio (1x1)\n1", "DT_fastlio (1x1)\n0"), reference, [], 1),
        ("different interval", fast.replace("DT_fastlio (1x1)\n1", "DT_fastlio (1x1)\n2"), reference, [], 1),
        ("invalid rotation", fast.replace("dR_fastlio (3x3)\n1", "dR_fastlio (3x3)\n2"), reference, [], 1),
    ]
    with tempfile.TemporaryDirectory(prefix="fastlio-comparator-test-") as tmp:
        fpath, rpath = pathlib.Path(tmp) / "fast.txt", pathlib.Path(tmp) / "ref.txt"
        for name, ftext, rtext, args, expected in cases:
            fpath.write_text(ftext)
            rpath.write_text(rtext)
            result = subprocess.run([binary, "--fastlio_all", str(fpath), "--gtsam_all", str(rpath)] + args,
                                    text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            if result.returncode != expected:
                print(f"[FAIL] {name}: expected exit {expected}, got {result.returncode}\n{result.stdout}")
                return 1
            print(f"[ OK ] {name}: exit {expected}")
    print("[ OK ] All 14 comparator control tests (synthetic, not algorithm validation)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
