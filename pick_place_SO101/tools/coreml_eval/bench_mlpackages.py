"""Time every .mlpackage in a directory with random inputs on each compute-unit setting.
    python -m pick_place_SO101.tools.coreml_eval.bench_mlpackages <dir> [--n 20]
Dummy inputs => latency only (not quality). Shapes/dtypes are read from the model spec."""
import argparse, glob, os, time
import numpy as np
import coremltools as ct

UNITS = {"ALL": ct.ComputeUnit.ALL, "CPU_AND_GPU": ct.ComputeUnit.CPU_AND_GPU,
         "CPU_AND_NE": ct.ComputeUnit.CPU_AND_NE, "CPU_ONLY": ct.ComputeUnit.CPU_ONLY}


def dummy(desc):
    if desc.type.WhichOneof("Type") == "imageType":
        from PIL import Image
        it = desc.type.imageType
        return Image.fromarray(np.random.randint(0, 255, (it.height, it.width, 3), np.uint8))
    t = desc.type.multiArrayType
    shape = [int(s) for s in t.shape]
    dt = {65600: np.float32, 65568: np.float32, 65552: np.float16, 131104: np.int32}.get(t.dataType, np.float32)
    return (np.random.rand(*shape).astype(dt) if dt != np.int32 else np.zeros(shape, dt))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("dir"); ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--units", default="ALL,CPU_AND_GPU,CPU_AND_NE"); a = ap.parse_args()
    for pkg in sorted(glob.glob(os.path.join(a.dir, "**", "*.mlpackage"), recursive=True)):
        name = os.path.basename(pkg)
        for u in a.units.split(","):
            try:
                t0 = time.time()
                m = ct.models.MLModel(pkg, compute_units=UNITS[u])
                load = time.time() - t0
                spec = m.get_spec()
                x = {i.name: dummy(i) for i in spec.description.input}
                m.predict(x)                       # warm-up (first call compiles for the unit)
                ts = []
                for _ in range(a.n):
                    t1 = time.perf_counter(); m.predict(x); ts.append((time.perf_counter() - t1) * 1000)
                print(f"{name:42s} {u:12s} load {load:5.1f}s  median {np.median(ts):7.1f} ms  p90 {np.percentile(ts,90):7.1f} ms", flush=True)
            except Exception as e:
                print(f"{name:42s} {u:12s} FAILED {type(e).__name__}: {str(e)[:100]}", flush=True)


if __name__ == "__main__":
    main()
