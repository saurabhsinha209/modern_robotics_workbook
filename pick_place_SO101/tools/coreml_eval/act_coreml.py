"""Random-weight ACT (lerobot 0.6.0) at the real input shape -> Core ML, latency per compute unit.
Weights are random: this measures conversion feasibility and latency only, not behaviour.
    python -m pick_place_SO101.tools.coreml_eval.act_coreml [--size 224] [--chunk 50] [--adim 7] [--out DIR]
"""
import argparse, os, time
import numpy as np, torch
import coremltools as ct
from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACT

UNITS = {"ALL": ct.ComputeUnit.ALL, "CPU_AND_GPU": ct.ComputeUnit.CPU_AND_GPU,
         "CPU_AND_NE": ct.ComputeUnit.CPU_AND_NE, "CPU_ONLY": ct.ComputeUnit.CPU_ONLY}


class Wrap(torch.nn.Module):
    def __init__(self, act): super().__init__(); self.act = act
    def forward(self, bench, wrist, state):
        out = self.act({"observation.state": state, "observation.images": [bench, wrist]})
        return out[0]                                    # (B, chunk, adim)


def bench(f, n=30, warm=5):
    for _ in range(warm): f()
    ts = []
    for _ in range(n):
        t = time.perf_counter(); f(); ts.append((time.perf_counter() - t) * 1e3)
    return np.median(ts), np.percentile(ts, 90)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--size", type=int, default=224); ap.add_argument("--chunk", type=int, default=50)
    ap.add_argument("--adim", type=int, default=7); ap.add_argument("--out", default="/tmp/act_random.mlpackage"); a = ap.parse_args()
    S = a.size
    cfg = ACTConfig(chunk_size=a.chunk, n_action_steps=a.chunk, pretrained_backbone_weights=None, device="cpu",
                    input_features={"observation.state": PolicyFeature(FeatureType.STATE, (6,)),
                                    "observation.images.bench": PolicyFeature(FeatureType.VISUAL, (3, S, S)),
                                    "observation.images.wrist": PolicyFeature(FeatureType.VISUAL, (3, S, S))},
                    output_features={"action": PolicyFeature(FeatureType.ACTION, (a.adim,))})
    act = ACT(cfg).eval()
    print("params (M):", sum(p.numel() for p in act.parameters()) / 1e6)
    m = Wrap(act).eval()
    ex = (torch.rand(1, 3, S, S), torch.rand(1, 3, S, S), torch.rand(1, 6))
    with torch.no_grad():
        ref = m(*ex); print("torch out", tuple(ref.shape))
        tr = torch.jit.trace(m, ex, check_trace=False)
    t0 = time.time()
    ml = ct.convert(tr, convert_to="mlprogram", compute_precision=ct.precision.FLOAT16, minimum_deployment_target=ct.target.macOS14,
                    inputs=[ct.TensorType(name="bench", shape=ex[0].shape), ct.TensorType(name="wrist", shape=ex[1].shape),
                            ct.TensorType(name="robot_q", shape=ex[2].shape)])
    print(f"converted in {time.time() - t0:.0f}s"); ml.save(a.out)
    xin = {"bench": ex[0].numpy(), "wrist": ex[1].numpy(), "robot_q": ex[2].numpy()}
    # parity with torch (fp16 vs fp32)
    pm = ct.models.MLModel(a.out, compute_units=ct.ComputeUnit.CPU_ONLY)
    y = list(pm.predict(xin).values())[0]
    print("parity max|diff| vs torch fp32:", float(np.abs(y - ref.numpy()).max()), " (output scale", float(np.abs(ref.numpy()).mean()), ")")
    for u, cu in UNITS.items():
        try:
            mm = ct.models.MLModel(a.out, compute_units=cu)
            med, p90 = bench(lambda: mm.predict(xin))
            print(f"Core ML {u:12s} median {med:6.1f} ms  p90 {p90:6.1f} ms", flush=True)
        except Exception as e:
            print(f"Core ML {u:12s} FAILED {type(e).__name__}: {str(e)[:120]}")
    if torch.backends.mps.is_available():
        mps = m.to("mps"); exm = [x.to("mps") for x in ex]
        def f():
            with torch.no_grad(): mps(*exm); torch.mps.synchronize()
        print("PyTorch MPS  median %.1f ms  p90 %.1f ms" % bench(f))
    cpu = m.to("cpu")
    print("PyTorch CPU  median %.1f ms  p90 %.1f ms" % bench(lambda: cpu(*ex), n=10, warm=2))


if __name__ == "__main__":
    main()
