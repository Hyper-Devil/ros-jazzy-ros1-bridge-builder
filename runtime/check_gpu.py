#!/usr/bin/env python3
"""Exercise Blackwell kernels, not just torch.cuda.is_available()."""
import json
import subprocess
import cupy as cp
import torch

assert torch.cuda.is_available()
x = torch.ones((32, 32), device='cuda')
assert float((x@x).mean()) == 32.
kernel = cp.ElementwiseKernel('float32 x', 'float32 y', 'y = 2*x', 'robotics_gpu_probe')
assert float(kernel(cp.ones(128, dtype=cp.float32)).mean()) == 2.
torch.cuda.synchronize()
print(json.dumps(dict(torch=torch.__version__, torch_cuda=torch.version.cuda,
                      cupy=cp.__version__, cupy_cuda=cp.cuda.runtime.runtimeGetVersion(),
                      gpu=torch.cuda.get_device_name(), architecture=torch.cuda.get_device_capability(),
                      nvcc=subprocess.check_output(['/usr/local/cuda-12.8/bin/nvcc','--version'], text=True).strip()), indent=2))
