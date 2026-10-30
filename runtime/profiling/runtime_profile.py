"""Opt-in wall-clock / CUDA-event audit helpers, used only by benchmark launch."""
import functools
import json
import os
from pathlib import Path
import threading
import time

_local=threading.local()
_lock=threading.Lock()
_file=None

def set_stamp(value):
    _local.stamp_ns=int(value)

def emit(record):
    global _file
    directory=os.environ.get('UAM2TE_PROFILE_DIR')
    if not directory:return
    with _lock:
        if _file is None:
            role=os.environ.get('UAM2TE_PROFILE_ROLE','python')
            _file=open(str(Path(directory)/(role+'_'+str(os.getpid())+'.jsonl')),'a',buffering=1)
        record.update(pid=os.getpid(),thread=threading.get_ident())
        _file.write(json.dumps(record,separators=(',',':'))+'\n')

class Span:
    def __init__(self,stage,stamp_ns=None,gpu=None):
        self.stage=stage
        self.stamp_ns=int(getattr(_local,'stamp_ns',0) if stamp_ns is None else stamp_ns)
        self.gpu=gpu
        self.closed=False
        self.start=self.last=time.perf_counter_ns()
        if gpu:
            if gpu=='torch':
                import torch
                self.a=torch.cuda.Event(enable_timing=True);self.b=torch.cuda.Event(enable_timing=True)
            else:
                import cupy
                self.a=cupy.cuda.Event();self.b=cupy.cuda.Event()
            self.a.record()
    def mark(self,stage):
        end=time.perf_counter_ns()
        emit(dict(stage=stage,start_ns=self.last,end_ns=end,stamp_ns=self.stamp_ns))
        self.last=time.perf_counter_ns()
    def close(self,error=False):
        if self.closed:return
        gpu_ms=None
        if self.gpu:
            self.b.record();self.b.synchronize()
            if self.gpu=='torch':gpu_ms=float(self.a.elapsed_time(self.b))
            else:
                import cupy
                gpu_ms=float(cupy.cuda.get_elapsed_time(self.a,self.b))
        end=time.perf_counter_ns()
        emit(dict(stage=self.stage,start_ns=self.start,end_ns=end,stamp_ns=self.stamp_ns,gpu_ms=gpu_ms,error=error))
        self.closed=True
    def __enter__(self):return self
    def __exit__(self,typ,value,tb):self.close(typ is not None)

def timed(function,stage,gpu=None,stamp=None):
    @functools.wraps(function)
    def wrapped(*args,**kwargs):
        if stamp is not None:set_stamp(stamp(*args,**kwargs))
        with Span(stage,gpu=gpu):return function(*args,**kwargs)
    return wrapped

def instrument_elevation(module):
    cls=module.ElevationMap
    original=cls.__init__
    def init(self,*args,**kwargs):
        original(self,*args,**kwargs)
        self.image_to_map_correspondence_kernel=timed(self.image_to_map_correspondence_kernel,'mapping.projection_kernel','cupy')
    cls.__init__=init
    for name,label in [('input_pointcloud','mapping.pointcloud_python'),('input_image','mapping.image_python'),
                       ('update_map_with_kernel','mapping.elevation_kernel_pipeline'),('get_map_with_name_ref','mapping.layer_to_cpu')]:
        setattr(cls,name,timed(getattr(cls,name),label,'cupy'))
    from elevation_mapping_cupy.fusion.image_ema_variance import ImageEMAVariance
    ImageEMAVariance.__call__=timed(ImageEMAVariance.__call__,'mapping.ema_world_fusion','cupy')
    from elevation_mapping_cupy.semantic_map import SemanticMap
    SemanticMap.update_layers_image=timed(SemanticMap.update_layers_image,'mapping.semantic_layers','cupy')
    emit(dict(stage='profiling.emcp_hook_ready',start_ns=time.perf_counter_ns(),end_ns=time.perf_counter_ns(),stamp_ns=0))
