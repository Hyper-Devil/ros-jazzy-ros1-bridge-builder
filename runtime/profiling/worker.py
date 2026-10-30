#!/usr/bin/env python3
"""Existing deployment algorithms with HTTP and model phase timestamps."""
import argparse
from http.server import BaseHTTPRequestHandler,HTTPServer
from io import BytesIO
import json
import os
from pathlib import Path
import sys

sys.path.insert(0,'/mnt/nvme1n1/whd/uam2te')
os.chdir('/mnt/nvme1n1/whd/uam2te')
import cv2
import numpy as np
import torch
from runtime_profile import Span,timed,set_stamp
from uam2te.speed_curve_deployment import SpeedCurveInference
import uam2te.speed_curve_deployment as deployment

parser=argparse.ArgumentParser()
parser.add_argument('--config',required=True);parser.add_argument('--checkpoint',required=True)
parser.add_argument('--speed',type=float,default=.8);parser.add_argument('--speeds',type=float,nargs='+')
parser.add_argument('--port',type=int,default=8765);parser.add_argument('--warmup-image',required=True)
args=parser.parse_args()
with Span('startup.model_load'):
    engine=SpeedCurveInference(args.config,args.checkpoint,args.speed,'cuda',reference_speeds=args.speeds)
original=engine.infer_many if args.speeds else engine.infer
image=cv2.imread(args.warmup_image)
with Span('startup.first_inference'):
    reference=original(image)
deployment.query_maps=timed(deployment.query_maps,'worker.speed_query','torch')
deployment.restore_image_only_prediction=timed(deployment.restore_image_only_prediction,'worker.spatial_restore')
engine.task.predict_curves=timed(engine.task.predict_curves,'worker.network_forward','torch')

@torch.inference_mode()
def infer(image):
    with Span('worker.engine_total'):
        with Span('worker.preprocess'):
            images=deployment.preprocess_image_only(image,engine.output_size)
        with Span('worker.host_to_device',gpu='torch'):
            images=images.to(engine.device)
        curves=engine.task.predict_curves(images)
        with Span('worker.query_restore_total'):
            if not args.speeds:
                return deployment.conservative_maps(curves,engine.reference_speed,engine.output_size,(image.shape[1],image.shape[0]))
            maps=[deployment.conservative_maps(curves,speed,engine.output_size,(image.shape[1],image.shape[0]),
                    fields=('conservative_score','support','score') if i==0 else ('conservative_score','support'))
                  for i,speed in enumerate(engine.reference_speeds)]
            return dict(conservative_score=np.stack([m['conservative_score'] for m in maps]),
                        support=np.stack([m['support'] for m in maps]),score=maps[0]['score'])

validation={}
for warmup in range(3):
    actual=infer(image)
    for key in reference:
        np.testing.assert_allclose(actual[key],reference[key],atol=1e-6,rtol=0)
        validation[key]=float(np.max(abs(actual[key]-reference[key])))
Path(os.environ['UAM2TE_PROFILE_DIR'],'worker_validation.json').write_text(json.dumps(dict(max_abs_difference=validation,metadata=engine.metadata),indent=2))
del reference,actual,image

class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args):pass
    def reply(self,code,body,kind):
        self.send_response(code);self.send_header('Content-Type',kind)
        self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
    def do_GET(self):
        if self.path=='/metadata':self.reply(200,json.dumps(engine.metadata).encode(),'application/json')
        else:self.reply(404,b'Unknown endpoint','text/plain')
    def do_POST(self):
        set_stamp(int(self.headers.get('X-Image-Stamp-Ns','0')))
        with Span('worker.http_handler'):
            try:
                if self.path not in ('/infer','/infer_speeds'):raise ValueError('Unknown endpoint')
                with Span('worker.receive_decode'):
                    length=int(self.headers.get('Content-Length','0'))
                    if not 0<length<=64*1024*1024:raise ValueError('Invalid image payload size')
                    data=self.rfile.read(length)
                    if len(data)!=length:raise ValueError('Incomplete image payload')
                    image=np.load(BytesIO(data),allow_pickle=False)
                    if image.dtype!=np.uint8 or image.ndim!=3 or image.shape[2]!=3:raise ValueError('Invalid BGR image')
                result=infer(image)
                with Span('worker.serialize_outputs'):
                    buffer=BytesIO();np.savez(buffer,**result);body=buffer.getvalue()
                with Span('worker.send_response'):
                    self.reply(200,body,'application/octet-stream')
            except Exception as exc:
                self.reply(500,str(exc).encode(),'text/plain');raise

print(json.dumps(dict(ready=True,metadata=engine.metadata)),flush=True)
HTTPServer(('127.0.0.1',args.port),Handler).serve_forever()
