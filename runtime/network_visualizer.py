#!/usr/bin/python3
"""Display the exact model q=score*support on separate visualization topics."""
import argparse
from collections import OrderedDict
import copy
from io import BytesIO
import json
from pathlib import Path
import threading
from urllib.request import Request, urlopen

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String


def colorize(value):
    value=np.clip(value,0,1)
    return np.stack((np.clip(2*(1-value),0,1),np.clip(2*value,0,1),np.zeros_like(value)),axis=-1)*255


class NetworkVisualizer:
    def __init__(self,session):
        self.output=session
        self.session=json.loads((session/'session.json').read_text())
        self.bridge=CvBridge()
        self.lock=threading.RLock()
        self.images=OrderedDict()
        self.size=(640,480)
        self.publishers={name:rospy.Publisher('/uam2te/visualization/'+name,Image,queue_size=1,latch=True)
                         for name in ['q','q_heatmap','q_overlay','score_heatmap','support_heatmap','camera','mapping_mask']}
        self.metadata=rospy.Publisher('/uam2te/visualization/metadata',String,queue_size=1,latch=True)
        self.subscribers=[rospy.Subscriber(self.session.get('image_topic','/hikrobot_camera_L/image_rect'),Image,self.on_image,
                                          queue_size=1,buff_size=32*1024*1024),
                          rospy.Subscriber('/uam2te/conservative_score',Image,self.on_q,
                                           queue_size=1,buff_size=32*1024*1024)]
        for name in ['score','support']:
            self.subscribers.append(rospy.Subscriber('/uam2te/'+name,Image,self.on_diagnostic,
                callback_args=name,queue_size=1,buff_size=32*1024*1024))

    def publish(self,name,image,encoding,header):
        msg=self.bridge.cv2_to_imgmsg(image,encoding)
        msg.header=copy.deepcopy(header)
        self.publishers[name].publish(msg)

    def on_image(self,msg):
        image=cv2.resize(self.bridge.imgmsg_to_cv2(msg,'bgr8'),self.size)
        with self.lock:
            self.images[msg.header.stamp.to_nsec()]=image
            while len(self.images)>8:self.images.popitem(last=False)

    def draw(self,q,header,image=None,label='LIVE'):
        q=cv2.resize(np.asarray(q,dtype=np.float32),self.size)
        self.publish('q',q,'32FC1',header)
        heat=colorize(q).astype(np.uint8)
        text='q = score x support   0:red   1:green'
        cv2.putText(heat,text,(9,24),cv2.FONT_HERSHEY_SIMPLEX,.48,(255,255,255),2,cv2.LINE_AA)
        self.publish('q_heatmap',heat,'rgb8',header)
        overlay=None
        if image is not None:
            self.publish('camera',image,'bgr8',header)
            rgb=cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
            overlay=cv2.addWeighted(rgb,.55,colorize(q).astype(np.uint8),.45,0)
            cv2.putText(overlay,label,(9,24),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),2,cv2.LINE_AA)
            self.publish('q_overlay',overlay,'rgb8',header)
        return heat,overlay

    def on_q(self,msg):
        q=self.bridge.imgmsg_to_cv2(msg,'32FC1')
        with self.lock:image=self.images.get(msg.header.stamp.to_nsec())
        self.draw(q,msg.header,image)

    def on_diagnostic(self,msg,name):
        value=self.bridge.imgmsg_to_cv2(msg,'32FC1')
        image=cv2.resize(value,self.size)
        self.publish(name+'_heatmap',colorize(image).astype(np.uint8),'rgb8',msg.header)
        if name=='support':
            threshold=self.session.get('support_min',0.)
            accepted=np.isfinite(value)&(value>=threshold)
            mask=cv2.resize(accepted.astype(np.uint8)*255,self.size,interpolation=cv2.INTER_NEAREST)
            mask=cv2.cvtColor(mask,cv2.COLOR_GRAY2RGB)
            cv2.putText(mask,'support >= %g; white: accepted'%threshold,(9,24),
                        cv2.FONT_HERSHEY_SIMPLEX,.5,(0,180,255),2,cv2.LINE_AA)
            self.publish('mapping_mask',mask,'rgb8',msg.header)

    def preview(self,path):
        expected=json.loads((self.output/'prediction_metadata.json').read_text())
        worker=self.session['worker_url']
        with urlopen(worker+'/metadata',timeout=10) as response:assert json.load(response)==expected
        image=cv2.imread(str(path))
        if image is None:raise ValueError('Cannot read rectified preview image')
        buffer=BytesIO();np.save(buffer,image,allow_pickle=False)
        request=Request(worker+'/infer',data=buffer.getvalue(),headers={'Content-Type':'application/octet-stream'})
        with urlopen(request,timeout=60) as response, np.load(BytesIO(response.read()),allow_pickle=False) as archive:
            maps={key:archive[key].copy() for key in ['score','support','conservative_score','cost']}
        with urlopen(worker+'/metadata',timeout=10) as response:assert json.load(response)==expected
        frames=json.loads((self.output/'result.json').read_text())['frames']
        frame=next(value for value in frames.values() if Path(value['path']).resolve()==path.resolve())
        header=Header(stamp=rospy.Time.from_sec(frame['stamp']),frame_id=frame['frame'])
        elapsed=frame['stamp']-self.session['bag_start']
        heat,overlay=self.draw(maps['conservative_score'],header,cv2.resize(image,self.size),
                               'FROZEN preview @ %.1f s'%elapsed)
        for name in ['score','support']:
            self.publish(name+'_heatmap',colorize(cv2.resize(maps[name],self.size)).astype(np.uint8),'rgb8',header)
        np.savez_compressed(self.output/'network_preview_maps.npz',**maps)
        cv2.imwrite(str(self.output/'network_q_heatmap.png'),cv2.cvtColor(heat,cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(self.output/'network_q_overlay.png'),cv2.cvtColor(overlay,cv2.COLOR_RGB2BGR))
        report=dict(mode='frozen_preview',source_image=str(path),source_stamp=frame['stamp'],
                    elapsed_s=elapsed,reference_speed=self.session['reference_speed'],fixed_color_range=[0,1],
                    feeds_mapping=False,checkpoint_sha256=expected['checkpoint_sha256'],
                    distribution={key:np.quantile(value,[0,.05,.25,.5,.75,.95,1]).tolist() for key,value in maps.items()})
        (self.output/'network_preview.json').write_text(json.dumps(report,indent=2)+'\n')
        self.metadata.publish(String(data=json.dumps(report)))
        print(json.dumps(report),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,required=True)
    parser.add_argument('--preview-image',type=Path)
    args=parser.parse_args()
    rospy.init_node('cvar_network_visualizer')
    visualizer=NetworkVisualizer(args.session)
    if args.preview_image:visualizer.preview(args.preview_image)
    rospy.spin()


if __name__=='__main__':main()
