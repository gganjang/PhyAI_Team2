"""Accepted-image evidence and durable reports, independent of ROS."""
import base64
import io
import json
from pathlib import Path
import zlib

import numpy as np


def jpeg(pixels):
    from PIL import Image
    output=io.BytesIO();Image.fromarray(pixels).save(output,format='JPEG',quality=85)
    return base64.b64encode(output.getvalue()).decode()


def packet_image(packet,layer):
    if layer in packet['images']:return base64.b64decode(packet['images'][layer])
    if layer!='masks' and not layer.startswith(('segment_','binary_')):raise KeyError(layer)
    from PIL import Image
    rgb=np.array(Image.open(io.BytesIO(base64.b64decode(packet['images']['rgb']))).convert('RGB'))
    if layer=='masks':
        # Earlier archives stored exact masks but only a boxed composite.
        # Reconstruct the mask-only view without rerunning SAM2.
        for segment in packet['segments']:
            if segment['decision']!='selected':continue
            index=segment['index']
            bits=np.frombuffer(zlib.decompress(base64.b64decode(segment.get('used_mask_bits') or segment['mask_bits'])),dtype=np.uint8)
            mask=np.unpackbits(bits,count=rgb.shape[0]*rgb.shape[1]).reshape(rgb.shape[:2]).astype(bool)
            color=np.array([(index*83+80)%220+30,(index*137+30)%220+30,(index*47+100)%220+30])
            rgb[mask]=(rgb[mask]*.55+color*.45).astype(np.uint8)
        return base64.b64decode(jpeg(rgb))
    index=int(layer.split('_',1)[1])
    segment=packet['segments'][index]
    bits=np.frombuffer(zlib.decompress(base64.b64decode(segment['mask_bits'])),dtype=np.uint8)
    mask=np.unpackbits(bits,count=rgb.shape[0]*rgb.shape[1]).reshape(rgb.shape[:2]).astype(bool)
    if layer.startswith('binary_'):
        return base64.b64decode(jpeg(np.repeat((mask.astype(np.uint8)*255)[:,:,None],3,axis=2)))
    rgb[mask]=(rgb[mask]*.5+np.array([255,130,30])*.5).astype(np.uint8)
    return base64.b64decode(jpeg(rgb))


def capture_packet(capture,detection,observer,*,session_id,revision,phase,focus):
    from PIL import Image,ImageDraw
    raw=capture.rgb.copy();sam2=raw.copy();accepted=raw.copy();segments=[]
    for entry in getattr(observer,'segmentation',[]):
        mask=entry['mask'];index=entry['index']
        color=np.array([(index*83+80)%220+30,(index*137+30)%220+30,(index*47+100)%220+30])
        sam2[mask]=(sam2[mask]*.55+color*.45).astype(np.uint8)
        used=entry.get('used_mask',mask) if entry['decision']=='selected' else np.zeros_like(mask)
        accepted[used]=(accepted[used]*.55+color*.45).astype(np.uint8)
        segments.append({**{k:v for k,v in entry.items() if k not in ('mask','used_mask')},
            'mask_bits':base64.b64encode(zlib.compress(np.packbits(mask).tobytes())).decode(),
            'used_mask_bits':base64.b64encode(zlib.compress(np.packbits(used).tobytes())).decode()})
    sam2_image=Image.fromarray(sam2);sam2_draw=ImageDraw.Draw(sam2_image)
    for entry in getattr(observer,'segmentation',[]):
        v,u=np.nonzero(entry['mask'])
        if len(u):sam2_draw.text((int(u.mean()),int(v.mean())),str(entry['index']),fill='white')
    image=Image.fromarray(accepted);draw=ImageDraw.Draw(image)
    for index,candidate in enumerate(detection.candidates):
        if len(candidate.pixels)==2:
            draw.rectangle([tuple(p) for p in candidate.pixels],outline='lime' if candidate.resolved else 'orange',width=2)
            draw.text(tuple(candidate.pixels[0]),str(index),fill='white')
    if focus:
        point=(np.asarray(focus['position'])-capture.camera_to_map[:3,3])@capture.camera_to_map[:3,:3]
        if point[2]>.01:
            k=capture.intrinsics;x=point[0]/point[2]*k.fx+k.cx;y=point[1]/point[2]*k.fy+k.cy
            draw.line((x-8,y,x+8,y),fill='cyan',width=2)
            draw.line((x,y-8,x,y+8),fill='cyan',width=2)
            draw.text((x+9,y),focus['candidate_id'],fill='cyan')
    identifier=f'{revision:05d}_{capture.camera}'
    original=io.BytesIO();Image.fromarray(raw).save(original,format='PNG')
    return {'id':identifier,'session_id':session_id,'camera':capture.camera,'stamp':capture.stamp,
        'viewpoint':capture.viewpoint,'phase':phase,'focus':focus,
        'frame':capture.map_frame,'camera_to_map':capture.camera_to_map.tolist(),
        'intrinsics':vars(capture.intrinsics),'calibration_revision':capture.calibration_revision,
        'segments':segments,'fits':getattr(observer,'diagnostics',{}),
        'segmentation_kind':('concept_prompted' if getattr(getattr(observer,'segmenter',None),'backend',None)=='sam3.1' else 'trained_class_instances' if getattr(getattr(observer,'segmenter',None),'backend',None)=='yolo26n-seg' else 'class_agnostic_regions'),
        'segmentation_backend':getattr(getattr(observer,'segmenter',None),'backend','sam2'),
        'segmentation_prompt':getattr(getattr(observer,'segmenter',None),'prompt',None),
        'segmentation_min_score':getattr(getattr(observer,'segmenter',None),'min_score',None),
        'candidates':[{'position':c.position,'resolved':c.resolved,'reasons':c.reasons,
                       'pose_uncertainty':c.pose_uncertainty} for c in detection.candidates],
        'inspection_evidence':detection.inspection_evidence,
        'source_png':base64.b64encode(original.getvalue()).decode(),
        'images':{'rgb':jpeg(raw),'sam2':jpeg(np.asarray(sam2_image)),
                  'masks':jpeg(accepted),'accepted':jpeg(np.asarray(image))}}


def capture_summary(packet):
    return {k:packet[k] for k in ('id','session_id','camera','stamp','viewpoint','phase')}|{
        'candidate_count':len(packet['candidates']),'segment_count':len(packet['segments'])}


class CaptureArchive:
    def __init__(self,directory):
        self.directory=Path(directory)
        self.indices={}

    def write(self,path,data):
        path.parent.mkdir(parents=True,exist_ok=True)
        temporary=path.with_suffix('.tmp')
        temporary.write_text(json.dumps(data,allow_nan=False))
        temporary.replace(path)

    def capture(self,packet):
        session=packet['session_id']
        self.write(self.directory/session/'captures'/(packet['id']+'.json'),packet)
        index=self.indices.setdefault(session,{})
        index[packet['id']]=capture_summary(packet)

    def report(self,snapshot):
        session=snapshot['workspace']['session_id']
        self.write(self.directory/session/'report.json',
            {**snapshot,'accepted_captures':list(self.indices.get(session,{}).values())})
        self.write(self.directory/'latest.json',{'session_id':session})

    def load_latest(self):
        latest=self.directory/'latest.json'
        if not latest.exists():return []
        session=json.loads(latest.read_text())['session_id']
        root=(self.directory/session).resolve()
        if not root.is_relative_to(self.directory.resolve()):raise ValueError('Invalid report session path.')
        return [json.loads(path.read_text()) for path in sorted((root/'captures').glob('*.json'))]
