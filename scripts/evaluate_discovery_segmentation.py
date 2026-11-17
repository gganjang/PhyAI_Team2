#!/usr/bin/env python3
"""Evaluate configured masks and geometry against archived RGB, without motion.

This reuses measured camera transforms. Robot exclusion masks were not archived,
so this is an image replay, not a replacement for a capture-matched live run.
"""
import argparse
import base64
from collections import Counter
import io
import json
from pathlib import Path
import sys
import time

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/phyai_discovery'))
from phyai_discovery import Capture, Intrinsics
from phyai_discovery.engine import make_detector
from phyai_discovery.diagnostics import capture_packet


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--report',type=Path,required=True,help='Existing session directory')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    config=json.loads(args.config.read_text())
    detector=make_detector(config)
    args.output.mkdir(parents=True,exist_ok=True)
    frames=[]
    try:
        for index,path in enumerate(sorted((args.report/'captures').glob('*.json')),1):
            old=json.loads(path.read_text())
            rgb=np.asarray(Image.open(io.BytesIO(base64.b64decode(old['source_png']))).convert('RGB'))
            capture=Capture(old['camera'],old['stamp'],'archived_optical',old['frame'],
                old['calibration_revision'],Intrinsics(**old['intrinsics']),None,
                np.asarray(old['camera_to_map']),old['viewpoint'],rgb=rgb,
                inspection_target=old.get('focus'))
            started=time.perf_counter();detection=detector.detect(capture);elapsed=time.perf_counter()-started
            packet=capture_packet(capture,detection,detector.observer,
                session_id='offline-replay',revision=index,phase=old['phase'],focus=old.get('focus'))
            (args.output/(old['id']+'.json')).write_text(json.dumps(packet,separators=(',',':'))+'\n')
            frames.append({'capture':old['id'],'phase':old['phase'],'camera':old['camera'],
                'seconds':elapsed,'raw_masks':len(packet['segments']),
                'candidate_observations':len(detection.candidates),
                'resolved_observations':sum(c.resolved for c in detection.candidates),
                'original_candidate_observations':len(old['candidates']),
                'classes':dict(Counter(s.get('class_name','unlabelled') for s in packet['segments']))})
            print(f'{old["id"]}: {len(packet["segments"])} masks, {len(detection.candidates)} candidate observations',flush=True)
    finally:
        close=getattr(detector.segmenter,'close',None)
        if close:close()
    seconds=[f['seconds'] for f in frames[1:]]
    summary={'source_session':args.report.name,'segmentation':config['segmentation'],
        'physical_motion':False,'robot_exclusion_masks_restored':False,
        'capture_count':len(frames),'captures_with_masks':sum(f['raw_masks']>0 for f in frames),
        'total_instance_masks':sum(f['raw_masks'] for f in frames),
        'candidate_observations':sum(f['candidate_observations'] for f in frames),
        'resolved_observations':sum(f['resolved_observations'] for f in frames),
        'median_warm_detection_seconds':float(np.median(seconds)) if seconds else None,
        'frames':frames}
    (args.output/'evaluation.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k not in ('frames','segmentation')},indent=2))


if __name__=='__main__':
    main()
