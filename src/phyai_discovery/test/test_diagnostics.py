"""Exact segmentation, durable reports and viewer evidence layers."""
import base64
import io
import json
from types import SimpleNamespace
from urllib.request import urlopen
import zlib
from dataclasses import replace
import numpy as np
from phyai_discovery import Capture, Discovery, Intrinsics
from phyai_discovery.perception import Candidate, Detection
from PIL import Image
from phyai_discovery.diagnostics import CaptureArchive, capture_packet, packet_image
from phyai_discovery.viewer import WorkspaceWeb


def detection(stamp=1.,candidates=None,evidence=None):
    empty=np.empty((0,3))
    return Detection('wrist',stamp,'base_link','calibration','view_'+str(stamp),
        None,empty,empty,empty,candidates or [],[],inspection_evidence=evidence)


def candidate(x=0.,resolved=False):
    return Candidate([x,0.,.0075],[.075,.025,.015],[1.,0.,0.,0.],
        'provisional',resolved,[],[],pose_uncertainty={'position_m':.003,'yaw_rad':.02})


def test_exact_segmentation_and_rgb_survive_archiving_and_viewer_restart(tmp_path):
    rgb=np.arange(12*16*3,dtype=np.uint8).reshape(12,16,3)
    capture=Capture('wrist',1.,'optical','base_link','calibration',
        Intrinsics(16,12,20.,20.,7.5,5.5),None,np.eye(4),'inspect_1',rgb=rgb)
    mask=np.zeros((12,16),bool);mask[3:7,4:9]=True
    observer=SimpleNamespace(segmentation=[{'index':0,'mask':mask,'decision':'small_geometry'}],
                             diagnostics={'fit':'incomplete'})
    feature=Discovery({'map_frame':'base_link'})
    packet=capture_packet(capture,detection(),observer,session_id=feature.workspace.session,
                          revision=1,phase='candidate_inspection',focus=None)
    np.testing.assert_array_equal(np.array(Image.open(io.BytesIO(base64.b64decode(packet['source_png'])))),rgb)
    bits=zlib.decompress(base64.b64decode(packet['segments'][0]['mask_bits']))
    np.testing.assert_array_equal(np.unpackbits(np.frombuffer(bits,dtype=np.uint8)).reshape(mask.shape),mask)
    feature.finish()
    archive=CaptureArchive(tmp_path);archive.capture(packet);archive.report(feature.snapshot())
    report=json.loads((tmp_path/feature.workspace.session/'report.json').read_text())
    assert report['accepted_captures'][0]['segment_count']==1
    viewer=WorkspaceWeb(0)
    try:
        viewer.load_report(tmp_path)
        root=f'http://127.0.0.1:{viewer.server.server_port}'
        assert json.load(urlopen(root+'/api/captures'))[0]['id']==packet['id']
        restored=json.load(urlopen(root+'/api/captures/'+packet['id']))
        assert restored==packet
        for layer in ('rgb','sam2','accepted','masks','segment_0','binary_0'):
            image=Image.open(io.BytesIO(urlopen(root+'/capture/'+packet['id']+'/'+layer+'.jpg').read()))
            assert image.size==(16,12)
        assert json.load(urlopen(root+'/api/workspace'))['snapshot']['block_info']['status']=='ready'
        feature.reset();viewer.update(feature.snapshot())
        assert json.load(urlopen(root+'/api/captures'))==[]
    finally:viewer.close()


def test_mask_only_image_contains_no_bounding_rectangle_and_supports_old_archives():
    capture=Capture('wrist',1.,'optical','base_link','calibration',
        Intrinsics(16,12,20.,20.,7.5,5.5),None,np.eye(4),'survey_1',
        rgb=np.zeros((12,16,3),np.uint8))
    box=replace(candidate(),pixels=[[3,3],[10,10]])
    packet=capture_packet(capture,detection(candidates=[box]),None,
        session_id='audit',revision=1,phase='survey',focus=None)
    filtered=np.array(Image.open(io.BytesIO(packet_image(packet,'masks'))))
    boxed=np.array(Image.open(io.BytesIO(packet_image(packet,'accepted'))))
    assert filtered.max()==0
    assert boxed.max()>100
    del packet['images']['masks']
    restored=np.array(Image.open(io.BytesIO(packet_image(packet,'masks'))))
    assert restored.max()==0
