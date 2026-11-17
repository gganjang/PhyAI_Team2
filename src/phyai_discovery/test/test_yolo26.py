"""YOLO image conventions, true masks, trained labels and setup configuration."""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from phyai_discovery.yolo26_worker import infer
from phyai_discovery.yolo26 import YOLO26Segmenter
from phyai_discovery import setup_yolo26


class Tensor:
    def __init__(self, array):self.array=np.asarray(array)
    def cpu(self):return self
    def numpy(self):return self.array


def test_yolo_rgb_to_bgr_exact_instance_masks_and_trained_class_labels():
    rgb=np.zeros((6,8,3),np.uint8);rgb[:,:,0]=231;rgb[:,:,2]=17
    mask=np.zeros((1,6,8),bool);mask[0,1:3,2:5]=True
    def predict(**options):
        np.testing.assert_array_equal(options['source'],rgb[:,:,::-1])
        assert options['retina_masks'] and options['classes']==[65]
        assert options['imgsz']==1280 and options['device']=='cpu'
        return [SimpleNamespace(masks=SimpleNamespace(data=Tensor(mask)),
            boxes=SimpleNamespace(conf=Tensor([.8]),cls=Tensor([65.])),names={65:'remote'})]
    output=infer(SimpleNamespace(predict=predict),rgb,
                 {'min_score':.25,'classes':[65],'image_size':1280,'device':'cpu'})
    np.testing.assert_array_equal(output['masks'],mask)
    metadata=YOLO26Segmenter.metadata(SimpleNamespace(backend='yolo26n-seg'),output)
    assert metadata==[{'backend':'yolo26n-seg','class_id':65,'class_name':'remote','score':.8}]
    assert output['scores'].tolist()==[.8]


def test_empty_yolo_output_and_boxes_without_masks_are_distinct():
    rgb=np.zeros((6,8,3),np.uint8)
    result=SimpleNamespace(masks=None,boxes=[])
    model=SimpleNamespace(predict=lambda **_: [result])
    assert infer(model,rgb,{})['masks'].shape==(0,6,8)
    result.boxes=[object()]
    with pytest.raises(ValueError,match='boxes without instance masks'):infer(model,rgb,{})


def test_yolo_runtime_preserves_calibration_and_never_injects_model_packages_into_ros(tmp_path):
    config=tmp_path/'discovery.json'
    config.write_text(json.dumps({'cameras':{'wrist':{'intrinsics_file':'calibration.json'}},
        'segmentation':{'backend':'yolo26n-seg','min_score':.25,'classes':None,'image_size':1280}}))
    runtime=tmp_path/'runtime';runtime.mkdir()
    checkpoint=runtime/'model.pt';checkpoint.write_bytes(b'weights')
    generated=setup_yolo26.write_runtime(runtime,Path('/isolated/python'),checkpoint,config,'cuda')
    data=json.loads(generated.read_text())
    assert data['cameras']['wrist']['intrinsics_file']==str(tmp_path/'calibration.json')
    assert data['segmentation']['backend']=='yolo26n-seg'
    assert data['segmentation']['classes'] is None
    assert len(data['segmentation']['checkpoint_sha256'])==64
    env=(runtime/'env.sh').read_text()
    assert 'PHYAI_YOLO26_PYTHON=' in env and 'PYTHONPATH' not in env
