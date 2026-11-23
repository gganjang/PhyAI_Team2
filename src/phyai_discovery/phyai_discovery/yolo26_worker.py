"""YOLO26 worker protocol: RGB in, exact instance masks and labels out."""
import contextlib
import json
import sys
import traceback


def infer(model, rgb, request):
    import numpy as np
    # Ultralytics' numpy input convention is BGR; capture archives are RGB.
    result = model.predict(source=np.ascontiguousarray(rgb[:, :, ::-1]),
        conf=request.get('min_score', .25), imgsz=request.get('image_size', 1280),
        classes=request.get('classes'), device=0 if request.get('device', 'cuda') == 'cuda' else 'cpu',
        retina_masks=True, verbose=False)[0]
    if result.masks is None:
        if result.boxes is not None and len(result.boxes):
            raise ValueError('YOLO returned boxes without instance masks.')
        return {'masks': np.empty((0, *rgb.shape[:2]), bool), 'scores': np.empty(0),
                'class_ids': np.empty(0, dtype=int), 'class_names': np.empty(0, dtype=str)}
    masks = result.masks.data.cpu().numpy().astype(bool)
    scores = result.boxes.conf.cpu().numpy()
    classes = result.boxes.cls.cpu().numpy().astype(int)
    if masks.shape != (len(scores), *rgb.shape[:2]):
        raise ValueError('YOLO masks do not match the original RGB dimensions.')
    return {'masks': masks, 'scores': scores, 'class_ids': classes,
            'class_names': np.asarray([result.names[int(c)] for c in classes], dtype=str)}


def main():
    import numpy as np
    protocol = sys.stdout
    with contextlib.redirect_stdout(sys.stderr):
        from ultralytics import YOLO
        model = YOLO(sys.argv[1], task='segment')
        if model.task != 'segment':
            raise ValueError('Discovery requires a YOLO segmentation checkpoint.')
    print(json.dumps({'ready': True, 'backend': 'yolo26n-seg'}), file=protocol, flush=True)
    for line in sys.stdin:
        try:
            request = json.loads(line)
            with contextlib.redirect_stdout(sys.stderr):
                output = infer(model, np.load(request['source'], allow_pickle=False), request)
                np.savez_compressed(request['output'], **output)
            print(json.dumps({'ok': True}), file=protocol, flush=True)
        except Exception as error:
            traceback.print_exc(file=sys.stderr)
            print(json.dumps({'error': str(error)}), file=protocol, flush=True)


if __name__ == '__main__':
    main()
