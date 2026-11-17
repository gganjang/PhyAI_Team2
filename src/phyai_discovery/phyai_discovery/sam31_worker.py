"""One persistent SAM 3.1 model; fresh sessions for independent still captures.

Executed by the isolated model interpreter, never imported by ROS.
"""
import contextlib
import json
import inspect
from pathlib import Path
import sys
import tempfile
import traceback


def configure_predictor(predictor):
    """Bridge the pinned upstream wrapper's unused CPU-state option.

    Its base start_session always supplies offload_state_to_cpu, but the 3.1
    multiplex model does not accept it. Our sessions keep state on CUDA.
    """
    init_state = predictor.model.init_state
    if 'offload_state_to_cpu' not in inspect.signature(init_state).parameters:
        def init_state_compat(*args, offload_state_to_cpu=False, **kwargs):
            if offload_state_to_cpu:
                raise ValueError('SAM 3.1 multiplex CPU-state offloading is unsupported.')
            return init_state(*args, **kwargs)
        predictor.model.init_state = init_state_compat
    return predictor


def infer(predictor, rgb, prompt, min_score):
    import numpy as np
    from PIL import Image
    # The official video loader takes numbered JPEG frames. A new single-frame
    # session prevents tracked masks surviving across different camera poses.
    with tempfile.TemporaryDirectory(prefix='sam31-frame-') as directory:
        Image.fromarray(rgb).save(Path(directory)/'00000.jpg', quality=100, subsampling=0)
        session = predictor.handle_request({'type': 'start_session', 'resource_path': directory})['session_id']
        try:
            output = predictor.handle_request({'type': 'add_prompt', 'session_id': session,
                'frame_index': 0, 'text': prompt, 'output_prob_thresh': min_score})['outputs']
            masks = np.asarray(output['out_binary_masks'], dtype=bool)
            scores = np.asarray(output['out_probs'], dtype=float)
            if not len(scores):
                masks = np.empty((0, *rgb.shape[:2]), dtype=bool)
            keep = np.isfinite(scores) & (scores >= min_score)
            return masks[keep], scores[keep]
        finally:
            predictor.handle_request({'type': 'close_session', 'session_id': session})


def main():
    import numpy as np
    protocol = sys.stdout
    with contextlib.redirect_stdout(sys.stderr):
        import torch
        from sam3.model_builder import build_sam3_predictor
        if not torch.cuda.is_available():
            raise RuntimeError('SAM 3.1 requires an available CUDA GPU.')
        predictor = build_sam3_predictor(checkpoint_path=sys.argv[1], version='sam3.1',
            compile=False, warm_up=False, use_fa3=False, async_loading_frames=False,
            max_num_objects=64)
        configure_predictor(predictor)
        if '--self-test' in sys.argv[2:]:
            infer(predictor, np.zeros((64,64,3),dtype=np.uint8), 'small rectangular block', .7)
    print(json.dumps({'ready': True, 'backend': 'sam3.1'}), file=protocol, flush=True)
    for line in sys.stdin:
        try:
            request = json.loads(line)
            with contextlib.redirect_stdout(sys.stderr):
                masks, scores = infer(predictor, np.load(request['source'], allow_pickle=False),
                                      request['prompt'], request['min_score'])
                np.savez_compressed(request['output'], masks=masks, scores=scores)
            print(json.dumps({'ok': True}), file=protocol, flush=True)
        except Exception as error:
            traceback.print_exc(file=sys.stderr)
            print(json.dumps({'error': str(error)}), file=protocol, flush=True)


if __name__ == '__main__':
    main()
