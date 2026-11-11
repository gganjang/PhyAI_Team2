"""Convert raw ROS camera frames to bounded browser previews."""
import io

import numpy as np
from PIL import Image


def camera_jpeg(msg):
    if msg.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8', 'mono8'):
        raise ValueError(f'Unsupported preview encoding: {msg.encoding}')
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}[msg.encoding]
    if (msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * channels
            or len(msg.data) != msg.height * msg.step):
        raise ValueError('Invalid camera image dimensions or buffer length.')
    pixels = np.ndarray((msg.height, msg.width, channels), dtype=np.uint8,
                        buffer=msg.data, strides=(msg.step, channels, 1))
    if channels == 1:
        image = Image.fromarray(pixels[:, :, 0])
    else:
        pixels = pixels[:, :, :3]
        if msg.encoding.startswith('bgr'):
            pixels = pixels[:, :, ::-1]
        image = Image.fromarray(pixels)
    image.thumbnail((960, 720))
    output = io.BytesIO()
    image.save(output, format='JPEG', quality=80)
    return output.getvalue()
