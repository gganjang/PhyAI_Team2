"""Ruler choices select independent browser streams, without editing the scene."""
from http.server import ThreadingHTTPServer
import threading
from urllib.error import HTTPError
from urllib.request import urlopen

import pytest

from piper_mujoco_sim.web import FrameHub, make_handler, overview_hub


@pytest.fixture
def server():
    hubs = {overview_hub(mask): FrameHub() for mask in range(64)}
    server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(hubs))
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f'http://127.0.0.1:{server.server_port}', hubs
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)


@pytest.mark.parametrize('query,mask', [('', 1), ('&rulers=0', 0), ('&rulers=2', 2),
                                      ('&rulers=32', 32), ('&rulers=63', 63)])
def test_stream_routes_to_requested_rulers(server, query, mask):
    url, hubs = server
    with urlopen(url + '/stream?view=overview' + query, timeout=3) as response:
        hub = hubs[overview_hub(mask)]
        with hub.cond:
            assert hub.cond.wait_for(lambda: hub.clients == 1, timeout=3)
        assert sum(h.clients for h in hubs.values()) == 1
        hub.publish(b'ruler-frame')
        assert response.readline() == b'--frame\r\n'
        assert response.readline() == b'Content-Type: image/jpeg\r\n'


@pytest.mark.parametrize('mask', ['-1', '64', 'oops'])
def test_invalid_ruler_selection(server, mask):
    url, hubs = server
    with pytest.raises(HTTPError) as error:
        urlopen(url + '/stream?view=overview&rulers=' + mask, timeout=3)
    assert error.value.code == 400
    assert all(hub.clients == 0 for hub in hubs.values())


def test_both_pages_offer_all_ruler_toggles(server):
    url, _ = server
    for path in ('/', '/edit'):
        with urlopen(url + path, timeout=3) as response:
            page = response.read()
        for bit in range(6):
            assert f'data-ruler="{bit}"'.encode() in page
        assert b"'&rulers='+rulerMask()" in page
