"""The shared browser viewer works directly with core snapshots, without ROS."""
import json
from urllib.request import urlopen

from phyai_discovery import Discovery
from phyai_discovery.viewer import WorkspaceWeb


def test_core_snapshot_viewer_serves_final_inventory_without_a_ros_client():
    feature = Discovery()
    feature.finish()
    snapshot = feature.snapshot()
    viewer = WorkspaceWeb(0)
    try:
        viewer.update(snapshot)
        root = f'http://127.0.0.1:{viewer.server.server_port}'
        page = urlopen(root).read().decode()
        assert 'Workspace map' in page and 'Live 3D viewer' in page
        packet = json.load(urlopen(root+'/api/workspace'))
        assert packet['simulator_port'] == 0
        assert packet['snapshot']['block_info']['status'] == 'ready'
        assert packet['snapshot']['block_info']['blocks'] == []
        assert 'block_info' not in snapshot
        explicit = {**snapshot, 'block_info': {'status': 'pending'}}
        viewer.update(explicit)
        packet = json.load(urlopen(root+'/api/workspace'))
        assert packet['snapshot']['block_info']['status'] == 'pending'
    finally:
        viewer.close()


def test_live_camera_preview_endpoints_and_stale_frames(monkeypatch):
    from urllib.error import HTTPError
    import pytest
    from phyai_discovery.viewer import web as module
    clock=[100.]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    viewer=WorkspaceWeb(0,cameras={'wrist':'/real/wrist/color','overhead':'/real/overhead/color'})
    root=f'http://127.0.0.1:{viewer.server.server_port}'
    try:
        packet=json.load(urlopen(root+'/api/workspace'))
        assert packet['cameras']['wrist']=={'topic':'/real/wrist/color','age':None}
        assert packet['snapshot'] is None  # Preview works before discovery starts.
        with pytest.raises(HTTPError) as error:urlopen(root+'/camera/wrist.jpg')
        assert error.value.code==503
        viewer.update_camera('wrist',b'jpeg-frame')
        with urlopen(root+'/camera/wrist.jpg?t=123') as response:
            assert response.headers['Content-Type']=='image/jpeg'
            assert response.read()==b'jpeg-frame'
        with pytest.raises(HTTPError) as error:urlopen(root+'/camera/unknown.jpg')
        assert error.value.code==404
        clock[0]+=2.1
        with pytest.raises(HTTPError) as error:urlopen(root+'/camera/wrist.jpg')
        assert error.value.code==503
        viewer.update_camera('wrist',b'fresh-frame')
        assert urlopen(root+'/camera/wrist.jpg').read()==b'fresh-frame'
    finally:viewer.close()


def test_viewer_listen_address_can_enable_lan_access():
    viewer=WorkspaceWeb(0,host='0.0.0.0')
    try:
        assert viewer.server.server_address[0]=='0.0.0.0'
        with urlopen(f'http://127.0.0.1:{viewer.server.server_port}/') as response:
            assert response.status==200
    finally:viewer.close()
