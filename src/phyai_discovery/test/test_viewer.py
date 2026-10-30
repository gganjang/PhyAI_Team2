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
