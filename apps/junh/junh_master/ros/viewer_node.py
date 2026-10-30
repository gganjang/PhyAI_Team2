"""Junh compatibility entry point for the shared discovery viewer."""
from phyai_discovery_ros.viewer import ViewerNode as SharedViewerNode
from phyai_discovery_ros.runtime import run


class ViewerNode(SharedViewerNode):
    def __init__(self, **kwargs):
        super().__init__(node_name='junh_workspace_viewer', interface_prefix='junh',
                         inventory_name='discovery', **kwargs)


def main(args=None):
    run(ViewerNode, args)
