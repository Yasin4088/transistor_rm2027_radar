from .flowgraph_node import run_node


def main() -> None:
    run_node(node_name='rm_gfsk_node', flowgraph_name='gfsk')
