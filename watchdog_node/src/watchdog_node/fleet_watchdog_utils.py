#!/usr/bin/env python
""" Utils for fleet monitors/actions.

Author: Charlie Street
Owner: Charlie Street
"""

import rosnode

def get_robot_ids():
    """ Get the list of robots by looking at node names (and namespaces).

    Returns:
        robot_ids: The list of robot_ids
    """

    robot_ids = []

    node_names = rosnode.get_node_names()

    for node in node_names:
        if 'robot_' in node:
            r_id = node[1:]
            r_id = r_id[0:r_id.index('/')]
            if r_id not in robot_ids:
                robot_ids.append(r_id)

    return robot_ids
