#prerequisites:
#1. Install graphviz 2.38 (or newer)
#2. Add the path to the binaries to system path
#2.1 Windows: C:\Program Files (x86)\Graphviz<version>\bin
#    Add to SYSTEM path in Windows, NOT user's path


#!/usr/bin/env python

import networkx as nx
from nxpd import draw

if __name__ == '__main__':
    labels = {}
    labels[0] = "Project"
    labels[1] = "Poco/1.7.3@lasote/stable"
    labels[2] = "OpenSSL/1.0.2h@lasote/stable"
    labels[3] = "electric-fence/2.2.0@lasote/stable"
    labels[4] = "zlib/1.2.8@lasote/stable"

    G = nx.DiGraph()

    G.add_node(labels[0])

    G.add_node(labels[1])
    G.add_edge(labels[0], labels[1])

    G.add_node(labels[2])
    G.add_edge(labels[1], labels[2])

    G.add_node(labels[3])
    G.add_node(labels[4])
    G.add_edge(labels[2], labels[3])
    G.add_edge(labels[2], labels[4])

    draw(G)