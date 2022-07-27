class Graph():
    def __init__(self):
        self.vertices = []

    def addVertex(self, v):
        self.vertices.append(v)

    def addDirectedEdge(self, v, u):
        u.addInNeighbour(v)
        v.addOutNeighbour(u)

    def getDirectedEdges(self):
        ret = list()
        for v in self.vertices:
            ret += [[v, u] for u in v.outNeighbours]
        return ret
