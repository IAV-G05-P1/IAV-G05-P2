# Minotaur - Base
Proyecto de videojuego actualizado a **Unity 2022.3.40f1** diseñador para servir como punto de partida en algunas prácticas.

Consiste en un entorno virtual 3D que representa el Laberinto del Minotauro, generado procedimentalmente, un personaje controlable por el jugador que es Teseo y uno o varios minotauros para ser controlados mediante IA.

## Licencia
Federico Peinado, autor de la documentación, código y recursos de este trabajo, concedo permiso permanente a los alumnos de la Facultad de Informática de la Universidad Complutense de Madrid para utilizar este material, con sus comentarios y evaluaciones, con fines educativos o de investigación; ya sea para obtener datos agregados de forma anónima como para utilizarlo total o parcialmente reconociendo expresamente mi autoría.

## Notas
Sobre esto hay quien implementa el A* con una estructura de registro de nodo muy simple (el identificador del nodo y el coste f), sólo usa lista de apertura, se apoya en tener toda la información completa del grafo a mano (costes incluidos) y como estructura de datos auxiliar usa una cola de prioridad muy simple.
Según el pseudocódigo que plantea Millington, la estructura de registro de nodo es más rica (identificador del nodo, conexión con el nodo padre, coste g y coste f), se usa una lista de apertura y una lista de cierre, no se asume que toda la información del grafo esté disponible y la cola de prioridad se puede implementar con PriorityQueue<TElement, TPriority> (estructura que se encuentra en el espacio de nombres System.Collections.Generic y fue introducida en .NET 6) o con un BinaryHeap como este: https://github.com/NikolajLeischner/csharp-binary-heap.

## Referencias
Los recursos de terceros utilizados son de uso público.
* *AI for Games*, Ian Millington.
* [Kaykit Medieval Builder Pack](https://kaylousberg.itch.io/kaykit-medieval-builder-pack)
* [Kaykit Dungeon](https://kaylousberg.itch.io/kaykit-dungeon)
* [Kaykit Animations](https://kaylousberg.itch.io/kaykit-animations)

## Pseudocódigo

### BFS
```
def pathfindDijkstra(graph, start, end):
 
 start = GetVertex(startObject)

 startRecord = new NodeRecord()
 startRecord.node = start
 startRecord.connection = None
 startRecord.costSoFar = 0

 open = PathfindingList()
 open += startRecord
 closed = PathfindingList()

 while length(open) > 0:

    current = open.smallestElement()

    if current.node == goal: break

    neighbours = graph.GetNeighbours(current)

    for neighbours.size:
 
        endNode = connection.getToNode()
        endNodeCost = current.costSoFar + connection.getCost()

        if !closed.contains(endNode):
            if open.contains(endNode):
                
                endNodeRecord = open.find(endNode)

                if endNodeRecord.cost <= endNodeCost: continue                
            else:
                endNodeRecord = new NodeRecord()
                endNodeRecord.node = endNode
                endNodeRecord.cost = endNodeCost
                endNodeRecord.connection = connection

                if not open.contains(endNode):
                open += endNodeRecord

        open -= current
        closed += current

    if current.node = goal:
        return path = []
``` 
### DFS

### A*
```
def GetPathAStar(startObject, endObject, heuristic):
    start = GetVertex(startObject)
    goal = GetVertex(endObject)

    startRec = new NodeRecord()
    startRec.node = start
    startRec.costSoFar = 0
    startRec.fromNode = null

    open = List<NodeRecord>()
    open += startRecord
    closed = List<ListRecord>()

    while length(open) > 0:
        current = open.min()

        if current.node == GetVertex(endObject): break

        neighbours = graph.getNeighbours(current)
        neighCost = graph.getNeighboursCosts(current)

        for length(neighbours):
            endNode = neighbours[i]
            endCost = current.costSoFar + neighCost[i]

            if closed.contains(endNode):

                endRec = closed.Find(endNode)

                if endCost < endNode.costSoFar:
                    closed -= endRec
                    endHeuristic = endNode.estimate - endRec.costSoFar
                else :
                    continue
            
            else if open.contains(endNode):

                endRec = closed.Find(endNode)

                if endCost < endNode.costSoFar:
                    endHeuristic = endNode.estimate - endRec.costSoFar
                else :
                    continue
            
            else:
                endRec = new NodeRecord()
                endRec.node = endNode;

                endHeuristic = [Heuristic Function]

            endRec.cosSoFar = endCost;
            endRec.fromNode = curren.node;
            endNode.estimate = endCost + endHeuristic

            if !open.contains(endRec):
                open += endRec 

        open -= current
        closed += current

    if current.node != start:
        return null

    else:
        path = new List<Vertex>

        while current.node != start:
            path += current.node
            current = closed.Find(current.fromNode)

        path.reverse()

        return path;
```

#### Heurística

```
def ManhattanHeuristic(a, b)
{
    posA = a.position;
    posB = b.position;

    return (sqrt(sq(posA.x-posB.x)) + sqrt(sq(posA.y-posB.y)));
}

float EuclideanHeuristic(Vertex a, Vertex b)
{
    posA = a.position;
    posB = b.position;

    return sqrt(sq(posA.x-posB.x) + sq(posA.y-posB.y))
}

float SquareHeuristic(Vertex a, Vertex b)
{
    posA = a.position;
    posB = b.position;

    return (sq(posA.x-posB.x) + sq(posA.y-posB.y))
}
```