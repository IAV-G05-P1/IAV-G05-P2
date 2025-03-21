/*    
   Copyright (C) 2020-2023 Federico Peinado
   http://www.federicopeinado.com
   Este fichero forma parte del material de la asignatura Inteligencia Artificial para Videojuegos.
   Esta asignatura se imparte en la Facultad de Informática de la Universidad Complutense de Madrid (España).
   Autor: Federico Peinado 
   Contacto: email@federicopeinado.com
*/
namespace UCM.IAV.Navegacion
{

    using UnityEngine;
    using System.Collections;
    using System.Collections.Generic;
    using System.Linq;
    using System;

    /// <summary>
    /// Abstract class for graphs
    /// </summary>
    public abstract class Graph : MonoBehaviour
    {
        // Aquí el grafo entero es representado con estas listas, que luego puede aprovechar el algoritmo A*.
        // El pseudocódigo de Millington no asume que tengamos toda la información del grafo representada y por eso va guardando registros de los nodos que visita.
        public GameObject vertexPrefab;
        protected List<Vertex> vertices;
        protected List<List<Vertex>> neighbourVertex;
        protected List<List<float>> costs;
        protected bool[,] mapVertices;
        protected float[,] costsVertices;
        protected int numCols, numRows;

        // this is for informed search like A*
        // Un delegado especifica la cabecera de una función, la que sea, que cumpla con esos parámetros y devuelva ese tipo.
        // Cuidado al implementarlas, porque no puede ser que la distancia -por ejemplo- entre dos casillas tenga una heurística más cara que el coste real de navegar de una a otra.
        public delegate float Heuristic(Vertex a, Vertex b);

        // Used for getting path in frames
        public List<Vertex> path;

        public struct NodeRecord
        {
            public Vertex node { get; set; }
            public float costSoFar { get; set; }
            public Vertex fromNode { get; set; }
        }

        protected virtual int GridToId(int x, int y)
        {
            return 0;
        }

        protected virtual Vector2 IdToGrid(int id)
        {
            return new Vector2();
        }

        #region Heurísticas
        public Heuristic GetHeuristic(int index)
        {
            switch (index)
            {
                case 1:
                    return ManhattanHeuristic;
                case 2:
                    return EuclideanHeuristic;
                case 3:
                    return SquareHeuristic;
                default:
                    return ManhattanHeuristic;
            }
        }
        float ManhattanHeuristic(Vertex a, Vertex b)
        {
            Vector2 posA = IdToGrid(a.id);
            Vector2 posB = IdToGrid(b.id);

            return (float)(Math.Sqrt(Math.Pow(posA.x - posB.x, 2)) + Math.Sqrt(Math.Pow(posA.y - posB.y, 2)));
        }

        float EuclideanHeuristic(Vertex a, Vertex b)
        {
            Vector2 posA = IdToGrid(a.id);
            Vector2 posB = IdToGrid(b.id);

            return (float)Math.Sqrt(Math.Pow(posA.x - posB.x, 2) + Math.Pow(posA.y - posB.y, 2));
        }

        float SquareHeuristic(Vertex a, Vertex b)
        {
            Vector2 posA = IdToGrid(a.id);
            Vector2 posB = IdToGrid(b.id);

            return (float)(Math.Pow(posA.x - posB.x, 2) + Math.Pow(posA.y - posB.y, 2));
        }
        #endregion


        public virtual void Start()
        {
            Load();
        }

        public virtual void Load() { }

        public virtual int GetSize()
        {
            if (ReferenceEquals(vertices, null))
                return 0;
            return vertices.Count;
        }

        public virtual void UpdateVertexCost(Vector3 position, float costMultipliyer) { }

        public virtual Vertex GetNearestVertex(Vector3 position)
        {
            return null;
        }

        public virtual GameObject GetRandomPos()
        {
            return null;
        }

        public virtual Vertex[] GetNeighbours(Vertex v)
        {
            if (ReferenceEquals(neighbourVertex, null) || neighbourVertex.Count == 0 ||
                v.id < 0 || v.id >= neighbourVertex.Count)
                return new Vertex[0];
            return neighbourVertex[v.id].ToArray();
        }

        public virtual float[] GetNeighboursCosts(Vertex v)
        {
            if (ReferenceEquals(neighbourVertex, null) || neighbourVertex.Count == 0 ||
                v.id < 0 || v.id >= neighbourVertex.Count)
                return new float[0];

            Vertex[] neighs = neighbourVertex[v.id].ToArray();
            float[] costsV = new float[neighs.Length];
            for (int neighbour = 0; neighbour < neighs.Length; neighbour++) {
                int j = (int)Mathf.Floor(neighs[neighbour].id / numCols);
                int i = (int)Mathf.Floor(neighs[neighbour].id % numCols);
                costsV[neighbour] = costsVertices[j, i];
            }

            return costsV;
        }

        //JAVIER
        // Encuentra caminos óptimos
        public List<Vertex> GetPathBFS(GameObject srcO, GameObject dstO)
        {
            // IMPLEMENTAR ALGORITMO BFS
            Vertex start = GetNearestVertex(srcO.transform.position);
            Vertex goal = GetNearestVertex(dstO.transform.position);
            
            NodeRecord startRec = new NodeRecord();
            startRec.node = start;
            startRec.costSoFar = 0;
            startRec.fromNode = null;

            List<NodeRecord> open = new List<NodeRecord>();
            open.Add(startRec);
            List<NodeRecord> closed = new List<NodeRecord>();
            Vertex curry = start;
            NodeRecord current = startRec;

            bool onGoal = false;            

            while (!onGoal && open.Count >0) {
                curry = open.Min(rec => rec.node);
                current = open.Find(rec => rec.node == curry);

                if (current.node == goal)
                {
                    onGoal = true;
                }
                else
                {
                    Vertex[] neighbours = GetNeighbours(current.node);
                    float[] neighCost = GetNeighboursCosts(current.node);

                    for (int i = 0; i < neighbours.Count(); i++)
                    {                        
                        Vertex endNode = neighbours[i];
                        float endCost = current.costSoFar + neighCost[i];

                        //Si no está entre los nodos cerrados
                        if (!closed.Any(rec => rec.node == endNode))
                        {                            
                            //Si está en los nodos abiertos
                            if (open.Any(rec => rec.node == endNode))
                            {
                                NodeRecord endRec = open.Find(rec => rec.node == endNode);
                                //Si tiene un coste menor al asignado, se sustituye
                                if (endCost < endRec.costSoFar)
                                {
                                    endRec.costSoFar = endCost;
                                    endRec.fromNode = current.node;
                                }
                            }
                            //Si ni está entre los nodos cerrados o abiertos se añade
                            else
                            {
                                NodeRecord endRec = new NodeRecord();
                                endRec.node = endNode;
                                endRec.fromNode = current.node;
                                endRec.costSoFar = endCost;
                                open.Add(endRec);
                            }
                        }
                    }
                }
                if (onGoal)
                {
                    List<Vertex> path = new List<Vertex>();

                    while (current.node != start)
                    {
                        path.Add(current.node);
                        current = closed.Find(rec => rec.node == current.fromNode);
                    }

                    path.Reverse();

                    return path;
                }
                open.Remove(current);
                closed.Add(current);
            }

            return null;
        }

        //JULIA
        // No encuentra caminos óptimos
        public List<Vertex> GetPathDFS(GameObject srcO, GameObject dstO)
        {
            // IMPLEMENTAR ALGORITMO DFS
            return new List<Vertex>();
        }

        //PABLO
        public List<Vertex> GetPathAstar(GameObject srcO, GameObject dstO, Heuristic h = null)
        {
            // IMPLEMENTAR ALGORITMO A*
            Vertex start = GetNearestVertex(srcO.transform.position);
            Vertex goal = GetNearestVertex(dstO.transform.position);

            NodeRecord startRec = new NodeRecord();
            startRec.node = start;
            startRec.costSoFar = 0;
            startRec.fromNode = null;

            List<NodeRecord> open = new List<NodeRecord>();
            open.Add(startRec);
            List<NodeRecord> closed = new List<NodeRecord>();
            Vertex curry = start;
            NodeRecord current = startRec;

            bool onGoal = false;

            while (open.Count > 0 && !onGoal) {
                curry = open.Min(rec => rec.node);
                current = open.Find(rec => rec.node == curry);

                if (current.node == goal)
                {
                    onGoal = true;
                }
                else
                {
                    Vertex[] neighbours = GetNeighbours(current.node);
                    float[] neighCost = GetNeighboursCosts(current.node);

                    for (int i = 0; i < neighbours.Count(); i++)
                    {
                        NodeRecord endRec;

                        Vertex endNode = neighbours[i];
                        float endCost = current.costSoFar + neighCost[i];
                        float endHeuristic = -1;

                        //Comprueba si está en la lista de cerrados para ver si hay una ruta mejor
                        if (closed.Any(rec => rec.node == endNode))
                        {
                            endRec = closed.Find(rec => rec.node == endNode);

                            if (endCost < endRec.costSoFar)
                            {
                                closed.Remove(endRec);

                                endHeuristic = endNode.cost - endRec.costSoFar;
                            }
                        }
                        //Comprueba si está en la lista de abiertos y hay una ruta mejor
                        else if (open.Any(rec => rec.node == endNode))
                        {
                            endRec = open.Find(rec => rec.node == endNode);

                            if (endCost < endRec.costSoFar)
                            {
                                endHeuristic = endNode.cost - endRec.costSoFar;
                            }
                        }
                        else
                        {
                            endRec = new NodeRecord();
                            endRec.node = endNode;

                            endHeuristic = h(endNode, goal);
                        }

                        //Si la heuristica no es -1 signifíca que el camino es óptimo
                        if (endHeuristic >= 0)
                        {
                            endRec.costSoFar = endCost;
                            endRec.fromNode = current.node;
                            endNode.cost = endCost + endHeuristic;

                            //Comprueba que no esté ya en la lista de abiertos para añadirlo
                            if (!open.Any(rec => rec.node == endNode))
                            {
                                open.Add(endRec);
                            }
                        }
                    }

                    open.Remove(current);
                    closed.Add(current);
                }
            }

            if (current.node != goal)
            {
                return null;
            }
            else
            {
                List<Vertex> path = new List<Vertex>();

                while (current.node != start)
                {
                    path.Add(current.node);
                    current = closed.Find(rec => rec.node == current.fromNode);
                }

                path.Reverse();

                return path;
            }
        }

        //JOSE
        public bool RayClear(Vertex fromPt, Vertex toPt)
        {
            float rayDrawBroad = 0.2f; //Cambiar el valor del parametro puede interferir en si cabe o no el objeto por el camino,
                                       //asi q cuanqto mas pequeño es el valor, mejor puede sortear los obstaculos
            Vector3 direction = toPt.transform.position - fromPt.transform.position;
            float distance = direction.magnitude;


            Vector3 origin1 = fromPt.transform.position + Vector3.up * 0.5f
                + Vector3.forward * rayDrawBroad + Vector3.right * -rayDrawBroad;

            Vector3 origin2 = fromPt.transform.position + Vector3.up * 0.5f
                + Vector3.forward * -rayDrawBroad + Vector3.right * rayDrawBroad;

            Vector3 origin3 = fromPt.transform.position + Vector3.up * 0.5f
                + Vector3.forward * -rayDrawBroad + Vector3.right * -rayDrawBroad;

            Vector3 origin4 = fromPt.transform.position + Vector3.up * 0.5f
                + Vector3.forward * rayDrawBroad + Vector3.right * rayDrawBroad;



            Vector3 dir1 = (toPt.transform.position + Vector3.forward * rayDrawBroad
                + Vector3.right * -rayDrawBroad) - origin1;

            Vector3 dir2 = (toPt.transform.position + Vector3.forward * -rayDrawBroad
                + Vector3.right * rayDrawBroad) - origin2;

            Vector3 dir3 = (toPt.transform.position + Vector3.forward * -rayDrawBroad
                + Vector3.right * -rayDrawBroad) - origin3;

            Vector3 dir4 = (toPt.transform.position + Vector3.forward * rayDrawBroad
                + Vector3.right * rayDrawBroad) - origin4;


            return (!Physics.Raycast(origin1, dir1, distance) && !Physics.Raycast(origin2, dir2, distance) &&
                    !Physics.Raycast(origin3, dir3, distance) && !Physics.Raycast(origin4, dir4, distance));
        }

        public List<Vertex> Smooth(List<Vertex> inputPath)
        {
            if (inputPath.Count == 2) return inputPath; // No se puede suavizar porque es un camino pequeño

            List<Vertex> outputPath = new List<Vertex> { inputPath[0] };
            int inputIndex = 2;

            while (inputIndex < inputPath.Count - 1)
            {
                Vertex fromPt = outputPath[outputPath.Count - 1];
                Vertex toPt = inputPath[inputIndex];

                //Si en el camino hay obstaculos, no se recorta

                if (!RayClear(fromPt, toPt))
                {
                    outputPath.Add(inputPath[inputIndex - 1]);
                }

                inputIndex++;
            }

            outputPath.Add(inputPath[inputPath.Count - 1]);
            return outputPath;
        }

        // Reconstruir el camino, dando la vuelta a la lista de nodos 'padres' /previos que hemos ido anotando
        private List<Vertex> BuildPath(int srcId, int dstId, ref int[] prevList)
        {
            List<Vertex> path = new List<Vertex>();

            if (dstId < 0 || dstId >= vertices.Count) 
                return path;

            int prev = dstId;
            do
            {
                path.Add(vertices[prev]);
                prev = prevList[prev];
            } while (prev != srcId);
            return path;
        }

    }
}
