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
        }

        protected virtual int GridToId(int x, int y)
        {
            return 0;
        }

        protected virtual Vector2 IdToGrid(int id)
        {
            return new Vector2();
        }

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
            return new List<Vertex>();
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

            NodeRecord startRec = new NodeRecord();
            startRec.node = start;
            startRec.costSoFar = 0;

            List<NodeRecord> open = new List<NodeRecord>();
            open.Add(startRec);
            List<NodeRecord> closed = new List<NodeRecord>();

            while (open.Count > 0) {
                Vertex curry = open.Min(rec => rec.node);
                NodeRecord current = open.Find(rec => rec.node == curry);

                if (current.node = GetNearestVertex(dstO.transform.position))
                    break;

                Vertex[] neighbours = GetNeighbours(current.node);
                float[] neighCost = GetNeighboursCosts(current.node);
                Vector2 currentPos = IdToGrid(current.node.id);

                NodeRecord endRec;

                for (int i = 0; i < neighbours.Count(); i++)
                {
                    Vertex endNode = neighbours[i];
                    float endCost = current.costSoFar + neighCost[i];

                    if(closed.Any(rec => rec.node == endNode))
                    {
                        endRec = closed.Find(rec => rec.node == endNode);
                        Vector2 endPos = IdToGrid(endNode.id);

                        if (endCost < endRec.costSoFar)
                        {
                            closed.Remove(endRec);

                            //endNode.cost = neighCost[i] - costsVertices[(int)endPos.x, (int)endPos.y];
                        }
                    }
                    else if (open.Any(rec => rec.node == endNode))
                    {
                        endRec = open.Find(rec => rec.node == endNode);
                        Vector2 endPos = IdToGrid(endNode.id);
                        
                        if (endCost < endRec.costSoFar)
                        {
                            //endNode.cost = neighCost[i] - costsVertices[(int)endPos.x, (int)endPos.y];
                        }
                    }
                    else
                    {
                        endRec = new NodeRecord();
                        endRec.node = endNode;

                        //Llamada a la Heurística que no tengo preparada todavía
                        //endNode.cost = 
                    }

                    endRec

                }
            }


            return new List<Vertex>();
        }

        //JODER
        public List<Vertex> Smooth(List<Vertex> inputPath)
        {
            // IMPLEMENTAR SUAVIZADO DE CAMINOS

            List<Vertex> outputPath = new List<Vertex>();

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
