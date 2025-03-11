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

            List<Vertex> open = new List<Vertex>();
            open.Add(start);
            List<Vertex> closed = new List<Vertex>();

            while (open.Count > 0) {
                Vertex current = open.Min();

                if (current = GetNearestVertex(dstO.transform.position))
                    break;

                Vertex[] neighbours = GetNeighbours(current);
                float[] neighCost = GetNeighboursCosts(current);
                Vector2 currentPos = IdToGrid(current.id);

                for (int i = 0; i < neighbours.Count(); i++)
                {
                    Vertex endNode = neighbours[i];
                    float endCost = costsVertices[(int)currentPos.x, (int)currentPos.y] + neighCost[i];

                    if(closed.Contains(endNode))
                    {
                        Vector2 endPos = IdToGrid(endNode.id);

                        if (endCost < costsVertices[(int)endPos.x, (int)endPos.y])
                        {
                            closed.Remove(endNode);

                            endNode.cost = neighCost[i] - costsVertices[(int)endPos.x, (int)endPos.y];
                        }
                    }
                    else if (open.Contains(endNode))
                    {
                        Vector2 endPos = IdToGrid(endNode.id);
                        
                        if (endCost <= costsVertices[(int)endPos.x, (int)endPos.y])
                        {
                            endNode.cost = neighCost[i] - costsVertices[(int)endPos.x, (int)endPos.y];
                        }
                    }
                    else
                    {
                        //Llamada a la Heurística que no tengo preparada todavía
                        //endNode.cost = 
                    }
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
