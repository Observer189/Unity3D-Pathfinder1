using System.Collections.Generic;
using Priority_Queue;
using UnityEngine;

namespace DefaultNamespace
{
    /// <summary>
    /// Статический класс для вызова поиска пути с поммощью алгоритма A*
    /// </summary>
    public class AstarPathfinder
    {
        public DiagonalPassingType DiagonalPassingType{ get; set; }
        public byte SearchClearance { get; set; } = 1;

        private Vector2Int[] directions = new[]
            { new Vector2Int(0, 1), new Vector2Int(0, -1), new Vector2Int(1, 0), new Vector2Int(-1, 0) };
        private Vector2Int[] diagonals = new[]
            { new Vector2Int(1, 1), new Vector2Int(1, -1), new Vector2Int(-1, 1), new Vector2Int(-1, -1) };
        private List<NeighborInfo> neighbors = new List<NeighborInfo>();
        private Vector2Int minBound;
        private Vector2Int maxBound;
        private FastPriorityQueue<AstarNode> openSet;
        private Dictionary<Vector2Int, AstarNode> visited;


        private delegate float HeuristicFunction(Vector2Int startPos, Vector2Int targetPos);

        private HeuristicFunction heuristic;

        private PathNode[,] _grid;
        public void Allocate(Vector2Int mapSize)
        {
            openSet = new FastPriorityQueue<AstarNode>(Mathf.Max(mapSize.x
                , mapSize.y)*100);
            visited = new Dictionary<Vector2Int, AstarNode>(Mathf.Max(mapSize.x
                , mapSize.y)*100);
        }

        public Path<Vector2Int> Pathfind(Vector2Int startPos, Vector2Int targetPos,  PathNode[,] grid,
            int minX = 0, int minY = 0, int maxX = -1, int maxY = -1, bool debug = false)
        {
            openSet.Clear();
            visited.Clear();

            _grid = grid;
            
            minBound = Vector2Int.zero;
            minBound.x = minX;
            minBound.y = minY;

            int openCount = 0;

            maxBound = new Vector2Int(grid.GetLength(0), grid.GetLength(1));
            if (maxX != -1)
            {
                maxBound.x = maxX;
                maxBound.y = maxY;
            }
 
            //heuristic = (DiagonalPassingType == DiagonalPassingType.NoPassing)?ManhattanDistance:EuclideanDistance;
            heuristic = EuclideanDistance;

            var startNode = new AstarNode {position = startPos,cost = 0, prev = null};
            openSet.Enqueue(startNode,0);
            visited[startPos] = startNode;
           
            while (openSet.Count > 0)
            {
                openCount++;
                var cur = openSet.Dequeue();
                if(debug)
                Debug.Log($"cur = {cur.position}");
                //Debug.Log(cur.position);
                if (cur.position == targetPos)
                {
                    var resList = new List<Vector2Int>();
                    float cost = cur.cost;
                    while (true)
                    {
                        resList.Add(cur.position);
                        if (cur.prev == null)
                        {
                            Debug.Log($"open count = {openCount}");
                            return new Path<Vector2Int>(resList,cost);
                        }
                        else
                        {
                            cur = cur.prev;
                        }
                    }
                }
                var neighbors = GetNeighbors(cur.position,grid);
                foreach (var neighbor in neighbors)
                {
                    if(debug)
                    Debug.Log($"neighbor = {neighbor.position}");
                    if (visited.TryGetValue(neighbor.position,out AstarNode n))
                    {
                        if (n.cost > cur.cost + neighbor.cost)
                        {
                            n.cost = cur.cost + neighbor.cost;
                            n.prev = cur;
                            var f = n.cost + heuristic(n.position,targetPos);
                            openSet.UpdatePriority(n,f);
                        }
                    }
                    else
                    {
                        var node = new AstarNode() {  position = neighbor.position, prev = cur,cost = (cur.cost+ neighbor.cost)};
                        visited[neighbor.position] = node;
                        openSet.Enqueue(node, cur.cost + neighbor.cost + heuristic(neighbor.position,targetPos));
                    }
                }
            }
            return null;
        }
        List<NeighborInfo> GetNeighbors(Vector2Int pos, PathNode[,] grid)
       {
           neighbors.Clear();
           for (int i = 0; i < directions.Length; i++)
           {
               NeighborInfo neighbor = new NeighborInfo()
               {
                   position = pos + directions[i],
               };
               
               if (neighbor.position.x >= minBound.x && neighbor.position.x < maxBound.x && neighbor.position.y >= minBound.y &&
                   neighbor.position.y < maxBound.y && grid[neighbor.position.x, neighbor.position.y].walkable)
               {
                   neighbor.cost = (grid[neighbor.position.x, neighbor.position.y].worldPosition - grid[pos.x,pos.y].worldPosition).magnitude;
                   neighbors.Add(neighbor);
               }
           }

           if (DiagonalPassingType != DiagonalPassingType.NoPassing)
           {
               for (int i = 0; i < diagonals.Length; i++)
               {
                   NeighborInfo neighbor = new NeighborInfo()
                   {
                       position = pos + diagonals[i],
                   };
                   if (neighbor.position.x >= minBound.x && neighbor.position.x < maxBound.x && neighbor.position.y >= minBound.y &&
                       neighbor.position.y < maxBound.y && grid[neighbor.position.x, neighbor.position.y].walkable)
                   {
                       if (DiagonalPassingType == DiagonalPassingType.PassWhenNoObstacles)
                       {
                           if (!grid[pos.x, pos.y + diagonals[i].y].walkable || !grid[pos.x + diagonals[i].x, pos.y].walkable)
                           {
                               continue;
                           }
                       }
                       else if(DiagonalPassingType == DiagonalPassingType.PassWhenLessThanTwoObstacles)
                       {
                           if (!grid[pos.x, pos.y + diagonals[i].y].walkable && !grid[pos.x + diagonals[i].x, pos.y].walkable)
                           {
                               continue;
                           }
                       }

                       neighbor.cost = neighbor.cost = (grid[neighbor.position.x, neighbor.position.y].worldPosition - grid[pos.x,pos.y].worldPosition).magnitude;;
                       neighbors.Add(neighbor);
                   }
               }
           }

           //Debug.Log($"n count = {neighbors.Count}");
           return neighbors;
       }
        
        private float EuclideanDistance(Vector2Int startPos, Vector2Int targetPos)
        {
            return (_grid[startPos.x, startPos.y].worldPosition - _grid[targetPos.x, targetPos.y].worldPosition).magnitude;
        }


    }
}

public class AstarNode:FastPriorityQueueNode
{
    public Vector2Int position;
    public float cost;
    public AstarNode prev;
}

public struct NeighborInfo
{
    public Vector2Int position;
    public float cost;
    public PathNode node;
}

public enum DiagonalPassingType
{
    NoPassing, PassAlways, PassWhenNoObstacles, PassWhenLessThanTwoObstacles
}