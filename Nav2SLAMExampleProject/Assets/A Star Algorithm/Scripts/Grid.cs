using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Grid : MonoBehaviour
{
    public Transform StartPosition;
    public LayerMask WallMask;
    public Vector2 gridWorldSize;//A vector2 to store the width and height of the graph in world units.
    public float nodeRadius;
    public float Distance;
    
    
    Node[,] grid;
    public List<Node> FinalPath;

    float nodeDiameter;
    int gridSizeX, gridSizeY;

    private void Awake()
    {
        nodeDiameter = nodeRadius * 2;
        gridSizeX = Mathf.RoundToInt(gridWorldSize.x / nodeDiameter);
        gridSizeY = Mathf.RoundToInt(gridWorldSize.y / nodeDiameter);
        CreateGrid();
    }
    void CreateGrid(){

        grid = new Node[gridSizeX, gridSizeY];
        Vector3 bottomLeft = transform.position - Vector3.right * gridWorldSize.x / 2 - Vector3.forward * gridWorldSize.y / 2;
        for (int y=0;y<gridSizeY; y++)
        {
            for (int x = 0; x < gridSizeX; x++)
            {
                Vector3 worldPoint = bottomLeft + Vector3.right * (x * nodeDiameter + nodeRadius) + Vector3.forward * (y * nodeDiameter + nodeRadius);//Get the world co ordinates of the bottom left of the graph
                bool Wall = true;//Make the node a wall

                //If the node is not being obstructed
                //Quick collision check against the current node and anything in the world at its position. If it is colliding with an object with a WallMask,
                //The if statement will return false.
                if (Physics.CheckSphere(worldPoint, nodeRadius, WallMask))
                {
                    Wall = false;//Object is not a wall
                }

                grid[x, y] = new Node(Wall, worldPoint, x, y);//Create a new node in the array.

            }

        }

    }


    //Function that gets the neighboring nodes of the given node.
    public List<Node> GetNeighboringNodes(Node a_NeighborNode)
    {
        List<Node> NeighborList = new List<Node>();//Make a new list of all available neighbors.
        int checkX;//Variable to check if the XPosition is within range of the node array to avoid out of range errors.
        int checkY;//Variable to check if the YPosition is within range of the node array to avoid out of range errors.

        //Check the right side of the current node.
        checkX = a_NeighborNode.gridX + 1;
        checkY = a_NeighborNode.gridY;
        if (checkX >= 0 && checkX < gridSizeX)//If the XPosition is in range of the array
        {
            if (checkY >= 0 && checkY < gridSizeY)//If the YPosition is in range of the array
            {
                NeighborList.Add(grid[checkX, checkY]);//Add the grid to the available neighbors list
            }
        }
        //Check the Left side of the current node.
        checkX = a_NeighborNode.gridX - 1;
        checkY = a_NeighborNode.gridY;
        if (checkX >= 0 && checkX < gridSizeX)//If the XPosition is in range of the array
        {
            if (checkY >= 0 && checkY < gridSizeY)//If the YPosition is in range of the array
            {
                NeighborList.Add(grid[checkX, checkY]);//Add the grid to the available neighbors list
            }
        }
        //Check the Top side of the current node.
        checkX = a_NeighborNode.gridX;
        checkY = a_NeighborNode.gridY + 1;
        if (checkX >= 0 && checkX < gridSizeX)//If the XPosition is in range of the array
        {
            if (checkY >= 0 && checkY < gridSizeY)//If the YPosition is in range of the array
            {
                NeighborList.Add(grid[checkX, checkY]);//Add the grid to the available neighbors list
            }
        }
        //Check the Bottom side of the current node.
        checkX = a_NeighborNode.gridX;
        checkY = a_NeighborNode.gridY - 1;
        if (checkX >= 0 && checkX < gridSizeX)//If the XPosition is in range of the array
        {
            if (checkY >= 0 && checkY < gridSizeY)//If the YPosition is in range of the array
            {
                NeighborList.Add(grid[checkX, checkY]);//Add the grid to the available neighbors list
            }
        }

        return NeighborList;//Return the neighbors list.
    }

    //Gets the closest node to the given world position.
    public Node NodeFromWorldPoint(Vector3 a_vWorldPos)
    {
        float ixPos = ((a_vWorldPos.x + gridWorldSize.x / 2) / gridWorldSize.x);
        float iyPos = ((a_vWorldPos.z + gridWorldSize.y / 2) / gridWorldSize.y);

        ixPos = Mathf.Clamp01(ixPos);
        iyPos = Mathf.Clamp01(iyPos);

        int ix = Mathf.RoundToInt((gridSizeX - 1) * ixPos);
        int iy = Mathf.RoundToInt((gridSizeY - 1) * iyPos);

        return grid[ix, iy];
    }

    //Function that draws the wireframe
    private void OnDrawGizmos()
    {

        Gizmos.DrawWireCube(transform.position, new Vector3(gridWorldSize.x, 1, gridWorldSize.y));//Draw a wire cube with the given dimensions from the Unity inspector

        if (grid != null)//If the grid is not empty
        {
            foreach (Node n in grid)//Loop through every node in the grid
            {
                if (n.IsWall)//If the current node is a wall node
                {
                    Gizmos.color = Color.white;//Set the color of the node
                }
                else
                {
                    Gizmos.color = Color.yellow;//Set the color of the node
                }


                if (FinalPath != null)//If the final path is not empty
                {
                    if (FinalPath.Contains(n))//If the current node is in the final path
                    {
                        Gizmos.color = Color.red;//Set the color of that node
                    }

                }


                Gizmos.DrawCube(n.Position, Vector3.one * (nodeRadius - Distance));//Draw the node at the position of the node.
            }
        }
    }
}
