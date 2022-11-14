using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Node{

    public int gridX;
    public int gridY;

    public bool IsWall;
    public Vector3 Position;

    public Node Parent;

    public int gCost;//Distance from the starting Node
    public int hCost;//Distance from the ending Node(Manhattan Distaance)

    public int FCost { get { return gCost + hCost; } }

    public Node(bool a_IsWall, Vector3 a_Pos, int a_gridX, int a_gridY)
    {

        IsWall = a_IsWall;
        Position = a_Pos;
        gridX = a_gridX;
        gridY = a_gridY;

    }
}
