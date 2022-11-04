using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
class Waypoints
    {
        internal Transform CurrentWaypoint => m_Waypoints[m_CurrentWaypointIdx];
        internal int Count => m_Waypoints.Count;
        
        const string k_WaypointTag = "Waypoint"; 
        List<Transform> m_Waypoints;
        int m_CurrentWaypointIdx;

        internal Waypoints()
        {
            var waypoints = GameObject.FindGameObjectsWithTag(k_WaypointTag).ToList();
            waypoints.Sort((g, o) => string.Compare(g.name, o.name));
            m_Waypoints = waypoints.Select(w => w.transform).ToList();
            m_CurrentWaypointIdx = -1;
            if (m_Waypoints.Count == 0)
            {
                Debug.Log("Somethings wrong");
            }
        }

        internal bool NextWaypoint()
        {
            m_CurrentWaypointIdx++;
            return m_CurrentWaypointIdx < m_Waypoints.Count;
        }
    }
