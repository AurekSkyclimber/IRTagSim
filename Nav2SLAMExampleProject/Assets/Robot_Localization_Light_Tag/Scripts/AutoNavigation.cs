using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class AutoNavigation : MonoBehaviour
{  
	const string k_RobotTag = "robot";

	const string k_RobotBaseName = "base_footprint/base_link";
	const string k_GoalPoseFrameId = "map";
	const string k_GoalPoseTopic = "/goal_pose";
	
	const float k_Nav2InitializeTime = 5.0f;
	const float k_SleepBetweenWaypointsTime = 2.0f;
	
	// Used to define a timeout for waypoint navigation based on distances between steps
	const float k_MinimumSpeedExpected = 0.15f;

	// How close the TurtleBot must get to the navigation target to be successful
	const float k_DistanceSuccessThreshold = 1f;
	
	Transform CurrentWaypoint => m_Waypoints[m_CurrentWaypointIdx];
    int WaypointCount => m_Waypoints.Count;
        
	const string k_WaypointTag = "Waypoint"; 
	List<Transform> m_Waypoints;
	int m_CurrentWaypointIdx;
   
     
   // Start is called before the first frame update
    void Start()
    {
        StartCoroutine(TurtleBotOnObstacleCourse_NavigateWaypoints_Succeeds());
    }

    // Update is called once per frame
    void OnDestroy()
    {
        StartCoroutine(TearDown());
    } 
    
	public IEnumerator TearDown()
	{
		ROSConnection.GetOrCreateInstance().Disconnect();
		yield return null;
	}
	static bool IsCloseEnough(Transform expected, Transform actual)
	{
		return (expected.position - actual.position).magnitude < k_DistanceSuccessThreshold;
	}

	static void ToRosMsg(Transform transform, out RosMessageTypes.Geometry.PoseMsg poseMsg)
	{
		poseMsg = new RosMessageTypes.Geometry.PoseMsg();
		poseMsg.position = transform.position.To<FLU>();
		poseMsg.orientation = transform.rotation.To<FLU>();
	}

	static RosMessageTypes.Geometry.PoseStampedMsg ToRosMsg(Transform transform)
	{
		ToRosMsg(transform, out var pose);
		var msg = new RosMessageTypes.Geometry.PoseStampedMsg
		{
			header =
			{
				stamp = new TimeStamp(Clock.time),
				frame_id = k_GoalPoseFrameId
			},
			pose = pose
		};
		return msg;
	}
	
	
	public IEnumerator TurtleBotOnObstacleCourse_NavigateWaypoints_Succeeds()
		
	{
		var ros = ROSConnection.GetOrCreateInstance();
		ros.ConnectOnStart = true;
		
		var robots = GameObject.FindGameObjectsWithTag(k_RobotTag);
		
		var robot = robots[0].transform.Find(k_RobotBaseName)?.gameObject;
		
		PathFinding path = GameObject.FindObjectOfType<PathFinding>();
		while(!path.WaypointsReady) {
			yield return null;
		}
		
		m_Waypoints = path.m_Waypoints;
		
		//~ var waypoints = new Waypoints();
		//~ WaypointTracker();
		
		
		// TODO: Implement some sort of confirmation mechanism on ROS side rather than use arbitrary sleep
		yield return new WaitForSeconds(k_Nav2InitializeTime);
		
		ros.RegisterPublisher<RosMessageTypes.Geometry.PoseStampedMsg>(k_GoalPoseTopic);

		while (NextWaypoint())
		{
			var timeNavigationStarted = Time.realtimeSinceStartup;
			var waypoint = CurrentWaypoint;
			var waypointTf = waypoint.transform;
			var robotTf = robot.transform;
			var distance = (waypointTf.position - robotTf.position).magnitude;
			var timeout = distance / k_MinimumSpeedExpected;
			
			Debug.Log("Next Destination is: " + waypointTf.position);
			ros.Send(k_GoalPoseTopic, ToRosMsg(waypointTf));

			yield return new WaitUntil(() => 
				IsCloseEnough(waypointTf, robotTf));// ||
				//Time.realtimeSinceStartup - timeNavigationStarted > timeout);
			
			
			// Because our success threshold may not match the navigation stack's success threshold, we "sleep" for
			// a small amount of time to ensure the nav stack has time to complete its route
			yield return new WaitForSeconds(k_SleepBetweenWaypointsTime);
		}
		
		
		yield return null;
	}
	
	private void WaypointTracker()
	{
		var waypoints = GameObject.FindGameObjectsWithTag(k_WaypointTag).ToList();
		waypoints.Sort((g, o) => string.Compare(g.name, o.name));
		m_Waypoints = waypoints.Select(w => w.transform).ToList();
		m_CurrentWaypointIdx = -1;
		if (m_Waypoints.Count == 0)
		{
			Debug.LogWarning(
				$"Found no GameObjects tagged with {k_WaypointTag} in {SceneManager.GetActiveScene().name}");
		}
	}

	private bool NextWaypoint()
	{
		m_CurrentWaypointIdx++;
		return m_CurrentWaypointIdx < m_Waypoints.Count;
	}

}
