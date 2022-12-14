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
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;

public class TurtleController : MonoBehaviour
{
    
    public GameObject WaypointPrefab;
    
    float rayLength = 5f;
    
    //Make a list that way waypoint class can access this 
    public List<Transform> Waypoint_Nodes = new List<Transform>();
    Vector3 noAngle;
        
    LineRenderer trail;
    
    const string k_RobotTag = "robot";

	const string k_RobotBaseName = "base_footprint/base_link";
	const string k_GoalPoseFrameId = "map";
	const string k_GoalPoseTopic = "/goal_pose";
	const string k_OdomTopic = "/odom";
	const string k_ScanTopic = "/scan";
	
	const float k_Nav2InitializeTime = 5.0f;
	const float k_SleepBetweenWaypointsTime = 2.0f;
	
	// Used to define a timeout for waypoint navigation based on distances between steps
	const float k_MinimumSpeedExpected = 0.15f;

	// How close the TurtleBot must get to the navigation target to be successful
	const float k_DistanceSuccessThreshold = 0.5f;
        
	const string k_WaypointTag = "Waypoint"; 
	List<Transform> m_Waypoints;
	int m_CurrentWaypointIdx;
	
	bool m_RosConnected = false;
	bool m_WaypointsReady = false;
   
   	int m_NextTag = -1;
   	
   	Transform robot;
   	Vector3 realPos;
	Quaternion realRot;
	float[] realRanges = new float[0];
	
	private const int numSensors = 8;
	public Transform[] sensorPoses = new Transform[numSensors];
   
	string guiMessage = "";
	
	public GameObject rangeVisualizationCubePrefab;
	private List<GameObject> rangeVisualizationCubes = new List<GameObject>();
	private List<MeshRenderer> rangeVisualizationRenderers = new List<MeshRenderer>();
   
    // Start is called before the first frame update
    void Start()
    {
        noAngle = this.transform.right;
        trail = this.GetComponent<LineRenderer>();
        
        robot = transform.Find(k_RobotBaseName);
        
        StartCoroutine(TurtleBotOnObstacleCourse_NavigateWaypoints_Succeeds());
    }

    // Update is called once per frame
    void Update()
    {
		Debug.Log("Real: " + realPos + " | " + realRot);
		
		int numRanges = realRanges.Length;
		if(numRanges > 0) { // Visualize Ranges
			while(rangeVisualizationCubes.Count < numRanges) {
				GameObject cube = Instantiate(rangeVisualizationCubePrefab);
				rangeVisualizationCubes.Add(cube);
				rangeVisualizationRenderers.Add(cube.GetComponent<MeshRenderer>());
			}

			float rotAngleStep = -360f / (float) numRanges;
			for(int i = 0; i < numRanges; i++) {
				rangeVisualizationCubes[i].SetActive(true);
				rangeVisualizationCubes[i].transform.position = robot.position + (Quaternion.Euler(0, rotAngleStep * (float) i, 0) * robot.forward * realRanges[i]);
				rangeVisualizationRenderers[i].material.color = Color.HSVToRGB(Mathf.Clamp01(realRanges[i] / 50f), 1, 1);
			}
			for(int i = numRanges; i < rangeVisualizationCubes.Count; i++) {
				rangeVisualizationCubes[i].SetActive(false);
			}
		}
    }
    
	// Update is called once per frame
    void OnDestroy()
    {
        StartCoroutine(TearDown());
    } 
    
	public IEnumerator TearDown()
	{
		ROSConnection.GetOrCreateInstance().Disconnect();
		m_RosConnected = false;
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
	
	public void NewWaypoints(Light_Tag tag) {
		if(tag.NextTag.TagID > m_NextTag) {
			List<Transform> path = tag.NextWaypoints.ToList();
			path.Insert(0, robot);
			DrawPath(trail,path);

			float tagIntensity = tag.GetComponent<Light>().intensity;
			float[] RSSI = new float[numSensors];
			float biggestRSSI = -1;
			int biggestIndex = 0;
			for(int i = 0; i < numSensors; i++) {
				float rssi = tagIntensity / (Vector3.Distance(sensorPoses[i].position, tag.transform.position));
				RSSI[i] = rssi;
				if(rssi > biggestRSSI) {
					biggestRSSI = rssi;
					biggestIndex = i;
				}
			}
			float[] tagRssDirection = RssAlgorithm(biggestIndex * 45, RSSI[(biggestIndex-1) % numSensors], RSSI[biggestIndex], RSSI[(biggestIndex+1) % numSensors]);
			guiMessage = "Tag ID: " + tag.TagID + "\n\nClosest Sensor: " + biggestIndex + "\n\nTag direction local: " + tagRssDirection[1] + " degrees\n\nRobot rotation: " + robot.rotation.eulerAngles + "\n\nTag intensity: " + tagRssDirection[0];
			Debug.Log(guiMessage);

			m_NextTag = tag.NextTag.TagID;
			m_Waypoints = tag.NextWaypoints.ToList();
			m_CurrentWaypointIdx = -1;
			m_WaypointsReady = true;
		}
	}
	
	
	
	
	
	void OnGUI()
    {
        //~ Tag.range = GUI.HorizontalSlider(new Rect(25, 25, 100, 30), Tag.range, originalRange, 100f);
        GUI.Label(new Rect(Screen.width - 230,Screen.height - 230,200,200), guiMessage);
    }
	
	public float[] RssAlgorithm(float baseRot, float rssi_l, float rssi_c, float rssi_r){
		float a= (((rssi_l+rssi_r)+(2f*rssi_c))/(2f*Mathf.Cos(Mathf.PI/4f)+2f));
		float b= ((rssi_r-rssi_l)/(2*Mathf.Sin(Mathf.PI/4)));
		float r= Mathf.Sqrt(a*a+b*b);
		float theta = baseRot + (Mathf.Atan2(a, b)*Mathf.Rad2Deg);
		// TODO: Lookup table to convert RSSI into Distance
		//float Distance = r;
	
		return new float[] {r, theta};
    }
	
	private Transform CurrentWaypoint() {
		if(m_CurrentWaypointIdx == -1) {
			m_CurrentWaypointIdx = 0;
		}
		return m_Waypoints[m_CurrentWaypointIdx];
	}
    int WaypointCount => m_Waypoints.Count;
	
	private bool NextWaypoint()
	{
		m_CurrentWaypointIdx++;
		return m_CurrentWaypointIdx < m_Waypoints.Count;
	}
	
	public IEnumerator TurtleBotOnObstacleCourse_NavigateWaypoints_Succeeds()
	{
		var ros = ROSConnection.GetOrCreateInstance();
		ros.ConnectOnStart = true;
		
		// TODO: Implement some sort of confirmation mechanism on ROS side rather than use arbitrary sleep
		yield return new WaitForSeconds(k_Nav2InitializeTime);
		
		ros.RegisterPublisher<RosMessageTypes.Geometry.PoseStampedMsg>(k_GoalPoseTopic);
		//~ ros.Subscribe<RosMessageTypes.Nav.OdometryMsg>(k_OdomTopic, OdomCallback);
		ros.Subscribe<RosMessageTypes.Sensor.LaserScanMsg>(k_ScanTopic, ScanCallback);
		
		m_RosConnected = true;

		while(m_RosConnected) {
			if(m_CurrentWaypointIdx > -1) {
				Debug.Log(m_WaypointsReady + " | " + IsCloseEnough(CurrentWaypoint(), robot) + " | " + CurrentWaypoint().position + " | " + robot.position);
			}
			if(m_WaypointsReady && (m_CurrentWaypointIdx == -1 || IsCloseEnough(CurrentWaypoint(), robot))) {
				Debug.Log(m_CurrentWaypointIdx + " | " + m_Waypoints.Count);
				if(NextWaypoint()) {
					Debug.Log(m_CurrentWaypointIdx + " | " + m_Waypoints.Count);
					// Because our success threshold may not match the navigation stack's success threshold, we "sleep" for
					// a small amount of time to ensure the nav stack has time to complete its route
					yield return new WaitForSeconds(k_SleepBetweenWaypointsTime);
					Debug.Log("Before Next Destination");
					Debug.Log("Next Destination is: " + CurrentWaypoint().position);
					Debug.Log("After Next Destination");
					ros.Send(k_GoalPoseTopic, ToRosMsg(CurrentWaypoint()));
				} else {
					m_WaypointsReady = false;
				}
			}
			
			yield return null;
		}
		
		yield return null;
	}
	
	void OdomCallback(RosMessageTypes.Nav.OdometryMsg msg) {
		realPos = PointMsgToVector3(msg.pose.pose.position);
		realRot = QuaternionMsgToQuaternion(msg.pose.pose.orientation);
	}
	
	void ScanCallback(RosMessageTypes.Sensor.LaserScanMsg msg) {
		realRanges = msg.ranges;
	}
	
	Vector3 PointMsgToVector3(PointMsg msg) {
		return new Vector3((float) msg.x, (float) msg.y, (float) msg.z);
	}
	
	Quaternion QuaternionMsgToQuaternion(QuaternionMsg msg) {
		return new Quaternion((float) msg.x, (float) msg.y, (float) msg.z, (float) msg.w);
	}

    
		
	public void DrawPath(LineRenderer lr,List<Transform> points){		
		 lr.SetColors(Color.blue, Color.blue);
		 lr.SetWidth(0.1f, 0.1f);		 
		 int n = points.Count; // + 1;
         Vector3[] pointLine = new Vector3[n];         
			 
		 for (int i = 0; i < (n/* - 1*/); i++)
		 {
			 pointLine[i] = points[i].position;
			
		 }         
        
		lr.positionCount = n;
		lr.SetPositions(pointLine);        
         
		
	}		
		
	
}
