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
using Unity.Robotics.UrdfImporter.Control;


public class Explorer_Mover : MonoBehaviour
{
	public int Id;

	public GameObject wheel1;
	public GameObject wheel2;

	private ArticulationBody wA1;
	private ArticulationBody wA2;

	public float maxLinearSpeed = 2; //  m/s
	public float maxRotationalSpeed = 1;//
	public float wheelRadius = 0.033f; //meters
	public float trackWidth = 0.288f; // meters Distance between tyres
	public float forceLimit = 10;
	public float damping = 10;

	public float ROSTimeout = 0.5f;
	private float lastCmdReceived = 0f;

	ROSConnection ros;
	private RotationDirection direction;
	private float rosLinear = 0f;
	private float rosAngular = 0f;

	//check if the bot is colliding with a wall currently
	bool isCurrentlyColliding;

	//Access Laser Script
	GameObject basescan;
	LaserScanSensor laserscandetails;
	float[] raymeasurement;
	float[] raydistance;

	RaycastHit[] Ray_foundValidMeasurementement;
	List<float> Dist = new List<float>();
	bool fDetected;

	const float k_Nav2InitializeTime = 5.0f;
	bool m_RosConnected = false;
	Transform robot;
	const string k_RobotBaseName = "base_footprint/base_link";

	public string ScanTopic = "/scan";

	float[] realRanges = new float[0];
	float[] realAngles = new float[0];
	float[] distances = new float[0];

	public GameObject rangeVisualizationCubePrefab;
	private List<GameObject> rangeVisualizationCubes = new List<GameObject>();
	private List<MeshRenderer> rangeVisualizationRenderers = new List<MeshRenderer>();
	public GameObject goalVisualizationCapsulePrefab;
	private Transform goalVisualizer = null;

	public float avgCellWidth = 1.4f;
	public float avgWallWidth = 0.4f;
	private float avgDistCellCenterToCellCenter;
	private float avgThreeQuarterCellWidth;
	private float avgHalfCellWidth;
	private float avgQuarterCellWidth;

	private bool isTurning = false;
	private bool isStuck = false;
	// Starting at East because atan2 returns 0 for East. Also, rotates left because Lidar rotates left.
	public const int numCardinalDirections = 4;
	public enum Direction {East = 0, North = 1, West = 2, South = 3}
	public Vector3[] CardinalDirectionVectors = new Vector3[] {Vector3.right, Vector3.forward, Vector3.left, Vector3.back};
	public enum LocalDirection {Forward = 0, Left = 1, Back = 2, Right = 3}
	// Make sure to update this public var if you change the starting rotation of the actual robot in the scene view
	public Direction currentPrimaryDirection = Direction.North;

	private enum NodeState {New = 0, VisitedByOther = 1, VisitedByMe = 2, DeadEnd = 3}
	private Color[] NodeStateColors = new Color[] {Color.white, Color.red, Color.blue, Color.black};
	private Vector2Int[] NodeDirectionVectors = new Vector2Int[] {Vector2Int.right, Vector2Int.up, Vector2Int.left, Vector2Int.down};
	private const int numNodesPerSide = 10;
	private NodeState[,] nodes = new NodeState[numNodesPerSide,numNodesPerSide];
	public Vector2Int currentNode = Vector2Int.zero;
	public Vector2Int nextNode = Vector2Int.zero;
	// (0,0) is the bottom left corner, (9,9) is the top right, coords are (column, row)
	public Vector3 startCoord;
	public Vector3 cornerOffset;
	public MeshRenderer[,] nodeStateVisualizers = new MeshRenderer[numNodesPerSide,numNodesPerSide];
	public bool CreateNodeStateVisualizers = false;
	public Explorer_Mover NodeStateVisualizerSource;

	public LaserScanSensor sensor;
	private Light_Tag nearbyTag = null;
	LineRenderer trail;
	public Color TagTalkColor;
	private float timeLastTalked;
	private float tagTalkFadeTime = 2;

	public Vector3 goalPos;

	public float StartDelay = 0;

	private void Awake() {
		avgDistCellCenterToCellCenter = avgCellWidth + avgWallWidth;
		avgThreeQuarterCellWidth = avgCellWidth * 0.75f;
		avgHalfCellWidth = avgCellWidth * 0.5f;
		avgQuarterCellWidth = avgCellWidth * 0.25f;
		// bottomRightCornerOffset = new Vector3(avgQuarterCellWidth, 0, -avgQuarterCellWidth);
		trail = this.GetComponent<LineRenderer>();

		for(int i = 0; i < numNodesPerSide; i++) {
			for(int j = 0; j < numNodesPerSide; j++) {
				nodes[i,j] = NodeState.New;
				if(CreateNodeStateVisualizers) {
					nodeStateVisualizers[i,j] = Instantiate(goalVisualizationCapsulePrefab, new Vector3(startCoord.x + i * avgDistCellCenterToCellCenter, 0, startCoord.z + j * avgDistCellCenterToCellCenter), Quaternion.identity).GetComponent<MeshRenderer>();
					nodeStateVisualizers[i,j].material.color = NodeStateColors[(int) nodes[i,j]];
				}
			}
		}
		nodes[currentNode.x,currentNode.y] = NodeState.VisitedByMe;
		if(CreateNodeStateVisualizers) {
			nodeStateVisualizers[currentNode.x,currentNode.y].material.color = NodeStateColors[(int) NodeState.VisitedByMe];
		}
		goalPos = startCoord + cornerOffset * avgQuarterCellWidth;
		Debug.Log("GOAL POS: " + goalPos);
	}

	// Start is called before the first frame update
	void Start()
	{
		wA1 = wheel1.GetComponent<ArticulationBody>();
		wA2 = wheel2.GetComponent<ArticulationBody>();
		SetParameters(wA1);
		SetParameters(wA2);

		if(!CreateNodeStateVisualizers) {
			nodeStateVisualizers = NodeStateVisualizerSource.nodeStateVisualizers;
		}

		// // Comment this next line out when you do the real control function
		// StartCoroutine(ExampleCornerTurner());

		// Start the Control Function
		robot = transform.Find(k_RobotBaseName);
		StartCoroutine(ControlFunction());
	}

	void FixedUpdate()
	{
		// Uncomment the next line if you want the robot to always move and never time out
		// lastCmdReceived = Time.time;
		ROSUpdate();
	}

	// IEnumerator ExampleCornerTurner() {
	// 	var ros = ROSConnection.GetOrCreateInstance();
	// 	// ros.ConnectOnStart = true;
	// 	// ros.Subscribe<RosMessageTypes.Sensor.LaserScanMsg>(k_ScanTopic, ScanCallback);


	// 	// This isn't a real way to navigate the maze.
	// 	// However, it does demonstrate how to move foward, how to turn in place, and how to stop.
	// 	// Look at the TODOs in ControlFunction to see the real way forward
	// 	// Make sure to comment this out when you're ready to run the real code in Control Function!!!

	// 	rosLinear = 0.5f;
	// 	rosAngular = 0.0f;
	// 	yield return new WaitForSeconds(4f);
	// 	rosLinear = 0.0f;
	// 	rosAngular = 0.5f;
	// 	yield return new WaitForSeconds(4f);
	// 	rosLinear = 0.5f;
	// 	rosAngular = 0.0f;
	// 	yield return new WaitForSeconds(6f);
	// 	rosLinear = 0.0f;
	// 	rosAngular = 0.5f;
	// 	yield return new WaitForSeconds(2f);
	// 	rosLinear = 0.5f;
	// 	rosAngular = 0.0f;
	// 	yield return new WaitForSeconds(2f);
	// 	rosLinear = 0.0f;
	// 	rosAngular = -0.5f;
	// 	yield return new WaitForSeconds(3f);
	// 	rosLinear = 0.5f;
	// 	rosAngular = 0.0f;
	// 	yield return new WaitForSeconds(4f);
	// 	rosLinear = 0.0f;
	// 	rosAngular = 0.0f;
	// }

	void ScanCallback(RosMessageTypes.Sensor.LaserScanMsg msg) {
		realRanges = msg.ranges;
		realAngles = sensor.angles.ToArray();
	}


	IEnumerator ControlFunction(){
		var ros = ROSConnection.GetOrCreateInstance();
		ros.ConnectOnStart = true;
		
		// TODO: Implement some sort of confirmation mechanism on ROS side rather than use arbitrary sleep
		yield return new WaitForSeconds(k_Nav2InitializeTime);
		
		// ros.RegisterPublisher<RosMessageTypes.Geometry.PoseStampedMsg>(k_GoalPoseTopic);
		//~ ros.Subscribe<RosMessageTypes.Nav.OdometryMsg>(k_OdomTopic, OdomCallback);
		ros.Subscribe<RosMessageTypes.Sensor.LaserScanMsg>(ScanTopic, ScanCallback);
		
		m_RosConnected = true;

		yield return new WaitForSeconds(StartDelay);

		while(m_RosConnected) {
			int numRanges = realRanges.Length;

			// Debug.Log("Laser Ranges Total Count: " + numRanges);
			// for(int i = 0; i < numRanges; i++) {
				// Debug.Log("Laser Range " + i + ": " + realRanges[i]);
				// Debug.Log("Laser Angle " + i + ": " + realAngles[i]);
			// }

			if(Time.time - timeLastTalked > tagTalkFadeTime) {
				trail.enabled = false;
			}

			if(numRanges > 0) {
				if(goalVisualizer == null) {
					goalVisualizer = Instantiate(goalVisualizationCapsulePrefab).transform;
					goalVisualizer.position = goalPos;
				}

				while(rangeVisualizationCubes.Count < numRanges) {
					GameObject cube = Instantiate(rangeVisualizationCubePrefab);
					rangeVisualizationCubes.Add(cube);
					rangeVisualizationRenderers.Add(cube.GetComponent<MeshRenderer>());
				}

				int[] cardinalDirectionIndices = new int[numCardinalDirections];
				float[] cardinalDirectionRanges = new float[numCardinalDirections];
				Vector3[] cardinalDirectionHits = new Vector3[numCardinalDirections];
				float rotAngleStep = (-360f / (float) numRanges);
				float closestDistance = 99999;
				for(int i = 0; i < numRanges; i++) {
					// float distance = realRanges[i];
					if(realRanges[i] < closestDistance) {
						closestDistance = realRanges[i];
					}

					float LaserAngle = rotAngleStep * (float) i;
					Vector3 hitPosition = robot.position + (Quaternion.Euler(0, LaserAngle, 0) * robot.forward * realRanges[i]);

					// Debug.Log("Laser Angle: " + LaserAngle);
					// Debug.Log("Hit Position: " + position);

					// Visualizer Cubes
					rangeVisualizationCubes[i].SetActive(true);
					rangeVisualizationCubes[i].transform.position = hitPosition;
					rangeVisualizationRenderers[i].material.color = Color.HSVToRGB(Mathf.Clamp01(realRanges[i] / 50f), 1, 1);

						// if (LaserAngle<= 180)
						// 	rosLinear = 0.5f;
						// 	rosAngular = 0.0f;
						// if (LaserAngle <= 90)
						// 	rosLinear = 0.0f;
						// 	rosAngular = 0.5f;
						// if (LaserAngle <= 45)
						// 	rosLinear = 0.0f;
						// 	rosAngular = -0.5f;
					
					// Find the closest reading to the world coordinate directions
					float worldAngle = Mathf.Atan2(hitPosition.z - robot.position.z, hitPosition.x - robot.position.x) * Mathf.Rad2Deg;
					// Debug.Log("World Angle: " + worldAngle);
					float worldMod90Angle = Modulus(worldAngle, 90);
					// Debug.Log("World Angle mod 90: " + worldMod90Angle);
					if(worldMod90Angle < 5 || worldMod90Angle > 85) {
						// Debug.Log("World Angle: Effectively a 90 multiple");
						for(int j = 0; j < numCardinalDirections; j++) {
							if(Mathf.Abs((90 * j) - Modulus(worldAngle, 360)) < 5 ) {
								// Debug.Log("World Angle: It was " + (Direction) j);
								cardinalDirectionIndices[j] = i;
								cardinalDirectionRanges[j] = realRanges[i];
								cardinalDirectionHits[j] = hitPosition;
							}
						}
					}
				}
				float robotWorldForwardAngleRad = Mathf.Atan2(robot.forward.z, robot.forward.x);

				// Keep this. Deactivates any extra visualization cubes.
				for(int i = numRanges; i < rangeVisualizationCubes.Count; i++) {
					rangeVisualizationCubes[i].SetActive(false);
				}

				// Get the eight indices that point forward, sides, back, and the corners
				// Starts forward and then rotates left by 45 degrees at each step
				// [forward, forward-left, left, back-left, back, back-right, right, forward-right]
				float rotIndexStepToImportantReadings = (float) numRanges / 8f;
				int[] rotIndices = new int[8];
				for(int i = 0; i < 8; i++) {
					rotIndices[i] = (int) (rotIndexStepToImportantReadings * i);
				}

				// TODO: Analyze the scan ranges and figure out where the walls are

				if(isTurning) { // The robot is currently turning.
					if(isStuck) {
						if(closestDistance < 0.25f) { // Too close to a wall! (Assuming it's a wall in front)
							rosLinear = -0.5f;
							rosAngular = 0f;
						} else {
							isStuck = false;
						}
					} else {
						// currentPrimaryDirection = ???
						// Debug.Log("TURNING");
						rosLinear = 0f;
						float goalAngleRad = Mathf.Atan2(goalPos.z - robot.position.z, goalPos.x - robot.position.x);
						float rotAngleRad = goalAngleRad - robotWorldForwardAngleRad;
						if(rotAngleRad > Mathf.PI) {
							rotAngleRad -= 2 * Mathf.PI;
						} else {
							if(rotAngleRad < -Mathf.PI) {
								rotAngleRad += 2 * Mathf.PI;
							}
						}
						rosAngular = 1.5f * rotAngleRad;
						// Debug.Log("TURNING Speed: " + rosAngular);

						if(Mathf.Abs(rotAngleRad) < 0.15f) {
							isTurning = false;
							rosAngular = 0f;
						}
					}
				} else { // Assume that the robot is mostly facing "forward" and is mostly aligned with the walls
					// Are we at the center of a node? If so, determine where to go next.
					// if(IN CENTER OF NODE) {
					float distToGoal = Vector3.Distance(robot.position, goalPos);
					if(distToGoal < 0.1f) {
						Debug.Log("PICKING GOAL");
						rosLinear = 0f;
						rosAngular = 0f;
						// TODO: Decide whether to move forward, or turn and then move forward
						currentNode = nextNode;

						// Check if the four main directions have walls or are open
						bool[] directionHasWall = new bool[numCardinalDirections];
						int numOpenings = 0;
						for(int i = 0; i < numCardinalDirections; i++) {
							directionHasWall[i] = cardinalDirectionRanges[i] < avgCellWidth;
							if(!directionHasWall[i]) {
								numOpenings ++;
							}
						}

						if(numOpenings > 2 && nearbyTag != null) {
							Debug.Log("FOUND A TAG");

							//TODO: DO A VISUAL
							List<Transform> points = new List<Transform>();
							points.Insert(0,nearbyTag.transform);
							points.Insert(0,robot);
							DrawPath(trail,points);
							timeLastTalked = Time.time;
							for(int i = 0; i < numCardinalDirections; i++) {
								Vector2Int nodeInDir = currentNode + NodeDirectionVectors[i];
								
								if(nodeInDir.x > -1 && nodeInDir.x < numNodesPerSide && nodeInDir.y > -1 && nodeInDir.y < numNodesPerSide) {
									if(nearbyTag.dirStates[i] == Light_Tag.NodeState.DeadEnd) {
										nodes[nodeInDir.x, nodeInDir.y] = NodeState.DeadEnd;
										nodeStateVisualizers[nodeInDir.x, nodeInDir.y].material.color = NodeStateColors[(int) NodeState.DeadEnd];
									} else {
										if(nodes[nodeInDir.x, nodeInDir.y] == NodeState.DeadEnd) {
											nearbyTag.dirStates[i] = Light_Tag.NodeState.DeadEnd;
										}
									}

									if(nodes[nodeInDir.x, nodeInDir.y] == NodeState.VisitedByMe) {
										nearbyTag.dirStates[i] = Light_Tag.NodeState.VisitedByOther;
										nearbyTag.dirVisitor[i] = Id;
									} else {
										if(nearbyTag.dirStates[i] == Light_Tag.NodeState.VisitedByOther && nearbyTag.dirVisitor[i] != Id) {
											nodes[nodeInDir.x, nodeInDir.y] = NodeState.VisitedByOther;
											nodeStateVisualizers[nodeInDir.x, nodeInDir.y].material.color = NodeStateColors[(int) NodeState.VisitedByOther];
										}
									}
								}
							}
						}

						int bestOption = 5;
						int bestDirection = 5;
						Vector2Int bestOptionIndex = Vector2Int.zero;
						for(int i = 0; i < numCardinalDirections; i++) {
							if(!directionHasWall[i]) {
								Vector2Int nodeInDir = currentNode + NodeDirectionVectors[i];
								if(nodeInDir.x > -1 && nodeInDir.x < numNodesPerSide && nodeInDir.y > -1 && nodeInDir.y < numNodesPerSide) {
									if((int) nodes[nodeInDir.x, nodeInDir.y] < bestOption) {
										bestOption = (int) nodes[nodeInDir.x, nodeInDir.y];
										bestDirection = i;
										bestOptionIndex = nodeInDir;
									}
								}
							}
							Debug.Log("WALL? " + ((Direction) i).ToString() + ": " + directionHasWall[i]);
						}
						
						if(bestOption == (int) NodeState.VisitedByMe) {
							nodes[currentNode.x, currentNode.y] = NodeState.DeadEnd;
							nodeStateVisualizers[currentNode.x, currentNode.y].material.color = NodeStateColors[(int) NodeState.DeadEnd];
						}

						if(bestOption < (int) NodeState.VisitedByMe) {
							nodes[currentNode.x, currentNode.y] = NodeState.VisitedByMe;
							nodeStateVisualizers[currentNode.x, currentNode.y].material.color = NodeStateColors[(int) NodeState.VisitedByMe];
						}

						if(bestOption < (int) NodeState.DeadEnd) {
							nextNode = bestOptionIndex;
							goalPos = goalPos + CardinalDirectionVectors[bestDirection] * avgDistCellCenterToCellCenter;
							goalVisualizer.position = goalPos;
							Debug.Log("GOAL POS: " + goalPos);
						}

						if(numOpenings > 2 && nearbyTag != null) {
							Debug.Log("FOUND A TAG");
							nearbyTag.dirStates[bestDirection] = Light_Tag.NodeState.VisitedByOther;
							nearbyTag.dirVisitor[bestDirection] = Id;
						}

						isTurning = true;
					} else {
						// Debug.Log("MOVING FORWARD");
						// Move forward
						if(distToGoal < 0.3f) {
							rosLinear = 0.1f;
						} else {
							rosLinear = 0.5f;
						}
						
						// // Calculate how far off of center / forward angle we are
						// int forwardIndex = (int) currentPrimaryDirection;
						// int leftIndex = (int) Modulus(((int) LocalDirection.Left + (int) currentPrimaryDirection), numCardinalDirections);
						// int rightIndex = (int) Modulus(((int) LocalDirection.Right + (int) currentPrimaryDirection), numCardinalDirections);
						// if(directionHasWall[leftIndex]) {
						// 	// Given robot position, wall direction, wall range,
						// 	// calculate a position that is in the middle of the next cell.
						// 	// Then rotate accordingly based on homework.
						// 	goalPos = cardinalDirectionHits[leftIndex] + CardinalDirectionVectors[rightIndex] * avgHalfCellWidth + CardinalDirectionVectors[forwardIndex] * avgQuarterCellWidth;
						// } else {
						// 	if(directionHasWall[rightIndex]) {
						// 		goalPos = cardinalDirectionHits[rightIndex] + CardinalDirectionVectors[rightIndex] * avgQuarterCellWidth + CardinalDirectionVectors[forwardIndex] * avgQuarterCellWidth;
						// 	} else { // There isn't a left or a right wall right now... Just keep moving forward
						// 		goalPos = goalPos + CardinalDirectionVectors[forwardIndex] * avgQuarterCellWidth;
						// 	}
						// }
						// goalVisualizer.position = goalPos;

						float goalAngleRad = Mathf.Atan2(goalPos.z - robot.position.z, goalPos.x - robot.position.x);
						float rotAngleRad = goalAngleRad - robotWorldForwardAngleRad;
						if(rotAngleRad > Mathf.PI) {
							rotAngleRad -= 2 * Mathf.PI;
						} else {
							if(rotAngleRad < -Mathf.PI) {
								rotAngleRad += 2 * Mathf.PI;
							}
						}

						if(Mathf.Abs(rotAngleRad) > Mathf.PI / 4) { // Robot is currently facing more than 45* from target
							isTurning = true;
							isStuck = true;
						} else {
							rosAngular = 0.8f * rotAngleRad; // Robot is at least mostly facing towards target
						}
					}
				}


				lastCmdReceived = Time.time;
			}
			yield return null;
		}
		
		yield return null;
	}

	private void SetParameters(ArticulationBody joint)
	{
		ArticulationDrive drive = joint.xDrive;
		drive.forceLimit = forceLimit;
		drive.damping = damping;
		joint.xDrive = drive;
	}

	private void SetSpeed(ArticulationBody joint, float wheelSpeed = float.NaN)
	{
		ArticulationDrive drive = joint.xDrive;
		if (float.IsNaN(wheelSpeed))
		{
			drive.targetVelocity = ((2 * maxLinearSpeed) / wheelRadius) * Mathf.Rad2Deg * (int)direction;
		}
		else
		{
			drive.targetVelocity = wheelSpeed;
		}
		joint.xDrive = drive;
	}

	private void ROSUpdate()
	{
		if (Time.time - lastCmdReceived > ROSTimeout)
		{
			rosLinear = 0f;
			rosAngular = 0f;
		}
		RobotInput(rosLinear, -rosAngular);
	} 

	private void RobotInput(float speed, float rotSpeed) // m/s and rad/s
	{
		if (speed > maxLinearSpeed)
		{
			speed = maxLinearSpeed;
		}
		if (rotSpeed > maxRotationalSpeed)
		{
			rotSpeed = maxRotationalSpeed;
		}
		float wheel1Rotation = (speed / wheelRadius);
		float wheel2Rotation = wheel1Rotation;
		float wheelSpeedDiff = ((rotSpeed * trackWidth) / wheelRadius);
		if (rotSpeed != 0)
		{
			wheel1Rotation = (wheel1Rotation + (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
			wheel2Rotation = (wheel2Rotation - (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
		}
		else
		{
			wheel1Rotation *= Mathf.Rad2Deg;
			wheel2Rotation *= Mathf.Rad2Deg;
		}
		SetSpeed(wA1, wheel1Rotation);
		SetSpeed(wA2, wheel2Rotation);
	}

	public float Modulus(float a, float b) {
		return ((a % b) + b) % b;
	}

	public void NearbyTag(Light_Tag tag) {
		nearbyTag = tag;
	}

	public void DrawPath(LineRenderer lr,List<Transform> points){	
		lr.enabled = true;
		lr.SetColors(TagTalkColor,TagTalkColor);
		lr.SetWidth(0.1f, 0.1f);		 
		int n = points.Count; // + 1;
		Vector3[] pointLine = new Vector3[n];         
			
		for (int i = 0; i < (n/* - 1*/); i++) {
			pointLine[i] = points[i].position;
		}         
        
		lr.positionCount = n;
		lr.SetPositions(pointLine);
	}
}