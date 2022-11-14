using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LuxSensorOnTheRobot : MonoBehaviour
{
  ////this script lives on the Lux sensors of the robot
  
  Vector3 noAngle;
  Quaternion frontAngle;
  Quaternion leftAngle_30;
  Quaternion rightAngle_30;
  Quaternion leftAngle_45;
  Quaternion rightAngle_45;
  Quaternion leftAngle_60;
  Quaternion rightAngle_60;
  Vector3 Left_30degVector;
  Vector3 Right_30degVector;
  Vector3 Left_45degVector;
  Vector3 Right_45degVector;
  Vector3 Left_60degVector;
  Vector3 Right_60degVector;
  Vector3 Front_Vector;   
  Vector3 locPos = Vector3.zero;
  
  float rayLength =5f;
  public int id;
  
  //Get access to the lighttagcontroller on light tag
  
  LIghtTagController Tag1;
  

  //HitReaction hitReaction;
  
  //RayCast Hit positions
  Vector3 hitpos;
  Vector3 hitEulerAngles;
  
  
  //Get the Raycast hit positions
  float hitposx;
  float hitposy;
  float hitposz;
  
  //Different RSSI values dirived from different tags
  float RSSI_Tag1;
  float RSSI_Tag2;
  float RSSI_Tag3;
  //GEneric Tag
  float RSSI_Tag;
  
  
  //Create Lists
  //Create Lists
    private List<Vector3> Hit_Pos = new List<Vector3>();
    public List<Vector3> Hit_Pos_pub => Hit_Pos;
    private List<Vector3> Hit_Angles = new List<Vector3>();
    public List<Vector3> Hit_Angles_pub => Hit_Angles;
    
  public float RayHitPos;
  // private TurtleController turtleController;
  private Explorer_Mover explorer;
      
  private void Awake()
  {
    // turtleController = transform.root.GetComponent<TurtleController>();
    explorer = transform.root.GetComponent<Explorer_Mover>();
    
    
    //example
    //hitReaction = GameObject.FindWithTag("HitReactor").GetComponent<HitReaction>();
    // Tag1 = GameObject.FindWithTag("LightTag").GetComponent<LIghtTagController>();
    
    
    noAngle = this.transform.forward;
    leftAngle_30 = Quaternion.AngleAxis(-30,new Vector3(0f,2f,0f));
    rightAngle_30 = Quaternion.AngleAxis(30,new Vector3(0f,2f,0f));
    leftAngle_45 = Quaternion.AngleAxis(-45,new Vector3(0f,2f,0f));
    rightAngle_45 = Quaternion.AngleAxis(45,new Vector3(0f,2f,0f));
    leftAngle_60 = Quaternion.AngleAxis(-60,new Vector3(0f,2f,0f));
    rightAngle_60 = Quaternion.AngleAxis(60,new Vector3(0f,2f,0f));
    
    Front_Vector = frontAngle*this.transform.forward;
    Left_30degVector = leftAngle_30*this.transform.forward;
    Right_30degVector = rightAngle_30*this.transform.forward;
    Left_45degVector = leftAngle_45*this.transform.forward;
    Right_45degVector = rightAngle_45*this.transform.forward;
    Left_60degVector = leftAngle_60*this.transform.forward;
    Right_60degVector = rightAngle_60*this.transform.forward;
  
  
  
  }
    
  
  // Start is called before the first frame update
  void Start()
  {
      
  }

  // Update is called once per frame
  void Update()
  {
    Vector3 distance = this.transform.right*rayLength;
    Debug.DrawRay(this.transform.position, distance, Color.black);
    Debug.DrawRay(this.transform.position, leftAngle_30*distance, Color.red);
    Debug.DrawRay(this.transform.position, rightAngle_30*distance, Color.red);
    Debug.DrawRay(this.transform.position, leftAngle_45*distance, Color.green);
    Debug.DrawRay(this.transform.position, rightAngle_45*distance, Color.green);
    Debug.DrawRay(this.transform.position, leftAngle_60*distance, Color.blue);
    Debug.DrawRay(this.transform.position, rightAngle_60*distance, Color.blue);
  }
    
    
  private void OnTriggerStay(Collider other){
		RaycastHit hit;
		//~ Debug.Log("Name of the object: " + other.gameObject.name);
    
		// if (other.gameObject.tag=="LightTag")
		// {
		// 	if (Physics.Raycast(transform.position, other.transform.position - transform.position, out hit)) ///*Front_Vector*/ noAngle*rayLength, out hit))
		// 	{
		// 		if(hit.transform.tag == "LightTag") {
		// 			//~ Debug.Log("I am talking to a lightTag");				
		// 			//~ Debug.Log("Point of contact: "+hit.point);
					
		// 			Light_Tag tag = hit.transform.GetComponent<Light_Tag>();
		// 			if(tag != null) {
		// 				turtleController.NewWaypoints(tag);
		// 			}
					
		// 			//~ locPos = transform.InverseTransformPoint(hit.point);
		// 			//~ Debug.Log("Point of locPos: "+ locPos); 	
		// 			hitpos	= hit.point;
		// 			//What I am doing here is that I am getting the relavant light intensity for the given Raycast hit
		// 			hitposx =hit.point.x;
		// 			hitposy =hit.point.y;
		// 			hitposz =hit.point.z;
		// 			Light lt= other.GetComponent<Light>(); 				
		// 			RSSI_Tag = SignalStrength(lt, Vector3.Distance(this.transform.position, other.transform.position),hitposx);				
		// 			hitEulerAngles=hit.transform.eulerAngles;			
		// 			//~ Debug.Log("Hit Angle: "+hitEulerAngles); 
		// 			//hitEulerAngles_x = hit.transform.eulerAngles.x;
		// 			while(Hit_Pos.Count > 10) {
		// 				Hit_Pos.RemoveAt(0);
		// 				Hit_Angles.RemoveAt(0);
		// 			}
		// 			Hit_Pos.Add(hitpos);
		// 			Hit_Angles.Add(hitEulerAngles);
		// 		}
		// 	}		
		// }

    if (other.gameObject.tag=="LightTag")
		{
			if (Physics.Raycast(transform.position, other.transform.position - transform.position, out hit)) ///*Front_Vector*/ noAngle*rayLength, out hit))
			{
				if(hit.transform.tag == "LightTag") {
					//~ Debug.Log("I am talking to a lightTag");				
					//~ Debug.Log("Point of contact: "+hit.point);
					
					Light_Tag tag = hit.transform.GetComponent<Light_Tag>();
					if(tag != null) {
						explorer.NearbyTag(tag);
					}
        }
      }
    }
	}
    	
    
    
  public float SignalStrength(Light light, float distance, float RayHitPos)
  {                 
      float RSSI = light.intensity / (distance * distance); //this theoretically corect but Distance is a vector
      hitposx = light.intensity;
      //float RSSI =light.intensity * (1f - Mathf.Clamp01 (distance / light.range));
      //~ Debug.Log("RSSI on Robot: "+RSSI);       
      return RSSI;
  }
	
}
