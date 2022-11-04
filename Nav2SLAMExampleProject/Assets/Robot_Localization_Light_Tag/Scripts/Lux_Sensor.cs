using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Lux_Sensor : MonoBehaviour
{
    ////GameObject FOV;
    //float rayLength =12f;
    //Vector3 noAngle;
    ////~ Quaternion frontAngle;
    ////~ Vector3 Front_Vector;
    
    
    //Vector3 locPos = Vector3.zero;
    
    ////RayCast Hit Positions
    //Vector3 hitpos;
     //Vector3 hitEulerAngles;
     
    ////Create Lists
     //private List<Vector3> Hit_Pos = new List<Vector3>();
     //public List<Vector3> Hit_Pos_pub => Hit_Pos;
     //private List<Vector3> Hit_Angles = new List<Vector3>();
     //public List<Vector3> Hit_Angles_pub => Hit_Angles;
    ////Distances from the center of the Turtlebot to the center of the each lux Sensor
        
    //private void Awake()
    //{		
		////FOV = GameObject.FindChild("Lux_Sensor_1/RaycastTriangle");    
		////gameobject.transform.Find("ChildName") ; 
		//noAngle = this.transform.forward;        
		////~ Front_Vector = frontAngle*noAngle;    
    //}
    
    
    
    
    //// Start is called before the first frame update
    //void Start()
    //{
        
    //}

    //// Update is called once per frame
    //void Update()
    //{
        ////Debug.DrawRay(this.transform.position, noAngle*rayLength, Color.black);
    //}
   
    //private void OnTriggerEnter(Collider other){
		//RaycastHit hit;
		//Debug.Log("Name of the object: " + other.gameObject.name);
    
		     
		//if (Physics.Raycast(transform.position, /*Front_Vector*/ noAngle*rayLength, out hit))
		//{
		 
			//if (other.gameObject.tag=="LightTag"){
				//Debug.Log("I am talking to a lightTag");

				//Debug.DrawRay(this.transform.position, Vector3.left*rayLength, Color.black, 0.2f);
				//Debug.Log("Point of contact: "+hit.point);
				////~ locPos = transform.InverseTransformPoint(hit.point);
				////~ Debug.Log("Point of locPos: "+ locPos); 	
				//hitpos	= hit.point;
				//hitEulerAngles=hit.transform.eulerAngles;
				//Debug.Log("Hit Angle: "+hitEulerAngles); 
				////hitEulerAngles_x = hit.transform.eulerAngles.x;
				//while(Hit_Pos.Count > 10) {
					//Hit_Pos.RemoveAt(0);
					//Hit_Angles.RemoveAt(0);
				//}
				//Hit_Pos.Add(hitpos);
				//Hit_Angles.Add(hitEulerAngles);
			//}		

		//}
	//}
    
    public void HitSensor(int id, float distance) {
		
	}
}


