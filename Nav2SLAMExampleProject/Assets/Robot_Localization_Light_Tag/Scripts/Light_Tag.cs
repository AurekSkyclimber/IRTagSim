using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Light_Tag : MonoBehaviour
{
	

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
    
    
    float rayLength =5;
    public int TagID;
    //Distances from the center of the Turtlebot to the center of the each lux Sensor
    
    public Light_Tag NextTag;
    public List<Transform> NextWaypoints;

    // Starting at East because atan2 returns 0 for East. Also, rotates left because Lidar rotates left.
    public const int numCardinalDirections = 4;
    public enum Direction {East = 0, North = 1, West = 2, South = 3}
    public enum NodeState {New = 0, VisitedByOther = 1, VisitedByMe = 2, DeadEnd = 3}
    public NodeState[] dirStates = new NodeState[numCardinalDirections];
    public int[] dirVisitor = new int[numCardinalDirections];
    
    private void Awake()
    {
    
      noAngle = this.transform.forward;
      leftAngle_30 = Quaternion.AngleAxis(-30,new Vector3(0f,2f,0f));
      rightAngle_30 = Quaternion.AngleAxis(30,new Vector3(0f,2f,0f));
      leftAngle_45 = Quaternion.AngleAxis(-45,new Vector3(0f,2f,0f));
      rightAngle_45 = Quaternion.AngleAxis(45,new Vector3(0f,2f,0f));
      leftAngle_60 = Quaternion.AngleAxis(-60,new Vector3(0f,2f,0f));
      rightAngle_60 = Quaternion.AngleAxis(60,new Vector3(0f,2f,0f));
      
      Front_Vector = frontAngle*noAngle;
      Left_30degVector = leftAngle_30*noAngle;
      Right_30degVector = rightAngle_30*noAngle;
      Left_45degVector = leftAngle_45*noAngle;
      Right_45degVector = rightAngle_45*noAngle;
      Left_60degVector = leftAngle_60*noAngle;
      Right_60degVector = rightAngle_60*noAngle;
      
      for(int i = 0; i < numCardinalDirections; i++) {
        dirStates[i] = NodeState.New;
        dirVisitor[i] = -1;
      }
    }
    
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        Debug.DrawRay(this.transform.position, Front_Vector*rayLength, Color.black);
        Debug.DrawRay(this.transform.position, Left_30degVector*rayLength, Color.red);
        Debug.DrawRay(this.transform.position, Right_30degVector*rayLength, Color.red);
        Debug.DrawRay(this.transform.position, Left_45degVector*rayLength, Color.green);
        Debug.DrawRay(this.transform.position, Right_45degVector*rayLength, Color.green);
        Debug.DrawRay(this.transform.position, Left_60degVector*rayLength, Color.blue);
        Debug.DrawRay(this.transform.position, Right_60degVector*rayLength, Color.blue);
    }
    
    
    
    
    //~ void FixedUpdate()
    //~ {
        //~ rayLength = transform.GetComponent<Light>().range;
        
        
        //~ Ray ray_center = new Ray(this.transform.position,Front_Vector);
        //~ if(Physics.Raycast(ray_center,out RaycastHit hit,rayLength)){
        
			//~ LuxSensorOnTheRobot lux_sensor = hit.transform.GetChild(0).GetComponent<LuxSensorOnTheRobot>();
			//~ if(lux_sensor!=null){
				//~ lux_sensor.HitSensor(lux_sensor.id, hit.angle, hit.distance);
        	//~ }   
            
    	//~ }  
    	
    	
    	//~ Ray ray_left30 = new Ray(this.transform.position,Left_30degVector);
        //~ if(Physics.Raycast(ray_left30,out hit,rayLength)){
			//~ if(hit.transform.tag == "LightTag") {
				//~ LuxSensorOnTheRobot lux_sensor = hit.transform.GetChild(0).GetComponent<LuxSensorOnTheRobot>();
			//~ }
			//~ if(lux_sensor!=null){
				//~ lux_sensor.HitSensor(lux_sensor.id,hit.angle,hit.distance);
				//~ }
              
        	//~ }   
            
    	  
    	//~ Ray ray_right30 = new Ray(this.transform.position,Right_30degVector);
        //~ if(Physics.Raycast(ray_right30,out hit,rayLength)){
        //~ if(hit.transform.tag == "LightTag") {
				//~ LuxSensorOnTheRobot lux_sensor = hit.transform.GetChild(0).GetComponent<LuxSensorOnTheRobot>();
			//~ }
			//~ if(lux_sensor!=null){
				//~ lux_sensor.HitSensor(lux_sensor.id,hit.angle,hit.distance);
				//~ }
            
    	//~ }   
    	
    	
    	
    	
    	
    	//~ Ray ray_left45 = new Ray(this.transform.position,Left_45degVector);
        //~ if(Physics.Raycast(ray_left45,out hit,rayLength)){
        //~ if(hit.transform.tag == "LightTag") {
				//~ LuxSensorOnTheRobot lux_sensor = hit.transform.GetChild[0].GetComponent<LuxSensorOnTheRobot>();
			//~ }
			//~ if(lux_sensor!=null){
				//~ lux_sensor.HitSensor(lux_sensor.id,hit.angle,hit.distance);
				//~ }
            
    	//~ }                      
             
        
        
        //~ Ray ray_right45 = new Ray(this.transform.position,Right_45degVector);
        //~ if(Physics.Raycast(ray_right45,out hit,rayLength)){
        //~ if(hit.transform.tag == "LightTag") {
				//~ LuxSensorOnTheRobot lux_sensor = hit.transform.GetChild(0).GetComponent<LuxSensorOnTheRobot>();
			//~ }
			//~ if(lux_sensor!=null){
				//~ lux_sensor.HitSensor(lux_sensor.id,hit.angle,hit.distance);
				//~ }
            
    	//~ }      
    	//~ Ray ray_left60 = new Ray(this.transform.position,Left_60degVector);
        //~ if(Physics.Raycast(ray_left60,out hit,rayLength)){
        //~ if(hit.transform.tag == "LightTag") {
				//~ LuxSensorOnTheRobot lux_sensor = hit.transform.GetChild(0).GetComponent<LuxSensorOnTheRobot>();
			//~ }
			//~ if(lux_sensor!=null){
				//~ lux_sensor.HitSensor(lux_sensor.id,hit.angle,hit.distance);
				//~ }
            
    	//~ }      
    	//~ Ray ray_right60 = new Ray(this.transform.position,Right_60degVector);
        //~ if(Physics.Raycast(ray_right60,out hit,rayLength)){
        //~ if(hit.transform.tag == "LightTag") {
				//~ LuxSensorOnTheRobot lux_sensor = hit.transform.GetChild(0).GetComponent<LuxSensorOnTheRobot>();
			//~ }
			//~ if(lux_sensor!=null){
				//~ lux_sensor.HitSensor(lux_sensor.id,hit.angle,hit.distance);
				//~ }
            
    	//~ }        
        
    //~ }
    
    
    
    //~ public void Lux_Hit_Details(int id, float HitAngle, float HitDistance){
				//~ GameObject hitObject = hit.collider.gameObject;
				//~ string LuxSensorName = hit.collider.gameObject.name;
				//~ HitAngle = 0f;//calculate this
				//~ HitDistance = hit.distance;
				//~ id = LuxSensorNumber;
					
		
	//~ }    
       
    
}
