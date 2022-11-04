using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotwithSensors : MonoBehaviour
{
   //this script lives on the Robot

   Rigidbody rb;
   
   private LuxSensorOnTheRobot lightsensor_l;
   private LuxSensorOnTheRobot lightsensor_c;
   private LuxSensorOnTheRobot lightsensor_r;
   
   
   float a,b,r,theta, Distance;   
     
    // Start is called before the first frame update
    void Start()
    {
        rb =GetComponent<Rigidbody>();
        lightsensor_l = transform.Find("base_footprint/Visuals/unnamed/Lux_Sensor_3/RaycastTriangle").GetComponent<LuxSensorOnTheRobot>();
        lightsensor_c= transform.Find("base_footprint/Visuals/unnamed/Lux_Sensor_1/RaycastTriangle").GetComponent<LuxSensorOnTheRobot>();
        lightsensor_r= transform.Find("base_footprint/Visuals/unnamed/Lux_Sensor_2/RaycastTriangle").GetComponent<LuxSensorOnTheRobot>();
        //LastReadingPosition_sl = Lux_Sensor_left.transform.position;
        //LastReadingPosition_sc = Lux_Sensor_center.transform.position;
        //LastReadingPosition_sr = Lux_Sensor_right.transform.position;
    }

    // Update is called once per frame
    void Update()
    {
		
		Debug.Log("List Values for lightsensor_l: "+lightsensor_l.Hit_Pos_pub); 
		Debug.Log("List Values for lightsensor_c: "+lightsensor_c.Hit_Pos_pub);
		Debug.Log("List Values for lightsensor_r: "+lightsensor_r.Hit_Pos_pub);  
        //Vector3 targetpos_sl =  Lux_Sensor_left.transform.position;
        //Vector3 targetpos_sc =  Lux_Sensor_center.transform.position;
        //Vector3 targetpos_sr =  Lux_Sensor_right.transform.position;
        
        //~ a= (((targetpos_sl+targetpos_sr)+(2*targetpos_sc))/(2f*Mathf.Cos(Mathf.PI/4f)+2f));
        //~ b= ((targetpos_sl-targetpos_sr)/(2*Mathf.Sin(Mathf.PI/4)));
        //~ r= Mathf.Sqrt(a*a+b*b);
        //~ theta = Mathf.Atan2(a/b)*Mathf.Rad2Deg;
        
        
        //~ //Quadrant Calculator
        
        //~ switch(theta)
        //~ {
		//~ case float x when x<=-90f:
			//~ return 3;
		//~ case float x when x<=0f:
			//~ return 4;
		//~ case float x when x<=90f:
			//~ return 1;
		//~ case float x when x<=180f:
			//~ return 2;
		//~ default:
			//~ return 0;
		//~ }
	}
	//listX = lightsensor_l.Hit_Pos_pub
	 public float GettingListData(List<Vector3> listX){			
		//~ if(listX != null){
			//~ for (int index = 0; index < listX.Count; index++)
            //~ {
                //~ Lux_Sensor List = listX[index];
                //~ //float xpos = listX[index];
                //~ float xpos = listX[0];
                //~ return xpos;               
            //~ }
            
            return listX[listX.Count - 1].x;
            
            // return listX.transform.position.x - transform.position.x;
		//~ }
	}
	
	//lightsensor_l_x = lightsensor_l.Hit_Pos_pub;
	//private Lux_Sensor lightsensor_l;	
	 public float[] Algorithm(float lightsensor_l_x, float lightsensor_c_x,float lightsensor_r_x){			
		    lightsensor_l_x = GettingListData(lightsensor_l.Hit_Pos_pub);
		    lightsensor_c_x = GettingListData(lightsensor_c.Hit_Pos_pub);
		    lightsensor_r_x = GettingListData(lightsensor_r.Hit_Pos_pub);
		
		    float a= (((lightsensor_l_x+lightsensor_r_x)+(2*lightsensor_c_x))/(2f*Mathf.Cos(Mathf.PI/4f)+2f));
		
		    float b= ((lightsensor_l_x-lightsensor_r_x)/(2*Mathf.Sin(Mathf.PI/4)));
		    float r= Mathf.Sqrt(a*a+b*b);
		    float theta = Mathf.Atan2(a, b)*Mathf.Rad2Deg;
		    float Distance = r;
		
            return new float[] {Distance, theta};
       }	
		
	}


	
	
	
