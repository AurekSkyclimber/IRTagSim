using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
public class LIghtTagController : MonoBehaviour
{
    
    
     GameObject lightMesh;
     Light Tag;
     GameObject RobotPos;
     float originalRange = 5.1f;
     float MaxRange;
     public float SignalStrength;//Future Dinithi, please keep this variable empty in the editor
     float SpotAngle;
     float ConeAngle;
     float FOV;
    // UI Controllers
    Slider sliderInstance;
     public Transform[] LightTags;
    
    
    // Start is called before the first frame update
    void Start()
    {
		
		
		LightTags = GetComponentsInChildren<Transform>();
		foreach(Transform lightTag in LightTags) {
            lightMesh = lightTag.Find("RaycastTriangle").gameObject;
        }
        
        Tag = GetComponent<Light>();
        Tag.type = LightType.Spot;
        originalRange = Tag.range;
        SpotAngle = Tag.spotAngle;
        //~ ConeAngle = Tag.GetComponent<SpotLight>().coneAngle;
        
        RobotPos = GameObject.Find("turtlebot3_manual_config");
    }

    // Update is called once per frame
    void Update()
    {
        LightIntensityController();
        //SignalStrength = RSSICalc(Tag,RobotPos.transform.position.z);
        //FOV = FOVCalc(Tag,originalRange,SpotAngle);
        
        
        
    }
    
    
    
    public void LightIntensityController() {
		
      lightMesh.gameObject.transform.localScale = new Vector3(Tag.range,Tag.range,0f);
    }   
      
    
         public float RSSICalc(Light light, float distance)
     {                 
         //float RSSI = light.intensity / (distance * distance); //this theoretically corect but Distance is a vector
         float RSSI =light.intensity * (1f - Mathf.Clamp01 (distance / light.range));
         Debug.Log("RSSI: "+RSSI);       
         return RSSI;
     }
     
     
     
     //I derived this using the equation of the cone. Unsure if this would be useful in the future
     //Inputs : Get the spot light Range, get the angle of the cone
         public float FOVCalc(Light light,float Range, float OutterspotAngle)
     {                 
         
         OutterspotAngle = light.spotAngle/2;
         Range = light.range;
         float SpotLightRadius = Mathf.Atan(Mathf.Tan(OutterspotAngle* Mathf.Deg2Rad)*Range);
         float SpotlightDiameter = 2*(SpotLightRadius);
         Debug.Log("SpotAngle: "+SpotAngle);  
         Debug.Log("FOV of the Tag: "+SpotlightDiameter);       
         return SpotlightDiameter;
     }
     
     
     
     
     public void SliderController(float value){   		 
		 //I am trying to change the range of the spot light
		 //Tag.range = Mathf.Lerp(originalRange,100f,sliderInstance.value);
		  
	}
	
	
	
	
	
	void OnGUI()
    {
        Tag.range = GUI.HorizontalSlider(new Rect(25, 25, 100, 30), Tag.range, originalRange, 100f);
        GUI.Label(new Rect(30,30,100,20), "RSSI is : " + SignalStrength.ToString());
    }
     
}
