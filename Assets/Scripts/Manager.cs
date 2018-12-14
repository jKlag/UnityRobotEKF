using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityStandardAssets.Characters.FirstPerson;

public class Manager : MonoBehaviour {

    public GameObject init_landmark;
    public GameObject landmark_prefab;
    public GameObject viapoint_prefab;

    public GameObject canvas;
    public GameObject instructions;
    public GameObject odometry_slider;
    public GameObject sensor_slider;
    public GameObject EKF_slider;
    public GameObject heading_slider;

    public GameObject robot;

    public static List<GameObject> landmarks;
    public static List<GameObject> viapoints;

    private static GameObject path;
    private static LineRenderer path_renderer;

    public static Manager instance;

    public static string state = "";

    // Use this for initialization
    void Start () {
        instance = this;

        landmarks = new List<GameObject>{init_landmark};

        viapoints = new List<GameObject>();

        // create optimal path display
        path = new GameObject();
        path_renderer = path.AddComponent<LineRenderer>();
        path_renderer.positionCount = 0;
        path_renderer.widthMultiplier = 0.1f;
        path.GetComponent<Renderer>().material.color = Color.black;

        Cursor.visible = true;
        canvas.SetActive(true);
        GameObject.Find("FPSController").GetComponent<FirstPersonController>().enabled = false;

        instructions.GetComponentInChildren<Text>().text = "";

        // initialize state
        state = "PARAMETERS";
    }
	
	// Update is called once per frame
	void Update () {
        if (state == "FINISHED" && Input.GetKeyDown(KeyCode.Return))
        {
            Reset();
        }
    }

    public void StartSetup()
    {
        Cursor.visible = false;
        canvas.SetActive(false);
        GameObject.Find("FPSController").GetComponent<FirstPersonController>().enabled = true;
        instructions.GetComponentInChildren<Text>().text = "Use W,A,S,D to move and click left" +
        	" mouse to place a landmark. When ready, press Enter to begin.";

        EKFController controller = robot.GetComponent<EKFController>();
        controller.odometry_noise_scale = odometry_slider.GetComponent<Slider>().value;
        controller.sensor_noise_scale = sensor_slider.GetComponent<Slider>().value;
        controller.EKF_correction_noise_scale = EKF_slider.GetComponent<Slider>().value;
        controller.heading_noise_scale = heading_slider.GetComponent<Slider>().value;

        state = "SETUP";
    }

    public static GameObject StaticInitLandmark
    {
        get
        {
            return instance.init_landmark;
        }
    }

    public static GameObject StaticLandmarkPrefab
    {
        get
        {
            return instance.landmark_prefab;
        }
    }

    public static GameObject StaticViapointPrefab
    {
        get
        {
            return instance.viapoint_prefab;
        }
    }

    public static GameObject StaticCanvas
    {
        get
        {
            return instance.canvas;
        }
    }

    public static GameObject StaticInstructions
    {
        get
        {
            return instance.instructions;
        }
    }

    public static GameObject AddLandmark(Vector3 pos){
        // create new landmark and add to list of landmarks
        GameObject new_landmark = Instantiate(Manager.StaticLandmarkPrefab);
        new_landmark.transform.position = pos;
        landmarks.Add(new_landmark);

        // create new viapoint
        GameObject new_viapoint = Instantiate(Manager.StaticViapointPrefab);
        // update last_landmark
        GameObject last_landmark = landmarks[landmarks.Count - 2];
        // get viapoint position based on last landmark and new landmark and assign
        float mid_x = (last_landmark.transform.position.x + new_landmark.transform.position.x) / 2f;
        float mid_z = (last_landmark.transform.position.z + new_landmark.transform.position.z) / 2f;
        new_viapoint.transform.position = new Vector3(mid_x, 0f, mid_z);
        // add viapoint to list of viapoints
        viapoints.Add(new_viapoint);

        // add to path display
        path_renderer.positionCount += 1;
        path_renderer.SetPosition(path_renderer.positionCount - 1, 
                                  new_viapoint.transform.position + new Vector3(0f,0.1f,0f));

        return new_landmark;
    }

    // start the simulation
    public static void StartRunning(){
        state = "RUNNING";
        StaticInstructions.GetComponentInChildren<Text>().text = "Press R to switch " +
        	"between cameras.";
    }

    // finish the simulation
    public static void Finish(){
        state = "FINISHED";
        StaticInstructions.GetComponentInChildren<Text>().text = "Use W,A,S,D to move. " +
        	"Press Enter to restart.";
    }

    public static void Reset()
    {
        // destroy landmarks and viapoints
        for (int i = 1; i < landmarks.Count; i++) {
            Destroy(landmarks[i]);
            Destroy(viapoints[i - 1]);
            Destroy(path);
        }

        landmarks.Clear();
        landmarks.Add(StaticInitLandmark);
        viapoints.Clear();

        // create optimal path display
        path = new GameObject();
        path_renderer = path.AddComponent<LineRenderer>();
        path_renderer.positionCount = 0;
        path_renderer.widthMultiplier = 0.1f;
        path.GetComponent<Renderer>().material.color = Color.black;

        //GameObject.Find("FPSController").GetComponent<LandmarkPlacer>().SwitchCameras();
        GameObject.Find("FPSController").GetComponent<FirstPersonController>().enabled = false;
        StaticCanvas.SetActive(true);
        Cursor.visible = true;
        Cursor.lockState = CursorLockMode.None;

        StaticInstructions.GetComponentInChildren<Text>().text = "";
        state = "PARAMETERS";
    }

}
