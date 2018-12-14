using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityStandardAssets.Characters.FirstPerson;

public class LandmarkPlacer : MonoBehaviour {

	public GameObject temp_landmark;
	public GameObject landmark;
	public GameObject viapoint;

    private Camera top_down_cam;
    private Camera first_person_cam;

	private GameObject cur_temp_landmark;
	private GameObject last_landmark;
	private GameObject cur_viapoint;
	
    private bool hittingGround;

    private Color valid_temp;
    private Color invalid_temp;

	// Use this for initialization
	void Start () {
		hittingGround = false;

		cur_viapoint = null;

        // set colors for valid and invalid landmark placements
        invalid_temp = Color.red;
        invalid_temp.a = 0.25f;
        valid_temp = Color.gray;
        valid_temp.a = 0.25f;

        // get cameras
        top_down_cam = GameObject.Find("Top-Down Camera").GetComponent<Camera>();
        first_person_cam = GameObject.Find("FirstPersonCharacter").GetComponent<Camera>();
    }
	
	// Update is called once per frame
	void Update () {
        // if in 'setup' stage
        if (Manager.state == "SETUP") {
            if (!first_person_cam.enabled)
            {
                SwitchCameras();
            }
            // check for 'start' key to start the simulation
            if (Input.GetButtonDown("Start") && Manager.landmarks.Count > 1)
            {
                Destroy(cur_temp_landmark);
                Destroy(cur_viapoint);
                hittingGround = false;
                Manager.StartRunning();
                return;
            }

            // set init_landmark if it hasn't been set yet
            if (last_landmark == null) {
                last_landmark = Manager.StaticInitLandmark;
            }

            // get cam transform of first person camera
            Transform camTransform = Camera.main.transform;

            // check if looking at the floor
            RaycastHit hit;
            if (Physics.Raycast(camTransform.position + new Vector3(0f, 0.2f, 0f), camTransform.forward, out hit, 6f) &&
                hit.transform.tag == "Floor")
            {
                // if no viapoint instantiated, instantiate new viapoint
                if (cur_viapoint == null && last_landmark != null)
                {
                    cur_viapoint = Instantiate(viapoint);
                }
                // if not previously looking at the ground
                if (!hittingGround)
                {
                    hittingGround = true;
                    cur_temp_landmark = Instantiate(temp_landmark);
                }

                // update position of temporary landmark object
                cur_temp_landmark.transform.position = hit.point + new Vector3(0f, 0.63f, 0f);

                // create temporary viapoint for visualization
                if (last_landmark != null && cur_viapoint != null)
                {
                    float mid_x = (cur_temp_landmark.transform.position.x + last_landmark.transform.position.x) / 2f;
                    float mid_z = (cur_temp_landmark.transform.position.z + last_landmark.transform.position.z) / 2f;
                    cur_viapoint.transform.position = new Vector3(mid_x, 0f, mid_z);
                }

                // check if valid position for new landmark
                bool valid_landmark = IsValidLandmark(cur_temp_landmark.transform.position);

                // place landmark if mouse clicked and valid position
                if (Input.GetMouseButtonDown(0) && valid_landmark)
                {
                    last_landmark = Manager.AddLandmark(cur_temp_landmark.transform.position);
                }

                // set color of temporary landmark object
                Renderer temp_landmark_renderer = cur_temp_landmark.GetComponent<Renderer>();
                if (!valid_landmark && temp_landmark_renderer.material.color != invalid_temp) {
                    temp_landmark_renderer.material.color = invalid_temp;
                } else if (valid_landmark && temp_landmark_renderer.material.color != valid_temp) {
                    temp_landmark_renderer.material.color = valid_temp;
                }
            }
            else
            {
                // destroy temp landmark and viapoint when user first looks away from floor
                if (hittingGround)
                {
                    Destroy(cur_temp_landmark);
                    if (cur_viapoint != null)
                    {
                        Destroy(cur_viapoint);
                        cur_viapoint = null;
                    }
                    cur_temp_landmark = null;
                    hittingGround = false;
                }
            }
        } else if (Manager.state == "RUNNING" || Manager.state == "FINISHED"){
            // check for camera switch button
            if (Input.GetButtonDown("CameraSwitch")){
                SwitchCameras();
            }
        }
	}

    // checks to make sure pos is valid position for a new landmark
    private bool IsValidLandmark(Vector3 pos) {
        // check distance from last landmark
        float dist = Vector3.Distance(last_landmark.transform.position, pos);
        if (dist > 10f) {
            return false;
        }

        // check for any nearby landmarks
        Collider[] hitColliders = Physics.OverlapSphere(pos, 2f);
        for (int i = 0; i < hitColliders.Length; i++){
            if (hitColliders[i].transform.tag == "Landmark") {
                return false;
            }
        }

        return true;
    }

    // switch between first person camera and top-down camera
    public void SwitchCameras() {
        // switch from first person to top-down
        if (first_person_cam.enabled)
        {
            GetComponent<FirstPersonController>().enabled = false;
            first_person_cam.enabled = false;
            top_down_cam.enabled = true;
            GameObject.Find("Light").GetComponent<Light>().shadows = LightShadows.None;
        }
        // switch from top-down to first person
        else if (top_down_cam.enabled)
        {
            GetComponent<FirstPersonController>().enabled = true;
            first_person_cam.enabled = true;
            top_down_cam.enabled = false;
            GameObject.Find("Light").GetComponent<Light>().shadows = LightShadows.Soft;
        }
    }
}
