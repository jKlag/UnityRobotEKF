using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TopDownCamera : MonoBehaviour {

    public GameObject robot;
    public float pan_speed;

	// Use this for initialization
	void Start () {

	}
	
	// Update is called once per frame
	void Update () {
        Vector3 pos = transform.position;

        if (Manager.state == "RUNNING")
        {
            // get robot position
            pos = robot.transform.position;

            // ignore robot height
            pos.y = transform.position.y;


        } 
        else if (Manager.state == "FINISHED")
        {
            if (Input.GetKey(KeyCode.D)) {
                pos += new Vector3(pan_speed, 0f, 0f);
            } else if(Input.GetKey(KeyCode.A))
            {
                pos -= new Vector3(pan_speed, 0f, 0f);
            } else if (Input.GetKey(KeyCode.W))
            {
                pos += new Vector3(0f, 0f, pan_speed);
            } else if (Input.GetKey(KeyCode.S))
            {
                pos -= new Vector3(0f, 0f, pan_speed);
            }
        }

        // stop camera if at edge of environment
        if (pos.z < 22)
        {
            pos.z = 22;
        }
        else if (pos.z > 50)
        {
            pos.z = 50;
        }

        if (pos.x < 9.5f)
        {
            pos.x = 9.5f;
        }
        else if (pos.x > 19f)
        {
            pos.x = 19f;
        }

        // assign position to camera
        transform.position = pos;
    }
}
