using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EKFController : MonoBehaviour {

    // class that holds (x,y) position and heading - made for readability
    private class Pose
    {
        public Vector2 pos;
        public float heading;

        public Pose(Vector2 init_pos, float init_heading)
        {
            pos = init_pos;
            heading = init_heading;
        }
    }

    // prefabs
    public GameObject robot_estimate_prefab;
    public GameObject landmark_estimate_prefab;
    public GameObject viapoint_esimate_prefab;

    // public variables
    public float Speed;
    public float odometry_noise_scale;
    public float sensor_noise_scale;
    public float EKF_correction_noise_scale;
    public float heading_noise_scale;
    public float angular_velocity_ff;

    // robot path renderer
    private static GameObject path;
    private static LineRenderer path_renderer;

    private Vector3 init_position;

    private float landmark_height;

    private Rigidbody mRigidBody;

    private int cur_viapoint_num;
    private Pose mu0;
    private Matrix4x4 sigma0;
    private float w0;

    // noise parameters
    private Matrix4x4 sNoiseCov;
    private Matrix4x4 estSNoiseCov;
    private float a1;
    private float a2;
    private float a3;
    private float a4;

    // visualization of robot's estimated pose
    private GameObject robot_estimate;

    private List<GameObject> landmarks;
    private List<GameObject> est_landmarks;
    private List<GameObject> est_viapoints;

    private bool initialized;
    private bool finished;

    // Use this for initialization
    void Start () {
        init_position = transform.position;

        mRigidBody = this.GetComponent<Rigidbody>();

        est_landmarks = new List<GameObject>();
        est_viapoints = new List<GameObject>();

        // reset the environment and all parameters
        Reset();
    }
	
	// Update is called once per frame
	void Update () {
        if (Manager.state == "SETUP" && finished)
        {
            // reset environment and all parameters
            Reset();
        }
        if (Manager.state == "RUNNING") {
            // just started running
            if (!initialized) {
                Initialize();
                return;
            }

            // add vertex to robot path
            path_renderer.positionCount += 1;
            path_renderer.SetPosition(path_renderer.positionCount - 1,
                                      transform.position + new Vector3(0f, 0.08f, 0f));

            // get noisy landmark sensor readings
            List<Vector2> landmarkReadings = SenseLandmarks();

            // estimate robot pose with EKF correction step
            mu0 = EKFLocalization(Time.deltaTime, landmarkReadings);

            // update robot estimate pose
            robot_estimate.transform.position = new Vector3(mu0.pos.x, 0.28f, mu0.pos.y);
            robot_estimate.transform.eulerAngles = new Vector3(0.0f,
              HeadingToRotation(mu0.heading), 0.0f);

            // recalculate landmarks
            RecalculateLandmarks(landmarkReadings);

            // recalculate viapoints
            RecalculateViapoints();

            // get distance to next viapoint
            GameObject cur_viapoint = est_viapoints[cur_viapoint_num];
            Vector2 viapoint_pos = new Vector2(cur_viapoint.transform.position.x,
                                               cur_viapoint.transform.position.z);
            float distance = Vector2.Distance(mu0.pos, viapoint_pos);

            // if reached viapoint, get distance to new viapoint
            if (distance < 0.7f)
            {
                // reached last viapoint
                if (cur_viapoint_num == est_viapoints.Count - 1)
                {
                    Manager.Finish();
                    return;
                }
                cur_viapoint_num += 1;
                cur_viapoint = est_viapoints[cur_viapoint_num];
                viapoint_pos = new Vector2(cur_viapoint.transform.position.x,
                                           cur_viapoint.transform.position.z);
                distance = Vector2.Distance(mu0.pos, viapoint_pos);
            }

            // calculate estimated bearing to next viapoint
            float est_bearing = Mathf.Atan2(viapoint_pos.y - mu0.pos.y,
                                            viapoint_pos.x - mu0.pos.x) - mu0.heading;

            // correct bearing for branch cut
            est_bearing = (est_bearing + Mathf.PI) % (2 * Mathf.PI) - Mathf.PI;

            // get new angular velocity from bearing, distance, and speed
            float w = est_bearing / (distance / Speed);

            // add angular velocity fudge factor
            w *= angular_velocity_ff;

            // update robot velocities
            mRigidBody.angularVelocity = new Vector3(0f, -w, 0f);
            mRigidBody.velocity = (transform.forward * Speed);

            w0 = -w;

            // calculate noise variances
            float xs = (a1 * Mathf.Pow(Speed, 2)) + (a2 * Mathf.Pow(w, 2));
            float ys = (a3 * Mathf.Pow(Speed, 2)) + (a4 * Mathf.Pow(w, 2));
            float ts = Mathf.Sqrt(xs + ys) * heading_noise_scale;

            // add noise to velocities
            mRigidBody.velocity += new Vector3(SampleNormDist(0.0f, xs), 0f, SampleNormDist(0.0f, ys));
            mRigidBody.angularVelocity += new Vector3(0f, SampleNormDist(0.0f, ts), 0f);

        } else if (Manager.state == "FINISHED") {
            // stop robot
            mRigidBody.angularVelocity = Vector3.zero;
            mRigidBody.velocity = Vector3.zero;

            finished = true;
        }
    }

    // reset all gameobjects and parameters
    private void Reset()
    {
        finished = false;
        initialized = false;

        // destroy landmarks and viapoints
        for (int i = 0; i < est_landmarks.Count; i++)
        {
            Destroy(est_landmarks[i]);
        }
        for (int i = 0; i < est_viapoints.Count; i++)
        {
            Destroy(est_viapoints[i]);
        }

        est_landmarks.Clear();
        est_viapoints.Clear();
        if (robot_estimate != null)
        {
            Destroy(robot_estimate);
        }

        // destroy and recreate path
        if (path != null)
        {
            Destroy(path);
        }
        path = new GameObject();
        path_renderer = path.AddComponent<LineRenderer>();
        path_renderer.positionCount = 0;
        path_renderer.widthMultiplier = 0.1f;
        path.GetComponent<Renderer>().material.color = Color.blue;

        cur_viapoint_num = 0;

        // initialize mu0
        transform.position = init_position;
        mu0 = new Pose(new Vector2(transform.position.x, transform.position.z),
                            GetHeading());

        // initialize sigma0
        sigma0 = Matrix4x4.identity;
        sigma0.SetColumn(3, Vector4.zero);
    }

    // initialize at the beginning of "running" state
    private void Initialize()
    {
        // initialize angular velocity
        mRigidBody.angularVelocity = new Vector3(0f, 0f, 0f);

        landmarks = Manager.landmarks;
        landmark_height = landmarks[0].transform.position.y;

        // initialize robot estimate visualizer
        robot_estimate = Instantiate(robot_estimate_prefab);
        robot_estimate.transform.position = transform.position;
        robot_estimate.transform.rotation = transform.rotation;

        // initialize landmark estimates
        for (int i = 0; i < landmarks.Count; i++)
        {
            GameObject est_landmark = Instantiate(landmark_estimate_prefab);
            est_landmark.transform.position = landmarks[i].transform.position;
            est_landmarks.Add(est_landmark);
        }

        // initialize viapoint estimates;
        for (int i = 1; i < landmarks.Count; i++)
        {
            Vector3 last_landmark_pos = est_landmarks[i - 1].transform.position;
            Vector3 next_landmark_pos = est_landmarks[i].transform.position;
            float mid_x = (last_landmark_pos.x + next_landmark_pos.x) / 2f;
            float mid_z = (last_landmark_pos.z + next_landmark_pos.z) / 2f;
            GameObject viapoint = Instantiate(viapoint_esimate_prefab);
            viapoint.transform.position = new Vector3(mid_x, 0f, mid_z);
            est_viapoints.Add(viapoint);
        }

        // initialize motion noise based on motion noise scale factor
        a1 = 0.05f * odometry_noise_scale;
        a2 = 0.05f * odometry_noise_scale;
        a3 = 0.05f * odometry_noise_scale;
        a4 = 0.05f * odometry_noise_scale;

        // initialize sensor noise based on sensor noise scale factor
        sNoiseCov = Matrix4x4.zero;
        for (int i = 0; i < 3; i++)
        {
            Vector4 row = Vector4.zero;
            row[i] = sensor_noise_scale;
            sNoiseCov.SetRow(i, row);
        }

        // initialize EKF correction-step noise based on EKF noise scale factor
        estSNoiseCov = Matrix4x4.zero;
        for (int i = 0; i < 3; i++)
        {
            Vector4 row = Vector4.zero;
            row[i] = EKF_correction_noise_scale;
            estSNoiseCov.SetRow(i, row);
        }

        initialized = true;
    }

    // recalculate estimated landmark positions based on sensor readings 
    // and estimated robot pose
    private void RecalculateLandmarks(List<Vector2> landmarkReadings)
    {
        for (int i = 0; i < est_landmarks.Count; i++)
        {
            Vector2 pos = GetPosFromDistBearing(mu0,
                landmarkReadings[i].x,
                landmarkReadings[i].y);

            est_landmarks[i].transform.position = new Vector3(pos.x, landmark_height, pos.y);
        }
    }

    // recalculate estimated viapoints based on estimated landmarks
    private void RecalculateViapoints()
    {
        for (int i = 1; i < landmarks.Count; i++)
        {
            Vector3 last_landmark_pos = est_landmarks[i - 1].transform.position;
            Vector3 next_landmark_pos = est_landmarks[i].transform.position;
            float mid_x = (last_landmark_pos.x + next_landmark_pos.x) / 2f;
            float mid_z = (last_landmark_pos.z + next_landmark_pos.z) / 2f;
            est_viapoints[i - 1].transform.position = new Vector3(mid_x, 0f, mid_z);
        }
    }

    // sample from a gaussian distribution
    private float SampleNormDist(float mean, float sig) {
        float sum = 0;
        for (int i = 0; i < 12; i++) {
            sum += Random.value;
        }

        return (sig * (sum - 6.0f)) + mean;
    }

    // return robot's heading as float in radians
    private float GetHeading()
    {
        return (Mathf.Deg2Rad * -transform.eulerAngles.y) + (Mathf.PI / 2.0f);
    }

    // convert heading to y-axis Euler angle in Unity's coordinate frame
    private float HeadingToRotation(float heading) {
        return (heading - (Mathf.PI / 2.0f)) / -Mathf.Deg2Rad;
    }

    // senses the landmarks (with noise) and returns list of Vector2's where
    // each Vector2 is (distance, bearing)
    private List<Vector2> SenseLandmarks() {
        List<Vector2> landmarkPoses = new List<Vector2>();

        // get noise
        float x_noise = 0;
        float y_noise = 0;
        float x_noise_cov = sNoiseCov[0, 0];
        float y_noise_cov = sNoiseCov[1, 1];
        if (!Mathf.Approximately(x_noise_cov, 0))
        {
            x_noise = SampleNormDist(0.0f, x_noise_cov);
        }
        if (!Mathf.Approximately(y_noise_cov, 0))
        {
            y_noise = SampleNormDist(0.0f, y_noise_cov);
        }

        // get distance and bearing between noisy robot pose and each landmark
        for (int i = 0; i < landmarks.Count; i++) {
            // get position of landmark[i]
            Vector2 landmarkPos = new Vector2(landmarks[i].transform.position.x,
                                              landmarks[i].transform.position.z);

            // get differences between x & y of robot and landmark
            float x_diff = landmarkPos.x - transform.position.x;
            float y_diff = landmarkPos.y - transform.position.z;

            // get bearing of landmark w.r.t. robot
            float bearing = Mathf.Atan2(y_diff, x_diff);
            bearing = ((bearing - GetHeading() + Mathf.PI) % (2 * Mathf.PI)) - Mathf.PI;

            // add noise to distances
            x_diff += x_noise;
            y_diff += y_noise;

            // get distance between noisy robot and landmark
            float distance = Mathf.Sqrt(Mathf.Pow(x_diff, 2) + Mathf.Pow(y_diff, 2));
            landmarkPoses.Add(new Vector2(distance, bearing));
        }

        return landmarkPoses;
    }

    // uses EKF to correct robot pose estimate given landmark "sensor readings"
    private Pose EKFLocalization(float dt, List<Vector2> z)
    {
        float w = w0;
        float v = Speed;

        // done to prevent infinite values
        if (Mathf.Approximately(w,0))
        {
            w = 0.000001f;
        }

        // get motion jacobian
        Matrix4x4 G1 = Matrix4x4.identity;
        G1.SetColumn(2, new Vector4(
            (-(v / w) * Mathf.Cos(mu0.heading)) + ((v / w) * Mathf.Cos(mu0.heading + (w * dt))),
            (-(v / w) * Mathf.Sin(mu0.heading)) + ((v / w) * Mathf.Sin(mu0.heading + (w * dt))), 
            1, 
            0));

        G1.SetColumn(3, Vector4.zero);

        // get control-space noise jacobian
        Matrix4x4 V1 = Matrix4x4.identity;
        V1.SetColumn(0, new Vector4(
           (-Mathf.Sin(mu0.heading) + Mathf.Sin(mu0.heading + w * dt)) / w,
            (Mathf.Cos(mu0.heading) - Mathf.Cos(mu0.heading + w * dt)) / w,
            0,
            0));

        V1.SetColumn(1, new Vector4(
            (v * (Mathf.Sin(mu0.heading) - Mathf.Sin(mu0.heading + w * dt)) / (Mathf.Pow(w,2))) + 
                (v * Mathf.Cos(mu0.heading + w * dt) * dt / w),
            (v * (Mathf.Cos(mu0.heading) - Mathf.Cos(mu0.heading + w * dt)) / (Mathf.Pow(w, 2))) + 
                (v * Mathf.Sin(mu0.heading + w * dt) * dt / w),
            dt,
            0));

        V1.SetColumn(2, Vector4.zero);
        V1.SetColumn(3, Vector4.zero);

        // get motion noise covariance
        Matrix4x4 M1 = Matrix4x4.identity;
        M1.SetColumn(0, new Vector4(
            a1 * (Mathf.Pow(v,2)) + a2 * (Mathf.Pow(w,2)),
            0,
            0,
            0));

        M1.SetColumn(1, new Vector4(
            0,
            a3 * Mathf.Pow(v, 2) + a4 * Mathf.Pow(w, 2),
            0,
            0));

        M1.SetColumn(2, Vector4.zero);
        M1.SetColumn(3, Vector4.zero);

        // estimate pose after motion
        Pose mu = GetNextPose(mu0, v, -w, dt);

        // estimate state transition covariance
        Matrix4x4 sigma1 = AddMatrices(G1 * sigma0 * G1.transpose, V1 * M1 * V1.transpose);

        sigma1.SetColumn(3, Vector4.zero);

        // for each landmark
        for (int i = 0; i < landmarks.Count; i++)
        {
            Vector2 landmark_pos = new Vector2(landmarks[i].transform.position.x,
                landmarks[i].transform.position.z);

            // compute landmark's expected distance and bearing
            float expected_dist = Vector2.Distance(mu.pos,landmark_pos);
            float expected_bearing = ((Mathf.Atan2(landmark_pos.y - mu.pos.y, landmark_pos.x - mu.pos.x) -
                mu0.heading +
                Mathf.PI) % (2 * Mathf.PI)) - Mathf.PI;

            // get measurement jacobian
            Matrix4x4 H1 = Matrix4x4.identity;
            H1.SetColumn(0, new Vector4(
                (landmark_pos.x - mu.pos.x) / -expected_dist,
                (landmark_pos.y - mu.pos.y) / Mathf.Pow(expected_dist,2),
                0,
                0));

            H1.SetColumn(1, new Vector4(
                (landmark_pos.y - mu.pos.y) / -expected_dist,
                (landmark_pos.x - mu.pos.x) / Mathf.Pow(expected_dist, 2),
                0,
                0));

            H1.SetColumn(2, new Vector4(
                0,
                -1,
                0,
                0));

            H1.SetColumn(3, Vector4.zero);

            // get measurement covariance
            Matrix4x4 S1 = AddMatrices(H1 * sigma1 * H1.transpose, estSNoiseCov);
            Matrix4x4 S1_inv = S1.inverse;
            S1_inv.SetColumn(3, Vector4.zero);

            // calculate kalman gain
            Matrix4x4 K = (sigma1 * H1.transpose) * S1_inv;

            // correct robot pose estimate
            Vector4 pose_diff_vector = new Vector4(
                z[i].x - expected_dist, 
                z[i].y - expected_bearing, 
                0,
                0);

            Matrix4x4 pose_diff_matrix = Matrix4x4.identity;
            pose_diff_matrix.SetColumn(0, pose_diff_vector);
            pose_diff_matrix.SetColumn(1, Vector4.zero);
            pose_diff_matrix.SetColumn(2, Vector4.zero);
            pose_diff_matrix.SetColumn(3, Vector4.zero);

            Vector4 correction = (K * pose_diff_matrix).GetColumn(0);

            mu.pos += new Vector2(correction.x, correction.y);
            mu.heading += correction.z;

            // deal with branch cut
            mu.heading = ((mu.heading + Mathf.PI) % (2 * Mathf.PI)) - Mathf.PI;

            // get correct state covariance
            sigma0 = SubtractMatrices(Matrix4x4.identity, K * H1) * sigma1;
            sigma0.SetColumn(3, Vector4.zero);
        }

        // return new pose estimate
        return mu;
    }

    // uses motion model to get next pose based on previous pose, forward velocity, 
    // angular velocity, and dt
    private Pose GetNextPose(Pose x0, float v, float w, float dt)
    {
        float new_x;
        float new_y;
        float new_heading;
        if (Mathf.Approximately(w, 0.0f))  // if angular velocity is zero
        {
            new_x = x0.pos.x + (v * Mathf.Cos(x0.heading) * dt);
            new_y = x0.pos.y + (v * Mathf.Sin(x0.heading) * dt);
            new_heading = x0.heading;
        }
        else  // if angular velocity is nonzero
        {
            float r = v / w;
            float wdt = w * dt;

            new_x = x0.pos.x + (-r * Mathf.Sin(x0.heading)) + (r * Mathf.Sin(x0.heading + wdt));
            new_y = x0.pos.y + (r * Mathf.Cos(x0.heading)) + (-r * Mathf.Cos(x0.heading + wdt));
            new_heading = x0.heading + wdt;
        }

        // correct heading for branch cut
        new_heading = ((new_heading + Mathf.PI) % (2 * Mathf.PI)) - Mathf.PI;

        return new Pose(new Vector2(new_x, new_y), new_heading);
    }

    // function to add two matrices
    private Matrix4x4 AddMatrices(Matrix4x4 m1, Matrix4x4 m2) {
        Matrix4x4 result = new Matrix4x4();

        Vector4 col0 = m1.GetColumn(0) + m2.GetColumn(0);
        Vector4 col1 = m1.GetColumn(1) + m2.GetColumn(1);
        Vector4 col2 = m1.GetColumn(2) + m2.GetColumn(2);
        Vector4 col3 = new Vector4(0, 0, 0, 1f); ;

        result.SetColumn(0, col0);
        result.SetColumn(1, col1);
        result.SetColumn(2, col2);
        result.SetColumn(3, col3);
        
        return result;
    }

    // function to subtract two matrices
    private Matrix4x4 SubtractMatrices(Matrix4x4 m1, Matrix4x4 m2)
    {
        Matrix4x4 result = new Matrix4x4();

        Vector4 col0 = m1.GetColumn(0) - m2.GetColumn(0);
        Vector4 col1 = m1.GetColumn(1) - m2.GetColumn(1);
        Vector4 col2 = m1.GetColumn(2) - m2.GetColumn(2);
        Vector4 col3 = new Vector4(0, 0, 0, 1f);

        result.SetColumn(0, col0);
        result.SetColumn(1, col1);
        result.SetColumn(2, col2);
        result.SetColumn(3, col3);

        return result;
    }

    // gets an x,y position based on robot pose, landmark distance, and landmark bearing
    private Vector2 GetPosFromDistBearing(Pose r, float distance, float bearing) {
        float angle = ((bearing + r.heading + Mathf.PI) % (2 * Mathf.PI)) - Mathf.PI;
        float x = r.pos.x + (distance * Mathf.Cos(angle));
        float y = r.pos.y + (distance * Mathf.Sin(angle));

        return new Vector2(x, y);
    }
}
