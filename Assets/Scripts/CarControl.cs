using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class CarControl : Agent
{
    [Header("Car Settings")]
    public float enginePower = 2000.0f;
    public float turnSpeed = 25.0f;
    public float turnSmoothness = 5.0f;

    [Header("Car References")]
    public Transform[] wheels;
    public Transform[] wheelMeshes;
    public Transform centerOfMass;
    public GameObject steeringWheel;

    private Rigidbody rb;

    private float currentTurnAngle = 0.0f;

    // Starting position for resetting the car
    private Vector3 startPosition;
    private Quaternion startRotation;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();

        rb.centerOfMass = centerOfMass.localPosition;

        startPosition = transform.position;
        startRotation = transform.rotation;
    }

    public override void OnEpisodeBegin()
    {
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        transform.position = startPosition;
        transform.rotation = startRotation;

        currentTurnAngle = 0f;

        foreach (Transform wheel in wheels)
        {
            WheelCollider wheelCollider = wheel.GetComponent<WheelCollider>();

            wheelCollider.motorTorque = 0f;
            wheelCollider.brakeTorque = 0f;
            wheelCollider.steerAngle = 0f;
        }
    }

    public Transform finishPoint;

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 localVelocity = transform.InverseTransformDirection(rb.linearVelocity);

        sensor.AddObservation(Mathf.Clamp(localVelocity.x / 20f, -1f, 1f));

        sensor.AddObservation(Mathf.Clamp(localVelocity.y / 20f, -1f, 1f));

        sensor.AddObservation(Mathf.Clamp(localVelocity.z / 20f, -1f, 1f));

        Vector3 localAngularVelocity = transform.InverseTransformDirection(rb.angularVelocity);

        sensor.AddObservation(Mathf.Clamp(localAngularVelocity.x / 10f, -1f, 1f));

        sensor.AddObservation(Mathf.Clamp(localAngularVelocity.y / 10f, -1f, 1f));

        sensor.AddObservation(Mathf.Clamp(localAngularVelocity.z / 10f, -1f, 1f));

        Vector3 directionToFinish = finishPoint.position - transform.position;

        float distanceToFinish = directionToFinish.magnitude;

        Vector3 localDirection = transform.InverseTransformDirection(directionToFinish.normalized);

        sensor.AddObservation(localDirection);

        sensor.AddObservation(Mathf.Clamp(distanceToFinish / 100f, 0f, 1f));
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("finishline"))
        {
            AddReward(1.0f);

            EndEpisode();
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float steering = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);

        float throttle = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);

        ApplyCarControls(steering, throttle);

        // Encourage forward movement
        float forwardSpeed = Vector3.Dot(rb.linearVelocity, transform.forward);

        float normalizedSpeed = Mathf.Clamp01(forwardSpeed / 20f);

        AddReward(normalizedSpeed * 0.01f);

        // Small penalty for taking time
        AddReward(-0.001f);
    }

    private void ApplyCarControls(float steering, float throttle)
    {
        // Convert ML action (-1 to +1)
        // into actual steering angle.
        float targetTurnAngle = steering * turnSpeed;

        currentTurnAngle = Mathf.Lerp(currentTurnAngle, targetTurnAngle, Time.fixedDeltaTime * turnSmoothness);

        // Steering wheel visual
        if (steeringWheel != null)
        {
            steeringWheel.transform.localEulerAngles = new Vector3(-64, 0, currentTurnAngle * 3);
        }

        // Apply to wheels
        for (int i = 0; i < wheels.Length; i++)
        {
            WheelCollider wheelCollider = wheels[i].GetComponent<WheelCollider>();

            // First two wheels steer
            if (i < 2)
            {
                wheelCollider.steerAngle = currentTurnAngle;
            }
            else
            {
                wheelCollider.steerAngle = 0f;
            }

            // All wheels receive engine torque
            wheelCollider.motorTorque = throttle * enginePower;

            // Update visual wheel
            if (i < wheelMeshes.Length)
            {
                Vector3 wheelPosition;
                Quaternion wheelRotation;

                wheelCollider.GetWorldPose(out wheelPosition,out wheelRotation);

                wheelMeshes[i].position = wheelPosition;

                wheelMeshes[i].rotation = wheelRotation;
            }
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var actions = actionsOut.ContinuousActions;

        float steering = 0f;
        float throttle = 0f;

        if (Input.GetKey(KeyCode.LeftArrow))
        {
            steering = -1f;
        }
        else if (Input.GetKey(KeyCode.RightArrow))
        {
            steering = 1f;
        }

        if (Input.GetKey(KeyCode.UpArrow))
        {
            throttle = 1f;
        }
        else if (Input.GetKey(KeyCode.DownArrow))
        {
            throttle = -1f;
        }

        actions[0] = steering;
        actions[1] = throttle;
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("building"))
        {
            AddReward(-1f);

            EndEpisode();
        }

        if (collision.gameObject.CompareTag("rock"))
        {
            AddReward(-1f);

            EndEpisode();
        }
    }
}