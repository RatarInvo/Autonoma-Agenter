using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class CarControl : Agent
{
    [Header("Car Settings")]
    public float enginePower = 2000.0f;
    public float wallPenalty = -1.0f;
    public float obstaclePenalty = -1.0f;
    public float turnSpeed = 25.0f;
    public float turnSmoothness = 5.0f;
    public float maxBrakeTorque = 3000.0f;

    [Header("Car References")]
    public Transform[] wheels;
    public Transform[] wheelMeshes;
    public Transform centerOfMass;
    public GameObject steeringWheel;

    public Transform landingPad;

    [Header("Arrival")]
    public bool soloTraining = true;
    public float arrivalRadius = 10.0f;
    public float arrivalSpeed = 0.5f;
    public int arrivalHoldSteps = 50;
    public float arrivalReward = 5.0f;
    public float centeringReward = 8.0f;

    [Header("Shaping")]
    public float progressRewardScale = 1.0f;
    public float timePenalty = -0.000667f;

    [Header("Rollover")]
    public float rolloverPenalty = -1.0f;
    public float rolloverUprightLimit = 0.3f;
    public int rolloverHoldSteps = 30;

    [Header("Ground Probe")]
    public float groundProbeDistance = 5.0f;
    public LayerMask groundMask = ~0;

    [Header("Observations")]
    public float distanceNormalizer = 707.0f;

    private Rigidbody rb;

    // training area for car
    private IRoverEnvironment environment;

    private float currentTurnAngle = 0.0f;

    private float previousDistance;

    // distance at the start of an episode
    private float initialDistance;

    private int arrivalTimer;

    private int rolloverTimer;

    private bool parked;

    // true once the rover is inside the meeting point
    public bool IsParked => parked;

    public enum EpisodeOutcome
    {
        TimedOut = 0,
        Arrived = 1,
        RolledOver = 2,
        HitObstacle = 3,
        HitWall = 4,
    }

    public EpisodeOutcome LastOutcome { get; private set; }

    private EpisodeOutcome currentOutcome;

    private bool outcomeSet;

    private readonly int[] outcomeCounts = new int[5];

    public int OutcomeCount(EpisodeOutcome outcome) => outcomeCounts[(int)outcome];

    public void ResetOutcomeCounts() => System.Array.Clear(outcomeCounts, 0, outcomeCounts.Length);

    private void EndWith(EpisodeOutcome outcome)
    {
        currentOutcome = outcome;

        outcomeSet = true;

        EndEpisode();
    }

    public Transform LandingPad => landingPad;

    // Starting position for resetting the car
    private Vector3 startPosition;
    private Quaternion startRotation;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();

        rb.centerOfMass = centerOfMass.localPosition;

        startPosition = transform.position;
        startRotation = transform.rotation;

        environment = GetComponentInParent<IRoverEnvironment>();
    }

    public override void OnEpisodeBegin()
    {
        if (CompletedEpisodes > 0)
        {
            LastOutcome = outcomeSet ? currentOutcome : EpisodeOutcome.TimedOut;

            outcomeCounts[(int)LastOutcome]++;
        }

        outcomeSet = false;

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        Vector3 spawnPosition = startPosition;
        Quaternion spawnRotation = startRotation;

        if (environment != null)
        {
            environment.ResetEnvironment();

            spawnPosition = environment.RoverSpawnPosition;
            spawnRotation = environment.RoverSpawnRotation;
        }

        transform.SetPositionAndRotation(spawnPosition, spawnRotation);

        rb.position = spawnPosition;
        rb.rotation = spawnRotation;

        currentTurnAngle = 0f;

        arrivalTimer = 0;

        rolloverTimer = 0;

        parked = false;

        previousDistance = HorizontalDistanceToFinish();

        initialDistance = Mathf.Max(previousDistance, 1f);

        foreach (Transform wheel in wheels)
        {
            WheelCollider wheelCollider = wheel.GetComponent<WheelCollider>();

            wheelCollider.motorTorque = 0f;
            wheelCollider.brakeTorque = 0f;
            wheelCollider.steerAngle = 0f;
        }
    }

    public Transform finishPoint;

    private float HorizontalDistanceToFinish()
    {
        Vector3 delta = finishPoint.position - transform.position;

        delta.y = 0f;

        return delta.magnitude;
    }

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

        directionToFinish.y = 0f;

        Vector3 localDirection = transform.InverseTransformDirection(directionToFinish.normalized);

        sensor.AddObservation(localDirection);

        sensor.AddObservation(Mathf.Clamp01(HorizontalDistanceToFinish() / distanceNormalizer));

        sensor.AddObservation(transform.up.x);

        sensor.AddObservation(transform.up.z);

        Vector3 localNormal = GroundNormalLocal();

        sensor.AddObservation(localNormal.x);

        sensor.AddObservation(localNormal.z);
    }

    private Vector3 GroundNormalLocal()
    {
        bool hit = Physics.Raycast(transform.position + Vector3.up, Vector3.down, out RaycastHit ground,
            groundProbeDistance, groundMask, QueryTriggerInteraction.Ignore);

        if (!hit)
        {
            return Vector3.zero;
        }

        return transform.InverseTransformDirection(ground.normal);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float steering = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);

        float throttle = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);

        float brake = Mathf.Clamp01(actions.ContinuousActions[2]);

        ApplyCarControls(steering, throttle, brake);

        float distanceToFinish = HorizontalDistanceToFinish();

        AddReward((previousDistance - distanceToFinish) / initialDistance * progressRewardScale);

        previousDistance = distanceToFinish;

        // Small penalty for taking time
        AddReward(timePenalty);
    }

    private void ApplyCarControls(float steering, float throttle, float brake)
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

            wheelCollider.brakeTorque = brake * maxBrakeTorque;

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
        float brake = 0f;

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

        if (Input.GetKey(KeyCode.Space))
        {
            brake = 1f;
        }

        actions[0] = steering;
        actions[1] = throttle;
        actions[2] = brake;
    }

    private void FixedUpdate()
    {
        CheckArrival();

        CheckRollover();
    }

    private void CheckArrival()
    {
        float distanceToFinish = HorizontalDistanceToFinish();

        bool stoppedInside = distanceToFinish <= arrivalRadius && rb.linearVelocity.magnitude <= arrivalSpeed;

        if (!stoppedInside)
        {
            arrivalTimer = 0;

            parked = false;

            return;
        }

        if (parked)
        {
            return;
        }

        arrivalTimer++;

        if (arrivalTimer < arrivalHoldSteps)
        {
            return;
        }

        // held inside the meeting point long enough to count as parked.
        parked = true;

        AddReward(arrivalReward
            + centeringReward * Mathf.Clamp01(1.0f - distanceToFinish / arrivalRadius));

        currentOutcome = EpisodeOutcome.Arrived;

        outcomeSet = true;

        if (soloTraining)
        {
            EndEpisode();
        }
    }

    private void CheckRollover()
    {
        if (transform.up.y > rolloverUprightLimit)
        {
            rolloverTimer = 0;

            return;
        }

        rolloverTimer++;

        if (rolloverTimer < rolloverHoldSteps)
        {
            return;
        }

        AddReward(rolloverPenalty);

        EndWith(EpisodeOutcome.RolledOver);
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("building"))
        {
            AddReward(obstaclePenalty);

            EndWith(EpisodeOutcome.HitObstacle);
        }

        if (collision.gameObject.CompareTag("rock"))
        {
            AddReward(obstaclePenalty);

            EndWith(EpisodeOutcome.HitObstacle);
        }

        if (collision.gameObject.CompareTag("walls"))
        {
            AddReward(wallPenalty);

            EndWith(EpisodeOutcome.HitWall);
        }
    }
}