using UnityEngine;
using UnityEngine.InputSystem;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class DroneMove : Agent
{
    [Header("Flight Control")]
    [SerializeField] private float stabilizingSpeed = 3f;
    [SerializeField] private float movementForce = 20f;
    [SerializeField] private float tiltAngle = 30f;
    [SerializeField] private float tiltSpeed = 5f;
    [SerializeField] private float verticalDamping = 5f;
    [SerializeField] private float horizontalDamping = 1f;
    [SerializeField] private float hoverHeightGain = 8f;

    [Header("Finish Line")]
    [SerializeField] private float rayLength = 300f;
    [SerializeField] private LayerMask rayMask = ~0;
    [SerializeField] private float finishReward = 1f;
    [SerializeField] private float progressRewardScale = 0.4f;
    [SerializeField] private float timePenalty = -0.001f;

    [Header("Truck Landing")]
    [SerializeField] private CarControl truck;
    [SerializeField] private bool coordinateTruckEpisode = true;
    [SerializeField] private float waitingHeight = 10f;
    [SerializeField] private float landingHeight = 0.25f;
    [SerializeField] private float landingHorizontalGain = 3f;
    [SerializeField] private float landingVerticalGain = 4f;
    [SerializeField] private float landingSpeed = 1f;
    [SerializeField] private float landingRadius = 1.5f;
    [SerializeField] private float landingHeightTolerance = 0.75f;
    [SerializeField] private float landingReward = 5f;
    [SerializeField] private float finishProximityRadius = 10f;
    [SerializeField] private float powerOffHeightAbovePad = 0.50f;

    [Header("Obstacle Sensors")]
    [SerializeField] private float obstacleRayLength = 300f;
    [SerializeField] private float obstacleSphereRadius = 0.8f;
    [SerializeField] private float obstacleVerticalAngle = 30f;

    [Header("Drone Visuals")]
    [SerializeField] private Transform[] propellers;
    [SerializeField] private float propellerSpeed = 500f;

    private Rigidbody rb;
    private Quaternion targetRotation;

    private Vector3 startingPosition;
    private Quaternion startingRotation;

    private float verticalInput;
    private Vector2 movementInput;

    private bool episodeEnding;
    private bool dronePowered = true;

    private float previousFinishDistance;
    private float hoverHeight;

    private TrainingArea area;

    private Transform finishCenter;
    private LandingPhase landingPhase;

    private enum LandingPhase
    {
        Approach,
        WaitingForTruck,
        Descending,
    }

    protected override void Awake()
    {
        rb = GetComponent<Rigidbody>();

        startingPosition = transform.position;
        startingRotation = transform.rotation;
        targetRotation = startingRotation;
        area = GetComponentInParent<TrainingArea>();
        FindTruck();
    }

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();

        startingPosition = transform.position;
        startingRotation = transform.rotation;
        targetRotation = startingRotation;

        area = GetComponentInParent<TrainingArea>();
        FindTruck();
    }

    public override void OnEpisodeBegin()
    {
        episodeEnding = false;
        dronePowered = true;

        if (rb != null)
        {
            rb.isKinematic = false;
        }

        ResetState();

        landingPhase = LandingPhase.Approach;
        finishCenter = area != null ? area.meetingPoint : null;

        Vector3 spawnPosition = startingPosition;
        Quaternion spawnRotation = startingRotation;

        if (area != null)
        {
            area.ResetArea();

            spawnPosition = area.DroneSpawnPosition;
            spawnRotation = area.DroneSpawnRotation;
        }

        ResetToSpawn(spawnPosition, spawnRotation);

        previousFinishDistance = FinishDistance();
    }

    public void Reset()
    {
        ResetToSpawn(startingPosition, startingRotation);
    }

    public void ResetToSpawn(Vector3 spawnPosition, Quaternion spawnRotation)
    {
        dronePowered = true;
        transform.SetPositionAndRotation(spawnPosition, spawnRotation);
        hoverHeight = spawnPosition.y;

        if (rb != null)
        {
            rb.isKinematic = false;
            rb.position = spawnPosition;
            rb.rotation = spawnRotation;
        }

        ResetState();
    }

    private void ResetState()
    {
        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        verticalInput = 0f;
        movementInput = Vector2.zero;

        targetRotation = transform.rotation;
        landingPhase = LandingPhase.Approach;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (sensor == null)
        {
            Debug.LogError("DroneMove.CollectObservations received a null VectorSensor.", this);
            return;
        }

        if (rb == null)
        {
            rb = GetComponent<Rigidbody>();

            if (rb == null)
            {
                Debug.LogError("DroneMove requires a Rigidbody component.", this);
                return;
            }
        }

        // Lokal linjär hastighet.
        Vector3 localVelocity =
            transform.InverseTransformDirection(rb.linearVelocity);

        sensor.AddObservation(
            Mathf.Clamp(localVelocity.x / movementForce, -1f, 1f)
        );

        sensor.AddObservation(
            Mathf.Clamp(localVelocity.y / movementForce, -1f, 1f)
        );

        sensor.AddObservation(
            Mathf.Clamp(localVelocity.z / movementForce, -1f, 1f)
        );

        // Lokal rotationshastighet.
        Vector3 localAngularVelocity =
            transform.InverseTransformDirection(rb.angularVelocity);

        sensor.AddObservation(
            Mathf.Clamp(localAngularVelocity.x / 10f, -1f, 1f)
        );

        sensor.AddObservation(
            Mathf.Clamp(localAngularVelocity.y / 10f, -1f, 1f)
        );

        sensor.AddObservation(
            Mathf.Clamp(localAngularVelocity.z / 10f, -1f, 1f)
        );

        // Drönarens lutning.
        sensor.AddObservation(transform.up.x);
        sensor.AddObservation(transform.up.y);
        sensor.AddObservation(transform.up.z);

        // Riktning och avstånd till MeetingPoint.
        RaycastHit finishHit;

        Vector3 finishDirection =
            FindClosestTaggedRay("finishline", out finishHit);

        Vector3 localFinishDirection =
            transform.InverseTransformDirection(finishDirection);

        sensor.AddObservation(localFinishDirection.x);
        sensor.AddObservation(localFinishDirection.y);
        sensor.AddObservation(localFinishDirection.z);

        sensor.AddObservation(
            finishHit.collider == null
                ? 1f
                : Mathf.Clamp01(finishHit.distance / rayLength)
        );

        // 24 hinderobservationer:
        // 8 horisontellt
        // 8 uppåt
        // 8 nedåt
        //AddObstacleObservations(sensor);
    }

    private void AddObstacleObservations(VectorSensor sensor)
    {
        // Horisontella riktningar:
        // framåt, framåt-höger, höger, bakåt-höger,
        // bakåt, bakåt-vänster, vänster, framåt-vänster.
        for (int i = 0; i < 8; i++)
        {
            float angle = i * 45f;

            Vector3 localDirection =
                Quaternion.Euler(0f, angle, 0f) * Vector3.forward;

            sensor.AddObservation(
                ObstacleSphereCastDistance(localDirection)
            );
        }

        // Riktningar uppåt.
        for (int i = 0; i < 8; i++)
        {
            float angle = i * 45f;

            Vector3 localDirection =
                Quaternion.Euler(-obstacleVerticalAngle, angle, 0f)
                * Vector3.forward;

            sensor.AddObservation(
                ObstacleSphereCastDistance(localDirection)
            );
        }

        // Riktningar nedåt
        for (int i = 0; i < 8; i++)
        {
            float angle = i * 45f;

            Vector3 localDirection =
                Quaternion.Euler(obstacleVerticalAngle, angle, 0f)
                * Vector3.forward;

            sensor.AddObservation(
                ObstacleSphereCastDistance(localDirection)
            );
        }
    }

    private float ObstacleSphereCastDistance(Vector3 localDirection)
    {
        Vector3 worldDirection =
            transform.TransformDirection(localDirection).normalized;

        RaycastHit[] hits = Physics.SphereCastAll(
            transform.position,
            obstacleSphereRadius,
            worldDirection,
            obstacleRayLength,
            rayMask,
            QueryTriggerInteraction.Ignore
        );

        float closestDistance = obstacleRayLength;
        bool foundObstacle = false;

        foreach (RaycastHit hit in hits)
        {
            if (hit.collider == null)
            {
                continue;
            }

            if (!IsTaggedAsObstacle(hit.collider))
            {
                continue;
            }

            if (hit.distance < closestDistance)
            {
                closestDistance = hit.distance;
                foundObstacle = true;
            }
        }

        if (!foundObstacle)
        {
            return 1f;
        }

        return Mathf.Clamp01(closestDistance / obstacleRayLength);
    }

    private Vector3 FindClosestTaggedRay(
        string tag,
        out RaycastHit closestHit)
    {
        closestHit = default;

        float closestDistance = rayLength;
        Vector3 closestDirection = Vector3.zero;

        for (int i = 0; i < 8; i++)
        {
            float angle = i * 45f;

            Vector3 direction =
                Quaternion.Euler(0f, angle, 0f) * transform.forward;

            if (!Physics.Raycast(
                    transform.position,
                    direction,
                    out RaycastHit hit,
                    rayLength,
                    rayMask,
                    QueryTriggerInteraction.Collide))
            {
                continue;
            }

            if (!IsTaggedWithTagInParent(hit.collider, tag))
            {
                continue;
            }

            if (hit.distance >= closestDistance)
            {
                continue;
            }

            closestHit = hit;
            closestDistance = hit.distance;
            closestDirection = direction;
        }

        return closestDirection;
    }

    private float FinishDistance()
    {
        FindClosestTaggedRay(
            "finishline",
            out RaycastHit finishHit
        );

        if (finishHit.collider == null)
        {
            return rayLength;
        }

        return finishHit.distance;
    }

    private bool IsTaggedAsObstacle(Collider collider)
    {
        return IsTaggedWithTagInParent(collider, "building")
            || IsTaggedWithTagInParent(collider, "rock");
    }

    private bool IsTaggedWithTagInParent(
        Collider collider,
        string tag)
    {
        if (collider == null)
        {
            return false;
        }

        Transform current = collider.transform;

        while (current != null)
        {
            if (current.CompareTag(tag))
            {
                return true;
            }

            current = current.parent;
        }

        return false;
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.collider == null)
        {
            return;
        }

        if (IsTaggedWithTagInParent(
                collision.collider,
                "finishline"))
        {
            BeginWaitingForTruck(collision.collider.transform);
            return;
        }

        if (IsTaggedAsObstacle(collision.collider))
        {
            ResetAfterObstacleHit();
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (IsTaggedWithTagInParent(other, "finishline"))
        {
            BeginWaitingForTruck(other.transform);
            return;
        }

        if (IsTaggedAsObstacle(other))
        {
            ResetAfterObstacleHit();
        }
    }

    private void BeginWaitingForTruck(Transform finishTransform)
    {
        if (episodeEnding || landingPhase != LandingPhase.Approach)
        {
            return;
        }

        landingPhase = LandingPhase.WaitingForTruck;
        finishCenter = area != null && area.meetingPoint != null
            ? area.meetingPoint
            : finishTransform;

        AddReward(finishReward);
    }

    private void FindTruck()
    {
        if (truck == null && area != null && area.rover != null)
        {
            truck = area.rover;
        }

        if (truck == null && area != null)
        {
            truck = area.GetComponentInChildren<CarControl>(true);
        }

        if (coordinateTruckEpisode && truck != null)
        {
            truck.soloTraining = false;
        }
    }

    private void UpdateLandingPhase()
    {
        if (truck == null)
        {
            FindTruck();
        }

        if (landingPhase == LandingPhase.Approach
            && finishCenter != null
            && HorizontalDistanceTo(finishCenter.position)
                <= finishProximityRadius)
        {
            BeginWaitingForTruck(finishCenter);
        }

        if (landingPhase == LandingPhase.WaitingForTruck
            && truck != null
            && truck.IsParked
            && truck.LandingPad != null)
        {
            landingPhase = LandingPhase.Descending;
        }

        if (landingPhase != LandingPhase.Descending
            || truck == null
            || truck.LandingPad == null)
        {
            return;
        }

        Vector3 landingPosition = truck.LandingPad.position
            + Vector3.up * powerOffHeightAbovePad;

        Vector3 horizontalError = landingPosition - transform.position;
        horizontalError.y = 0f;

        bool readyToPowerOff = horizontalError.magnitude <= landingRadius
            && transform.position.y <= landingPosition.y + 0.05f;

        if (readyToPowerOff && !episodeEnding)
        {
            episodeEnding = true;
            AddReward(landingReward);
            PowerOffDrone();
            Debug.Log("Drone has landed successfully!");
        }
    }

    private void PowerOffDrone()
    {
        dronePowered = false;
        verticalInput = 0f;
        movementInput = Vector2.zero;
        rb.linearVelocity = new Vector3(
            0f,
            Mathf.Min(rb.linearVelocity.y, 0f),
            0f
        );
        rb.angularVelocity = Vector3.zero;
        rb.isKinematic = false;
        rb.useGravity = true;
    }

    private float HorizontalDistanceTo(Vector3 position)
    {
        Vector3 offset = position - transform.position;
        offset.y = 0f;
        return offset.magnitude;
    }

    private void ApplyLandingControl()
    {
        if (landingPhase == LandingPhase.Descending
            && (truck == null || truck.LandingPad == null))
        {
            return;
        }

        Vector3 targetPosition = landingPhase == LandingPhase.WaitingForTruck
            ? finishCenter != null
                ? finishCenter.position + Vector3.up * waitingHeight
                : transform.position
            : truck.LandingPad.position
                + Vector3.up * powerOffHeightAbovePad;

        Vector3 horizontalError = targetPosition - transform.position;
        horizontalError.y = 0f;

        Vector3 horizontalVelocity = rb.linearVelocity;
        horizontalVelocity.y = 0f;

        rb.AddForce(
            horizontalError * landingHorizontalGain
            - horizontalVelocity * horizontalDamping,
            ForceMode.Force
        );

        verticalInput = Mathf.Clamp(
            (targetPosition.y - transform.position.y)
            * landingVerticalGain
            - rb.linearVelocity.y * verticalDamping,
            -movementForce,
            movementForce
        );

        movementInput = Vector2.zero;
    }

    private void ResetAfterObstacleHit()
    {
        if (episodeEnding)
        {
            return;
        }

        episodeEnding = true;

        AddReward(-1f);
        EndEpisode();
    }

    public override void Heuristic(
        in ActionBuffers actionsOut)
    {
        var discreteActionsOut =
            actionsOut.DiscreteActions;

        discreteActionsOut[0] = 0;
        discreteActionsOut[1] = 0;
        discreteActionsOut[2] = 0;

        if (Keyboard.current == null)
        {
            return;
        }

        // Framåt eller bakåt.
        if (Keyboard.current.wKey.isPressed)
        {
            discreteActionsOut[0] = 1;
        }
        else if (Keyboard.current.sKey.isPressed)
        {
            discreteActionsOut[0] = 2;
        }

        // Vänster eller höger.
        if (Keyboard.current.aKey.isPressed)
        {
            discreteActionsOut[1] = 1;
        }
        else if (Keyboard.current.dKey.isPressed)
        {
            discreteActionsOut[1] = 2;
        }

        // Uppåt eller nedåt.
        if (Keyboard.current.eKey.isPressed)
        {
            discreteActionsOut[2] = 1;
        }
        else if (Keyboard.current.qKey.isPressed)
        {
            discreteActionsOut[2] = 2;
        }
    }

    public override void OnActionReceived(
        ActionBuffers actions)
    {
        var discreteActions =
            actions.DiscreteActions;

        movementInput = Vector2.zero;

        // Framåt eller bakåt.
        if (discreteActions[0] == 1)
        {
            movementInput.y = 1f;
        }
        else if (discreteActions[0] == 2)
        {
            movementInput.y = -1f;
        }

        // Vänster eller höger.
        if (discreteActions[1] == 1)
        {
            movementInput.x = -1f;
        }
        else if (discreteActions[1] == 2)
        {
            movementInput.x = 1f;
        }

        // Uppåt eller nedåt.
        verticalInput = 0f;

        if (discreteActions[2] == 1)
        {
            verticalInput = movementForce;
        }
        else if (discreteActions[2] == 2)
        {
            verticalInput = -movementForce;
        }

        float finishDistance = FinishDistance();

        AddReward(
            (previousFinishDistance - finishDistance)
            / rayLength
            * progressRewardScale
        );

        previousFinishDistance = finishDistance;

        AddReward(timePenalty);
    }

    private void FixedUpdate()
    {
        if (rb == null || !dronePowered)
        {
            return;
        }

        UpdateLandingPhase();

        if (!dronePowered)
        {
            return;
        }

        if (landingPhase != LandingPhase.Approach)
        {
            ApplyLandingControl();
        }

        Hover(verticalInput);
        TiltDrone(movementInput);
        MoveDrone(movementInput);
        StabilizeHorizontalDrift();
        AutoLevel(movementInput);
    }

    private void Update()
    {
        if (dronePowered)
        {
            AnimatePropellers();
        }
    }

    private void Hover(float vertical)
    {
        if (rb.isKinematic)
        {
            return;
        }

        float gravity = Physics.gravity.magnitude;

        float angle =
            Vector3.Angle(transform.up, Vector3.up);

        float cosine =
            Mathf.Cos(angle * Mathf.Deg2Rad);

        cosine = Mathf.Max(cosine, 0.1f);

        float baseHoverForce =
            (rb.mass * gravity) / cosine;

        float totalHoverForce =
            baseHoverForce + vertical;

        rb.AddForce(
            transform.up * totalHoverForce,
            ForceMode.Force
        );

        if (Mathf.Approximately(vertical, 0f))
        {
            float heightError = hoverHeight - rb.position.y;

            rb.AddForce(
                Vector3.up * heightError * hoverHeightGain,
                ForceMode.Force
            );

            float dampedVerticalVelocity =
                Mathf.MoveTowards(
                    rb.linearVelocity.y,
                    0f,
                    verticalDamping
                    * Time.fixedDeltaTime
                );

            rb.linearVelocity = new Vector3(
                rb.linearVelocity.x,
                dampedVerticalVelocity,
                rb.linearVelocity.z
            );
        }
    }

    private void TiltDrone(Vector2 move)
    {
        float tiltX = move.y * tiltAngle;
        float tiltZ = -move.x * tiltAngle;

        Quaternion desiredTilt =
            Quaternion.Euler(
                tiltX,
                targetRotation.eulerAngles.y,
                tiltZ
            );

        targetRotation =
            Quaternion.Slerp(
                targetRotation,
                desiredTilt,
                Time.fixedDeltaTime * tiltSpeed
            );

        rb.MoveRotation(targetRotation);
    }

    private void MoveDrone(Vector2 move)
    {
        Vector3 forward =
            Vector3.ProjectOnPlane(
                transform.forward,
                Vector3.up
            ).normalized;

        Vector3 right =
            Vector3.ProjectOnPlane(
                transform.right,
                Vector3.up
            ).normalized;

        Vector3 force =
            forward * move.y * movementForce
            + right * move.x * movementForce;

        rb.AddForce(force, ForceMode.Force);
    }

    private void StabilizeHorizontalDrift()
    {
        Vector3 horizontalVelocity =
            new Vector3(
                rb.linearVelocity.x,
                0f,
                rb.linearVelocity.z
            );

        rb.AddForce(
            -horizontalVelocity * horizontalDamping,
            ForceMode.Force
        );
    }

    private void AutoLevel(Vector2 move)
    {
        if (move.sqrMagnitude > 0.01f)
        {
            return;
        }

        Quaternion upright =
            Quaternion.Euler(
                0f,
                targetRotation.eulerAngles.y,
                0f
            );

        targetRotation =
            Quaternion.Slerp(
                targetRotation,
                upright,
                Time.fixedDeltaTime * stabilizingSpeed
            );

        rb.MoveRotation(targetRotation);
    }

    private void AnimatePropellers()
    {
        if (propellers == null)
        {
            return;
        }

        float totalSpeed =
            propellerSpeed
            + Mathf.Abs(verticalInput) * 500f;

        foreach (Transform propeller in propellers)
        {
            if (propeller == null)
            {
                continue;
            }

            propeller.Rotate(
                Vector3.forward,
                totalSpeed * Time.deltaTime
            );
        }
    }
}