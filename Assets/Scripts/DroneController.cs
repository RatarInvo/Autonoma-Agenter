using UnityEngine;
using UnityEngine.InputSystem;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class DroneMove : Agent
{       
    public float stabilizingSpeed = 3f; 
    private float baseThrustOffset = 0f;   
    private float defaultBaseThrust;        
    public float movementForce = 20f;         
    public float tiltAngle = 30f;           
    public float tiltSpeed = 5f;                      
    public float verticalDamping = 5f;
    public float horizontalDamping = 1f;

    [Header("Finish Line")]
    [SerializeField] private int finishHoldSteps = 50;
    [SerializeField] private float finishHoldSpeed = 0.5f;
    [SerializeField] private float finishHoldReward = 0.02f;
    [SerializeField] private float timePenalty = -0.0002f;
    [SerializeField] private float rayLength = 40f;
    [SerializeField] private LayerMask rayMask = ~0;

    public Transform[] propellers;            
    public float propellerSpeed = 1000f;      
    [SerializeField] private bool requestDecisionsInCode = true;
    [SerializeField] private float finishLineReward = 1f;

    private Rigidbody rb;
    private Quaternion targetRotation;
    private Vector3 startingPosition;
    private Quaternion startingRotation;
    private float verticalInput;
    private Vector2 movementInput;
    private bool reachedFinishLine;
    private int finishHoldTimer;

        protected override void Awake()
    {
        rb = GetComponent<Rigidbody>();
        startingPosition = transform.position;
        startingRotation = transform.rotation;
        targetRotation = startingRotation;
    }

        public override void OnEpisodeBegin()
        {
            Reset();
        }

        public void Reset()
    {
        transform.SetPositionAndRotation(startingPosition, startingRotation);

        Rigidbody body = GetComponent<Rigidbody>();
        if (body != null)
        {
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
        }

        verticalInput = 0f;
        movementInput = Vector2.zero;
        reachedFinishLine = false;
        finishHoldTimer = 0;
        targetRotation = startingRotation;
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (IsTaggedAsBuilding(collision.collider))
            ResetAfterBuildingHit();
    }

    private void OnTriggerEnter(Collider other)
    {
        if (IsTaggedAsBuilding(other))
        {
            ResetAfterBuildingHit();
            return;
        }

        if (reachedFinishLine || !other.CompareTag("finishline"))
            return;

        reachedFinishLine = true;
        finishHoldTimer = 0;
    }

    private bool IsTaggedAsBuilding(Collider collider)
    {
        Transform current = collider.transform;

        while (current != null)
        {
            if (current.CompareTag("building"))
                return true;

            current = current.parent;
        }

        return false;
    }

    private void ResetAfterBuildingHit()
    {
        AddReward(-1f);
        EndEpisode();
    }

    private void OnTriggerExit(Collider other)
    {
        if (!other.CompareTag("finishline"))
            return;

        reachedFinishLine = false;
        finishHoldTimer = 0;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 localVelocity = transform.InverseTransformDirection(rb.linearVelocity);
        sensor.AddObservation(Mathf.Clamp(localVelocity.x / movementForce, -1f, 1f));
        sensor.AddObservation(Mathf.Clamp(localVelocity.y / movementForce, -1f, 1f));
        sensor.AddObservation(Mathf.Clamp(localVelocity.z / movementForce, -1f, 1f));

        Vector3 localAngularVelocity = transform.InverseTransformDirection(rb.angularVelocity);
        sensor.AddObservation(Mathf.Clamp(localAngularVelocity.x / 10f, -1f, 1f));
        sensor.AddObservation(Mathf.Clamp(localAngularVelocity.y / 10f, -1f, 1f));
        sensor.AddObservation(Mathf.Clamp(localAngularVelocity.z / 10f, -1f, 1f));

        sensor.AddObservation(transform.up.x);
        sensor.AddObservation(transform.up.z);

        RaycastHit finishHit;
        Vector3 finishDirection = FindClosestTaggedRay("finishline", out finishHit);
        Vector3 localFinishDirection = transform.InverseTransformDirection(finishDirection);
        sensor.AddObservation(localFinishDirection.x);
        sensor.AddObservation(localFinishDirection.z);
        sensor.AddObservation(finishHit.collider == null ? 1f : finishHit.distance / rayLength);

        sensor.AddObservation(BuildingRayDistance(transform.forward));
        sensor.AddObservation(BuildingRayDistance(transform.right));
    }

    private Vector3 FindClosestTaggedRay(string tag, out RaycastHit closestHit)
    {
        closestHit = default;
        float closestDistance = rayLength;
        Vector3 closestDirection = Vector3.zero;

        for (int i = 0; i < 8; i++)
        {
            float angle = i * 45f;
            Vector3 direction = Quaternion.Euler(0f, angle, 0f) * transform.forward;

            if (!Physics.Raycast(transform.position, direction, out RaycastHit hit, rayLength, rayMask,
                    QueryTriggerInteraction.Collide) || !hit.collider.CompareTag(tag) || hit.distance >= closestDistance)
                continue;

            closestHit = hit;
            closestDistance = hit.distance;
            closestDirection = direction;
        }

        return closestDirection;
    }

    private float BuildingRayDistance(Vector3 direction)
    {
        if (Physics.Raycast(transform.position, direction, out RaycastHit hit, rayLength, rayMask,
                QueryTriggerInteraction.Ignore) && hit.collider.CompareTag("building"))
            return Mathf.Clamp01(hit.distance / rayLength);

        return 1f;
    }

    void Start()
    {

    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var discreteActionsOut = actionsOut.DiscreteActions;
        discreteActionsOut[0] = 0;
        discreteActionsOut[1] = 0;
        discreteActionsOut[2] = 0;


        if (Keyboard.current == null)
            return;

        if (Keyboard.current.wKey.isPressed)
            discreteActionsOut[0] = 1;
        else if (Keyboard.current.sKey.isPressed)
            discreteActionsOut[0] = 2;

        if (Keyboard.current.aKey.isPressed)
            discreteActionsOut[1] = 1;
        else if (Keyboard.current.dKey.isPressed)
            discreteActionsOut[1] = 2;

        if (Keyboard.current.eKey.isPressed)
            discreteActionsOut[2] = 1;
        else if (Keyboard.current.qKey.isPressed)
            discreteActionsOut[2] = 2;
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        
        var discreteActions = actions.DiscreteActions;
        movementInput = Vector2.zero;

        if (discreteActions[0] == 1)
            movementInput.y = 1f;
        else if (discreteActions[0] == 2)
            movementInput.y = -1f;

        if (discreteActions[1] == 1)
            movementInput.x = -1f;
        else if (discreteActions[1] == 2)
            movementInput.x = 1f;

        verticalInput = 0f;
        if (discreteActions[2] == 1)
            verticalInput = movementForce;
        else if (discreteActions[2] == 2)
            verticalInput = -movementForce;

        AddReward(timePenalty);
    }

    private void FixedUpdate()
    {
        if (requestDecisionsInCode)
            RequestDecision();

        Hover(verticalInput);
        TiltDrone(movementInput);
        MoveDrone(movementInput);
        StabilizeHorizontalDrift();
        AutoLevel(movementInput);
        CheckFinishLineHold();
    }

    private void CheckFinishLineHold()
    {
        if (!reachedFinishLine)
            return;

        if (rb.linearVelocity.magnitude > finishHoldSpeed)
        {
            finishHoldTimer = 0;
            return;
        }

        finishHoldTimer++;
        AddReward(finishHoldReward);

        if (finishHoldTimer < finishHoldSteps)
            return;

        AddReward(finishLineReward);
        EndEpisode();
    }

    void Update()
    {
        AnimatePropellers();
    }

    void Hover(float vertical)
    {
        float gravity = Physics.gravity.magnitude;
        float angle = Vector3.Angle(transform.up, Vector3.up);

        float baseHoverForce = (rb.mass * gravity) / Mathf.Cos(angle * Mathf.Deg2Rad);
        float totalHoverForce = baseHoverForce + baseThrustOffset;

        rb.AddForce(transform.up * (totalHoverForce + vertical), ForceMode.Force);

        if (vertical == 0f)
        {
            float dampedVerticalVelocity = Mathf.MoveTowards(rb.linearVelocity.y, 0f, verticalDamping * Time.fixedDeltaTime);

            rb.linearVelocity = new Vector3(rb.linearVelocity.x, dampedVerticalVelocity, rb.linearVelocity.z);
        }
    }

    void TiltDrone(Vector2 move)
    {
        float tiltX = move.y * tiltAngle;
        float tiltZ = -move.x * tiltAngle;

        Quaternion desiredTilt = Quaternion.Euler(tiltX, targetRotation.eulerAngles.y, tiltZ);
        targetRotation = Quaternion.Slerp(targetRotation, desiredTilt, Time.fixedDeltaTime * tiltSpeed);

        rb.MoveRotation(targetRotation);
    }

    void MoveDrone(Vector2 move)
    {
        Vector3 forward = Vector3.ProjectOnPlane(transform.forward, Vector3.up).normalized;
        Vector3 right = Vector3.ProjectOnPlane(transform.right, Vector3.up).normalized;
        Vector3 force =
            forward * move.y * movementForce +
            right * move.x * movementForce;

        rb.AddForce(force, ForceMode.Force);
    }

    void StabilizeHorizontalDrift()
    {
        Vector3 horizontalVelocity = new Vector3(rb.linearVelocity.x, 0f, rb.linearVelocity.z);
        rb.AddForce(-horizontalVelocity * horizontalDamping, ForceMode.Force);
    }

    void AutoLevel(Vector2 move)
    {
        if (move.sqrMagnitude > 0.01f)
            return;

        Quaternion upright = Quaternion.Euler(0f, targetRotation.eulerAngles.y, 0f);
        targetRotation = Quaternion.Slerp(targetRotation, upright, Time.fixedDeltaTime * stabilizingSpeed);

        rb.MoveRotation(targetRotation);
    }

    void AnimatePropellers()
    {
        float totalSpeed = propellerSpeed + Mathf.Abs(verticalInput) * 500f;

        foreach (Transform prop in propellers)
        {
            prop.Rotate(Vector3.forward, totalSpeed * Time.deltaTime);
        }
    }
}