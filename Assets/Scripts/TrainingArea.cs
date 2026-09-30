using System.Collections.Generic;
using UnityEngine;

public class TrainingArea : MonoBehaviour, IRoverEnvironment
{
    public enum Level
    {
        // fixed start and target
        FixedFlat = 1,

        // start and target randomized every episode.
        RandomFlat = 2,

        // hills on top of the randomized positions.
        RandomHills = 3,

        // obstacles that land in the same spots every episode.
        FixedObstacles = 4,

        // obstacles that move every episode.
        RandomObstacles = 5,
    }

    [Header("Level")]
    public Level level = Level.FixedFlat;

    [Header("Layout")]
    public float areaSize = 150.0f;

    [Header("Ground Mesh")]
    public int gridResolution = 40;
    public float hillAmplitude = 12.0f;
    public float hillFrequency = 0.012f;

    [Header("Fixed Placement")]
    public Vector3 fixedRoverLocal = new Vector3(0.0f, 0.0f, -35.0f);
    public Vector3 fixedMeetingPointLocal = new Vector3(0.0f, 0.0f, 35.0f);

    [Header("Random Placement")]
    public float spawnMargin = 15.0f;
    public float minSeparation = 45.0f;
    public float roverSpawnHeight = 1.0f;
    public int placementAttempts = 40;

    [Header("Obstacles")]
    public GameObject[] obstaclePrefabs;
    public int fixedObstacleCount = 6;
    public int minObstacles = 4;
    public int maxObstacles = 10;
    public float obstacleRadius = 18.0f;
    public float meetingPointRadius = 14.0f;
    public float roverRadius = 8.0f;
    public int fixedObstacleSeed = 20260922;

    [Header("References")]
    public CarControl rover;
    public Transform meetingPoint;
    public MeshFilter groundMeshFilter;
    public MeshCollider groundMeshCollider;
    public Transform obstacleRoot;

    public Vector3 RoverSpawnPosition { get; private set; }

    public Quaternion RoverSpawnRotation { get; private set; }

    private struct Reserved
    {
        public Vector2 center;
        public float radius;
    }

    private readonly List<Reserved> reserved = new List<Reserved>();

    private Mesh groundMesh;

    private Vector2 noiseOffset;

    private bool groundBuilt;

    private Vector3 meetingPointLocal;

    private bool RandomizePositions => level >= Level.RandomFlat;

    private bool HillsEnabled => level >= Level.RandomHills;

    private bool ObstaclesEnabled => level >= Level.FixedObstacles;

    private bool RandomizeObstacles => level >= Level.RandomObstacles;

    private void Awake()
    {
        groundMesh = new Mesh();

        groundMesh.name = "AreaGround";

        groundMesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;

        groundMeshFilter.sharedMesh = groundMesh;

        if (rover != null)
        {
            rover.distanceNormalizer = areaSize * 1.4142f;
        }
    }

    void IRoverEnvironment.ResetEnvironment() => ResetArea();

    public void ResetArea()
    {
        BuildGround();

        reserved.Clear();

        PlaceObstacles();

        PlaceMeetingPoint();

        PlaceRover();
    }

    private float GroundHeight(float localX, float localZ)
    {
        if (!HillsEnabled || hillAmplitude <= 0.0f)
        {
            return 0.0f;
        }

        float nx = (localX + noiseOffset.x) * hillFrequency;

        float nz = (localZ + noiseOffset.y) * hillFrequency;

        float height = Mathf.PerlinNoise(nx, nz) + 0.5f * Mathf.PerlinNoise(nx * 2.3f, nz * 2.3f);

        return height / 1.5f * hillAmplitude;
    }

    private void BuildGround()
    {
        // flat ground never change
        if (groundBuilt && !HillsEnabled)
        {
            return;
        }

        if (HillsEnabled)
        {
            noiseOffset = new Vector2(Random.Range(0.0f, 1000.0f), Random.Range(0.0f, 1000.0f));
        }

        int side = gridResolution + 1;

        float step = areaSize / gridResolution;

        float half = areaSize * 0.5f;

        Vector3[] vertices = new Vector3[side * side];

        Vector2[] uvs = new Vector2[side * side];

        for (int z = 0; z < side; z++)
        {
            for (int x = 0; x < side; x++)
            {
                float localX = -half + x * step;

                float localZ = -half + z * step;

                int index = z * side + x;

                vertices[index] = new Vector3(localX, GroundHeight(localX, localZ), localZ);

                uvs[index] = new Vector2((float)x / gridResolution, (float)z / gridResolution);
            }
        }

        int[] triangles = new int[gridResolution * gridResolution * 6];

        int t = 0;

        for (int z = 0; z < gridResolution; z++)
        {
            for (int x = 0; x < gridResolution; x++)
            {
                int index = z * side + x;

                triangles[t++] = index;

                triangles[t++] = index + side;

                triangles[t++] = index + side + 1;

                triangles[t++] = index;

                triangles[t++] = index + side + 1;

                triangles[t++] = index + 1;
            }
        }

        groundMesh.Clear();

        groundMesh.vertices = vertices;

        groundMesh.uv = uvs;

        groundMesh.triangles = triangles;

        groundMesh.RecalculateNormals();

        groundMesh.RecalculateBounds();

        groundMeshCollider.sharedMesh = null;

        groundMeshCollider.sharedMesh = groundMesh;

        groundBuilt = true;
    }

    private Vector3 SampleGroundPointLocal()
    {
        float half = areaSize * 0.5f - spawnMargin;

        float x = Random.Range(-half, half);

        float z = Random.Range(-half, half);

        return new Vector3(x, GroundHeight(x, z), z);
    }

    private bool IsClear(Vector3 localPoint, float radius)
    {
        Vector2 candidate = new Vector2(localPoint.x, localPoint.z);

        foreach (Reserved item in reserved)
        {
            if (Vector2.Distance(candidate, item.center) < item.radius + radius)
            {
                return false;
            }
        }

        return true;
    }

    private void Reserve(Vector3 localPoint, float radius)
    {
        reserved.Add(new Reserved
        {
            center = new Vector2(localPoint.x, localPoint.z),
            radius = radius,
        });
    }

    private bool TryFindSpot(float radius, out Vector3 localPoint)
    {
        for (int attempt = 0; attempt < placementAttempts; attempt++)
        {
            localPoint = SampleGroundPointLocal();

            if (IsClear(localPoint, radius))
            {
                return true;
            }
        }

        localPoint = SampleGroundPointLocal();

        return false;
    }

    private void PlaceObstacles()
    {
        for (int i = obstacleRoot.childCount - 1; i >= 0; i--)
        {
            GameObject old = obstacleRoot.GetChild(i).gameObject;

            old.SetActive(false);

            Destroy(old);
        }

        if (!ObstaclesEnabled || obstaclePrefabs == null || obstaclePrefabs.Length == 0)
        {
            return;
        }

        Random.State outerState = Random.state;

        if (!RandomizeObstacles)
        {
            Random.InitState(fixedObstacleSeed);
        }

        int count = RandomizeObstacles ? Random.Range(minObstacles, maxObstacles + 1) : fixedObstacleCount;

        for (int i = 0; i < count; i++)
        {
            if (!TryFindSpot(obstacleRadius, out Vector3 localPoint))
            {
                continue;
            }

            GameObject prefab = obstaclePrefabs[Random.Range(0, obstaclePrefabs.Length)];

            GameObject instance = Instantiate(prefab, obstacleRoot);

            instance.transform.localPosition = localPoint;

            instance.transform.localRotation = Quaternion.Euler(0.0f, Random.Range(0.0f, 360.0f), 0.0f);

            Reserve(localPoint, obstacleRadius);
        }

        if (!RandomizeObstacles)
        {
            Random.state = outerState;
        }
    }

    private void PlaceMeetingPoint()
    {
        if (RandomizePositions)
        {
            TryFindSpot(meetingPointRadius, out meetingPointLocal);
        }
        else
        {
            meetingPointLocal = fixedMeetingPointLocal;

            meetingPointLocal.y = GroundHeight(meetingPointLocal.x, meetingPointLocal.z);
        }

        meetingPoint.localPosition = meetingPointLocal;

        Reserve(meetingPointLocal, meetingPointRadius);
    }

    private void PlaceRover()
    {
        Vector3 localPoint;

        if (RandomizePositions)
        {
            Reserve(meetingPointLocal, minSeparation);

            TryFindSpot(roverRadius, out localPoint);
        }
        else
        {
            localPoint = fixedRoverLocal;

            localPoint.y = GroundHeight(localPoint.x, localPoint.z);
        }

        RoverSpawnPosition = transform.TransformPoint(localPoint + Vector3.up * roverSpawnHeight);

        float yaw = RandomizePositions ? Random.Range(0.0f, 360.0f) : 0.0f;

        RoverSpawnRotation = transform.rotation * Quaternion.Euler(0.0f, yaw, 0.0f);

        Reserve(localPoint, roverRadius);
    }
}