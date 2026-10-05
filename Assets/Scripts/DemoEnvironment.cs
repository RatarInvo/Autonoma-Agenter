using System.Collections.Generic;
using UnityEngine;

public class DemoEnvironment : MonoBehaviour, IRoverEnvironment
{
    [Header("References")]
    public Terrain terrain;
    public CarControl rover;
    public Transform meetingPoint;
    public Transform obstacleRoot;
    public GameObject[] obstaclePrefabs;
    public Transform drone;

    [Header("Terrain")]
    public bool generateHills = true;
    public float hillAmplitude = 24.0f;
    public float hillFrequency = 0.012f;

    [Header("Placement")]
    public float margin = 70.0f;
    public float minSeparation = 45.0f;
    public float maxSeparation = 125.0f;
    public float roverSpawnHeight = 1.0f;
    public int placementAttempts = 60;

    [Header("Obstacles")]
    public int minObstacles = 5;
    public int maxObstacles = 50;
    public float obstacleRadius = 20.0f;
    public float corridorHalfWidth = 22.0f;
    public float corridorPadding = 25.0f;
    public int corridorObstacles = 3;
    public float obstacleSpacing = 26.0f;
    public float meetingPointRadius = 14.0f;
    public float roverRadius = 8.0f;

    [Header("Demo Loop")]
    public bool restartAfterArrival = true;
    public float holdSecondsAfterArrival = 10.0f;

    [Header("Marker")]
    public float meetingPointYOffset = 46.3f;

    [Header("Drone")]
    public float droneMinHeight = 30.0f;
    public float droneMaxHeight = 60.0f;
    public float droneMaxOffset = 80.0f;

    public Vector3 RoverSpawnPosition { get; private set; }

    public Quaternion RoverSpawnRotation { get; private set; }

    private struct Reserved
    {
        public Vector2 center;
        public float radius;
    }

    private readonly List<Reserved> reserved = new List<Reserved>();

    private Vector2 noiseOffset;

    private bool terrainBuilt;

    private Vector3 meetingPointGround;

    private Vector3 roverGround;

    private float parkedSeconds;

    public void ResetEnvironment()
    {
        BuildTerrain();

        reserved.Clear();

        PlaceMeetingPoint();

        PlaceRover();

        PlaceObstacles();

        PlaceDrone();

        Physics.SyncTransforms();
    }

    private float GroundHeight(float worldX, float worldZ)
    {
        if (!generateHills || hillAmplitude <= 0.0f)
        {
            return 0.0f;
        }

        float nx = (worldX + noiseOffset.x) * hillFrequency;

        float nz = (worldZ + noiseOffset.y) * hillFrequency;

        float height = Mathf.PerlinNoise(nx, nz) + 0.5f * Mathf.PerlinNoise(nx * 2.3f, nz * 2.3f);

        return height / 1.5f * hillAmplitude;
    }

    private void BuildTerrain()
    {
        if (terrain == null)
        {
            return;
        }

        // Flat ground never changes so it only has to be written once.
        if (terrainBuilt && !generateHills)
        {
            return;
        }

        if (generateHills)
        {
            noiseOffset = new Vector2(Random.Range(0.0f, 1000.0f), Random.Range(0.0f, 1000.0f));
        }

        TerrainData data = terrain.terrainData;

        int res = data.heightmapResolution;

        float[,] heights = new float[res, res];

        for (int z = 0; z < res; z++)
        {
            for (int x = 0; x < res; x++)
            {
                float worldX = (float)x / (res - 1) * data.size.x;

                float worldZ = (float)z / (res - 1) * data.size.z;

                // SetHeights is indexed: z, x and expects 0..1 of size.y.
                heights[z, x] = GroundHeight(worldX, worldZ) / data.size.y;
            }
        }

        data.SetHeights(0, 0, heights);

        terrainBuilt = true;
    }

    private float SampleGround(float worldX, float worldZ)
    {
        if (terrain == null)
        {
            return 0.0f;
        }

        return terrain.SampleHeight(new Vector3(worldX, 0.0f, worldZ)) + terrain.transform.position.y;
    }

    private Vector3 RandomGroundPoint()
    {
        Vector3 origin = terrain != null ? terrain.transform.position : Vector3.zero;

        Vector3 size = terrain != null ? terrain.terrainData.size : new Vector3(500.0f, 0.0f, 500.0f);

        float x = Random.Range(origin.x + margin, origin.x + size.x - margin);

        float z = Random.Range(origin.z + margin, origin.z + size.z - margin);

        return new Vector3(x, SampleGround(x, z), z);
    }

    private bool IsClear(Vector3 point, float radius)
    {
        Vector2 candidate = new Vector2(point.x, point.z);

        foreach (Reserved item in reserved)
        {
            if (Vector2.Distance(candidate, item.center) < item.radius + radius)
            {
                return false;
            }
        }

        return true;
    }

    private void Reserve(Vector3 point, float radius)
    {
        reserved.Add(new Reserved
        {
            center = new Vector2(point.x, point.z),
            radius = radius,
        });
    }

    private void PlaceMeetingPoint()
    {
        meetingPointGround = RandomGroundPoint();

        meetingPoint.position = meetingPointGround + Vector3.up * meetingPointYOffset;

        Reserve(meetingPointGround, meetingPointRadius);
    }

    private void PlaceRover()
    {
        Vector3 point = meetingPointGround;

        for (int attempt = 0; attempt < placementAttempts; attempt++)
        {
            Vector2 direction = Random.insideUnitCircle.normalized;

            float range = Random.Range(minSeparation, maxSeparation);

            float x = meetingPointGround.x + direction.x * range;

            float z = meetingPointGround.z + direction.y * range;

            Vector3 candidate = new Vector3(x, SampleGround(x, z), z);

            if (!InsideMap(candidate))
            {
                continue;
            }

            point = candidate;

            break;
        }

        roverGround = point;

        RoverSpawnPosition = point + Vector3.up * roverSpawnHeight;

        RoverSpawnRotation = Quaternion.Euler(0.0f, Random.Range(0.0f, 360.0f), 0.0f);

        Reserve(point, roverRadius);
    }

    private bool InsideMap(Vector3 point)
    {
        Vector3 origin = terrain != null ? terrain.transform.position : Vector3.zero;

        Vector3 size = terrain != null ? terrain.terrainData.size : new Vector3(500.0f, 0.0f, 500.0f);

        return point.x > origin.x + margin && point.x < origin.x + size.x - margin
            && point.z > origin.z + margin && point.z < origin.z + size.z - margin;
    }

    private Vector3 RandomCorridorPoint()
    {
        Vector3 along = meetingPointGround - roverGround;

        along.y = 0.0f;

        float length = along.magnitude;

        if (length < 1.0f)
        {
            return RandomGroundPoint();
        }

        Vector3 forward = along / length;

        Vector3 side = new Vector3(-forward.z, 0.0f, forward.x);

        float padding = corridorPadding / length;

        float t = Random.Range(-padding, 1.0f + padding);

        float offset = Random.Range(-corridorHalfWidth, corridorHalfWidth);

        Vector3 point = roverGround + forward * (t * length) + side * offset;

        point.y = SampleGround(point.x, point.z);

        return InsideMap(point) ? point : RandomGroundPoint();
    }

    private void PlaceObstacles()
    {
        if (obstacleRoot == null)
        {
            return;
        }

        for (int i = obstacleRoot.childCount - 1; i >= 0; i--)
        {
            GameObject old = obstacleRoot.GetChild(i).gameObject;
            old.SetActive(false);

            Destroy(old);
        }

        if (obstaclePrefabs == null || obstaclePrefabs.Length == 0)
        {
            return;
        }

        int count = Random.Range(minObstacles, maxObstacles + 1);

        for (int i = 0; i < count; i++)
        {
            Vector3 point = Vector3.zero;

            bool found = false;

            for (int attempt = 0; attempt < placementAttempts; attempt++)
            {
                bool tryCorridor = i < corridorObstacles && attempt < placementAttempts / 2;

                point = tryCorridor ? RandomCorridorPoint() : RandomGroundPoint();

                if (IsClear(point, obstacleSpacing))
                {
                    found = true;

                    break;
                }
            }

            if (!found)
            {
                continue;
            }

            GameObject prefab = obstaclePrefabs[Random.Range(0, obstaclePrefabs.Length)];

            GameObject instance = Instantiate(prefab, obstacleRoot);

            instance.transform.position = point;

            instance.transform.rotation = Quaternion.Euler(0.0f, Random.Range(0.0f, 360.0f), 0.0f);

            Reserve(point, obstacleSpacing);
        }
    }

    private void Update()
    {
        if (!restartAfterArrival || rover == null)
        {
            return;
        }

        if (!rover.IsParked)
        {
            parkedSeconds = 0.0f;

            return;
        }

        parkedSeconds += Time.deltaTime;

        if (parkedSeconds < holdSecondsAfterArrival)
        {
            return;
        }

        parkedSeconds = 0.0f;

        rover.EndEpisode();
    }

    private void PlaceDrone()
    {
        if (drone == null)
        {
            return;
        }

        Vector2 offset = Random.insideUnitCircle * droneMaxOffset;

        float x = meetingPointGround.x + offset.x;

        float z = meetingPointGround.z + offset.y;

        float height = SampleGround(x, z) + Random.Range(droneMinHeight, droneMaxHeight);

        Vector3 spawnPosition = new Vector3(x, height, z);
        Quaternion spawnRotation =
            Quaternion.Euler(0.0f, Random.Range(0.0f, 360.0f), 0.0f);

        DroneMove droneAgent = drone.GetComponent<DroneMove>();

        if (droneAgent != null)
        {
            droneAgent.ResetToSpawn(spawnPosition, spawnRotation);
        }
        else
        {
            drone.SetPositionAndRotation(spawnPosition, spawnRotation);

            Rigidbody body = drone.GetComponent<Rigidbody>();

            if (body != null)
            {
                body.linearVelocity = Vector3.zero;
                body.angularVelocity = Vector3.zero;
            }
        }
    }
}