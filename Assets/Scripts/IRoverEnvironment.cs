using UnityEngine;

public interface IRoverEnvironment
{
    void ResetEnvironment();

    Vector3 RoverSpawnPosition { get; }

    Quaternion RoverSpawnRotation { get; }
}