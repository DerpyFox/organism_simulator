using UnityEngine;
using UnityEngine.Events;

public class WallController : MonoBehaviour
{
    [System.Serializable]
    public class CollisionEvent : UnityEvent<Collision> { }

    [Header("Collision Callbacks")]
    public CollisionEvent onCollisionEnterEvent = new CollisionEvent();

    void OnCollisionEnter(Collision collision)
    {
        onCollisionEnterEvent.Invoke(collision);
    }
}
