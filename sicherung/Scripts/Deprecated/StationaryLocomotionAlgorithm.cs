using UnityEngine;

namespace Htw.Cave.Locomotion
{
    public interface StationaryLocomotionAlgorithm<T>
    {
        void SetTransformToMove(Transform transform);
        void SetRigidbodyToMove(Rigidbody rigidbody);
        void UpdateLocomotion(T data);
    }
}
