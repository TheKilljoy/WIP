using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Htw.Cave.Locomotion
{
    public class HumanJoystick : StationaryLocomotionAlgorithm<Vector2>
    {
        public float moveSpeed = 20.0f;
        public float deadzone = 0.1f;

        private Transform targetTransform;
        private Rigidbody targetRigidbody;

        public void SetRigidbodyToMove(Rigidbody rigidbody)
        {
            this.targetRigidbody = rigidbody;
        }

        public void SetTransformToMove(Transform transform)
        {
            this.targetTransform = transform;
        }

        public void UpdateLocomotion(Vector2 data)
        {
            if(data.magnitude <= deadzone)
                return;

            if(targetRigidbody)
            {
                targetRigidbody.AddForce(new Vector3(data.x * moveSpeed * Time.deltaTime, 0.0f, data.y * moveSpeed * Time.deltaTime), ForceMode.Acceleration);
                return;
            }

            if(targetTransform)
            {
                targetTransform.position += new Vector3(data.x * moveSpeed * Time.deltaTime, 0.0f, data.y * moveSpeed * Time.deltaTime);
                return;
            }
        }
    }
}

