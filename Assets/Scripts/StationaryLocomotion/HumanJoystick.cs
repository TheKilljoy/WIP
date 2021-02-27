using System.Collections;
using System.Collections.Generic;
using Htw.Cave.Kinect;
using UnityEngine;
using UnityEditor;

namespace Htw.Cave.Locomotion
{
    public class HumanJoystick : StationaryLocomotionAlgorithm
    {
        private float m_maxWalkingspeed = 2.5f;
        private float m_deadZone = 0.25f;

        public float WalkingSpeed { get => m_maxWalkingspeed; set => m_maxWalkingspeed = value; }

        public void UpdateTarget(KinectActor trackedActor, Transform targetToMove)
        {
            if(!trackedActor || !trackedActor.isTracked) return;

            Vector2 lean = trackedActor.lean;
            float sqrMag = lean.sqrMagnitude;

            if(lean.sqrMagnitude < m_deadZone * m_deadZone) return;

            Vector3 moveDir = new Vector3(lean.x, 0, lean.y);

            targetToMove.transform.position += moveDir * Mathf.Lerp(0.0f, m_maxWalkingspeed, sqrMag) * Time.deltaTime;
        }

        public void UpdateTarget(KinectActor trackedActor, Rigidbody targetToMove)
        {
            if(!trackedActor || !trackedActor.isTracked) return;

            Vector2 lean = trackedActor.lean;
            float sqrMag = lean.sqrMagnitude;

            if(lean.sqrMagnitude < m_deadZone * m_deadZone) return;

            Vector3 moveDir = new Vector3(lean.x, 0, lean.y);

            targetToMove.velocity = moveDir * Mathf.Lerp(0.0f, m_maxWalkingspeed, sqrMag);
        }
    }
}