using System.Collections;
using System.Collections.Generic;
using Htw.Cave.Kinect;
using UnityEngine;
using UnityEditor;

namespace Htw.Cave.Locomotion
{
    [RequireComponent(typeof(StationaryLocomotion))]
    public class HumanJoystick : MonoBehaviour
    {
        private StationaryLocomotion m_Locomotion;

        [SerializeField]
        private float m_deadZone = 0.25f;

        public void Awake()
        {
            m_Locomotion = GetComponent<StationaryLocomotion>();
        }

        public void Update()
        {
            KinectActor trackedActor = m_Locomotion.Actor;

            if(trackedActor == null)
                return;

            Vector2 lean = trackedActor.lean;
            float sqrMag = lean.sqrMagnitude;

            if(lean.sqrMagnitude < m_deadZone * m_deadZone) return;

            Vector3 moveDir = new Vector3(lean.x, 0, lean.y);

            Rigidbody rb = m_Locomotion.rigidbdy;

            if(rb != null)
            {
                rb.velocity = moveDir * Mathf.Lerp(0.0f, m_Locomotion.maxWalkingSpeed, sqrMag);
                return;
            }

            m_Locomotion.target.position += moveDir * Mathf.Lerp(0.0f, m_Locomotion.maxWalkingSpeed, sqrMag) * Time.deltaTime;
        }
    }
}