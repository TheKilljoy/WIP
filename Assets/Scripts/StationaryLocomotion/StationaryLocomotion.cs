using Htw.Cave.Kinect;
using UnityEngine;

namespace Htw.Cave.Locomotion
{
    public class StationaryLocomotion : MonoBehaviour
    {
        public float maxWalkingSpeed = 2.5f;

        public float minWalkingSpeed = 0.5f;

        public KinectTrackingArea trackingArea;

        [SerializeField]
        private Transform m_Target;

        public Transform target
        {
            get => this.m_Target;
            set
            {
                this.m_Target = value;
                this.m_Rigidbody = value.GetComponent<Rigidbody>();
            }
        }

        [SerializeField]
        private Rigidbody m_Rigidbody;

        public Rigidbody rigidbdy => this.m_Rigidbody;

        private KinectActor m_actor;

        public KinectActor Actor => m_actor;


        void Awake()
        {
            trackingArea.onActorChanged += UpdateCurrentActor;
        }

        private void OnDestroy()
        {
            trackingArea.onActorChanged -= UpdateCurrentActor;
        }

        private void UpdateCurrentActor(KinectActor actor)
        {
            m_actor = trackingArea.actor;
        }
    }
}


