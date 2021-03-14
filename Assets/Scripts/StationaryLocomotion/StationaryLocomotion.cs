using Htw.Cave.Kinect;
using UnityEngine;

namespace Htw.Cave.Locomotion
{
    public class StationaryLocomotion : MonoBehaviour
    {
        public float maxWalkingSpeed = 2.5f;

        public float minWalkingSpeed = 0.5f;

        public KinectActorTracker tracker;

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

        public KinectActor Actor => tracker.GetLongestTracked();
    }
}


