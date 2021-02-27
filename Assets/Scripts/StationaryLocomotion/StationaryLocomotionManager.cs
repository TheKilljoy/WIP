using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Htw.Cave.Locomotion
{
    public class StationaryLocomotionManager : MonoBehaviour
    {
        public LocomotionAlgorithm usedLocomotionAlgorithm = LocomotionAlgorithm.WalkInPlace;
        public bool useRigidbody = true;
        public Transform t_target;
        public Rigidbody rb_target;
        public Kinect.KinectTrackingArea trackingArea;
        public float walkingSpeed = 2.5f;

        public UnityEngine.UI.Text angleLeft;
        public UnityEngine.UI.Text angleRight;
        public UnityEngine.UI.Text lastFoot;
        public UnityEngine.UI.Text walkStatus;
        public UnityEngine.UI.Text stepCount;
        public UnityEngine.UI.Text hipHeight;
        public UnityEngine.UI.Text velocity;
        public UnityEngine.UI.Text stepDone;

        private StationaryLocomotionAlgorithm locomotionAlgorithm;

        // Start is called before the first frame update
        void Start()
        {
            switch(usedLocomotionAlgorithm)
            {
                case LocomotionAlgorithm.HumanJoystick:
                {
                    locomotionAlgorithm = new HumanJoystick();
                }break;
                case LocomotionAlgorithm.WalkInPlace:
                {
                    locomotionAlgorithm = new WalkInPlace();
                }break;
            }

            //ToDo(robin) put into a custom inspector script
            locomotionAlgorithm.WalkingSpeed = walkingSpeed;
        }

        // Update is called once per frame
        void Update()
        {
            if(!useRigidbody)
            {
                if(t_target)
                    locomotionAlgorithm.UpdateTarget(trackingArea.actor, t_target);
                else
                    Debug.LogError("No target for stationary locomotion specified");
            }

            WalkInPlace wip = locomotionAlgorithm as WalkInPlace;
            if(wip == null) return;
            angleLeft.text = "Angle Left: " + wip.angleLeft.ToString("F4");
            angleRight.text = "Angle Right: " + wip.angleRight.ToString("F4");
            lastFoot.text = "Last Foot: " + wip.m_lastFoot;
            walkStatus.text = "Walk Status: " + wip.m_status;
            stepCount.text = "Step Count: " + wip.stepCount;
            hipHeight.text = "Hip height: " + wip.m_standingHipHeight + " | " + wip.m_currentStandingHipHeight;
            velocity.text = "Velocity: " + wip.m_currentSpeed.ToString("F4");
            stepDone.text = "Step Done: " + wip.stepDone.ToString("F4");
        }

        private void FixedUpdate()
        {
            if(useRigidbody)
            {
                if(rb_target)
                    locomotionAlgorithm.UpdateTarget(trackingArea.actor, rb_target);
                else
                    Debug.LogError("No target for stationary locomotion specified");
            }                
        }
    }
}