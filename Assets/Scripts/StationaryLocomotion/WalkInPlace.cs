using System.Collections;
using System.Collections.Generic;
using Htw.Cave.Kinect;
using UnityEngine;

namespace Htw.Cave.Locomotion
{
    public enum WIPPhase
    {
        Stationary           = 0,
        BeginUpMove          = 1,
        BeginTurnDirection   = 2,
        BeginDownMove        = 3,
        EndStep              = 4,
        SmoothEndStep        = 5,
    }

    public enum WIPFoot
    {
        None,
        Left,
        Right,
    }

    [RequireComponent(typeof(StationaryLocomotion))]
    public class WalkInPlace : MonoBehaviour
    {
        [SerializeField]
        [Tooltip("Angle Threshold for beginning a step")]
        [Range(20.0f, 90.0f)]
        private float m_BeginStepAngle = 25.0f;

        [SerializeField]
        [Tooltip("Angle Threshold for ending a step")]
        [Range(5.0f, 20.0f)]
        private float m_EndStepAngle = 15.0f;

        [SerializeField]
        [Tooltip("Maximum walking speed is reached at this angle/(1/30s)")]
        [Range(1.0f, 45.0f)]
        private float m_MaxVelocityDeltaAngle = 10.0f;

        private StationaryLocomotion m_Locomotion;

        private WIPStateMachine m_StateMachine;

        public void Awake()
        {
            m_Locomotion = GetComponent<StationaryLocomotion>();
            m_StateMachine = new WIPStateMachine(m_Locomotion, m_BeginStepAngle, 
                                                 m_EndStepAngle, m_MaxVelocityDeltaAngle);
        }

        public void Update()
        {
            m_StateMachine.Update();
        }

        protected static float MapRangeToRange(float min, float max, float targetMin, float targetMax, float value)
        {
            return (value <= min) ? targetMin :
                   (value >= max) ? targetMax :
                   (value - min) * (targetMax - targetMin) / (max - min) + targetMin;
        }
    }

    public class WIPStateMachine
    {
        public WIPState this[WIPPhase phase] => this.m_States[(int)phase];

        private WIPState[] m_States;

        private WIPState m_Current;

        public WIPState currentState => m_Current;

        public WIPStateMachine(StationaryLocomotion locomotion, float beginStepAngle, 
                               float endStepAngle, float maxVelocityAngle)
        {
            m_States = new WIPState[]
            {
                new Stationary(this, locomotion, beginStepAngle),
                new BeginUpMove(this, locomotion, beginStepAngle, maxVelocityAngle),
                new TurnDirection(this, locomotion),
                new BeginDownMove(this, locomotion, beginStepAngle, endStepAngle, maxVelocityAngle),
                new EndStep(this, locomotion, endStepAngle),
                new SmoothEndStep(this, locomotion,beginStepAngle, endStepAngle),
            };

            m_Current = m_States[0];
        }

        public void Update()
        {
            WIPState newState = m_Current.Update();
            if(newState != m_Current)
            {
                m_Current = newState;
                WIPState.lastStatusChange = Time.time;
            }
        }
    }

    public abstract class WIPState
    {
        public static float lastStatusChange;
        protected readonly static float m_StatusChangeResetDuration = 2.0f;

        protected static KinectDynamicJoint m_kneeLeft;
        protected static KinectDynamicJoint m_kneeRight;
        protected static KinectDynamicJoint m_hipLeft;
        protected static KinectDynamicJoint m_hipRight;
        protected static KinectDynamicJoint m_neck;

        protected static Vector3 m_KneeLeftPos;
        protected static Vector3 m_KneeRightPos;
        protected static Vector3 m_KneeLeftPrevPos;
        protected static Vector3 m_KneeRightPrevPos;
        protected static Vector3 m_HipLeftPos;
        protected static Vector3 m_HipRightPos;
        protected static Vector3 m_NeckPos;

        protected static float m_CurrentAngle;
        protected static float m_LastAngle;
        protected static float m_DeltaAngle;
        protected static float m_CurrentMovespeed;
        protected static float m_LastMovespeed;

        protected static WIPFoot m_LastFoot = WIPFoot.None;

        protected static StationaryLocomotion m_Locomotion;

        protected static WIPStateMachine m_StateMachine;

        protected static KinectActor m_currentActor;

        public WIPPhase myPhase;

        protected static void UpdateInternalState()
        {
            TryUpdateNewActor();

            if(m_currentActor == null) return;

            UpdateNodePositions();
        }

        protected static void TryUpdateNewActor()
        {
            KinectActor actor = m_Locomotion.Actor;

            if(actor == null) return;

            if(m_currentActor != null && m_currentActor.trackingId == actor.trackingId)
                return;

            if(m_currentActor != null)
            {
                m_currentActor.onTrack -= OnJointTrack;
                m_currentActor.onUntrack -= OnJointUntrack;
            }

            m_currentActor = actor;

            m_currentActor.onTrack += OnJointTrack;
            m_currentActor.onUntrack += OnJointUntrack;

            InitializeJoints();
        }

        protected static void InitializeJoints()
        {
            if(m_currentActor == null) return;

            foreach(KinectTrackable trackable in m_currentActor.trackables)
            {
                KinectDynamicJoint joint = trackable as KinectDynamicJoint;
                if(joint == null) continue;

                switch(joint.jointType)
                {
                    case Windows.Kinect.JointType.KneeLeft:
                        m_kneeLeft = joint;
                        break;
                    case Windows.Kinect.JointType.KneeRight:
                        m_kneeRight = joint;
                        break;
                    case Windows.Kinect.JointType.HipLeft:
                        m_hipLeft = joint;
                        break;
                    case Windows.Kinect.JointType.HipRight:
                        m_hipRight = joint;
                        break;
                    case Windows.Kinect.JointType.Neck:
                        m_neck = joint;
                        break;
                }
            }
        }

        protected static void OnJointTrack(KinectTrackable trackable)
        {
            KinectDynamicJoint joint = trackable as KinectDynamicJoint;
            if(joint == null) return;

            switch(joint.jointType)
            {
                case Windows.Kinect.JointType.KneeLeft:
                m_kneeLeft = joint;
                break;
                case Windows.Kinect.JointType.KneeRight:
                m_kneeRight = joint;
                break;
                case Windows.Kinect.JointType.HipLeft:
                m_hipLeft = joint;
                break;
                case Windows.Kinect.JointType.HipRight:
                m_hipRight = joint;
                break;
                case Windows.Kinect.JointType.Neck:
                m_neck = joint;
                break;
            }
        }

        protected static void OnJointUntrack(KinectTrackable trackable)
        {
            KinectDynamicJoint joint = trackable as KinectDynamicJoint;
            if(joint == null) return;

            switch(joint.jointType)
            {
                case Windows.Kinect.JointType.KneeLeft:
                m_kneeLeft = null;
                break;
                case Windows.Kinect.JointType.KneeRight:
                m_kneeRight = null;
                break;
                case Windows.Kinect.JointType.HipLeft:
                m_hipLeft = null;
                break;
                case Windows.Kinect.JointType.HipRight:
                m_hipRight = null;
                break;
                case Windows.Kinect.JointType.Neck:
                m_neck = null;
                break;
            }
        }

        protected static void UpdateNodePositions()
        {
            m_KneeLeftPrevPos = m_KneeLeftPos;
            m_KneeRightPrevPos = m_KneeRightPos;
            m_HipLeftPos = m_hipLeft.transform.position;
            m_HipRightPos = m_hipRight.transform.position;
            m_KneeLeftPos = m_kneeLeft.transform.position;
            m_KneeRightPos = m_kneeRight.transform.position;
            m_NeckPos = m_neck.transform.position;
        }

        protected static float MapRangeToRange(float min, float max, float targetMin, float targetMax, float value)
        {
            return (value <= min) ? targetMin :
                   (value >= max) ? targetMax :
                   (value - min) * (targetMax - targetMin) / (max - min) + targetMin;
        }

        public WIPState(WIPStateMachine stateMachine, StationaryLocomotion locomotion)
        {
            m_StateMachine = stateMachine;
            m_Locomotion = locomotion;
            TryUpdateNewActor();
        }

        public abstract WIPState Update();

        protected void MoveTarget()
        {
            Vector3 movementDir = m_neck.transform.forward;
            movementDir.y = 0.0f;
            movementDir.Normalize();

            Rigidbody rb = m_Locomotion.rigidbdy;
            if(rb != null)
            {
                rb.velocity = m_CurrentMovespeed * -movementDir;
            }
            else
            {
                m_Locomotion.target.position += -movementDir * m_CurrentMovespeed * Time.deltaTime;
            }
        }
    }

    public class Stationary : WIPState
    {
        private float m_BeginStepAngle;
        private float m_AngleLeft;
        private float m_AngleRight;

        public Stationary(WIPStateMachine stateMachine, StationaryLocomotion locomotion, float beginStepAngle) 
            : base(stateMachine, locomotion)
        {
            myPhase = WIPPhase.Stationary;
            m_BeginStepAngle = beginStepAngle;
        }

        public override WIPState Update()
        {
            if(Time.time - lastStatusChange >= m_StatusChangeResetDuration)
            {
                m_LastFoot = WIPFoot.None;
                lastStatusChange = Time.time;
            }

            UpdateInternalState();

            if(m_kneeLeft == null || m_kneeRight == null)
                return m_StateMachine[WIPPhase.Stationary];

            Vector3 leftKneeHipDir = m_HipLeftPos - m_KneeLeftPos;
            Vector3 rightKneeHipDir = m_HipRightPos - m_KneeRightPos;
            Vector3 centerHip = m_HipLeftPos + (m_HipRightPos - m_HipLeftPos) * 0.5f;
            Vector3 upDir = (m_NeckPos - centerHip).normalized;

            m_AngleLeft = Vector3.Angle(leftKneeHipDir, upDir);
            m_AngleRight = Vector3.Angle(rightKneeHipDir, upDir);

            if(m_AngleLeft >= m_BeginStepAngle && m_AngleRight >= m_BeginStepAngle)
                return m_StateMachine[WIPPhase.Stationary];

            if((m_LastFoot == WIPFoot.None || m_LastFoot == WIPFoot.Right) &&
                m_AngleLeft >= m_BeginStepAngle)
            {
                m_LastFoot = WIPFoot.Left;
                return m_StateMachine[WIPPhase.BeginUpMove];
            }

            if((m_LastFoot == WIPFoot.None || m_LastFoot == WIPFoot.Left) &&
                m_AngleRight >= m_BeginStepAngle)
            {
                m_LastFoot = WIPFoot.Right;
                return m_StateMachine[WIPPhase.BeginUpMove];
            }

            return m_StateMachine[WIPPhase.Stationary];
        }
    }

    public class BeginUpMove : WIPState
    {
        private float m_BeginStepAngle;
        private float m_MaxVelocityAngle;

        private bool m_isInitialized = false;
        private float m_LastCheck;

        private const float m_CheckAngleChangedInverval = 0.0333f;
        private const float m_MinAngleChangePerInterval = 0.5f;

        public BeginUpMove(WIPStateMachine stateMachine, StationaryLocomotion locomotion, 
                           float beginStepAngle, float maxVelocityAngle) 
            : base(stateMachine, locomotion)
        {
            myPhase = WIPPhase.BeginUpMove;
            m_BeginStepAngle = beginStepAngle;
            m_MaxVelocityAngle = maxVelocityAngle;
        }

        public override WIPState Update()
        {
            if(Time.time - lastStatusChange >= m_StatusChangeResetDuration)
                return m_StateMachine[WIPPhase.EndStep];

            UpdateInternalState();
            Vector3 centerHip = m_HipLeftPos + (m_HipRightPos - m_HipLeftPos) * 0.5f;
            Vector3 upDir = (m_NeckPos - centerHip).normalized;

            if(m_LastFoot == WIPFoot.Left)
            {
                Vector3 leftKneeHipDir = m_HipLeftPos - m_KneeLeftPos;
                m_CurrentAngle = Vector3.Angle(leftKneeHipDir, upDir);
            }
            else
            {
                Vector3 rightKneeHipDir = m_HipRightPos - m_KneeRightPos;
                m_CurrentAngle = Vector3.Angle(rightKneeHipDir, upDir);
            }

            if (!m_isInitialized)
            {
                m_isInitialized = true;
                m_LastCheck = Time.time;
                m_LastAngle = m_CurrentAngle;
            }

            if(Time.time - m_LastCheck >= m_CheckAngleChangedInverval)
            {
                m_LastCheck = Time.time;

                m_DeltaAngle = m_CurrentAngle - m_LastAngle;
                m_LastAngle = m_CurrentAngle;

                if(m_DeltaAngle < m_MinAngleChangePerInterval)
                {
                    m_isInitialized = false;
                    return m_StateMachine[WIPPhase.BeginTurnDirection];
                }
            }

            float speedMultiplier = MapRangeToRange(0.0f, m_MaxVelocityAngle, 
                                                    0.75f, 1.0f,
                                                    m_DeltaAngle);

            m_CurrentMovespeed = Mathf.Max(speedMultiplier * speedMultiplier * 
                                            m_Locomotion.maxWalkingSpeed, 
                                            m_Locomotion.minWalkingSpeed) ;

            MoveTarget();

            return m_StateMachine[WIPPhase.BeginUpMove];
        }
    }

    public class TurnDirection : WIPState
    {
        private const float m_TurnDirectionDuration = 0.1f;

        public TurnDirection(WIPStateMachine stateMachine, StationaryLocomotion locomotion) 
            : base(stateMachine, locomotion)
        {
            myPhase = WIPPhase.BeginTurnDirection;
        }

        public override WIPState Update()
        {
            UpdateInternalState();

            if(Time.time - lastStatusChange > m_TurnDirectionDuration)
                return m_StateMachine[WIPPhase.BeginDownMove];

            MoveTarget();

            return m_StateMachine[WIPPhase.BeginTurnDirection];
        }
    }

    public class BeginDownMove : WIPState
    {
        private float m_BeginStepAngle;
        private float m_EndStepAngle;
        private float m_MaxVelocityAngle;

        private bool m_isInitialized = false;
        private float m_LastCheck;
        private const float m_CheckAngleChangedInverval = 0.0333f;
        private const float m_MinAngleChange = 0.1f;

        public BeginDownMove(WIPStateMachine stateMachine, StationaryLocomotion locomotion, 
                             float beginStepAngle, float endStepAngle, float maxVelocityAngle) 
            : base(stateMachine, locomotion)
        {
            myPhase = WIPPhase.BeginDownMove;
            m_BeginStepAngle = beginStepAngle;
            m_EndStepAngle = endStepAngle;
            m_MaxVelocityAngle = maxVelocityAngle;
        }

        public override WIPState Update()
        {
            if(Time.time - lastStatusChange >= m_StatusChangeResetDuration)
                return m_StateMachine[WIPPhase.EndStep];

            UpdateInternalState();

            Vector3 centerHip = m_HipLeftPos + (m_HipRightPos - m_HipLeftPos) * 0.5f;
            Vector3 upDir = (m_NeckPos - centerHip).normalized;

            if(m_LastFoot == WIPFoot.Left)
            {
                Vector3 leftKneeHipDir = m_HipLeftPos - m_KneeLeftPos;
                m_CurrentAngle = Vector3.Angle(leftKneeHipDir, upDir);
            }
            else
            {
                Vector3 rightKneeHipDir = m_HipRightPos - m_KneeRightPos;
                m_CurrentAngle = Vector3.Angle(rightKneeHipDir, upDir);
            }

            if(!m_isInitialized)
            {
                m_isInitialized = true;
                m_LastCheck = Time.time;
            }

            if(Time.time - m_LastCheck >= m_CheckAngleChangedInverval)
            {
                m_LastCheck = Time.time;

                m_DeltaAngle = m_CurrentAngle - m_LastAngle;
                m_LastAngle = m_CurrentAngle;

                if(m_DeltaAngle > m_MinAngleChange)
                {
                    m_isInitialized = false;
                    return m_StateMachine[WIPPhase.EndStep];
                }
            }

            float speedMultiplier = MapRangeToRange(0.0f, m_MaxVelocityAngle, 
                                                    0.75f, 1.0f,
                                                    Mathf.Abs(m_DeltaAngle));

            m_CurrentMovespeed = Mathf.Max(speedMultiplier * speedMultiplier * 
                                            m_Locomotion.maxWalkingSpeed, 
                                            m_Locomotion.minWalkingSpeed);

            MoveTarget();

            return m_StateMachine[WIPPhase.BeginDownMove];
        }
    }

    public class EndStep : WIPState
    {
        private float m_EndStepAngle;
        private float m_AngleLeft;
        private float m_AngleRight;

        public EndStep(WIPStateMachine stateMachine, StationaryLocomotion locomotion, float endStepAngle) 
            : base(stateMachine, locomotion)
        {
            myPhase = WIPPhase.EndStep;
            m_EndStepAngle = endStepAngle;
        }

        public override WIPState Update()
        {
            UpdateInternalState();

            if(m_kneeLeft == null || m_kneeRight == null)
                return m_StateMachine[WIPPhase.Stationary];

            Vector3 leftKneeHipDir = m_HipLeftPos - m_KneeLeftPos;
            Vector3 rightKneeHipDir = m_HipRightPos - m_KneeRightPos;
            Vector3 centerHip = m_HipLeftPos + (m_HipRightPos - m_HipLeftPos) * 0.5f;
            Vector3 upDir = (m_NeckPos - centerHip).normalized;

            m_AngleLeft = Vector3.Angle(leftKneeHipDir, upDir);
            m_AngleRight = Vector3.Angle(rightKneeHipDir, upDir);

            if(m_LastFoot == WIPFoot.Left && m_AngleLeft < m_EndStepAngle)
            {
                return m_StateMachine[WIPPhase.Stationary];
            }

            if(m_LastFoot == WIPFoot.Right && m_AngleRight < m_EndStepAngle)
            {
                return m_StateMachine[WIPPhase.Stationary];
            }

            return m_StateMachine[WIPPhase.EndStep];
        }
    }

    public class SmoothEndStep : WIPState
    {
        private float m_BeginStepAngle;
        private float m_EndStepAngle;
        private float m_AngleLeft;
        private float m_AngleRight;
        private const float m_FadeOutDuration = 0.3f;

        public SmoothEndStep(WIPStateMachine stateMachine, StationaryLocomotion locomotion, float beginStepAngle, float endStepAngle)
            : base(stateMachine, locomotion)
        {
            myPhase = WIPPhase.SmoothEndStep;
            m_EndStepAngle = endStepAngle;
            m_BeginStepAngle = beginStepAngle;
        }

        public override WIPState Update()
        {
            if(Time.time - lastStatusChange >= m_StatusChangeResetDuration)
            {
                m_LastFoot = WIPFoot.None;
                lastStatusChange = Time.time;
                return m_StateMachine[WIPPhase.Stationary];
            }

            UpdateInternalState();

            if(m_kneeLeft == null || m_kneeRight == null)
                return m_StateMachine[WIPPhase.Stationary];

            Vector3 leftKneeHipDir = m_HipLeftPos - m_KneeLeftPos;
            Vector3 rightKneeHipDir = m_HipRightPos - m_KneeRightPos;
            Vector3 centerHip = m_HipLeftPos + (m_HipRightPos - m_HipLeftPos) * 0.5f;
            Vector3 upDir = (m_NeckPos - centerHip).normalized;

            m_AngleLeft = Vector3.Angle(leftKneeHipDir, upDir);
            m_AngleRight = Vector3.Angle(rightKneeHipDir, upDir);

            float deltaTime = lastStatusChange - Time.time + m_FadeOutDuration;
            
            float movementSpeedMultiplier = MapRangeToRange(0.0f, m_FadeOutDuration, 0.0f, 1.0f, deltaTime);
            
            // smooth out
            float movementSpeed = movementSpeedMultiplier * m_CurrentMovespeed;

            MoveTarget();

            if(m_AngleLeft >= m_BeginStepAngle && m_AngleRight >= m_BeginStepAngle)
                return m_StateMachine[WIPPhase.SmoothEndStep];

            if((m_LastFoot == WIPFoot.None || m_LastFoot == WIPFoot.Right) &&
                m_AngleLeft >= m_BeginStepAngle)
            {
                m_LastFoot = WIPFoot.Left;
                return m_StateMachine[WIPPhase.BeginUpMove];
            }

            if((m_LastFoot == WIPFoot.None || m_LastFoot == WIPFoot.Left) &&
                m_AngleRight >= m_BeginStepAngle)
            {
                m_LastFoot = WIPFoot.Right;
                return m_StateMachine[WIPPhase.BeginUpMove];
            }

            return m_StateMachine[WIPPhase.SmoothEndStep];
        }
    }
}