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
        [Tooltip("Angle Threshold for beginning a step")]
        [Range(20.0f, 90.0f)]
        public float beginStepAngle = 25.0f;

        [Tooltip("Angle Threshold for ending a step")]
        [Range(5.0f, 20.0f)]
        public float endStepAngle = 15.0f;

        [Tooltip("Maximum walking speed is reached at this angle/(1/30s)")]
        [Range(1.0f, 45.0f)]
        public float maxVelocityDeltaAngle = 10.0f;

        [Tooltip("Rotate target using leaning instead of walking towards head-rotation")]
        public bool useLeaningForRotation = false;

        [Tooltip("Deadzone used for Leaning")]
        [Range(0.01f, 1.0f)]
        public float deadzone = 0.25f;

        [Range(0.01f, 1.0f)]
        public float rotationSpeedLeaning = 0.5f;

        private StationaryLocomotion m_Locomotion;

        private WIPStateMachine m_StateMachine;

        public void Awake()
        {
            m_Locomotion = GetComponent<StationaryLocomotion>();
            m_StateMachine = new WIPStateMachine(m_Locomotion, this);
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

        public WIPStateMachine(StationaryLocomotion locomotion, WalkInPlace wip)
        {
            m_States = new WIPState[]
            {
                new Stationary(this, locomotion, wip),
                new BeginUpMove(this, locomotion, wip),
                new TurnDirection(this, locomotion, wip),
                new BeginDownMove(this, locomotion, wip),
                new EndStep(this, locomotion, wip),
                new SmoothEndStep(this, locomotion, wip),
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

        protected static WalkInPlace m_WIP;

        protected static WIPStateMachine m_StateMachine;

        protected static KinectActor m_currentActor;

        public WIPPhase myPhase;

        protected static void UpdateInternalState()
        {
            TryUpdateNewActor();

            if(m_currentActor == null) return;

            UpdateNodePositions();

            UpdateTargetRotation();
        }

        protected static void TryUpdateNewActor()
        {
            KinectActor actor = m_Locomotion.Actor;

            if(actor == null) return;

            if(m_currentActor != null && m_currentActor.trackingId == actor.trackingId)
                return;

            m_currentActor = actor;
        }
       
        protected static void UpdateNodePositions()
        {
            m_KneeLeftPrevPos = m_KneeLeftPos;
            m_KneeRightPrevPos = m_KneeRightPos;
            m_HipLeftPos = m_currentActor.bodyFrame[Windows.Kinect.JointType.HipLeft].position;
            m_HipRightPos = m_currentActor.bodyFrame[Windows.Kinect.JointType.HipRight].position;
            m_KneeLeftPos = m_currentActor.bodyFrame[Windows.Kinect.JointType.KneeLeft].position;
            m_KneeRightPos = m_currentActor.bodyFrame[Windows.Kinect.JointType.KneeRight].position;
            m_NeckPos = m_currentActor.bodyFrame[Windows.Kinect.JointType.Neck].position;
        }

        protected static void UpdateTargetRotation()
        {
            if(!m_WIP.useLeaningForRotation) return;

            Vector2 lean = m_currentActor.lean;

            if(Mathf.Abs(lean.x) <= m_WIP.deadzone) return;

            Rigidbody rb = m_Locomotion.rigidbdy;

            float rotationValue = Mathf.Lerp(0, m_WIP.rotationSpeedLeaning, Mathf.Abs(lean.x)) * Mathf.Sign(lean.x);

            if(rb != null)
            {
                rb.rotation *= Quaternion.Euler(0,rotationValue, 0);
                return;
            }

            m_Locomotion.target.rotation *= 
                Quaternion.Euler(0, rotationValue * Time.deltaTime, 0);
        }

        protected static float MapRangeToRange(float min, float max, float targetMin, float targetMax, float value)
        {
            return (value <= min) ? targetMin :
                   (value >= max) ? targetMax :
                   (value - min) * (targetMax - targetMin) / (max - min) + targetMin;
        }

        public WIPState(WIPStateMachine stateMachine, StationaryLocomotion locomotion, WalkInPlace wip)
        {
            m_StateMachine = stateMachine;
            m_Locomotion = locomotion;
            m_WIP = wip;
            TryUpdateNewActor();
        }

        public abstract WIPState Update();

        protected void MoveTarget()
        {
            Rigidbody rb = m_Locomotion.rigidbdy;

            Vector3 movementDir = (!m_WIP.useLeaningForRotation) ? 
                                   m_currentActor.bodyFrame[Windows.Kinect.JointType.Neck].rotation * -Vector3.forward :
                                  (rb != null) ? rb.rotation * Vector3.forward : m_Locomotion.transform.forward;
            movementDir.y = 0.0f;
            movementDir.Normalize();

            if(rb != null)
            {
                rb.velocity = m_CurrentMovespeed * movementDir;
            }
            else
            {
                m_Locomotion.target.position += movementDir * m_CurrentMovespeed * Time.deltaTime;
            }
        }
    }

    public class Stationary : WIPState
    {
        private float m_AngleLeft;
        private float m_AngleRight;

        public Stationary(WIPStateMachine stateMachine, StationaryLocomotion locomotion, WalkInPlace wip) 
            : base(stateMachine, locomotion, wip)
        {
            myPhase = WIPPhase.Stationary;
        }

        public override WIPState Update()
        {
            if(Time.time - lastStatusChange >= m_StatusChangeResetDuration)
            {
                m_LastFoot = WIPFoot.None;
                lastStatusChange = Time.time;
            }

            UpdateInternalState();

            if(m_currentActor == null) return m_StateMachine[WIPPhase.Stationary];

            Vector3 leftKneeHipDir = m_HipLeftPos - m_KneeLeftPos;
            Vector3 rightKneeHipDir = m_HipRightPos - m_KneeRightPos;
            Vector3 centerHip = m_HipLeftPos + (m_HipRightPos - m_HipLeftPos) * 0.5f;
            Vector3 upDir = (m_NeckPos - centerHip).normalized;

            m_AngleLeft = Vector3.Angle(leftKneeHipDir, upDir);
            m_AngleRight = Vector3.Angle(rightKneeHipDir, upDir);

            if(m_AngleLeft >= m_WIP.beginStepAngle && m_AngleRight >= m_WIP.beginStepAngle)
                return m_StateMachine[WIPPhase.Stationary];

            if((m_LastFoot == WIPFoot.None || m_LastFoot == WIPFoot.Right) &&
                m_AngleLeft >= m_WIP.beginStepAngle)
            {
                m_LastFoot = WIPFoot.Left;
                return m_StateMachine[WIPPhase.BeginUpMove];
            }

            if((m_LastFoot == WIPFoot.None || m_LastFoot == WIPFoot.Left) &&
                m_AngleRight >= m_WIP.beginStepAngle)
            {
                m_LastFoot = WIPFoot.Right;
                return m_StateMachine[WIPPhase.BeginUpMove];
            }

            return m_StateMachine[WIPPhase.Stationary];
        }
    }

    public class BeginUpMove : WIPState
    {
        private bool m_isInitialized = false;
        private float m_LastCheck;

        private const float m_CheckAngleChangedInverval = 0.0333f;
        private const float m_MinAngleChangePerInterval = 0.5f;

        public BeginUpMove(WIPStateMachine stateMachine, StationaryLocomotion locomotion, WalkInPlace wip) 
            : base(stateMachine, locomotion, wip)
        {
            myPhase = WIPPhase.BeginUpMove;
        }

        public override WIPState Update()
        {
            if(Time.time - lastStatusChange >= m_StatusChangeResetDuration)
                return m_StateMachine[WIPPhase.EndStep];

            UpdateInternalState();

            if(m_currentActor == null) return m_StateMachine[WIPPhase.Stationary];

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

            float speedMultiplier = MapRangeToRange(0.0f, m_WIP.maxVelocityDeltaAngle, 
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

        public TurnDirection(WIPStateMachine stateMachine, StationaryLocomotion locomotion, WalkInPlace wip) 
            : base(stateMachine, locomotion, wip)
        {
            myPhase = WIPPhase.BeginTurnDirection;
        }

        public override WIPState Update()
        {
            UpdateInternalState();

            if(m_currentActor == null) return m_StateMachine[WIPPhase.Stationary];

            if(Time.time - lastStatusChange > m_TurnDirectionDuration)
                return m_StateMachine[WIPPhase.BeginDownMove];

            MoveTarget();

            return m_StateMachine[WIPPhase.BeginTurnDirection];
        }
    }

    public class BeginDownMove : WIPState
    {
        private bool m_isInitialized = false;
        private float m_LastCheck;
        private const float m_CheckAngleChangedInverval = 0.0333f;
        private const float m_MinAngleChange = 0.1f;

        public BeginDownMove(WIPStateMachine stateMachine, StationaryLocomotion locomotion, WalkInPlace wip) 
            : base(stateMachine, locomotion, wip)
        {
            myPhase = WIPPhase.BeginDownMove;
        }

        public override WIPState Update()
        {
            if(Time.time - lastStatusChange >= m_StatusChangeResetDuration)
                return m_StateMachine[WIPPhase.EndStep];

            UpdateInternalState();

            if(m_currentActor == null) return m_StateMachine[WIPPhase.Stationary];

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

            float speedMultiplier = MapRangeToRange(0.0f, m_WIP.maxVelocityDeltaAngle, 
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
        private float m_AngleLeft;
        private float m_AngleRight;

        public EndStep(WIPStateMachine stateMachine, StationaryLocomotion locomotion, WalkInPlace wip) 
            : base(stateMachine, locomotion, wip)
        {
            myPhase = WIPPhase.EndStep;
        }

        public override WIPState Update()
        {
            UpdateInternalState();

            if(m_currentActor == null) return m_StateMachine[WIPPhase.Stationary];

            Vector3 leftKneeHipDir = m_HipLeftPos - m_KneeLeftPos;
            Vector3 rightKneeHipDir = m_HipRightPos - m_KneeRightPos;
            Vector3 centerHip = m_HipLeftPos + (m_HipRightPos - m_HipLeftPos) * 0.5f;
            Vector3 upDir = (m_NeckPos - centerHip).normalized;

            m_AngleLeft = Vector3.Angle(leftKneeHipDir, upDir);
            m_AngleRight = Vector3.Angle(rightKneeHipDir, upDir);

            if(m_LastFoot == WIPFoot.Left && m_AngleLeft < m_WIP.endStepAngle)
            {
                return m_StateMachine[WIPPhase.Stationary];
            }

            if(m_LastFoot == WIPFoot.Right && m_AngleRight < m_WIP.endStepAngle)
            {
                return m_StateMachine[WIPPhase.Stationary];
            }

            return m_StateMachine[WIPPhase.EndStep];
        }
    }

    public class SmoothEndStep : WIPState
    {
        private float m_AngleLeft;
        private float m_AngleRight;
        private const float m_FadeOutDuration = 0.3f;

        public SmoothEndStep(WIPStateMachine stateMachine, StationaryLocomotion locomotion, WalkInPlace wip)
            : base(stateMachine, locomotion, wip)
        {
            myPhase = WIPPhase.SmoothEndStep;
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

            if(m_currentActor == null) return m_StateMachine[WIPPhase.Stationary];

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

            if(m_AngleLeft >= m_WIP.beginStepAngle && m_AngleRight >= m_WIP.beginStepAngle)
                return m_StateMachine[WIPPhase.SmoothEndStep];

            if((m_LastFoot == WIPFoot.None || m_LastFoot == WIPFoot.Right) &&
                m_AngleLeft >= m_WIP.beginStepAngle)
            {
                m_LastFoot = WIPFoot.Left;
                return m_StateMachine[WIPPhase.BeginUpMove];
            }

            if((m_LastFoot == WIPFoot.None || m_LastFoot == WIPFoot.Left) &&
                m_AngleRight >= m_WIP.beginStepAngle)
            {
                m_LastFoot = WIPFoot.Right;
                return m_StateMachine[WIPPhase.BeginUpMove];
            }

            return m_StateMachine[WIPPhase.SmoothEndStep];
        }
    }
}