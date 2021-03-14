using System.Collections;
using System.Collections.Generic;
using Htw.Cave.Kinect;
using UnityEngine;

namespace Htw.Cave.Locomotion
{
    public enum WIPPhaseType
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

        private StationaryLocomotion locomotion;

        private WIPStateMachine stateMachine;

        public void Awake()
        {
            locomotion = GetComponent<StationaryLocomotion>();
            stateMachine = new WIPStateMachine(locomotion, this);
        }

        public void Update()
        {
            stateMachine.Update();
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
        public WIPPhase this[WIPPhaseType phase] => this.m_States[(int)phase];

        private WIPPhase[] m_States;

        private WIPPhase m_Current;

        public WIPPhase currentState => m_Current;

        private WIPState m_WIPState;

        public WIPStateMachine(StationaryLocomotion locomotion, WalkInPlace wip)
        {
            m_WIPState = new WIPState(this, locomotion, wip);
            m_States = new WIPPhase[]
            {
                new Stationary(m_WIPState),
                new BeginUpMove(m_WIPState),
                new TurnDirection(m_WIPState),
                new BeginDownMove(m_WIPState),
                new EndStep(m_WIPState),
                new SmoothEndStep(m_WIPState),
            };

            m_Current = m_States[0];
        }

        public void Update()
        {
            WIPPhase newState = m_Current.Update();
            if(newState != m_Current)
            {
                m_Current = newState;
                m_WIPState.lastStatusChange = Time.time;
            }
        }
    }

    public class WIPState
    {
        public float lastStatusChange;
        public readonly float StatusChangeResetDuration = 2.0f;

        public Vector3 kneeLeftPos;
        public Vector3 kneeRightPos;
        public Vector3 kneeLeftPrevPos;
        public Vector3 kneeRightPrevPos;
        public Vector3 hipLeftPos;
        public Vector3 hipRightPos;
        public Vector3 neckPos;

        public float currentAngle;
        public float lastAngle;
        public float deltaAngle;
        public float currentMovespeed;
        public float lastMovespeed;

        public WIPFoot lastFoot = WIPFoot.None;
               
        public StationaryLocomotion locomotion;
               
        public WalkInPlace walkInPlace;
               
        public WIPStateMachine stateMachine;
               
        public KinectActor currentActor;

        public WIPState(WIPStateMachine stateMachine, StationaryLocomotion locomotion, WalkInPlace wip)
        {
            this.stateMachine = stateMachine;
            this.locomotion = locomotion;
            this.walkInPlace = wip;
        }
    }

    public abstract class WIPPhase
    {
        protected WIPState m_State;
        protected void UpdateInternalState()
        {
            TryUpdateNewActor();

            if(m_State.currentActor == null) return;

            UpdateNodePositions();

            UpdateTargetRotation();
        }

        protected void TryUpdateNewActor()
        {
            KinectActor actor = m_State.locomotion.Actor;

            if(actor == null) return;

            if(m_State.currentActor != null && m_State.currentActor.trackingId == actor.trackingId)
                return;

            m_State.currentActor = actor;
        }
       
        protected void UpdateNodePositions()
        {
            m_State.kneeLeftPrevPos = m_State.kneeLeftPos;
            m_State.kneeRightPrevPos = m_State.kneeRightPos;
            m_State.hipLeftPos = m_State.currentActor.bodyFrame[Windows.Kinect.JointType.HipLeft].position;
            m_State.hipRightPos = m_State.currentActor.bodyFrame[Windows.Kinect.JointType.HipRight].position;
            m_State.kneeLeftPos = m_State.currentActor.bodyFrame[Windows.Kinect.JointType.KneeLeft].position;
            m_State.kneeRightPos = m_State.currentActor.bodyFrame[Windows.Kinect.JointType.KneeRight].position;
            m_State.neckPos = m_State.currentActor.bodyFrame[Windows.Kinect.JointType.Neck].position;
        }

        protected void UpdateTargetRotation()
        {
            if(!m_State.walkInPlace.useLeaningForRotation) return;

            Vector2 lean = m_State.currentActor.lean;

            if(Mathf.Abs(lean.x) <= m_State.walkInPlace.deadzone) return;

            Rigidbody rb = m_State.locomotion.rigidbdy;

            //float rotationValue = Mathf.Lerp(m_State.walkInPlace.deadzone, m_State.walkInPlace.rotationSpeedLeaning, Mathf.Abs(lean.x)) * Mathf.Sign(lean.x);
            float rotationValue = MapRangeToRange(m_State.walkInPlace.deadzone, 1.0f,
                                                  0.0f, m_State.walkInPlace.rotationSpeedLeaning,
                                                  Mathf.Abs(lean.x)) * Mathf.Sign(lean.x);

            if(rb != null)
            {
                rb.rotation *= Quaternion.Euler(0,rotationValue, 0);
                return;
            }

            m_State.locomotion.target.rotation *= 
                Quaternion.Euler(0, rotationValue * Time.deltaTime, 0);
        }

        protected float MapRangeToRange(float min, float max, float targetMin, float targetMax, float value)
        {
            return (value <= min) ? targetMin :
                   (value >= max) ? targetMax :
                   (value - min) * (targetMax - targetMin) / (max - min) + targetMin;
        }

        public WIPPhase(WIPState state)
        {
            m_State = state;
            TryUpdateNewActor();
        }

        public abstract WIPPhase Update();

        protected void MoveTarget()
        {
            Rigidbody rb = m_State.locomotion.rigidbdy;

            Vector3 movementDir = (!m_State.walkInPlace.useLeaningForRotation) ? 
                                   m_State.currentActor.bodyFrame[Windows.Kinect.JointType.Neck].rotation * -Vector3.forward :
                                  (rb != null) ? rb.rotation * Vector3.forward : m_State.locomotion.transform.forward;
            movementDir.y = 0.0f;
            movementDir.Normalize();

            if(rb != null)
            {
                rb.velocity = m_State.currentMovespeed * movementDir;
            }
            else
            {
                m_State.locomotion.target.position += movementDir * m_State.currentMovespeed * Time.deltaTime;
            }
        }
    }

    public class Stationary : WIPPhase
    {
        private float m_AngleLeft;
        private float m_AngleRight;

        public Stationary(WIPState state) 
            : base(state)
        {
        }

        public override WIPPhase Update()
        {
            if(Time.time - m_State.lastStatusChange >= m_State.StatusChangeResetDuration)
            {
                m_State.lastFoot = WIPFoot.None;
                m_State.lastStatusChange = Time.time;
            }

            UpdateInternalState();

            if(m_State.currentActor == null) return m_State.stateMachine[WIPPhaseType.Stationary];

            Vector3 leftKneeHipDir = m_State.hipLeftPos - m_State.kneeLeftPos;
            Vector3 rightKneeHipDir = m_State.hipRightPos - m_State.kneeRightPos;
            Vector3 centerHip = m_State.hipLeftPos + (m_State.hipRightPos - m_State.hipLeftPos) * 0.5f;
            Vector3 upDir = (m_State.neckPos - centerHip).normalized;

            m_AngleLeft = Vector3.Angle(leftKneeHipDir, upDir);
            m_AngleRight = Vector3.Angle(rightKneeHipDir, upDir);

            if(m_AngleLeft >= m_State.walkInPlace.beginStepAngle && m_AngleRight >= m_State.walkInPlace.beginStepAngle)
                return m_State.stateMachine[WIPPhaseType.Stationary];

            if((m_State.lastFoot == WIPFoot.None || m_State.lastFoot == WIPFoot.Right) &&
                m_AngleLeft >= m_State.walkInPlace.beginStepAngle)
            {
                m_State.lastFoot = WIPFoot.Left;
                return m_State.stateMachine[WIPPhaseType.BeginUpMove];
            }

            if((m_State.lastFoot == WIPFoot.None || m_State.lastFoot == WIPFoot.Left) &&
                m_AngleRight >= m_State.walkInPlace.beginStepAngle)
            {
                m_State.lastFoot = WIPFoot.Right;
                return m_State.stateMachine[WIPPhaseType.BeginUpMove];
            }

            return m_State.stateMachine[WIPPhaseType.Stationary];
        }
    }

    public class BeginUpMove : WIPPhase
    {
        private bool m_isInitialized = false;
        private float m_LastCheck;

        private const float m_CheckAngleChangedInverval = 0.0333f;
        private const float m_MinAngleChangePerInterval = 0.5f;

        public BeginUpMove(WIPState state) 
            : base(state)
        {
        }

        public override WIPPhase Update()
        {
            if(Time.time - m_State.lastStatusChange >= m_State.StatusChangeResetDuration)
                return m_State.stateMachine[WIPPhaseType.EndStep];

            UpdateInternalState();

            if(m_State.currentActor == null) return m_State.stateMachine[WIPPhaseType.Stationary];

            Vector3 centerHip = m_State.hipLeftPos + (m_State.hipRightPos - m_State.hipLeftPos) * 0.5f;
            Vector3 upDir = (m_State.neckPos - centerHip).normalized;

            if(m_State.lastFoot == WIPFoot.Left)
            {
                Vector3 leftKneeHipDir = m_State.hipLeftPos - m_State.kneeLeftPos;
                m_State.currentAngle = Vector3.Angle(leftKneeHipDir, upDir);
            }
            else
            {
                Vector3 rightKneeHipDir = m_State.hipRightPos - m_State.kneeRightPos;
                m_State.currentAngle = Vector3.Angle(rightKneeHipDir, upDir);
            }

            if (!m_isInitialized)
            {
                m_isInitialized = true;
                m_LastCheck = Time.time;
                m_State.lastAngle = m_State.currentAngle;
            }

            if(Time.time - m_LastCheck >= m_CheckAngleChangedInverval)
            {
                m_LastCheck = Time.time;

                m_State.deltaAngle = m_State.currentAngle - m_State.lastAngle;
                m_State.lastAngle = m_State.currentAngle;

                if(m_State.deltaAngle < m_MinAngleChangePerInterval)
                {
                    m_isInitialized = false;
                    return m_State.stateMachine[WIPPhaseType.BeginTurnDirection];
                }
            }

            float speedMultiplier = MapRangeToRange(0.0f, m_State.walkInPlace.maxVelocityDeltaAngle, 
                                                    0.75f, 1.0f,
                                                    m_State.deltaAngle);

            m_State.currentMovespeed = Mathf.Max(speedMultiplier * speedMultiplier * 
                                            m_State.locomotion.maxWalkingSpeed, 
                                            m_State.locomotion.minWalkingSpeed) ;

            MoveTarget();

            return m_State.stateMachine[WIPPhaseType.BeginUpMove];
        }
    }

    public class TurnDirection : WIPPhase
    {
        private const float m_TurnDirectionDuration = 0.1f;

        public TurnDirection(WIPState state) 
            : base(state)
        {
        }

        public override WIPPhase Update()
        {
            UpdateInternalState();

            if(m_State.currentActor == null) return m_State.stateMachine[WIPPhaseType.Stationary];

            if(Time.time - m_State.lastStatusChange > m_TurnDirectionDuration)
                return m_State.stateMachine[WIPPhaseType.BeginDownMove];

            MoveTarget();

            return m_State.stateMachine[WIPPhaseType.BeginTurnDirection];
        }
    }

    public class BeginDownMove : WIPPhase
    {
        private bool m_isInitialized = false;
        private float m_LastCheck;
        private const float m_CheckAngleChangedInverval = 0.0333f;
        private const float m_MinAngleChange = 0.1f;

        public BeginDownMove(WIPState state) 
            : base(state)
        {
        }

        public override WIPPhase Update()
        {
            if(Time.time - m_State.lastStatusChange >= m_State.StatusChangeResetDuration)
                return m_State.stateMachine[WIPPhaseType.EndStep];

            UpdateInternalState();

            if(m_State.currentActor == null) return m_State.stateMachine[WIPPhaseType.Stationary];

            Vector3 centerHip = m_State.hipLeftPos + (m_State.hipRightPos - m_State.hipLeftPos) * 0.5f;
            Vector3 upDir = (m_State.neckPos - centerHip).normalized;

            if(m_State.lastFoot == WIPFoot.Left)
            {
                Vector3 leftKneeHipDir = m_State.hipLeftPos - m_State.kneeLeftPos;
                m_State.currentAngle = Vector3.Angle(leftKneeHipDir, upDir);
            }
            else
            {
                Vector3 rightKneeHipDir = m_State.hipRightPos - m_State.kneeRightPos;
                m_State.currentAngle = Vector3.Angle(rightKneeHipDir, upDir);
            }

            if(!m_isInitialized)
            {
                m_isInitialized = true;
                m_LastCheck = Time.time;
            }

            if(Time.time - m_LastCheck >= m_CheckAngleChangedInverval)
            {
                m_LastCheck = Time.time;

                m_State.deltaAngle = m_State.currentAngle - m_State.lastAngle;
                m_State.lastAngle = m_State.currentAngle;

                if(m_State.deltaAngle > m_MinAngleChange)
                {
                    m_isInitialized = false;
                    return m_State.stateMachine[WIPPhaseType.EndStep];
                }
            }

            float speedMultiplier = MapRangeToRange(0.0f, m_State.walkInPlace.maxVelocityDeltaAngle, 
                                                    0.75f, 1.0f,
                                                    Mathf.Abs(m_State.deltaAngle));

            m_State.currentMovespeed = Mathf.Max(speedMultiplier * speedMultiplier * 
                                            m_State.locomotion.maxWalkingSpeed, 
                                            m_State.locomotion.minWalkingSpeed);

            MoveTarget();

            return m_State.stateMachine[WIPPhaseType.BeginDownMove];
        }
    }

    public class EndStep : WIPPhase
    {
        private float m_AngleLeft;
        private float m_AngleRight;

        public EndStep(WIPState state) 
            : base(state)
        {
        }

        public override WIPPhase Update()
        {
            UpdateInternalState();

            if(m_State.currentActor == null) return m_State.stateMachine[WIPPhaseType.Stationary];

            Vector3 leftKneeHipDir = m_State.hipLeftPos - m_State.kneeLeftPos;
            Vector3 rightKneeHipDir = m_State.hipRightPos - m_State.kneeRightPos;
            Vector3 centerHip = m_State.hipLeftPos + (m_State.hipRightPos - m_State.hipLeftPos) * 0.5f;
            Vector3 upDir = (m_State.neckPos - centerHip).normalized;

            m_AngleLeft = Vector3.Angle(leftKneeHipDir, upDir);
            m_AngleRight = Vector3.Angle(rightKneeHipDir, upDir);

            if(m_State.lastFoot == WIPFoot.Left && m_AngleLeft < m_State.walkInPlace.endStepAngle)
            {
                return m_State.stateMachine[WIPPhaseType.Stationary];
            }

            if(m_State.lastFoot == WIPFoot.Right && m_AngleRight < m_State.walkInPlace.endStepAngle)
            {
                return m_State.stateMachine[WIPPhaseType.Stationary];
            }

            return m_State.stateMachine[WIPPhaseType.EndStep];
        }
    }

    public class SmoothEndStep : WIPPhase
    {
        private float m_AngleLeft;
        private float m_AngleRight;
        private const float m_FadeOutDuration = 0.3f;

        public SmoothEndStep(WIPState state)
            : base(state)
        {
        }

        public override WIPPhase Update()
        {
            if(Time.time - m_State.lastStatusChange >= m_State.StatusChangeResetDuration)
            {
                m_State.lastFoot = WIPFoot.None;
                m_State.lastStatusChange = Time.time;
                return m_State.stateMachine[WIPPhaseType.Stationary];
            }

            UpdateInternalState();

            if(m_State.currentActor == null) return m_State.stateMachine[WIPPhaseType.Stationary];

            Vector3 leftKneeHipDir = m_State.hipLeftPos - m_State.kneeLeftPos;
            Vector3 rightKneeHipDir = m_State.hipRightPos - m_State.kneeRightPos;
            Vector3 centerHip = m_State.hipLeftPos + (m_State.hipRightPos - m_State.hipLeftPos) * 0.5f;
            Vector3 upDir = (m_State.neckPos - centerHip).normalized;

            m_AngleLeft = Vector3.Angle(leftKneeHipDir, upDir);
            m_AngleRight = Vector3.Angle(rightKneeHipDir, upDir);

            float deltaTime = m_State.lastStatusChange - Time.time + m_FadeOutDuration;
            
            float movementSpeedMultiplier = MapRangeToRange(0.0f, m_FadeOutDuration, 0.0f, 1.0f, deltaTime);
            
            // smooth out
            float movementSpeed = movementSpeedMultiplier * m_State.currentMovespeed;

            MoveTarget();

            if(m_AngleLeft >= m_State.walkInPlace.beginStepAngle && m_AngleRight >= m_State.walkInPlace.beginStepAngle)
                return m_State.stateMachine[WIPPhaseType.SmoothEndStep];

            if((m_State.lastFoot == WIPFoot.None || m_State.lastFoot == WIPFoot.Right) &&
                m_AngleLeft >= m_State.walkInPlace.beginStepAngle)
            {
                m_State.lastFoot = WIPFoot.Left;
                return m_State.stateMachine[WIPPhaseType.BeginUpMove];
            }

            if((m_State.lastFoot == WIPFoot.None || m_State.lastFoot == WIPFoot.Left) &&
                m_AngleRight >= m_State.walkInPlace.beginStepAngle)
            {
                m_State.lastFoot = WIPFoot.Right;
                return m_State.stateMachine[WIPPhaseType.BeginUpMove];
            }

            return m_State.stateMachine[WIPPhaseType.SmoothEndStep];
        }
    }
}