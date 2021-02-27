using System.Collections;
using System.Collections.Generic;
using Htw.Cave.Kinect;
using UnityEngine;

namespace Htw.Cave.Locomotion
{
    public class WalkInPlace : StationaryLocomotionAlgorithm
    {
        private float m_maxWalkingSpeed = 2.5f;
        public float WalkingSpeed { get => m_maxWalkingSpeed; set => m_maxWalkingSpeed = value; }

        public float m_currentSpeed = 0.0f;
        public Vector3 m_stepBeginPosition;
        public Vector3 m_stepEndPosition;
        public Vector3 m_forwardDirection;
        public WIPStatus m_status = WIPStatus.Stationary;
        public WIPFoot m_lastFoot = WIPFoot.None;
        private float m_lastStatusChangeTime = 0.0f;
        private const float m_ResetTimeDeltaInSeconds = 2.0f;

        private KinectActor m_currentActor;
        private KinectDynamicJoint m_ankleLeft;
        private KinectDynamicJoint m_ankleRight;
        private KinectDynamicJoint m_kneeLeft;
        private KinectDynamicJoint m_kneeRight;
        private KinectDynamicJoint m_hipLeft;
        private KinectDynamicJoint m_hipRight;
        private KinectDynamicJoint m_neck;

        Vector3 m_currentTargetPosition;
        Vector3 m_kneeLeftPos;
        Vector3 m_kneeRightPos;
        Vector3 m_hipLeftPos;
        Vector3 m_hipRightPos;
        Vector3 m_neckPos;

        private Vector3 m_kneeLeftPrevPos;
        private Vector3 m_kneeRightPrevPos;

        private const float m_maxStepAngle = 90.0f;
        private const float m_stepStartThreshold = 30.0f;
        private const float m_StepEndThreshold = 15.0f;
        public float m_standingHipHeight = 0.0f;
        public float m_currentStandingHipHeight = 0.0f;
        public float stepDone = 0.0f;

        //debug
        public float angleLeft;
        public float angleRight;
        public int stepCount = 0;
        public int bufferIndex = 0;
        public const int bufferSize = 60 * 4;
        public float _angle = 0.0f;
        public float[] angleBuffer = new float[bufferSize];
        public float[] moveSpeedTimeBuffer = new float[bufferSize];
        public Vector2 bufferPosition = new Vector2(0, Screen.height - 100);
        public Vector2 bufferDimension = new Vector2(Screen.width, 200.0f);


        public void UpdateTarget(KinectActor trackedActor, Transform targetToMove)
        {
            m_currentTargetPosition = targetToMove.position;
            DoWalkInPlace(trackedActor);

            if(m_neck)
                targetToMove.position += m_forwardDirection * m_currentSpeed * Time.deltaTime;
        }

        public void UpdateTarget(KinectActor trackedActor, Rigidbody targetToMove)
        {
            m_currentTargetPosition = targetToMove.position;
            DoWalkInPlace(trackedActor);

            if(m_neck)
                targetToMove.velocity = m_forwardDirection * m_currentSpeed;

            //debug
            moveSpeedTimeBuffer[bufferIndex] = m_currentSpeed;
            angleBuffer[bufferIndex] = _angle;
            bufferIndex = (bufferIndex + 1) % bufferSize;

            Camera camera = targetToMove.GetComponent<Camera>();
            Vector3 prevPos = new Vector3(0, 10.0f, 0.5f);
            Vector3 prevPosAngle = new Vector3(0, 10.0f, 0.49f);
            for(int i = 0; i < bufferSize; ++i)
            {
                float xPos = mapToRange(0.0f, bufferSize, 0.0f, bufferDimension.x, i);
                float yPos = mapToRange(0.0f, m_maxWalkingSpeed, 0.0f, bufferDimension.y, moveSpeedTimeBuffer[i]);
                float yPosAngle = mapToRange(0.0f, m_maxStepAngle, 0.0f, bufferDimension.y, angleBuffer[i]);
                Vector3 pos = new Vector3(xPos, yPos + 10, 0.5f);
                Vector3 posAngle = new Vector3(xPos, yPosAngle, 0.49f);
                Debug.DrawLine(camera.ScreenToWorldPoint(prevPos), camera.ScreenToWorldPoint(pos), Color.red);
                Debug.DrawLine(camera.ScreenToWorldPoint(prevPosAngle), camera.ScreenToWorldPoint(posAngle), Color.blue);
                prevPos = pos;
                prevPosAngle = posAngle;
            }
        }

        private void DoWalkInPlace(KinectActor trackedActor)
        {
            if(!trackedActor || !trackedActor.isTracked) return;

            if(!m_currentActor || m_currentActor.trackingId != trackedActor.trackingId)
            {
                InitNewKinectActor(trackedActor);
            }

            if(Time.time - m_lastStatusChangeTime >= m_ResetTimeDeltaInSeconds)
            {
                m_lastFoot = WIPFoot.None;
                m_status = WIPStatus.Stationary;
            }

            DetectStepStatus();
        }

        private void InitNewKinectActor(KinectActor actor)
        {
            m_currentActor = actor;

            foreach(KinectTrackable trackable in actor.trackables)
            {
                KinectDynamicJoint joint = trackable as KinectDynamicJoint;
                if(!joint) continue;

                switch(joint.jointType)
                {
                    case Windows.Kinect.JointType.AnkleLeft:
                    {
                        m_ankleLeft = joint;
                    }break;
                    case Windows.Kinect.JointType.AnkleRight:
                    {
                        m_ankleRight = joint;
                    }break;
                    case Windows.Kinect.JointType.KneeLeft:
                    {
                        m_kneeLeft = joint;
                    }break;
                    case Windows.Kinect.JointType.KneeRight:
                    {
                        m_kneeRight = joint;
                    }break;
                    case Windows.Kinect.JointType.HipLeft:
                    {
                        m_hipLeft = joint;
                    }break;
                    case Windows.Kinect.JointType.HipRight:
                    {
                        m_hipRight = joint;
                    }break;
                    case Windows.Kinect.JointType.Neck:
                    {
                        m_neck = joint;
                    }break;
                }
            }
        }

        private void DetectStepStatus()
        {
            m_kneeLeftPos = m_kneeLeft.transform.position;
            m_kneeRightPos = m_kneeRight.transform.position;
            m_hipLeftPos = m_hipLeft.transform.position;
            m_hipRightPos = m_hipRight.transform.position;
            m_neckPos = m_neck.transform.position;

            Vector3 forward = -m_neck.transform.forward;
            m_forwardDirection = new Vector3(forward.x, 0.0f, forward.z).normalized;

            if(m_status == WIPStatus.Stationary || 
               m_status == WIPStatus.EndStep)
            {
                DetectBeginningStep();
            }

            if(m_status == WIPStatus.BeginStep)
            {
                DetectEndStep();
            }
        }

        public void DetectBeginningStep()
        {
            m_currentSpeed = 0.0f;

            if(m_kneeLeft.trackingState < Windows.Kinect.TrackingState.Tracked ||
               m_kneeRight.trackingState < Windows.Kinect.TrackingState.Tracked)
                return;

            Vector3 leftKneeHipDir = m_hipLeftPos - m_kneeLeftPos;
            Vector3 rightKneeHipDir = m_hipRightPos - m_kneeRightPos;
            Vector3 centerHip = m_hipLeftPos + (m_hipRightPos - m_hipLeftPos) * 0.5f;
            Vector3 upDir = (m_neckPos - centerHip).normalized;

            angleLeft = Vector3.Angle(leftKneeHipDir, upDir);
            angleRight = Vector3.Angle(rightKneeHipDir, upDir);

            m_standingHipHeight = m_currentActor.height * 0.5f;
            m_currentStandingHipHeight = centerHip.y;

            //if the person is squatting it shouldn't count as walking, 
            //so if the hip is more towards the ground, don't check for steps
            float currentHipPositionInPercentage = m_currentStandingHipHeight / m_standingHipHeight;
            if(currentHipPositionInPercentage < 0.9f)
                return;

            if((m_lastFoot == WIPFoot.None ||
                m_lastFoot == WIPFoot.Right) &&
                angleLeft >= m_stepStartThreshold)
            {
                m_lastStatusChangeTime = Time.time;
                m_status = WIPStatus.BeginStep;
                m_lastFoot = WIPFoot.Left;
                ++stepCount;
                m_currentSpeed = m_maxWalkingSpeed;
                m_stepBeginPosition = m_currentTargetPosition;
                //m_currentVelocity = m_maxWalkingSpeed * m_neck.transform.forward;
            }

            if((m_lastFoot == WIPFoot.None ||
                m_lastFoot == WIPFoot.Left) &&
                angleRight >= m_stepStartThreshold)
            {
                m_lastStatusChangeTime = Time.time;
                m_status = WIPStatus.BeginStep;
                m_lastFoot = WIPFoot.Right;
                ++stepCount;
                m_currentSpeed = m_maxWalkingSpeed;
                m_stepBeginPosition = m_currentTargetPosition;
                //m_currentVelocity = m_maxWalkingSpeed * m_neck.transform.forward;
            }
        }

        public void DetectEndStep()
        {
            Vector3 centerHip = m_hipLeftPos + (m_hipRightPos - m_hipLeftPos) * 0.5f;
            Vector3 upDir = (m_neckPos - centerHip).normalized;
            float percentageDoneStep = 0.0f;
            float angle = 0.0f;

            if(m_lastFoot == WIPFoot.Left)
            {
                Vector3 kneeHipDirLeft = m_hipLeftPos - m_kneeLeftPos;
                angleLeft = Vector3.Angle(kneeHipDirLeft, upDir);
                percentageDoneStep = (angleLeft - m_stepStartThreshold) / (m_StepEndThreshold - m_stepStartThreshold);
                angle = angleLeft;    
            }
            else
            {
                Vector3 kneeHipDirRight = m_hipRightPos - m_kneeRightPos;
                angleRight = Vector3.Angle(kneeHipDirRight, upDir);
                percentageDoneStep = (angleRight - m_stepStartThreshold) / (m_StepEndThreshold - m_stepStartThreshold);
                angle = angleRight;
            }

            _angle = angle;
            float angleMapped = mapToRange(m_StepEndThreshold, m_maxStepAngle, 0.0f, Mathf.PI * 0.5f, angle);
            Debug.Log("Angle Mapped: " + angleMapped + " Angle: " + angle);
            float moveSpeedValue = (Mathf.Sin(angleMapped) + 1.0f) * 0.5f;

            m_currentSpeed = moveSpeedValue * m_maxWalkingSpeed;

            if(angle <= m_StepEndThreshold)
            {
                m_status = WIPStatus.EndStep;
                m_currentSpeed = 0.0f;
            }
        }

        private float mapToRange(float min, float max, float targetMin, float targetMax, float value)
        {
            return (value <= min) ? 0.0f :
                   (value >= max) ? 1.0f :
                   (value - min) * (targetMax - targetMin) / (max - min) + targetMin;
        }
    }
}