using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Htw.Cave.Locomotion
{
    public class WalkInPlace : StationaryLocomotionAlgorithm<Vector3[]>
    {
        public enum WalkState
        {
            Stationary,
            InStep,
        };

        public enum Foot
        {
            None,
            Left,
            Right
        };

        public float moveSpeed = 20.0f;

        private Transform targetTransform;
        private Rigidbody targetRigidbody;

        public float thresholdAngleInDegreeInStep = 50.0f;
        public float thresholdKneeHipLengthInPercentageInStep = 0.7f;
        public float thresholdAngleInDegreeOutStep = 65.0f;
        public float thresholdKneeHipLengthInPercentageOutStep = 0.85f;
        private float avgKneeHipLength = 0.0f;

        public WalkState currentWalkState = WalkState.Stationary;
        public Foot lastFootStepped = Foot.None;
        private const float StepResetDeltaTimeInSeconds = 1.5f;
        private float lastStepTaken = 0.0f;

        public void SetRigidbodyToMove(Rigidbody rigidbody)
        {
            this.targetRigidbody = rigidbody;
        }

        public void SetTransformToMove(Transform transform)
        {
            this.targetTransform = transform;
        }

        /// <summary>
        /// 6 Vectors: 
        /// 1: SpineBase,
        /// 2: LeftKneePosition,
        /// 3: First Direction Left (like knee to foot), 
        /// 4: Second Direction Left (like knee to hip), 
        /// 5: RightKneePosition,
        /// 6: First Direction Right (like knee to foot), 
        /// 7: Second Direction Right (like knee to hip), 
        /// 8: Axis
        /// 9: SpineBaseForwardDirection
        /// </summary>
        /// <param name="data"></param>
        public void UpdateLocomotion(Vector3[] data)
        {
            Debug.Assert(data.Length == 9);

            if(Time.time - lastStepTaken >= StepResetDeltaTimeInSeconds)
            {
                lastFootStepped = Foot.None;
                currentWalkState = WalkState.Stationary;
            }

            switch(currentWalkState)
            {
                case WalkState.Stationary:
                {
                    CheckForStep(data);
                }break;
                case WalkState.InStep:
                {
                    CheckForStepFinished(data);
                }break;
            }
        }

        private void CheckForStep(Vector3[] data)
        {
            if(lastFootStepped == Foot.None)
            {
                //check left foot
                if(CheckForStepPerFoot(new Vector3[] {data[0], data[1], data[2], data[3], data[7] }))
                {
                    lastFootStepped = Foot.Left;
                    lastStepTaken = Time.time;
                    return;
                }

                if(CheckForStepPerFoot(new Vector3[] { data[0], data[4], data[5], data[6], data[7] }))
                {
                    lastFootStepped = Foot.Right;
                    lastStepTaken = Time.time;
                    return;
                }
            }

            if(lastFootStepped == Foot.Right)
            {
                if(CheckForStepPerFoot(new Vector3[] { data[0], data[1], data[2], data[3], data[7] }))
                {
                    lastFootStepped = Foot.Left;
                    lastStepTaken = Time.time;
                    return;
                }
            }

            if(lastFootStepped == Foot.Left)
            {
                if(CheckForStepPerFoot(new Vector3[] { data[0], data[4], data[5], data[6], data[7] }))
                {
                    lastFootStepped = Foot.Right;
                    lastStepTaken = Time.time;
                    return;
                }
            }
        }

        private bool CheckForStepPerFoot(Vector3[] data)
        {

            if(Vector3.Angle(data[2], data[3]) > thresholdAngleInDegreeInStep)
                return false;

            float kneeHipLength = (data[0] - data[1]).magnitude;

            if(avgKneeHipLength < 0.01f)
                avgKneeHipLength = kneeHipLength;

            if ( Mathf.Abs((kneeHipLength / avgKneeHipLength) - 1.0f) < 0.05f)
            {
                avgKneeHipLength = (avgKneeHipLength + kneeHipLength) * 0.5f;
            }

            float heightDifference = Mathf.Abs(data[0].y - data[1].y);

            if((heightDifference / avgKneeHipLength) >= thresholdKneeHipLengthInPercentageInStep)
                return false;

            currentWalkState = WalkState.InStep;
            return true;
        }

        private void CheckForStepFinished(Vector3[] data)
        {
            if(lastFootStepped == Foot.Left)
            {
                if(CheckForStepPerFootFinished(new Vector3[] { data[0], data[1], data[2], data[3], data[7] }, -data[8]))
                {
                    lastStepTaken = Time.time;
                    currentWalkState = WalkState.Stationary;
                    return;
                }
            }

            if(lastFootStepped == Foot.Right)
            {
                if(CheckForStepPerFootFinished(new Vector3[] { data[0], data[4], data[5], data[6], data[7] }, -data[8]))
                {
                    lastStepTaken = Time.time;
                    currentWalkState = WalkState.Stationary;
                    return;
                }
            }
        }

        private bool CheckForStepPerFootFinished(Vector3[] data, Vector3 forward)
        {
            if(Vector3.Angle(data[2], data[3]) < thresholdAngleInDegreeOutStep)
            {
                Debug.Log("Angle is too small for step finish");
                return false;
            }

            //float kneeHipLength = (data[0] - data[1]).magnitude;

            //if(avgKneeHipLength < 0.01f)
            //    avgKneeHipLength = kneeHipLength;

            //if(Mathf.Abs((kneeHipLength / avgKneeHipLength) - 1.0f) < 0.05f)
            //{
            //    avgKneeHipLength = (avgKneeHipLength + kneeHipLength) * 0.5f;
            //}

            //float heightDifference = Mathf.Abs(data[0].y - data[1].y);

            //if((heightDifference / avgKneeHipLength) < thresholdKneeHipLengthInPercentageOutStep)
            //{
            //    Debug.Log("Foot position is not low enough");
            //    return false;
            //}

            currentWalkState = WalkState.Stationary;

            forward.y = 0.0f;
            forward.Normalize();

            if(targetRigidbody)
            {
                targetRigidbody.AddForce(forward * moveSpeed * Time.deltaTime, ForceMode.Acceleration);
                return true;
            }

            if(targetTransform)
            {
                targetTransform.position += forward * moveSpeed * Time.deltaTime;
                return true;
            }

            return true;
        }
    }
}

