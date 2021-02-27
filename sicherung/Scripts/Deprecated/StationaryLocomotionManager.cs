using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Htw.Cave.Locomotion
{
    [ExecuteAlways]
    public class StationaryLocomotionManager : MonoBehaviour
    {
        //ToDo(Robin): Make a fallback order of tracked nodes for locomotion variants
        //ToDo(Robin): Make a sequence of locomotion variants for switching between them  

        //public StationaryLocomotionVariant[] locomotionFallbackOrder;

        public float thresholdAngleInDegreeInStep = 50.0f;
        public float thresholdKneeHipLengthInPercentageInStep = 0.7f;
        public float thresholdAngleInDegreeOutStep = 65.0f;
        public float thresholdKneeHipLengthInPercentageOutStep = 0.85f;

        //private StationaryLocomotionVariant currentLocomotionVariant;
        public Transform target;
        private WalkInPlace walkingAlgorithm = new WalkInPlace();

        [SerializeField]
        private Transform Head;

        [SerializeField]
        private Transform LeftShoulder;

        [SerializeField]
        private Transform RightShoulder;

        [SerializeField]
        private Transform LeftElbow;

        [SerializeField]
        private Transform RightElbow;

        [SerializeField]
        private Transform LeftHand;

        [SerializeField]
        private Transform RightHand;

        [SerializeField]
        private Transform LeftHip;

        [SerializeField]
        private Transform RightHip;

        [SerializeField]
        private Transform LeftKnee;

        [SerializeField]
        private Transform RightKnee;

        [SerializeField]
        private Transform LeftFoot;

        [SerializeField]
        private Transform RightFoot;

        [SerializeField]
        private Transform SpineBase;

        public UnityEngine.UI.Text currentStateText;
        public UnityEngine.UI.Text LastFootText;
        public UnityEngine.UI.Text LeftAngleText;
        public UnityEngine.UI.Text RightAngleText;
        
        private float rightAngle;
        private float leftAngle;

        public void Start()
        {
            walkingAlgorithm.SetTransformToMove(target);
        }

        public void Update()
        {
            Vector3 leftKneeToFoot = LeftFoot.position - LeftKnee.position;
            Vector3 leftKneeToHip = LeftHip.position - LeftKnee.position;

            Vector3 rightKneeToFoot = RightFoot.position - RightKnee.position;
            Vector3 rightKneeToHip = RightHip.position - RightKnee.position;

            walkingAlgorithm.thresholdAngleInDegreeInStep = thresholdAngleInDegreeInStep;
            walkingAlgorithm.thresholdAngleInDegreeOutStep = thresholdAngleInDegreeOutStep;
            walkingAlgorithm.thresholdKneeHipLengthInPercentageInStep = thresholdKneeHipLengthInPercentageInStep;
            walkingAlgorithm.thresholdKneeHipLengthInPercentageOutStep = thresholdKneeHipLengthInPercentageOutStep;

            walkingAlgorithm.UpdateLocomotion(
                new Vector3[]{
                    SpineBase.position,
                    LeftKnee.position,
                    leftKneeToFoot,
                    leftKneeToHip,
                    RightKnee.position,
                    rightKneeToFoot,
                    rightKneeToHip,
                    SpineBase.right,
                    SpineBase.forward
                });
            WalkInPlace();
        }

        public void WalkInPlace()
        {
            Vector3 rightKneeToFoot = RightFoot.position - RightKnee.position;
            Vector3 rightKneeToHip =  RightHip.position -  RightKnee.position;
            rightAngle = Vector3.Angle(rightKneeToFoot, rightKneeToHip);

            Vector3 leftKneeToFoot = LeftFoot.position - LeftKnee.position;
            Vector3 leftKneeToHip =  LeftHip.position -  LeftKnee.position;
            leftAngle = Vector3.Angle(leftKneeToFoot, leftKneeToHip);

            currentStateText.text = "CurrentState: " + walkingAlgorithm.currentWalkState;
            LastFootText.text = "Last Foot: " + walkingAlgorithm.lastFootStepped;
            LeftAngleText.text = "LeftAngle: " + leftAngle;
            RightAngleText.text = "RightAngle: " + rightAngle;
        }

        public void OnDrawGizmos()
        {
            UnityEditor.Handles.color = Color.red;
            UnityEditor.Handles.DrawSolidArc(RightKnee.position, 
                                            SpineBase.right, 
                                            (RightFoot.position - RightKnee.position).normalized, 
                                            rightAngle, 
                                            0.25f);

            UnityEditor.Handles.DrawSolidArc(LeftKnee.position,
                                             SpineBase.right,
                                            (LeftFoot.position - LeftKnee.position).normalized,
                                             leftAngle,
                                             0.25f);

            UnityEditor.Handles.PositionHandle(LeftKnee.position, LeftKnee.rotation);
            UnityEditor.Handles.PositionHandle(LeftFoot.position, LeftFoot.rotation);
            UnityEditor.Handles.PositionHandle(SpineBase.position, SpineBase.rotation);
            UnityEditor.Handles.PositionHandle(RightKnee.position, RightKnee.rotation);
            UnityEditor.Handles.PositionHandle(RightFoot.position, RightFoot.rotation);

            UnityEditor.Handles.color = Color.black;
            UnityEditor.Handles.DrawAAPolyLine(
                    Head.position,
                    LeftShoulder.position,
                    LeftElbow.position,
                    LeftHand.position,
                    LeftElbow.position,
                    LeftShoulder.position,
                    LeftHip.position,
                    LeftShoulder.position,
                    RightShoulder.position,
                    RightElbow.position,
                    RightHand.position,
                    RightElbow.position,
                    RightShoulder.position,
                    Head.position,
                    RightShoulder.position,
                    RightHip.position,
                    LeftHip.position,
                    LeftKnee.position,
                    LeftFoot.position,
                    LeftKnee.position,
                    LeftHip.position,
                    RightHip.position,
                    RightKnee.position,
                    RightFoot.position
                );
        }
    }

}
