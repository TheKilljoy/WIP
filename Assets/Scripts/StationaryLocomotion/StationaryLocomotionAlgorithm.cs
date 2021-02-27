using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Htw.Cave.Locomotion
{
    public interface StationaryLocomotionAlgorithm
    {
        float WalkingSpeed { get; set; }
        void UpdateTarget(Kinect.KinectActor trackedActor, Transform targetToMove);
        void UpdateTarget(Kinect.KinectActor trackedActor, Rigidbody targetToMove);
    }
}