using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Htw.Cave.Locomotion
{
    public enum LocomotionAlgorithm
    {
        WalkInPlace,
        HumanJoystick,
    }

    public enum WIPStatus
    {
        Stationary,
        BeginStep,
        EndStep,
    }

    public enum WIPFoot
    {
        None,
        Left,
        Right,
    }
}
