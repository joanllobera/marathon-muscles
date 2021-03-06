using System;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents.Sensors;
using UnityEngine;

public abstract class ObservationSource: MonoBehaviour
{
    public abstract void FeedObservationsToSensor(VectorSensor sensor);
    public abstract void OnAgentInitialize();
    public abstract int Size { get; }
}
