using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using System.Collections.Generic;
using Random = UnityEngine.Random;

[RequireComponent(typeof(JointDriveController))]
public class CrawlerAgentVisual6LegsWalker : Agent
{
    [Header("Walk Speed")]
    [Range(0.1f, m_maxWalkingSpeed)]
    [SerializeField]
    private float m_TargetWalkingSpeed = m_maxWalkingSpeed;
    public float TotalReward = 0f;
    const float m_maxWalkingSpeed = 10;

    public float TargetWalkingSpeed
    {
        get
        {
            if (UseHighLevelControl)
            {
                //if (AllowSpeedControl)
                //    return Mathf.Clamp(NormalizedTargetSpeed, 0.1f, 1f) * m_maxWalkingSpeed;
                //else
                return m_maxWalkingSpeed;
            }
            else
            {
                return m_TargetWalkingSpeed;
            }
        }
        set
        {
            if (!UseHighLevelControl)
                m_TargetWalkingSpeed = Mathf.Clamp(value, .1f, m_maxWalkingSpeed);
        }
    }

    [Header("Target To Walk Towards")]
    public Transform TargetPrefab;
    private Transform m_Target;

    [Header("Body Parts")][Space(10)]
    public Transform body;
    public Transform leg0Upper, leg0Lower, leg1Upper, leg1Lower, leg2Upper, leg2Lower, leg3Upper, leg3Lower, leg4Upper, leg4Lower, leg5Upper, leg5Lower;

    OrientationCubeController m_OrientationCube;
    public DirectionIndicator m_DirectionIndicator;
    JointDriveController m_JdController;

    [Header("Foot Grounded Visualization")]
    public bool useFootGroundedVisualization;
    public MeshRenderer foot0, foot1, foot2, foot3, foot4, foot5;
    public Material groundedMaterial, unGroundedMaterial;

    [Header("High-level control")]
    [Tooltip("Set by high-level agent (e.g., CrawlerAgent6Legs). -1=left, 0=forward, 1=right")]
    public float NormalizedTargetAngle = 0f;
    public float maxTurnAngle = 270f;

    [Tooltip("If true, walker uses direction from high-level agent. If false, uses target position.")]
    public bool UseHighLevelControl = false;
    //public bool AllowSpeedControl = false;

    public float NormalizedTargetSpeed = 1f;

    public override void Initialize()
    {
        SpawnTarget(TargetPrefab, transform.position);

        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();
        m_JdController = GetComponent<JointDriveController>();

        m_JdController.SetupBodyPart(body);
        m_JdController.SetupBodyPart(leg0Upper);
        m_JdController.SetupBodyPart(leg0Lower);
        m_JdController.SetupBodyPart(leg1Upper);
        m_JdController.SetupBodyPart(leg1Lower);
        m_JdController.SetupBodyPart(leg2Upper);
        m_JdController.SetupBodyPart(leg2Lower);
        m_JdController.SetupBodyPart(leg3Upper);
        m_JdController.SetupBodyPart(leg3Lower);
        m_JdController.SetupBodyPart(leg4Upper);
        m_JdController.SetupBodyPart(leg4Lower);
        m_JdController.SetupBodyPart(leg5Upper);
        m_JdController.SetupBodyPart(leg5Lower);
    }

    void SpawnTarget(Transform prefab, Vector3 pos)
    {
        if (UseHighLevelControl)
            return; // Не спавним цели в режиме high-level управления
        m_Target = Instantiate(prefab, pos, Quaternion.identity, transform.parent);
    }

    public override void OnEpisodeBegin()
    {
        //Debug.Log($"Searcher OnEpisodeBegin: {"Ноги заработали"}");
        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            bodyPart.Reset(bodyPart);
        }

        body.rotation = Quaternion.Euler(0, Random.Range(0.0f, 360.0f), 0);

        UpdateOrientationObjects();

        TargetWalkingSpeed = Random.Range(0.1f, m_maxWalkingSpeed);
        TotalReward = 0f;
        NormalizedTargetAngle = 0f;
    }

    public void CollectObservationBodyPart(BodyPart bp, VectorSensor sensor)
    {
        sensor.AddObservation(bp.groundContact.touchingGround);
        if (bp.rb.transform != body)
        {
            sensor.AddObservation(bp.currentStrength / m_JdController.maxJointForceLimit);
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Добавляем управляющий параметр от головы
        if (UseHighLevelControl) 
        {
            sensor.AddObservation(NormalizedTargetAngle);
        }
        else
        {
            sensor.AddObservation(0f);
        }

        var cubeForward = m_OrientationCube.transform.forward;
        Vector3 moveDir;
        Vector3 velGoal;

        if (UseHighLevelControl)
        {
            moveDir = Quaternion.AngleAxis(NormalizedTargetAngle * maxTurnAngle, Vector3.up) * cubeForward;
            velGoal = moveDir * TargetWalkingSpeed;
        }
        else
        {
            moveDir = cubeForward;
            velGoal = cubeForward * TargetWalkingSpeed;
        }
        var avgVel = GetAvgVelocity();

        sensor.AddObservation(Vector3.Distance(velGoal, avgVel));
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(avgVel));
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(velGoal));
        sensor.AddObservation(Quaternion.FromToRotation(body.forward, moveDir));


        // Можно убрать наблюдение за позицией цели, если управляет голова
        if (!UseHighLevelControl)
        {
            sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(m_Target.position));
        }
        else
        {
            sensor.AddObservation(Vector3.zero);
        }

        RaycastHit hit;
        float maxRaycastDist = 10;
        if (Physics.Raycast(body.position, Vector3.down, out hit, maxRaycastDist))
        {
            sensor.AddObservation(hit.distance / maxRaycastDist);
        }
        else
            sensor.AddObservation(1);

        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            CollectObservationBodyPart(bodyPart, sensor);
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var bpDict = m_JdController.bodyPartsDict;
        var continuousActions = actions.ContinuousActions;
        var i = -1;

        bpDict[leg0Upper].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[leg1Upper].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[leg2Upper].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[leg3Upper].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[leg4Upper].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[leg5Upper].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[leg0Lower].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[leg1Lower].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[leg2Lower].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[leg3Lower].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[leg4Lower].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[leg5Lower].SetJointTargetRotation(continuousActions[++i], 0, 0);

        bpDict[leg0Upper].SetJointStrength(continuousActions[++i]);
        bpDict[leg1Upper].SetJointStrength(continuousActions[++i]);
        bpDict[leg2Upper].SetJointStrength(continuousActions[++i]);
        bpDict[leg3Upper].SetJointStrength(continuousActions[++i]);
        bpDict[leg4Upper].SetJointStrength(continuousActions[++i]);
        bpDict[leg5Upper].SetJointStrength(continuousActions[++i]);
        bpDict[leg0Lower].SetJointStrength(continuousActions[++i]);
        bpDict[leg1Lower].SetJointStrength(continuousActions[++i]);
        bpDict[leg2Lower].SetJointStrength(continuousActions[++i]);
        bpDict[leg3Lower].SetJointStrength(continuousActions[++i]);
        bpDict[leg4Lower].SetJointStrength(continuousActions[++i]);
        bpDict[leg5Lower].SetJointStrength(continuousActions[++i]);
    }

    void FixedUpdate()
    {
        UpdateOrientationObjects();

        if (useFootGroundedVisualization)
        {
            foot0.material = m_JdController.bodyPartsDict[leg0Lower].groundContact.touchingGround
                ? groundedMaterial
                : unGroundedMaterial;
            foot1.material = m_JdController.bodyPartsDict[leg1Lower].groundContact.touchingGround
                ? groundedMaterial
                : unGroundedMaterial;
            foot2.material = m_JdController.bodyPartsDict[leg2Lower].groundContact.touchingGround
                ? groundedMaterial
                : unGroundedMaterial;
            foot3.material = m_JdController.bodyPartsDict[leg3Lower].groundContact.touchingGround
                ? groundedMaterial
                : unGroundedMaterial;
        }

        // Используем направление из NormalizedTargetAngle
        var cubeForward = m_OrientationCube.transform.forward;
        Vector3 moveDir = cubeForward;
        if (UseHighLevelControl)
        {
            moveDir = Quaternion.AngleAxis(NormalizedTargetAngle * maxTurnAngle, Vector3.up) * cubeForward;
        }
        else
        {
            moveDir = cubeForward;
        }

        var matchSpeedReward = GetMatchingVelocityReward(moveDir * TargetWalkingSpeed, GetAvgVelocity());
        var lookAtTargetReward = (Vector3.Dot(moveDir, body.forward) + 1) * .5F;

        AddReward(matchSpeedReward * lookAtTargetReward);
        TotalReward += matchSpeedReward * lookAtTargetReward;
        // Debug.Log($"Episode Summary — MovementRewardSum: {TotalReward:F3}, StepCount: {Academy.Instance.TotalStepCount}");
        //Debug.Log($"moveDir: {moveDir}, TargetWalkingSpeed: {TargetWalkingSpeed}, NormalizedTargetAngle: {NormalizedTargetAngle}");
    }

    void UpdateOrientationObjects()
    {
        if (UseHighLevelControl)
        {
            // Поворачиваем только по Y (горизонтально)
            Quaternion lookRot = Quaternion.AngleAxis(NormalizedTargetAngle * 90f, Vector3.up);
            m_OrientationCube.transform.SetPositionAndRotation(
                body.position,
                lookRot * Quaternion.Euler(0, body.rotation.eulerAngles.y, 0)
            );
        }
        else
        {
            m_OrientationCube.UpdateOrientation(body, m_Target);
        }
        if (m_DirectionIndicator)
        {
            m_DirectionIndicator.MatchOrientation(m_OrientationCube.transform);
        }
    }

    Vector3 GetAvgVelocity()
    {
        Vector3 velSum = Vector3.zero;
        int numOfRb = 0;
        foreach (var item in m_JdController.bodyPartsList)
        {
            numOfRb++;
            velSum += item.rb.linearVelocity;
        }
        return velSum / numOfRb;
    }

    public float GetMatchingVelocityReward(Vector3 velocityGoal, Vector3 actualVelocity)
    {
        var velDeltaMagnitude = Mathf.Clamp(Vector3.Distance(actualVelocity, velocityGoal), 0, TargetWalkingSpeed);
        return Mathf.Pow(1 - Mathf.Pow(velDeltaMagnitude / TargetWalkingSpeed, 2), 2);
    }

    public void TouchedTarget()
    {
        AddReward(1f);
    }
}