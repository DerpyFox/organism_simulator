using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using System.Collections.Generic;
using Random = UnityEngine.Random;

[RequireComponent(typeof(JointDriveController))]
public class CrawlerAgentVisual6Legs : Agent
{
    [Header("Walk Speed")]
    [Range(0.1f, m_maxWalkingSpeed)]
    [SerializeField]
    private float m_TargetWalkingSpeed = m_maxWalkingSpeed;

    const float m_maxWalkingSpeed = 15;

    public float TargetWalkingSpeed
    {
        get { return m_TargetWalkingSpeed; }
        set { m_TargetWalkingSpeed = Mathf.Clamp(value, .1f, m_maxWalkingSpeed); }
    }

    [Header("Target To Walk Towards")] 
    public Transform TargetPrefab; 
    private List<Transform> m_Targets = new List<Transform>();

    [Header("Body Parts")][Space(10)] 
    public Transform body;
    public Transform leg0Upper, leg0Lower, leg1Upper, leg1Lower, leg2Upper, leg2Lower, leg3Upper, leg3Lower, leg4Upper, leg4Lower, leg5Upper, leg5Lower;

    [Header("Camera Sensor")] 
    public Camera agentCamera; 
    public bool useVisualObservations = true;

    private OrientationCubeController m_OrientationCube;
    private JointDriveController m_JdController;

    [Header("Foot Grounded Visualization")]
    public bool useFootGroundedVisualization;
    public MeshRenderer foot0, foot1, foot2, foot3, foot4, foot5;
    public Material groundedMaterial, unGroundedMaterial;
    

    private float lastDistanceToTarget;
    private float movementRewardSum = 0f;
    private float foodRewardSum = 0f;
    private float wallsRewardSum = 0f;
    private float lastDistanceToVisibleTarget = 0f;
    private float visibleTargetRewardSum = 0f;

    public override void Initialize()
    {
        agentCamera = GetComponentInChildren<Camera>();
        if (agentCamera == null)
            Debug.LogError("Camera not found in agent's children!");

        if (TargetPrefab != null)
        {
            for (int i = 0; i < 5; i++)
            {
                SpawnTarget(TargetPrefab, transform.position);
            }
        }
        else
            Debug.LogError("TargetPrefab is not assigned!");
        
        foreach (var target in m_Targets)
        {
            var tc = target.GetComponent<TargetController>();
            if (tc != null)
            {
                tc.onCollisionEnterEvent.AddListener(OnTargetTouched);
            }
        }
        
        // Настраиваем обработку столкновений со стенами
        var walls = FindObjectsOfType<WallController>();
        foreach (var wall in walls)
        {
            wall.onCollisionEnterEvent.AddListener(OnWallCollision);
        }


        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        m_JdController = GetComponent<JointDriveController>();

        if (m_JdController == null || m_OrientationCube == null)
            Debug.LogError("JDController or OrientationCube missing!");

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

        if (agentCamera == null)
            Debug.LogError("AgentCamera not assigned!");
    }

    void SpawnTarget(Transform prefab, Vector3 pos)
    {
        var newTarget = Instantiate(prefab, pos, Quaternion.identity, transform.parent);
        m_Targets.Add(newTarget);
    }

    public override void OnEpisodeBegin()
    {
        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            bodyPart.Reset(bodyPart);
        }

        body.rotation = Quaternion.Euler(0, Random.Range(0.0f, 360.0f), 0);
        TargetWalkingSpeed = Random.Range(0.1f, m_maxWalkingSpeed);

        if (m_Targets != null && m_Targets.Count > 0)
        {
            var visibleTarget = GetVisibleTarget();
            if (visibleTarget != null)
            {
                lastDistanceToVisibleTarget = Vector3.Distance(transform.position, visibleTarget.position);
            }
            else
            {
                lastDistanceToVisibleTarget = 20f; // или максимальное расстояние по арене
            }
        }
        Debug.Log($"Episode Summary — FoodRewardSum: {foodRewardSum:F3}, MovementRewardSum: {movementRewardSum:F3}, wallsRewardSum: {wallsRewardSum:F3}, visibleTargetRewardSum: {visibleTargetRewardSum:F3}, StepCount: {Academy.Instance.TotalStepCount}");

        foodRewardSum = 0f;
        movementRewardSum = 0f;
        wallsRewardSum = 0f;
        visibleTargetRewardSum = 0f;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Наблюдения для движения (без данных о цели)
        var velGoal = body.forward * TargetWalkingSpeed;
        var avgVel = GetAvgVelocity();

        sensor.AddObservation(Vector3.Distance(velGoal, avgVel));
        sensor.AddObservation(avgVel.magnitude);
        sensor.AddObservation(Quaternion.FromToRotation(body.forward, velGoal.normalized));

        var visibleTarget = GetVisibleTarget();
        sensor.AddObservation(visibleTarget != null ? 1f : 0f);

        // Наблюдения за контактом с землёй
        RaycastHit hit;
        if (Physics.Raycast(body.position, Vector3.down, out hit, 10))
        {
            sensor.AddObservation(hit.distance / 10);
        }
        else
        {
            sensor.AddObservation(1);
        }

        // Состояние суставов и ног
        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            sensor.AddObservation(bodyPart.groundContact.touchingGround);
            if (bodyPart.rb.transform != body)
            {
                sensor.AddObservation(bodyPart.currentStrength / m_JdController.maxJointForceLimit);
            }
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

    private Transform GetVisibleTarget()
    {
        foreach (var target in m_Targets)
        {
            Vector3 directionToTarget = (target.position - agentCamera.transform.position).normalized;
            float dotProduct = Vector3.Dot(agentCamera.transform.forward, directionToTarget);

            if (dotProduct > Mathf.Cos(agentCamera.fieldOfView * 0.5f * Mathf.Deg2Rad))
                return target;
        }
        return null;
    }
    void FixedUpdate()
    {
        // Награда за скорость (без ориентации на цель)
        //var velGoal = body.forward * TargetWalkingSpeed;
        //var avgVel = GetAvgVelocity();
        //var matchSpeedReward = GetMatchingVelocityReward(velGoal, avgVel) * 0.1f;
        
        //AddReward(matchSpeedReward);
        //movementRewardSum += matchSpeedReward;
        
        if (m_Targets != null && m_Targets.Count > 0)
        {
            var visibleTarget = GetVisibleTarget();
            if (visibleTarget != null)
            {
                Vector3 toTarget = (visibleTarget.position - transform.position).normalized;
                Vector3 velocityGoal = toTarget * TargetWalkingSpeed;
                Vector3 agentVelocity = GetAvgVelocity();

                // Награда за совпадение скорости по направлению и модулю
                float matchSpeedReward = GetMatchingVelocityReward(velocityGoal, agentVelocity) * 0.5f;
                AddReward(matchSpeedReward);
                movementRewardSum += matchSpeedReward;

                // Можно оставить shaped reward за приближение, но уменьшить коэффициент
                float distanceToTarget = Vector3.Distance(transform.position, visibleTarget.position);
                float distanceDelta = lastDistanceToVisibleTarget - distanceToTarget;
                AddReward(distanceDelta * 2f); // уменьшить коэффициент
                visibleTargetRewardSum += distanceDelta;
                lastDistanceToVisibleTarget = distanceToTarget;
            }
        }

        // Визуализация контакта с землёй
        if (useFootGroundedVisualization)
        {
            foot0.material = m_JdController.bodyPartsDict[leg0Lower].groundContact.touchingGround ? groundedMaterial : unGroundedMaterial;
            foot1.material = m_JdController.bodyPartsDict[leg1Lower].groundContact.touchingGround ? groundedMaterial : unGroundedMaterial;
            foot2.material = m_JdController.bodyPartsDict[leg2Lower].groundContact.touchingGround ? groundedMaterial : unGroundedMaterial;
            foot3.material = m_JdController.bodyPartsDict[leg3Lower].groundContact.touchingGround ? groundedMaterial : unGroundedMaterial;
            foot4.material = m_JdController.bodyPartsDict[leg4Lower].groundContact.touchingGround ? groundedMaterial : unGroundedMaterial;
            foot5.material = m_JdController.bodyPartsDict[leg5Lower].groundContact.touchingGround ? groundedMaterial : unGroundedMaterial;
        }
    }

    Vector3 GetAvgVelocity()
    {
        Vector3 velSum = Vector3.zero;
        foreach (var item in m_JdController.bodyPartsList)
        {
            velSum += item.rb.linearVelocity;
        }
        return velSum / m_JdController.bodyPartsList.Count;
    }

    public float GetMatchingVelocityReward(Vector3 velocityGoal, Vector3 actualVelocity)
    {
        var velDeltaMagnitude = Mathf.Clamp(Vector3.Distance(actualVelocity, velocityGoal), 0, TargetWalkingSpeed);
        return Mathf.Pow(1 - Mathf.Pow(velDeltaMagnitude / TargetWalkingSpeed, 2), 2);
    }

    void OnTargetTouched(Collision col)
    {
        AddReward(75f);
        foodRewardSum += 75f;
    }

    void OnWallCollision(Collision col)
    {
        AddReward(-0.1f); // Штраф за касание стены
        wallsRewardSum += -0.1f;
    }


    void LateUpdate()
    {
        if (agentCamera == null || body == null) return;

        // Камера уже на нужной позиции (локально), просто стабилизируем направление
        Vector3 flatForward = body.forward;
        flatForward.y = 0;
        if (flatForward.sqrMagnitude < 0.001f) return;

        agentCamera.transform.rotation = Quaternion.LookRotation(flatForward.normalized, Vector3.up);
    }  
}