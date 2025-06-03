using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using System.Collections.Generic;

public class CrawlerAgentVisual6LegsSearcher : Agent
{
    [Header("References")]
    public CrawlerAgentVisual6LegsWalker walker; // Ссылка на prefab ног
    public Camera agentCamera; // Камера с CameraSensor, прикреплённая к body

    [Header("Target To Walk Towards")] 
    public Transform TargetPrefab;
    private List<Transform> m_Targets = new List<Transform>();

    [Header("Control")]
    [Tooltip("Максимальный угол поворота (градусы) для NormalizedTargetAngle")]
    public float maxTurnAngle = 270f;
    //public bool controlSpeed = false;


    private float foodRewardSum = 0f;

    public override void Initialize()
    {
        if (walker == null)
        {
            walker = GetComponentInChildren<CrawlerAgentVisual6LegsWalker>();
            if (walker == null)
                Debug.LogError("CrawlerAgentVisual6LegsWalker not found!");
        }
        // Отключаем самостоятельное управление у ног
        walker.UseHighLevelControl = true;

        walker.maxTurnAngle = maxTurnAngle;

        if (TargetPrefab != null)
        {
            for (int i = 0; i < 8; i++)
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
    }

    void SpawnTarget(Transform prefab, Vector3 pos)
    {
        var newTarget = Instantiate(prefab, pos, Quaternion.identity, transform.parent);
        m_Targets.Add(newTarget);
    }

    public override void OnEpisodeBegin()
    {
        //Debug.Log($"Searcher OnEpisodeBegin: {"Голова заработала"}");
        // Сбросить ноги и позицию головы
        walker.OnEpisodeBegin();
        // Можно добавить сброс позиции головы, если требуется
        Debug.Log($"Episode Summary — FoodRewardSum: {foodRewardSum:F3}");
        foodRewardSum = 0f;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Визуальные наблюдения идут через CameraSensor, вручную ничего добавлять не нужно
        // Можно добавить дополнительные наблюдения, если нужно (например, энергию, если она есть)
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Ожидаем одно непрерывное действие: направление движения [-1, 1]
        float direction = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);

        // Передаём управление ногам
        walker.NormalizedTargetAngle = direction;
        /*
        if (controlSpeed && actions.ContinuousActions.Length > 1)
        {
            float speed = Mathf.Clamp01(actions.ContinuousActions[1]);
            walker.NormalizedTargetSpeed = speed;
        }
        */
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal"); // угол

        // Управление скоростью: стрелка вверх — быстрее, вниз — медленнее
        /*
        float speed = 1f;
        if (controlSpeed)
        {
            if (Input.GetKey(KeyCode.UpArrow))
                speed = 1f;
            else if (Input.GetKey(KeyCode.DownArrow))
                speed = 0.3f;
            else
                speed = 0.7f; // средняя скорость по умолчанию

            continuousActionsOut[1] = speed;
        }
        */
    }

    void OnTargetTouched(Collision col)
    {
        AddReward(1f);
        foodRewardSum += 1f;
    }

    void LateUpdate()
    {
        if (agentCamera == null || walker == null || walker.body == null)
            return;

        // Камера следует за body Walker-а
        //agentCamera.transform.position = walker.body.position + new Vector3(0, 0.3f, 0); // смещение по высоте (можно настроить)

        // Стабилизируем направление по горизонтали
        Vector3 flatForward = walker.body.forward;
        flatForward.y = 0;
        if (flatForward.sqrMagnitude < 0.001f) return;

        agentCamera.transform.rotation = Quaternion.LookRotation(flatForward.normalized, Vector3.up);
    }
}