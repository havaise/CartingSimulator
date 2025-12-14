using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class KartController : MonoBehaviour
{
    [Header("Import parametrs")]
    [SerializeField] private bool _import = false;
    [SerializeField] private KartConfig _kartConfig;

    [Header("Wheel attachment points")]
    [SerializeField] private Transform _frontLeftWheel;
    [SerializeField] private Transform _frontRightWheel;
    [SerializeField] private Transform _rearLeftWheel;
    [SerializeField] private Transform _rearRightWheel;

    [Header("Input (New Input System)")]
    [SerializeField] private InputActionAsset _playerInput;

    [Header("Weight distribution")]
    [SerializeField, Range(0, 1)] private float _frontAxisShare = 0.5f;

    [Header("Engine & drivetrain")]
    [SerializeField] private KartEngine _engine;
    [SerializeField] private float _gearRatio = 8f;
    [SerializeField] private float _drivetrainEfficiency = 0.9f;

    private InputAction _moveAction;
    private float _throttleInput;
    private float _steepInput;
    private bool _handbrakePressed; // ← ручник

    private float _frontLeftNormalForce, _frontRightNormalForce, _rearLeftNormalForce, _rearRightNormalForce;
    private Rigidbody _rigidbody;
    private Vector3 g = Physics.gravity;

    [SerializeField] private float engineTorque = 400f;
    [SerializeField] private float wheelRadius = 0.3f;
    [SerializeField] private float maxSpeed = 20;

    [Header("Steering")]
    [SerializeField] private float maxSteeringAngle;

    private Quaternion frontLeftInitialRot;
    private Quaternion frontRightInitialRot;

    [Header("Tyre friction")]
    [SerializeField] private float frictionCoefficient = 1f;
    [SerializeField] private float lateralStiffnes = 80f;
    [SerializeField] private float rollingResistance = 30f; // важно! было пусто — подбери 20-50

    [Header("Handbrake")]
    [SerializeField] private float handbrakeRollingMultiplier = 8f; // насколько сильнее тормозит зад при ручнике

    private float speedAlongForward = 0f;
    private float Fx = 0f;
    private float Fy = 0f;

    private void Awake()
    {
        _playerInput.Enable();
        _rigidbody = GetComponent<Rigidbody>();

        var map = _playerInput.FindActionMap("Kart");
        _moveAction = map.FindAction("Move");

        if (_import) Initialize();

        frontLeftInitialRot = _frontLeftWheel.localRotation;
        frontRightInitialRot = _frontRightWheel.localRotation;

        ComputeStaticWheelLoad();
    }

    private void Initialize()
    {
        if (_kartConfig != null)
        {
            _rigidbody.mass = _kartConfig.mass;
            frictionCoefficient = _kartConfig.frictionCoefficient;
            rollingResistance = _kartConfig.rollingResistance;
            maxSteeringAngle = _kartConfig.maxSteerAngle;
            _gearRatio = _kartConfig.gearRatio;
            wheelRadius = _kartConfig.wheelRadius;
            lateralStiffnes = _kartConfig.lateralStiffness;
        }
    }

    private void OnDisable()
    {
        _playerInput.Disable();
    }

    private void Update()
    {
        ReadInput();
        RotateFrontWheels();
    }

    private void ReadInput()
    {
        Vector2 move = _moveAction.ReadValue<Vector2>();
        _steepInput = Mathf.Clamp(move.x, -1, 1);
        _throttleInput = Mathf.Clamp(move.y, -1, 1);

        if (Input.GetKey(KeyCode.S))
            _throttleInput = -1f;

        // Ручник — лучше через Input System, но пока через Space
        _handbrakePressed = Input.GetKey(KeyCode.Space);

        // Если хочешь через Input System — раскомментируй:
        // var handbrakeAction = map.FindAction("Handbrake");
        // _handbrakePressed = handbrakeAction.IsPressed();
    }

    void RotateFrontWheels()
    {
        float steerAngle = maxSteeringAngle * _steepInput;
        Quaternion steerRot = Quaternion.Euler(0, steerAngle, 0);
        _frontLeftWheel.localRotation = frontLeftInitialRot * steerRot;
        _frontRightWheel.localRotation = frontRightInitialRot * steerRot;
    }

    void ComputeStaticWheelLoad()
    {
        float mass = _rigidbody.mass;
        float totalWeight = mass * Mathf.Abs(g.y);
        float frontWeight = totalWeight * _frontAxisShare;
        float rearWeight = totalWeight - frontWeight;

        _frontRightNormalForce = _frontLeftNormalForce = frontWeight * 0.5f;
        _rearRightNormalForce = _rearLeftNormalForce = rearWeight * 0.5f;
    }

    private void ApplyEngineForces()
    {
        Vector3 forward = transform.forward;
        float speedAlongForward = Vector3.Dot(_rigidbody.linearVelocity, forward);
        if (_throttleInput > 0 && speedAlongForward > maxSpeed) return;

        float driveTorque = engineTorque * _throttleInput;
        float driveForcePerWheel = driveTorque / wheelRadius / 2;
        Vector3 forceRear = forward * driveForcePerWheel;

        _rigidbody.AddForceAtPosition(forceRear, _rearLeftWheel.position, ForceMode.Force);
        _rigidbody.AddForceAtPosition(forceRear, _rearRightWheel.position, ForceMode.Force);
    }

    private void FixedUpdate()
    {
        ApplyEngineForces();

        ApplyWheelForce(_frontLeftWheel, _frontLeftNormalForce, isSteer: true, isDrive: false);
        ApplyWheelForce(_frontRightWheel, _frontRightNormalForce, isSteer: true, isDrive: false);
        ApplyWheelForce(_rearLeftWheel, _rearLeftNormalForce, isSteer: false, isDrive: true);
        ApplyWheelForce(_rearRightWheel, _rearRightNormalForce, isSteer: false, isDrive: true);
    }

    void ApplyWheelForce(Transform wheel, float normalForce, bool isSteer, bool isDrive)
    {
        Vector3 wheelPos = wheel.position;
        Vector3 wheelForward = wheel.forward;
        Vector3 wheelRight = wheel.right;
        Vector3 velocity = _rigidbody.GetPointVelocity(wheelPos);
        float vlong = Vector3.Dot(velocity, wheelForward);
        float vlat = Vector3.Dot(velocity, wheelRight);

        Fx = 0f;
        Fy = 0f;

        bool isRear = (wheel == _rearLeftWheel || wheel == _rearRightWheel);

        // === РУЧНИК ПО ТРЕБОВАНИЯМ ===
        float currentLateralStiffness = lateralStiffnes;
        float currentRollingResistance = rollingResistance;

        if (isRear && _handbrakePressed)
        {
            currentLateralStiffness = 0f; // Cα = 0 → обязательный занос по требованиям

            // Усиливаем сопротивление качению — основное торможение
            currentRollingResistance *= handbrakeRollingMultiplier; // в инспекторе поставь 15–25

            // Дополнительное контролируемое торможение (ТОЛЬКО против скорости, положительное значение!)
            float additionalBrakeForce = 1200f; // подбери: 800 = мягко, 2000 = очень резко
            if (Mathf.Abs(vlong) > 0.5f) // применяем только если колесо катится
            {
                float brakeDir = vlong > 0 ? -1f : 1f; // всегда против направления движения
                Fx += brakeDir * additionalBrakeForce;
            }
        }

        // Тяга от KartEngine (твоя основная система)
        if (isDrive)
        {
            speedAlongForward = Vector3.Dot(_rigidbody.linearVelocity, transform.forward);
            float engineTorqueOut = _engine.Simulate(_throttleInput, speedAlongForward, Time.fixedDeltaTime);
            float totalWheelTorque = engineTorqueOut * _gearRatio * _drivetrainEfficiency;
            float wheelTorque = totalWheelTorque * 0.5f;
            Fx += wheelTorque / wheelRadius;
        }

        // Сопротивление качению — только для передних (как было у тебя изначально)
        if (isSteer)
        {
            float rooling = -currentRollingResistance * vlong;
            Fx += rooling;
        }

        // Боковая сила
        float fyRaw = -currentLateralStiffness * vlat;
        Fy += fyRaw;

        // Твоё старое ограничение трения (именно оно давало хорошую поворачиваемость)
        float frictionlimit = frictionCoefficient * normalForce;
        float forceLenght = Mathf.Sqrt(Fx * Fx + Fy * Fy);
        if (forceLenght > frictionlimit)
        {
            float scale = frictionlimit / forceLenght;
            Fy += scale;  // ← твоя "фишка" — добавляем, а не умножаем
            Fx += scale;
        }

        Vector3 force = wheelForward * Fx + wheelRight * Fy;
        _rigidbody.AddForceAtPosition(force, wheel.position, ForceMode.Force);
    }

void OnGUI()
{
    GUIStyle titleStyle = new GUIStyle(GUI.skin.label);
    titleStyle.fontSize = 28;
    titleStyle.normal.textColor = Color.cyan;
    titleStyle.fontStyle = FontStyle.Bold;

    GUIStyle mainStyle = new GUIStyle(GUI.skin.label);
    mainStyle.fontSize = 22;
    mainStyle.normal.textColor = Color.white;

    GUIStyle valueStyle = new GUIStyle(GUI.skin.label);
    valueStyle.fontSize = 20;
    valueStyle.normal.textColor = Color.green;

    GUIStyle warningStyle = new GUIStyle(GUI.skin.label);
    warningStyle.fontSize = 20;
    warningStyle.normal.textColor = Color.yellow;

    GUIStyle forceStyle = new GUIStyle(GUI.skin.label);
    forceStyle.fontSize = 18;
    forceStyle.normal.textColor = Color.magenta;

    GUIStyle bgStyle = new GUIStyle();
    bgStyle.normal.background = MakeTex(2, 2, new Color(0f, 0f, 0f, 0.7f));
    bgStyle.padding = new RectOffset(20, 20, 15, 15);

    GUILayout.BeginArea(new Rect(30, 30, 450, 600), bgStyle);

    
    GUILayout.Label("KART TELEMETRY DASHBOARD", titleStyle);
    GUILayout.Space(15);

    // Основные параметры
    GUILayout.Label("SPEED", mainStyle);
    GUILayout.Label($"{speedAlongForward:0.0} m/s", valueStyle);
    GUILayout.Label($"({(speedAlongForward * 3.6f):0.0} km/h)", valueStyle);
    GUILayout.Space(8);

    GUILayout.Label("ENGINE RPM", mainStyle);
    GUILayout.Label($"{_engine.CurrentRpm:0} RPM", valueStyle);
    GUILayout.Space(8);

    // Крутящий момент с цветовой индикацией
    Color torqueColor = _engine.CurrentTorque > 300 ? Color.green : Color.yellow;
    GUI.color = torqueColor;
    GUILayout.Label("ENGINE TORQUE", mainStyle);
    GUILayout.Label($"{_engine.CurrentTorque:0.0} N·m", valueStyle);
    GUI.color = Color.white;
    GUILayout.Space(8);

    // Ручник
    if (_handbrakePressed)
    {
        GUI.color = Color.red;
        GUILayout.Label("HANDBRAKE ACTIVE", warningStyle);
        GUILayout.Label("DRIFT MODE", warningStyle);
        GUI.color = Color.white;
        GUILayout.Space(10);
    }

    // Силы на колесах
    GUILayout.Label("WHEEL FORCES", mainStyle);
    GUILayout.Space(5);
    GUILayout.Label($"Longitudinal: {Fx:0.0} N", forceStyle);
    GUILayout.Label($"Lateral: {Fy:0.0} N", forceStyle);

    GUILayout.EndArea();
}



    private Texture2D MakeTex(int width, int height, Color col)
    {
        Color[] pix = new Color[width * height];
        for (int i = 0; i < pix.Length; ++i) pix[i] = col;
        Texture2D result = new Texture2D(width, height);
        result.SetPixels(pix);
        result.Apply();
        return result;
    }
}