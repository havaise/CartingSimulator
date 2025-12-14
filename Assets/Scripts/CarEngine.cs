using UnityEngine;

public class KartEngine : MonoBehaviour
{
    [Header("Import parametrs")]
    [SerializeField] private bool _import = false;
    [SerializeField] private KartConfig _kartConfig;

    [Header("RPM settings")]
    [SerializeField] private float _idleRpm = 1000f;
    [SerializeField] private float _maxRpm = 8000f;
    [SerializeField] private float _revLimiterRpm = 7500f;

    [Header("Drivetrain")]
    [SerializeField] private float _gearRatio = 8f;
    [SerializeField] private float _wheelRadius = 0.3f;

    [Header("Torque curve")]
    [SerializeField] private AnimationCurve _torqueCurve;

    [Header("Inertia & response")]
    [SerializeField] private float _flywheelInertia = 0.2f;
    [SerializeField] private float _throttleResponse = 5f;
    [SerializeField] private float _rpmResponse = 8f; // Новая: скорость следования RPM за скоростью

    [Header("Losses & load")]
    [SerializeField] private float _engineFrictionCoeff = 0.02f;
    [SerializeField] private float _loadTorqueCoeff = 5f;

    public float CurrentRpm { get; private set; }
    public float CurrentTorque { get; private set; }
    public float SmoothedThrottle { get; private set; }
    public float RevLimiterFactor { get; private set; } = 1f;

    private float _invInertiaFactor;

    private void Awake()
    {
        if (_import)
            Initialize();

        CurrentRpm = _idleRpm;
        _invInertiaFactor = 60f / (2f * Mathf.PI * Mathf.Max(_flywheelInertia, 0.0001f));
    }

    private void Initialize()
    {
        if (_kartConfig != null)
        {
            _torqueCurve = _kartConfig.engineTorqueCurve;
            _invInertiaFactor = _kartConfig.engineInertia;
            _maxRpm = _kartConfig.maxRpm;
            _gearRatio = _kartConfig.gearRatio;
            _wheelRadius = _kartConfig.wheelRadius;
        }
    }

    public float Simulate(float throttleInput, float forwardSpeed, float deltaTime)
    {
        // Сглаживание газа
        float targetThrottle = Mathf.Clamp(throttleInput, -1f, 1f);
        SmoothedThrottle = Mathf.MoveTowards(SmoothedThrottle, targetThrottle, _throttleResponse * deltaTime);

        // НОВОЕ: Расчет целевого RPM от скорости
        float wheelAngularSpeed = Mathf.Abs(forwardSpeed) / _wheelRadius; // рад/сек
        float targetRpmFromSpeed = wheelAngularSpeed * _gearRatio * 60f / (2f * Mathf.PI); // RPM
        
        // Целевой RPM = от скорости * газ + холостой ход
        float targetRpm = Mathf.Max(_idleRpm, targetRpmFromSpeed * SmoothedThrottle);
        targetRpm = Mathf.Clamp(targetRpm, _idleRpm, _maxRpm);

        // Плавное следование за целевым RPM
        CurrentRpm = Mathf.Lerp(CurrentRpm, targetRpm, _rpmResponse * deltaTime);

        // Обновление лимитера
        UpdateRevLimiterFactor();

        // Расчет крутящего момента (остается прежним)
        float maxTorqueAtRpm = _torqueCurve.Evaluate(CurrentRpm);
        float effectiveThrottle = SmoothedThrottle * RevLimiterFactor;
        float driveTorque = maxTorqueAtRpm * effectiveThrottle;

        float frictionTorque = _engineFrictionCoeff * CurrentRpm;
        float loadTorque = _loadTorqueCoeff * Mathf.Abs(forwardSpeed);
        float netTorque = driveTorque - frictionTorque - loadTorque;

        // Остаточная физическая инерция (опционально, можно убрать для чистой зависимости)
        float rpmDot = netTorque * _invInertiaFactor * 0.3f; // Уменьшенный вклад
        CurrentRpm += rpmDot * deltaTime;
        CurrentRpm = Mathf.Clamp(CurrentRpm, _idleRpm, _maxRpm);

        CurrentTorque = driveTorque;
        return CurrentTorque;
    }

    private void UpdateRevLimiterFactor()
    {
        if (CurrentRpm <= _revLimiterRpm)
        {
            RevLimiterFactor = 1f;
            return;
        }

        if (CurrentRpm >= _maxRpm)
        {
            RevLimiterFactor = 0f;
            return;
        }

        float t = (CurrentRpm - _revLimiterRpm) / (_maxRpm - _revLimiterRpm);
        RevLimiterFactor = 1f - t;
    }
}
