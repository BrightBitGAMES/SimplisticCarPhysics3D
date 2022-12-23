using UnityEngine;

public class Car : MonoBehaviour
{
    [SerializeField] private bool IsPlayerControlled = false;
    [SerializeField] [Range(0f, 1f)] private float CGHeight = 0.55f;
    [SerializeField] [Range(0f, 2f)] private float InertiaScale = 1f;
    [SerializeField] private float BrakePower = 12000;
    [SerializeField] private float EBrakePower = 5000;
    [SerializeField] [Range(0f, 1f)] private float WeightTransfer = 0.35f;
    [SerializeField] [Range(0f, 1f)] private float MaxSteerAngle = 0.75f;
    [SerializeField] [Range(0f, 20f)] private float CornerStiffnessFront = 5.0f;
    [SerializeField] [Range(0f, 20f)] private float CornerStiffnessRear = 5.2f;
    [SerializeField] [Range(0f, 20f)] private float AirResistance = 2.5f;
    [SerializeField] [Range(0f, 20f)] private float RollingResistance = 8.0f;
    [SerializeField] [Range(0f, 1f)] private float EBrakeGripRatioFront = 0.9f;
    [SerializeField] [Range(0f, 5f)] private float TotalTireGripFront = 2.5f;
    [SerializeField] [Range(0f, 1f)] private float EBrakeGripRatioRear = 0.4f;
    [SerializeField] [Range(0f, 5f)] private float TotalTireGripRear = 2.5f;
    [SerializeField] [Range(0f, 5f)] private float SteerSpeed = 2.5f;
    [SerializeField] [Range(0f, 5f)] private float SteerAdjustSpeed = 1f;
    [SerializeField] [Range(0f, 1000f)] private float SpeedSteerCorrection = 300f;
    [SerializeField] [Range(0f, 20f)] private float SpeedTurningStability = 10f;
    [SerializeField] [Range(0f, 10f)] private float AxleDistanceCorrection = 2f;

    public float SpeedKilometersPerHour
    {
        get
        {
            return Rigidbody.velocity.magnitude * 18f / 5f;
        }
    }

    // Variables that get initialized via code
    private float Inertia    = 1;
    private float WheelBase  = 1;
    private float TrackWidth = 1;

    // Private vars
    private float HeadingAngle;
    private float Speed;
    private float AngularVelocity;
    private float SteerDirection;
    private float SteerAngle;

    private Vector3 Velocity;
    private Vector3 Acceleration;
    private Vector3 LocalVelocity;
    private Vector3 LocalAcceleration;

    private float Throttle;
    private float Brake;
    private float EBrake;

    private Rigidbody Rigidbody;

    private Axle   AxleFront;
    private Axle   AxleRear;
    private Engine Engine;

    private GameObject CenterOfGravity;

    private void Awake()
    {
        Rigidbody       = GetComponent<Rigidbody>();
        CenterOfGravity = transform.Find("CenterOfGravity").gameObject;

        AxleFront = transform.Find("AxleFront").GetComponent<Axle>();
        AxleRear  = transform.Find("AxleRear").GetComponent<Axle>();

        Engine = transform.Find("Engine").GetComponent<Engine>();

        Init();
    }

    private void Init()
    {
        Velocity = Vector3.zero;
        Speed    = 0;

        // Dimensions
        AxleFront.DistanceToCG = Vector3.Distance(CenterOfGravity.transform.position, AxleFront.transform.position);
        AxleRear.DistanceToCG  = Vector3.Distance(CenterOfGravity.transform.position, AxleRear.transform.position);
        // Extend the calculations past actual car dimensions for better simulation
        AxleFront.DistanceToCG *= AxleDistanceCorrection;
        AxleRear.DistanceToCG  *= AxleDistanceCorrection;

        WheelBase = AxleFront.DistanceToCG + AxleRear.DistanceToCG;
        Inertia   = Rigidbody.mass * InertiaScale;

        // Set starting angle of car
        HeadingAngle = transform.rotation.eulerAngles.y * Mathf.Deg2Rad;
    }

    private void Start()
    {
        AxleFront.Init(Rigidbody, WheelBase);
        AxleRear.Init(Rigidbody, WheelBase);

        TrackWidth = Vector3.Distance(AxleRear.TireLeft.transform.position, AxleRear.TireRight.transform.position);
    }

    private void Update()
    {
        if (IsPlayerControlled)
        {
            // Handle Input
            Throttle = 0;
            Brake    = 0;
            EBrake   = 0;

            float steerInput = 0;

            if (Input.GetKey(KeyCode.Space)) EBrake = 1;

                 if (Input.GetKey(KeyCode.UpArrow)) Throttle   = +1;
            else if (Input.GetKey(KeyCode.DownArrow)) Throttle = -1; // Brake = 1;

                 if (Input.GetKey(KeyCode.LeftArrow)) steerInput  = -1;
            else if (Input.GetKey(KeyCode.RightArrow)) steerInput = +1;

                 if (Input.GetKeyDown(KeyCode.A)) Engine.ShiftUp();
            else if (Input.GetKeyDown(KeyCode.Z)) Engine.ShiftDown();

            // Apply filters to our steer direction
            SteerDirection = SmoothSteering(steerInput);
            SteerDirection = SpeedAdjustedSteering(SteerDirection);

            // Calculate the current angle the tires are pointing
            SteerAngle = SteerDirection * MaxSteerAngle;

            var rotation = Quaternion.Euler(0, Mathf.Rad2Deg * SteerAngle, 0);

            // Set front axle tires rotation
            AxleFront.TireRight.transform.localRotation = rotation;
            AxleFront.TireLeft.transform.localRotation  = rotation;
        }

        // Calculate weight center of four tires
        // This is just to draw that red dot over the car to indicate what tires have the most weight
        var pos = Vector3.zero;

        if (LocalAcceleration.magnitude > 1f)
        {
            var wfl = Mathf.Max(0, (AxleFront.TireLeft.ActiveWeight  - AxleFront.TireLeft.RestingWeight));
            var wfr = Mathf.Max(0, (AxleFront.TireRight.ActiveWeight - AxleFront.TireRight.RestingWeight));
            var wrl = Mathf.Max(0, (AxleRear.TireLeft.ActiveWeight   - AxleRear.TireLeft.RestingWeight));
            var wrr = Mathf.Max(0, (AxleRear.TireRight.ActiveWeight  - AxleRear.TireRight.RestingWeight));

            pos = (AxleFront.TireLeft.LocalPosition)  * wfl +
                  (AxleFront.TireRight.LocalPosition) * wfr +
                  (AxleRear.TireLeft.LocalPosition)   * wrl +
                  (AxleRear.TireRight.LocalPosition)  * wrr;

            var weightTotal = wfl + wfr + wrl + wrr;

            if (weightTotal > 0)
            {
                pos /= weightTotal;
                pos.Normalize();
                pos.z = Mathf.Clamp(pos.z, -0.6f, 0.6f);
            }
            else
            {
                pos = Vector3.zero;
            }
        }

        // Update the "Center Of Gravity" dot to indicate the weight shift
        CenterOfGravity.transform.localPosition = Vector3.Lerp(CenterOfGravity.transform.localPosition, pos, 0.1f);

        // Skidmarks
        if (Mathf.Abs(LocalAcceleration.y) > 18 || EBrake == 1)
        {
            AxleRear.TireRight.SetTrailActive(true);
            AxleRear.TireLeft.SetTrailActive(true);
        }
        else
        {
            AxleRear.TireRight.SetTrailActive(false);
            AxleRear.TireLeft.SetTrailActive(false);
        }

        // Automatic transmission
        Engine.UpdateAutomaticTransmission(Rigidbody);
    }

    private void FixedUpdate()
    {
        // Update from rigidbody to retain collision responses
        Velocity     = Rigidbody.velocity;
        HeadingAngle = transform.rotation.eulerAngles.y * Mathf.Deg2Rad;

        var sin = Mathf.Sin(HeadingAngle);
        var cos = Mathf.Cos(HeadingAngle);

        // Get local velocity
        LocalVelocity.z = cos * Velocity.z + sin * Velocity.x;
        LocalVelocity.x = cos * Velocity.x - sin * Velocity.z;

        // Weight transfer
        var transferLongitudinal = WeightTransfer * LocalAcceleration.z * CGHeight / WheelBase;
        var transferLateral      = WeightTransfer * LocalAcceleration.x * CGHeight / TrackWidth * 20; // exaggerate the weight transfer on the y-axis

        // Weight on each axle
        var weightFront = Rigidbody.mass * (AxleFront.WeightRatio * -Physics.gravity.y - transferLongitudinal);
        var weightRear  = Rigidbody.mass * (AxleRear.WeightRatio  * -Physics.gravity.y + transferLongitudinal);

        // Weight on each tire
        AxleFront.TireLeft.ActiveWeight  = weightFront - transferLateral;
        AxleFront.TireRight.ActiveWeight = weightFront + transferLateral;
        AxleRear.TireLeft.ActiveWeight   = weightRear  - transferLateral;
        AxleRear.TireRight.ActiveWeight  = weightRear  + transferLateral;

        // Velocity of each tire
        AxleFront.TireLeft.AngularVelocity  = AxleFront.DistanceToCG * AngularVelocity;
        AxleFront.TireRight.AngularVelocity = AxleFront.DistanceToCG * AngularVelocity;
        AxleRear.TireLeft.AngularVelocity   = -AxleRear.DistanceToCG * AngularVelocity;
        AxleRear.TireRight.AngularVelocity  = -AxleRear.DistanceToCG * AngularVelocity;

        // Slip angle
        AxleFront.SlipAngle = Mathf.Atan2(LocalVelocity.x + AxleFront.AngularVelocity, Mathf.Abs(LocalVelocity.z)) - Mathf.Sign(LocalVelocity.z) * SteerAngle;
        AxleRear.SlipAngle  = Mathf.Atan2(LocalVelocity.x + AxleRear.AngularVelocity, Mathf.Abs(LocalVelocity.z));

        // Brake and Throttle power
        var activeBrake    = Mathf.Min(Brake * BrakePower + EBrake * EBrakePower, BrakePower);
        var activeThrottle = (Throttle * Engine.GetTorque(Rigidbody)) * (Engine.GearRatio * Engine.EffectiveGearRatio);

        // Torque of each tire (rear wheel drive)
        AxleRear.TireLeft.Torque  = activeThrottle / AxleRear.TireLeft.Radius;
        AxleRear.TireRight.Torque = activeThrottle / AxleRear.TireRight.Radius;

        // Grip and Friction of each tire
        AxleFront.TireLeft.Grip  = TotalTireGripFront * (1.0f - EBrake * (1.0f - EBrakeGripRatioFront));
        AxleFront.TireRight.Grip = TotalTireGripFront * (1.0f - EBrake * (1.0f - EBrakeGripRatioFront));
        AxleRear.TireLeft.Grip   = TotalTireGripRear  * (1.0f - EBrake * (1.0f - EBrakeGripRatioRear));
        AxleRear.TireRight.Grip  = TotalTireGripRear  * (1.0f - EBrake * (1.0f - EBrakeGripRatioRear));

        AxleFront.TireLeft.FrictionForce  = Mathf.Clamp(-CornerStiffnessFront * AxleFront.SlipAngle, -AxleFront.TireLeft.Grip,  AxleFront.TireLeft.Grip)  * AxleFront.TireLeft.ActiveWeight;
        AxleFront.TireRight.FrictionForce = Mathf.Clamp(-CornerStiffnessFront * AxleFront.SlipAngle, -AxleFront.TireRight.Grip, AxleFront.TireRight.Grip) * AxleFront.TireRight.ActiveWeight;
        AxleRear.TireLeft.FrictionForce   = Mathf.Clamp(-CornerStiffnessRear  * AxleRear.SlipAngle,  -AxleRear.TireLeft.Grip,   AxleRear.TireLeft.Grip)   * AxleRear.TireLeft.ActiveWeight;
        AxleRear.TireRight.FrictionForce  = Mathf.Clamp(-CornerStiffnessRear  * AxleRear.SlipAngle,  -AxleRear.TireRight.Grip,  AxleRear.TireRight.Grip)  * AxleRear.TireRight.ActiveWeight;

        // Forces
        var   tractionForceZ = AxleRear.Torque - activeBrake * Mathf.Sign(LocalVelocity.z);
        float tractionForceX = 0;

        var dragForceZ = -RollingResistance * LocalVelocity.z - AirResistance * LocalVelocity.z * Mathf.Abs(LocalVelocity.z);
        var dragForceX = -RollingResistance * LocalVelocity.x - AirResistance * LocalVelocity.x * Mathf.Abs(LocalVelocity.x);

        var totalForceZ = dragForceZ + tractionForceZ;
        var totalForceX = dragForceX + tractionForceX + Mathf.Cos(SteerAngle) * AxleFront.FrictionForce + AxleRear.FrictionForce;

        //adjust Y force so it levels out the car heading at high speeds
        if (Speed > 10)
        {
            totalForceX *= (Speed + 1) / (21f - SpeedTurningStability);
        }

        // If we are not pressing gas, add artificial drag - helps with simulation stability
        if (Throttle == 0)
        {
            Velocity = Vector3.Lerp(Velocity, Vector3.zero, 0.005f);
        }

        // Acceleration
        LocalAcceleration.z = totalForceZ / Rigidbody.mass;
        LocalAcceleration.x = totalForceX / Rigidbody.mass;

        Acceleration.z = cos * LocalAcceleration.z - sin * LocalAcceleration.x;
        Acceleration.x = sin * LocalAcceleration.z + cos * LocalAcceleration.x;

        // Velocity and speed
        Velocity.z += Acceleration.z * Time.deltaTime;
        Velocity.x += Acceleration.x * Time.deltaTime;

        Speed = Velocity.magnitude;

        // Angular torque of car
        var angularTorque = (AxleFront.FrictionForce * AxleFront.DistanceToCG) - (AxleRear.FrictionForce * AxleRear.DistanceToCG);

        // Car will drift away at low speeds
        if (Speed < 0.5f && activeThrottle == 0)
        {
            LocalAcceleration         = Vector3.zero;
            Speed                     = 0;
            Velocity                  = Vector3.zero;
            angularTorque             = 0;
            AngularVelocity           = 0;
            Acceleration              = Vector3.zero;
            Rigidbody.angularVelocity = Vector3.zero;
        }

        var angularAcceleration = angularTorque / Inertia;

        // Update 
        AngularVelocity += angularAcceleration * Time.deltaTime;

        // Simulation likes to calculate high angular velocity at very low speeds - adjust for this
        if (Speed < 1 && Mathf.Abs(SteerAngle) < 0.05f)
        {
            AngularVelocity = 0;
        }
        else if (SpeedKilometersPerHour < 0.75f)
        {
            AngularVelocity = 0;
        }

        HeadingAngle += AngularVelocity * Time.deltaTime;

        var oldVelocity = Rigidbody.velocity;
        var newVelocity = new Vector3(Velocity.x, oldVelocity.y, Velocity.z);

        var velocityDelta = newVelocity - oldVelocity;

        var rotation = Quaternion.AngleAxis(Mathf.Rad2Deg * HeadingAngle, Vector3.up);

        Rigidbody.AddForce(velocityDelta, ForceMode.VelocityChange);
        Rigidbody.MoveRotation(rotation);
    }

    private float SmoothSteering(float steerInput)
    {
        float steer = 0;

        if (Mathf.Abs(steerInput) > 0.001f)
        {
            steer = Mathf.Clamp(SteerDirection + steerInput * Time.deltaTime * SteerSpeed, -1.0f, 1.0f);
        }
        else
        {
            if (SteerDirection > 0)
            {
                steer = Mathf.Max(SteerDirection - Time.deltaTime * SteerAdjustSpeed, 0);
            }
            else if (SteerDirection < 0)
            {
                steer = Mathf.Min(SteerDirection + Time.deltaTime * SteerAdjustSpeed, 0);
            }
        }

        return steer;
    }

    private float SpeedAdjustedSteering(float steerInput)
    {
        return steerInput * (1.0f - (Speed / SpeedSteerCorrection));
    }

    private void OnGUI()
    {
        if (IsPlayerControlled)
        {
            GUI.Label(new Rect(5, 5,   300, 20), "Speed: "             + SpeedKilometersPerHour.ToString());
            GUI.Label(new Rect(5, 25,  300, 20), "RPM: "               + Engine.GetRPM(Rigidbody).ToString());
            GUI.Label(new Rect(5, 45,  300, 20), "Gear: "              + (Engine.CurrentGear + 1).ToString());
            GUI.Label(new Rect(5, 65,  300, 20), "LocalAcceleration: " + LocalAcceleration.ToString());
            GUI.Label(new Rect(5, 85,  300, 20), "Acceleration: "      + Acceleration.ToString());
            GUI.Label(new Rect(5, 105, 300, 20), "LocalVelocity: "     + LocalVelocity.ToString());
            GUI.Label(new Rect(5, 125, 300, 20), "Velocity: "          + Velocity.ToString());
            GUI.Label(new Rect(5, 145, 300, 20), "SteerAngle: "        + SteerAngle.ToString());
            GUI.Label(new Rect(5, 165, 300, 20), "Throttle: "          + Throttle.ToString());
            GUI.Label(new Rect(5, 185, 300, 20), "Brake: "             + Brake.ToString());

            GUI.Label(new Rect(5, 205, 300, 20), "HeadingAngle: "    + HeadingAngle.ToString());
            GUI.Label(new Rect(5, 225, 300, 20), "AngularVelocity: " + AngularVelocity.ToString());

            GUI.Label(new Rect(5, 245, 300, 20), "TireFL Weight: " + AxleFront.TireLeft.ActiveWeight.ToString());
            GUI.Label(new Rect(5, 265, 300, 20), "TireFR Weight: " + AxleFront.TireRight.ActiveWeight.ToString());
            GUI.Label(new Rect(5, 285, 300, 20), "TireRL Weight: " + AxleRear.TireLeft.ActiveWeight.ToString());
            GUI.Label(new Rect(5, 305, 300, 20), "TireRR Weight: " + AxleRear.TireRight.ActiveWeight.ToString());

            GUI.Label(new Rect(5, 325, 300, 20), "TireFL Friction: " + AxleFront.TireLeft.FrictionForce.ToString());
            GUI.Label(new Rect(5, 345, 300, 20), "TireFR Friction: " + AxleFront.TireRight.FrictionForce.ToString());
            GUI.Label(new Rect(5, 365, 300, 20), "TireRL Friction: " + AxleRear.TireLeft.FrictionForce.ToString());
            GUI.Label(new Rect(5, 385, 300, 20), "TireRR Friction: " + AxleRear.TireRight.FrictionForce.ToString());

            GUI.Label(new Rect(5, 405, 300, 20), "TireFL Grip: " + AxleFront.TireLeft.Grip.ToString());
            GUI.Label(new Rect(5, 425, 300, 20), "TireFR Grip: " + AxleFront.TireRight.Grip.ToString());
            GUI.Label(new Rect(5, 445, 300, 20), "TireRL Grip: " + AxleRear.TireLeft.Grip.ToString());
            GUI.Label(new Rect(5, 465, 300, 20), "TireRR Grip: " + AxleRear.TireRight.Grip.ToString());

            GUI.Label(new Rect(5, 485, 300, 20), "AxleF SlipAngle: " + AxleFront.SlipAngle.ToString());
            GUI.Label(new Rect(5, 505, 300, 20), "AxleR SlipAngle: " + AxleRear.SlipAngle.ToString());

            GUI.Label(new Rect(5, 525, 300, 20), "AxleF Torque: " + AxleFront.Torque.ToString());
            GUI.Label(new Rect(5, 545, 300, 20), "AxleR Torque: " + AxleRear.Torque.ToString());
        }
    }

    private void OnDrawGizmosSelected()
    {
        DebugDraw();
    }

    private static void DrawCube(Vector3 pos, Quaternion rotation, float scaling, Color c, float duration = 0.0f)
    {
        var m = new Matrix4x4();
        m.SetTRS(pos, rotation, Vector3.one * scaling);

        var point1 = m.MultiplyPoint(new Vector3(-0.25f, -0.5f, 0.5f));
        var point2 = m.MultiplyPoint(new Vector3(0.25f,  -0.5f, 0.5f));
        var point3 = m.MultiplyPoint(new Vector3(0.25f,  -0.5f, -0.5f));
        var point4 = m.MultiplyPoint(new Vector3(-0.25f, -0.5f, -0.5f));

        var point5 = m.MultiplyPoint(new Vector3(-0.25f, 0.5f, 0.5f));
        var point6 = m.MultiplyPoint(new Vector3(0.25f,  0.5f, 0.5f));
        var point7 = m.MultiplyPoint(new Vector3(0.25f,  0.5f, -0.5f));
        var point8 = m.MultiplyPoint(new Vector3(-0.25f, 0.5f, -0.5f));

        Debug.DrawLine(point1, point2, c, duration, false);
        Debug.DrawLine(point2, point3, c, duration, false);
        Debug.DrawLine(point3, point4, c, duration, false);
        Debug.DrawLine(point4, point1, c, duration, false);

        Debug.DrawLine(point5, point6, c, duration, false);
        Debug.DrawLine(point6, point7, c, duration, false);
        Debug.DrawLine(point7, point8, c, duration, false);
        Debug.DrawLine(point8, point5, c, duration, false);

        Debug.DrawLine(point1, point5, c, duration, false);
        Debug.DrawLine(point2, point6, c, duration, false);
        Debug.DrawLine(point3, point7, c, duration, false);
        Debug.DrawLine(point4, point8, c, duration, false);
    }

    private void DebugDraw()
    {
        if (AxleFront == null) return;

        var rotation = transform.rotation;

        var steeringRotation = Quaternion.AngleAxis(SteerAngle * Mathf.Rad2Deg, Vector3.up);
        var steeringDir      = steeringRotation * Vector3.forward;

        var offset        = Vector3.up * 0.05f;
        var frontPosLeft  = AxleFront.TireLeft.transform.position  + offset;
        var frontPosRight = AxleFront.TireRight.transform.position + offset;
        var rearPosLeft   = AxleRear.TireLeft.transform.position   + offset;
        var rearPosRight  = AxleRear.TireRight.transform.position  + offset;

        DrawCube(frontPosLeft,  rotation * steeringRotation, 0.3f, Color.yellow);
        DrawCube(frontPosRight, rotation * steeringRotation, 0.3f, Color.yellow);
        DrawCube(rearPosLeft,   rotation,                    0.3f, Color.yellow);
        DrawCube(rearPosRight,  rotation,                    0.3f, Color.yellow);

        Debug.DrawLine(frontPosLeft, frontPosLeft + transform.TransformVector(steeringDir) * 0.1f, Color.yellow, 0.0f, false);
        Debug.DrawLine(frontPosRight, frontPosRight + transform.TransformVector(steeringDir) * 0.1f, Color.yellow, 0.0f, false);
        Debug.DrawLine(rearPosLeft, rearPosLeft + transform.forward * 0.1f, Color.yellow, 0.0f, false);
        Debug.DrawLine(rearPosRight, rearPosRight + transform.forward * 0.1f, Color.yellow, 0.0f, false);
    }
}
