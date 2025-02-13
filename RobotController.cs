using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotController : MonoBehaviour
{
    // naming constraints do not change
    [SerializeField] private WheelCollider FLWC;
    [SerializeField] private WheelCollider FRWC;
    [SerializeField] private WheelCollider BLWC;
    [SerializeField] private WheelCollider BRWC;

    [SerializeField] private Transform FLWT;
    [SerializeField] private Transform FRWT;
    [SerializeField] private Transform BLWT;
    [SerializeField] private Transform BRWT;

    [SerializeField] private Transform SFR;
    [SerializeField] private Transform SL1;
    [SerializeField] private Transform SL2;
    [SerializeField] private Transform SL3;
    [SerializeField] private Transform SR1;
    [SerializeField] private Transform SR2;
    [SerializeField] private Transform SR3;
    [SerializeField] private Transform SOR;

    [SerializeField] private float maxSteeringAngle = 50;
    [SerializeField] private float motorForce = 50;
    [SerializeField] private float brakeForce;
    private Rigidbody rb;
    [SerializeField] private float angle_x;
    [SerializeField] private float angle_z;
    [SerializeField] private float CarVelocity;

    private float steerAngle;
    private bool isBreaking;

    private float s1dist = 8;
    private float s2dist = 8;
    private float s3dist = 8;

    private void Start()
    {
        rb = GetComponent<Rigidbody>();
        float s1x = 0; float s1y = 10; float s1z = 0;
        float s2x = 8; float s2y = 30; float s2z = 0;
        float s3x = 16; float s3y = 60; float s3z = 0;

        PositionSensors(SFR, 20, 0, 0);
        PositionSensors(SL1, s1x, -s1y, s1z);
        PositionSensors(SL2, s2x, -s2y, s2z);
        PositionSensors(SL3, s3x, -s3y, s3z);
        PositionSensors(SR1, s1x, s1y, s1z);
        PositionSensors(SR2, s2x, s2y, s2z);
        PositionSensors(SR3, s3x, s3y, s3z);
        PositionSensors(SOR, 50, 180, 0);
    }



    private void PositionSensors(Transform sensor, float x_angle, float y_angle, float z_angle)
    {
        sensor.transform.Rotate(x_angle, y_angle, z_angle);
    }



    private void UpdateWheels()
    {
        UpdateWheelsPosition(FLWC, FLWT);
        UpdateWheelsPosition(FRWC, FRWT);
        UpdateWheelsPosition(BLWC, BLWT);
        UpdateWheelsPosition(BRWC, BRWT);
    }

    private void UpdateWheelsPosition(WheelCollider collider, Transform transform)
    {
        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);
        transform.position = position;
        transform.rotation = rotation;
    }

    private void SteeringControl(float direction)
    {
        steerAngle = direction * maxSteeringAngle;
        FLWC.steerAngle = steerAngle;
        FRWC.steerAngle = steerAngle;
    }

    private bool Sense(Transform sensor, float dist, string layer)
    {
        LayerMask mask = LayerMask.GetMask(layer);
        if (Physics.Raycast(sensor.position, sensor.TransformDirection(Vector3.forward), dist, mask))
        {
            Debug.DrawRay(sensor.position, sensor.TransformDirection(Vector3.forward) * dist, Color.yellow);
            return true;
        }
        else
        {
            Debug.DrawRay(sensor.position, sensor.TransformDirection(Vector3.forward) * dist, Color.white);
            return false;
        }
    }







    private void MaintainTrack()
    {
        if (!Sense(SL3, s3dist, "Road") || !Sense(SR3, s3dist, "Road"))
        {
            if (!Sense(SL3, s3dist, "Road"))
            {
                SteeringControl(1);
            }
            if (!Sense(SR3, s3dist, "Road"))
            {
                SteeringControl(-1);
            }
        }
        else
        {
            SteeringControl(0);
        }
    }

    private void SpeedControl()
    {
        if (CarVelocity < 2 && motorForce <= 50)
        {
            motorForce = motorForce + 540f;
        }
        else if (CarVelocity > 6 && motorForce > 0)
        {
            motorForce = motorForce - 30f;
        }
        else if (CarVelocity > 12 && motorForce < 0)
        {
            motorForce = motorForce - 80f;
        }
    }

    private void ObstaclesAvoidance()
    {
        if (Sense(SL1, s1dist, "Obstacles"))
        {
            SteeringControl(1);
        }
        if (Sense(SR1, s1dist, "Obstacles"))
        {
            SteeringControl(-1);
        }

        if (Sense(SL2, s2dist, "Obstacles"))
        {
            SteeringControl(1);
        }
        if (Sense(SR2, s2dist, "Obstacles"))
        {
            SteeringControl(-1);
        }
    }

    private void FixedUpdate()
    {
        MaintainTrack();
        ObstaclesAvoidance();
        SpeedControl();
        MotorPower();
        UpdateWheels();

        angle_x = SOR.eulerAngles.x;
        angle_z = SOR.eulerAngles.z;

        CarVelocity = rb.velocity.magnitude;
    }

