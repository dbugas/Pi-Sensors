import imupy
import time

# ------------------ Initialize IMU ------------------
# Choose a performance mode (Ultra, High, Medium, Low)
imu = imupy.IMU(imupy.PerformanceMode.High, Use_Mag=True, Use_Barometer=True)

# Start the background sensor thread (runs continuously)
imu.start_sensor_thread()
imu.update_quat_thread()

# ------------------ Read Data Loop ------------------
try:
    for i in range(10):  # read 10 times
        # Raw sensor readings
        ax, ay, az = imu.Accel_raw()
        gx, gy, gz = imu.Gyro_raw()
        mx, my, mz = imu.Mag_raw()
        pressure, temp, altitude = imu.Baro_raw()

        quat = imu.GetQuat()
        qw, qx, qy, qz = quat.w, quat.x, quat.y, quat.z

        # Print nicely
        print(f"Accel: ({ax*9.81:.5f}, {ay*9.81:.5f}, {az*9.81:.5f}) m/s^2")
        print(f"Gyro:  ({gx:.5f}, {gy:.5f}, {gz:.5f}) rad/s")
        print(f"Mag:   ({mx:.5f}, {my:.5f}, {mz:.5f}) (unitless)")
        print(f"Baro:  {pressure:.5f} Pa, {temp:.5f} C, {altitude:.5f} m")
        print(f"Quat:  w={qw:.3f}, x={qx:.3f}, y={qy:.3f}, z={qz:.3f}")
        print("-" * 40)

        time.sleep(0.5)

finally:
    print("Finished reading IMU data")
