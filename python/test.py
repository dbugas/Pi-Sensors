import imupy
import time
from scipy.spatial.transform import Rotation as R

# ------------------ Initialize IMU ------------------
# Choose a performance mode (Ultra, High, Medium, Low)
imu = imupy.IMU(imupy.PerformanceMode.High, Use_Mag=True, Use_Barometer=True)

# Start the background sensor thread (runs continuously)
imu.start_sensor_thread()
imu.update_quat_thread()

# ------------------ Read Data Loop ------------------
gx_a= 0.0
gy_a= 0.0
gz_a = 0.0
num = 100
time.sleep(0.5)
try:
    for i in range(num):  
        # Raw sensor readings
        ax, ay, az = imu.Accel_raw()
        gx, gy, gz = imu.Gyro_raw()
        mx, my, mz = imu.Mag_raw()
        pressure, temp, altitude = imu.Baro_raw()

        quat = imu.GetQuat()
        rot = R.from_quat([float(quat.x), float(quat.y), float(quat.z), float(quat.w)]) 
        qw, qx, qy, qz = quat.w, quat.x, quat.y, quat.z
        gx_a += float(gx)
        gy_a += float(gy)
        gz_a += float(gz) 

        print(f"Accel: ({ax*9.806:.5f}, {ay*9.806:.5f}, {az*9.806:.5f}) m/s^2")
        print(f"Gyro:  ({gx:.5f}, {gy:.5f}, {gz:.5f}) rad/s")
        print(f"Mag:   ({mx:.5f}, {my:.5f}, {mz:.5f}) (unitless)")
        print(f"Baro:  {pressure:.5f} Pa, {temp:.5f} C, {altitude:.5f} m")
        print(f"Quat:  w= {qw:.3f}, x= {qx:.3f}, y= {qy:.3f}, z= {qz:.3f}")
        print("-" * 40)

        time.sleep(0.25)
finally:
    print(f"average gx: {gx_a/num:.8f}, gy: {gy_a/num:.8f}, gz: {gz_a/num:.8f}")
    print("Finished reading IMU data")
