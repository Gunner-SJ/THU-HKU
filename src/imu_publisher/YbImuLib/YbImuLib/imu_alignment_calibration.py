import time
import sys
import numpy as np
from scipy.spatial.transform import Rotation

try:
    from YbImuLib import YbImuSerial
except ImportError:
    print("[ERROR] Could not import YbImuLib. Ensure it is in your PYTHONPATH.")
    sys.exit(1)

# FIXED: Changed to match your working port
SERIAL_PORT = '/dev/ttyUSB0'  
SAMPLE_RATE = 0.02            # 50 Hz

def collect_gyro_data(imu, duration_sec, prompt_text):
    """Logs gyro data for a specified duration."""
    print(f"\n{prompt_text}")
    for i in range(3, 0, -1):
        print(f"Starting in {i}...")
        time.sleep(1)
        
    print(">>> RECORDING DATA. PERFORM ACTION NOW! <<<")
    end_time = time.time() + duration_sec
    data = []
    
    while time.time() < end_time:
        # Fetching data using the exact method from your working script
        try:
            gx, gy, gz = imu.get_gyroscope_data()
            data.append([float(gx), float(gy), float(gz)])
        except Exception as e:
            print(f"Warning: Dropped frame or read error: {e}")
            
        time.sleep(SAMPLE_RATE)
        
    print(">>> RECORDING COMPLETE <<<")
    return np.array(data)

def extract_principal_axis(data_array, expected_nominal_axis):
    """Uses SVD to find the dominant axis of rotation during an isolated movement."""
    # Mean-center the data
    centered_data = data_array - np.mean(data_array, axis=0)
    
    # SVD to find principal component
    U, S, Vt = np.linalg.svd(centered_data, full_matrices=False)
    dominant_vector = Vt[0]
    
    # SVD sign ambiguity correction: ensure it generally points in the expected direction
    if np.dot(dominant_vector, expected_nominal_axis) < 0:
        dominant_vector = -dominant_vector
        
    return dominant_vector

def main():
    print("=== Bicycle IMU Alignment Calibration ===")
    
    try:
        imu = YbImuSerial(SERIAL_PORT, debug=False)
        imu.create_receive_threading()
        time.sleep(1.0) # Wait for thread to stabilize
        print(f"Successfully connected to IMU on {SERIAL_PORT}")
    except Exception as e:
        print(f"Failed to connect to IMU on {SERIAL_PORT}: {e}")
        sys.exit(1)

    # ---------------------------------------------------------
    # Phase 1: Static Bias Calibration
    # ---------------------------------------------------------
    static_data = collect_gyro_data(
        imu, 
        duration_sec=5.0, 
        prompt_text="PHASE 1/3: Keep the bicycle perfectly STILL and UPRIGHT to measure Gyro Bias."
    )
    
    gyro_bias_s_rad = np.mean(static_data, axis=0)
    gyro_bias_s_dps = np.rad2deg(gyro_bias_s_rad)

    # ---------------------------------------------------------
    # Phase 2: Pure Roll (Bike X-Axis)
    # ---------------------------------------------------------
    roll_data = collect_gyro_data(
        imu, 
        duration_sec=8.0, 
        prompt_text="PHASE 2/3: Continuously rock the bike LEFT AND RIGHT (Pure Roll) around its wheels."
    )
    
    roll_data_unbiased = roll_data - gyro_bias_s_rad
    sensor_x_axis = extract_principal_axis(roll_data_unbiased, expected_nominal_axis=[1, 0, 0])

    # ---------------------------------------------------------
    # Phase 3: Pure Pitch (Bike Y-Axis)
    # ---------------------------------------------------------
    pitch_data = collect_gyro_data(
        imu, 
        duration_sec=8.0, 
        prompt_text="PHASE 3/3: Continuously lift and lower the FRONT WHEEL (Pure Pitch)."
    )
    
    pitch_data_unbiased = pitch_data - gyro_bias_s_rad
    sensor_y_axis = extract_principal_axis(pitch_data_unbiased, expected_nominal_axis=[0, 1, 0])

    # ---------------------------------------------------------
    # Phase 4: Solve Wahba Problem (align_vectors)
    # ---------------------------------------------------------
    bike_x_axis = np.array([1.0, 0.0, 0.0])
    bike_y_axis = np.array([0.0, 1.0, 0.0])

    print("\nCalculating R_BS mounting rotation matrix...")
    
    sensor_to_bike_est, rssd = Rotation.align_vectors(
        [bike_x_axis, bike_y_axis],         # Destination frame (Bike)
        [sensor_x_axis, sensor_y_axis]      # Source frame (Sensor)
    )
    
    yaw, pitch, roll = sensor_to_bike_est.as_euler("ZYX", degrees=True)
    
    print("\n" + "="*50)
    print("🎉 CALIBRATION COMPLETE! COPY THESE INTO YOUR ROS 2 CONFIG:")
    print("="*50)
    
    print(f"\n1. gyro_bias_sensor_dps:")
    print(f"   [{gyro_bias_s_dps[0]:.4f}, {gyro_bias_s_dps[1]:.4f}, {gyro_bias_s_dps[2]:.4f}]")
    print(f"   (Internal Note: Subtracted from raw IMU data BEFORE rotation)")
    
    print(f"\n2. sensor_to_bike_rpy_deg:")
    print(f"   [{roll:.4f}, {pitch:.4f}, {yaw:.4f}]")
    print(f"   (Internal Note: Fit Error RSSD = {rssd:.4f})")
    print("="*50 + "\n")

if __name__ == '__main__':
    main()