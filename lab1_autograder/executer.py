import os
import time
import rclpy
from rclpy.node import Node
import sys
from datetime import datetime, timezone

class GPSEvaluator(Node):
    def __init__(self):
        super().__init__('evaluator')

DATA = "BREAD"
argv = sys.argv
port = str(argv[2])
callback_received = False

def callback(data):
    global DATA, callback_received
    callback_received = True
    DATA = data


if __name__ == "__main__":
    rclpy.init(args=argv)
    evaluator = GPSEvaluator()
    workspace_dir = str(argv[1])
    print(f"\nWorkspace: {workspace_dir}")
    
    home_files = os.listdir(workspace_dir)
    assert "src" in home_files, "No src folder found"
    
    src_dir = os.path.join(workspace_dir, "src/")
    files = os.listdir(src_dir)
    print(f"Packages found: {files}")

    # Detect package structure
    structure_type = None
    package = None
    
    # Check for new structure: custom_msgs + gps_driver (Python-based)
    if "custom_msgs" in files and "gps_driver" in files:
        gps_pkg_dir = os.path.join(src_dir, "gps_driver/")
        if os.path.exists(os.path.join(gps_pkg_dir, "setup.py")):
            structure_type = "new"
            package = "gps_driver"
            print("Structure: Two-package (custom_msgs + gps_driver with setup.py)")
    
    # Check for old structure: gps_driver with CMakeLists.txt containing messages
    if structure_type is None:
        if "gps_driver" in files or "gps-driver" in files:
            package = "gps_driver" if "gps_driver" in files else "gps-driver"
            gps_pkg_dir = os.path.join(src_dir, package + "/")
            if os.path.exists(os.path.join(gps_pkg_dir, "CMakeLists.txt")):
                if os.path.exists(os.path.join(gps_pkg_dir, "msg/")):
                    structure_type = "old"
                    print("Structure: Single-package (CMake-based with msg/)")

    assert structure_type is not None, "Could not detect valid package structure. Expected either:\n  - Old: gps_driver/ with CMakeLists.txt and msg/\n  - New: custom_msgs/ + gps_driver/ with setup.py"

    package_dir = os.path.join(src_dir, package + "/")

    # Validate structure based on type
    if structure_type == "old":
        assert "msg" in os.listdir(package_dir), "Missing msg/ folder in gps_driver"
        assert "launch" in os.listdir(package_dir), "Missing launch/ folder in gps_driver"
        assert "gps_driver" in os.listdir(package_dir), "Missing gps_driver/ Python folder"
        assert "Customgps.msg" in os.listdir(package_dir + "msg/"), "Missing Customgps.msg"
        assert "standalone_driver.py" in os.listdir(package_dir + "gps_driver/"), "Missing standalone_driver.py"
    else:  # new structure
        custom_msgs_dir = os.path.join(src_dir, "custom_msgs/")
        assert os.path.exists(custom_msgs_dir), "Missing custom_msgs package"
        assert "msg" in os.listdir(custom_msgs_dir), "Missing msg/ folder in custom_msgs"
        assert "Customgps.msg" in os.listdir(custom_msgs_dir + "msg/"), "Missing Customgps.msg in custom_msgs"
        assert "launch" in os.listdir(package_dir), "Missing launch/ folder in gps_driver"
        assert "gps_driver" in os.listdir(package_dir), "Missing gps_driver/ Python folder"
        assert "standalone_driver.py" in os.listdir(package_dir + "gps_driver/"), "Missing standalone_driver.py"

    # Find launch file
    launch_files = os.listdir(package_dir + "launch/")
    launch_file = None
    if "standalone_driver.launch.py" in launch_files:
        launch_file = "standalone_driver.launch.py"
    elif "standalone_driver.launch" in launch_files:
        launch_file = "standalone_driver.launch"
    assert launch_file is not None, "Missing standalone_driver.launch.py or standalone_driver.launch"

    # Build moved to script.sh (with source install/setup.bash)

    # Launch node
    os.system('screen -S ros_node -dm bash -c "source ' + workspace_dir + 'install/setup.bash && ros2 launch ' + package + ' ' + launch_file + ' port:=' + port + '"')
    print("\nStarting ROS node...")
    time.sleep(3)

    # Import message
    try:
        if structure_type == "new":
            from custom_msgs.msg import Customgps
        else:
            from gps_driver.msg import Customgps
    except:
        try:
            from custom_msgs.msg import Customgps
        except:
            try:
                from gps_driver.msg import Customgps
            except:
                assert False, "Unable to import Customgps.msg. Have you sourced install/setup.bash?"

    # Subscribe and wait for message
    sub = evaluator.create_subscription(Customgps, "/gps", callback, 10)
    cur_time = time.time()

    print("Waiting for /gps topic", end="", flush=True)
    dots = 0
    while isinstance(DATA, str) and time.time() - cur_time < 30:
        rclpy.spin_once(evaluator, timeout_sec=0.5)
        if dots < 10:
            print(".", end="", flush=True)
            dots += 1

    print()
    if isinstance(DATA, str):
        print("\n[FAIL] Not publishing over topic /gps")
        os.system("screen -S ros_node -X quit")
        sys.exit(1)

    # Expected values
    time_ = int(datetime.combine(datetime.now(timezone.utc).date(), 
                datetime.strptime("02:34:58", "%H:%M:%S").time(), timezone.utc).timestamp())
    expected = {
        "lat": 34.02019816666667,
        "lon": -118.41129950000001,
        "easting": 369695.4373543182,
        "northing": 3765293.4953880184,
        "zone": 11,
        "letter": "S",
        "altitude": 0.0,
        "nsec": int(0.23 * 10**9)
    }

    # Grading
    grades = {
        "gpgga_parsing": {"passed": True, "details": []},
        "lat_lon_decimal": {"passed": True, "details": []},
        "utm_conversion": {"passed": True, "details": []},
        "time_handling": {"passed": True, "details": []},
        "message_structure": {"passed": True, "details": []}
    }

    # 1. GPGGA Parsing
    try:
        if hasattr(DATA, 'gpgga_read') and '$GPGGA' in DATA.gpgga_read:
            grades["gpgga_parsing"]["details"].append("[OK] GPGGA string captured")
        else:
            grades["gpgga_parsing"]["passed"] = False
            grades["gpgga_parsing"]["details"].append("[FAIL] GPGGA string not found")
    except:
        grades["gpgga_parsing"]["passed"] = False
        grades["gpgga_parsing"]["details"].append("[FAIL] gpgga_read field missing")

    # 2. Lat/Lon Conversion
    try:
        if abs(DATA.latitude - expected["lat"]) <= 0.00001:
            grades["lat_lon_decimal"]["details"].append(f"[OK] Latitude: {DATA.latitude:.8f}")
        else:
            diff = abs(DATA.latitude - expected["lat"])
            grades["lat_lon_decimal"]["passed"] = False
            grades["lat_lon_decimal"]["details"].append(f"[FAIL] Latitude: {DATA.latitude:.8f} (expected: {expected['lat']:.8f}, diff: {diff:.8f})")
    except:
        grades["lat_lon_decimal"]["passed"] = False
        grades["lat_lon_decimal"]["details"].append("[FAIL] latitude field missing")

    try:
        if abs(DATA.longitude - expected["lon"]) <= 0.00001:
            grades["lat_lon_decimal"]["details"].append(f"[OK] Longitude: {DATA.longitude:.8f}")
        else:
            diff = abs(DATA.longitude - expected["lon"])
            grades["lat_lon_decimal"]["passed"] = False
            grades["lat_lon_decimal"]["details"].append(f"[FAIL] Longitude: {DATA.longitude:.8f} (expected: {expected['lon']:.8f}, diff: {diff:.8f})")
    except:
        grades["lat_lon_decimal"]["passed"] = False
        grades["lat_lon_decimal"]["details"].append("[FAIL] longitude field missing")

    # 3. UTM Conversion
    try:
        if abs(DATA.utm_easting - expected["easting"]) <= 1:
            grades["utm_conversion"]["details"].append(f"[OK] Easting: {DATA.utm_easting:.2f}")
        else:
            diff = abs(DATA.utm_easting - expected["easting"])
            grades["utm_conversion"]["passed"] = False
            grades["utm_conversion"]["details"].append(f"[FAIL] Easting: {DATA.utm_easting:.2f} (expected: {expected['easting']:.2f}, diff: {diff:.2f}m)")
    except:
        grades["utm_conversion"]["passed"] = False
        grades["utm_conversion"]["details"].append("[FAIL] utm_easting missing")

    try:
        if abs(DATA.utm_northing - expected["northing"]) <= 1:
            grades["utm_conversion"]["details"].append(f"[OK] Northing: {DATA.utm_northing:.2f}")
        else:
            diff = abs(DATA.utm_northing - expected["northing"])
            grades["utm_conversion"]["passed"] = False
            grades["utm_conversion"]["details"].append(f"[FAIL] Northing: {DATA.utm_northing:.2f} (expected: {expected['northing']:.2f}, diff: {diff:.2f}m)")
    except:
        grades["utm_conversion"]["passed"] = False
        grades["utm_conversion"]["details"].append("[FAIL] utm_northing missing")

    try:
        if DATA.zone == expected["zone"]:
            grades["utm_conversion"]["details"].append(f"[OK] Zone: {DATA.zone}")
        else:
            grades["utm_conversion"]["passed"] = False
            grades["utm_conversion"]["details"].append(f"[FAIL] Zone: {DATA.zone} (expected: {expected['zone']})")
    except:
        grades["utm_conversion"]["passed"] = False
        grades["utm_conversion"]["details"].append("[FAIL] zone missing")

    try:
        if DATA.letter == expected["letter"]:
            grades["utm_conversion"]["details"].append(f"[OK] Letter: {DATA.letter}")
        else:
            grades["utm_conversion"]["passed"] = False
            grades["utm_conversion"]["details"].append(f"[FAIL] Letter: {DATA.letter} (expected: {expected['letter']})")
    except:
        grades["utm_conversion"]["passed"] = False
        grades["utm_conversion"]["details"].append("[FAIL] letter missing")

    # 4. Time Handling
    try:
        if DATA.header.stamp.sec == time_:
            grades["time_handling"]["details"].append(f"[OK] Epoch sec: {DATA.header.stamp.sec}")
        else:
            diff = abs(DATA.header.stamp.sec - time_)
            grades["time_handling"]["passed"] = False
            grades["time_handling"]["details"].append(f"[FAIL] Epoch sec: {DATA.header.stamp.sec} (expected: {time_}, diff: {diff}s)")
    except:
        grades["time_handling"]["passed"] = False
        grades["time_handling"]["details"].append("[FAIL] stamp.sec missing")

    try:
        nsec_tolerance = int(expected["nsec"] * 0.01)
        if abs(DATA.header.stamp.nanosec - expected["nsec"]) <= nsec_tolerance:
            grades["time_handling"]["details"].append(f"[OK] Nanosec: {DATA.header.stamp.nanosec}")
        else:
            diff = abs(DATA.header.stamp.nanosec - expected["nsec"])
            grades["time_handling"]["passed"] = False
            grades["time_handling"]["details"].append(f"[FAIL] Nanosec: {DATA.header.stamp.nanosec} (expected: {expected['nsec']}, diff: {diff}ns)")
    except:
        grades["time_handling"]["passed"] = False
        grades["time_handling"]["details"].append("[FAIL] stamp.nanosec missing")

    # 5. Message Structure
    try:
        if DATA.header.frame_id.upper() == "GPS1_FRAME":
            grades["message_structure"]["details"].append(f"[OK] Frame ID: {DATA.header.frame_id}")
        else:
            grades["message_structure"]["passed"] = False
            grades["message_structure"]["details"].append(f"[FAIL] Frame ID: {DATA.header.frame_id} (expected: GPS1_Frame)")
    except:
        grades["message_structure"]["passed"] = False
        grades["message_structure"]["details"].append("[FAIL] frame_id missing")

    try:
        if DATA.altitude == expected["altitude"]:
            grades["message_structure"]["details"].append(f"[OK] Altitude: {DATA.altitude}")
        else:
            grades["message_structure"]["passed"] = False
            grades["message_structure"]["details"].append(f"[FAIL] Altitude: {DATA.altitude} (expected: {expected['altitude']})")
    except:
        grades["message_structure"]["passed"] = False
        grades["message_structure"]["details"].append("[FAIL] altitude missing")

    grades["message_structure"]["details"].append("[OK] Publishing to /gps")

    # Cleanup
    os.system("screen -S ros_node -X quit")

    # Print Results
    print("\n" + "=" * 70)
    print("LAB 1 DRIVER GRADING RESULTS")
    print("=" * 70)

    rubric = [
        ("1. GPGGA String Parsing", "gpgga_parsing", 4),
        ("2. Lat/Lon to Decimal", "lat_lon_decimal", 4),
        ("3. Decimal to UTM", "utm_conversion", 4),
        ("4. Time Handling", "time_handling", 4),
        ("5. ROS Message Structure", "message_structure", 4),
    ]

    total_passed = 0
    total_points = 0

    for name, key, points in rubric:
        result = grades[key]
        status = "PASS" if result["passed"] else "FAIL"
        total_points += points
        if result["passed"]:
            total_passed += points
        
        print(f"\n{name} ({points} pts): [{status}]")
        print("-" * 50)
        for detail in result["details"]:
            print(f"   {detail}")

    print("\n" + "=" * 70)
    print(f"DRIVER SCORE: {total_passed}/{total_points} points")
    if total_passed == total_points:
        print("Perfect score!")
    elif total_passed >= total_points * 0.8:
        print("Good job! Minor issues.")
    else:
        print("Some requirements need attention.")
    print("=" * 70)

    sys.exit(0)
