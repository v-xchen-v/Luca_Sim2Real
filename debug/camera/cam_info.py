import pyrealsense2 as rs

def print_device_info():
    ctx = rs.context()
    for device in ctx.query_devices():
        print(f"Device: {device.get_info(rs.camera_info.name)}")
        for sensor in device.query_sensors():
            print(f"Sensor: {sensor.get_info(rs.camera_info.name)}")
            for profile in sensor.get_stream_profiles():
                print(f" - Stream Type: {profile.stream_type()} | Format: {profile.format()} | "
                      f"Resolution: {profile.as_video_stream_profile().width()}x{profile.as_video_stream_profile().height()} "
                      f"| FPS: {profile.fps()}")

print_device_info()
""" 
 - Stream Type: stream.infrared | Format: format.uyvy | Resolution: 1280x720 | FPS: 5
 - Stream Type: stream.infrared | Format: format.bgra8 | Resolution: 1280x720 | FPS: 5
 - Stream Type: stream.infrared | Format: format.rgba8 | Resolution: 1280x720 | FPS: 5
 - Stream Type: stream.infrared | Format: format.bgr8 | Resolution: 1280x720 | FPS: 5
 - Stream Type: stream.infrared | Format: format.rgb8 | Resolution: 1280x720 | FPS: 5
  - Stream Type: stream.depth | Format: format.z16 | Resolution: 1280x720 | FPS: 5
 - Stream Type: stream.depth | Format: format.z16 | Resolution: 848x480 | FPS: 10
 - Stream Type: stream.depth | Format: format.z16 | Resolution: 848x480 | FPS: 5
 - Stream Type: stream.depth | Format: format.z16 | Resolution: 640x480 | FPS: 30
 - Stream Type: stream.depth | Format: format.z16 | Resolution: 640x480 | FPS: 15
 - Stream Type: stream.depth | Format: format.z16 | Resolution: 640x480 | FPS: 5
 - Stream Type: stream.depth | Format: format.z16 | Resolution: 640x360 | FPS: 30
 - Stream Type: stream.depth | Format: format.z16 | Resolution: 480x270 | FPS: 60
 - Stream Type: stream.depth | Format: format.z16 | Resolution: 480x270 | FPS: 30
 - Stream Type: stream.depth | Format: format.z16 | Resolution: 480x270 | FPS: 15
 - Stream Type: stream.depth | Format: format.z16 | Resolution: 480x270 | FPS: 5
 - Stream Type: stream.depth | Format: format.z16 | Resolution: 256x144 | FPS: 90
 Sensor: RGB Camera
 - Stream Type: stream.color | Format: format.rgb8 | Resolution: 1280x800 | FPS: 8
 - Stream Type: stream.color | Format: format.y8 | Resolution: 1280x800 | FPS: 8
 - Stream Type: stream.color | Format: format.bgra8 | Resolution: 1280x800 | FPS: 8
 - Stream Type: stream.color | Format: format.rgba8 | Resolution: 1280x800 | FPS: 8
 - Stream Type: stream.color | Format: format.bgr8 | Resolution: 1280x800 | FPS: 8
 - Stream Type: stream.color | Format: format.yuyv | Resolution: 1280x800 | FPS: 8
 - Stream Type: stream.color | Format: format.rgb8 | Resolution: 1280x720 | FPS: 15
 - Stream Type: stream.color | Format: format.rgb8 | Resolution: 1280x720 | FPS: 10
 - Stream Type: stream.color | Format: format.rgb8 | Resolution: 1280x720 | FPS: 5
 - Stream Type: stream.color | Format: format.y8 | Resolution: 1280x720 | FPS: 15
 - Stream Type: stream.color | Format: format.y8 | Resolution: 1280x720 | FPS: 10
 - Stream Type: stream.color | Format: format.y8 | Resolution: 1280x720 | FPS: 5"""