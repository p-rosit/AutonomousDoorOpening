import pyrealsense2 as rs

def main():
    pipeline = rs.pipeline()
    config = rs.config()

    wrapper = rs.pipeline_wrapper(pipeline)
    wrapper_profile = config.resolve(wrapper)

    device = wrapper_profile.get_device()

    product_line = str(device.get_info(rs.camera_info.product_line))
    
    config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)

    profile = pipeline.start(config)

    for s in profile.get_streams():
        if s.stream_name() == 'Color':
            v = s.as_video_stream_profile()
            print(v.get_intrinsics())

if __name__ == '__main__':
    main()