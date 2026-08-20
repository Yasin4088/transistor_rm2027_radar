import time


def wait_for_initial_camera_frame(
    *,
    get_frame,
    get_error,
    should_stop,
    report_status,
    logger=print,
    poll_interval=0.5,
    status_interval=1.0,
    log_interval=5.0,
    monotonic=time.monotonic,
    wall_time=time.time,
    sleeper=time.sleep,
):
    """
    Keep the already-started match process alive until a camera frame arrives.

    The camera is optional for process startup: communication and monitoring can
    remain online while the camera worker waits for hardware.  Recording stays
    explicitly disabled throughout this wait and is initialized by the caller
    only after a real frame is returned.
    """
    next_status_at = 0.0
    next_log_at = 0.0

    while not should_stop():
        frame = get_frame()
        if frame is not None:
            return frame

        now = monotonic()
        error = get_error()
        if now >= next_status_at:
            report_status(
                phase="running",
                heartbeat_at=wall_time(),
                camera_ready=False,
                camera_error=error,
                camera_fps=None,
                processing_fps=0.0,
                fps=0.0,
                recording=False,
                recording_suspended_reason="camera_unavailable",
            )
            next_status_at = now + max(float(status_interval), 0.0)

        if now >= next_log_at:
            detail = f"：{error}" if error else ""
            logger(f"相机未连接，主程序保持运行且不会录像{detail}")
            next_log_at = now + max(float(log_interval), 0.0)

        sleeper(max(float(poll_interval), 0.0))

    return None
