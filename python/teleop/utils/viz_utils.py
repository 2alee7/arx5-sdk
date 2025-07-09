import cv2
import time
import numpy as np

def display_camera_views(latest_frames, frames_queue, recording_event, stop_event):
    # TODO: Not working, poorly written
    # Needs to pull frames from a shared memory queue asynchronously from saving thread
    cv2.namedWindow("Camera Views", cv2.WINDOW_NORMAL)
    while not stop_event.is_set():
        frames = []
        for i in range(4):
            frame = latest_frames.get(i, np.zeros((480, 848, 3), dtype=np.uint8))
            if frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            frames.append(frame)

        row1 = cv2.hconcat([frames[0], frames[1]])
        row2 = cv2.hconcat([frames[3], frames[2]])
        composite = cv2.vconcat([row1, row2])

        labels = ["top_vew", "front_view", "wrist_left", "wrist_right"]
        positions = [(0, 480), (848, 480), (0, 960), (848, 960)]
        for label, pos in zip(labels, positions):
            cv2.putText(composite, label, pos, cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)

        cv2.imshow("Camera Views", composite)
        # key = cv2.waitKey(1) & 0xFF
        # if key == 32:
        #     if recording_event.is_set():
        #         print("Recording stopped...")
        #         recording_event.clear()
        #     else:
        #         print("Recording started...")
        #         recording_event.set()
        # elif key == 27:
        #     stop_event.set()
        #     break

        # if not recording_event.is_set() and not frames_queue.empty():
        #     print("Recording session ended. Press 's' to save or any other key to discard.")
        #     key = cv2.waitKey(0) & 0xFF
        #     traj_path = get_next_traj_folder("observations")
        #     if key == ord('s'):
        #         save_frames_and_metadata(frames_queue, traj_path)
        #         print(f"Recording saved as {traj_path}.")
        #     else:
        #         while not frames_queue.empty():
        #             frames_queue.get()
        #         print("Recording discarded.")

    time.sleep(0.1)