import os
import threading
import tkinter as tk
import functools
import socket

import ui
from colour_monitor import ColourMonitor, ColourTimeoutError
from kinetics_live import LiveKineticsRunner
from traffic_light_experiment import main as run_experiment_logic
from positions_list import PositionList
from utils import ika_handler
from utils.UR_Functions import URfunctions as URControl
from utils.robotiq_gripper import RobotiqGripper
from gripper_helpers import OpenGripper
from aruco_placer import ArucoPlacementSystem


HOST = "192.168.0.2"
PORT = 30003

def main_experiment():
    # create hardware controllers, UI, and run the experiment.
    root_window = tk.Tk()

    # mode selection: aruco/hard-coded coordinates
    mode_selection = {"mode": None}

    root_window.title("Select Operation Mode")
    root_window.resizable(False, False)
    dialog_width, dialog_height = 350, 150
    root_window.update_idletasks()
    sw = root_window.winfo_screenwidth()
    sh = root_window.winfo_screenheight()
    root_window.geometry(f"{dialog_width}x{dialog_height}+{(sw-dialog_width)//2}+{(sh-dialog_height)//2}")

    def select_mode(choice):
        mode_selection["mode"] = choice
        for widget in root_window.winfo_children():
            widget.destroy()
        root_window.quit()  # exit the local mainloop() call below

    tk.Label(root_window, text="Choose an operation mode:", font=("Arial", 12)).pack(pady=10)
    tk.Button(root_window, text="Aruco Mode (Vision-based)",    command=lambda: select_mode('A'), width=30).pack(pady=5)
    tk.Button(root_window, text="Coordinate Mode (Hard-coded)", command=lambda: select_mode('C'), width=30).pack(pady=5)
    root_window.protocol("WM_DELETE_WINDOW", lambda: select_mode(None))

    root_window.mainloop()  # blocking until select_mode() calls quit()

    chosen_mode = mode_selection["mode"]
    if chosen_mode is None:
        print("No mode selected. Exiting.")
        root_window.destroy()
        return

    # Reconfigure root_window for the main experiment UI
    root_window.resizable(True, True)

    # hardware references in a dict so safe_stop can always read the latest
    # assigned value without nonlocal + unbound-variable issues on early crash
    hw = dict(robot=None, gripper=None, ih=None,
              filmMonitor=None, plateMonitor=None,
              display=None, kinetics_runner=None)
    _stop_event = threading.Event()

    def safe_stop(shutdown_ui=False):
        if _stop_event.is_set():
            return  # stop process already initiated
        _stop_event.set()

        d = hw.get('display')
        if d:
            if hasattr(d, 'abort_button'):
                d.abort_button.config(state=tk.DISABLED, text="STOPPING...")
            d.log_event("EMERGENCY STOP - Returning vial and shutting down...")

        def _do_stop():
            try:
                # 1. release monitors immediately to unblock experiment thread
                if hw['filmMonitor']:  hw['filmMonitor'].release()
                if hw['plateMonitor']: hw['plateMonitor'].release()

                # 2. send stop command to UR robot
                if hw['robot']:
                    try:
                        sk = socket.socket()
                        sk.connect((HOST, PORT))
                        sk.send(b"stopj(2.0)\n")
                        sk.close()
                    except Exception:
                        pass

                # 3. return vial and go home
                if hw['robot'] and hw['gripper']:
                    try:
                        if hw['display']: hw['display'].log_event("[SAFESTOP] Returning vial to rack...")
                        hw['robot'].move_joint_list(PositionList.aboveVialRack, 0.2, 0.2, 0.02)
                        hw['robot'].move_joint_list(PositionList.onVialRack,    0.2, 0.2, 0.02)
                        OpenGripper(hw['gripper'])
                        hw['robot'].move_joint_list(PositionList.aboveVialRack, 0.2, 0.2, 0.02)
                    except Exception as e:
                        print(f"Error returning vial: {e}")
                    try:
                        if hw['display']: hw['display'].log_event("[SAFESTOP] Moving to home position...")
                        hw['robot'].move_joint_list(PositionList.defaultPos, 0.2, 0.2, 0.02)
                    except Exception as e:
                        print(f"Error moving to home: {e}")
                else:
                    print("Robot or gripper not initialized, skipping return to home.")

                if hw['ih']:
                    try:
                        hw['ih'].stopStirring()
                    except Exception as e:
                        print(f"Error stopping stirrer: {e}")

                if hw['display']:
                    hw['display'].stop_updates()
                    hw['display'].log_event("Experiment stopped. UI is now static. You can export graphs or close the window.")

                print("Stopped safely.")

            except Exception as e:
                print(f"An error occurred during safe_stop: {e}")
            finally:
                if shutdown_ui:
                    def _destroy():
                        try: root_window.destroy()
                        except Exception: pass
                    try:
                        root_window.after(0, _destroy)
                    except RuntimeError:
                        _destroy()

        threading.Thread(target=_do_stop, daemon=False).start()

    try:
        # hardware + UI setup
        hw['robot'] = URControl(ip=HOST, port=PORT)
        hw['gripper'] = RobotiqGripper()
        hw['gripper'].connect(HOST, 63352)
        hw['ih'] = ika_handler.IKAHandler()
        aruco = ArucoPlacementSystem(hw['robot'], hw['gripper'])

        hw['filmMonitor']  = ColourMonitor(camera_index=0, show_preview=False, monitor_id=1, monitor_name="Filming camera")
        hw['plateMonitor'] = ColourMonitor(camera_index=2, show_preview=False, monitor_id=2, monitor_name="Plate camera")

        hw['kinetics_runner'] = LiveKineticsRunner(monitor1=hw['filmMonitor'], monitor2=hw['plateMonitor'])
        hw['kinetics_runner'].start()

        hw['display'] = ui.UI(root_window, hw['filmMonitor'], hw['plateMonitor'],
                              abort_callback=safe_stop, kinetics_runner=hw['kinetics_runner'])
        hw['filmMonitor'].status_callback  = hw['display'].queue_monitor_update
        hw['plateMonitor'].status_callback = hw['display'].queue_monitor_update

        # when window closed, stop everything and shut down the UI
        root_window.protocol("WM_DELETE_WINDOW", lambda: safe_stop(shutdown_ui=True))

        # threading
        def on_experiment_complete():
            """Called from experiment thread on normal completion."""
            if hw['display']:
                hw['display'].log_event("Experiment complete. UI is now static.")
                hw['display'].stop_updates()

        # after_idle to ensure UI calls are made from the main thread
        on_complete_callback = lambda: root_window.after_idle(on_experiment_complete)

        experiment_args = (hw['display'], hw['robot'], hw['gripper'], hw['ih'],
                           hw['filmMonitor'], hw['plateMonitor'], aruco, chosen_mode,
                           on_complete_callback)

        experiment_thread = threading.Thread(target=run_experiment_logic, args=experiment_args, daemon=False)
        experiment_thread.start()

        # run UI in main thread (this blocks until UI is closed)
        root_window.mainloop()

        # wait for experiment thread to finish
        if experiment_thread.is_alive():
            experiment_thread.join()

    # error + exception handlingsc
    except ColourTimeoutError as e:
        if hw['display']: hw['display'].log_event(f"[EXPERIMENT] ERROR! COLOUR TIMEOUT: {e}")
        if hw['display']: hw['display'].log_event("[EXPERIMENT] Reaction may have stalled. Stopping safely.")
        safe_stop()
    except (KeyboardInterrupt, SystemExit):
        if hw['display']: hw['display'].log_event("[EXPERIMENT] Interrupted by user. Stopping safely.")
        safe_stop()
    except Exception as e:
        print(f"An unhandled exception occurred in main_experiment: {e}")
        if hw['display']: hw['display'].log_event(f"FATAL ERROR: {e}")
        safe_stop()
    finally:
        _stop_event.set()
        if hw['filmMonitor']:     hw['filmMonitor'].release()
        if hw['plateMonitor']:    hw['plateMonitor'].release()
        if hw['kinetics_runner']: hw['kinetics_runner'].stop()
        if hw['ih']: del hw['ih']
        print("Experiment finished.")

if __name__ == '__main__':
    main_experiment()