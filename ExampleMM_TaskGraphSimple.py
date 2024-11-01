# THIS COMMENT LINE SHOULD BE THE FIRST LINE OF THE FILE
# DON'T CHANGE ANY OF THE BELOW; NECESSARY FOR JOINING SIMULATION
import os, sys, time, datetime, traceback
import spaceteams as st
def custom_exception_handler(exctype, value, tb):
    error_message = "".join(traceback.format_exception(exctype, value, tb))
    st.logger_fatal(error_message)
    exit(1)
sys.excepthook = custom_exception_handler
st.connect_to_sim(sys.argv)
import numpy as np
# DON'T CHANGE ANY OF THE ABOVE; NECESSARY FOR JOINING SIMULATION
#################################################################

from API.STU_Common import *
import API.MissionManagerFuncs as MM
mm = MM.MissionManager()
import API.EntityTelemetry as ET
import TaskGraph as TG

#############################
##  Mission Manager Setup  ##
#############################

# Get the main entity (LTV1) for the search
entities = st.GetThisSystem().GetParamArray(st.VarType.entityRef, "Entities")
LTV1: st.Entity = entities[0]

LTV1_task_graph = TG.TaskGraph()

###########################
##  Command Definitions  ##
###########################

def LTV1_TaskComplete(payload : st.ParamMap):
    LTV1_task_graph.mark_completed(payload.GetParam(st.VarType.string, ["Orig_Cmd", "TaskID"]))

def LTV1_TaskFail(payload : st.ParamMap):
    LTV1_task_graph.mark_failed(payload.GetParam(st.VarType.string, ["Orig_Cmd", "TaskID"]))

def MoveToCoord_LTV1_Complete(payload : st.ParamMap):
    st.OnScreenLogMessage("MoveToCoord command complete.", "Surface Movement", st.Severity.Info)
    LTV1_TaskComplete(payload)
mm.OnCommandComplete(LTV1, "MoveToCoord", MoveToCoord_LTV1_Complete)

def MoveToCoord_LTV1_Failed(payload : st.ParamMap):
    st.OnScreenLogMessage("MoveToCoord command failed.", "Surface Movement", st.Severity.Info)
    LTV1_TaskFail(payload)
mm.OnCommandFail(LTV1, "MoveToCoord", MoveToCoord_LTV1_Failed)

################################
##  Define Search Waypoints   ##
################################

# Set initial coordinates and define an expanding search pattern
xy_LTV1, had_comms = ET.GetCurrentXY(LTV1)

# Define a spiral search pattern to cover the vicinity of the crash site
initial_distance = 10  # Smaller initial step for localized search
waypoints = [
    XY(xy_LTV1.x + initial_distance, xy_LTV1.y),
    XY(xy_LTV1.x, xy_LTV1.y + initial_distance),
    XY(xy_LTV1.x - initial_distance, xy_LTV1.y),
    XY(xy_LTV1.x, xy_LTV1.y - initial_distance),
    XY(xy_LTV1.x + initial_distance * 2, xy_LTV1.y),
    XY(xy_LTV1.x, xy_LTV1.y + initial_distance * 2),
    XY(xy_LTV1.x - initial_distance * 2, xy_LTV1.y),
    XY(xy_LTV1.x, xy_LTV1.y - initial_distance * 2),
    # Add more waypoints as necessary to create a full spiral/grid pattern
]

# Add each waypoint as a task in the task graph
for i, waypoint in enumerate(waypoints):
    task_id = f"Move{i+1}"
    task = TG.Task(task_id, Command_MoveToCoord(LTV1, waypoint, task_id))
    dependencies = [f"Move{i}"] if i > 0 else []
    LTV1_task_graph.add_task(task, dependencies)

#################################
##  Simulation Initialization  ##
#################################

time.sleep(5.0)
st.OnScreenLogMessage("Starting LTV1 movement.", "Mission Manager", st.Severity.Info)

#######################
##  Simulation Loop  ##
#######################

exit_flag = False
while not exit_flag:
    LoopFreqHz = st.GetThisSystem().GetParam(st.VarType.double, "LoopFreqHz")
    time.sleep(1.0 / LoopFreqHz)

    # Check for crash site proximity using LIDAR
    _, radii, had_comms = ET.GetLidarObstacles(LTV1)
    if had_comms and any(radius <= 20 for radius in radii):
        st.OnScreenLogMessage("Crash site detected within 20m radius. Stopping rover.", "Mission Manager", st.Severity.Info)
        exit_flag = True
        continue

    # Start all unstarted tasks
    for task_id in LTV1_task_graph.pending_tasks:
        task = LTV1_task_graph.get_task(task_id)
        if not task.started:
            received = mm.SendCommand(LTV1, task.command.command_type, task.command)
            if received:
                task.started = True

st.leave_sim()
