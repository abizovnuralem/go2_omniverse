# Command time stamp of the latest Cortex command received.
time cortex_command_time

# ID of the latest command received.
int64 cortex_command_id

# If there is a slight accumulative clock rate difference between the Cortex
# commander and the low-level controller, the time offset gives how much
# further ahead the controller's clock is from the Cortex commander's clock (note
# it can be negative). So synchronizing the clocks would entail
#
#    <cortex_commander_time_synced> = <cortex_commander_time> + time_offset
duration time_offset
