# Basic joint position, velocity, and feed-forward acceleration command.
# Typically, q and qd are used for PID control, and qdd is used by inverse
# dynamics to compute a feed forward term.
#
# These message contain enough information to reconstruct the specific integral
# curve on the receiving side using quintic polynomial interpolation
# (rectifying the messages against jitter). See CommandStreamInterpolator.

# Contains the wall-clock time stamp (unless otherwise specified explicitly
# during construction of the Cortex commander).
std_msgs/Header header

# id's increment by one each, and period gives the amount of time between the
# the previous message (with message ID id-1) and this
int64 id
duration period

# This time stamp is the exact controller time in the sense that
#
#   msg[id+1].t - msg[id].t = msg[id+1].period
#
# The header gives the wall-clock time at publication (unless otherwise
# specified during initialization of the Cortex commander) so we can observe any
# jitter in plotters such as rqt_plot which read the header time stamps to
# rectify the incoming messages.
time t

string[] names
float64[] q
float64[] qd
float64[] qdd

#SymmetricMatrix32 metric
