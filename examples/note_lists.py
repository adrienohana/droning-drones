    
import numpy as np
HOVER_THRUST = 38726.7
MIN_THRUST = 20000
MAX_THRUST = 65535

QUAVER = 0.4
BLACK = 2*QUAVER
WHITE = 2*BLACK

major_up_notes = [38726.7, 48195.19271581488, 60055.49312577782]
major_down_notes = [38726.7, 34731.37329743071, 27969.038385567823, 22560.592561926514]
minor_up_notes = [38726.7, 48195.19271581488, 53791.33480208646]
minor_down_notes = [38726.7, 31162.583174734853, 27970.253268488214, 22561.564635655854]

chromatic_up_notes = [38726.7, 43195.050418249404, 48195.19271581488, 53791.33480208646, 60055.49312577782]
chromatic_down_notes = [38726.7, 34732.892894114884, 31162.583174734853, 27970.253268488214, 25115.313095392405, 22561.564635655854, 20276.724791548004]

all_notes = chromatic_down_notes[::-1] + chromatic_up_notes[1:]


amazing_grace_scale= [(all_notes[3],BLACK), (all_notes[6],WHITE), (all_notes[8],QUAVER), (all_notes[6],QUAVER), (all_notes[8],WHITE), (all_notes[7],BLACK), (all_notes[6], WHITE), (all_notes[4], BLACK), (all_notes[3], WHITE)]

uniform_up_notes = np.linspace(HOVER_THRUST, MAX_THRUST, 10)
uniform_down_notes = np.linspace(HOVER_THRUST, MIN_THRUST, 10)

def generate_scale(note_list, note_length):
    return [(note, note_length) for note in note_list]

def generate_rate_scale(nb_rates, start_rate, stop_rate, note_length):
    yaw_rates = np.linspace(start_rate,stop_rate,nb_rates)
    return [(rate, note_length) for rate in yaw_rates]

def cmd_to_acc(cmd):
    return 0.5*(cmd**2)/1600000000

all_notes_acc = [cmd_to_acc(cmd) for cmd in all_notes]
amazing_grace_scale_acc = [(all_notes_acc[3],BLACK), (all_notes_acc[6],WHITE), (all_notes_acc[8],QUAVER), (all_notes_acc[6],QUAVER), (all_notes_acc[8],WHITE), (all_notes_acc[7],BLACK), (all_notes_acc[6], WHITE), (all_notes_acc[4], BLACK), (all_notes_acc[3], WHITE)]

