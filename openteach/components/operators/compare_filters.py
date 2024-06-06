import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation, Slerp
from scipy.signal import butter, lfilter

class OldFilter:
    def __init__(self, state, comp_ratio=0.5):
        self.pos_state = state[:3]
        self.ori_state = state[3:7]
        self.comp_ratio = comp_ratio

    def __call__(self, next_state):
        print(self.pos_state, next_state[:3])
        self.pos_state = np.array(self.pos_state[:3]) * self.comp_ratio + np.array(next_state[:3]) * (1 - self.comp_ratio)
        ori_interp = Slerp([0, 1], Rotation.from_rotvec(
            np.stack([self.ori_state, next_state[3:7]], axis=0)),)
        self.ori_state = ori_interp([1 - self.comp_ratio])[0].as_rotvec()
        return np.concatenate([self.pos_state, self.ori_state])
    
class Filter:
    def __init__(self, state, comp_ratio=0.5, cutoff_freq=1/30, fs=1.0, order=2):
        self.pos_state = state[:3]
        self.ori_state = state[3:7]
        self.comp_ratio = comp_ratio
        self.cutoff_freq = cutoff_freq
        self.fs = fs  # Sampling frequency
        self.order = order
        # Create buffers for filtered output to stabilize initial outputs
        self.filtered_pos = np.array([self.pos_state] * 10)
        self.filtered_ori = np.array([self.ori_state] * 10)

        # Create a Butterworth filter
        self.b, self.a = butter(self.order, self.cutoff_freq / (0.5 * fs), btype='low')

    def __call__(self, next_state):
        # Calculate new interpolated state
        new_pos = np.array(self.pos_state) * self.comp_ratio + np.array(next_state[:3]) * (1 - self.comp_ratio)
        ori_interp = Slerp([0, 1], Rotation.from_rotvec(
            np.stack([self.ori_state, next_state[3:7]], axis=0)))
        new_ori = ori_interp([1 - self.comp_ratio])[0].as_rotvec()

        # Update buffers
        self.filtered_pos = np.vstack((self.filtered_pos[1:], new_pos))
        self.filtered_ori = np.vstack((self.filtered_ori[1:], new_ori))

        # Apply Butterworth filter
        filtered_pos = filtfilt(self.b, self.a, self.filtered_pos, axis=0)[-1]
        filtered_ori = filtfilt(self.b, self.a, self.filtered_ori, axis=0)[-1]

        # Update internal state
        self.pos_state = filtered_pos
        self.ori_state = filtered_ori

        return np.concatenate([filtered_pos, filtered_ori])
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
from scipy.spatial.transform import Rotation, Slerp

# Simulating noisy position data
np.random.seed(0)
time = np.linspace(0, 10, 100)
true_positions = np.sin(time)
noisy_positions = true_positions + np.random.normal(0, 0.1, size=time.shape)

# Initial state
initial_state = np.concatenate([[0, 0, 0], Rotation.from_euler('xyz', [0, 0, 0]).as_rotvec()])

# Instantiate filters
old_filter = OldFilter(initial_state.copy())
new_filter = Filter(initial_state.copy())

# Apply filters
old_filtered_positions = []
new_filtered_positions = []
for pos in noisy_positions:
    state = np.concatenate([[pos, 0, 0], Rotation.from_euler('xyz', [0, 0, 0]).as_rotvec()])
    old_filtered_positions.append(old_filter(state)[0])
    new_filtered_positions.append(new_filter(state)[0])

# Plotting
plt.figure(figsize=(10, 5))
# plt.plot(time, true_positions, label='True Position', linestyle='--')
plt.plot(time, noisy_positions, label='Noisy Position', alpha=0.5)
# plt.plot(time, old_filtered_positions, label='OldFilter Output')
plt.plot(time, new_filtered_positions, label='NewFilter Output')
plt.xlabel('Time')
plt.ylabel('Position')
plt.title('Filter Comparison')
plt.legend()
plt.show()