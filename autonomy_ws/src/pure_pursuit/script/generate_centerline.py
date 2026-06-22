import numpy as np
import matplotlib.pyplot as plt

from PIL import Image
from scipy import ndimage
from skimage.morphology import skeletonize, binary_dilation, disk
from skimage.measure import label, regionprops

# Input map image and output files
MAP_IMAGE = "/home/f1tenth-open/blaze_racer/maps/track.png"
OUT_CSV = "/home/f1tenth-open/blaze_racer/autonomy_ws/src/pure_pursuit/waypoints/track_centerline.csv"
OUT_PREVIEW = "/home/f1tenth-open/blaze_racer/autonomy_ws/src/pure_pursuit/waypoints/track_centerline_preview.png"

# Map metadata — must match maps/track.yaml exactly.
# Resolution (metres per pixel) and the lower-left corner origin in the map frame.
RESOLUTION = 0.05       # maps/track.yaml  → resolution: 0.05
ORIGIN_X   = -3.01      # maps/track.yaml  → origin: [-3.01, -8.99, 0]
ORIGIN_Y   = -8.99

# Load map image as grayscale
img = Image.open(MAP_IMAGE).convert("L")
arr = np.array(img)

# Detect walls/obstacles: dark pixels are considered boundaries
# If the track is not detected correctly, tune this threshold
obstacles = arr < 120

# Slightly thicken walls to close small gaps in the boundary
obstacles = binary_dilation(obstacles, disk(2))

# Free space is everything that is not obstacle
free = ~obstacles


# Start flood-fill from image borders to find outside free area
outside_seed = np.zeros_like(free, dtype=bool)
outside_seed[0, :] = free[0, :]
outside_seed[-1, :] = free[-1, :]
outside_seed[:, 0] = free[:, 0]
outside_seed[:, -1] = free[:, -1]
outside = ndimage.binary_propagation(outside_seed, mask=free)


# Track area = free space inside the closed track boundary
track_area = free & (~outside)


# Keep only the largest connected track region
lbl = label(track_area)
regions = regionprops(lbl)
if not regions:
    raise RuntimeError("No track area found. Try changing threshold or dilation.")
largest = max(regions, key=lambda r: r.area)
track_area = lbl == largest.label


# Extract the centerline skeleton of the track area
skel = skeletonize(track_area)
ys, xs = np.where(skel)
if len(xs) == 0:
    raise RuntimeError("No skeleton found.")


# Convert image pixel coordinates to map/world coordinates
# Image y-axis goes down, but map y-axis goes up
map_x = ORIGIN_X + xs * RESOLUTION
map_y = ORIGIN_Y + (arr.shape[0] - ys) * RESOLUTION
points = np.vstack((map_x, map_y)).T


# Order unordered skeleton points using nearest-neighbor traversal
ordered = [points[0]]
remaining = points[1:].tolist()
while remaining:
    last = ordered[-1]
    rem = np.array(remaining)
    dists = np.linalg.norm(rem - last, axis=1)
    idx = np.argmin(dists)
    ordered.append(rem[idx])
    remaining.pop(idx)

ordered = np.array(ordered)


# Save centerline as waypoint CSV: x, y, speed
# Speed is only a placeholder here; real speed will be adjusted later
speed = np.ones(len(ordered)) * 1.0
out = np.column_stack((ordered[:, 0], ordered[:, 1], speed))
np.savetxt(OUT_CSV, out, delimiter=",", fmt="%.6f")


# Preview
plt.figure(figsize=(8, 8))
plt.imshow(arr, cmap="gray")
plt.scatter(xs, ys, s=1, c="red")
plt.axis("equal")
plt.title("Extracted Centerline Preview")
plt.savefig(OUT_PREVIEW, dpi=200)

print("Saved centerline:", OUT_CSV)
print("Saved preview:", OUT_PREVIEW)
print("Number of waypoints:", len(ordered))