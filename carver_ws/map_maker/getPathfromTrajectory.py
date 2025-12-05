import open3d as o3d
import numpy as np
import yaml
import os
import matplotlib.pyplot as plt

os.makedirs('output', exist_ok=True)

pcd = o3d.io.read_point_cloud("pcd/trajectory.pcd")
points = np.asarray(pcd.points)

path = []
for i in range(len(points)):
    if i < len(points) - 1:
        dx = points[i+1, 0] - points[i, 0]
        dy = points[i+1, 1] - points[i, 1]
        yaw = np.arctan2(dy, dx)
    else:
        yaw = path[-1]['yaw']
    
    path.append({
        'x': float(points[i, 0]),
        'y': float(points[i, 1]),
        'yaw': float(yaw)
    })

with open('output/path.yaml', 'w') as f:
    yaml.dump(path, f, default_flow_style=False, sort_keys=False)

plt.figure(figsize=(10, 8))
plt.plot(points[:, 0], points[:, 1], 'b-', linewidth=2)
plt.plot(points[0, 0], points[0, 1], 'go', markersize=10, label='Start')
plt.plot(points[-1, 0], points[-1, 1], 'ro', markersize=10, label='End')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Robot Trajectory')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.savefig('output/path.png', dpi=300, bbox_inches='tight')
plt.close()

print(f"Saved {len(path)} waypoints to output/path.yaml")
print(f"Saved trajectory plot to output/path.png")