import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("pcd/GlobalMap.pcd")
o3d.visualization.draw_geometries([pcd])

points = np.asarray(pcd.points)

print(f"X range: {points[:,0].min():.2f} to {points[:,0].max():.2f}")
print(f"Y range: {points[:,1].min():.2f} to {points[:,1].max():.2f}")
print(f"Z range: {points[:,2].min():.2f} to {points[:,2].max():.2f}")
print(f"\nCenter: X={points[:,0].mean():.2f}, Y={points[:,1].mean():.2f}, Z={points[:,2].mean():.2f}")
