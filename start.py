import numpy as np
import open3d as o3d
from pathlib import Path
import matplotlib.pyplot as plt

def save_image():
    pcd = o3d.io.read_point_cloud("top_view_pcd.pcd")
    points = np.asarray(pcd.points)
    min_x, min_y = np.min(points[:, 0]), np.min(points[:, 1])
    max_x, max_y = np.max(points[:, 0]), np.max(points[:, 1])
    resolution = 0.001  
    scale = 1 / resolution
    width = int((max_x - min_x) * scale) + 1
    height = int((max_y - min_y) * scale) + 1
    image = np.zeros((height, width, 3), dtype=np.uint8)
    x_pixels = ((points[:, 0] - min_x) * scale).astype(int)
    y_pixels = ((points[:, 1] - min_y) * scale).astype(int)
    min_z = np.min(points[:, 2])
    max_z = np.max(points[:, 2])
    normalized_depth = (points[:, 2] - min_z) / (max_z - min_z) * 255
    for x, y, d in zip(x_pixels, y_pixels, normalized_depth):
        if 0 <= y < height and 0 <= x < width:
            image[y, x] = (d, d, d)  # Set pixel color based on depth value
    plt.imshow(image)
    plt.axis("off")
    plt.savefig("top_view.png", bbox_inches="tight", pad_inches=0)
    plt.show()

def deproject_depth_to_point_cloud(depth, fx, fy, cx, cy, scale, cam_pose):
    height, width = depth.shape
    u = np.arange(width)
    v = np.arange(height)
    u, v = np.meshgrid(u, v)
    z = depth.flatten() * scale
    x = (u.flatten() - cx) * z / fx
    y = (v.flatten() - cy) * z / fy
    points_homogeneous = np.vstack((x, y, z, np.ones_like(x)))
    transformed_points = np.matmul(cam_pose, points_homogeneous)
    transformed_points = transformed_points[:3, :].T
    return transformed_points

def top_view(new_pose, table_pcd):
    old_points = np.array(table_pcd.points)
    ones_column = np.ones((old_points.shape[0], 1))
    old_points_with_ones = np.hstack((old_points, ones_column))
    transformed_points = np.matmul(np.linalg.inv(new_pose), old_points_with_ones.T).T
    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(transformed_points[:,:3])
    return new_pcd

if __name__ == '__main__':
    base_path = Path(__file__).parent.absolute()

    # Image size: 1280 x 960
    depth = np.load(base_path / 'depth.npy')

    # Camera Intrinsics
    fx, fy = 800, 800
    cx, cy = 640, 480
    z_near, z_far = 0.05, 100.0

    # Camera Extrinsics
    cam_pose = np.array([
        [0.0, -np.sqrt(2)/2, np.sqrt(2)/2, 0.5],
        [1.0, 0.0,           0.0,          0.0],
        [0.0, np.sqrt(2)/2,  np.sqrt(2)/2, 0.4],
        [0.0, 0.0,           0.0,          1.0],
    ])


    scale = 1
    point_cloud = deproject_depth_to_point_cloud(depth, fx, fy, cx, cy, scale, cam_pose)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)

    voxel_size = 0.01 
    downsampled_pcd = pcd.voxel_down_sample(voxel_size)
    o3d.visualization.draw_geometries([downsampled_pcd])

    plane_model, inliers = downsampled_pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    table_pcd = downsampled_pcd.select_by_index(inliers, invert=True)
    o3d.visualization.draw_geometries([table_pcd])

    # z_values = np.asarray(downsampled_pcd.points)[:, 2]
    # min_z = np.min(z_values)
    # max_z = np.max(z_values)

    # print("Min z:", min_z)
    # print("Max z:", max_z)

    new_pose = np.eye(4)
    new_pose[1][1] = -1
    new_pose[2][2] = -1
    new_pose[3][0] = 0.85
    new_pose[3][2] = 0.9

    top_down_pcd = top_view(new_pose, table_pcd)
    o3d.visualization.draw_geometries([top_down_pcd])
    o3d.io.write_point_cloud("top_view_pcd.pcd", top_down_pcd)
    save_image()
    
