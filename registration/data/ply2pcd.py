import open3d as o3d
pcd = o3d.io.read_point_cloud("triangle_d1000.ply")
o3d.io.write_point_cloud("triangle_d1000.pcd", pcd)