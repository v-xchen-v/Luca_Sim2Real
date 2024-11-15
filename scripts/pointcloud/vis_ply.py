import open3d as o3d

def main():
    # Load the .ply file
    ply_file_path = "data/scene_data/test_scene_data/test_scene.ply"
    point_cloud = o3d.io.read_point_cloud(ply_file_path)

    # Visualize the point cloud 
    o3d.visualization.draw_geometries([point_cloud],
                                    window_name='Colored Point Cloud',
                                    width=800, height=600,
                                    left=0, top=1,
                                    point_show_normal=False)

if __name__ == "__main__":  
    main()