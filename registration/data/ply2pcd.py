import open3d as o3d
import sys
from pathlib import Path

if __name__ == '__main__':
    if (len(sys.argv) < 2):
        print("usage: python3 ply2pcd.py <filename>")
        sys.exit(0)

    filename = Path(sys.argv[1])
    export_filename = filename.stem + ".pcd"

    print("Reading from {}".format(filename))
    pcd = o3d.io.read_point_cloud(str(filename))

    print("Exporting to {}".format(export_filename))
    o3d.io.write_point_cloud(export_filename, pcd)
