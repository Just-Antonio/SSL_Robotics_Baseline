import os
import sys
import math
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts')))

import numpy as np
from ollama_pick_place import CamModel, pick_cluster_closest_to_bbox_center


def test_cammodel_projection():
    cam = CamModel()
    cam.fx = 100.0
    cam.fy = 100.0
    cam.cx = 50.0
    cam.cy = 50.0
    cam.w = 200
    cam.h = 100
    cam.have_info = True

    # A point at Xc=1,Yc=0,Zc=1 should project to (fx*(1/1)+cx, fy*(0/1)+cy) = (150,50)
    uv = cam.project_cam_xyz_to_pixel(1.0, 0.0, 1.0)
    assert uv is not None
    assert abs(uv[0] - 150.0) < 1e-6
    assert abs(uv[1] - 50.0) < 1e-6


def test_pick_cluster_closest_to_bbox_center():
    clusters_xyz = [(0.1, 0.1, 0.5), (0.2, 0.2, 0.5), (0.8, 0.8, 0.5)]
    uv01_list = [(0.1, 0.1), (0.2, 0.2), (0.8, 0.8)]
    bbox = [0.18, 0.18, 0.1, 0.1]  # center near second cluster
    idx, cluster = pick_cluster_closest_to_bbox_center(clusters_xyz, uv01_list, bbox)
    assert idx == 1
    assert cluster == clusters_xyz[1]
