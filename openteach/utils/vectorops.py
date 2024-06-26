import numpy as np
import cv2
from shapely.geometry import Point, Polygon 
from shapely.ops import nearest_points

def normalize_vector(vector):
    return vector / np.linalg.norm(vector)

def moving_average(vector, moving_average_queue, limit):
    moving_average_queue.append(vector)

    if len(moving_average_queue) > limit:
        moving_average_queue.pop(0)

    mean_vector = np.mean(moving_average_queue, axis = 0)
    return mean_vector

def get_distance(start_vector, end_vector):
    return np.linalg.norm(end_vector - start_vector)

def linear_transform(curr_val, source_bound, target_bound):
    multiplier = (target_bound[1] - target_bound[0]) / (source_bound[1] - source_bound[0])
    target_val = ((curr_val - source_bound[0]) * multiplier) + target_bound[0]
    
    return max(target_bound[0],min(target_val,target_bound[1]))# to clip the result so the result is inside the bound

def persperctive_transform(input_coordinates, given_bound, target_bound):
    transformation_matrix = cv2.getPerspectiveTransform(np.float32(given_bound), np.float32(target_bound))
    transformed_coordinate = np.matmul(np.array(transformation_matrix), np.array([input_coordinates[0], input_coordinates[1], 1]))
    transformed_coordinate = transformed_coordinate / transformed_coordinate[-1]
    planar_point = Point(transformed_coordinate)
    planar_thumb_bounds = Polygon(target_bound)
        # Get the closest point from the thumb to the point within the bounds
    closest_point = nearest_points(planar_thumb_bounds, planar_point)[0]
    return closest_point.x, closest_point.y


def calculate_angle_between_vectors(vector_1,vector_2):

    inner_product = np.inner(vector_1, vector_2)
    norm = np.linalg.norm(vector_1) * np.linalg.norm(vector_2)
    angle = np.arccos(inner_product / norm)
    return angle
    
def calculate_angle(coord_1, coord_2, coord_3):
    vector_1 = coord_2 - coord_1
    vector_2 = coord_3 - coord_2

    inner_product = np.inner(vector_1, vector_2)
    norm = np.linalg.norm(vector_1) * np.linalg.norm(vector_2)
    angle = np.arccos(inner_product / norm)
    return angle

def coord_in_bound(bound, coord):
    return cv2.pointPolygonTest(np.float32(bound), np.float32(coord), False)