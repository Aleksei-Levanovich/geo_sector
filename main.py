import numpy as np
import osmnx as ox
from shapely.geometry import Point, LineString
from geopy.distance import geodesic
import math


def move_point(lat, lon, bearing, distance_m):
    origin = (lon, lat)
    destination = geodesic(meters=distance_m).destination(origin, bearing)
    return destination.latitude, destination.longitude


def is_point_in_sector(point, center, direction, angle):
    dx = point.x - center.x
    dy = point.y - center.y
    point_angle = math.degrees(math.atan2(dx, dy))
    point_angle = (point_angle + 360) % 360
    direction = (direction + 360) % 360
    half_angle = angle / 2
    lower_bound = (direction - half_angle + 360) % 360
    upper_bound = (direction + half_angle + 360) % 360
    if lower_bound <= upper_bound:
        return lower_bound <= point_angle <= upper_bound
    else:
        return point_angle >= lower_bound or point_angle <= upper_bound


def get_nearest_point(geometry, target_point):
    if geometry.geom_type == 'Point':
        return geometry
    elif geometry.geom_type == 'LineString':
        return geometry.interpolate(geometry.project(target_point))
    elif geometry.geom_type == 'Polygon':
        return geometry.boundary.interpolate(geometry.boundary.project(target_point))
    else:
        return geometry.centroid


def calculate_azimuth(start, end):
    lon1, lat1 = start
    lon2, lat2 = end
    angle = math.degrees(math.atan2(lon2 - lon1, lat2 - lat1))
    return (angle + 360) % 360


def move_point_along_line(point, azimuth, distance_m, reverse=False):
    lat, lon = point.y, point.x
    if reverse:
        azimuth = (azimuth + 180) % 360
    new_point = geodesic(meters=distance_m).destination((lat, lon), azimuth)
    return Point(new_point.longitude, new_point.latitude)


def is_point_inside_polygon(point, polygon):
    return polygon.contains(point)


def main():
    latitude = 59.93520094
    longitude = 30.2773029
    target_point = Point(longitude, latitude)

    gdf = ox.features_from_point((latitude, longitude), tags={'building': True}, dist=10)
    if gdf.empty:
        print("Нет объектов вблизи точки. Попробуйте увеличить радиус или изменить теги.")
        return 0

    print("Найденные объекты:")
    print(gdf[['name', 'geometry']])

    object_found = False
    polygon = None
    for idx, row in gdf.iterrows():
        if row['geometry'].contains(target_point):
            print(f"Точка принадлежит объекту: {row.get('name', 'Без имени')}")
            print("Контурные координаты объекта:")
            print(row['geometry'].exterior.coords)
            polygon = row['geometry']
            object_found = True
            break

    if not object_found:
        print("Точка не принадлежит ни одному из найденных объектов.")
    else:
        contour_coords = list(polygon.exterior.coords)
        min_distance = float('inf')
        nearest_point = None
        nearest_segment = None

        for i in range(len(contour_coords) - 1):
            start = contour_coords[i]
            end = contour_coords[i + 1]
            segment = LineString([start, end])
            projection = segment.interpolate(segment.project(target_point))
            distance = target_point.distance(projection)

            if distance < min_distance:
                min_distance = distance
                nearest_point = projection
                nearest_segment = segment

        print(f"Ближайшая точка на контуре: {nearest_point}")
        start = nearest_segment.coords[0]
        end = nearest_segment.coords[1]
        print(f"Координаты ближайшей границы здания: {start}, {end}")

        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = np.sqrt(dx ** 2 + dy ** 2)
        dx /= length
        dy /= length

        perpendicular_dx = -dy
        perpendicular_dy = dx
        perpendicular_length_m = 60

        angle1 = np.degrees(np.arctan2(perpendicular_dy, perpendicular_dx))
        angle2 = angle1 + 180
        lat, lon = target_point.y, target_point.x
        endpoint1 = move_point(lat, lon, angle1, perpendicular_length_m / 2)
        endpoint2 = move_point(lat, lon, angle2, perpendicular_length_m / 2)

        perpendicular_line = LineString([endpoint1, endpoint2])
        print(f"Перпендикулярная линия: {perpendicular_line}")

        intersection = nearest_segment.intersection(perpendicular_line)

        if intersection.is_empty:
            print("Пересечений с контуром дома не найдено.")
        elif isinstance(intersection, Point):
            print(f"Точка пересечения: {intersection}")
            intersection_point = Point(intersection.x, intersection.y)
            start_point = perpendicular_line.coords[0]
            end_point = perpendicular_line.coords[1]

            azimuth = calculate_azimuth(start_point, end_point)
            point1 = move_point_along_line(intersection_point, azimuth, 1)
            point2 = move_point_along_line(intersection_point, azimuth, 1, reverse=True)

            print(f"Точка 1 (1 метр вперёд): ({point1.x}, {point1.y})")
            print(f"Точка 2 (1 метр назад): ({point2.x}, {point2.y})")

            inside1 = is_point_inside_polygon(point1, polygon.buffer(0.00001))
            inside2 = is_point_inside_polygon(point2, polygon.buffer(0.00001))

            if inside1:
                print("Точка 1 находится внутри здания.")
            else:
                print("Точка 1 находится снаружи здания.")

            if inside2:
                print("Точка 2 находится внутри здания.")
            else:
                print("Точка 2 находится снаружи здания.")

            outside_point = point2 if not inside2 else point1
            outside_azimuth = calculate_azimuth((intersection.x, intersection.y), (outside_point.x, outside_point.y))
            print(f"Азимут вектора до точки снаружи: {outside_azimuth}°")

            radius = 100
            sector_angle = 90
            direction = outside_azimuth
            tags = {'building': True, 'highway': True, 'landuse': True}
            gdf = ox.features_from_point((latitude, longitude), tags=tags, dist=radius)
            center_point = Point(longitude, latitude)

            filtered_gdf = gdf[gdf.geometry.apply(
                lambda geom: is_point_in_sector(geom.centroid, center_point, direction, sector_angle))]

            allowed_highways = ["residential"]
            origin_point = (latitude, longitude)

            for idx, row in filtered_gdf.iterrows():
                geometry = row.geometry
                nearest_point = get_nearest_point(geometry, center_point)
                nearest_point_coords = (nearest_point.y, nearest_point.x)
                distance = geodesic(origin_point, nearest_point_coords).meters

                if distance < radius * 1.1:
                    if row.highway in allowed_highways:
                        print(f"Найден highway: {row.highway}, расстояние: {distance:.2f} метров")
                    if row.building == 'yes':
                        print(f"Найден building, расстояние: {distance:.2f} метров")


if __name__ == '__main__':
    main()
