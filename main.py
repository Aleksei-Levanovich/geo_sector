import math

import numpy as np
import osmnx as ox
from geopy.distance import geodesic
from shapely.geometry import Point, LineString
from shapely.geometry.multilinestring import MultiLineString
from shapely.geometry.multipoint import MultiPoint

from Sector import Sector


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


def get_distance_to_nearest_boundary_point(geometry, detected_sector: Sector):
    """
    Определяет минимальное расстояние от точки внутри сектора к ближайшей границе объекта geometry.

    Параметры:
    - geometry: объект shapely (Polygon, MultiPolygon, LineString, MultiLineString)
    - detected_sector: объект Sector (Polygon)

    Возвращает:
    - float — минимальное расстояние до ближайшей точки внутри сектора в метрах.
    """
    center = Point(detected_sector.center)  # Центр сектора
    boundary = geometry.boundary  # Граница объекта (может быть MultiLineString или LineString)
    min_distance = float("inf")
    nearest_point = None
    # Обрабатываем случай, если граница состоит из нескольких частей
    if isinstance(boundary, MultiLineString):
        lines = list(boundary.geoms)  # Достаём отдельные LineString
    elif isinstance(boundary, LineString):
        lines = [boundary]  # Обычный LineString кладём в список
    elif isinstance(boundary, MultiPoint):
        if isinstance(geometry, LineString):
            lines = [geometry]
        else:
            # Получаем первую и последнюю точку в MultiPoint
            first_point = (boundary.bounds[0], boundary.bounds[1])
            last_point = (boundary.bounds[-2], boundary.bounds[-1])
            # Строим LineString между первой и последней точкой
            lines = [LineString([first_point, last_point])]
    else:
        return None  # Неизвестный тип границы

    for line in lines:
        if isinstance(line, LineString):
            # Если линия — это LineString, то просто обрабатываем её как один отрезок
            nearest_point_on_segment = line.interpolate(line.project(center))

            # Проверяем, находится ли ближайшая точка внутри сектора
            if detected_sector.contains_point(nearest_point_on_segment):
                # Расстояние между центром сектора и ближайшей точкой на сегменте
                distance = geodesic((center.y, center.x),
                                    (nearest_point_on_segment.y, nearest_point_on_segment.x)).meters
                if distance < min_distance:
                    min_distance = distance
                    nearest_point = nearest_point_on_segment
        else:
            # Если линия не LineString, игнорируем или обрабатываем по-другому
            continue

    return min_distance if nearest_point is not None else None


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
            print("=================================")

            radius = 60
            sector_angle = 70
            tags = {'building': True, 'highway': ['residential'], 'landuse': True}
            gdf = ox.features_from_point((latitude, longitude), tags=tags, dist=1000)
            detection_sector = Sector(center=(outside_point.x, outside_point.y), azimuth=outside_azimuth,
                                      angle=sector_angle,
                                      radius=radius)

            for idx, row in gdf.iterrows():
                if not isinstance(row.geometry, Point):
                    geometry = row.geometry
                    distance = get_distance_to_nearest_boundary_point(geometry, detection_sector)
                    if distance is not None:
                        if str(row.building) != 'nan':
                            print(
                                f"Здание ({idx}) - Улица {row.get('addr:street')}, Дом {row.get('addr:housenumber')}, distance: {distance}")
                        elif str(row.highway) != 'nan':
                            print(
                                f"Улица {row.get('name')} ({idx}), distance: {distance}")


if __name__ == '__main__':
    main()
