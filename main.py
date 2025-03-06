import json
import math

import matplotlib.pyplot as plt
import numpy as np
import osmnx as ox
import pyproj
from geopy.distance import geodesic
from matplotlib.patches import PathPatch
from matplotlib.path import Path
from shapely.geometry import Point, LineString
from shapely.geometry.multilinestring import MultiLineString
from shapely.geometry.multipoint import MultiPoint
from shapely.geometry.polygon import Polygon
from shapely.ops import transform

from Sector import Sector

# Исходная точка
LONGITUDE = 30.2773029
LATITUDE = 59.93520094

# Параметры сектора
RADIUS = 30
SECTOR_ANGLE = 150

NEAREST_POINTS = []


def move_point(lat, lon, bearing, distance_m):
    origin = (lon, lat)
    destination = geodesic(meters=distance_m).destination(origin, bearing)
    return destination.latitude, destination.longitude


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

    point_to_draw = None
    distance = float("inf")
    for line in lines:
        if isinstance(line, LineString):
            if detected_sector.intersects_line(line):
                axis_intersection_point, distance = detected_sector.get_axis_intersection_point(line)
                if axis_intersection_point:
                    point_to_draw = axis_intersection_point
                    min_distance = min(min_distance, distance)
        else:
            continue
    if point_to_draw:
        NEAREST_POINTS.append(point_to_draw)
    return distance if distance < float("inf") else None, point_to_draw


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
    target_point = Point(LONGITUDE, LATITUDE)

    gdf = ox.features_from_point((LATITUDE, LONGITUDE), tags={'building': True}, dist=10)
    if gdf.empty:
        print("Нет объектов вблизи точки. Попробуйте увеличить радиус или изменить теги.")
        return 0

    object_found = False
    polygon = None
    door_of_idx = None

    for idx, row in gdf.iterrows():
        if row.get('geometry') and row.get('geometry').contains(target_point):
            print(f"Точка принадлежит объекту: {row.get('name', 'Без имени')}")
            polygon = row['geometry']
            object_found = True
            door_of_idx = idx
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

            detection_sector = Sector(center=(outside_point.x, outside_point.y), azimuth=outside_azimuth,
                                      angle=SECTOR_ANGLE, radius=RADIUS)

            tags = {'building': True, 'highway': ['residential'], 'landuse': True}
            gdf = ox.features_from_point((LATITUDE, LONGITUDE), tags=tags, dist=100)

            points_in_sector = []

            for idx, row in gdf.iterrows():
                if idx != door_of_idx:
                    if row.get('landuse') is None or str(row.get('landuse')) == 'nan':  # Убираем пересечения с landuse
                        if not isinstance(row.geometry, Point):
                            geometry = row.geometry
                            distance, point_of_crossing = get_distance_to_nearest_boundary_point(geometry,
                                                                                                 detection_sector)
                            if distance is not None:
                                if row.get('building') is not None and str(row.get('building')) != 'nan':
                                    points_in_sector.append({'type': 'building', 'distance': distance, 'idx': idx,
                                                      'address': f"Адрес: {row.get('addr:street')}, дом {row.get('addr:housenumber')}",
                                                      'lon': point_of_crossing.x,
                                                      'lat': point_of_crossing.y,
                                                      })
                                elif row.get('highway') is not None and str(row.get('highway')) != 'nan':
                                    points_in_sector.append({'type': 'street', 'distance': distance, 'idx': idx,
                                                      'address': f"Улица: {row.get('name')}",
                                                      'lon': point_of_crossing.x,
                                                      'lat': point_of_crossing.y,
                                                      })

            sorted_points_by_distance = sorted(points_in_sector, key=lambda p: p['distance'])
            print(json.dumps(sorted_points_by_distance, indent=2, ensure_ascii=False))

            if gdf.crs is None:
                gdf.set_crs(epsg=4326, inplace=True)

            # Project the GeoDataFrame to UTM Zone 36N (EPSG:32636) for Saint Petersburg
            gdf = gdf.to_crs(epsg=32636)

            # Plot the GeoDataFrame
            fig, ax = plt.subplots(figsize=(10, 10))
            gdf.plot(ax=ax, color='blue', edgecolor='black', alpha=0.5)

            sector_polygon = detection_sector.polygon

            # Определяем преобразование в EPSG:32636 (UTM Zone 36N)
            transformer = pyproj.Transformer.from_crs("EPSG:4326", "EPSG:32636", always_xy=True)

            # Преобразуем sector_polygon в EPSG:32636
            sector_polygon_transformed = transform(transformer.transform, sector_polygon)

            # Validate the transformed polygon
            if not isinstance(sector_polygon_transformed, Polygon):
                raise ValueError(f"Transformation failed: got {type(sector_polygon_transformed)}")
            if sector_polygon_transformed.is_empty:
                raise ValueError("Transformed polygon is empty")

            coords = list(sector_polygon_transformed.exterior.coords)

            # Create a Matplotlib Path from the coordinates
            vertices = [(x, y) for x, y in coords]
            codes = [Path.MOVETO] + [Path.LINETO] * (len(vertices) - 2) + [Path.CLOSEPOLY]
            path = Path(vertices, codes)

            # Create and add the patch
            patch = PathPatch(path, facecolor='red', edgecolor='black', alpha=0.5, zorder=2)
            ax.add_patch(patch)

            transformed_points = [transform(transformer.transform, point) for point in NEAREST_POINTS]

            if len(transformed_points) > 0:
                # Extract x and y coordinates
                x_coords, y_coords = zip(*[(point.x, point.y) for point in transformed_points])
                # Plot the points
                ax.scatter(x_coords, y_coords, color='yellow', marker='o', zorder=3)

            x_coords, y_coords = zip(*coords)
            ax.set_xlim(min(x_coords) - 100, max(x_coords) + 100)
            ax.set_ylim(min(y_coords) - 100, max(y_coords) + 100)

            # Add a title and show the plot
            plt.title("Buildings around Saint Petersburg (EPSG:32636)")
            plt.axis('off')  # Turn off axis for cleaner visualization
            plt.show()


if __name__ == '__main__':
    main()
